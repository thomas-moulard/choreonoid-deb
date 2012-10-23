/**
   @author Shin'ichiro Nakaoka
*/

#include "LazyCaller.h"
#include <QObject>
#include <QEvent>
#include <QCoreApplication>
#include <QThread>
#include <QSemaphore>
#include <boost/bind.hpp>

using namespace cnoid;

namespace {

    class CallEvent : public QEvent
    {
    public:
        CallEvent(const boost::function<void(void)>& function)
            : QEvent(QEvent::User),
              function(function) {
        }
        boost::function<void(void)> function;
    };

    class SynchronousCallEvent : public QEvent
    {
    public:
        SynchronousCallEvent(const boost::function<void(void)>& function, QSemaphore& semaphore)
            : QEvent(QEvent::User),
              function(function),
              semaphore(semaphore) {
        }
        boost::function<void(void)> function;
        QSemaphore& semaphore;
    };
    
    class CallEventHandler : public QObject
    {
      public:
        CallEventHandler() {
            mainThreadId = QThread::currentThreadId();
        }
        virtual bool event(QEvent* e);
        Qt::HANDLE mainThreadId;
    };
    
    CallEventHandler callEventHandler;
}


void cnoid::callLater(const boost::function<void(void)>& function, int priority)
{
    QCoreApplication::postEvent(&callEventHandler, new CallEvent(function), priority);
}


void cnoid::callSynchronously(const boost::function<void(void)>& function, int priority)
{
    if(QThread::currentThreadId() == callEventHandler.mainThreadId){
        callLater(function, priority);
    } else {
        QSemaphore semaphore;
        QCoreApplication::postEvent(&callEventHandler, new SynchronousCallEvent(function, semaphore), priority);
        semaphore.acquire(); // wait for finish
    }
}


bool CallEventHandler::event(QEvent* e)
{
    CallEvent* callEvent = dynamic_cast<CallEvent*>(e);
    if(callEvent){
        callEvent->function();
        return true;
    } else {
        SynchronousCallEvent* scEvent = dynamic_cast<SynchronousCallEvent*>(e);
        if(scEvent){
            scEvent->function();
            scEvent->semaphore.release(); // wake up
            return true;
        }
    }
    return false;
}


LazyCaller::LazyCaller()
{
    isCallEventPending = false;
    priority = IDLE_PRIORITY_HIGH;
}


LazyCaller::LazyCaller(const boost::function<void(void)>& function, int priority)
    : function(function),
      priority(priority)
{
    isCallEventPending = false;
}


void LazyCaller::set(const boost::function<void(void)>& function, int priority)
{
    this->function = function;
    this->priority = priority;
}


void LazyCaller::setPriority(int priority)
{
    this->priority = priority;
}
        

void LazyCaller::request()
{
    if(!isCallEventPending){
        isCallEventPending = true;
        callLater(boost::bind(&LazyCaller::onRequest, this), priority);
    }
}


void LazyCaller::onRequest()
{
    isCallEventPending = false;
    function();
}
