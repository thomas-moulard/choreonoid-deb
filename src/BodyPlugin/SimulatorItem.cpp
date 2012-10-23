/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SimulatorItem.h"
#include <cnoid/TimeBar>
#include <cnoid/MessageView>
#include <cnoid/LazyCaller>
#include <QThread>
#include <QMutex>
#include <boost/bind.hpp>

#if QT_VERSION >= 0x040700
#include <QElapsedTimer>
#else
#include <QTime>
typedef QTime QElapsedTimer;
#endif

#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace cnoid {

    class SimulatorItemImpl : QThread
    {
    public:
        SimulatorItemImpl(SimulatorItem* self);
        SimulatorItemImpl(SimulatorItem* self, const SimulatorItemImpl& org);
            
        SimulatorItem* self;
        ostream& os;
        bool isAllLinkPositionOutputMode;
        bool isDoingSimulationLoop;
        bool isBufsUpdatedEvenetPending;
        bool stopRequested;
        LazyCaller flushCaller;
        TimeBar* timeBar;
        int fillLevelId;
        QMutex mutex;
        double actualSimulationTime;

        boost::signal<void()> sigSimulationFinished_;

        bool startSimulation();
        virtual void run();
        void stopSimulation();
            
        void onSimulationLoopStopped();
        void onFlushingResultsRequested();
    };
}


SimulatorItem::SimulatorItem()
{
    impl = new SimulatorItemImpl(this);
}


SimulatorItemImpl::SimulatorItemImpl(SimulatorItem* self)
    : self(self),
      os(MessageView::mainInstance()->cout()),
      flushCaller(bind(&SimulatorItemImpl::onFlushingResultsRequested, this))
{
    timeBar = TimeBar::instance();
    isDoingSimulationLoop = false;
    isAllLinkPositionOutputMode = false;
}


SimulatorItem::SimulatorItem(const SimulatorItem& org)
    : Item(org)
{
    impl = new SimulatorItemImpl(this);
    impl->isAllLinkPositionOutputMode = org.impl->isAllLinkPositionOutputMode;
}


SimulatorItem::~SimulatorItem()
{
    delete impl;
}


void SimulatorItem::setAllLinkPositionOutputMode(bool on)
{
    impl->isAllLinkPositionOutputMode = on;
}


bool SimulatorItem::isAllLinkPositionOutputMode()
{
    return impl->isAllLinkPositionOutputMode;
}


bool SimulatorItem::startSimulation()
{
    return impl->startSimulation();
}


bool SimulatorItemImpl::startSimulation()
{
    if(self->isSimulationRunning()){
        // use a conditional variable here ?
        stopSimulation();
    }
    
    bool result = self->doStartSimulation();

    if(result){

        isBufsUpdatedEvenetPending = false;
        isDoingSimulationLoop = true;
        stopRequested = false;

        fillLevelId = timeBar->startFillLevelUpdate();

        if(!timeBar->isDoingPlayback()){
            timeBar->setTime(0.0);
            timeBar->startPlayback();
        }

        os << (format(_("Simulation by %1% has started.")) % self->name()) << endl;

        start();
    }

    return result;
}


// Simulation loop
void SimulatorItemImpl::run()
{
    QElapsedTimer timer;
    timer.start();

    while(true){
        if(!self->doStepSimulation() || stopRequested){
            break;
        }
    }

    actualSimulationTime = (timer.elapsed() / 1000.0);

    isDoingSimulationLoop = false;

    callLater(bind(&SimulatorItemImpl::onSimulationLoopStopped, this));
}


void SimulatorItem::stopSimulation()
{
    impl->stopSimulation();
}


void SimulatorItemImpl::stopSimulation()
{
    stopRequested = true;
}


void SimulatorItemImpl::onSimulationLoopStopped()
{
    double fillLevel = self->doFlushResults();
    timeBar->updateFillLevel(fillLevelId, fillLevel);

    double finishTime = self->doStopSimulation();

    timeBar->stopFillLevelUpdate(fillLevelId);

    os << (format(_("Simulation by %1% has finished at %2% [s].\n")) % self->name() % finishTime);
    os << (format(_(" Actual elapsed time = %1% [s], actual / virtual = %2%."))
           % actualSimulationTime % (actualSimulationTime / finishTime))
       << endl;

    sigSimulationFinished_();
}


bool SimulatorItem::isSimulationRunning()
{
    return impl->isDoingSimulationLoop;
}


SignalProxy< boost::signal<void()> > SimulatorItem::sigSimulationFinished()
{
    return impl->sigSimulationFinished_;
}


void SimulatorItem::lockResults()
{
    impl->mutex.lock();
}


void SimulatorItem::unlockResults()
{
    impl->mutex.unlock();
}


void SimulatorItem::requestToFlushResults()
{
    impl->flushCaller.request();
}


void SimulatorItemImpl::onFlushingResultsRequested()
{
    double fillLevel = self->doFlushResults();

    timeBar->updateFillLevel(fillLevelId, fillLevel);

    mutex.lock();
    isBufsUpdatedEvenetPending = false;
    mutex.unlock();
}
