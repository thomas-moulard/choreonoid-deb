/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_LAZY_CALLER_H_INCLUDED
#define CNOID_GUIBASE_LAZY_CALLER_H_INCLUDED

#include <Qt>
#include <boost/function.hpp>
#include "exportdecl.h"

namespace cnoid {
    
    enum {
        IDLE_PRIORITY_HIGH   = Qt::HighEventPriority,
        IDLE_PRIORITY_NORMAL = Qt::NormalEventPriority,
        IDLE_PRIORITY_LOW    = Qt::LowEventPriority
    };

    class CNOID_EXPORT LazyCaller
    {
        boost::function<void(void)> function;
        bool isCallEventPending;
        int priority;
        
    public:
        LazyCaller();
        LazyCaller(const boost::function<void(void)>& function, int priority = IDLE_PRIORITY_HIGH);

        void set(const boost::function<void(void)>& function, int priority = IDLE_PRIORITY_HIGH);
        void setPriority(int priority);
        void request();

    private:
        void onRequest();
    };


    CNOID_EXPORT void callLater(const boost::function<void()>& function, int priority = IDLE_PRIORITY_NORMAL);
    CNOID_EXPORT void callSynchronously(const boost::function<void()>& function, int priority = IDLE_PRIORITY_NORMAL);
}
        
#endif
