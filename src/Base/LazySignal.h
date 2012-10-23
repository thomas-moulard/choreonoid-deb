/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_LAZY_SIGNAL_H_INCLUDED
#define CNOID_GUIBASE_LAZY_SIGNAL_H_INCLUDED

#include "LazyCaller.h"
#include <boost/signal.hpp>
#include <boost/function.hpp>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

    // Base class for hiding the boost.bind header into the cpp file.
    class CNOID_EXPORT LazySignalBase
    {
    public:
        void request();

    protected:
        boost::function<void()> emitFunction;
        int priority;
        bool isIdleEventPending;
        std::vector<boost::signals::connection> connectionsToBlock;
        
        virtual void defaultEmitFunction() = 0;

    private:
        bool onIdle();
    };

    template <class SignalType> class LazySignal : public LazySignalBase
    {
    public:
        LazySignal(int priority = IDLE_PRIORITY_HIGH) {
            this->priority = priority;
            isIdleEventPending = false;
        }
        
        LazySignal(boost::function<void()> emitFunction, int priority = IDLE_PRIORITY_HIGH) {
            this->emitFunction = emitFunction;
            this->priority = priority;
            isIdleEventPending = false;
        }
        
        inline SignalType& signal() { return signal_; }
        
        inline void requestBlocking(boost::signals::connection connection){
            connectionsToBlock.push_back(connection);
        }
        
        inline bool isBeingRequested() {
            return isIdleEventPending;
        }

    protected:
        virtual void defaultEmitFunction() {
            signal_();
        }

    private:
        SignalType signal_;
    };
}
        
#endif
