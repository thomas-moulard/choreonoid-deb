/**
   @author Shin'ichiro Nakaoka
*/

#include "LazySignal.h"
#include <boost/bind.hpp>

using namespace cnoid;

void LazySignalBase::request()
{
    if(!isIdleEventPending){
        isIdleEventPending = true;
        callLater(boost::bind(&LazySignalBase::onIdle, this), priority);
    }
}


bool LazySignalBase::onIdle()
{
    isIdleEventPending = false;
    for(size_t i=0; i < connectionsToBlock.size(); ++i){
        connectionsToBlock[i].block();
    }

    if(emitFunction){
        emitFunction();
    } else {
        defaultEmitFunction();
    }
    
    for(size_t i=0; i < connectionsToBlock.size(); ++i){
        connectionsToBlock[i].unblock();
    }
    connectionsToBlock.clear();
    
    return false;
}
