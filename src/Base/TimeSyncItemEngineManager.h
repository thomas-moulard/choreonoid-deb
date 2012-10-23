/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_TIME_SYNC_ITEM_ENGINE_MANAGER_H_INCLUDED
#define CNOID_GUIBASE_TIME_SYNC_ITEM_ENGINE_MANAGER_H_INCLUDED

#include "Item.h"
#include <boost/function.hpp>
#include <boost/signals.hpp>
#include "exportdecl.h"

namespace cnoid {

    class AppImpl;

    class CNOID_EXPORT TimeSyncItemEngine : public Referenced, boost::signals::trackable
    {
    public:
        virtual bool onTimeChanged(double time) = 0;

    protected:
        void notifyUpdate();
    };

    typedef boost::intrusive_ptr<TimeSyncItemEngine> TimeSyncItemEnginePtr;

    typedef boost::function<TimeSyncItemEnginePtr(Item* sourceItem)> TimeSyncItemEngineFactoryFunc;

    class CNOID_EXPORT TimeSyncItemEngineManager
    {
      public:
        static void initialize();
        
        TimeSyncItemEngineManager(const std::string& moduleName);
        ~TimeSyncItemEngineManager();
        
        void addEngineFactory(TimeSyncItemEngineFactoryFunc factoryFunc);
        
      private:
        std::string moduleName;
    };
}

#endif
