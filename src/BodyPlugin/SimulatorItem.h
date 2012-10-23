/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_SIMULATOR_ITEM_H_INCLUDED
#define CNOID_BODYPLUGIN_SIMULATOR_ITEM_H_INCLUDED

#include <cnoid/Item>
#include <QWidget>
#include "exportdecl.h"

namespace cnoid {
        
    class SimulatorItemImpl;
        
    class CNOID_EXPORT SimulatorItem : public Item
    {
      public:
        SimulatorItem();
        SimulatorItem(const SimulatorItem& org);
        virtual ~SimulatorItem();

        void setAllLinkPositionOutputMode(bool on);
        bool isAllLinkPositionOutputMode();
            
        bool startSimulation();
        void stopSimulation();
        bool isSimulationRunning();

        SignalProxy< boost::signal<void()> > sigSimulationFinished();
            
      protected:
            
        virtual QWidget* settingPanel() = 0;
            
        virtual bool doStartSimulation() = 0;
        virtual bool doStepSimulation() = 0;

        /**
           @return time at the last flush
        */
        virtual double doFlushResults() = 0;

        /**
           @return finish time
        */
        virtual double doStopSimulation() = 0;

        void lockResults();
        void unlockResults();

        /**
           This must be called within the critical section
        */
        void requestToFlushResults();
            
      private:
            
        SimulatorItemImpl* impl;

        friend class SimulatorView;
        friend class SimulatorItemImpl;
    };
        
    typedef boost::intrusive_ptr<SimulatorItem> SimulatorItemPtr;
}

#endif
