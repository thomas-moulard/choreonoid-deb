/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_DYNAMICS_SIMULATOR_ITEM_H_INCLUDED
#define CNOID_BODYPLUGIN_DYNAMICS_SIMULATOR_ITEM_H_INCLUDED

#include "SimulatorItem.h"

namespace cnoid {

    class DSIImpl;
        
    class DynamicsSimulatorItem : public SimulatorItem
    {
      public:
        DynamicsSimulatorItem();
        DynamicsSimulatorItem(const DynamicsSimulatorItem& org);
        virtual ~DynamicsSimulatorItem();

        virtual bool doStartSimulation();
        virtual bool doStepSimulation();
        virtual double doFlushResults();
        virtual double doStopSimulation();

      protected:

        virtual QWidget* settingPanel();
        virtual ItemPtr doDuplicate() const;
        virtual void doPutProperties(PutPropertyFunction& putProperty);
        virtual bool store(Archive& archive);
        virtual bool restore(const Archive& archive);
            
      private:
        DSIImpl* impl;
        friend class DSIImpl;
    };

    typedef boost::intrusive_ptr<DynamicsSimulatorItem> DynamicsSimulatorItemPtr;

    void initializeDynamicsSimulatorItem(ExtensionManager& ext);
}

#endif
