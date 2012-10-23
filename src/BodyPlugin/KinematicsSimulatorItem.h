/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_KINEMATICS_SIMULATOR_ITEM_H_INCLUDED
#define CNOID_BODYPLUGIN_KINEMATICS_SIMULATOR_ITEM_H_INCLUDED

#include "SimulatorItem.h"

namespace cnoid {

    class KSIImpl;
        
    class KinematicsSimulatorItem : public SimulatorItem
    {
    public:
        KinematicsSimulatorItem();
        KinematicsSimulatorItem(const KinematicsSimulatorItem& org);
        virtual ~KinematicsSimulatorItem();

    protected:

        virtual QWidget* settingPanel();
        virtual bool doStartSimulation();
        virtual bool doStepSimulation();
        virtual double doFlushResults();
        virtual double doStopSimulation();

        virtual ItemPtr doDuplicate() const;
        virtual bool store(Archive& archive);
        virtual bool restore(const Archive& archive);
            
    private:
        KSIImpl* impl;
        friend class KSIImpl;

    };

    typedef boost::intrusive_ptr<KinematicsSimulatorItem> KinematicsSimulatorItemPtr;

    void initializeKinematicsSimulatorItem(ExtensionManager& ext);
}

#endif
