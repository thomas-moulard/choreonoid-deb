/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_CHOREOGRAPHY_BODY_MOTION_GENERATION_BAR_H_INCLUDED
#define CNOID_CHOREOGRAPHY_BODY_MOTION_GENERATION_BAR_H_INCLUDED

#include <cnoid/ToolBar>
#include <cnoid/LazySignal>
#include <cnoid/ConnectionSet>
#include <cnoid/Body>
#include <cnoid/BodyMotionItem>
#include "exportdecl.h"

namespace cnoid {

    class ExtensionManager;
    class TimeBar;
    class PoseProvider;
    class BodyMotionPoseProvider;
    class PoseProviderToBodyMotionConverter;
    class BodyMotionGenerationSetupDialog;
    class ToggleToolButton;
    class Action;

    class CNOID_EXPORT BodyMotionGenerationBar : public ToolBar
    {
      public:
        static void initializeInstance(ExtensionManager* ext);
            
        static BodyMotionGenerationBar* instance();

        virtual ~BodyMotionGenerationBar();

        bool shapeBodyMotion(
            BodyPtr body, PoseProvider* provider, BodyMotionItemPtr motionItem, bool putMessages = false);

        typedef boost::function<bool(BodyPtr& body, PoseProvider* provider,
                                     BodyMotionItemPtr motionItem, bool putMessages)> BalancerFunc;

        void setBalancer(BalancerFunc func, QWidget* panel);
        void unsetBalancer();

        bool isAutoInterpolationUpdateMode() const;
        bool isBalancerEnabled() const;
        bool isAutoGenerationMode() const;
            
        double timeScaleRatio() const;
        double preInitialDuration() const;
        double postFinalDuration() const;
        
        double timeToStartBalancer() const;
        int balancerIterations() const;
        int boundaryConditionType() const;
        int boundarySmootherType() const;
        double boundarySmootherTime() const;
        double dynamicsTimeRatio() const;
        bool isTimeBarRangeOnly() const;
        int initialWaistTrajectoryMode() const;
        bool isStealthyStepMode() const;
        double stealthyHeightRatioThresh() const;
        double flatLiftingHeight() const;
        double flatLandingHeight() const;
        double impactReductionHeight() const;
        double impactReductionTime() const;
        bool isAutoZmpAdjustmentMode() const;
        double minZmpTransitionTime() const;
        double zmpCenteringTimeThresh() const;
        double zmpTimeMarginBeforeLifting() const;
        bool isSe3Enabled() const;
        bool isLipSyncMixMode() const;
            
        SignalProxy< boost::signal<void()> > sigInterpolationParametersChanged() {
            return sigInterpolationParametersChanged_.signal();
        }
            
      private:
    
        BodyMotionPoseProvider* bodyMotionPoseProvider;
        PoseProviderToBodyMotionConverter* poseProviderToBodyMotionConverter;
    
        BalancerFunc applyBalancer;
        QWidget* balancerPanel;

        TimeBar* timeBar;
        BodyMotionGenerationSetupDialog* setup;
            
        Action* autoInterpolationUpdateCheck;
        ToolButton* balancerToggle;
        ToolButton* autoGenerationToggle;

        LazySignal< boost::signal<void()> >sigInterpolationParametersChanged_;

        ConnectionSet interpolationParameterWidgetsConnection;

        BodyMotionGenerationBar();

        void notifyInterpolationParametersChanged();

        void onGenerationButtonClicked();

        bool shapeBodyMotionWithSimpleInterpolation
            (BodyPtr& body, PoseProvider* provider, BodyMotionItemPtr motionItem);
            
        virtual bool storeState(Archive& archive);
        virtual bool restoreState(const Archive& archive);
    };
}

#endif
