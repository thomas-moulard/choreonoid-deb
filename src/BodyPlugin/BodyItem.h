/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_BODY_ITEM_H_INCLUDED
#define CNOID_BODYPLUGIN_BODY_ITEM_H_INCLUDED

#include <deque>
#include <bitset>
#include <boost/dynamic_bitset.hpp>
#include <cnoid/Item>
#include <cnoid/YamlNodes>
#include <cnoid/LazySignal>
#include <cnoid/LazyCaller>
#include <cnoid/Body>
#include <cnoid/ModelNodeSet>
#include <cnoid/ColdetLinkPair>
#include <cnoid/InverseKinematics>
#include <cnoid/LinkGroup>
#include "exportdecl.h"

namespace cnoid {
        
    class BodyItem;
    typedef boost::intrusive_ptr<BodyItem> BodyItemPtr;

    class WorldItem;

    class PinDragIK;
    typedef boost::shared_ptr<PinDragIK> PinDragIKptr;

    class PenetrationBlocker;
    typedef boost::shared_ptr<PenetrationBlocker> PenetrationBlockerPtr;

    class LinkGroup;
    class KinematicsBar;

    void initializeBodyItem(ExtensionManager& ext);

    class CNOID_EXPORT BodyItem : public Item
    {
      public:

        BodyItem();
        BodyItem(const BodyItem& org);
        virtual ~BodyItem();

        void init();

        bool loadModelFile(const std::string& filename);
            
        virtual void setName(const std::string& name);

        inline Body* body() { return body_.get(); }
        inline ModelNodeSetPtr modelNodeSet() { return modelNodeSet_; }
        const std::string& errorMessage() { return errorMessage_; }

        enum PresetPoseID { INITIAL_POSE, STANDARD_POSE };

        void moveToOrigin();

        void setPresetPose(PresetPoseID id);

        inline Link* currentBaseLink() const { return currentBaseLink_; }
        void setCurrentBaseLink(Link* link);

        void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false);

        void copyKinematicState();
        void pasteKinematicState();

        struct KinematicState
        {
        private:
            std::vector<double> q;
            Vector3 p;
            Matrix3 R;
            Vector3 zmp;
            friend class BodyItem;
            friend void cnoid::initializeBodyItem(ExtensionManager& ext);
        };

        void storeKinematicState(KinematicState& state);
        bool restoreKinematicState(const KinematicState& state);

        // for undo, redo operations
        void beginKinematicStateEdit();
        void acceptKinematicStateEdit();
        bool undoKinematicState();
        bool redoKinematicState();

        PinDragIKptr pinDragIK();
        InverseKinematicsPtr getCurrentIK(Link* targetLink);
        PenetrationBlockerPtr createPenetrationBlocker(Link* link, bool excludeSelfCollisions = false);

        /**
           @if jp
           ロボットの関節角、関節角速度、root位置・姿勢などの「運動学的」状態に変更が生じたときに
           発行されるシグナル。
           Item::sigUpdated() はモデル自体が変わった場合とし、そちらとは区別して使う。
           @endif
        */
        SignalProxy< boost::signal<void()> > sigKinematicStateChanged() {
            return sigKinematicStateChanged_.signal();
        }

        void notifyKinematicStateChange(
            bool requestFK = false, bool requestVelFK = false, bool requestAccFK = false);
            
        void notifyKinematicStateChange(
            boost::signals::connection& connectionToBlock,
            bool requestFK = false, bool requestVelFK = false, bool requestAccFK = false);

        SignalProxy< boost::signal<void()> > sigKinematicStateEdited() {
            return sigKinematicStateEdited_.signal();
        }

        /**
           @ret WorldItem that contains this body item if exists.
        */
        WorldItem* worldItem() { return worldItem_; }

        void updateColdetModelPositions(bool force = false);

        void enableSelfCollisionDetection(bool on);
        bool isSelfCollisionDetectionEnabled() {
            return isSelfCollisionDetectionEnabled_;
        }
        bool updateSelfCollisions(bool force = false);
        void clearSelfCollisions();

        std::vector<ColdetLinkPairPtr> selfColdetPairs;
        boost::dynamic_bitset<> selfCollisionLinkBitSet;

        SignalProxy< boost::signal<void()> > sigSelfCollisionsUpdated() {
            return sigSelfCollisionsUpdated_;
        }
        SignalProxy< boost::signal<void()> > sigSelfCollisionLinkSetChanged() {
            return sigSelfCollisionLinkSetChanged_;
        }
            
        std::vector<ColdetLinkPairPtr>& worldColdetPairsOfLink(int linkIndex) {
            return worldColdetPairsOfLink_[linkIndex];
        }
        const std::vector<ColdetLinkPairPtr>& worldColdetPairsOfLink(int linkIndex) const {
            return worldColdetPairsOfLink_[linkIndex];
        }

        boost::dynamic_bitset<> worldCollisionLinkBitSet;

        SignalProxy< boost::signal<void()> > sigWorldCollisionsUpdated() {
            return sigWorldCollisionsUpdated_;
        }
        SignalProxy< boost::signal<void()> > sigWorldCollisionLinkSetChanged() {
            return sigWorldCollisionLinkSetChanged_;
        }
        void notifyWorldCollisionLinkSetChange() {
            sigWorldCollisionLinkSetChanged_();
        }
        void notifyWorldCollisionUpdate() {
            sigWorldCollisionsUpdated_();
        }

        const Vector3& centerOfMass();

        bool doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor = false);

        inline const Vector3& zmp() { return zmp_; }
        void setZmp(const Vector3& zmp) { zmp_ = zmp; }

        void editZmp(const Vector3& zmp);

        enum PositionType { CM_PROJECTION, HOME_COP, RIGHT_HOME_COP, LEFT_HOME_COP, ZMP };
            
        boost::optional<Vector3> getParticularPosition(PositionType posType);

        //void setZmp(ZmpPosition position);

        bool setStance(double width);
            
        const std::string modelFilePath() { return modelFilePath_; }


      protected:
            
        virtual ItemPtr doDuplicate() const;
        virtual void doPutProperties(PutPropertyFunction& putProperty);
        virtual bool store(Archive& archive);
        virtual bool restore(const Archive& archive);
            
      private:

        BodyPtr body_;
        ModelNodeSetPtr modelNodeSet_;
        std::string modelFilePath_;
        std::string errorMessage_;

        enum { UF_POSITIONS, UF_VELOCITIES, UF_ACCELERATIONS, UF_CM, UF_ZMP, NUM_UPUDATE_FLAGS };
        std::bitset<NUM_UPUDATE_FLAGS> updateFlags;

        typedef boost::shared_ptr<KinematicState> KinematicStatePtr;
        std::deque<KinematicStatePtr> kinematicStateHistory;
        size_t currentHistoryIndex;
        bool isCurrentKinematicStateInHistory;
        bool needToAppendKinematicStateToHistory;

        LazySignal< boost::signal<void()> > sigKinematicStateChanged_;
        LazySignal< boost::signal<void()> > sigKinematicStateEdited_;
        LazySignal< boost::signal<void()> > sigStateUpdated_;

        bool isCallingSlotsOnKinematicStateEdited;
        bool isFkRequested;
        bool isVelFkRequested;
        bool isAccFkRequested;
        Link* currentBaseLink_;
        LinkTraverse fkTraverse;
        PinDragIKptr pinDragIK_;
        Vector3 zmp_;

        WorldItem* worldItem_;
        std::vector< std::vector<ColdetLinkPairPtr> > worldColdetPairsOfLink_;

        bool isSelfCollisionDetectionEnabled_;
        bool isSelfCollisionUpdateNeeded;
        bool isColdetModelPositionUpdateNeeded;
        KinematicsBar* kinematicsBar;
        LazyCaller updateSelfCollisionsCaller;
        boost::signal<void()> sigSelfCollisionsUpdated_;
        boost::signal<void()> sigSelfCollisionLinkSetChanged_;
        boost::signal<void()> sigWorldCollisionsUpdated_;
        boost::signal<void()> sigWorldCollisionLinkSetChanged_;
            
        void initBody();
        void emitSigKinematicStateChanged();
        void emitSigKinematicStateEdited();
        void appendKinematicStateToHistory();
        void updateSelfColdetPairs();
        bool onSelfCollisionDetectionPropertyChanged(bool on);
        void onPositionChanged();
    };
}

#endif
