/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_CHOREOGRAPHY_BODY_POSE_SEQ_ITEM_H_INCLUDED
#define CNOID_CHOREOGRAPHY_BODY_POSE_SEQ_ITEM_H_INCLUDED

#include "PoseSeq.h"
#include "PoseSeqInterpolator.h"
#include <cnoid/Item>
#include <cnoid/BodyMotionItem>
#include <set>
#include "exportdecl.h"

namespace cnoid {

    class TimeBar;
    class BodyItem;
    class BodyMotionGenerationBar;

    class CNOID_EXPORT PoseSeqItem : public cnoid::Item
    {
      public:
        static void initializeClass(ExtensionManager* ext);        
        
        PoseSeqItem();
        PoseSeqItem(const PoseSeqItem& org);
        ~PoseSeqItem();
            
        virtual void setName(const std::string& name);

        inline PoseSeqPtr poseSeq() {
            return seq;
        }

        inline PoseSeqInterpolatorPtr interpolator(){
            return interpolator_;
        }

        inline BodyMotionItem* bodyMotionItem(){
            return bodyMotionItem_.get();
        }

        virtual bool updateInterpolation();
        virtual bool updateTrajectory(bool putMessages = false);

        void beginEditing();
        bool endEditing(bool actuallyModified = true);
        void clearEditHistory();
            
        bool undo();
        bool redo();

        /**
           temporary treatment.
        */
        bool updateKeyPosesWithBalancedTrajectories(std::ostream& os);

      protected:

        virtual ItemPtr doDuplicate() const;
        virtual void doPutProperties(PutPropertyFunction& putProperty);
        virtual bool store(Archive& archive);
        virtual bool restore(const Archive& archive);

        BodyItem* ownerBodyItem;
        PoseSeqPtr seq;
        PoseSeqInterpolatorPtr interpolator_;
        BodyMotionItemPtr bodyMotionItem_;
        boost::signals::connection sigInterpolationParametersChangedConnection;

        ConnectionSet editConnections;

        struct EditHistory {
            /*
              Unify these containers into one which contains elements
              in the operated orders and restore them in the same order
              when undo or redo is carried out.
            */
            PoseSeqPtr removed;
            PoseSeqPtr added;
            EditHistory(){
                removed = new PoseSeq();
                added = new PoseSeq();
            }
            bool empty(){
                return removed->empty() && added->empty();
            }
            void clear(){
                if(!empty()){
                    removed = new PoseSeq();
                    added = new PoseSeq();
                }
            }
        };

        struct PoseIterComp {
            bool operator()(const PoseSeq::iterator it1, const PoseSeq::iterator it2) const {
                return &(*it1) < &(*it2);
            }
        };
        std::set<PoseSeq::iterator, PoseIterComp> inserted;
        std::set<PoseSeq::iterator, PoseIterComp> modified;
            
        double modifyingPoseTime;
        double modifyingPoseTTime;
        PoseUnitPtr modifyingPoseUnitOrg;
        PoseSeq::iterator modifyingPoseIter;

        std::deque<EditHistory> editHistories;
        EditHistory newHistory;
        int currentHistory;

        BodyMotionGenerationBar* generationBar;
        TimeBar* timeBar;

        bool isSelectedPoseMoving;

        void init();
        void convert(BodyPtr orgBody);
        bool convertSub(BodyPtr orgBody, const YamlMapping& convInfo);
        void updateInterpolationParameters();
        void onInserted(PoseSeq::iterator p, bool isMoving);
        void onRemoving(PoseSeq::iterator p, bool isMoving);
        void onModifying(PoseSeq::iterator p);
        void onModified(PoseSeq::iterator p);
        PoseSeq::iterator removeSameElement(PoseSeq::iterator current, PoseSeq::iterator p);
        void onPositionChanged();
    };

    typedef boost::intrusive_ptr<PoseSeqItem> PoseSeqItemPtr;
}

#endif
