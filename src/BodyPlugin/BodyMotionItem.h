/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_BODY_MOTION_ITEM_H_INCLUDED
#define CNOID_BODYPLUGIN_BODY_MOTION_ITEM_H_INCLUDED

#include <cnoid/BodyMotion>
#include <cnoid/MultiSeqItem>
#include <cnoid/MultiValueSeqItem>
#include <cnoid/MultiAffine3SeqItem>
#include <cnoid/Vector3SeqItem>
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT BodyMotionItem : public MultiSeqItemBase
    {
      public:
        BodyMotionItem();

        virtual MultiSeqBasePtr seqBase() { return bodyMotion_; }

        inline const BodyMotionPtr& motion() { return bodyMotion_; }

        inline MultiValueSeqItem* jointPosSeqItem() {
            return jointPosSeqItem_.get();
        }

        inline const MultiValueSeqPtr& jointPosSeq() {
            return bodyMotion_->jointPosSeq();
        }

        inline MultiAffine3SeqItem* linkPosSeqItem() {
            return linkPosSeqItem_.get();
        }
            
        inline const MultiAffine3SeqPtr& linkPosSeq() {
            return bodyMotion_->linkPosSeq();
        }

        bool hasRelativeZmpSeqItem() { return bodyMotion_->hasRelativeZmpSeq(); }
        Vector3SeqItem* relativeZmpSeqItem();

        inline Vector3SeqPtr relativeZmpSeq() {
            return relativeZmpSeqItem()->seq();
        }

        virtual void notifyUpdate();

        void updateChildItemLineup();

      protected:

        BodyMotionItem(const BodyMotionItem& org);

        virtual ItemPtr doDuplicate() const;
        virtual bool store(Archive& archive);
        virtual bool restore(const Archive& archive);

      private:

        BodyMotionPtr bodyMotion_;
            
        MultiValueSeqItemPtr jointPosSeqItem_;
        MultiAffine3SeqItemPtr linkPosSeqItem_;
        Vector3SeqItemPtr relativeZmpSeqItem_;

        void initialize();
        void onSubItemUpdated(Item* childItem);
    };

    typedef boost::intrusive_ptr<BodyMotionItem> BodyMotionItemPtr;

    void initializeBodyMotionItem(ExtensionManager& ext);
        
}

#endif
