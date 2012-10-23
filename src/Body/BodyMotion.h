/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#ifndef CNOID_BODY_BODY_MOTION_H_INCLUDED
#define CNOID_BODY_BODY_MOTION_H_INCLUDED

#include <cnoid/MultiValueSeq>
#include <cnoid/MultiAffine3Seq>
#include <cnoid/Vector3Seq>
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT BodyMotion : public MultiSeqBase
    {
      public:
        BodyMotion();
        BodyMotion(const BodyMotion& org);
            
        virtual void setDimension(int numFrames, int numJoints, bool clearNewArea = false);

        void setDimension(int numFrames, int numJoints, int numLinks, bool clearNewArea = false);

        virtual void setNumParts(int numParts, bool clearNewElements = false);
        virtual int getNumParts() const;

        inline int numJoints() const { return jointPosSeq_->numParts(); }
        inline int numLinks() const { return linkPosSeq_->numParts(); }

        inline double frameRate() const { return jointPosSeq_->frameRate(); }
        virtual double getFrameRate() const;
        virtual void setFrameRate(double frameRate);

        inline int numFrames() const {
            return std::max(jointPosSeq_->numFrames(), linkPosSeq_->numFrames());
        }
        virtual int getNumFrames() const;
        virtual void setNumFrames(int n, bool clearNewArea = false);

        inline MultiValueSeqPtr& jointPosSeq() {
            return jointPosSeq_;
        }

        inline const MultiValueSeqPtr& jointPosSeq() const {
            return jointPosSeq_;
        }

        inline MultiAffine3SeqPtr& linkPosSeq() {
            return linkPosSeq_;
        }

        inline const MultiAffine3SeqPtr& linkPosSeq() const {
            return linkPosSeq_;
        }
        
        inline bool hasRelativeZmpSeq() { return relativeZmpSeq_; }
        const Vector3SeqPtr& relativeZmpSeq();

        class Frame {
            const BodyMotion& motion_;
            int frame_;
            Frame(const BodyMotion& motion, int frame) : motion_(motion), frame_(frame) { }
        public:
            Frame(const Frame& org) : motion_(org.motion_), frame_(org.frame_) { }
            inline BodyMotion& motion() { return const_cast<BodyMotion&>(motion_); }
            inline const BodyMotion& motion() const { return motion_; }
            inline int frame() const { return frame_; }

            friend class BodyMotion;
        };
            
        inline Frame frame(int frame) { return Frame(*this, frame); }
        inline const Frame frame(int frame) const { return Frame(*this, frame); }

        virtual bool read(const YamlMapping& archive);
        virtual bool write(YamlWriter& writer);

        bool loadStandardYamlFormat(const std::string& filename);
        bool saveAsStandardYamlFormat(const std::string& filename);
            
      private:

        MultiValueSeqPtr jointPosSeq_;
        MultiAffine3SeqPtr linkPosSeq_;
        Vector3SeqPtr relativeZmpSeq_;
    };

    typedef boost::shared_ptr<BodyMotion> BodyMotionPtr;

    class Body;

    CNOID_EXPORT BodyMotion::Frame& operator<<(const BodyMotion::Frame& frame, const Body& body);
    CNOID_EXPORT BodyMotion::Frame& operator>>(const BodyMotion::Frame& frame, const Body& body);
}

#endif
