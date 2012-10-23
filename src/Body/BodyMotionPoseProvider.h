/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_MOTION_POSE_PROVIDER_H_INCLUDED
#define CNOID_BODY_BODY_MOTION_POSE_PROVIDER_H_INCLUDED

#include "Body.h"
#include "Link.h"
#include "PoseProvider.h"
#include "BodyMotion.h"
#include "exportdecl.h"

namespace cnoid {

    class CNOID_EXPORT BodyMotionPoseProvider : public PoseProvider
    {
    public:
        BodyMotionPoseProvider();
        BodyMotionPoseProvider(BodyPtr body, BodyMotionPtr motion);

        void initialize(BodyPtr body, BodyMotionPtr motion);

        bool updateMotion();

        virtual Body* body() const;
        virtual double beginningTime() const;
        virtual double endingTime() const;
        virtual bool seek(double time);
        virtual bool seek(double time, int waistLinkIndex, const Vector3& waistTranslation);
        virtual int baseLinkIndex() const;
        virtual bool getBaseLinkPosition(Vector3& out_p, Matrix3& out_R) const;
        virtual void getJointPositions(std::vector< boost::optional<double> >& out_q) const;
        virtual boost::optional<Vector3> zmp() const;

    private:
        BodyPtr body_;
        BodyMotionPtr motion;
        int minNumJoints;
        std::vector<Link*> footLinks;
        std::vector<JointPathPtr> ikPaths;
        MultiAffine3SeqPtr footLinkPositions;
        std::vector<double> qTranslated;
        Vector3 p_waist;
        Matrix3 R_waist;
        Vector3 zmp_;

        bool seek(double time, int waistLinkIndex, const Vector3& waistTranslation, bool applyWaistTranslation);
    };
}

#endif
