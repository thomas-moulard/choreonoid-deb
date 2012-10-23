/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "BodyMotionPoseProvider.h"
#include "JointPath.h"

using namespace std;
using namespace boost;
using namespace cnoid;

#ifdef _MSC_VER
namespace {
	inline long lround(double x) {
		return static_cast<long>((x > 0.0) ? floor(x + 0.5) : ceil(x -0.5));
	}
}
#endif


BodyMotionPoseProvider::BodyMotionPoseProvider()
{

}


BodyMotionPoseProvider::BodyMotionPoseProvider(BodyPtr body_, BodyMotionPtr motion)
{
    initialize(body_, motion);
}


void BodyMotionPoseProvider::initialize(BodyPtr body__, BodyMotionPtr motion)
{
    body_ = body__->duplicate();
    this->motion = motion;
    footLinkPositions.reset(new MultiAffine3Seq());
    footLinks.clear();
    ikPaths.clear();

    YamlSequence& footLinkInfos = *body_->info()->findSequence("footLinks");
    if(footLinkInfos.isValid()){
        for(int i=0; i < footLinkInfos.size(); ++i){
            const YamlMapping& footLinkInfo = *footLinkInfos[i].toMapping();
            Link* link = body_->link(footLinkInfo["link"].toString());

            JointPathPtr ikPath = body_->getJointPath(body_->rootLink(), link);
            if(ikPath && ikPath->hasAnalyticalIK()){
                footLinks.push_back(link);
                ikPaths.push_back(ikPath);
            }
        }
    }

    updateMotion();
}    


bool BodyMotionPoseProvider::updateMotion()
{
    const int numFrames = motion->numFrames();
    footLinkPositions->setDimension(numFrames, footLinks.size());
    footLinkPositions->setFrameRate(motion->frameRate());

    MultiValueSeqPtr qseq = motion->jointPosSeq();
    MultiAffine3SeqPtr pseq = motion->linkPosSeq();

    if(pseq->numParts() < 1){
        return false;
    }

    Link* rootLink = body_->rootLink();
    minNumJoints = std::min(body_->numJoints(), qseq->numParts());
    qTranslated.resize(minNumJoints);

    for(int frame=0; frame < numFrames; ++frame){

        const Affine3& p = pseq->at(frame, 0);
        rootLink->p = p.translation();
        rootLink->R = p.linear();

        MultiValueSeq::View q = qseq->frame(frame);

        for(int i=0; i < minNumJoints; ++i){
            body_->joint(i)->q = q[i];
        }

        body_->calcForwardKinematics();
        
        for(size_t i=0; i < footLinks.size(); ++i){
            Link* footLink = footLinks[i];
            Affine3& p = footLinkPositions->at(frame, i);
            p.translation() = footLink->p;
            p.linear() = footLink->R;
        }
    }

    return true;
}


Body* BodyMotionPoseProvider::body() const
{
    return body_.get();
}


double BodyMotionPoseProvider::beginningTime() const
{
    return 0.0;
}


double BodyMotionPoseProvider::endingTime() const
{
    return (motion->numFrames() - 1) / motion->frameRate();
}


bool BodyMotionPoseProvider::seek
(double time, int waistLinkIndex, const Vector3& waistTranslation, bool applyWaistTranslation)
{
    int frame = lround(time * motion->frameRate());
    if(frame >= motion->numFrames()){
        frame = motion->numFrames() - 1;
    }
    const MultiValueSeq::View q = motion->jointPosSeq()->frame(frame);
    for(int i=0; i < minNumJoints; ++i){
        qTranslated[i] = q[i];
    }

    if(waistLinkIndex != 0){
        return false;
    }
    
    const Affine3& waist = motion->linkPosSeq()->at(frame, 0);
    p_waist = waist.translation();
    R_waist = waist.linear();
    if(applyWaistTranslation){
        p_waist += waistTranslation;
        for(size_t i=0; i < footLinks.size(); ++i){
            const Affine3& foot = footLinkPositions->at(frame, i);
            JointPathPtr ikPath = ikPaths[i];
            ikPath->calcInverseKinematics(p_waist, R_waist, foot.translation(), foot.linear());
            for(int j=0; j < ikPath->numJoints(); ++j){
                Link* joint = ikPath->joint(j);
                qTranslated[joint->jointId] = joint->q;
            }
        }
    }

    if(motion->hasRelativeZmpSeq()){
        const Vector3& relZmp = motion->relativeZmpSeq()->at(frame);
        zmp_.noalias() = waist.linear() * relZmp + waist.translation(); // original world position 
    }
    
    return true;
}


bool BodyMotionPoseProvider::seek(double time)
{
    return seek(time, 0, Vector3::Zero(), false);
}


bool BodyMotionPoseProvider::seek(double time, int waistLinkIndex, const Vector3& waistTranslation)
{
    return seek(time, waistLinkIndex, waistTranslation, true);
}


int BodyMotionPoseProvider::baseLinkIndex() const
{
    return 0;
}


bool BodyMotionPoseProvider::getBaseLinkPosition(Vector3& out_p, Matrix3& out_R) const
{
    out_p = p_waist;
    out_R = R_waist;
    return true;
}


void BodyMotionPoseProvider::getJointPositions(std::vector< boost::optional<double> >& out_q) const
{
    int n = body_->numJoints();
    out_q.resize(n);
    for(int i=0; i < n; ++i){
        out_q[i] = qTranslated[i];
    }
}


boost::optional<Vector3> BodyMotionPoseProvider::zmp() const
{
    return zmp_;
}
