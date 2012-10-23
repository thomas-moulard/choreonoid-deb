/**
   @file
   @author Shin'ichiro NAKAOKA
*/

#include "PoseProviderToBodyMotionConverter.h"
#include "Link.h"
#include "LinkPath.h"
#include "Sensor.h"
#include "BodyMotion.h"
#include "PoseProvider.h"

using namespace std;
using namespace boost;
using namespace cnoid;

PoseProviderToBodyMotionConverter::PoseProviderToBodyMotionConverter()
{
    setFullTimeRange();
    allLinkPositionOutputMode = true;
}

    

void PoseProviderToBodyMotionConverter::setTimeRange(double lower, double upper)
{
    lowerTime = std::max(0.0, lower);
    upperTime = std::max(lowerTime, upper);
}

    
void PoseProviderToBodyMotionConverter::setFullTimeRange()
{
    lowerTime = 0.0;
    upperTime = std::numeric_limits<double>::max();
}


void PoseProviderToBodyMotionConverter::setAllLinkPositionOutput(bool on)
{
    allLinkPositionOutputMode = on;
}


bool PoseProviderToBodyMotionConverter::convert(BodyPtr body, PoseProvider* provider, BodyMotion& motion)
{
    const double frameRate = motion.frameRate();
    const int beginningFrame = static_cast<int>(frameRate * std::max(provider->beginningTime(), lowerTime));
    const int endingFrame = static_cast<int>(frameRate * std::min(provider->endingTime(), upperTime));
    const int numJoints = body->numJoints();
    const int numLinksToPut = (allLinkPositionOutputMode ? body->numLinks() : 1);
    
    motion.setDimension(endingFrame + 1, numJoints, numLinksToPut, true);

    MultiValueSeq& qseq = *motion.jointPosSeq();
    MultiAffine3Seq& pseq = *motion.linkPosSeq();
    Vector3Seq& relZmpSeq = *motion.relativeZmpSeq();
    bool isZmpValid = false;

    Link* rootLink = body->rootLink();
    Link* baseLink = rootLink;

    shared_ptr<LinkTraverse> fkTraverse;
    if(allLinkPositionOutputMode){
        fkTraverse.reset(new LinkTraverse(baseLink, true, true));
    } else {
        fkTraverse.reset(new LinkPath(baseLink, rootLink));
    }

    // store the original state
    vector<double> orgq(numJoints);
    for(int i=0; i < numJoints; ++i){
        orgq[i] = body->joint(i)->q;
    }
    Affine3 orgp;
    orgp.translation() = rootLink->p;
    orgp.linear() = rootLink->R;

    std::vector< boost::optional<double> > jointPositions(numJoints);

    for(int frame = beginningFrame; frame <= endingFrame; ++frame){

        provider->seek(frame / frameRate);

        const int baseLinkIndex = provider->baseLinkIndex();
        if(baseLinkIndex >= 0){
            if(baseLinkIndex != baseLink->index){
                baseLink = body->link(baseLinkIndex);
                if(allLinkPositionOutputMode){
                    fkTraverse->find(baseLink, true, true);
                } else {
                    static_pointer_cast<LinkPath>(fkTraverse)->find(baseLink, rootLink);
                }
            }
            provider->getBaseLinkPosition(baseLink->p, baseLink->R);
        }

        MultiValueSeq::View qs = qseq.frame(frame);
        provider->getJointPositions(jointPositions);
        for(int i=0; i < numJoints; ++i){
            const optional<double>& q = jointPositions[i];
            qs[i] = q ? *q : 0.0;
            body->joint(i)->q = qs[i];
        }

        if(allLinkPositionOutputMode || baseLink != rootLink){
            fkTraverse->calcForwardKinematics();
        }

        for(int i=0; i < numLinksToPut; ++i){
            Affine3& p = pseq.at(frame, i);
            Link* link = body->link(i);
            p.translation() = link->p;
            p.linear() = link->R;
        }

        optional<Vector3> zmp = provider->zmp();
        if(zmp){
            relZmpSeq[frame].noalias() = rootLink->R.transpose() * (*zmp - rootLink->p);
            isZmpValid = true;
        }

    }

    if(!isZmpValid){
        //bodyMotionItem->clearRelativeZmpSeq();
    }

    // restore the original state
    for(int i=0; i < numJoints; ++i){
        body->joint(i)->q = orgq[i];
    }
    rootLink->p = orgp.translation();
    rootLink->R = orgp.linear();
    body->calcForwardKinematics();

    return true;
}
