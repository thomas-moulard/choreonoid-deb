/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "CompositeIK.h"
#include "Link.h"
#include "JointPath.h"

using namespace std;
using namespace boost;
using namespace cnoid;


CompositeIK::CompositeIK(BodyPtr body, Link* targetLink)
{
    body_ = body;
    targetLink_ = targetLink;
    isAnalytical_ = false;
}


CompositeIK::~CompositeIK()
{

}


bool CompositeIK::addBaseLink(Link* baseLink)
{
    if(baseLink && targetLink_){
        JointPathPtr path = body_->getJointPath(targetLink_, baseLink);
        if(path){
            isAnalytical_ = pathList.empty() ? path->hasAnalyticalIK() : (isAnalytical_ && path->hasAnalyticalIK());
            PathInfo info;
            info.path = path;
            info.p0 = baseLink->p;
            info.R0 = baseLink->R;
            pathList.push_back(info);
            return true;
        }
    }
    return false;
}


void CompositeIK::setMaxIKerror(double e)
{
    for(size_t i=0; i < pathList.size(); ++i){
        pathList[i].path->setMaxIKerror(e);
    }
}


bool CompositeIK::hasAnalyticalIK() const
{
    return isAnalytical_;
}


bool initialize();


bool CompositeIK::calcInverseKinematics(const Vector3& p, const Matrix3& R)
{
    const int n = body_->numJoints();

    Vector3 p0 = targetLink_->p;
    Matrix3 R0 = targetLink_->R;
    std::vector<double> q0(n);
    for(int i=0; i < n; ++i){
        q0[i] = body_->joint(i)->q;
    }

    targetLink_->p = p;
    targetLink_->R = R;
    bool solved = true;
    for(size_t i=0; i < pathList.size(); ++i){
        PathInfo& info = pathList[i];
        solved = info.path->calcInverseKinematics(p, R, info.p0, info.R0);
        if(!solved){
            break;
        }
        Link* endLink = info.path->endLink();
        endLink->p = info.p0;
        endLink->R = info.R0;
    }

    if(!solved){
        targetLink_->p = p0;
        targetLink_->R = R0;
        for(int i=0; i < n; ++i){
            body_->joint(i)->q = q0[i];
        }
        for(size_t i=0; i < pathList.size(); ++i){
            PathInfo& info = pathList[i];
            info.path->calcForwardKinematics();
            Link* endLink = info.path->endLink();
            endLink->p = info.p0;
            endLink->R = info.R0;
        }
    }

    return solved;
}
