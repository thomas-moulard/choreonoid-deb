/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "LeggedBody.h"
#include "Link.h"
#include "JointPath.h"
#include <cnoid/YamlNodes>
#include <cnoid/EigenYaml>

using namespace cnoid;

LeggedBody::LeggedBody()
{

}


LeggedBody::LeggedBody(const LeggedBody& org)
    : Body(org)
{
    
}


LeggedBody::~LeggedBody()
{

}


BodyPtr LeggedBody::duplicate() const
{
    return new LeggedBody(*this);
}


bool LeggedBody::checkBodyInfoAsLeggedBody(const YamlMappingPtr info)
{
    return info->findSequence("footLinks")->isValid();
}


void LeggedBody::doResetInfo(const YamlMapping& info)
{
    Body::doResetInfo(info);
    
    footInfos.clear();
    const YamlSequence& footLinkNodes = *info["footLinks"].toSequence();
    for(int i=0; i < footLinkNodes.size(); ++i){
        FootInfo footInfo;
        footInfo.info = footLinkNodes[i].toMapping();
        const YamlMapping& footLinkNode = *footInfo.info;
        footInfo.link = link(footLinkNode["link"].toString());
        if(footInfo.link){
            readEx(footLinkNode, "soleCenter", footInfo.soleCenter);
            
            if(!read(footLinkNode, "homeCop", footInfo.homeCop)){
                footInfo.homeCop = footInfo.soleCenter;
            }
            footInfos.push_back(footInfo);
        }
    }
}


bool LeggedBody::doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor)
{
    if(footInfos.empty()){
        return false;
    }

    static const int MAX_ITERATION = 100;
    static const double ERROR_THRESH_SQR = 1.0e-6 * 1.0e-6;
    
    Link* baseFoot = footInfos[0].link;
    Link* waist = rootLink();
    LinkTraverse fkTraverse(waist);
    Vector3 c0 = calcCM();
    bool failed = false;
    int loopCounter = 0;

    while(true){
        Vector3 e = c - c0;
        if(onlyProjectionToFloor){
            e.z() = 0.0;
        }
        if(e.squaredNorm() < ERROR_THRESH_SQR){
            break;
        }
        size_t numDone = 0;
        JointPathPtr baseToWaist = getJointPath(baseFoot, waist);
        if(baseToWaist && baseToWaist->calcInverseKinematics(waist->p + e, waist->R)){
            numDone++;
            for(size_t j=1; j < footInfos.size(); ++j){
                Link* foot = footInfos[j].link;
                JointPathPtr waistToFoot = getJointPath(waist, foot);
                if(waistToFoot){
                    bool ikDone;
                    if(waistToFoot->hasAnalyticalIK()){
                        ikDone = waistToFoot->calcInverseKinematics(foot->p, foot->R);
                    } else {
                        Vector3 p0 = foot->p;
                        Matrix3 R0 = foot->R;
                        waistToFoot->calcForwardKinematics();
                        ikDone = waistToFoot->calcInverseKinematics(p0, R0);
                    }
                    if(ikDone){
                        numDone++;
                    } else {
                        break;
                    }
                }
            }
        }
        if(numDone < footInfos.size()){
            failed = true;
            break;
        }
        if(++loopCounter < MAX_ITERATION){
            fkTraverse.calcForwardKinematics();
            c0 = calcCM();
        } else {
            break;
        }
    }

    return !failed;
}


bool LeggedBody::setStance(double width, Link* baseLink)
{
    if(footInfos.size() != 2){
        return false;
    }

    bool result = false;
    Link* foot[2];
    double sign;
    
    if(footInfos[1].link == baseLink){
        foot[0] = footInfos[1].link;
        foot[1] = footInfos[0].link;
        sign = -1.0;
    } else {
        foot[0] = footInfos[0].link;
        foot[1] = footInfos[1].link;
        sign = 1.0;
    }
        
    const Matrix3& R0 = foot[0]->R;
    const Vector3 baseY(R0(0,1), sign * R0(1,1), 0.0);
    
    Link* waist = rootLink();

    foot[1]->p = foot[0]->p + baseY * width;
    Vector3 wp = (foot[0]->p + foot[1]->p) / 2.0;
    wp[2] = waist->p[2];
        
    JointPathPtr ikPath = getJointPath(foot[0], waist);
    if(ikPath && ikPath->calcInverseKinematics(wp, waist->R)){
        ikPath = getJointPath(waist, foot[1]);
        if(ikPath && ikPath->calcInverseKinematics(foot[1]->p, foot[1]->R)){
            LinkTraverse fkTraverse(baseLink);
            fkTraverse.calcForwardKinematics();
            result = true;
        }
    }

    return result;
}


Vector3 LeggedBody::centerOfSole(int footIndex) const
{
    const FootInfo& info = footInfos[footIndex];
    return info.link->p + info.link->R * info.soleCenter;
}


Vector3 LeggedBody::centerOfSoles() const
{
    Vector3 p = Vector3::Zero();
    int n = footInfos.size();
    if(n == 0){
        throw "LeggedBody::centerOfSoles(): not foot information";
    } else {
        for(int i=0; i < n; ++i){
            const FootInfo& info = footInfos[i];
            p.noalias() += info.link->p + info.link->R * info.soleCenter;
        }
    }
    return p / n;
}


Vector3 LeggedBody::homeCopOfSole(int footIndex) const
{
    const FootInfo& info = footInfos[footIndex];
    return info.link->p + info.link->R * info.homeCop;
}


Vector3 LeggedBody::homeCopOfSoles() const
{
    Vector3 p = Vector3::Zero();
    int n = footInfos.size();
    if(n == 0){
        throw "LeggedBody::homeCopOfSoles(): not foot information";
    } else {
        for(size_t i=0; i < footInfos.size(); ++i){
            const FootInfo& info = footInfos[i];
            p.noalias() += info.link->p + info.link->R * info.homeCop;
        }
    }
    return p / n;
}
