/**
   \file
   \brief Implementations of the LinkPath class
   \author Shin'ichiro Nakaoka
*/
  
#include "JointPath.h"
#include "Link.h"
#include <algorithm>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

JointPath::JointPath()
{
    initialize();
}


JointPath::JointPath(Link* base, Link* end)
    : linkPath(base, end), 
      joints(linkPath.size())
{
    initialize();
    extractJoints();
}


JointPath::JointPath(Link* end)
    : linkPath(end), 
      joints(linkPath.size())
{
    initialize();
    extractJoints();
}


void JointPath::initialize()
{
    maxIKerrorSqr = 1.0e-6 * 1.0e-6;
    isBestEffortIKMode = false;
}
	

JointPath::~JointPath()
{

}


bool JointPath::find(Link* base, Link* end)
{
    if(linkPath.find(base, end)){
        extractJoints();
    }
    onJointPathUpdated();

    return (!joints.empty());
}


bool JointPath::find(Link* end)
{
    linkPath.find(end);
    extractJoints();
    onJointPathUpdated();
	
    return !joints.empty();
}


void JointPath::extractJoints()
{
    numUpwardJointConnections = 0;

    int n = linkPath.size();
    if(n <= 1){
        joints.clear();
    } else {
        int i = 0;
        if(linkPath.isDownward(i)){
            i++;
        }
        joints.resize(n); // reserve size n buffer
        joints.clear();
        int m = n - 1;
        while(i < m){
            Link* link = linkPath[i];
            if(link->jointId >= 0){
                if(link->jointType == Link::ROTATIONAL_JOINT || link->jointType == Link::SLIDE_JOINT){
                    joints.push_back(link);
                    if(!linkPath.isDownward(i)){
                        numUpwardJointConnections++;
                    }
                }
            }
            ++i;
        }
        if(linkPath.isDownward(m-1)){
            Link* link = linkPath[m];
            if(link->jointId >= 0){
                if(link->jointType == Link::ROTATIONAL_JOINT || link->jointType == Link::SLIDE_JOINT){
                    joints.push_back(link);
                }
            }
        }
    }
}


void JointPath::onJointPathUpdated()
{

}


void JointPath::calcJacobian(Eigen::MatrixXd& out_J) const
{
    const int n = joints.size();
    out_J.resize(6, n);
	
    if(n > 0){
		
        Link* targetLink = linkPath.endLink();
		
        for(int i=0; i < n; ++i){
			
            Link* link = joints[i];
			
            switch(link->jointType){
				
            case Link::ROTATIONAL_JOINT:
            {
                Vector3 omega = link->R * link->a;
                const Vector3 arm = targetLink->p - link->p;
                if(!isJointDownward(i)){
                    omega = -omega;
                }
                out_J.col(i) << omega.cross(arm), omega;
            }
            break;
				
            case Link::SLIDE_JOINT:
            {
                Vector3 dp = link->R * link->d;
                if(!isJointDownward(i)){
                    dp = -dp;
                }
                out_J.col(i) << dp, Vector3::Zero();
            }
            break;
				
            default:
                out_J.col(i).setZero();
            }
        }
    }
}


void JointPath::setMaxIKerror(double e)
{
    maxIKerrorSqr = e * e;
}


void JointPath::setBestEffortIKMode(bool on)
{
    isBestEffortIKMode = on;
}


bool JointPath::calcInverseKinematics
(const Vector3& base_p, const Matrix3& base_R, const Vector3& end_p, const Matrix3& end_R)
{
    Link* baseLink = linkPath.baseLink();
    baseLink->p = base_p;
    baseLink->R = base_R;

    if(!hasAnalyticalIK()){
        calcForwardKinematics();
    }
	
    return calcInverseKinematics(end_p, end_R);
}


bool JointPath::calcInverseKinematics(const Vector3& end_p, const Matrix3& end_R)
{
    static const int MAX_IK_ITERATION = 50;
    static const double LAMBDA = 0.9;
    
    if(joints.empty()){
        if(linkPath.empty()){
            return false;
        }
        if(baseLink() == endLink()){
            baseLink()->p = end_p;
            baseLink()->R = end_R;
            return true;
        } else {
            // \todo implement here
            return false;
        }
    }
    
    const int n = numJoints();

    Link* target = linkPath.endLink();

    std::vector<double> qorg(n);
    for(int i=0; i < n; ++i){
        qorg[i] = joints[i]->q;
    }

    MatrixXd J(6, n);
    VectorXd dq(n);

    double errsqr = std::numeric_limits<double>::max();
    bool converged = false;

    for(int i=0; i < MAX_IK_ITERATION; i++){
	
        const Vector3 dp = end_p - target->p;
        const Vector3 omegaLocal = omegaFromRot(target->R.transpose() * end_R);

        if(isBestEffortIKMode){
            const double errsqr0 = errsqr;
            errsqr = dp.squaredNorm() + omegaLocal.squaredNorm();
            if(fabs(errsqr - errsqr0) < maxIKerrorSqr){
                converged = true;
                break;
            }
        } else {
            errsqr = dp.squaredNorm() + omegaLocal.squaredNorm();
            if(errsqr < maxIKerrorSqr){
                converged = true;
                break;
            }
        }

        calcJacobian(J);
        Eigen::Matrix<double, 6, 1> v;
        v << dp, (target->R * omegaLocal);
        if(n == 6){
            dq = J.colPivHouseholderQr().solve(v);
        } else {
            //dq = J.fullPivHouseholderQr().solve(v);
            dq = Eigen::JacobiSVD<MatrixXd>(J, Eigen::ComputeThinU | Eigen::ComputeThinV).solve(v);
        }
		
        for(int j=0; j < n; ++j){
            joints[j]->q += LAMBDA * dq(j);
        }

        calcForwardKinematics();
    }

    if(!converged){
        for(int i=0; i < n; ++i){
            joints[i]->q = qorg[i];
        }
        calcForwardKinematics();
    }
    
    return converged;
}


bool JointPath::hasAnalyticalIK() const
{
    return false;
}


std::ostream& operator<<(std::ostream& os, JointPath& path)
{
    int n = path.numJoints();
    for(int i=0; i < n; ++i){
        Link* link = path.joint(i);
        os << link->name();
        if(i != n){
            os << (path.isJointDownward(i) ? " => " : " <= ");
        }
    }
    os << std::endl;
    return os;
}
