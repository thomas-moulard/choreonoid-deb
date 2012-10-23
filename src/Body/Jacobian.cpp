
#include "Jacobian.h"
#include "Link.h"
#include "JointPath.h"
#include <iostream>

namespace cnoid {

    /**
       @brief compute CoM Jacobian
       @param base link fixed to the environment
       @param J CoM Jacobian
       @note Link::wc must be computed by calcCM() before calling
    */
    void calcCMJacobian(const BodyPtr& body, Link* base, Eigen::MatrixXd& J)
    {
        // prepare subm, submwc

        const int nj = body->numJoints();
        Link* rootLink = body->rootLink();
        
        JointPathPtr jp;
        if(base){
            jp = body->getJointPath(rootLink, base);
            Link* skip = jp->joint(0);
            skip->subm = rootLink->m;
            skip->submwc = rootLink->m * rootLink->wc;
            Link* l = rootLink->child;
            if(l){
                if(l != skip){
                    l->calcSubMassCM();
                    skip->subm += l->subm;
                    skip->submwc += l->submwc;
                }
                l = l->sibling;
                while(l){
                    if (l != skip){
                        l->calcSubMassCM();
                        skip->subm += l->subm;
                        skip->submwc += l->submwc;
                    }
                    l = l->sibling;
                }
            }
        
            // assuming there is no branch between base and root
            for(int i=1; i < jp->numJoints(); i++){
                l = jp->joint(i);
                l->subm = l->parent->m + l->parent->subm;
                l->submwc = l->parent->m * l->parent->wc + l->parent->submwc;
            }
        
            J.resize(3, nj);
        }else{
            rootLink->calcSubMassCM();
            J.resize(3, nj + 6);
        }
    
        // compute Jacobian
        std::vector<int> sgn(nj, 1);
        if(jp){
            for(int i=0; i < jp->numJoints(); i++){
                sgn[jp->joint(i)->jointId] = -1;
            }
        }
    
        for(int i=0; i < nj; i++){
            Link* j = body->joint(i);
            switch(j->jointType){
            case Link::ROTATIONAL_JOINT:
            {
                const Vector3 omega = sgn[j->jointId] * j->R * j->a;
                const Vector3 arm = (j->submwc - j->subm * j->p) / body->totalMass();
                const Vector3 dp = omega.cross(arm);
                J.col(j->jointId) = dp;
                break;
            }
            default:
                std::cerr << "calcCMJacobian() : unsupported jointType("
                          << j->jointType << std::endl;
            }
        }
        if(!base){
            const int c = nj;
            J.block(0, c, 3, 3).setIdentity();

            const Vector3 dp = rootLink->submwc / body->totalMass() - rootLink->p;

            J.block(0, c + 3, 3, 3) <<
                   0.0,  dp(2), -dp(1),
                -dp(2),    0.0,  dp(0),
                 dp(1), -dp(0),    0.0;
        }
    }
}

