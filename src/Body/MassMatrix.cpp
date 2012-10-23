/**
   \author Shin'ichiro Nakaoka
*/

#include "MassMatrix.h"
#include "Link.h"
#include "InverseDynamics.h"

using namespace cnoid;

namespace {

    void setColumnOfMassMatrix(const BodyPtr& body, MatrixXd& out_M, int column)
    {
        Vector3 f, tau;
        
        calcInverseDynamics(body, body->rootLink(), f, tau);
        
        if(!body->isStaticModel()){
            tau -= body->rootLink()->p.cross(f);
            out_M.block<6, 1>(0, column) << f, tau;
        }
        
        const int n = body->numJoints();
        for(int i = 0; i < n; ++i){
            Link* joint = body->joint(i);
            out_M(i + 6, column) = joint->u;
        }
    }
}


namespace cnoid {

    /**
       calculate the mass matrix using the unit vector method
       \todo replace the unit vector method here with a more efficient method
       
       The motion equation (dv != dvo)
       |       |   | dv   |   |    |   | fext      |
       | out_M | * | dw   | + | b1 | = | tauext    |
       |       |   |ddq   |   |    |   | u         |
    */
    void calcMassMatrix(const BodyPtr& body, Eigen::MatrixXd& out_M)
    {
        const Vector3 g(0, 0, 9.8);
        int nJ = body->numJoints();
        int totaldof = nJ;
        if(!body->isStaticModel()){
            totaldof += 6;
        }
        Link* rootLink = body->rootLink();
        out_M.resize(totaldof, totaldof);
        
        // preserve and clear the joint accelerations
        VectorXd ddqorg(nJ);
        VectorXd uorg(nJ);
        for(int i = 0; i < nJ; ++i){
            Link* joint = body->joint(i);
            ddqorg[i] = joint->ddq;
            uorg  [i] = joint->u;
            joint->ddq = 0.0;
        }

        // preserve and clear the root link acceleration
        const Vector3 dvoorg = rootLink->dvo;
        const Vector3 dworg  = rootLink->dw;
        const Vector3 root_w_x_v = rootLink->w.cross(rootLink->vo + rootLink->w.cross(rootLink->p));
        rootLink->dvo = g - root_w_x_v;   // dv = g, dw = 0
        rootLink->dw.setZero();
	
        MatrixXd b1(totaldof, 1);
        
        setColumnOfMassMatrix(body, b1, 0);

        if(!body->isStaticModel()){
            for(int i=0; i < 3; ++i){
                rootLink->dvo[i] += 1.0;
                setColumnOfMassMatrix(body, out_M, i);
                rootLink->dvo[i] -= 1.0;
            }
            for(int i=0; i < 3; ++i){
                rootLink->dw[i] = 1.0;
                const Vector3 dw_x_p = rootLink->dw.cross(rootLink->p);	// spatial acceleration caused by ang. acc.
                rootLink->dvo -= dw_x_p;
                setColumnOfMassMatrix(body, out_M, i + 3);
                rootLink->dvo += dw_x_p;
                rootLink->dw[i] = 0.0;
            }
        }

        for(int i = 0; i < nJ; ++i){
            Link* joint = body->joint(i);
            joint->ddq = 1.0;
            int j = i + 6;
            setColumnOfMassMatrix(body, out_M, j);
            out_M(j, j) += joint->Jm2; // motor inertia
            joint->ddq = 0.0;
        }

        // subtract the constant term
        for(int i = 0; i < out_M.cols(); ++i){
            out_M.col(i) -= b1;
        }

        // recover state
        for(int i = 0; i < nJ; ++i){
            Link* joint = body->joint(i);
            joint->ddq  = ddqorg[i];
            joint->u    = uorg  [i];
        }
        rootLink->dvo = dvoorg;
        rootLink->dw  = dworg;
    }
}
