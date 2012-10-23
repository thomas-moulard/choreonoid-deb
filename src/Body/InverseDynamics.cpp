/**
   @author Shin'ichiro Nakaoka
*/

#include "InverseDynamics.h"
#include "Link.h"
#include <cnoid/EigenUtil>

/*
 *  see Kajita et al. Humanoid Robot Ohm-sha,  p.210
 */

using namespace cnoid;

namespace cnoid {

    void calcInverseDynamics(const BodyPtr& body, Link* ptr, Vector3& out_f, Vector3& out_tau)
    {	
        Link* parent = ptr->parent;
        if(parent){
            Vector3 sv, sw;

            if(ptr->jointType != Link::FIXED_JOINT){
                sw.noalias() = parent->R * ptr->a;
                sv = ptr->p.cross(sw);
            } else {
                sw.setZero();
                sv.setZero();
            }
            const Vector3 dsv = parent->w.cross(sv) + parent->vo.cross(sw);
            const Vector3 dsw = parent->w.cross(sw);

            ptr->dw  = parent->dw  + dsw * ptr->dq + sw * ptr->ddq;
            ptr->dvo = parent->dvo + dsv * ptr->dq + sv * ptr->ddq;

            ptr->sw = sw;
            ptr->sv = sv;
        }
	
        const Vector3 c = ptr->R * ptr->c + ptr->p;
        Matrix3 I = ptr->R * ptr->I * ptr->R.transpose();
        const Matrix3 c_hat = hat(c);
        I.noalias() += ptr->m * c_hat * c_hat.transpose();
        const Vector3 P = ptr->m * (ptr->vo + ptr->w.cross(c));
        const Vector3 L = ptr->m * c.cross(ptr->vo) + I * ptr->w;

        out_f   = ptr->m * (ptr->dvo + ptr->dw.cross(c)) + ptr->w.cross(P);
        out_tau = ptr->m * c.cross(ptr->dvo) + I * ptr->dw + ptr->vo.cross(P) + ptr->w.cross(L);

        if(ptr->child){
            Vector3 f_c;
            Vector3 tau_c;
            calcInverseDynamics(body, ptr->child, f_c, tau_c);
            out_f   += f_c;
            out_tau += tau_c;
        }

        ptr->u = ptr->sv.dot(out_f) + ptr->sw.dot(out_tau);

        if(ptr->sibling){
            Vector3 f_s;
            Vector3 tau_s;
            calcInverseDynamics(body, ptr->sibling, f_s, tau_s);
            out_f   += f_s;
            out_tau += tau_s;
        }
    }
}

