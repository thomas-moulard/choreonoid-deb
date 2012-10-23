/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_INVERSE_DYNAMICS_H_INCLUDED
#define CNOID_BODY_INVERSE_DYNAMICS_H_INCLUDED

#include "Body.h"
#include "exportdecl.h"

namespace cnoid {

    CNOID_EXPORT void calcInverseDynamics(const BodyPtr& body, Link* link, Vector3& out_f, Vector3& out_tau);

}

#endif
