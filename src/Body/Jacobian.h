
#ifndef CNOID_BODY_JACOBIAN_H_INCLUDED
#define CNOID_BODY_JACOBIAN_H_INCLUDED

#include "Body.h"
#include "exportdecl.h"

namespace cnoid {

    CNOID_EXPORT void calcCmJacobian(const BodyPtr& body, Link* base, Eigen::MatrixXd& J);
    
}

#endif
