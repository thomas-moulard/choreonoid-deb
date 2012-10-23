/** 
    \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_INVERSE_KINEMATICS_H_INCLUDED
#define CNOID_INVERSE_KINEMATICS_H_INCLUDED

#include <boost/shared_ptr.hpp>
#include <cnoid/EigenTypes>

namespace cnoid {

    class InverseKinematics
    {
      public:
        enum AxisSet { NO_AXES = 0, TRANSLATION_3D = 0x1, ROTATION_3D = 0x2, TRANSFORM_6D = 0x3 };
        virtual ~InverseKinematics() {  }
        virtual AxisSet axisType() const { return TRANSFORM_6D; }
        virtual bool calcInverseKinematics(const Vector3& end_p, const Matrix3& end_R) = 0;
    };

    typedef boost::shared_ptr<InverseKinematics> InverseKinematicsPtr;
}

#endif
