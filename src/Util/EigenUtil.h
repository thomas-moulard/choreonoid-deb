/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EIGEN_UTIL_H_INCLUDED
#define CNOID_UTIL_EIGEN_UTIL_H_INCLUDED

#include "EigenTypes.h"
#include "EigenYaml.h"
#include "exportdecl.h"

namespace cnoid {

    const double PI = 3.14159265358979323846;
    const double PI_2 = 1.57079632679489661923;
        
    inline double degree(double rad) { return (180.0 * rad / PI); }
    inline double radian(double deg) { return (PI * deg / 180.0); }
    
    template<typename Derived>
    inline Eigen::Matrix<typename Eigen::MatrixBase<Derived>::Scalar, 3, 1>
    rpyFromRot(const Eigen::MatrixBase<Derived>& R) {
        Vector3 ea = R.eulerAngles(2, 1, 0);
        return Vector3(ea[2], ea[1], ea[0]);
    }

    CNOID_EXPORT Matrix3 rotFromRpy(double r, double p, double y);

    inline Matrix3 rotFromRpy(const Vector3& rpy) {
        return rotFromRpy(rpy[0], rpy[1], rpy[2]);
    }
        
    CNOID_EXPORT Vector3 omegaFromRot(const Matrix3& R);

    inline Matrix3 hat(const Vector3& x) {
        Matrix3 M;
        M <<  0.0, -x(2),   x(1),
             x(2),   0.0,  -x(0),
            -x(1),  x(0),   0.0;
        return M;
    }
}

#endif
