/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_EIGEN_TYPES_H_INCLUDED
#define CNOID_UTIL_EIGEN_TYPES_H_INCLUDED

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace cnoid {

    using Eigen::Matrix2d;
    using Eigen::Vector2d;
    using Eigen::Matrix3d;
    using Eigen::Vector3d;
    using Eigen::Matrix4d;
    using Eigen::Vector4d;
    using Eigen::MatrixXd;
    using Eigen::VectorXd;
    using Eigen::AngleAxisd;

    typedef Eigen::Matrix2d Matrix2;
    typedef Eigen::Vector2d Vector2;
    typedef Eigen::Matrix3d Matrix3;
    typedef Eigen::Vector3d Vector3;
    typedef Eigen::Matrix4d Matrix4;
    typedef Eigen::Vector4d Vector4;
    typedef Eigen::Affine3d Affine3;
}

#endif
