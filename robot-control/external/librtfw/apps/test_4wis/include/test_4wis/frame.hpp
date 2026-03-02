#pragma once

#include <eigen3/Eigen/Dense>

using SO3 = Eigen::Matrix3d;
using so3 = Eigen::Vector3d;

class SE3 {
public:
    SE3(Eigen::Matrix3d rot, Eigen::Vector3d trs) : R(rot), T(trs) {}
    // SE3(Eigen::Matrix3d rot) : R(rot), T(Eigen::Vector3d::Zero()) {}
    SE3() : R(Eigen::Matrix3d::Identity()), T(Eigen::Vector3d::Zero()) {}

    SE3 inverse() {
        return SE3(R.transpose(), -R.transpose() * T);
    }

    Eigen::Matrix3d R;
    Eigen::Vector3d T;
};

SE3 operator *(SE3 lhs, SE3 rhs) {
    return SE3(lhs.R * rhs.R, lhs.R * rhs.T + lhs.T);
}

SE3 operator *(Eigen::Matrix3d lhs, SE3 rhs) {
    return SE3(lhs * rhs.R, lhs * rhs.T);
}

SE3 operator *(SE3 lhs, Eigen::Matrix3d rhs) {
    return SE3(lhs.R * rhs, lhs.T);
}

class se3{
public:
    se3() {
        linear.setZero();
        angular.setZero();
    }
    se3(Eigen::Vector3d linear, Eigen::Vector3d angular): linear(linear), angular(angular) {}

    Eigen::Vector3d linear;
    Eigen::Vector3d angular;

    se3 operator *(double scalar) {
        return se3(linear*scalar, angular*scalar);
    }
    se3 operator /(double scalar) {
        return se3(linear/scalar, angular/scalar);
    }
};

so3 diff(const Eigen::Matrix3d& rot, const Eigen::Matrix3d& base) {
    Eigen::AngleAxisd aa(rot * base.transpose());
    return aa.axis() * aa.angle();
}

se3 diff(const SE3& tf, const SE3& base) {
    return se3(tf.T - base.T, diff(tf.R, base.R));
}
