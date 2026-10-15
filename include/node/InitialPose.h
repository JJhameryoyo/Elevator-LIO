/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Manual body-pose seed conversion, independent of ROS message types.
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <cmath>

namespace elevator_lio {

using PoseCovariance = Eigen::Matrix<double, 6, 6>;

struct InitialPoseSeed {
    Eigen::Quaterniond orientation;
    Eigen::Vector3d position;
    // ESKF ordering: local IMU rotation error, world position error.
    PoseCovariance covariance;
};

inline bool makeInitialPoseSeed(const Eigen::Vector3d& body_position,
                                const Eigen::Quaterniond& body_orientation,
                                const PoseCovariance& body_covariance,
                                const Eigen::Matrix3d& imu_R_body,
                                const Eigen::Vector3d& imu_t_body,
                                InitialPoseSeed& seed) {
    const double norm = body_orientation.norm();
    if (!body_position.allFinite() || !body_orientation.coeffs().allFinite() ||
        !std::isfinite(norm) || norm < 1e-6 || !body_covariance.allFinite() ||
        !imu_R_body.allFinite() || !imu_t_body.allFinite()) return false;

    // ROS covariance order: x, y, z, fixed-axis rotation about world x, y, z.
    PoseCovariance covariance = body_covariance;
    if (covariance.isZero(0.0)) {
        covariance.setZero();
        covariance.diagonal().head<3>().setConstant(0.25); // 0.5 m standard deviation
        constexpr double angle_sigma = 0.2617993877991494; // 15 degrees
        covariance.diagonal().tail<3>().setConstant(angle_sigma * angle_sigma);
    } else {
        if (!covariance.isApprox(covariance.transpose(), 1e-8)) return false;
        covariance = (0.5 * (covariance + covariance.transpose())).eval();
        Eigen::SelfAdjointEigenSolver<PoseCovariance> solver(covariance);
        if (solver.info() != Eigen::Success || solver.eigenvalues().minCoeff() < -1e-9)
            return false;
        // A zero variance must not lock an unobserved axis during matching.
        covariance = solver.eigenvectors() *
            solver.eigenvalues().cwiseMax(1e-6).asDiagonal() * solver.eigenvectors().transpose();
    }

    seed.orientation = (body_orientation.normalized() * Eigen::Quaterniond(imu_R_body).conjugate()).normalized();
    const Eigen::Matrix3d world_R_imu = seed.orientation.toRotationMatrix();
    const Eigen::Vector3d offset = world_R_imu * imu_t_body;
    seed.position = body_position - offset;

    Eigen::Matrix3d offset_skew;
    offset_skew << 0.0, -offset.z(), offset.y(),
                   offset.z(), 0.0, -offset.x(),
                   -offset.y(), offset.x(), 0.0;
    PoseCovariance jacobian = PoseCovariance::Zero();
    jacobian.block<3, 3>(0, 3) = world_R_imu.transpose();
    jacobian.block<3, 3>(3, 0).setIdentity();
    jacobian.block<3, 3>(3, 3) = offset_skew;
    seed.covariance = jacobian * covariance * jacobian.transpose();
    return seed.position.allFinite() && seed.orientation.coeffs().allFinite() &&
           seed.covariance.allFinite();
}

} // namespace elevator_lio
