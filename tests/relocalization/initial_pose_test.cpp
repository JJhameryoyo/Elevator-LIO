#include "node/InitialPose.h"
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace elevator_lio;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

int main() {
    const Eigen::Vector3d p(5.0, 2.0, 8.0);
    const Eigen::Quaterniond q(Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()));
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    const PoseCovariance unspecified = PoseCovariance::Zero();
    InitialPoseSeed seed;

    require(makeInitialPoseSeed(p, q, unspecified, identity, zero, seed), "valid seed rejected");
    require(seed.position.isApprox(p) && seed.orientation.angularDistance(q) < 1e-12,
            "identity extrinsics changed the body pose");
    require(seed.covariance.diagonal().tail<3>().isApprox(Eigen::Vector3d::Constant(0.25)),
            "missing covariance did not use position defaults");

    // Reconstruct the requested body pose from a nontrivial IMU seed.
    const Eigen::Matrix3d imu_R_body =
        (Eigen::AngleAxisd(-0.4, Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    const Eigen::Vector3d imu_t_body(0.3, -0.2, 0.5);
    require(makeInitialPoseSeed(p, q, unspecified, imu_R_body, imu_t_body, seed),
            "nonidentity extrinsics rejected");
    require((seed.position + seed.orientation * imu_t_body).isApprox(p),
            "body to IMU translation used the wrong transform direction");
    require((seed.orientation * Eigen::Quaterniond(imu_R_body)).angularDistance(q) < 1e-12,
            "body to IMU rotation used the wrong transform direction");

    // Compare covariance propagation with numerical derivatives in ROS fixed axes.
    PoseCovariance input_covariance = PoseCovariance::Zero();
    input_covariance.diagonal() << 0.2, 0.3, 0.4, 0.01, 0.02, 0.03;
    require(makeInitialPoseSeed(p, q, input_covariance, imu_R_body, imu_t_body, seed),
            "valid covariance rejected");
    PoseCovariance numerical_jacobian;
    constexpr double step = 1e-6;
    for (int axis = 0; axis < 6; ++axis) {
        auto perturbed_p = p;
        auto perturbed_q = q;
        if (axis < 3) perturbed_p(axis) += step;
        else perturbed_q = Eigen::Quaterniond(
            Eigen::AngleAxisd(step, Eigen::Vector3d::Unit(axis - 3))) * q;
        InitialPoseSeed perturbed;
        require(makeInitialPoseSeed(perturbed_p, perturbed_q, input_covariance,
                                   imu_R_body, imu_t_body, perturbed), "perturbed pose rejected");
        const Eigen::AngleAxisd rotation_error(seed.orientation.conjugate() * perturbed.orientation);
        numerical_jacobian.block<3, 1>(0, axis) = rotation_error.axis() * rotation_error.angle() / step;
        numerical_jacobian.block<3, 1>(3, axis) = (perturbed.position - seed.position) / step;
    }
    require(seed.covariance.isApprox(numerical_jacobian * input_covariance * numerical_jacobian.transpose(), 1e-5),
            "covariance propagation disagrees with finite differences");

    Eigen::Quaterniond unnormalized = q;
    unnormalized.coeffs() *= 3.0;
    require(makeInitialPoseSeed(p, unnormalized, unspecified, identity, zero, seed) &&
            seed.orientation.angularDistance(q) < 1e-12, "quaternion was not normalized");
    require(!makeInitialPoseSeed(p, Eigen::Quaterniond(0, 0, 0, 0), unspecified, identity, zero, seed),
            "zero quaternion accepted");
    auto invalid_p = p;
    invalid_p.x() = std::numeric_limits<double>::quiet_NaN();
    require(!makeInitialPoseSeed(invalid_p, q, unspecified, identity, zero, seed), "NaN position accepted");
    auto invalid_covariance = input_covariance;
    invalid_covariance(0, 0) = -1.0;
    require(!makeInitialPoseSeed(p, q, invalid_covariance, identity, zero, seed), "negative covariance accepted");
    invalid_covariance = input_covariance;
    invalid_covariance(0, 1) = 0.1;
    require(!makeInitialPoseSeed(p, q, invalid_covariance, identity, zero, seed), "asymmetric covariance accepted");
    invalid_covariance = input_covariance;
    invalid_covariance(0, 0) = std::numeric_limits<double>::infinity();
    require(!makeInitialPoseSeed(p, q, invalid_covariance, identity, zero, seed), "infinite covariance accepted");
    invalid_covariance = input_covariance;
    invalid_covariance(0, 1) = invalid_covariance(1, 0) = 1.0;
    require(!makeInitialPoseSeed(p, q, invalid_covariance, identity, zero, seed), "indefinite covariance accepted");
    std::cout << "initial pose conversion tests passed\n";
}
