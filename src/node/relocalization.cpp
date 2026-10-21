/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "support/LIONode.h"
#include "node/InitialPose.h"
#include "estimator/IMUProcess.h"
#include "elevator/ElevatorSelfExit.h"

extern std::shared_ptr<EskfEstimator> p_eskf_estimator;
extern std::shared_ptr<SharedBuffers> shared_bufs;
extern IMUProcess imu_process;
extern ElevatorSelfExit ele_self_exit;
extern bool EXIT_FROM_ELEVATOR;
extern PointCloudXYZI::Ptr down_effect_cloud_lidar;
extern PointCloudXYZI::Ptr down_reject_cloud_lidar;

namespace {
bool poseSeed(const lio_ros::PoseWithCovarianceStamped& message,
              elevator_lio::InitialPoseSeed& seed) {
    const auto& p = message.pose.pose.position;
    const auto& q = message.pose.pose.orientation;
    elevator_lio::PoseCovariance covariance;
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col)
            covariance(row, col) = message.pose.covariance[row * 6 + col];
    }
    return elevator_lio::makeInitialPoseSeed(
        Eigen::Vector3d(p.x, p.y, p.z), Eigen::Quaterniond(q.w, q.x, q.y, q.z), covariance,
        imu_R_lidar * lidar_R_body, imu_t_lidar + imu_R_lidar * lidar_t_body, seed);
}
} // namespace

void LIONode::initialPoseCallback(const lio_ros::PoseWithCovarianceStampedConstPtr& msg) {
    if (!relocation_enable) return;
    initial_pose_event_ = "received";
    // The loaded PCD is already in world. Do not silently reinterpret another frame.
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "world" &&
        msg->header.frame_id != "/world") {
        initial_pose_event_ = "rejected_frame";
        LOG_WARN(Core, "[relocalization] initial pose rejected: frame_id must be world");
        return;
    }
    elevator_lio::InitialPoseSeed seed;
    if (!poseSeed(*msg, seed)) {
        initial_pose_event_ = "rejected_invalid_pose_or_covariance";
        LOG_WARN(Core, "[relocalization] initial pose rejected: invalid position, quaternion or covariance");
        return;
    }
    // Callbacks and timers run in the node's single-threaded executor. Apply only
    // at an IMU boundary, including requests received before IMU initialization.
    pending_initial_pose_ = msg;
    initial_pose_event_ = "queued";
    LOG_INFO(Core, "[relocalization] initial body pose queued; waiting for initialized IMU data");
}

void LIONode::applyPendingInitialPose(double stamp) {
    if (!pending_initial_pose_) return;
    elevator_lio::InitialPoseSeed seed;
    if (!poseSeed(*pending_initial_pose_, seed)) {
        initial_pose_event_ = "failed_to_apply_invalid_pose";
        pending_initial_pose_.reset();
        return;
    }

    State state;
    imu_process.get_state_at_t(state, stamp);
    const Eigen::MatrixXd calibration_covariance = state.P.block<9, 9>(StateIndex::BW, StateIndex::BW);
    state.q = seed.orientation;
    state.p = seed.position;
    state.v.setZero();
    state.in_elevator = false;
    state.z = state.vz = state.az = 0.0;
    state.time = stamp;
    // Retain bias/gravity calibration, but discard old pose/velocity correlations.
    state.P.setZero();
    state.P.block<6, 6>(StateIndex::R, StateIndex::R) = seed.covariance;
    state.P.block<3, 3>(StateIndex::V, StateIndex::V).setIdentity();
    state.P.block<3, 3>(StateIndex::V, StateIndex::V) *= 0.01;
    state.P.block<9, 9>(StateIndex::BW, StateIndex::BW) = calibration_covariance;
    state.P(StateIndex::Z, StateIndex::Z) = 1e-3;
    state.P(StateIndex::VZ, StateIndex::VZ) = 1e-2;
    state.P(StateIndex::AZ, StateIndex::AZ) = 1e-1;

    ELEVATOR_TRIGGER = false;
    EXIT_FROM_ELEVATOR = false;
    ele_self_exit.reset();
    imu_process.elev_process = ElevatorProcess{};
    imu_process.reset_state(state, stamp);
    p_eskf_estimator->set_cur_state(state);
    p_eskf_estimator->state_vector_prior_.clear();
    p_eskf_estimator->state_vector_post_.clear();
    down_effect_cloud_lidar->clear();
    down_reject_cloud_lidar->clear();
    shared_bufs->clearSafe(shared_bufs->wheel_buf);

    // No scan may straddle the discontinuity introduced by a manual pose reset.
    relocalization_time_ = stamp;
    while (const auto frame = shared_bufs->getFrontLidarFrameSafe()) {
        if (frame->base_time > stamp) break;
        shared_bufs->popFrontLidarFrameSafe();
    }
    odom_path_.poses.clear();
    odom_path_.header.frame_id = "world";
    odom_path_.header.stamp = get_ros_time(stamp);
    odom_path_pub_.publish(odom_path_);
    pending_initial_pose_.reset();
    localization_ready_ = true;
    initial_pose_event_ = "applied";
    LOG_INFO(Core, "[relocalization] initial pose applied at IMU time " << stamp
                   << "; matching subsequent scans against the loaded map");
}
