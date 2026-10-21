#pragma once
#include <cstdint>

namespace elevator_lio {
struct LocalizationProgress {
    std::uint64_t imu_messages = 0, lidar_messages = 0, pose_messages = 0;
    bool imu_initialized = false, ready = false, seed_pending = false, has_frame = false;
    double imu_age = 0.0, lidar_age = 0.0, pose_age = 0.0;
    double imu_time = 0.0, frame_begin = 0.0, frame_end = 0.0, seed_time = -1.0;
};
// Diagnostic labels only; this function never changes the estimator state.
inline const char* localizationProgressReason(const LocalizationProgress& p) {
    if (p.imu_messages == 0) return "no_imu_received_by_lio";
    if (p.imu_age > 2.0) return "imu_input_stale";
    if (!p.imu_initialized) return "initializing_imu";
    if (!p.ready) return p.seed_pending ? "initial_pose_queued_not_applied" : "waiting_for_initial_pose";
    if (p.lidar_messages == 0) return "no_lidar_received_by_lio";
    if (p.lidar_age > 2.0) return "lidar_input_stale";
    if (p.pose_messages > 0 && p.pose_age <= 2.0) return "publishing_current_pose";
    if (!p.has_frame) return "waiting_for_valid_lidar_frame";
    if (p.frame_begin <= p.seed_time) return "scan_time_not_after_initial_pose";
    if (p.frame_end > p.imu_time) return "waiting_for_imu_to_reach_scan_end";
    return "waiting_for_scan_update";
}
}  // namespace elevator_lio
