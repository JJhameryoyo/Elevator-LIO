#include "node/LocalizationProgress.h"
#include <iostream>
#include <stdexcept>
#include <string>

int main() {
    elevator_lio::LocalizationProgress p;
    unsigned int checks = 0;
    auto expect = [&](const char* expected) {
        const std::string actual = elevator_lio::localizationProgressReason(p);
        if (actual != expected) throw std::runtime_error(actual + " != " + expected);
        ++checks;
    };
    expect("no_imu_received_by_lio");
    p.imu_messages = 200;
    expect("initializing_imu");
    p.imu_initialized = true;
    expect("waiting_for_initial_pose");
    p.seed_pending = true;
    expect("initial_pose_queued_not_applied");
    p.ready = true;
    expect("no_lidar_received_by_lio");
    p.lidar_messages = 10;
    p.lidar_age = 3.0;
    expect("lidar_input_stale");
    p.lidar_age = 0.0;
    expect("waiting_for_valid_lidar_frame");
    p.has_frame = true;
    p.frame_begin = 10.0;
    p.frame_end = 10.1;
    p.imu_time = 10.0;
    p.seed_time = 9.0;
    expect("waiting_for_imu_to_reach_scan_end");
    p.seed_time = 10.0;
    expect("scan_time_not_after_initial_pose");
    p.seed_time = 9.0;
    p.imu_time = 10.2;
    expect("waiting_for_scan_update");
    p.pose_messages = 1;
    p.pose_age = 0.2;
    expect("publishing_current_pose");
    p.pose_age = 3.0;
    p.has_frame = false;
    expect("waiting_for_valid_lidar_frame");
    p.imu_age = 3.0;
    expect("imu_input_stale");
    std::cout << "PASS: " << checks << " localization progress checks\n";
}
