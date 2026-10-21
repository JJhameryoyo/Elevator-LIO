#include "support/LIONode.h"
#include "support/common_lib.h"

void LIONode::enqueue_loop_frame(const State& state, const PointCloudXYZI::Ptr& cloud) {
#if LIO_ENABLE_LOOP_CLOSURE
    if (!loop_backend_) return;
    ++loop_input_frames_;
    loop_last_scan_points_ = cloud ? cloud->size() : 0;
    loop_last_in_elevator_ = state.in_elevator;
    if (state.in_elevator || !cloud || cloud->empty()) return;
    Eigen::Matrix4d world_T_imu = Eigen::Matrix4d::Identity();
    world_T_imu.block<3, 3>(0, 0) = state.q.toRotationMatrix();
    world_T_imu.block<3, 1>(0, 3) = state.p;
    Eigen::Matrix4d imu_T_lidar = Eigen::Matrix4d::Identity();
    imu_T_lidar.block<3, 3>(0, 0) = imu_R_lidar;
    imu_T_lidar.block<3, 1>(0, 3) = imu_t_lidar;
    elevator_lio::LoopCloud::Ptr scan(new elevator_lio::LoopCloud);
    scan->reserve(cloud->size());
    for (const auto& point : *cloud) {
        pcl::PointXYZI converted;
        converted.x = point.x; converted.y = point.y; converted.z = point.z;
        converted.intensity = point.intensity;
        scan->push_back(converted);
    }
    loop_backend_->enqueue(lidar_end_time, world_T_imu * imu_T_lidar, scan, false);
#else
    (void)state; (void)cloud;
#endif
}

void LIONode::publish_loop_odometry(const lio_ros::Odometry& odometry) {
#if LIO_ENABLE_LOOP_CLOSURE
    if (!loop_backend_) return;
    const Eigen::Matrix4d correction = loop_backend_->correction();
    const auto& orientation = odometry.pose.pose.orientation;
    const auto& position = odometry.pose.pose.position;
    const Eigen::Quaterniond raw_q(orientation.w, orientation.x, orientation.y, orientation.z);
    const Eigen::Quaterniond q(correction.block<3, 3>(0, 0) * raw_q.toRotationMatrix());
    const Eigen::Vector3d p = correction.block<3, 3>(0, 0) *
        Eigen::Vector3d(position.x, position.y, position.z) + correction.block<3, 1>(0, 3);
    auto corrected = odometry;
    corrected.header.frame_id = "map";
    corrected.pose.pose.position.x = p.x();
    corrected.pose.pose.position.y = p.y();
    corrected.pose.pose.position.z = p.z();
    corrected.pose.pose.orientation.x = q.x();
    corrected.pose.pose.orientation.y = q.y();
    corrected.pose.pose.orientation.z = q.z();
    corrected.pose.pose.orientation.w = q.w();
    loop_odom_pub_.publish(corrected);

    lio_ros::TransformStamped tf;
    tf.header = corrected.header;
    tf.child_frame_id = "world";
    const Eigen::Quaterniond map_q(correction.block<3, 3>(0, 0));
    tf.transform.translation.x = correction(0, 3);
    tf.transform.translation.y = correction(1, 3);
    tf.transform.translation.z = correction(2, 3);
    tf.transform.rotation.x = map_q.x();
    tf.transform.rotation.y = map_q.y();
    tf.transform.rotation.z = map_q.z();
    tf.transform.rotation.w = map_q.w();
    tf_broadcaster_->sendTransform(tf);
#else
    (void)odometry;
#endif
}

void LIONode::publish_loop_results() {
#if LIO_ENABLE_LOOP_CLOSURE
    if (!loop_backend_) return;
    const auto result = loop_backend_->snapshot();
    const auto now = std::chrono::steady_clock::now();
    static auto last_status = std::chrono::steady_clock::time_point{};
    if (now - last_status >= std::chrono::seconds(5)) {
        last_status = now;
        LOG_INFO(Core, "loop map status: publish_map=" << loop_publish_map_
                      << " input_frames=" << loop_input_frames_
                      << " last_scan_points=" << loop_last_scan_points_
                      << " in_elevator=" << loop_last_in_elevator_
                      << " keyframes=" << result.poses.size()
                      << " map_points=" << (result.map ? result.map->size() : 0)
                      << " dropped_frames=" << result.dropped_frames);
    }
    if (!result.error.empty() && result.error != loop_error_) {
        loop_error_ = result.error;
        LOG_ERROR(Core, "loop closure backend stopped: " << loop_error_);
    }
    if (result.loops != loop_count_) {
        loop_count_ = result.loops;
        LOG_INFO(Core, "accepted loop constraints=" << loop_count_
                      << " keyframes=" << result.poses.size()
                      << " dropped input frames=" << result.dropped_frames);
    }
    if (result.poses.empty()) return;
    lio_ros::Path path;
    path.header.frame_id = "map";
    path.header.stamp = get_ros_time(result.poses.back().stamp);
    for (const auto& key : result.poses) {
        lio_ros::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = get_ros_time(key.stamp);
        pose.pose.position.x = key.pose(0, 3);
        pose.pose.position.y = key.pose(1, 3);
        pose.pose.position.z = key.pose(2, 3);
        const Eigen::Quaterniond q(key.pose.block<3, 3>(0, 0));
        pose.pose.orientation.x = q.x(); pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
        path.poses.push_back(pose);
    }
    loop_path_pub_.publish(path);
    if (loop_publish_map_) {
        // Map assembly runs on the worker; published clouds are immutable snapshots.
        loop_backend_->requestMap();
        if (result.map && !result.map->empty()) {
            lio_ros::PointCloud2 message;
            pcl::toROSMsg(*result.map, message);
            message.header = path.header;
            message.header.stamp = get_ros_time(result.map_stamp);
            loop_map_pub_.publish(message);
        }
    }
#endif
}

void LIONode::save_loop_results() {
#if LIO_ENABLE_LOOP_CLOSURE
    if (!loop_backend_) return;
    try {
        const auto map_path = map_output_path("loop_map.pcd");
        loop_backend_->save(current_tempdir_log, map_path);
        LOG_INFO(Core, "loop map output: " << map_path << "; trajectory/constraints: " << current_tempdir_log);
    } catch (const std::exception& error) {
        LOG_ERROR(Core, "failed to save loop results: " << error.what());
    }
#endif
}
