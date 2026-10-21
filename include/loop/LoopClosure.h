#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>
#include <vector>

namespace elevator_lio {
struct LoopClosureConfig {
    bool enabled = false;
    bool scan_context = true;
    bool publish_map = false;
    double frequency = 2.0;
    double keyframe_distance = 0.3;
    double keyframe_angle = 0.1;
    double keyframe_period = 0.5;
    double search_radius = 15.0;
    double min_time = 30.0;
    int exclude_recent = 30;
    double max_height_difference = 1.0;
    int history_frames = 10;
    double voxel_size = 0.3; // ICP / Scan Context point-cloud resolution
    double map_voxel_size = 0.05; // retained keyframes and online map; PCD keeps frame overlap
    double fitness_threshold = 0.3;
    double max_correspondence_distance = 2.0;
    double min_overlap = 0.5;
    int min_source_points = 300;
    int min_target_points = 1000;
    double sc_distance_threshold = 0.2;
    void validate() const;
    static LoopClosureConfig fromYaml(const std::string& runtime_yaml);
};

using LoopCloud = pcl::PointCloud<pcl::PointXYZI>;
struct LoopPose {
    double stamp = 0.0;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity(); // map_T_lidar
};
struct LoopSnapshot {
    Eigen::Matrix4d map_T_world = Eigen::Matrix4d::Identity();
    std::vector<LoopPose> poses;
    LoopCloud::ConstPtr map;
    double map_stamp = 0.0;
    std::size_t loops = 0;
    std::size_t dropped_frames = 0;
    std::string error;
};

// Only the worker accesses Scan Context, ICP, keyframes and iSAM2.
// Frontend states and the local ikd-tree remain in the continuous world frame.
class LoopClosure {
public:
    explicit LoopClosure(const LoopClosureConfig& config);
    ~LoopClosure();
    LoopClosure(const LoopClosure&) = delete;
    LoopClosure& operator=(const LoopClosure&) = delete;
    void enqueue(double stamp, const Eigen::Matrix4d& world_T_lidar,
                 const LoopCloud::ConstPtr& cloud, bool in_elevator);
    LoopSnapshot snapshot() const;
    Eigen::Matrix4d correction() const;
    void requestMap();
    void stop(); // drain, final detection, assemble dense PCD map, join (idempotent)
    // Trajectory/constraints remain in directory; map_file can select a separate PCD output.
    void save(const std::string& directory, const std::string& map_file = ""); // call after stop
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace elevator_lio
