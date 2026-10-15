#include "loop/LoopClosure.h"
#include "loop/scancontext/Scancontext.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace elevator_lio {
void LoopClosureConfig::validate() const {
    const double positive[] = {frequency, keyframe_distance, keyframe_angle, keyframe_period,
        search_radius, min_time, max_height_difference, voxel_size, map_voxel_size, fitness_threshold,
        max_correspondence_distance, min_overlap, sc_distance_threshold};
    for (double value : positive)
        if (!std::isfinite(value) || value <= 0.0)
            throw std::invalid_argument("loop_closure parameters must be finite and positive");
    if (exclude_recent < 1 || history_frames < 0 || min_source_points < 3 ||
        min_target_points < 3 || min_overlap > 1.0 || sc_distance_threshold > 1.0)
        throw std::invalid_argument("invalid loop_closure count/overlap/Scan Context threshold");
}

LoopClosureConfig LoopClosureConfig::fromYaml(const std::string& file) {
    LoopClosureConfig config;
    const YAML::Node root = YAML::LoadFile(file);
    const YAML::Node node = root["loop_closure"];
    if (node) {
#define LIO_LOOP_READ(name) if (node[#name]) config.name = node[#name].as<decltype(config.name)>();
        LIO_LOOP_READ(enabled)
        LIO_LOOP_READ(scan_context)
        LIO_LOOP_READ(publish_map)
        LIO_LOOP_READ(frequency)
        LIO_LOOP_READ(keyframe_distance)
        LIO_LOOP_READ(keyframe_angle)
        LIO_LOOP_READ(keyframe_period)
        LIO_LOOP_READ(search_radius)
        LIO_LOOP_READ(min_time)
        LIO_LOOP_READ(exclude_recent)
        LIO_LOOP_READ(max_height_difference)
        LIO_LOOP_READ(history_frames)
        LIO_LOOP_READ(voxel_size)
        LIO_LOOP_READ(map_voxel_size)
        LIO_LOOP_READ(fitness_threshold)
        LIO_LOOP_READ(max_correspondence_distance)
        LIO_LOOP_READ(min_overlap)
        LIO_LOOP_READ(min_source_points)
        LIO_LOOP_READ(min_target_points)
        LIO_LOOP_READ(sc_distance_threshold)
#undef LIO_LOOP_READ
    }
    config.validate();
    return config;
}

namespace {
gtsam::Pose3 toPose(const Eigen::Matrix4d& matrix) {
    return gtsam::Pose3(gtsam::Rot3(matrix.block<3, 3>(0, 0)),
                       gtsam::Point3(matrix.block<3, 1>(0, 3)));
}
LoopCloud::Ptr downsample(const LoopCloud::ConstPtr& input, double leaf) {
    LoopCloud::Ptr result(new LoopCloud);
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setLeafSize(leaf, leaf, leaf);
    filter.setInputCloud(input);
    filter.filter(*result);
    return result;
}
} // namespace

struct LoopClosure::Impl {
    struct Frame {
        double stamp;
        Eigen::Matrix4d raw;
        LoopCloud::ConstPtr cloud;
    };
    struct Keyframe {
        Frame frame;
        Eigen::Matrix4d optimized;
    };
    LoopClosureConfig config;
    mutable std::mutex mutex;
    std::condition_variable wake;
    std::deque<Frame> pending;
    bool stopping = false;
    bool map_requested = false;
    LoopSnapshot output;
    std::thread worker;
    std::vector<Keyframe> keys;
    SCManager sc;
    gtsam::ISAM2 isam;
    std::vector<std::pair<size_t, size_t>> edges;
    size_t checked_count = 0;
    size_t mapped_key_count = 0;
    size_t mapped_edge_count = 0;
    bool mapped_for_save = false;

    explicit Impl(const LoopClosureConfig& cfg) : config(cfg) {
        config.validate();
        sc.SC_DIST_THRES = config.sc_distance_threshold;
        if (config.enabled) worker = std::thread([this]() { run(); });
    }

    void updateOutput(const gtsam::Values& estimate) {
        LoopSnapshot next;
        next.poses.reserve(keys.size());
        for (size_t i = 0; i < keys.size(); ++i) {
            keys[i].optimized = estimate.at<gtsam::Pose3>(i).matrix();
            next.poses.push_back({keys[i].frame.stamp, keys[i].optimized});
        }
        if (!keys.empty()) next.map_T_world = keys.back().optimized * keys.back().frame.raw.inverse();
        std::lock_guard<std::mutex> lock(mutex);
        next.map = output.map;
        next.map_stamp = output.map_stamp;
        next.loops = edges.size();
        next.dropped_frames = output.dropped_frames;
        output = std::move(next);
    }

    void addFrame(const Frame& frame) {
        if (!keys.empty()) {
            if (frame.stamp <= keys.back().frame.stamp) return;
            const Eigen::Matrix4d delta = keys.back().frame.raw.inverse() * frame.raw;
            const double distance = delta.block<3, 1>(0, 3).norm();
            const double angle = Eigen::AngleAxisd(delta.block<3, 3>(0, 0)).angle();
            if (distance < config.keyframe_distance && angle < config.keyframe_angle &&
                frame.stamp - keys.back().frame.stamp < config.keyframe_period) return;
        }
        Frame key = frame;
        key.cloud = downsample(frame.cloud, config.map_voxel_size);
        // Sparse scans can still contribute to the odometry-backed map. The
        // ICP source threshold controls loop verification, not map retention.
        if (key.cloud->size() < 3) return;
        const size_t index = keys.size();
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial;
        Eigen::Matrix4d seed = frame.raw;
        if (index == 0) {
            auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-4);
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, toPose(seed), noise));
        } else {
            const Eigen::Matrix4d relative = keys.back().frame.raw.inverse() * frame.raw;
            seed = keys.back().optimized * relative;
            auto noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished());
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(index - 1, index, toPose(relative), noise));
        }
        initial.insert(index, toPose(seed));
        isam.update(graph, initial);
        isam.update();
        keys.push_back({key, seed});
        // Indices must stay identical to the pose graph, including when SC is disabled.
        if (config.scan_context) {
            auto descriptor_cloud = downsample(key.cloud, config.voxel_size);
            sc.makeAndSaveScancontextAndKeys(*descriptor_cloud);
        }
        updateOutput(isam.calculateEstimate());
    }

    bool eligible(size_t previous, size_t current) const {
        return current - previous >= size_t(config.exclude_recent) &&
            keys[current].frame.stamp - keys[previous].frame.stamp >= config.min_time &&
            std::abs(keys[current].frame.raw(2, 3) - keys[previous].frame.raw(2, 3)) <=
                config.max_height_difference;
    }

    bool verify(size_t previous, size_t current, bool use_yaw, double yaw) {
        const auto source = downsample(keys[current].frame.cloud, config.voxel_size);
        if (source->size() < size_t(config.min_source_points)) return false;
        // Target submap is in the historical LiDAR frame, source in current LiDAR frame.
        // ICP returns previous_T_current; BetweenFactor(current,previous) needs its inverse.
        LoopCloud::Ptr target(new LoopCloud);
        const size_t first = previous > size_t(config.history_frames) ?
            previous - config.history_frames : 0;
        const size_t last = std::min(previous + size_t(config.history_frames), current - 1);
        const Eigen::Matrix4d base_inverse = keys[previous].optimized.inverse();
        for (size_t i = first; i <= last; ++i) {
            if (!eligible(i, current)) continue;
            LoopCloud transformed;
            const Eigen::Matrix4f transform = (base_inverse * keys[i].optimized).cast<float>();
            pcl::transformPointCloud(*keys[i].frame.cloud, transformed, transform);
            *target += transformed;
        }
        target = downsample(target, config.voxel_size);
        if (target->size() < size_t(config.min_target_points)) return false;
        Eigen::Matrix4d guess = base_inverse * keys[current].optimized;
        if (use_yaw) {
            // SC shifts the historical descriptor into the current one, hence inverse yaw.
            guess.block<3, 3>(0, 0) = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        }
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setMaxCorrespondenceDistance(config.max_correspondence_distance);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
        icp.setInputSource(source);
        icp.setInputTarget(target);
        LoopCloud aligned;
        icp.align(aligned, guess.cast<float>());
        const double score = icp.getFitnessScore(
            config.max_correspondence_distance * config.max_correspondence_distance);
        const Eigen::Matrix4d relative = icp.getFinalTransformation().cast<double>();
        if (!icp.hasConverged() || !relative.allFinite() || !std::isfinite(score) ||
            score > config.fitness_threshold) return false;
        // Fitness alone can accept a tiny matching fragment. Require source coverage too.
        pcl::KdTreeFLANN<pcl::PointXYZI> tree;
        tree.setInputCloud(target);
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        size_t matches = 0;
        const double range2 = config.max_correspondence_distance * config.max_correspondence_distance;
        for (const auto& point : aligned)
            if (tree.nearestKSearch(point, 1, indices, distances) == 1 && distances[0] <= range2)
                ++matches;
        if (double(matches) / aligned.size() < config.min_overlap) return false;
        const Eigen::Matrix4d corrected = keys[previous].optimized * relative;
        if (std::abs(corrected(2, 3) - keys[previous].optimized(2, 3)) >
            config.max_height_difference) return false;
        const auto diagonal = gtsam::noiseModel::Isotropic::Variance(6, std::max(score, 1e-3));
        const auto noise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1.0), diagonal);
        gtsam::NonlinearFactorGraph graph;
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(current, previous, toPose(relative.inverse()), noise));
        isam.update(graph, gtsam::Values());
        for (int i = 0; i < 5; ++i) isam.update();
        edges.emplace_back(current, previous);
        updateOutput(isam.calculateEstimate());
        return true;
    }

    void detect() {
        if (keys.size() == checked_count || keys.empty()) return;
        checked_count = keys.size();
        const size_t current = keys.size() - 1;
        std::vector<size_t> candidates;
        size_t nearest = current;
        double best = config.search_radius;
        for (size_t i = 0; i < current; ++i) {
            if (!eligible(i, current)) continue;
            candidates.push_back(i);
            const double distance = (keys[i].optimized.block<3, 1>(0, 3) -
                                     keys[current].optimized.block<3, 1>(0, 3)).norm();
            if (distance < best) { best = distance; nearest = i; }
        }
        if (nearest != current && verify(nearest, current, false, 0.0)) return;
        if (!config.scan_context) return;
        const auto match = sc.detectLoopClosureID(candidates);
        if (match.first >= 0) verify(size_t(match.first), current, true, match.second);
    }

    void rebuildMap(bool for_save = false) {
        if (mapped_key_count == keys.size() && mapped_edge_count == edges.size() &&
            mapped_for_save == for_save) return;
        LoopCloud::Ptr map(new LoopCloud);
        if (for_save) {
            size_t point_count = 0;
            for (const auto& key : keys) point_count += key.frame.cloud->size();
            map->reserve(point_count);
        }
        for (const auto& key : keys) {
            LoopCloud transformed;
            const Eigen::Matrix4f transform = key.optimized.cast<float>();
            pcl::transformPointCloud(*key.frame.cloud, transformed, transform);
            *map += transformed;
        }
        // Keep online snapshots compact. At shutdown retain overlapping keyframe
        // samples, like MID360's cloudGlobal.pcd, without another global filter.
        if (!for_save && !map->empty()) map = downsample(map, config.map_voxel_size);
        std::lock_guard<std::mutex> lock(mutex);
        output.map = map;
        output.map_stamp = keys.empty() ? 0.0 : keys.back().frame.stamp;
        mapped_key_count = keys.size();
        mapped_edge_count = edges.size();
        mapped_for_save = for_save;
    }

    void run() {
        try {
            const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / config.frequency));
            auto next_detection = std::chrono::steady_clock::now() + period;
            while (true) {
                std::deque<Frame> frames;
                bool finish;
                bool build_map;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    wake.wait_until(lock, next_detection, [this]() {
                        return stopping || !pending.empty() || map_requested;
                    });
                    frames.swap(pending);
                    finish = stopping;
                    build_map = map_requested;
                    map_requested = false;
                }
                for (const auto& frame : frames) addFrame(frame);
                const auto now = std::chrono::steady_clock::now();
                if (finish || now >= next_detection) {
                    detect();
                    next_detection = now + period;
                }
                if (build_map || finish) rebuildMap(finish);
                if (finish) return;
            }
        } catch (const std::exception& error) {
            std::lock_guard<std::mutex> lock(mutex);
            output.error = error.what();
            stopping = true;
            pending.clear();
        }
    }
};

LoopClosure::LoopClosure(const LoopClosureConfig& config) : impl_(new Impl(config)) {}
LoopClosure::~LoopClosure() { stop(); }
void LoopClosure::enqueue(double stamp, const Eigen::Matrix4d& pose,
                          const LoopCloud::ConstPtr& cloud, bool in_elevator) {
    if (!impl_->config.enabled || in_elevator || !cloud || cloud->empty() ||
        !std::isfinite(stamp) || !pose.allFinite()) return;
    LoopCloud::Ptr copy(new LoopCloud);
    copy->reserve(cloud->size());
    for (const auto& point : *cloud)
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
            copy->push_back(point);
    if (copy->size() < 3) return;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->stopping) return;
        if (impl_->pending.size() >= 8) {
            impl_->pending.pop_front();
            ++impl_->output.dropped_frames;
        }
        impl_->pending.push_back({stamp, pose, copy});
    }
    impl_->wake.notify_one();
}
LoopSnapshot LoopClosure::snapshot() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->output;
}
Eigen::Matrix4d LoopClosure::correction() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->output.map_T_world;
}
void LoopClosure::requestMap() {
    { std::lock_guard<std::mutex> lock(impl_->mutex); impl_->map_requested = true; }
    impl_->wake.notify_one();
}
void LoopClosure::stop() {
    { std::lock_guard<std::mutex> lock(impl_->mutex); impl_->stopping = true; }
    impl_->wake.notify_one();
    if (impl_->worker.joinable()) impl_->worker.join();
}
void LoopClosure::save(const std::string& directory) {
    stop();
    const auto result = snapshot();
    if (!result.error.empty()) throw std::runtime_error("loop backend failed: " + result.error);
    if (result.poses.empty()) return;
    std::filesystem::create_directories(directory);
    std::ofstream trajectory(std::filesystem::path(directory) / "loop_trajectory.tum");
    std::ofstream constraints(std::filesystem::path(directory) / "loop_constraints.csv");
    if (!trajectory || !constraints) throw std::runtime_error("cannot save loop trajectory/constraints");
    trajectory << std::fixed << std::setprecision(9);
    for (const auto& pose : result.poses) {
        Eigen::Quaterniond q(pose.pose.block<3, 3>(0, 0));
        trajectory << pose.stamp << ' ' << pose.pose(0, 3) << ' ' << pose.pose(1, 3) << ' '
                   << pose.pose(2, 3) << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
    }
    constraints << "current,previous\n";
    for (const auto& edge : impl_->edges) constraints << edge.first << ',' << edge.second << '\n';
    trajectory.flush();
    constraints.flush();
    if (!trajectory || !constraints) throw std::runtime_error("failed writing loop trajectory/constraints");
    if (result.map && !result.map->empty() &&
        pcl::io::savePCDFileBinary((std::filesystem::path(directory) / "loop_map.pcd").string(), *result.map) != 0)
        throw std::runtime_error("cannot save loop_map.pcd");
}
} // namespace elevator_lio
