#include "loop/LoopClosure.h"
#include "loop/scancontext/Scancontext.h"
#include <pcl/common/transforms.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

using namespace elevator_lio;
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

LoopCloud::Ptr scene() {
    LoopCloud::Ptr cloud(new LoopCloud);
    std::mt19937 generator(17);
    std::uniform_real_distribution<float> xy(-8.0f, 8.0f);
    std::uniform_real_distribution<float> z(-1.0f, 3.0f);
    for (int i = 0; i < 1500; ++i) {
        pcl::PointXYZI p;
        p.x = xy(generator); p.y = xy(generator); p.z = z(generator); p.intensity = i;
        cloud->push_back(p);
    }
    return cloud;
}

LoopClosureConfig testConfig() {
    LoopClosureConfig config;
    config.enabled = true;
    config.frequency = 0.01; // tests deterministically detect once on stop
    config.keyframe_distance = 0.05;
    config.min_time = 10.0;
    config.exclude_recent = 1;
    config.history_frames = 0;
    config.voxel_size = 0.03;
    config.min_source_points = 100;
    config.min_target_points = 100;
    config.fitness_threshold = 0.05;
    config.max_height_difference = 1.0;
    return config;
}

LoopSnapshot twoFrames(LoopClosureConfig config, double z, double stamp, bool elevator) {
    LoopClosure backend(config);
    const auto cloud = scene();
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    backend.enqueue(1.0, pose, cloud, false);
    pose(0, 3) = 0.2;
    pose(2, 3) = z;
    backend.enqueue(stamp, pose, cloud, elevator);
    backend.stop();
    backend.stop(); // stop must be idempotent
    return backend.snapshot();
}

int main() {
    try {
        SCManager manager;
        require(manager.detectLoopClosureID().first == -1, "empty SC history");
        LoopCloud empty;
        const auto empty_descriptor = manager.makeScancontext(empty);
        Eigen::MatrixXd a = empty_descriptor, b = empty_descriptor;
        require(manager.distDirectSC(a, b) == 1.0, "empty descriptors cannot match");
        const auto cloud = scene();
        manager.makeAndSaveScancontextAndKeys(*cloud);
        manager.makeAndSaveScancontextAndKeys(*cloud);
        require(manager.detectLoopClosureID(std::vector<size_t>{0}).first == 0,
                "single KNN candidate must work");
        require(manager.detectLoopClosureID(std::vector<size_t>{}).first == -1,
                "excluded history must not match");
        require(manager.detectLoopClosureID(std::vector<size_t>{2}).first == -1,
                "invalid candidate must be rejected");
        LoopCloud axis_cloud;
        pcl::PointXYZI origin; origin.x = origin.y = origin.z = 0.0f;
        axis_cloud.push_back(origin);
        origin.x = std::numeric_limits<float>::quiet_NaN(); axis_cloud.push_back(origin);
        require(manager.makeScancontext(axis_cloud).allFinite(), "origin/NaN descriptor safety");

        const double pi = std::acos(-1.0);
        Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        rotation.block<3, 3>(0, 0) = Eigen::AngleAxisf(pi / 3.0, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        LoopCloud rotated;
        pcl::transformPointCloud(*cloud, rotated, rotation);
        SCManager yaw_manager;
        yaw_manager.makeAndSaveScancontextAndKeys(*cloud);
        yaw_manager.makeAndSaveScancontextAndKeys(rotated);
        const auto yaw_match = yaw_manager.detectLoopClosureID(std::vector<size_t>{0});
        require(yaw_match.first == 0 && std::abs(yaw_match.second - pi / 3.0) < 0.11,
                "SC shift direction must yield inverse current-to-history yaw");

        auto config = testConfig();
        const auto distance = twoFrames(config, 0.0, 61.0, false);
        require(distance.error.empty() && distance.loops == 1 && distance.poses.size() == 2,
                "distance ICP loop must be accepted");
        require(distance.poses.back().pose(0, 3) < 0.1999 && distance.map_T_world(0, 3) < 0.0,
                "loop measurement must reduce positive frontend drift");
        Eigen::Matrix4d raw = Eigen::Matrix4d::Identity(); raw(0, 3) = 0.2;
        require((distance.map_T_world * raw - distance.poses.back().pose).norm() < 1e-8,
                "map/world transform composition");
        require(distance.map && !distance.map->empty(), "final corrected map assembly");

        config.search_radius = 0.01; // force Scan Context fallback beyond distance radius
        const auto sc = twoFrames(config, 0.0, 61.0, false);
        require(sc.error.empty() && sc.loops == 1, "Scan Context fallback loop");
        {
            LoopClosure yaw_backend(config);
            yaw_backend.enqueue(1.0, Eigen::Matrix4d::Identity(), cloud, false);
            Eigen::Matrix4d drift = Eigen::Matrix4d::Identity(); drift(0, 3) = 0.2;
            LoopCloud::Ptr yaw_cloud(new LoopCloud(rotated));
            yaw_backend.enqueue(61.0, drift, yaw_cloud, false);
            yaw_backend.stop();
            const auto yaw_result = yaw_backend.snapshot();
            require(yaw_result.error.empty() && yaw_result.loops == 1,
                    "SC yaw initialization must align a sixty-degree revisit");
        }
        require(twoFrames(config, 3.0, 61.0, false).loops == 0, "different floor rejection");
        require(twoFrames(config, 0.0, 3.0, false).loops == 0, "recent time rejection");
        config.exclude_recent = 2;
        require(twoFrames(config, 0.0, 61.0, false).loops == 0, "recent keyframe rejection");
        config.exclude_recent = 1;
        const auto elevator = twoFrames(config, 0.0, 61.0, true);
        require(elevator.poses.size() == 1 && elevator.loops == 0, "elevator frame suppression");
        config.enabled = false;
        require(twoFrames(config, 0.0, 61.0, false).poses.empty(), "disabled backend");
        config.frequency = 0.0;
        bool rejected = false;
        try { config.validate(); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "invalid frequency rejection");
        config = testConfig();
        config.min_overlap = 1.1;
        rejected = false;
        try { config.validate(); } catch (const std::invalid_argument&) { rejected = true; }
        require(rejected, "invalid overlap rejection");

        config = testConfig();
        LoopClosure saved(config);
        saved.enqueue(1.0, Eigen::Matrix4d::Identity(), cloud, false);
        const auto directory = std::filesystem::temp_directory_path() /
            ("elevator-loop-test-" + std::to_string(std::random_device{}()));
        saved.save(directory.string());
        require(std::filesystem::file_size(directory / "loop_map.pcd") > 0, "PCD save");
        require(std::filesystem::file_size(directory / "loop_trajectory.tum") > 0, "trajectory save");
        require(std::filesystem::file_size(directory / "loop_constraints.csv") > 0, "constraint save");
        // Remove only the three files created by this test, then the empty directory.
        for (const char* name : {"loop_map.pcd", "loop_trajectory.tum", "loop_constraints.csv"})
            std::filesystem::remove(directory / name);
        std::filesystem::remove(directory);
        std::cout << "loop backend regression passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
