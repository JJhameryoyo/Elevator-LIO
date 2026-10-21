#include "loop/LoopClosure.h"
#include "loop/scancontext/Scancontext.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <thread>

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
        {
            LoopClosure sparse_backend(config);
            LoopCloud::Ptr sparse(new LoopCloud);
            for (size_t i = 0; i < 50; ++i) sparse->push_back(cloud->points[i]);
            sparse_backend.enqueue(1.0, Eigen::Matrix4d::Identity(), sparse, false);
            Eigen::Matrix4d drift = Eigen::Matrix4d::Identity(); drift(0, 3) = 0.2;
            sparse_backend.enqueue(61.0, drift, sparse, false);
            sparse_backend.stop();
            const auto sparse_result = sparse_backend.snapshot();
            require(sparse_result.error.empty() && sparse_result.poses.size() == 2 &&
                    sparse_result.map && !sparse_result.map->empty(),
                    "sparse scans must still produce a map below the ICP source threshold");
            require(sparse_result.loops == 0, "sparse scans must not bypass the ICP source threshold");
        }
        {
            auto dense_config = testConfig();
            dense_config.voxel_size = 0.3;
            dense_config.map_voxel_size = 0.05;
            LoopCloud::Ptr fine_scan(new LoopCloud);
            for (int x = 0; x < 40; ++x) {
                for (int y = 0; y < 40; ++y) {
                    pcl::PointXYZI point;
                    point.x = x * 0.025f;
                    point.y = y * 0.025f;
                    point.z = 1.0f;
                    point.intensity = 1.0f;
                    fine_scan->push_back(point);
                }
            }
            pcl::VoxelGrid<pcl::PointXYZI> coarse_filter;
            coarse_filter.setLeafSize(0.3f, 0.3f, 0.3f);
            coarse_filter.setInputCloud(fine_scan);
            LoopCloud coarse;
            coarse_filter.filter(coarse);
            require(coarse.size() < size_t(dense_config.min_source_points),
                    "density fixture must be too sparse for ICP");
            LoopClosure dense_backend(dense_config);
            dense_backend.enqueue(1.0, Eigen::Matrix4d::Identity(), fine_scan, false);
            Eigen::Matrix4d drift = Eigen::Matrix4d::Identity(); drift(0, 3) = 0.2;
            dense_backend.enqueue(61.0, drift, fine_scan, false);
            dense_backend.stop();
            const auto dense_map = dense_backend.snapshot();
            require(dense_map.error.empty() && dense_map.map &&
                    dense_map.map->size() > coarse.size() * 5,
                    "map must preserve finer detail than the ICP point cloud");
            require(dense_map.loops == 0,
                    "dense stored scans must not bypass the coarse ICP source-count check");
        }
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
        config.scan_context = false;
        LoopClosure saved(config);
        saved.enqueue(1.0, Eigen::Matrix4d::Identity(), cloud, false);
        // Identical poses/clouds isolate overlap retention from pose drift.
        saved.enqueue(2.0, Eigen::Matrix4d::Identity(), cloud, false);
        saved.requestMap();
        LoopSnapshot online;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        do {
            online = saved.snapshot();
            if (!online.error.empty() || (online.map && online.map_stamp == 2.0)) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while (std::chrono::steady_clock::now() < deadline);
        require(online.error.empty() && online.map && online.map_stamp == 2.0 &&
                online.poses.size() == 2, "online map must be available before saving");
        pcl::VoxelGrid<pcl::PointXYZI> key_filter;
        key_filter.setLeafSize(config.map_voxel_size, config.map_voxel_size, config.map_voxel_size);
        key_filter.setInputCloud(cloud);
        LoopCloud key_cloud;
        key_filter.filter(key_cloud);
        require(online.map->size() <= key_cloud.size(), "online map must still remove frame overlap");
        const auto online_points = online.map->size();
        const auto directory = std::filesystem::temp_directory_path() /
            ("elevator-loop-test-" + std::to_string(std::random_device{}()));
        saved.save(directory.string());
        saved.stop(); // preserve the dense final cache on repeated shutdown
        const auto final_map = saved.snapshot();
        require(final_map.error.empty() && final_map.map &&
                final_map.map->size() == 2 * key_cloud.size(),
                "shutdown must replace the compact cache and retain every keyframe sample");
        require(online.map->size() == online_points, "published snapshots must remain immutable");
        require(final_map.loops == online.loops &&
                (final_map.poses.back().pose - online.poses.back().pose).norm() < 1e-8,
                "dense export must not change estimation or loop results");
        LoopCloud exported;
        require(pcl::io::loadPCDFile<pcl::PointXYZI>((directory / "loop_map.pcd").string(), exported) == 0 &&
                exported.size() == 2 * key_cloud.size() && exported.size() > online_points,
                "saved PCD must retain overlap rather than the compact visualization map");
        for (size_t i = 0; i < key_cloud.size(); ++i) {
            require((exported[i].getVector3fMap() - key_cloud[i].getVector3fMap()).norm() < 1e-5f &&
                    (exported[i + key_cloud.size()].getVector3fMap() - key_cloud[i].getVector3fMap()).norm() < 1e-5f,
                    "export must preserve each transformed keyframe's geometry");
        }
        require(std::filesystem::file_size(directory / "loop_map.pcd") > 0, "PCD save");
          const auto custom_map = directory / "maps" / "custom_loop_map.pcd";
          saved.save(directory.string(), custom_map.string());
          LoopCloud custom_exported;
          require(pcl::io::loadPCDFile<pcl::PointXYZI>(custom_map.string(), custom_exported) == 0 &&
                  custom_exported.size() == exported.size(),
                  "custom map directory must be created and preserve exported points");
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
