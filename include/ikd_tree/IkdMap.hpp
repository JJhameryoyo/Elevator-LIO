/*
 * Copyright (c) 2026 xiaofan
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Project-level ikd-tree map wrapper for elevator-lio.
 * The local map update workflow follows the FAST-LIO / ikd-Tree design.
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>

#include "ikd_Tree.h"

template <typename PointType>
class IkdMap final {
public:
    using PointCloudConstPtr = typename pcl::PointCloud<PointType>::ConstPtr;
    using KDTreeType = KD_TREE<PointType>;
    using KDPointVector = typename KDTreeType::PointVector;

    struct NeighborCacheEntry {
        KDPointVector points;
        std::vector<float> distance_sq;
    };

    struct Config {
        int knn = 5;
        double default_map_resolution = 0.5;
        double voxel_half_extent_ratio = 0.5;
        double local_map_cube_len = 1500.0;
        double local_map_detect_range = 200.0;
        double local_map_move_threshold = 1.5;
        double local_map_move_scale = 0.9;
        double cache_quantization_scale = 100.0;
        int cache_key_axis_bits = 21;
        std::string backend = "ikdtree";
        double ivox_grid_resolution = 2.0;
        int ivox_nearby_type = 6;
        double ivox_max_search_range = 5.0;
        double ivox_insert_voxel_size = 0.4;
    };

    explicit IkdMap(const Config& config = Config(), KDTreeType* external_tree = nullptr)
            : config_(config),
              tree_ptr_(external_tree != nullptr ? external_tree : &owned_tree_),
              use_ivox_(config_.backend == "ivox"),
              map_resolution_(config_.default_map_resolution),
              cube_len_(config_.local_map_cube_len),
              det_range_(config_.local_map_detect_range),
              mov_threshold_(config_.local_map_move_threshold) {}

    ~IkdMap() = default;

    void init(double map_resolution) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (map_resolution > 0.0) {
            map_resolution_ = map_resolution;
            tree_ptr_->set_downsample_param(static_cast<float>(map_resolution));
        }
        initialized_ = true;
    }

    void add(const PointCloudConstPtr& cloud) {
        if (!cloud || cloud->points.empty()) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) {
            return;
        }

        if (use_ivox_) {
            addIvoxLocked(*cloud);
            return;
        }

        KDPointVector points;
        points.reserve(cloud->points.size());
        for (const auto& p : cloud->points) {
            points.push_back(p);
        }

        // 首次插入需要 Build，后续帧才使用 Add_Points 增量更新
        if (tree_ptr_->Root_Node == nullptr) {
            tree_ptr_->Build(points);
            return;
        }

        KDPointVector points_need_downsample;
        KDPointVector points_no_need_downsample;
        points_need_downsample.reserve(points.size());
        points_no_need_downsample.reserve(points.size());

        const int kNumMatchPoints = std::max(1, config_.knn);
        const double filter_size_map_min =
                (map_resolution_ > 0.0) ? map_resolution_ : config_.default_map_resolution;
        const double half_filter = config_.voxel_half_extent_ratio * filter_size_map_min;
        constexpr float kMinInsertDistanceSq = 0.02f * 0.02f; // 2cm

        for (const auto& world_point : points) {
            const NeighborCacheEntry near = nearest(world_point, kNumMatchPoints);
            const auto& points_near = near.points;
            const auto& distance_sq = near.distance_sq;

            if (!points_near.empty()) {
                // 如果当前点离最近邻过近（<=2cm），则不再添加该点
                if (!distance_sq.empty() && distance_sq[0] <= kMinInsertDistanceSq) {
                    continue;
                }

                bool need_add = true;
                const PointType mid_point = voxelCenter(world_point, filter_size_map_min, half_filter);
                const float dist = dist2(world_point, mid_point);
                if (std::fabs(points_near[0].x - mid_point.x) > half_filter &&
                    std::fabs(points_near[0].y - mid_point.y) > half_filter &&
                    std::fabs(points_near[0].z - mid_point.z) > half_filter) {
                    points_no_need_downsample.push_back(world_point);
                    continue;
                }

                // 判断当前点的 NUM_MATCH_POINTS 个邻近点与包围盒中心的关系
                for (int idx = 0; idx < kNumMatchPoints; ++idx) {
                    if (static_cast<int>(points_near.size()) < kNumMatchPoints) {
                        break; // 邻近点不足时直接保留 need_add=true
                    }
                    if (dist2(points_near[static_cast<size_t>(idx)], mid_point) < dist) {
                        need_add = false;
                        break;
                    }
                }
                if (need_add) {
                    points_need_downsample.push_back(world_point);
                }
            } else {
                // 如果周围没有点，则加入到需要降采样集合
                points_need_downsample.push_back(world_point);
            }
        }

        tree_ptr_->Add_Points(points_need_downsample, true);
        tree_ptr_->Add_Points(points_no_need_downsample, false);
        nearest_cache_.clear(); // 地图已更新，旧近邻结果失效
    }

    // Point-LIO map insertion path: reuse the neighbors obtained by the
    // LiDAR measurement model instead of querying the (post-update) point
    // again at frame commit time.
    void addWithMeasurementNeighbors(
            const PointCloudConstPtr& cloud,
            const std::vector<KDPointVector>& measurement_neighbors) {
        if (!cloud || cloud->empty()) return;
        if (!use_ivox_ || measurement_neighbors.size() != cloud->size()) {
            add(cloud);
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) return;
        if (!ivox_has_initial_map_) {
            KDPointVector initial_points;
            initial_points.reserve(cloud->size());
            for (const auto& point : cloud->points) initial_points.push_back(point);
            insertIvoxPoints(initial_points);
            ivox_has_initial_map_ = true;
            return;
        }

        KDPointVector points_to_add;
        points_to_add.reserve(cloud->size());
        const double voxel = config_.ivox_insert_voxel_size;
        const double half = 0.5 * voxel;
        for (std::size_t i = 0; i < cloud->size(); ++i) {
            const PointType& point = cloud->points[i];
            const PointType center = voxelCenter(point, voxel, half);
            bool need_add = true;
            for (const auto& near : measurement_neighbors[i]) {
                if (std::abs(near.x - center.x) < half &&
                    std::abs(near.y - center.y) < half &&
                    std::abs(near.z - center.z) < half) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) points_to_add.push_back(point);
        }
        insertIvoxPoints(points_to_add);
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (use_ivox_) {
            ivox_grids_.clear();
            ivox_point_count_ = 0;
            ivox_has_initial_map_ = false;
            return;
        }
        KDPointVector empty_points;
        tree_ptr_->Build(empty_points);
        nearest_cache_.clear();
        localmap_initialized_ = false;
        local_map_points_ = BoxPointType{};
    }

    void removeFar(const Eigen::Vector3d& current_pos) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_) {
            return;
        }
        // Point-LIO's IVox retains grids through its capacity/LRU policy rather
        // than the ikd-tree moving-cube deletion scheme.  The regression bags
        // are far below that capacity, so no spatial deletion is needed here.
        if (use_ivox_) return;

        if (!localmap_initialized_) {
            // 以 world 系当前位置为中心，初始化局部地图包围盒（长宽高均为 cube_len_）
            for (int i = 0; i < 3; ++i) {
                local_map_points_.vertex_min[i] = static_cast<float>(current_pos(i) - cube_len_ * 0.5);
                local_map_points_.vertex_max[i] = static_cast<float>(current_pos(i) + cube_len_ * 0.5);
            }
            localmap_initialized_ = true;
            return;
        }

        double dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; ++i) {
            dist_to_map_edge[i][0] = std::fabs(current_pos(i) - local_map_points_.vertex_min[i]);
            dist_to_map_edge[i][1] = std::fabs(current_pos(i) - local_map_points_.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= mov_threshold_ * det_range_ ||
                dist_to_map_edge[i][1] <= mov_threshold_ * det_range_) {
                need_move = true;
            }
        }
        if (!need_move) {
            return;
        }

        BoxPointType new_local_map_points = local_map_points_;
        std::vector<BoxPointType> boxes_to_remove;
        boxes_to_remove.reserve(3);

        const double mov_dist = std::max(
                config_.local_map_move_scale * (0.5 * cube_len_ - mov_threshold_ * det_range_),
                det_range_ * (mov_threshold_ - 1.0));

        for (int i = 0; i < 3; ++i) {
            BoxPointType tmp_box = local_map_points_;
            if (dist_to_map_edge[i][0] <= mov_threshold_ * det_range_) {
                // 如果与坐标较小的那个面较近，包围框整体向负方向移动
                new_local_map_points.vertex_min[i] -= static_cast<float>(mov_dist);
                new_local_map_points.vertex_max[i] -= static_cast<float>(mov_dist);
                tmp_box.vertex_min[i] = local_map_points_.vertex_max[i] - static_cast<float>(mov_dist);
                boxes_to_remove.push_back(tmp_box);
            } else if (dist_to_map_edge[i][1] <= mov_threshold_ * det_range_) {
                // 如果与坐标较大的那个面较近，包围框整体向正方向移动
                new_local_map_points.vertex_min[i] += static_cast<float>(mov_dist);
                new_local_map_points.vertex_max[i] += static_cast<float>(mov_dist);
                tmp_box.vertex_max[i] = local_map_points_.vertex_min[i] + static_cast<float>(mov_dist);
                boxes_to_remove.push_back(tmp_box);
            }
        }

        local_map_points_ = new_local_map_points;
        if (!boxes_to_remove.empty()) {
            tree_ptr_->Delete_Point_Boxes(boxes_to_remove);
            nearest_cache_.clear(); // 地图已更新，旧近邻结果失效
        }
    }

    bool searchKnn(const PointType& point, int k, KDPointVector& points,
                   std::vector<float>& distance_sq) const {
        if (use_ivox_) return searchIvox(point, k, points, distance_sq);
        tree_ptr_->Nearest_Search(point, k, points, distance_sq);
        return static_cast<int>(points.size()) >= k;
    }

    bool empty() const {
        return use_ivox_ ? ivox_point_count_ == 0 : tree_ptr_->Root_Node == nullptr;
    }

    bool usesIvox() const { return use_ivox_; }

private:
    struct GridKey {
        int x = 0;
        int y = 0;
        int z = 0;
        bool operator==(const GridKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct GridKeyHash {
        std::size_t operator()(const GridKey& key) const {
            std::size_t seed = std::hash<int>{}(key.x);
            seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
            seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
            return seed;
        }
    };

    GridKey ivoxKey(const PointType& point) const {
        const double inv = 1.0 / std::max(1e-6, config_.ivox_grid_resolution);
        return {static_cast<int>(std::floor(point.x * inv)),
                static_cast<int>(std::floor(point.y * inv)),
                static_cast<int>(std::floor(point.z * inv))};
    }

    bool includeNearbyOffset(int dx, int dy, int dz) const {
        const int nonzero = (dx != 0) + (dy != 0) + (dz != 0);
        if (config_.ivox_nearby_type == 6) return nonzero <= 1;
        if (config_.ivox_nearby_type == 18) return nonzero <= 2;
        return true;
    }

    bool searchIvox(const PointType& point, int k, KDPointVector& points,
                    std::vector<float>& distance_sq) const {
        points.clear();
        distance_sq.clear();
        if (k <= 0 || ivox_grids_.empty()) return false;

        struct Candidate {
            float distance_sq;
            PointType point;
        };
        std::vector<Candidate> candidates;
        const GridKey center = ivoxKey(point);
        const float max_distance_sq = static_cast<float>(
                config_.ivox_max_search_range * config_.ivox_max_search_range);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (!includeNearbyOffset(dx, dy, dz)) continue;
                    const auto it = ivox_grids_.find({center.x + dx, center.y + dy, center.z + dz});
                    if (it == ivox_grids_.end()) continue;
                    for (const auto& candidate_point : it->second) {
                        const float d2 = dist2(point, candidate_point);
                        if (d2 < max_distance_sq) candidates.push_back({d2, candidate_point});
                    }
                }
            }
        }
        if (candidates.empty()) return false;

        const size_t keep = std::min(static_cast<size_t>(k), candidates.size());
        if (candidates.size() > keep) {
            std::nth_element(candidates.begin(), candidates.begin() + static_cast<std::ptrdiff_t>(keep),
                             candidates.end(),
                             [](const Candidate& a, const Candidate& b) {
                                 return a.distance_sq < b.distance_sq;
                             });
            candidates.resize(keep);
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& a, const Candidate& b) {
                      return a.distance_sq < b.distance_sq;
                  });
        points.reserve(keep);
        distance_sq.reserve(keep);
        for (const auto& candidate : candidates) {
            points.push_back(candidate.point);
            distance_sq.push_back(candidate.distance_sq);
        }
        return static_cast<int>(points.size()) >= k;
    }

    void insertIvoxPoints(const KDPointVector& points) {
        for (const auto& point : points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
            ivox_grids_[ivoxKey(point)].push_back(point);
            ++ivox_point_count_;
        }
    }

    void addIvoxLocked(const pcl::PointCloud<PointType>& cloud) {
        KDPointVector input;
        input.reserve(cloud.size());
        for (const auto& point : cloud.points) input.push_back(point);
        if (!ivox_has_initial_map_) {
            insertIvoxPoints(input);
            ivox_has_initial_map_ = true;
            return;
        }

        KDPointVector points_to_add;
        points_to_add.reserve(input.size());
        const double voxel = config_.ivox_insert_voxel_size;
        const double half = 0.5 * voxel;
        for (const auto& point : input) {
            KDPointVector nearest_points;
            std::vector<float> nearest_distance_sq;
            searchIvox(point, std::max(1, config_.knn), nearest_points, nearest_distance_sq);
            const PointType center = voxelCenter(point, voxel, half);
            bool need_add = true;
            for (const auto& near : nearest_points) {
                if (std::abs(near.x - center.x) < half &&
                    std::abs(near.y - center.y) < half &&
                    std::abs(near.z - center.z) < half) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) points_to_add.push_back(point);
        }
        insertIvoxPoints(points_to_add);
    }

    NeighborCacheEntry nearest(const PointType& point, int k) const {
        const std::uint64_t cache_key = key(point);
        const auto it = nearest_cache_.find(cache_key);
        if (it != nearest_cache_.end() && static_cast<int>(it->second.points.size()) >= k) {
            return it->second;
        }

        NeighborCacheEntry out;
        tree_ptr_->Nearest_Search(point, k, out.points, out.distance_sq);
        nearest_cache_[cache_key] = out;
        return out;
    }

    static PointType voxelCenter(const PointType& point, double size, double half) {
        PointType center;
        center.x = static_cast<float>(std::floor(point.x / size) * size + half);
        center.y = static_cast<float>(std::floor(point.y / size) * size + half);
        center.z = static_cast<float>(std::floor(point.z / size) * size + half);
        return center;
    }

    std::uint64_t key(const PointType& p) const {
        const int axis_bits = std::max(1, std::min(21, config_.cache_key_axis_bits));
        const std::uint64_t axis_mask = (1ULL << axis_bits) - 1ULL;
        const int shift_y = axis_bits;
        const int shift_x = axis_bits * 2;

        const double q = (config_.cache_quantization_scale > 0.0) ? config_.cache_quantization_scale : 100.0;
        const int64_t xi = static_cast<int64_t>(std::llround(static_cast<double>(p.x) * q));
        const int64_t yi = static_cast<int64_t>(std::llround(static_cast<double>(p.y) * q));
        const int64_t zi = static_cast<int64_t>(std::llround(static_cast<double>(p.z) * q));
        const std::uint64_t ux = static_cast<std::uint64_t>(xi) & axis_mask;
        const std::uint64_t uy = static_cast<std::uint64_t>(yi) & axis_mask;
        const std::uint64_t uz = static_cast<std::uint64_t>(zi) & axis_mask;
        return (ux << shift_x) | (uy << shift_y) | uz;
    }

    static float dist2(const PointType& p1, const PointType& p2) {
        return (p1.x - p2.x) * (p1.x - p2.x) +
               (p1.y - p2.y) * (p1.y - p2.y) +
               (p1.z - p2.z) * (p1.z - p2.z);
    }

    mutable std::mutex mutex_;
    Config config_;
    mutable KDTreeType owned_tree_;
    KDTreeType* tree_ptr_ = nullptr;
    bool use_ivox_ = false;
    bool initialized_ = false;
    double map_resolution_;
    bool localmap_initialized_ = false;
    BoxPointType local_map_points_{};
    double cube_len_;
    double det_range_;
    double mov_threshold_;
    mutable std::unordered_map<std::uint64_t, NeighborCacheEntry> nearest_cache_;
    std::unordered_map<GridKey, KDPointVector, GridKeyHash> ivox_grids_;
    size_t ivox_point_count_ = 0;
    bool ivox_has_initial_map_ = false;
};
