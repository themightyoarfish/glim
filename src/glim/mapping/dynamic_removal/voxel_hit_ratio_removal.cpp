#include <glim/mapping/dynamic_removal/voxel_hit_ratio_removal.hpp>

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <spdlog/spdlog.h>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/util/fast_floor.hpp>
#include <gtsam_points/util/vector3i_hash.hpp>
#include <glim/util/config.hpp>

namespace glim {

namespace {

Eigen::Vector3i voxel_coord(const Eigen::Vector4d& point, const double inv_resolution) {
  return gtsam_points::fast_floor(point * inv_resolution).head<3>();
}

}  // namespace

VoxelHitRatioRemoval::VoxelHitRatioRemoval(const Config& config) {
  voxel_resolution = config.param<double>("sub_mapping", "dynamic_removal_voxel_resolution", 0.3);
  min_hit_ratio = config.param<double>("sub_mapping", "dynamic_removal_min_hit_ratio", 0.4);
  min_total_frames = config.param<int>("sub_mapping", "dynamic_removal_min_total_frames", 3);
  count_mode = config.param<std::string>("sub_mapping", "dynamic_removal_count_mode", "OBSERVED");
  std::transform(count_mode.begin(), count_mode.end(), count_mode.begin(), ::toupper);
}

VoxelHitRatioRemoval::~VoxelHitRatioRemoval() {}

gtsam_points::PointCloud::Ptr VoxelHitRatioRemoval::filter(
  const gtsam_points::PointCloud::ConstPtr& merged_origin,
  const std::vector<gtsam_points::PointCloud::ConstPtr>& keyframes,
  const std::vector<Eigen::Isometry3d>& poses_origin_keyframe) const {
  if (!merged_origin || merged_origin->size() == 0) {
    return nullptr;
  }

  if (keyframes.empty() || keyframes.size() != poses_origin_keyframe.size()) {
    spdlog::warn("voxel hit ratio removal: invalid keyframe inputs (keyframes={} poses={})", keyframes.size(), poses_origin_keyframe.size());
    return nullptr;
  }

  const int total_frames = static_cast<int>(keyframes.size());
  if (total_frames < min_total_frames) {
    spdlog::debug("voxel hit ratio removal: skipped (frames={} < min_total_frames={})", total_frames, min_total_frames);
    return nullptr;
  }

  const double inv_resolution = 1.0 / voxel_resolution;
  const bool count_observed = count_mode != "OCCUPIED";

  std::unordered_map<Eigen::Vector3i, int, gtsam_points::Vector3iHash> hit_counts;
  hit_counts.reserve(merged_origin->size());

  for (int frame_idx = 0; frame_idx < total_frames; frame_idx++) {
    const auto& keyframe = keyframes[frame_idx];
    const auto& T_origin_keyframe = poses_origin_keyframe[frame_idx];
    if (!keyframe || keyframe->size() == 0) {
      continue;
    }

    if (count_observed) {
      std::unordered_set<Eigen::Vector3i, gtsam_points::Vector3iHash> observed_voxels;
      observed_voxels.reserve(keyframe->size());

      for (int i = 0; i < keyframe->size(); i++) {
        const Eigen::Vector4d pt_origin = T_origin_keyframe * keyframe->points[i];
        observed_voxels.insert(voxel_coord(pt_origin, inv_resolution));
      }

      for (const auto& coord : observed_voxels) {
        hit_counts[coord]++;
      }
    } else {
      for (int i = 0; i < keyframe->size(); i++) {
        const Eigen::Vector4d pt_origin = T_origin_keyframe * keyframe->points[i];
        hit_counts[voxel_coord(pt_origin, inv_resolution)]++;
      }
    }
  }

  std::vector<int> keep_indices;
  keep_indices.reserve(merged_origin->size());

  for (int i = 0; i < merged_origin->size(); i++) {
    const Eigen::Vector3i coord = voxel_coord(merged_origin->points[i], inv_resolution);
    const auto found = hit_counts.find(coord);
    const int hit_count = found != hit_counts.end() ? found->second : 0;

    const double hit_ratio = static_cast<double>(hit_count) / static_cast<double>(total_frames);
    if (hit_ratio >= min_hit_ratio) {
      keep_indices.push_back(i);
    }
  }

  spdlog::debug(
    "voxel hit ratio removal: kept {}/{} points (min_hit_ratio={} frames={})",
    keep_indices.size(),
    merged_origin->size(),
    min_hit_ratio,
    total_frames);

  if (keep_indices.empty()) {
    spdlog::warn("voxel hit ratio removal: all points removed; keeping original submap cloud");
    return nullptr;
  }

  if (keep_indices.size() == static_cast<size_t>(merged_origin->size())) {
    return nullptr;
  }

  return gtsam_points::sample(merged_origin, keep_indices);
}

}  // namespace glim
