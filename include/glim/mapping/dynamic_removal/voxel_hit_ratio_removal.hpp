#pragma once

#include <glim/mapping/dynamic_removal/dynamic_removal_base.hpp>

namespace glim {

class Config;

/**
 * @brief Remove points in voxels that are observed in only a fraction of contributing keyframes.
 */
class VoxelHitRatioRemoval : public DynamicRemovalBase {
public:
  explicit VoxelHitRatioRemoval(const Config& config);
  ~VoxelHitRatioRemoval() override;

  gtsam_points::PointCloud::Ptr filter(
    const gtsam_points::PointCloud::ConstPtr& merged_origin,
    const std::vector<gtsam_points::PointCloud::ConstPtr>& keyframes,
    const std::vector<Eigen::Isometry3d>& poses_origin_keyframe) const override;

private:
  double voxel_resolution;
  double min_hit_ratio;
  int min_total_frames;
  std::string count_mode;
};

}  // namespace glim
