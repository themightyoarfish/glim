#pragma once

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <gtsam_points/types/point_cloud.hpp>

namespace glim {

class Config;

/**
 * @brief Base class for dynamic (moving) object removal strategies applied to submap point clouds.
 */
class DynamicRemovalBase {
public:
  virtual ~DynamicRemovalBase() {}

  /**
   * @brief Filter dynamic points from a merged submap cloud.
   * @param merged_origin          Merged submap cloud in the submap origin frame
   * @param keyframes              Contributing keyframe clouds in their sensor frames
   * @param poses_origin_keyframe  Poses from each keyframe sensor frame to the submap origin (T_origin_keyframe)
   * @return Filtered cloud in the submap origin frame, or nullptr to keep merged_origin unchanged
   */
  virtual gtsam_points::PointCloud::Ptr filter(
    const gtsam_points::PointCloud::ConstPtr& merged_origin,
    const std::vector<gtsam_points::PointCloud::ConstPtr>& keyframes,
    const std::vector<Eigen::Isometry3d>& poses_origin_keyframe) const = 0;

  /**
   * @brief Create a dynamic removal strategy from config.
   * @param name   Strategy name ("NONE", "VOXEL_HIT_RATIO", ...)
   * @param config Configuration loader
   * @return Strategy instance, or nullptr if disabled / unknown
   */
  static std::unique_ptr<DynamicRemovalBase> create(const std::string& name, const Config& config);
};

}  // namespace glim
