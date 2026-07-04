#include <glim/mapping/dynamic_removal/dynamic_removal_base.hpp>

#include <algorithm>
#include <spdlog/spdlog.h>
#include <glim/mapping/dynamic_removal/voxel_hit_ratio_removal.hpp>
#include <glim/util/config.hpp>

namespace glim {

std::unique_ptr<DynamicRemovalBase> DynamicRemovalBase::create(const std::string& name, const Config& config) {
  std::string method = name;
  std::transform(method.begin(), method.end(), method.begin(), ::toupper);

  if (method.empty() || method == "NONE") {
    return nullptr;
  }

  if (method == "VOXEL_HIT_RATIO") {
    return std::make_unique<VoxelHitRatioRemoval>(config);
  }

  spdlog::warn("unknown dynamic_removal_method: {}", name);
  return nullptr;
}

}  // namespace glim
