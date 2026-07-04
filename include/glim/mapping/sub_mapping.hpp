#pragma once

#include <any>
#include <deque>
#include <random>
#include <memory>
#include <glim/mapping/sub_mapping_base.hpp>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace glim {

class IMUIntegration;
class CloudDeskewing;
class CloudCovarianceEstimation;
class DynamicRemovalBase;

/**
 * @brief Parameters for the default sub-mapping backend (loaded from config_sub_mapping).
 *
 * Controls how odometry frames are accumulated into a local factor graph, when keyframes are
 * inserted, and how finished submaps are merged and post-processed.
 */
struct SubMappingParams {
public:
  /** @brief Load defaults from config_sub_mapping. */
  SubMappingParams();
  ~SubMappingParams();

public:
  /** @brief True when registration_error_factor_type selects a GPU backend (e.g. VGICP_GPU). */
  bool enable_gpu;
  /** @brief If true, integrate IMU for deskewing, ImuFactors, and velocity/bias variables. */
  bool enable_imu;
  /** @brief If false, skip LM optimization in create_submap() and keep odometry poses. */
  bool enable_optimization;

  /** @brief Maximum keyframes per submap; reaching this triggers create_submap(). */
  int max_num_keyframes;
  /** @brief Keyframe gate: "OVERLAP" (view overlap) or "DISPLACEMENT" (motion threshold). */
  std::string keyframe_update_strategy;
  /** @brief Frames with fewer points are never promoted to keyframes. */
  int keyframe_update_min_points;
  /** @brief Min rotation [rad] between keyframes when strategy is DISPLACEMENT. */
  double keyframe_update_interval_rot;
  /** @brief Min translation [m] between keyframes when strategy is DISPLACEMENT. */
  double keyframe_update_interval_trans;
  /** @brief Max scan overlap with last keyframe before inserting a new one (OVERLAP strategy). */
  double max_keyframe_overlap;

  /** @brief If true, add BetweenFactor constraints between consecutive odometry poses. */
  bool create_between_factors;
  /** @brief Relative pose noise model for between factors: "GICP", "NONE", etc. */
  std::string between_registration_type;

  /** @brief Scan-matching factor type for keyframe pairs: "VGICP", "VGICP_GPU", etc. */
  std::string registration_error_factor_type;
  /** @brief Fraction of deskewed points kept per keyframe (1.0 = all points). */
  double keyframe_randomsampling_rate;
  /** @brief Base voxel size [m] for keyframe Gaussian voxel maps. */
  double keyframe_voxel_resolution;
  /** @brief Number of voxel-map pyramid levels built per keyframe. */
  int keyframe_voxelmap_levels;
  /** @brief Resolution multiplier between consecutive voxel-map levels. */
  double keyframe_voxelmap_scaling_factor;

  /** @brief Voxel downsample resolution [m] when merging keyframes into submap->frame. */
  double submap_downsample_resolution;
  /** @brief [Deprecated] Reserved for submap voxel settings used downstream in global mapping. */
  double submap_voxel_resolution;
  /** @brief Optional cap on merged submap points; disabled when < 0. Applied after dynamic removal. */
  int submap_target_num_points;

  /** @brief Dynamic object removal strategy name ("NONE", "VOXEL_HIT_RATIO", ...). */
  std::string dynamic_removal_method;
};

/**
 * @brief Default sub-mapping: local pose-graph optimization and submap assembly.
 *
 * Consumes marginalized odometry frames (and optional IMU), maintains an incremental GTSAM
 * factor graph over all frames in the current submap window, and emits SubMap objects when
 * max_num_keyframes is reached or at end-of-sequence.
 *
 * ## Data flow
 *
 * 1. insert_imu() buffers IMU samples for preintegration and scan deskewing.
 * 2. insert_frame() processes odometry frames one step behind (pairs needed for IMU interval):
 *    - Smooths an IMU-rate trajectory for motion compensation.
 *    - Adds X(i) pose nodes and optional V(i)/B(i) IMU nodes to values_/graph_.
 *    - Adds between-factors (odometry/GICP), ImuFactors, and VGICP keyframe constraints.
 *    - Selectively calls insert_keyframe() when view overlap or displacement criteria are met.
 * 3. When the keyframe budget is full, create_submap() optimizes the graph, merges keyframe
 *    scans into SubMap::frame (submap-origin coordinates), optionally runs dynamic removal,
 *    and pushes the result to submap_queue.
 * 4. get_submaps() drains submap_queue; submit_end_of_sequence() forces a final submap from
 *    any remaining frames.
 *
 * Graph variables (GTSAM symbols): X(i) = sensor pose, V(i) = IMU velocity, B(i) = IMU bias.
 */
class SubMapping : public SubMappingBase {
public:
  /**
   * @brief Construct sub-mapping with the given parameters.
   * @param params  Configuration; defaults are read from config_sub_mapping when constructed inline.
   */
  SubMapping(const SubMappingParams& params = SubMappingParams());
  virtual ~SubMapping() override;

  /**
   * @brief Buffer an IMU sample for preintegration and deskewing.
   *
   * Forwarded to IMUIntegration when enable_imu is true. Also triggers SubMappingCallbacks::on_insert_imu.
   */
  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;

  /**
   * @brief Ingest one marginalized odometry frame and extend the local factor graph.
   *
   * Holds the incoming frame in delayed_input_queue until a consecutive pair exists, then:
   * deskews/smooths IMU trajectory, registers graph nodes and factors, may add a keyframe,
   * and creates a submap when max_num_keyframes is satisfied.
   */
  virtual void insert_frame(const EstimationFrame::ConstPtr& odom_frame) override;

  /**
   * @brief Return and clear all submaps finished since the last call.
   *
   * Submaps are produced asynchronously inside insert_frame() when the keyframe window fills.
   */
  virtual std::vector<SubMap::Ptr> get_submaps() override;

  /**
   * @brief Flush any in-progress submap at end of the input sequence.
   *
   * Calls create_submap(force_create=true) if odom_frames is non-empty.
   */
  virtual std::vector<SubMap::Ptr> submit_end_of_sequence() override;

private:
  /**
   * @brief Build a keyframe scan and multi-resolution voxel maps for registration factors.
   *
   * Re-deskews from raw points when IMU trajectory is available, subsamples the cloud,
   * stores it on keyframes, and records keyframe_indices[current] = graph index X(current).
   *
   * @param current     Index of this frame in odom_frames / the factor graph.
   * @param odom_frame  Source odometry frame (poses, raw scan, IMU trajectory).
   */
  void insert_keyframe(const int current, const EstimationFrame::ConstPtr& odom_frame);

  /**
   * @brief Optimize the current submap graph and assemble a SubMap.
   *
   * Runs when keyframes.size() >= max_num_keyframes, or when force_create is true.
   * Optimizes values_/graph_, writes refined poses into SubMap::frames, merges keyframe
   * point clouds into SubMap::frame, applies optional dynamic removal and point-cap sampling.
   *
   * @param force_create  If true, create a submap even below max_num_keyframes (end-of-sequence).
   * @return New submap, or nullptr if the window is not ready and force_create is false.
   */
  SubMap::Ptr create_submap(bool force_create = false) const;

private:
  using Params = SubMappingParams;
  Params params;

  /** @brief RNG for keyframe random sampling and submap point-cap downsampling. */
  std::mt19937 mt;
  /** @brief Monotonic id assigned to each completed submap. */
  int submap_count;

  /** @brief IMU queue and preintegration used for ImuFactors and trajectory smoothing. */
  std::unique_ptr<IMUIntegration> imu_integration;
  /** @brief Motion compensation of raw scans for keyframes. */
  std::unique_ptr<CloudDeskewing> deskewing;
  /** @brief Per-point covariance estimation after deskewing. */
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;
  /** @brief Optional moving-object filter applied to merged submap clouds (nullptr if disabled). */
  std::unique_ptr<DynamicRemovalBase> dynamic_removal;

  /** @brief CUDA stream (GTSAM_POINTS_USE_CUDA); stored as void* to avoid header dependency. */
  std::shared_ptr<void> stream;
  /** @brief Round-robin GPU temp buffers for VGICP_GPU factors. */
  std::shared_ptr<void> stream_buffer_roundrobin;

  /**
   * @brief Single-frame delay so insert_frame always has [frame_i, frame_{i+1}] for IMU integration.
   */
  std::deque<EstimationFrame::ConstPtr> delayed_input_queue;
  /**
   * @brief All odometry frames in the current submap window (graph index i == odom_frames[i]).
   *
   * Point clouds on older entries may be stripped via clone_wo_points() to save memory.
   */
  std::vector<EstimationFrame::ConstPtr> odom_frames;

  /**
   * @brief keyframe_indices[k] is the odom_frames / X(i) index for keyframes[k].
   */
  std::vector<int> keyframe_indices;
  /** @brief Subset of frames promoted to keyframes; carry subsampled scans and voxel maps. */
  std::vector<EstimationFrame::Ptr> keyframes;

  /** @brief Current submap pose/velocity/bias estimates (initial values + optimization result). */
  std::unique_ptr<gtsam::Values> values;
  /** @brief Incremental factor graph: priors, between, Imu, and VGICP factors for the active window. */
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph;

  /** @brief Completed submaps waiting for get_submaps() / submit_end_of_sequence(). */
  std::vector<SubMap::Ptr> submap_queue;

  /** @brief Optional single-thread TBB arena for submap LM optimization (GTSAM_USE_TBB). */
  std::shared_ptr<void> tbb_task_arena;
};

}  // namespace glim
