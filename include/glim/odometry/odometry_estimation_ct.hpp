#pragma once

#include <deque>
#include <memory>
#include <future>

#include <glim/odometry/odometry_estimation_base.hpp>

namespace gtsam {
class Values;
}

namespace gtsam_points {
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;

class IncrementalCovarianceVoxelMap;
using iVoxCovarianceEstimation = IncrementalCovarianceVoxelMap;
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace glim {

class CloudCovarianceEstimation;

/**
 * @brief Configuration parameters for continuous-time (CT) GICP odometry.
 *
 * CT-GICP treats each LiDAR sweep as a continuous motion between a start pose and
 * an end pose, which allows deskewing and registration in one step. These parameters
 * control the target map (iVox), per-scan LM registration, and the fixed-lag smoother
 * that maintains a sliding window of pose estimates. Defaults are read from
 * config_odometry.json when constructed with no arguments.
 */
struct OdometryEstimationCTParams {
public:
  /** @brief Load defaults from config_odometry.json. */
  OdometryEstimationCTParams();
  ~OdometryEstimationCTParams();

public:
  int num_threads;  ///< Thread count for covariance estimation and CT-GICP factor evaluation.

  double ivox_resolution;       ///< Voxel edge length [m] of the incremental target map (iVox).
  double ivox_min_points_dist;  ///< Minimum spatial separation [m] between points stored in the same iVox cell.
  int ivox_lru_thresh;          ///< LRU horizon: number of recent scans whose voxels are retained in the target map.

  double max_correspondence_distance;     ///< Outlier rejection threshold [m] for point-to-voxel correspondences in CT-GICP.
  double location_consistency_inf_scale;  ///< Information (precision) scale for the prior tying the new scan start pose to the previous scan end pose.
  double constant_velocity_inf_scale;     ///< Information scale for the between factor enforcing near-zero motion within a single sweep (scan begin to scan end).
  int lm_max_iterations;                  ///< Maximum Levenberg–Marquardt iterations for the per-scan pose initialization before inserting into the smoother.

  // iSAM2 params
  double smoother_lag;    ///< Fixed-lag smoothing window length [sec]; poses older than this are marginalized and returned to the caller.
  bool use_isam2_dogleg;  ///< If true, iSAM2 uses the Dogleg optimizer instead of Gauss-Newton for back-end updates.
  double isam2_relinearize_skip;   ///< Number of smoother updates to skip between variable relinearizations (iSAM2 parameter).
  double isam2_relinearize_thresh; ///< Linearization error threshold that triggers relinearization (iSAM2 parameter).
};

/**
 * @brief LiDAR-only odometry based on continuous-time GICP scan-to-model matching.
 *
 * Each incoming preprocessed scan is registered against an iVox target built from
 * previously aligned scans. Two pose variables per frame (X = sweep start, Y = sweep
 * end) model intra-scan motion; CT-GICP factors deskew points while minimizing
 * distribution-to-distribution error against the map. A fixed-lag iSAM2 smoother
 * fuses sequential between-factors and priors over the sliding window. No IMU is
 * required or used.
 */
class OdometryEstimationCT : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Construct the estimator and initialize the target map and smoother.
   * @param params  Registration, iVox, and smoother settings.
   */
  OdometryEstimationCT(const OdometryEstimationCTParams& params = OdometryEstimationCTParams());
  virtual ~OdometryEstimationCT() override;

  /**
   * @brief Returns false because this backend is pure LiDAR odometry.
   */
  virtual bool requires_imu() const override { return false; }

  /**
   * @brief Register a new scan, update the target map, and advance the smoother.
   *
   * For the first scan, initializes the world frame and adds anchor priors. For
   * subsequent scans, runs a local LM solve with CT-GICP against the iVox target,
   * deskews the scan, inserts transformed points into the map, and updates the
   * fixed-lag smoother. Frames that fall outside the smoothing window are moved
   * into @p marginalized_frames.
   *
   * @param frame                Preprocessed point cloud (with per-point timestamps).
   * @param marginalized_frames  [out] Frames removed from the active smoothing window.
   * @return                     Estimation result for the newly inserted scan.
   */
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;

private:
  using Params = OdometryEstimationCTParams;
  Params params;  ///< Copy of the configuration used for this estimator instance.

  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;  ///< Estimates per-point normals and covariances for GICP.

  int marginalized_cursor;                            ///< Index of the oldest frame still held in @c frames; earlier slots are empty after marginalization.
  std::vector<EstimationFrame::Ptr> frames;         ///< Active estimation frames indexed by frame id (sparse before @c marginalized_cursor).
  std::shared_ptr<gtsam_points::iVox> target_ivox;  ///< Incremental voxel target map used for scan-to-model CT-GICP registration.
  EstimationFrame::ConstPtr target_ivox_frame;      ///< Snapshot of target map points for visualization callbacks only (not used in registration).

  // Optimizer
  using FixedLagSmootherExt = gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;  ///< Fixed-lag iSAM2 smoother maintaining the sliding-window pose graph.

  std::shared_ptr<void> tbb_task_arena;  ///< Optional single-thread TBB arena to serialize smoother updates when GTSAM_USE_TBB is defined.
};

}  // namespace glim
