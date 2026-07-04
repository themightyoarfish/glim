# GLIM — Q&A

Frequently asked questions gathered from development and code review. Covers the full SLAM pipeline, sub-mapping, point cloud representation, IMU/GTSAM graph semantics, utilities, and CT odometry.

---

## GLIM SLAM process overview

This section gives a high-level picture of how GLIM turns raw sensor data into a globally consistent map. GLIM is a **hierarchical SLAM system**: a fast local front-end (odometry) feeds a mid-level builder (sub-mapping), which in turn feeds a slow back-end (global mapping) that closes loops and distributes error across the full trajectory.

```
Raw sensors
    │
    ▼
Preprocessing  ──►  PreprocessedFrame  (filtered, downsampled LiDAR + k-NN indices)
    │
    ▼
Odometry       ──►  EstimationFrame     (scan-matched poses, deskewed clouds, IMU state)
    │                  │ marginalized frames leave the fixed-lag window
    ▼                  ▼
Sub-mapping    ──►  SubMap              (local chunks: merged cloud + optimized keyframes)
    │
    ▼
Global mapping ──►  World-consistent map (submap poses + loop closure + export)
```

Each stage runs in its own thread in the live system (`AsyncOdometryEstimation`, `AsyncSubMapping`, `AsyncGlobalMapping`), connected by concurrent queues. The standalone `pcd_mapper` runs the same chain synchronously on a directory of PCD files.

---

### Preprocessing

**Role:** Convert raw LiDAR packets into a clean, registration-ready point cloud before any pose estimation happens.

`CloudPreprocessor` (`include/glim/preprocess/cloud_preprocessor.hpp`) takes a `RawPoints` buffer and produces a `PreprocessedFrame`:

| Output field | Purpose |
|---|---|
| `points` | Filtered 3-D points (homogeneous coordinates) |
| `times` | Per-point timestamps relative to scan start (needed for deskewing) |
| `intensities` | Optional reflectivity values |
| `neighbors` / `k_neighbors` | k-nearest-neighbor indices for local covariance estimation |
| `stamp` / `scan_end_time` | Scan start and end times |

Typical steps (`config_preprocess.json`):

1. **Range filtering** — Remove points closer than `distance_near_thresh` or farther than `distance_far_thresh`.
2. **Downsampling** — Voxel grid or random-grid downsampling to `downsample_target` points.
3. **Outlier removal** — Optional statistical outlier filter.
4. **Crop-box filter** — Optional region-of-interest in LiDAR or IMU frame.
5. **k-NN search** — Build neighbor lists used later by GICP/CT-GICP covariance estimation.

If `global_shutter_lidar` is set in `config_sensors.json`, per-point timestamps are zeroed (deskewing disabled).

Preprocessing is **LiDAR-only**. IMU and camera data bypass this stage and are ingested directly by odometry and mapping modules.

---

### Odometry

**Role:** Estimate a locally consistent trajectory and produce deskewed scans fast enough for real-time use. Odometry operates in a **drifting world frame** (call it "odom"). Downstream modules treat these poses as initial guesses and later align them globally.

All odometry backends inherit from `OdometryEstimationBase` and are loaded as shared libraries (`libodometry_estimation_{gpu,cpu,ct}.so`). Each implements `insert_frame()` and optionally `insert_imu()`.

#### Three odometry backends

| Backend | Library | Sensors | Registration | Primary target structure |
|---------|---------|---------|--------------|--------------------------|
| **GPU** (default) | `libodometry_estimation_gpu.so` | LiDAR + IMU | GPU VGICP scan-to-keyframe-map | Multi-level `GaussianVoxelMapGPU` per keyframe |
| **CPU** | `libodometry_estimation_cpu.so` | LiDAR + IMU | GICP or VGICP scan-to-model | iVox (`GICP`) or `GaussianVoxelMapCPU` (`VGICP`) |
| **CT** | `libodometry_estimation_ct.so` | LiDAR only | CT-GICP (continuous-time) | iVox in world frame |

##### GPU odometry (`OdometryEstimationGPU`)

The default and most capable front-end. Extends `OdometryEstimationIMU`.

1. **Initialization** — Waits for IMU-based initial state (`InitialStateEstimation`) before processing the first scan.
2. **Per scan** — Deskews points using IMU preintegration, builds multi-resolution GPU voxel maps on the current frame.
3. **Keyframe management** — Maintains a sliding window of keyframes selected by overlap, displacement, or entropy (`keyframe_update_strategy`). Old keyframes with insufficient overlap are marginalized.
4. **Scan matching** — Adds VGICP_GPU factors between the current frame and recent keyframes / consecutive frames. Voxel resolution adapts to median scan range.
5. **IMU factors** — `ImuFactor` connects consecutive poses with preintegrated IMU; bias random-walk factors on `B(i)`.
6. **Smoother** — Fixed-lag iSAM2 (`IncrementalFixedLagSmootherExtWithFallback`) over `X(i)`, `V(i)`, `B(i)`.

Variables: `X(i)` = IMU pose, `V(i)` = velocity, `B(i)` = bias.

##### CPU odometry (`OdometryEstimationCPU`)

Semi-tightly coupled LiDAR-IMU odometry on CPU. Also extends `OdometryEstimationIMU`.

1. Same IMU initialization and smoother structure as GPU.
2. **Per scan** — Runs a local Levenberg–Marquardt solve with either:
   - **GICP** against an iVox target map (`IntegratedGICPFactor`), or
   - **VGICP** against multi-level CPU voxel maps.
3. **Target update** — Matched scans are transformed into the target frame and inserted. After frame 5, points are random-downsampled before insertion.
4. Target map bounded by LRU eviction (`lru_thresh`).

The target frame moves with the sensor (`T_target_imu`), unlike CT odometry which keeps the iVox in world coordinates.

##### CT odometry (`OdometryEstimationCT`)

LiDAR-only; no IMU required (`requires_imu() == false`). See the dedicated CT section below for details.

1. Two pose variables per sweep: `X(i)` = scan start, `Y(i)` = scan end.
2. Local LM with CT-GICP factor against a world-frame iVox, plus location-consistency and constant-velocity priors.
3. Deskews the scan, inserts transformed points into iVox, updates fixed-lag iSAM2 smoother.
4. Sets `EstimationFrame::frame_id = LIDAR`.

#### Odometry output: marginalized frames

Each backend maintains a fixed-lag smoother window (`smoother_lag` seconds). When a frame ages out, it is pushed to `marginalized_frames` and handed to sub-mapping. The active window keeps only recent states for low-latency optimization.

---

### Sub-mapping

**Role:** Aggregate odometry frames into local **submaps** — merged point cloud chunks with locally optimized poses — and emit them to global mapping.

`SubMapping::insert_frame()` receives marginalized `EstimationFrame`s and:

1. **IMU trajectory smoothing** (if `enable_imu`) — Preintegrates IMU between consecutive LiDAR stamps, optimizes a small pose-only graph, stores an IMU-rate trajectory on the frame for high-quality re-deskewing.
2. **Graph growth** — Inserts pose `X(i)` initialized from `odom_frame->T_world_sensor()`.
3. **Inter-frame constraints** — Optional scan-matching between-factors and IMU factors (`ImuFactor`, velocity/bias priors).
4. **Keyframe selection** — Not every odometry frame becomes a keyframe. Selection uses overlap or displacement thresholds.
5. **Scan-to-map factors** — New keyframes get VGICP / VGICP_GPU registration-error factors against the submap's voxel maps.
6. **Submap emission** — When `max_num_keyframes` is reached, `create_submap()` runs batch optimization (Levenberg–Marquardt if `enable_optimization`), merges keyframe scans into a single cloud in submap-origin coordinates, optionally runs dynamic object removal, and pushes a `SubMap` to the output queue.

Each `SubMap` contains:

- `frame` — Merged point cloud in submap-origin frame
- `voxelmaps` — Multi-resolution Gaussian voxel maps for registration
- `frames` / `odom_frames` — Optimized and original odometry metadata
- `T_world_origin` — Pose of the submap origin in the (still local) odometry world

Sub-mapping operates in the **odometry frame**. Its "world" is the drifting odometry coordinate system.

**Important:** With CT (LiDAR-only) odometry, set `enable_imu: false` in sub-mapping config. IMU deskewing and `ImuFactor` machinery expect `frame_id = IMU`.

---

### Global mapping

**Role:** Align submaps into a single **globally consistent map** by detecting geometric overlap between distant submaps (implicit loop closure) and optionally running explicit loop detection.

Two implementations exist:

| Class | Loop detection | Typical use |
|-------|---------------|-------------|
| `GlobalMapping` | Implicit (overlap-based VGICP factors between nearby submaps) | Default; real-time mapping |
| `GlobalMappingPoseGraph` | Explicit (ScanContext, DBoW, etc. via extension modules) | Offline / large-scale |

When a submap arrives (`insert_submap()`):

1. **Initialize submap pose** — Chain from the previous submap's optimized pose using odometry relative motion between submap endpoints.
2. **Between-factors** — Connect consecutive submaps (optional scan-matched or odometry-derived).
3. **Matching-cost factors** — VGICP / VGICP_GPU registration-error factors between the new submap and all previously overlapping submaps within `max_implicit_loop_distance` and above `min_implicit_loop_overlap`. These act as **implicit loop closures**.
4. **IMU factors** (if `enable_imu`) — Connect submap endpoint velocities and biases.
5. **iSAM2 update** — Incrementally optimize the growing submap pose graph.
6. **Pose write-back** — Update each submap's `T_world_origin` from optimized `X(i)`.

The global map is stored as a **vector of submaps**, each with its own local cloud and world pose. A single merged cloud is produced only on export via `GlobalMapping::export_points()`.

Global mapping's "world" is the **true global frame** — the one that gets corrected when loops close.

---

### Sensor modalities

#### LiDAR

The primary sensing modality. Every pipeline stage depends on LiDAR point clouds:

- **Preprocessing** filters and downsamples raw scans.
- **Odometry** registers scans (GICP, VGICP, or CT-GICP).
- **Sub-mapping** merges deskewed scans into submap clouds.
- **Global mapping** aligns submap clouds for loop closure.

Per-point **timestamps** are essential for motion-compensated (deskewed) registration in rotating LiDAR sensors.

#### IMU

Used by GPU and CPU odometry backends and optionally by sub-mapping and global mapping.

| Stage | IMU usage |
|-------|-----------|
| Odometry | `ImuFactor` between consecutive poses; bias estimation; scan deskewing via preintegration; initial state estimation |
| Sub-mapping | Re-deskewing with IMU-rate trajectory; optional `ImuFactor` in submap graph |
| Global mapping | IMU factors between submap endpoints (if `enable_imu`) |
| CT odometry | **Not used** |

IMU data enters through `insert_imu(stamp, linear_acc, angular_vel)`. The `AsyncOdometryEstimation` thread synchronizes LiDAR frames to IMU time — a scan is not processed until IMU data covers its `scan_end_time`.

Extrinsics (`T_lidar_imu` in `config_sensors.json`) define the rigid transform between LiDAR and IMU frames. `EstimationFrame` stores both `T_world_lidar` and `T_world_imu`, kept consistent via `T_lidar_imu`.

#### Visual features / cameras

GLIM core does **not** perform visual SLAM or feature tracking internally. Camera support is optional (`GLIM_USE_OPENCV`):

- `insert_image(stamp, image)` exists on odometry, sub-mapping, and global mapping base classes.
- Images are forwarded through the async wrappers and exposed via **callback slots** for extension modules.

Visual integration is expected to come from **extension modules** in [glim_ext](https://github.com/koide3/glim_ext), for example:

- **ORB-SLAM frontend** — Loosely coupled visual odometry constraints injected into the odometry factor graph via callbacks.
- **DBoW / ScanContext loop detectors** — Explicit loop closure in global mapping.

The core pipeline is LiDAR-centric; vision is an optional add-on through the callback/extension mechanism described in `docs/extend.md`.

---

### Factor graphs and optimization

GLIM uses [GTSAM](https://gtsam.org/) factor graphs at every stage. Each stage has its own variables, factors, and optimizer.

#### Variable naming convention

GTSAM symbols label variables in the graph:

| Symbol | Meaning (context-dependent) |
|--------|----------------------------|
| `X(i)` | Pose (IMU, LiDAR, or submap — depends on stage) |
| `Y(i)` | CT odometry scan-end pose only |
| `V(i)` | IMU velocity in world/odom frame |
| `B(i)` | IMU bias (gyro + accel) |

See `docs/extend.md` for the full variable reference.

#### Factor types used across the pipeline

| Factor | Where used | Purpose |
|--------|-----------|---------|
| `IntegratedGICPFactor` / `IntegratedCT_GICPFactor` | Odometry | Point-to-distribution scan matching |
| `IntegratedVGICPFactor` / `_GPU` | Odometry, sub-mapping, global mapping | Voxelized GICP (faster, multi-resolution) |
| `ImuFactor` | Odometry, sub-mapping, global mapping | Preintegrated IMU motion model |
| `BetweenFactor<Pose3>` | All stages | Relative pose constraint |
| `PriorFactor<Pose3>` | All stages | Absolute pose anchor |
| `LinearDampingFactor` | Odometry init, global mapping | Soft origin fix (gauge setting) |
| Bias `BetweenFactor` | Odometry, sub-mapping | IMU bias random walk |

#### Optimizers at each stage

| Stage | Optimizer | Window |
|-------|-----------|--------|
| Odometry (per-scan LM) | `LevenbergMarquardtOptimizerExt` | Single scan (CT, CPU) |
| Odometry (back-end) | Fixed-lag iSAM2 (`IncrementalFixedLagSmootherExtWithFallback`) | `smoother_lag` seconds |
| Sub-mapping (submap creation) | Levenberg–Marquardt (batch) | All keyframes in current submap |
| Global mapping | iSAM2 (`ISAM2Ext`) | All submaps (unbounded) |

Local LM solves provide good initial guesses; iSAM2 incrementally refines and relinearizes the graph as new data arrives.

---

### iSAM2 in GLIM

[iSAM2](https://gtsam.org/doxygen/classgtsam_1_1ISAM2.html) (Incremental Smoothing and Mapping) is GTSAM's incremental nonlinear optimizer. Instead of re-solving the entire graph from scratch, it updates a Bayes tree when new factors arrive, relinearizing only variables whose linearization error exceeds a threshold.

GLIM uses two iSAM2 wrappers from gtsam_points:

| Wrapper | Used in | Key feature |
|---------|---------|-------------|
| `IncrementalFixedLagSmootherExtWithFallback` | Odometry (all backends) | Fixed-lag window: old variables are marginalized out after `smoother_lag` seconds. Fallback recovery if the graph becomes ill-conditioned. |
| `ISAM2Ext` | Global mapping | Unbounded incremental optimizer over all submap poses. Supports GPU factors. |

Configuration knobs (shared across stages):

- `smoother_lag` — How long odometry states stay in the graph (odometry only).
- `use_isam2_dogleg` — Dogleg vs Gauss-Newton optimizer.
- `isam2_relinearize_skip` — Skip N updates between relinearizations.
- `isam2_relinearize_thresh` — Relinearization error threshold.

**Why fixed-lag in odometry but full iSAM2 in global mapping?**

Odometry must run in real time on a sliding window — keeping the entire history would be too slow and memory-heavy. Global mapping operates on submaps (orders of magnitude fewer variables than scans) and needs the full history to close loops, so it uses unbounded iSAM2.

Callbacks (`on_smoother_update`, `on_smoother_update_finish`) expose the smoother state to extension modules that can inject additional factors (e.g., visual constraints, velocity suppression).

---

### iVox — incremental voxel map

The **iVox** (`gtsam_points::iVox` = `IncrementalVoxelMap<FlatContainer>`) is an incremental voxel hash map used as a nearest-neighbor target for GICP registration. It follows the Faster-LIO design (Bai et al., IEEE RA-L 2022).

**Structure:**

- Points are hashed into voxels of edge length `ivox_resolution`.
- Each voxel stores up to one point (controlled by `ivox_min_points_dist` / `ivox_min_dist`).
- Supports k-NN search with neighbor-voxel lookup (`set_neighbor_voxel_mode`).

**LRU eviction:**

- `set_lru_horizon(N)` — Voxels not accessed during correspondence search for N insertion steps are deleted.
- This bounds memory and keeps the target map local to the recent trajectory.
- Eviction is based on **access during registration**, not on spatial distance alone.

**Where iVox is used:**

| Module | Role |
|--------|------|
| CT odometry | World-frame scan-to-model target for CT-GICP |
| CPU odometry (GICP mode) | Sensor-attached scan-to-model target |
| Loose initial state estimation | Short-range map for IMU init |

iVox is **not** used in GPU odometry (which uses Gaussian voxel maps) or in sub-mapping / global mapping (which use `GaussianVoxelMap` for VGICP).

**iVox vs GaussianVoxelMap:**

| | iVox | GaussianVoxelMap |
|---|---|---|
| Point storage | Raw points per voxel | Voxel mean + covariance |
| Registration | GICP (point-to-distribution) | VGICP (voxel-to-voxel) |
| Typical use | Local odometry target | Keyframe/submap representation |
| GPU support | CPU only | CPU and GPU |

---

### EstimationFrame — the central data object

`EstimationFrame` (`include/glim/odometry/estimation_frame.hpp`) is the primary data structure passed between odometry, sub-mapping, and (indirectly) global mapping. Each LiDAR scan produces one estimation frame.

#### Core fields

| Field | Purpose |
|-------|---------|
| `id`, `stamp` | Frame index and scan-start timestamp |
| `raw_frame` | Original preprocessed cloud (LiDAR frame) |
| `frame` | Deskewed point cloud used for registration (may be in IMU or LiDAR frame) |
| `frame_id` | Which coordinate frame `frame` and `T_world_sensor()` refer to (`IMU` or `LIDAR`) |
| `voxelmaps` | Multi-resolution Gaussian voxel maps (GPU/CPU odometry attach these to keyframes) |

#### Coordinate frames

Three rigid transforms are stored:

| Transform | Meaning |
|-----------|---------|
| `T_world_lidar` | LiDAR pose in the odometry world frame |
| `T_world_imu` | IMU pose in the odometry world frame |
| `T_lidar_imu` | Fixed extrinsic: LiDAR frame → IMU frame |

`T_world_sensor()` dispatches on `frame_id`:

- `FrameID::IMU` → returns `T_world_imu` (GPU/CPU odometry)
- `FrameID::LIDAR` → returns `T_world_lidar` (CT odometry)

`set_T_world_sensor(frame_id, T)` updates the primary pose and derives the other via extrinsics:

```
T_world_imu  = T_world_lidar * T_lidar_imu       (when frame_id = LIDAR)
T_world_lidar = T_world_imu * T_lidar_imu⁻¹      (when frame_id = IMU)
```

This lets sub-mapping use a single API (`T_world_sensor()`) regardless of which odometry backend produced the frame.

#### Why IMU fields are retained on a LiDAR-centric object

Even when the primary output is a LiDAR pose, the frame carries IMU state for downstream modules:

| Field | Used by |
|-------|---------|
| `v_world_imu` | Sub-mapping velocity priors; global mapping IMU factors |
| `imu_bias` | Sub-mapping and global mapping bias variables |
| `imu_rate_trajectory` | Sub-mapping re-deskewing (8 × N matrix: time + pose at IMU rate) |

CT odometry zeroes these fields (`v_world_imu`, `imu_bias`) and sets `T_lidar_imu = Identity`, but the fields remain so the same struct works across all backends without polymorphism.

#### Why both `raw_frame` and `frame` exist

- `raw_frame` — Preprocessed but **not deskewed**; points are in the LiDAR sensor frame with per-point timestamps. Sub-mapping uses this for high-quality re-deskewing with the IMU-rate trajectory.
- `frame` — **Deskewed** cloud in the sensor frame (`IMU` or `LIDAR` depending on backend), with GICP covariances attached. Used for scan matching and voxel map building.

#### Memory management

- `clone()` — Shallow copy (shared point data).
- `clone_wo_points()` — Copy metadata only; strips clouds to save memory.
- `SubMap::drop_frame_points()` — Removes per-frame clouds from a finalized submap after merging.

#### Custom data

`custom_data` is a string-keyed map for extension modules to attach arbitrary payloads to a frame without modifying the core struct.

---

## Sub-mapping overview

**Q: What does `SubMapping::insert_frame()` do?**

A: Each call queues an odometry frame and processes the *previous* frame once a consecutive pair exists (one-frame delay for IMU intervals). Processing has five stages:

1. **IMU trajectory smoothing** (`enable_imu`) — Preintegrate IMU between two LiDAR stamps, optimize a small pose-only graph anchored at both endpoints, store the result on `odom_frame->imu_rate_trajectory` for scan deskewing in `insert_keyframe()`.

2. **Submap graph growth** — Append the frame to `odom_frames`, insert pose `X(current)` into `values_`, fix gauge with a strong prior on `X(0)`.

3. **Inter-frame constraints** (`current > 0`) — Optional between-factors (`create_between_factors`, `between_registration_type`); optional IMU factors (`enable_imu`) with velocity/bias variables and `ImuFactor` between consecutive frames.

4. **Keyframe selection** (`keyframe_update_strategy`, overlap/displacement thresholds) — Not every frame is a keyframe. New keyframes trigger `insert_keyframe()` and VGICP / VGICP_GPU scan-to-map factors (`registration_error_factor_type`).

5. **Submap emission** (`max_num_keyframes`) — `create_submap()` optimizes the graph (`enable_optimization`), merges keyframes, optionally runs dynamic removal, pushes a `SubMap` to `submap_queue`, and resets state.

See: `src/glim/mapping/sub_mapping.cpp`, `include/glim/mapping/sub_mapping.hpp`

**Q: What does `insert_keyframe()` do?**

A: Builds a registration-quality scan for the submap factor graph:

1. Optionally re-deskews from raw points using the smoothed `imu_rate_trajectory` (better than odometry's initial deskew).
2. Random-subsamples the cloud (`keyframe_randomsampling_rate`) for VGICP cost.
3. Builds multi-resolution Gaussian voxel maps (`keyframe_voxel_resolution`, `keyframe_voxelmap_levels`).
4. Registers the keyframe in `keyframes` with `keyframe_indices[k]` mapping to graph index `X(i)`.

On CPU, voxel maps are built before `keyframe->frame` is assigned to the subsampled cloud — voxel maps may therefore use the pre-copy odometry cloud unless that ordering is fixed.

---

## Fused point clouds in submaps and global maps

**Q: How is the fused point cloud stored in a submap?**

A: Each `SubMap` holds one merged cloud on **`SubMap::frame`** — a `gtsam_points::PointCloud::Ptr` in **submap-origin coordinates** (not world). The origin is the center frame pose `T_world_origin`.

It is built in `SubMapping::create_submap()` by merging keyframe scans with `gtsam_points::merge_frames_auto()`, using optimized poses relative to the origin. Optional post-steps: dynamic object removal, then `submap_target_num_points` random downsampling.

Each submap also carries:
- `frames` / `odom_frames` — per-scan odometry metadata (points may be stripped to save memory).
- `voxelmaps` — multi-resolution Gaussian voxel maps rebuilt in global mapping for loop closure.

See: `include/glim/mapping/sub_map.hpp`, `src/glim/mapping/sub_mapping.cpp`

**Q: How is the global map represented?**

A: During mapping, the global map is **not** one monolithic fused cloud. `GlobalMapping` stores:

- `std::vector<SubMap::Ptr> submaps` — each with its own `frame` and `T_world_origin`.
- `subsampled_submaps` — random-sampled copies used for registration.

A single world-frame cloud is produced only on **export**, via `GlobalMapping::export_points()`: iterate all submaps, transform `submap->frame->points[i]` by `submap->T_world_origin`, concatenate into a `PointCloudCPU`.

The map editor indexes points as `(submap_id << 32) | point_id` in `MapCell` structures for spatial queries.

See: `src/glim/mapping/global_mapping.cpp`, `include/glim/viewer/editor/map_cell.hpp`

---

## Moving object removal

**Q: How was moving object removal designed in GLIM?**

A: A **pluggable in-tree interface** (`DynamicRemovalBase`) invoked from `SubMapping::create_submap()` **after** `merge_frames_auto` and **before** point-cap downsampling.

**Why not a pure extension module?** `SubMappingCallbacks::on_new_submap` passes `SubMap::ConstPtr` (read-only). An extension module subscribing to callbacks cannot mutate `submap->frame` in place. Filtering must happen inside `create_submap()` before the submap is finalized.

**First method:** `VoxelHitRatioRemoval` — for each voxel in the merged submap (origin frame), count how many contributing keyframes observe it; remove points in voxels with hit ratio below `dynamic_removal_min_hit_ratio`.

Config (`config_sub_mapping_*.json`):
- `dynamic_removal_method`: `"NONE"` or `"VOXEL_HIT_RATIO"`
- `dynamic_removal_voxel_resolution`, `dynamic_removal_min_hit_ratio`, `dynamic_removal_min_total_frames`, `dynamic_removal_count_mode` (`OBSERVED` or `OCCUPIED`)

New strategies: subclass `DynamicRemovalBase`, register in the factory, select via config.

See: `include/glim/mapping/dynamic_removal/`, `src/glim/mapping/dynamic_removal/`

**Q: Does submap-level removal affect the global map?**

A: Yes, automatically. Global mapping stores submaps; `export_points()` concatenates filtered `submap->frame` clouds. `GlobalMapping::insert_submap` rebuilds voxel maps from the filtered cloud.

---

## GTSAM graph semantics in sub-mapping

**Q: What does `T_world_sensor()` mean when seeding `X(current)`?**

A: `EstimationFrame::T_world_sensor()` dispatches on `frame_id`:

| `frame_id` | Returns | Typical odometry backend |
|------------|---------|------------------------|
| `FrameID::IMU` | `T_world_imu` | `odometry_estimation_imu`, `_cpu` |
| `FrameID::LIDAR` | `T_world_lidar` | `odometry_estimation_ct` |

```cpp
values->insert(X(current), gtsam::Pose3(odom_frame->T_world_sensor().matrix()));
```

This sets the **initial value** (linearization point) for LM — not a hard constraint. Odometry backends must set `frame_id` and the matching pose before marginalization (CT calls `set_T_world_sensor(FrameID::LIDAR, …)` after scan matching; IMU backends update `T_world_imu` from the smoother).

After submap optimization, `create_submap()` writes optimized `X(i)` back via `set_T_world_sensor(odom_frames[i]->frame_id, …)`.

**Important:** The "world" here is the **local odometry frame** (drifting). Global mapping aligns submaps later via `T_world_origin`.

**Q: What happens with different odometry backends?**

| Backend | `X(i)` is | Point cloud frame | With `enable_imu` |
|---------|-----------|-------------------|-------------------|
| IMU odometry | `T_world_imu` | IMU | Works as intended |
| CT odometry | `T_world_lidar` | LiDAR | Warning — ImuFactor/deskewing expect IMU frame |

Between-factor deltas remain valid when both endpoints share the same `frame_id`. IMU-specific machinery breaks if CT frames are used with `enable_imu`.

See: `src/glim/odometry/estimation_frame.cpp`, `src/glim/mapping/sub_mapping.cpp`

**Q: Why does `ImuFactor` use `B(last)` but not `B(current)`?**

A: GTSAM's `ImuFactor` API connects five keys:

```cpp
ImuFactor(X(last), V(last), X(current), V(current), B(last), preintegrated_imu)
```

IMU preintegration over `[t_last, t_current]` assumes **constant bias over that interval**, linearized around the bias at the **start** (`B(last)`). The preintegrated deltas predict how `(X(last), V(last))` should evolve to `(X(current), V(current))`.

`B(current)` **is** in the graph — via a separate **bias random-walk** factor:

```cpp
BetweenFactor<imuBias>(B(last), B(current), zero, noise)
```

So: `ImuFactor` = dynamics using bias at interval start; `BetweenFactor` on bias = slow drift between frames. `B(current)` affects the **next** interval's ImuFactor when it becomes `B(last)`.

**Q: What is the difference between `values->insert()` and `PriorFactor`?**

A: GTSAM separates **state** from **constraints**:

| | `values->insert(V, v)` | `PriorFactor(V, v, noise)` |
|---|---|---|
| Role | Initial guess / linearization point | Soft cost term pulling V toward v |
| Required? | Yes — every optimized key needs a value | Optional regularization |
| Effect | Starting point for Gauss-Newton / LM | Penalizes deviating from odometry estimate |

Example in sub-mapping:

```cpp
values->insert(V(current), odom_frame->v_world_imu);           // initialize
graph->emplace_shared<PriorFactor<...>>(V(current), ..., noise); // soft anchor
```

Both are needed: insert provides `x₀` for linearization; prior adds weak regularization while ImuFactor and other constraints move the estimate.

Same pattern for `X(current)` (insert only, except strong prior on `X(0)`) and `B(current)`.

---

## IMU integration (`IMUIntegration::integrate_imu`)

**Q: What does the trajectory-predicting `integrate_imu()` overload do?**

A: Integrates buffered IMU samples over `[start_time, end_time]` and outputs a pose at each IMU timestamp plus endpoints.

**Inputs:**
- `start_time`, `end_time` — integration window
- `state` — initial `NavState` (pose + velocity) at `start_time`
- `bias` — constant IMU bias for preintegration
- `imu_queue` (member) — buffered `[stamp, acc, gyro]` samples

**Outputs (appended):**
- `pred_times` — timestamps (start, each IMU stamp in range, end)
- `pred_poses` — `T_world_imu` at those times via `preintegrated_imu->predict(state, bias)`

**Return:** cursor into `imu_queue` for `erase_imu_data()`.

**Algorithm:**
1. Reset preintegrator with bias; seed output with pose at `start_time`.
2. For each IMU sample in range: integrate `(a, ω)` over `dt`, predict pose, append.
3. If `end_time` is after the last sample: one more partial step using the last `(a, ω)` held constant.

Used in sub-mapping for IMU-rate trajectory smoothing before scan deskewing.

See: `src/glim/common/imu_integration.cpp`

---

## Utilities — `convert_to_string` and `std::vector<bool>`

**Q: Why does `convert_to_string(default_value)` fail when `T` is `std::vector<bool>`?**

A: `std::vector<bool>` is a specialization: `operator[]` returns a **proxy** (`std::__bit_const_reference`), not a `bool&`.

The vector overload calls `convert_to_string(values[i])` for each element, which hits the generic `fmt::format("{}", value)` path. `{fmt}` cannot format the proxy type → compile error:

```
type_is_unformattable_for<std::__bit_const_reference<std::vector<bool>>, char>
```

**Fix:** Cast to real `bool` in the loop (`static_cast<bool>(values[i])`), or add a dedicated `std::vector<bool>` overload.

Other vector element types (`int`, `double`, …) work because they return real references that `{fmt}` can format.

See: `include/glim/util/convert_to_string.hpp`, `include/glim/util/config_impl.hpp`

---

## Extension modules vs in-tree hooks

**Q: Can moving object removal be done purely as an extension module?**

A: **Not for mutating the fused submap cloud.** Extension modules load via `ExtensionModule::load_module()` and communicate through callback slots (`SubMappingCallbacks`, `GlobalMappingCallbacks`, etc.).

`on_new_submap` delivers `SubMap::ConstPtr` — read-only. The extension cannot replace `submap->frame` after the fact.

Callbacks are still useful for **observation** (logging, visualization, triggering `request_to_optimize`) and for **adding factors** via `on_optimize_submap`. In-tree pluggable stages (like `DynamicRemovalBase`) are needed when the pipeline must **modify** submap data before it is queued.

See: `docs/extend.md`, `docs/extensions.md`, `include/glim/util/extension_module.hpp`

---

## CT odometry (`OdometryEstimationCT`)

### What is `OdometryEstimationCT`?

**Q: What does the CT odometry module do?**

A: `OdometryEstimationCT` is GLIM's LiDAR-only odometry backend. It registers each incoming scan against an incremental voxel target map (iVox) using continuous-time GICP (CT-GICP). Each sweep is modeled with two pose variables — scan start (`X(i)`) and scan end (`Y(i)`) — so motion distortion can be handled during registration. A fixed-lag iSAM2 smoother maintains a sliding window of pose estimates. No IMU is required or used.

See: `include/glim/odometry/odometry_estimation_ct.hpp`, `src/glim/odometry/odometry_estimation_ct.cpp`

### How is CT odometry configured?

**Q: Where do the default parameters come from?**

A: `OdometryEstimationCTParams` loads defaults from `config_odometry.json` (or whichever odometry config is active via `config.json`). Key parameters include:

| Parameter | Purpose |
|-----------|---------|
| `ivox_resolution` | Voxel edge length [m] of the target iVox map |
| `ivox_min_points_dist` | Minimum separation [m] between points in the same iVox cell |
| `ivox_lru_thresh` | LRU horizon — voxels not accessed for this many insertion steps are evicted |
| `max_correspondence_distance` | Outlier rejection threshold [m] for CT-GICP correspondences |
| `location_consistency_inf_scale` | Prior weight tying the new scan start pose to the previous scan end pose |
| `constant_velocity_inf_scale` | Weight for near-zero intra-scan motion (scan begin → scan end) |
| `lm_max_iterations` | Max Levenberg–Marquardt iterations for per-scan pose initialization |
| `smoother_lag` | Fixed-lag smoothing window [sec]; older poses are marginalized |
| `use_isam2_dogleg` | Use Dogleg instead of Gauss-Newton in iSAM2 |
| `isam2_relinearize_skip` / `isam2_relinearize_thresh` | iSAM2 relinearization controls |
| `num_threads` | Threads for covariance estimation and CT-GICP factor evaluation |

The shipped CT config is in `config/config_odometry_ct.json` (e.g. `ivox_lru_thresh: 200`, `smoother_lag: 1.0`).

### What do the GTSAM symbols X and Y mean?

**Q: Why are there two pose variables per frame?**

A: In CT-GICP, `X(i)` is the LiDAR pose at the **start** of sweep `i`, and `Y(i)` is the pose at the **end** of sweep `i`. CT-GICP factors interpolate motion between them to deskew points while minimizing distribution-to-distribution error against the target map. This is the core of continuous-time registration.

CT sets `EstimationFrame::frame_id = LIDAR` and publishes `T_world_lidar` at scan start via `set_T_world_sensor(FrameID::LIDAR, …)` — that is the pose sub-mapping reads through `T_world_sensor()` when CT odometry is selected.

### What happens in `insert_frame()`?

**Q: What is the per-scan processing pipeline?**

A: Each call to `insert_frame()` runs through these stages:

1. **Build estimation frame** — Attach preprocessed points, per-point timestamps, and GICP covariances (normals + 3×3 covariances from local neighborhoods).

2. **First scan** — Fix the world origin with strong priors on `X(0)` and `Y(0)`.

3. **Subsequent scans** — Estimate a constant-velocity twist from prior frames, extrapolate initial pose guesses, then run a local LM optimization with:
   - CT-GICP factor against the iVox target
   - Location-consistency prior (new scan start ≈ previous scan end)
   - Constant-velocity prior (small motion within one sweep)

4. **Deskew** — Replace raw scan points with deskewed points and re-estimate covariances.

5. **Grow target map** — Transform deskewed points into the world frame and insert into `target_ivox`.

6. **Smoother update** — Insert new values/factors into the fixed-lag iSAM2 smoother.

7. **Marginalize** — Frames older than `smoother_lag` are moved to `marginalized_frames` and released from the active window.

8. **Write back** — Update active frame poses from smoothed estimates via `set_T_world_sensor(FrameID::LIDAR, T_world_lidar)`.

9. **Visualization (optional)** — Every 100 frames, publish a downsampled iVox snapshot via callbacks (not used for registration).

### How is CT odometry used in GLIM?

**Q: Where does `OdometryEstimationCT` fit in the full pipeline?**

A: CT odometry acts as the **front-end odometry module**. It is used in two ways:

**1. Dynamically loaded module (main GLIM / ROS pipeline)**

When `config.json` points to `config_odometry_ct.json`, GLIM loads `libodometry_estimation_ct.so` via `OdometryEstimationBase::load_module()`. The instance is wrapped by `AsyncOdometryEstimation`, which calls `insert_frame()` on each preprocessed scan in a background thread and forwards marginalized frames downstream to sub-mapping and global mapping.

**2. Direct use in `pcd_mapper`**

The standalone `pcd_mapper` tool constructs `OdometryEstimationCT` directly, processes a directory of PCD files, and pipes marginalized frames into `SubMapping` → `GlobalMapping` to produce a merged map and trajectory.

See: `pcd_mapper/src/pcd_mapper.cpp`, `pcd_mapper/README.md`

### Does the target iVox ever get reset?

**Q: `insert_frame()` never resets `target_ivox`. Is it reset from outside?**

A: **No.** There is no `clear()`, `reset()`, or `target_ivox.reset(...)` call anywhere after construction. The iVox is created once in the constructor and lives for the entire lifetime of the `OdometryEstimationCT` instance. The only way to get a fresh map is to create a new estimator object.

The same pattern applies to `OdometryEstimationCPU` — the iVox is created once and only grows via `insert()`, with LRU eviction rather than periodic rebuilds.

### Does the target map grow without bound?

**Q: Does CT odometry accumulate infinite point clouds?**

A: **No, but drift does accumulate.** The map size is bounded by LRU eviction, not by explicit resets.

**LRU eviction (`ivox_lru_thresh`)**

At construction, the iVox is configured with:

```cpp
target_ivox->set_lru_horizon(params.ivox_lru_thresh);
```

In gtsam_points, `lru_horizon` means: *voxels that have not been accessed for this many insertion steps are deleted*. Default in code is 30; `config/config_odometry_ct.json` sets **200**.

As the sensor moves, voxels left behind on the trajectory stop being hit during correspondence search and are eventually evicted. The target map is therefore a **sliding local map** (similar to Faster-LIO-style iVox), not an ever-growing global cloud.

**Full-scan insertion (no downsampling)**

Unlike `OdometryEstimationCPU`, which random-downsamples points before insertion (after frame 5), CT inserts the **full deskewed scan** every frame. Within the LRU window the map can be dense, but it is still bounded by eviction.

### Does odometry drift accumulate?

**Q: With no map reset, does registration error compound over long sequences?**

A: **Yes — that is expected for incremental scan-to-model odometry.** Several design choices contribute:

1. **World-frame target map** — Points are stored in world coordinates using the current pose estimate. CT does not use CPU odometry's trick of keeping the target in a sensor-attached frame that moves with the robot.

2. **Insert-before-smooth ordering** — Points are inserted using the LM pose *before* the fixed-lag smoother refines poses. When the smoother adjusts recent poses, **already-inserted iVox points are not re-transformed**, creating a map/pose inconsistency within the local window.

3. **Short smoother window** — With `smoother_lag: 1.0` s in the CT config, the pose graph only optimizes a ~1 s window. That smooths short-term motion but does not correct long-range drift.

4. **Downstream correction** — Marginalized odometry frames are fed to sub-mapping and global mapping (loop closure, bundle adjustment). Long-range error is meant to be corrected there, not in the odometry iVox.

### What gets reset vs. what does not?

**Q: Which internal state is bounded or cleared during a long run?**

| Component | Explicit reset? | Bounding mechanism |
|-----------|-----------------|-------------------|
| `target_ivox` | No | LRU eviction (`ivox_lru_thresh`) |
| Pose graph (`smoother`) | Partial | Fixed-lag marginalization (`smoother_lag`) |
| `frames` vector | Partial | Old slots nulled after marginalization |
| Global map | Separate system | Built in sub-mapping / global mapping |

### Is there a `get_remaining_frames()` gap?

**Q: Are frames at end-of-sequence lost for CT odometry?**

A: **Potentially yes.** `OdometryEstimationCT` does **not** override `get_remaining_frames()`. The base class returns an empty vector:

```cpp
virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() {
  return std::vector<EstimationFrame::ConstPtr>();
}
```

By contrast, `OdometryEstimationIMU` implements this method and flushes frames still inside the smoother window when the sequence ends. Both `AsyncOdometryEstimation` and `pcd_mapper` call `get_remaining_frames()` at shutdown — so the last ~`smoother_lag` seconds of CT odometry frames may never reach sub-mapping unless `get_remaining_frames()` is added to the CT class.

### How does CT odometry compare to CPU odometry?

**Q: What are the main architectural differences?**

| Aspect | CT (`OdometryEstimationCT`) | CPU (`OdometryEstimationCPU`) |
|--------|----------------------------|-------------------------------|
| IMU | Not used | Semi-tightly coupled LiDAR-IMU |
| Registration | CT-GICP (two poses per sweep) | GICP or VGICP (single pose) |
| Target frame | World coordinates | Sensor/target-attached frame (`T_target_imu`) |
| Target downsampling | None (full scan inserted) | Random downsampling after frame 5 |
| Map eviction | LRU (`ivox_lru_thresh`) | LRU (`lru_thresh`) |
| Explicit map reset | No | No |
| `get_remaining_frames()` | Not implemented (base returns empty) | Implemented |
| `EstimationFrame::frame_id` | `LIDAR` | `IMU` |

---

## Related files

| File | Role |
|------|------|
| `include/glim/preprocess/cloud_preprocessor.hpp` | Point cloud preprocessing |
| `include/glim/preprocess/preprocessed_frame.hpp` | Preprocessed scan data structure |
| `include/glim/odometry/odometry_estimation_base.hpp` | Odometry base class and module loading |
| `include/glim/odometry/odometry_estimation_imu.hpp` | IMU-coupled odometry base (GPU/CPU) |
| `include/glim/odometry/odometry_estimation_gpu.hpp` | GPU VGICP odometry |
| `include/glim/odometry/odometry_estimation_cpu.hpp` | CPU GICP/VGICP odometry |
| `include/glim/odometry/odometry_estimation_ct.hpp` | CT-GICP LiDAR-only odometry |
| `include/glim/odometry/estimation_frame.hpp` | Central frame data object |
| `include/glim/odometry/async_odometry_estimation.hpp` | Async odometry wrapper |
| `include/glim/mapping/sub_mapping.hpp` | Sub-mapping class, params, data-flow docs |
| `src/glim/mapping/sub_mapping.cpp` | Sub-mapping implementation |
| `include/glim/mapping/sub_map.hpp` | SubMap structure (`frame`, poses, voxelmaps) |
| `include/glim/mapping/global_mapping.hpp` | Global mapping with implicit loop closure |
| `include/glim/mapping/global_mapping_pose_graph.hpp` | Global mapping with explicit loop detection |
| `src/glim/mapping/global_mapping.cpp` | Global map, `export_points()` |
| `include/glim/mapping/dynamic_removal/` | Moving object removal strategies |
| `src/glim/common/imu_integration.cpp` | IMU preintegration utilities |
| `include/glim/util/convert_to_string.hpp` | Config value stringification |
| `docs/extend.md` | Extension modules, callback slots, GTSAM variable reference |
| `docs/extensions.md` | glim_ext visual/loop-closure modules |
| `docs/parameters.md` | Official parameter documentation |
| `pcd_mapper/src/pcd_mapper.cpp` | Standalone batch mapper |
| `config/config.json` | Top-level config (selects odometry/mapping configs) |
| `config/config_preprocess.json` | Preprocessing parameters |
| `config/config_sensors.json` | Sensor extrinsics and shutter mode |
| `config/config_odometry_{gpu,cpu,ct}.json` | Odometry backend parameters |
| `config/config_sub_mapping_*.json` | Sub-mapping parameters |
| `config/config_global_mapping_*.json` | Global mapping parameters |
