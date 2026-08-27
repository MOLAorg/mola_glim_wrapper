/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
*/

/**
 * @file   Types.hpp
 * @brief  Plain-data API boundary of glim_core: no MOLA, no MRPT, no GTSAM,
 *         no GLIM headers. Only Eigen and the standard library.
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace glim_core
{

/** One LiDAR point, with its acquisition time as an offset (seconds) from the
 * beginning of the scan. Offsets are always >= 0. */
struct LidarPoint
{
  float x = 0;
  float y = 0;
  float z = 0;
  float intensity = 0;
  double t_offset_sec = 0;
};

using LidarPointVector = std::vector<LidarPoint>;

/** One IMU sample. `t` is in seconds on the same clock as the LiDAR scan
 * timestamps handed to GlimCore::processScan(). */
struct ImuSample
{
  double t = 0;
  Eigen::Vector3d acc{0, 0, 0};
  Eigen::Vector3d gyro{0, 0, 0};
};

/** Output point, in whatever frame the accessor documents. */
struct XYZIPoint
{
  float x = 0;
  float y = 0;
  float z = 0;
  float intensity = 0;
};

using XYZIPointVector = std::vector<XYZIPoint>;

/** Mirrors the subset of GLIM's own JSON schema that this wrapper exposes.
 *
 * Every field maps one-to-one onto an upstream `config_preprocess.json` /
 * `config_odometry_cpu.json` / `config_sensors.json` key, and the defaults
 * below are upstream's own defaults. `glim_config_path`, when non-empty,
 * loads those JSON files first, so this struct then acts as an override
 * layer on top of them.
 */
struct Config
{
  // ---- extrinsics (config_sensors.json) ----

  /** Pose of the IMU frame expressed in the LiDAR frame, i.e. GLIM's own
   * `T_lidar_imu`. Verified against EstimationFrame::set_T_world_sensor(),
   * which computes `T_world_imu = T_world_lidar * T_lidar_imu`. */
  Eigen::Isometry3d T_lidar_imu = Eigen::Isometry3d::Identity();

  double imu_bias_noise = 1e-5;
  double imu_acc_noise = 0.05;
  double imu_gyro_noise = 0.02;
  double imu_int_noise = 0.001;

  // ---- preprocessing (config_preprocess.json) ----

  double distance_near_thresh = 0.5;
  double distance_far_thresh = 100.0;
  bool global_shutter = false;
  bool use_random_grid_downsampling = true;
  double downsample_resolution = 1.0;
  int downsample_target = 10000;
  double downsample_rate = 0.1;
  bool enable_outlier_removal = false;
  int outlier_removal_k = 10;
  double outlier_std_mul_factor = 1.0;
  int k_correspondences = 10;

  // ---- odometry (config_odometry_cpu.json) ----

  /** "GICP" (iVox target) or "VGICP" (voxelmap target). */
  std::string registration_type = "GICP";
  int max_iterations = 8;
  int lru_thresh = 100;
  double target_downsampling_rate = 0.1;

  double ivox_resolution = 1.0;
  double ivox_min_dist = 0.1;

  double vgicp_resolution = 0.5;
  int vgicp_voxelmap_levels = 1;
  double vgicp_voxelmap_scaling_factor = 2.0;

  bool fix_imu_bias = false;
  /** "LOOSE" or "NAIVE": how the initial attitude/velocity is bootstrapped. */
  std::string initialization_mode = "LOOSE";
  /** Seconds of LOOSE bootstrapping before the first pose is produced. Every
   * scan in this window is consumed and reported as invalid, so it is also
   * the length of trajectory missing from the head of every run. */
  double initialization_window_size = 3.0;
  double init_pose_damping_scale = 1e10;

  double smoother_lag = 5.0;
  bool use_isam2_dogleg = false;
  int isam2_relinearize_skip = 1;
  double isam2_relinearize_thresh = 0.1;

  bool validate_imu = true;
  /** Off by default here (upstream ships it on): the per-IMU-sample
   * trajectory is only consumed by GLIM's own viewer and mapping stages,
   * neither of which this wrapper builds. */
  bool save_imu_rate_trajectory = false;

  /** Threads used by both the preprocessor and the per-factor parallelism. */
  int num_threads = 2;

  // ---- wrapper-level ----

  /** Directory holding GLIM's own `config.json` and friends; required.
   *
   * GLIM reads several parameters straight from those JSON files with no
   * programmatic path in (the crop-box filter, the marginal-covariance
   * switch). Every field above is instead applied
   * ON TOP of whatever the JSON says, so for those keys the pipeline YAML
   * is the single source of truth and the JSON value is inert. The defaults
   * above are upstream's own JSON defaults, so an un-set key changes
   * nothing. */
  std::string glim_config_path;

  /** spdlog level applied to GLIM's own loggers: "off", "error", "warn",
   * "info", "debug", "trace".
   *
   * "error" by default rather than upstream's "info": GLIM emits a
   * per-scan warning whenever an IMU stream is sparser than it expects,
   * which on a synthetic-IMU dataset is every single scan and buries the
   * run's real output under tens of megabytes of text. Raise it to "warn"
   * to diagnose a run. */
  std::string log_level = "error";

  /** If true, keep the deskewed scan in OdometryOutput (costs one copy per
   * scan); the offline CLI leaves it off, the GUI turns it on. */
  bool export_deskewed_points = false;
};

/** One odometry solution, produced by GlimCore::processScan(). */
struct OdometryOutput
{
  /** False while GLIM is still bootstrapping its initial state: those first
   * scans legitimately produce no pose at all. */
  bool valid = false;
  double timestamp = 0;

  /** T_world_lidar, the pose GLIM reports for the LiDAR frame. */
  Eigen::Isometry3d pose_lidar = Eigen::Isometry3d::Identity();
  /** T_world_imu, the filter's own state. */
  Eigen::Isometry3d pose_imu = Eigen::Isometry3d::Identity();

  Eigen::Vector3d vel{0, 0, 0};
  Eigen::Vector3d bias_acc{0, 0, 0};
  Eigen::Vector3d bias_gyro{0, 0, 0};

  /** Gravity as seen in the IMU body frame, i.e. the world gravity rotated
   * by the estimated attitude. GLIM does not estimate gravity as a state,
   * so this is a readout of the attitude rather than of a filter state. */
  Eigen::Vector3d gravity_body{0, 0, 0};

  std::size_t num_points_raw = 0;
  std::size_t num_points_preprocessed = 0;

  /** Deskewed scan in the IMU body frame; only set when
   * Config::export_deskewed_points is true. */
  std::shared_ptr<const XYZIPointVector> deskewed_points_body;
};

}  // namespace glim_core
