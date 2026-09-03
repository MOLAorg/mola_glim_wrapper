/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: BSD-3-Clause
 See LICENSE for full license information.
 See THIRD_PARTY_NOTICES.md: wraps the MIT-licensed GLIM algorithm
 (third_party/glim), copyright Kenji Koide.
*/

/**
 * @file   GlimOdometry.h
 * @brief  MOLA front-end module wrapping GLIM's LiDAR-inertial odometry
 */
#pragma once

// Only the light, GTSAM-free plain-data header: this keeps consumers of this
// public header from having to add glim/gtsam_points include dirs to their
// own build. The full glim_core/GlimCore.h is included only from the .cpp.
#include <mola_kernel/GuiWidgetDescription.h>
#include <mola_kernel/interfaces/FrontEndBase.h>
#include <mola_kernel/interfaces/LocalizationSourceBase.h>
#include <mola_kernel/interfaces/MapSourceBase.h>
#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/viz/CSetOfLines.h>

#include <atomic>
#include <glim_core/Types.hpp>
#include <memory>
#include <mutex>
#include <regex>

namespace glim_core
{
class GlimCore;
}

namespace mola
{
/** MOLA online front-end wrapping GLIM's LiDAR-inertial odometry
 * (see `glim_core`, in `third_party/`).
 *
 * Consumes `mrpt::obs::CObservationPointCloud` (LiDAR) and
 * `mrpt::obs::CObservationIMU` (IMU) observations and publishes live
 * localization + map updates via `LocalizationSourceBase`/`MapSourceBase`.
 *
 * **Only GLIM's odometry stage runs here.** Upstream GLIM is a full SLAM
 * system whose sub-mapping and global-mapping stages add loop closure and
 * global optimization; instantiating those would make its numbers
 * incomparable with the pure-odometry wrappers this package sits next to.
 *
 * LiDAR scans are processed on a single-thread worker pool with the
 * `POLICY_DROP_OLD` policy: fine for live use, but NOT loss-free. The
 * offline CLI (`mola-glim-cli`) achieves the loss-free guarantee by feeding
 * one observation at a time and busy-waiting on `isBusy()` before advancing.
 *
 * \ingroup mola_glim_wrapper_grp
 */
class GlimOdometry : public FrontEndBase, public LocalizationSourceBase, public MapSourceBase
{
  DEFINE_MRPT_OBJECT(GlimOdometry, mola)

public:
  GlimOdometry();
  ~GlimOdometry() override;

  // ExecutableBase
  void spinOnce() override;
  void onQuit() override;

  // RawDataConsumer
  void onNewObservation(const mrpt::obs::CObservation::ConstPtr & o) override;

  /** True while the LiDAR worker still has a scan queued or running.
   * Polled by `mola-glim-cli` to guarantee no scan is ever dropped. */
  bool isBusy() const;

  /** Trajectory accumulated so far, in the output frame (`base_link`, or the
   * LiDAR frame when `baselink2lidar_pose_str` is left at identity). */
  mrpt::poses::CPose3DInterpolator estimatedTrajectory() const;

  /** The pose this module reports, from the one GLIM estimates.
   *
   * GLIM already reports the LiDAR pose in the world frame
   * (`EstimationFrame::T_world_lidar`), so unlike the Fast-LIO2 wrapper only
   * one extrinsic is involved, and it still has to be INVERTED:
   *
   *   T_w_base = T_w_lidar . T_base_lidar^-1
   *
   * @param worldFromLidar      what GLIM estimates, `T_w_lidar`.
   * @param lidarPoseInBaseLink `T_base_lidar`, YAML `baselink2lidar_pose_str`.
   */
  static mrpt::poses::CPose3D worldFromBaseLink(
    const mrpt::poses::CPose3D & worldFromLidar, const mrpt::poses::CPose3D & lidarPoseInBaseLink);

protected:
  void initialize_frontend(const Yaml & cfg) override;

private:
  void onLidarObservation(const mrpt::obs::CObservation::ConstPtr & o);
  void onImuObservation(const mrpt::obs::CObservation::ConstPtr & o);
  void processLidarScan(mrpt::obs::CObservation::ConstPtr o);
  void publishOutput(const glim_core::OdometryOutput & out);

  /// Drains `worker_lidar_` and, if requested, writes the accumulated
  /// trajectory to disk. Called from both `onQuit()` and the destructor;
  /// idempotent via `shutdown_cleanup_done_`.
  void shutdownCleanup();

  // --- GUI / 3D scene visualization (mola_viz / mola_viz_imgui) ---
  void updateVisualization(
    const mrpt::poses::CPose3D & outputPose, const glim_core::OdometryOutput & out);
  void updateVisualizationPath(const mrpt::poses::CPose3D & outputPose);
  void updateVisualizationScan(const glim_core::OdometryOutput & out);
  void internalBuildGUI();
  void updateVisualizationTextLabels(const glim_core::OdometryOutput & out);

  std::regex lidar_sensor_label_regex_{"lidar"};
  std::regex imu_sensor_label_regex_{"imu"};

  /// Pose of the LiDAR frame wrt `base_link`, i.e. `T_base_lidar`. Identity
  /// when the LiDAR itself is the reference frame.
  mrpt::poses::CPose3D baselink_pose_of_lidar_;

  /// Name of the frame the reported pose lands in.
  std::string output_frame_id_ = "lidar";

  /// Used only as a fallback when an incoming point cloud has no per-point
  /// timestamp channel.
  double fallback_scan_period_ = 0.1;

  /// Near-range cut applied before handing the scan to GLIM, in addition to
  /// GLIM's own `distance_near_thresh`; kept for parity with the other
  /// wrappers' `blind` parameter, and 0 (disabled) by default.
  double blind_ = 0.0;

  glim_core::Config core_cfg_;
  std::unique_ptr<glim_core::GlimCore> core_;

  /// GLIM's estimator is not internally synchronized, and IMU samples arrive
  /// on the caller's thread while scans run on the worker below.
  std::mutex core_mtx_;

  mrpt::WorkerThreadsPool worker_lidar_{1, mrpt::WorkerThreadsPool::POLICY_DROP_OLD, "glim_lidar"};
  std::atomic_int tasks_in_flight_{0};

  mutable std::mutex trajectory_mtx_;
  mrpt::poses::CPose3DInterpolator trajectory_;

  std::atomic_bool shutdown_cleanup_done_{false};

  /// So the per-point-timestamp source is reported once, not per scan.
  std::atomic_bool timestamp_source_logged_{false};

  /// Mirrors `mola::LidarOdometry`'s `estimated_trajectory` pipeline block,
  /// so `mola-cli` (online mode) can export a trajectory the same way the
  /// offline CLI does.
  bool save_trajectory_to_file_ = false;
  std::string trajectory_output_file_;

  struct VisualizationParams
  {
    std::atomic_bool show_trajectory{true};
    std::atomic_bool show_scan{true};
    std::atomic_bool camera_follows_vehicle{true};

    float current_pose_corner_size = 1.0f;
    float scan_point_size = 2.0f;

    /// How many processed scans to wait between point-cloud viz refreshes.
    int map_update_decimation = 2;
  } visualization_params_;

  /// Only touched from the (single-threaded) LiDAR worker.
  mrpt::viz::CSetOfLines::Ptr gl_estimated_path_;
  int map_viz_update_counter_ = 0;
  bool gui_created_ = false;

  struct Gui
  {
    mola::gui::LiveString::Ptr lbStatus;
  } gui_;
};

}  // namespace mola
