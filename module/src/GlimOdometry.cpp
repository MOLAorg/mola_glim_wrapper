/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 See THIRD_PARTY_NOTICES.md: wraps the MIT-licensed GLIM algorithm
 (third_party/glim), copyright Kenji Koide.
*/

/**
 * @file   GlimOdometry.cpp
 * @brief  MOLA front-end module wrapping GLIM's LiDAR-inertial odometry
 */
#include <glim_core/GlimCore.h>
#include <mola_glim_wrapper/GlimOdometry.h>
#include <mola_kernel/interfaces/VizInterface.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>

#include <sstream>

#include "GlimOdometry_Adapters.h"

namespace mola
{

IMPLEMENTS_MRPT_OBJECT(GlimOdometry, FrontEndBase, mola)

namespace
{
/// Parses "x y z yaw_deg pitch_deg roll_deg".
mrpt::poses::CPose3D parsePose6D(const std::string & s)
{
  std::istringstream iss(s);
  double x, y, z, yaw, pitch, roll;
  if (!(iss >> x >> y >> z >> yaw >> pitch >> roll)) {
    THROW_EXCEPTION_FMT(
      "Could not parse pose string '%s' (expected: 'x y z yaw_deg pitch_deg roll_deg')", s.c_str());
  }
  return {x, y, z, mrpt::DEG2RAD(yaw), mrpt::DEG2RAD(pitch), mrpt::DEG2RAD(roll)};
}

mrpt::poses::CPose3D isometryToPose(const Eigen::Isometry3d & iso)
{
  mrpt::math::CMatrixDouble44 m;
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      m(r, c) = iso.matrix()(r, c);
    }
  }
  return mrpt::poses::CPose3D(m);
}

Eigen::Isometry3d poseToIsometry(const mrpt::poses::CPose3D & p)
{
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = p.getRotationMatrix().asEigen();
  iso.translation() = Eigen::Vector3d(p.x(), p.y(), p.z());
  return iso;
}
}  // namespace

GlimOdometry::GlimOdometry() = default;
GlimOdometry::~GlimOdometry() { shutdownCleanup(); }

void GlimOdometry::onQuit() { shutdownCleanup(); }

void GlimOdometry::shutdownCleanup()
{
  if (shutdown_cleanup_done_.exchange(true)) {
    return;
  }

  worker_lidar_.clear();

  if (save_trajectory_to_file_ && !trajectory_output_file_.empty()) {
    auto lck = mrpt::lockHelper(trajectory_mtx_);
    MRPT_LOG_INFO_STREAM(
      "Saving estimated trajectory with " << trajectory_.size() << " keyframes to file '"
                                          << trajectory_output_file_ << "' in TUM format...");
    trajectory_.saveToTextFile_TUM(trajectory_output_file_);
    MRPT_LOG_INFO("Final trajectory saved.");
  }
}

void GlimOdometry::initialize_frontend(const Yaml & cfg)
{
  MRPT_TRY_START

  std::string lidar_sensor_label = "lidar";
  std::string imu_sensor_label = "imu";
  YAML_LOAD_OPT(lidar_sensor_label, std::string);
  YAML_LOAD_OPT(imu_sensor_label, std::string);
  lidar_sensor_label_regex_ = std::regex(lidar_sensor_label);
  imu_sensor_label_regex_ = std::regex(imu_sensor_label);

  // NOTE: YAML_LOAD_OPT takes the YAML key from the VARIABLE name, so these
  // locals must be named exactly as the keys documented in the pipeline files.
  std::string imu_pose_in_lidar = "0 0 0 0 0 0";
  YAML_LOAD_OPT(imu_pose_in_lidar, std::string);

  // Pose of the LiDAR frame wrt base_link (T_base_lidar): same key name and
  // same convention as the DLIO and Fast-LIO2 wrappers', so a dataset's
  // pipeline files carry the same number across methods.
  std::string baselink2lidar_pose_str = "0 0 0 0 0 0";
  YAML_LOAD_OPT(baselink2lidar_pose_str, std::string);
  baselink_pose_of_lidar_ = parsePose6D(baselink2lidar_pose_str);

  YAML_LOAD_OPT(fallback_scan_period_, double);
  YAML_LOAD_OPT(blind_, double);

  YAML_LOAD_OPT3(core_cfg_, glim_config_path, std::string);
  YAML_LOAD_OPT3(core_cfg_, log_level, std::string);

  YAML_LOAD_OPT3(core_cfg_, imu_bias_noise, double);
  YAML_LOAD_OPT3(core_cfg_, imu_acc_noise, double);
  YAML_LOAD_OPT3(core_cfg_, imu_gyro_noise, double);
  YAML_LOAD_OPT3(core_cfg_, imu_int_noise, double);

  YAML_LOAD_OPT3(core_cfg_, distance_near_thresh, double);
  YAML_LOAD_OPT3(core_cfg_, distance_far_thresh, double);
  YAML_LOAD_OPT3(core_cfg_, global_shutter, bool);
  YAML_LOAD_OPT3(core_cfg_, use_random_grid_downsampling, bool);
  YAML_LOAD_OPT3(core_cfg_, downsample_resolution, double);
  YAML_LOAD_OPT3(core_cfg_, downsample_target, int);
  YAML_LOAD_OPT3(core_cfg_, downsample_rate, double);
  YAML_LOAD_OPT3(core_cfg_, enable_outlier_removal, bool);
  YAML_LOAD_OPT3(core_cfg_, outlier_removal_k, int);
  YAML_LOAD_OPT3(core_cfg_, outlier_std_mul_factor, double);
  YAML_LOAD_OPT3(core_cfg_, k_correspondences, int);

  YAML_LOAD_OPT3(core_cfg_, registration_type, std::string);
  YAML_LOAD_OPT3(core_cfg_, max_iterations, int);
  YAML_LOAD_OPT3(core_cfg_, lru_thresh, int);
  YAML_LOAD_OPT3(core_cfg_, target_downsampling_rate, double);
  YAML_LOAD_OPT3(core_cfg_, ivox_resolution, double);
  YAML_LOAD_OPT3(core_cfg_, ivox_min_dist, double);
  YAML_LOAD_OPT3(core_cfg_, vgicp_resolution, double);
  YAML_LOAD_OPT3(core_cfg_, vgicp_voxelmap_levels, int);
  YAML_LOAD_OPT3(core_cfg_, vgicp_voxelmap_scaling_factor, double);

  YAML_LOAD_OPT3(core_cfg_, fix_imu_bias, bool);
  YAML_LOAD_OPT3(core_cfg_, initialization_mode, std::string);
  YAML_LOAD_OPT3(core_cfg_, initialization_window_size, double);
  YAML_LOAD_OPT3(core_cfg_, init_pose_damping_scale, double);
  YAML_LOAD_OPT3(core_cfg_, smoother_lag, double);
  YAML_LOAD_OPT3(core_cfg_, use_isam2_dogleg, bool);
  YAML_LOAD_OPT3(core_cfg_, isam2_relinearize_skip, int);
  YAML_LOAD_OPT3(core_cfg_, isam2_relinearize_thresh, double);
  YAML_LOAD_OPT3(core_cfg_, validate_imu, bool);
  YAML_LOAD_OPT3(core_cfg_, save_imu_rate_trajectory, bool);
  YAML_LOAD_OPT3(core_cfg_, num_threads, int);

  YAML_LOAD_OPT3(visualization_params_, show_trajectory, bool);
  YAML_LOAD_OPT3(visualization_params_, show_scan, bool);
  YAML_LOAD_OPT3(visualization_params_, camera_follows_vehicle, bool);
  YAML_LOAD_OPT3(visualization_params_, current_pose_corner_size, float);
  YAML_LOAD_OPT3(visualization_params_, scan_point_size, float);
  YAML_LOAD_OPT3(visualization_params_, map_update_decimation, int);

  YAML_LOAD_MEMBER_OPT(save_trajectory_to_file, bool);
  YAML_LOAD_MEMBER_OPT(trajectory_output_file, std::string);

  // GLIM's own `T_lidar_imu` is the IMU pose expressed in the LiDAR frame,
  // i.e. the very convention `imu_pose_in_lidar` states (upstream's own
  // config_sensors.json documents `T_world_imu = T_world_lidar *
  // T_lidar_imu`, and EstimationFrame::set_T_world_sensor() implements
  // exactly that). So this one goes in uninverted -- unlike the Fast-LIO2
  // wrapper's extrinsic, which is the inverse.
  core_cfg_.T_lidar_imu = poseToIsometry(parsePose6D(imu_pose_in_lidar));

  if (core_cfg_.glim_config_path.empty()) {
    core_cfg_.glim_config_path = MOLA_GLIM_DEFAULT_CONFIG_DIR;
  }

  core_cfg_.export_deskewed_points = visualization_params_.show_scan;

  output_frame_id_ =
    baselink_pose_of_lidar_ == mrpt::poses::CPose3D::Identity() ? "lidar" : "base_link";

  core_ = std::make_unique<glim_core::GlimCore>(core_cfg_);

  MRPT_LOG_INFO_STREAM(
    "GlimOdometry initialized. lidar_sensor_label='"
    << lidar_sensor_label << "' imu_sensor_label='" << imu_sensor_label << "' registration='"
    << core_cfg_.registration_type << "' num_threads=" << core_cfg_.num_threads
    << (core_cfg_.num_threads == 1 ? " (deterministic)" : " (NOT deterministic)")
    << " config_path='" << core_cfg_.glim_config_path << "'");

  MRPT_TRY_END
}

void GlimOdometry::onNewObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  MRPT_TRY_START
  ASSERT_(o);

  if (std::regex_match(o->sensorLabel, imu_sensor_label_regex_)) {
    onImuObservation(o);
  } else if (std::regex_match(o->sensorLabel, lidar_sensor_label_regex_)) {
    onLidarObservation(o);
  }
  MRPT_TRY_END
}

void GlimOdometry::onImuObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  auto imu = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(o);
  if (!imu || !core_) {
    return;
  }

  // Cheap: run synchronously. IMU is the deskew and propagation clock, it
  // must never be dropped, not even in live/online mode.
  auto lck = mrpt::lockHelper(core_mtx_);
  core_->addImuSample(toGlimImu(*imu));
}

void GlimOdometry::onLidarObservation(const mrpt::obs::CObservation::ConstPtr & o)
{
  if (!core_) {
    return;
  }

  tasks_in_flight_++;
  auto fut = worker_lidar_.enqueue(&GlimOdometry::processLidarScan, this, o);
  (void)fut;
}

void GlimOdometry::processLidarScan(mrpt::obs::CObservation::ConstPtr o)
{
  MRPT_TRY_START
  auto pc = std::dynamic_pointer_cast<const mrpt::obs::CObservationPointCloud>(o);
  if (pc && core_) {
    double t_beg = 0;
    double t_end = 0;
    bool perPointTimes = false;
    const auto cloud =
      toGlimCloud(*pc, fallback_scan_period_, t_beg, t_end, blind_, &perPointTimes);

    // Which of the two per-point clocks is in use decides whether GLIM's
    // deskewing sees the real intra-scan geometry or a synthesized one, so
    // report it once rather than leaving it to be inferred.
    if (!timestamp_source_logged_.exchange(true)) {
      MRPT_LOG_INFO_STREAM(
        "Per-point scan times: "
        << (perPointTimes ? "taken from the cloud's own timestamp channel"
                          : "SYNTHESIZED from azimuth (no timestamp channel in this cloud)")
        << ", scan span " << (t_end - t_beg) << " s, " << cloud.size() << " points");
    }

    glim_core::OdometryOutput out;
    bool ok = false;
    {
      auto lck = mrpt::lockHelper(core_mtx_);
      ok = core_->processScan(cloud, t_beg, t_end, out);
    }
    if (ok && out.valid) {
      publishOutput(out);
    }
  }
  tasks_in_flight_--;
  MRPT_TRY_END
}

void GlimOdometry::publishOutput(const glim_core::OdometryOutput & out)
{
  // GLIM reports the LiDAR pose in the world frame; report `base_link`
  // instead, which is the frame the rest of MOLA -- and every dataset's
  // ground truth -- uses.
  const mrpt::poses::CPose3D lidarPose = isometryToPose(out.pose_lidar);
  const mrpt::poses::CPose3D outputPose = worldFromBaseLink(lidarPose, baselink_pose_of_lidar_);

  const auto timestamp = mrpt::Clock::fromDouble(out.timestamp);

  {
    auto lck = mrpt::lockHelper(trajectory_mtx_);
    trajectory_.insert(timestamp, outputPose);
  }

  if (anyUpdateLocalizationSubscriber()) {
    LocalizationUpdate lu;
    lu.timestamp = timestamp;
    lu.reference_frame = "map";
    lu.child_frame = output_frame_id_;
    lu.method = "glim";
    lu.pose = outputPose.asTPose();
    advertiseUpdatedLocalization(lu);
  }

  if (anyUpdateMapSubscriber() && out.deskewed_points_body) {
    // GLIM's odometry target is an internal iVox/voxelmap with no exported
    // accessor, so what is published is the deskewed scan placed in the
    // world frame: enough for a consumer to see the map being built,
    // without reaching into upstream internals.
    auto pts = toMrptPointsMap(*out.deskewed_points_body);
    pts->changeCoordinatesReference(isometryToPose(out.pose_imu));

    MapUpdate mu;
    mu.timestamp = timestamp;
    mu.reference_frame = "map";
    mu.method = "glim";
    mu.map_name = "deskewed_scan";
    mu.map = pts;
    mu.keep_last_one_only = true;
    advertiseUpdatedMap(mu);
  }

  updateVisualization(outputPose, out);
}

mrpt::poses::CPose3D GlimOdometry::worldFromBaseLink(
  const mrpt::poses::CPose3D & worldFromLidar, const mrpt::poses::CPose3D & lidarPoseInBaseLink)
{
  mrpt::poses::CPose3D baseLinkPoseInLidar = lidarPoseInBaseLink;
  baseLinkPoseInLidar.inverse();
  return worldFromLidar + baseLinkPoseInLidar;
}

void GlimOdometry::updateVisualization(
  const mrpt::poses::CPose3D & outputPose, const glim_core::OdometryOutput & out)
{
  if (!visualizer_) {
    return;
  }

  if (visualization_params_.current_pose_corner_size > 0) {
    auto glVehicle = mrpt::opengl::CSetOfObjects::Create();
    glVehicle->insert(
      mrpt::opengl::stock_objects::CornerXYZ(visualization_params_.current_pose_corner_size));
    glVehicle->setPose(outputPose);
    visualizer_->update_3d_object("glim/vehicle", glVehicle);
  }

  updateVisualizationPath(outputPose);
  updateVisualizationScan(out);

  if (visualization_params_.camera_follows_vehicle) {
    const mrpt::math::TPoint3Df lookAt(outputPose.x(), outputPose.y(), outputPose.z());
    visualizer_->update_viewport_look_at(lookAt);
  }

  if (!gui_created_) {
    internalBuildGUI();
    gui_created_ = true;
  }
  updateVisualizationTextLabels(out);
}

void GlimOdometry::updateVisualizationPath(const mrpt::poses::CPose3D & outputPose)
{
  if (!visualization_params_.show_trajectory) {
    visualizer_->update_3d_object("glim/path", mrpt::opengl::CSetOfObjects::Create());
    return;
  }

  if (!gl_estimated_path_) {
    gl_estimated_path_ = mrpt::opengl::CSetOfLines::Create();
    gl_estimated_path_->setColor_u8(0x20, 0xc0, 0x40, 0xff);
  }

  const auto t = outputPose.translation();
  if (gl_estimated_path_->empty()) {
    gl_estimated_path_->appendLine(t, t);
  } else {
    gl_estimated_path_->appendLineStrip(t);
  }

  // Hand a fresh copy to the GUI thread so this (worker-thread-owned) buffer
  // can keep growing without racing with the GUI thread's read.
  auto pathGrp = mrpt::opengl::CSetOfObjects::Create();
  pathGrp->insert(mrpt::opengl::CSetOfLines::Create(*gl_estimated_path_));
  visualizer_->update_3d_object("glim/path", pathGrp);
}

void GlimOdometry::updateVisualizationScan(const glim_core::OdometryOutput & out)
{
  if (!visualization_params_.show_scan) {
    visualizer_->update_3d_object("glim/scan", mrpt::opengl::CSetOfObjects::Create());
    return;
  }

  if (++map_viz_update_counter_ < visualization_params_.map_update_decimation) {
    return;
  }
  map_viz_update_counter_ = 0;

  if (!out.deskewed_points_body || out.deskewed_points_body->empty()) {
    return;
  }

  auto pointsMap = toMrptPointsMap(*out.deskewed_points_body);
  pointsMap->changeCoordinatesReference(isometryToPose(out.pose_imu));

  auto glCloud = mrpt::opengl::CPointCloudColoured::Create();
  glCloud->loadFromPointsMap(pointsMap.get());
  glCloud->setPointSize(visualization_params_.scan_point_size);

  const auto bbox = pointsMap->boundingBox();
  glCloud->recolorizeByCoordinate(bbox.min.z, bbox.max.z, 2 /*Z*/, mrpt::img::TColormap::cmJET);

  auto grp = mrpt::opengl::CSetOfObjects::Create();
  grp->insert(glCloud);
  visualizer_->update_3d_object("glim/scan", grp);
}

void GlimOdometry::internalBuildGUI()
{
  using namespace mola::gui;

  gui_.lbStatus = std::make_shared<LiveString>(" ");

  Tab tab;
  tab.title = "Status";
  tab.widgets.emplace_back(Label{gui_.lbStatus});
  tab.widgets.emplace_back(CheckBox{
    "Show trajectory", visualization_params_.show_trajectory,
    [this](bool checked) { visualization_params_.show_trajectory = checked; }});
  tab.widgets.emplace_back(CheckBox{
    "Show deskewed scan", visualization_params_.show_scan,
    [this](bool checked) { visualization_params_.show_scan = checked; }});
  tab.widgets.emplace_back(CheckBox{
    "Camera follows vehicle", visualization_params_.camera_follows_vehicle,
    [this](bool checked) { visualization_params_.camera_follows_vehicle = checked; }});

  WindowDescription desc;
  desc.title = "GLIM";
  desc.position = {5, 700};
  desc.size = {300, 0};
  desc.tabs.emplace_back(std::move(tab));

  visualizer_->create_subwindow_from_description(desc);
}

void GlimOdometry::updateVisualizationTextLabels(const glim_core::OdometryOutput & out)
{
  if (!gui_.lbStatus) {
    return;
  }

  gui_.lbStatus->set(mrpt::format(
    "t=%.03f | scan points: %zu/%zu | |v|=%.02f m/s | busy: %s", out.timestamp,
    out.num_points_preprocessed, out.num_points_raw, out.vel.norm(), isBusy() ? "yes" : "no"));
}

void GlimOdometry::spinOnce() {}

bool GlimOdometry::isBusy() const
{
  return tasks_in_flight_.load() > 0 || worker_lidar_.pendingTasks() > 0;
}

mrpt::poses::CPose3DInterpolator GlimOdometry::estimatedTrajectory() const
{
  auto lck = mrpt::lockHelper(trajectory_mtx_);
  return trajectory_;
}

}  // namespace mola
