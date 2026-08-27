/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: BSD-3-Clause
 See LICENSE for full license information.
 See THIRD_PARTY_NOTICES.md: drives the MIT-licensed GLIM algorithm
 (third_party/glim), copyright Kenji Koide.
*/

#include <glim_core/GlimCore.h>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <fstream>
#include <glim/common/cloud_covariance_estimation.hpp>
#include <glim/odometry/estimation_frame.hpp>
#include <glim/odometry/odometry_estimation_cpu.hpp>
#include <glim/preprocess/cloud_preprocessor.hpp>
#include <glim/util/config.hpp>
#include <glim/util/raw_points.hpp>
#include <glim/util/time_keeper.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <stdexcept>

namespace glim_core
{

namespace
{
/// Magnitude of the gravity vector GTSAM's "up" preintegration convention
/// uses, which is what glim::IMUIntegration selects.
constexpr double kGravity = 9.81;

std::ofstream * gravity_dump_stream()
{
  static const char * path = ::getenv("MOLA_GLIM_DUMP_ESTIMATED_GRAV");
  static std::ofstream f(path != nullptr ? path : "/dev/null");
  return path != nullptr ? &f : nullptr;
}
}  // namespace

struct GlimCore::Impl
{
  Config cfg;

  std::unique_ptr<glim::TimeKeeper> time_keeper;
  std::unique_ptr<glim::CloudPreprocessor> preprocessor;
  std::unique_ptr<glim::OdometryEstimationCPU> odometry;

  std::size_t dropped_imu = 0;
  std::size_t scan_count = 0;
};

GlimCore::GlimCore(const Config & cfg) : impl_(std::make_unique<Impl>())
{
  impl_->cfg = cfg;

  if (cfg.glim_config_path.empty()) {
    throw std::runtime_error(
      "glim_core: Config::glim_config_path is empty. GLIM reads several of its own "
      "parameters straight from its JSON config directory, so a valid path is required.");
  }
  {
    const std::ifstream f(cfg.glim_config_path + "/config.json");
    if (!f) {
      throw std::runtime_error("glim_core: cannot open '" + cfg.glim_config_path + "/config.json'");
    }
  }

  // Process-wide singleton, overwritten on purpose so that a second core in
  // the same process picks up this instance's config directory.
  glim::GlobalConfig::instance(cfg.glim_config_path, true);

  // spdlog is process-wide too, and GLIM's module loggers inherit whatever
  // level is set when they are created, so this has to happen before the
  // estimator is constructed.
  spdlog::set_level(spdlog::level::from_str(cfg.log_level));
  spdlog::flush_on(spdlog::level::err);

  impl_->time_keeper = std::make_unique<glim::TimeKeeper>();

  // Preprocessing: start from GLIM's own JSON values, then apply the subset
  // this wrapper exposes through the MOLA pipeline YAML.
  glim::CloudPreprocessorParams pp;
  pp.distance_near_thresh = cfg.distance_near_thresh;
  pp.distance_far_thresh = cfg.distance_far_thresh;
  pp.global_shutter = cfg.global_shutter;
  pp.use_random_grid_downsampling = cfg.use_random_grid_downsampling;
  pp.downsample_resolution = cfg.downsample_resolution;
  pp.downsample_target = cfg.downsample_target;
  pp.downsample_rate = cfg.downsample_rate;
  pp.enable_outlier_removal = cfg.enable_outlier_removal;
  pp.outlier_removal_k = cfg.outlier_removal_k;
  pp.outlier_std_mul_factor = cfg.outlier_std_mul_factor;
  pp.k_correspondences = cfg.k_correspondences;
  pp.num_threads = cfg.num_threads;
  pp.T_imu_lidar = cfg.T_lidar_imu.inverse();
  impl_->preprocessor = std::make_unique<glim::CloudPreprocessor>(pp);

  glim::OdometryEstimationCPUParams op;
  op.T_lidar_imu = cfg.T_lidar_imu;
  op.imu_bias_noise = cfg.imu_bias_noise;
  op.imu_acc_noise = cfg.imu_acc_noise;
  op.imu_gyro_noise = cfg.imu_gyro_noise;
  op.imu_int_noise = cfg.imu_int_noise;
  op.fix_imu_bias = cfg.fix_imu_bias;
  op.initialization_mode = cfg.initialization_mode;
  op.initialization_window_size = cfg.initialization_window_size;
  op.init_pose_damping_scale = cfg.init_pose_damping_scale;
  op.smoother_lag = cfg.smoother_lag;
  op.use_isam2_dogleg = cfg.use_isam2_dogleg;
  op.isam2_relinearize_skip = cfg.isam2_relinearize_skip;
  op.isam2_relinearize_thresh = cfg.isam2_relinearize_thresh;
  op.validate_imu = cfg.validate_imu;
  op.save_imu_rate_trajectory = cfg.save_imu_rate_trajectory;
  op.num_threads = cfg.num_threads;
  op.registration_type = cfg.registration_type;
  op.max_iterations = cfg.max_iterations;
  op.lru_thresh = cfg.lru_thresh;
  op.target_downsampling_rate = cfg.target_downsampling_rate;
  op.ivox_resolution = cfg.ivox_resolution;
  op.ivox_min_dist = cfg.ivox_min_dist;
  op.vgicp_resolution = cfg.vgicp_resolution;
  op.vgicp_voxelmap_levels = cfg.vgicp_voxelmap_levels;
  op.vgicp_voxelmap_scaling_factor = cfg.vgicp_voxelmap_scaling_factor;

  impl_->odometry = std::make_unique<glim::OdometryEstimationCPU>(op);
}

GlimCore::~GlimCore() = default;

void GlimCore::addImuSample(const ImuSample & s)
{
  if (!impl_->time_keeper->validate_imu_stamp(s.t)) {
    impl_->dropped_imu++;
    return;
  }
  impl_->odometry->insert_imu(s.t, s.acc, s.gyro);
}

bool GlimCore::processScan(
  const LidarPointVector & points, double t_beg, double t_end, OdometryOutput & out)
{
  out = OdometryOutput();

  if (points.empty()) {
    return false;
  }

  auto raw = std::make_shared<glim::RawPoints>();
  raw->stamp = t_beg;
  raw->points.resize(points.size());
  raw->times.resize(points.size());
  raw->intensities.resize(points.size());
  for (std::size_t i = 0; i < points.size(); i++) {
    const auto & p = points[i];
    raw->points[i] = Eigen::Vector4d(p.x, p.y, p.z, 1.0);
    raw->times[i] = p.t_offset_sec;
    raw->intensities[i] = p.intensity;
  }

  // t_end only enters through the per-point offsets, which the caller has
  // already normalized against it; keeping the argument makes the call site
  // read the same as the other wrappers'.
  (void)t_end;

  if (!impl_->time_keeper->process(raw)) {
    return false;
  }

  const auto preprocessed = impl_->preprocessor->preprocess(raw);
  if (!preprocessed || preprocessed->size() == 0) {
    return false;
  }

  impl_->scan_count++;

  std::vector<glim::EstimationFrame::ConstPtr> marginalized;
  const auto frame = impl_->odometry->insert_frame(preprocessed, marginalized);

  out.timestamp = raw->stamp;
  out.num_points_raw = points.size();
  out.num_points_preprocessed = static_cast<std::size_t>(preprocessed->size());

  if (!frame) {
    // GLIM is still bootstrapping its initial state; this is expected for
    // the first few scans and is not an error.
    return true;
  }

  out.valid = true;
  out.timestamp = frame->stamp;
  out.pose_lidar = frame->T_world_lidar;
  out.pose_imu = frame->T_world_imu;
  out.vel = frame->v_world_imu;
  out.bias_acc = frame->imu_bias.head<3>();
  out.bias_gyro = frame->imu_bias.tail<3>();

  const Eigen::Vector3d g_world(0, 0, -kGravity);
  out.gravity_body = frame->T_world_imu.linear().transpose() * g_world;

  if (auto * f = gravity_dump_stream(); f != nullptr) {
    (*f) << out.timestamp << " " << out.gravity_body.x() << " " << out.gravity_body.y() << " "
         << out.gravity_body.z() << "\n";
  }

  if (impl_->cfg.export_deskewed_points && frame->frame) {
    auto pts = std::make_shared<XYZIPointVector>();
    pts->resize(frame->frame->size());
    for (std::size_t i = 0; i < pts->size(); i++) {
      const auto & p = frame->frame->points[i];
      (*pts)[i].x = static_cast<float>(p.x());
      (*pts)[i].y = static_cast<float>(p.y());
      (*pts)[i].z = static_cast<float>(p.z());
      (*pts)[i].intensity = frame->frame->intensities != nullptr
                              ? static_cast<float>(frame->frame->intensities[i])
                              : 0.f;
    }
    out.deskewed_points_body = pts;
  }

  return true;
}

std::size_t GlimCore::droppedImuSamples() const { return impl_->dropped_imu; }

std::size_t GlimCore::scanCount() const { return impl_->scan_count; }

const Config & GlimCore::config() const { return impl_->cfg; }

}  // namespace glim_core
