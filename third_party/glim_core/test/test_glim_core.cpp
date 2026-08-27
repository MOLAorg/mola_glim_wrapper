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
 * @file   test_glim_core.cpp
 * @brief  Constructs a GlimCore and feeds it a synthetic scan + IMU sequence.
 */

#include <glim_core/GlimCore.h>
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{
constexpr double kScanPeriod = 0.1;
constexpr double kImuPeriod = 0.005;
constexpr double kGravity = 9.81;

/// A box-shaped room, sampled densely enough that the three orthogonal wall
/// pairs constrain all six degrees of freedom.
glim_core::LidarPointVector makeSyntheticScan(double sensorX)
{
  glim_core::LidarPointVector pts;
  const int nAz = 180;
  const int nEl = 16;
  pts.reserve(nAz * nEl);

  // Half-extents of the room, and where the sensor is inside it.
  const Eigen::Vector3d half(10.0, 10.0, 3.0);
  const Eigen::Vector3d org(sensorX, 0.0, 0.0);

  for (int ia = 0; ia < nAz; ia++) {
    const double az = 2.0 * M_PI * ia / nAz;
    for (int ie = 0; ie < nEl; ie++) {
      const double el = (-15.0 + 30.0 * ie / (nEl - 1)) * M_PI / 180.0;

      const Eigen::Vector3d dir(
        std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));

      // Exit point of the ray from an axis-aligned box: the smallest positive
      // per-axis crossing.
      double t = std::numeric_limits<double>::max();
      for (int k = 0; k < 3; k++) {
        if (std::abs(dir[k]) < 1e-9) {
          continue;
        }
        const double wall = dir[k] > 0 ? half[k] : -half[k];
        const double tk = (wall - org[k]) / dir[k];
        if (tk > 0 && tk < t) {
          t = tk;
        }
      }
      if (t == std::numeric_limits<double>::max()) {
        continue;
      }

      glim_core::LidarPoint p;
      p.x = static_cast<float>(dir.x() * t);
      p.y = static_cast<float>(dir.y() * t);
      p.z = static_cast<float>(dir.z() * t);
      p.intensity = 100.f;
      p.t_offset_sec = kScanPeriod * ia / nAz;
      pts.push_back(p);
    }
  }
  return pts;
}
}  // namespace

TEST(GlimCore, RejectsEmptyConfigPath)
{
  glim_core::Config cfg;
  EXPECT_THROW(glim_core::GlimCore{cfg}, std::runtime_error);
}

TEST(GlimCore, ProcessesSyntheticSequence)
{
  glim_core::Config cfg;
  cfg.glim_config_path = GLIM_CORE_TEST_CONFIG_DIR;
  cfg.num_threads = 2;
  cfg.registration_type = "VGICP";
  // The synthetic room is small, so the default 1.0 m near cut would drop
  // nothing but the default 0.25 m downsampling would leave too few points.
  cfg.downsample_resolution = 0.1;

  glim_core::GlimCore core(cfg);

  const int nScans = 60;
  double t = 1000.0;
  const double vx = 0.5;  // m/s, straight-line motion along +X

  int nValid = 0;
  int firstValid = -1;
  int lastValid = -1;
  Eigen::Vector3d firstValidPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d lastValidPos = Eigen::Vector3d::Zero();
  double firstValidT = 0;
  double lastValidT = 0;

  for (int i = 0; i < nScans; i++) {
    const double tBeg = t + i * kScanPeriod;
    const double tEnd = tBeg + kScanPeriod;

    // IMU: level and stationary in attitude, so gravity is all the
    // accelerometer sees; the motion is constant velocity.
    for (double ti = tBeg - kScanPeriod; ti < tEnd; ti += kImuPeriod) {
      glim_core::ImuSample s;
      s.t = ti;
      s.acc = Eigen::Vector3d(0, 0, kGravity);
      s.gyro = Eigen::Vector3d::Zero();
      core.addImuSample(s);
    }

    glim_core::OdometryOutput out;
    ASSERT_TRUE(core.processScan(makeSyntheticScan(vx * i * kScanPeriod), tBeg, tEnd, out));
    if (!out.valid) {
      continue;
    }
    nValid++;
    if (firstValid < 0) {
      firstValid = i;
      firstValidPos = out.pose_lidar.translation();
      firstValidT = out.timestamp;
    }
    lastValid = i;
    lastValidPos = out.pose_lidar.translation();
    lastValidT = out.timestamp;
  }

  EXPECT_EQ(core.scanCount(), static_cast<std::size_t>(nScans));

  // The LOOSE initialization spends a fixed window bootstrapping the initial
  // attitude and velocity, and returns no pose at all until it finishes: on a
  // 10 Hz stream that is the first ~30 scans. What must hold is that once
  // poses start they never stop.
  ASSERT_GE(firstValid, 0);
  EXPECT_EQ(nValid, lastValid - firstValid + 1);
  EXPECT_GE(nValid, 15);

  // And that the estimate tracks the motion it was given, rather than merely
  // producing well-formed poses: constant velocity along +X.
  const double elapsed = lastValidT - firstValidT;
  const double expected = vx * elapsed;
  ASSERT_GT(elapsed, 1.0);
  EXPECT_NEAR((lastValidPos - firstValidPos).norm(), expected, 0.25 * expected + 0.05);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
