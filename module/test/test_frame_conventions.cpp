/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: BSD-3-Clause
 See LICENSE for full license information.
*/

/**
 * @file   test_frame_conventions.cpp
 * @brief  Pose algebra of the reported output frame.
 *
 * Getting these compositions wrong is invisible in an aligned APE and
 * catastrophic in every relative metric, which is why they are tested
 * separately from anything that needs an estimator.
 */

#include <gtest/gtest.h>
#include <mola_glim_wrapper/GlimOdometry.h>
#include <mrpt/poses/CPose3D.h>

namespace
{
mrpt::poses::CPose3D pose6D(
  double x, double y, double z, double yawDeg, double pitchDeg, double rollDeg)
{
  return {x, y, z, mrpt::DEG2RAD(yawDeg), mrpt::DEG2RAD(pitchDeg), mrpt::DEG2RAD(rollDeg)};
}
}  // namespace

TEST(GlimFrames, IdentityExtrinsicIsANoOp)
{
  const auto worldFromLidar = pose6D(1, 2, 3, 30, 10, -5);
  const auto out =
    mola::GlimOdometry::worldFromBaseLink(worldFromLidar, mrpt::poses::CPose3D::Identity());
  EXPECT_NEAR((out.asVectorVal() - worldFromLidar.asVectorVal()).norm(), 0.0, 1e-9);
}

TEST(GlimFrames, OxfordSpiresBaseLinkComposition)
{
  // T_base_lidar for Oxford Spires: base_link is the raw LiDAR frame yawed
  // 180 deg and 0.124 m up.
  const auto lidarInBase = pose6D(0, 0, 0.124, 180, 0, 0);

  // With the estimator at the origin, the reported base_link pose must be
  // exactly the inverse of that extrinsic.
  const auto out =
    mola::GlimOdometry::worldFromBaseLink(mrpt::poses::CPose3D::Identity(), lidarInBase);

  mrpt::poses::CPose3D expected = lidarInBase;
  expected.inverse();
  EXPECT_NEAR((out.asVectorVal() - expected.asVectorVal()).norm(), 0.0, 1e-9);
}

TEST(GlimFrames, ComposingBackRecoversTheLidarPose)
{
  const auto worldFromLidar = pose6D(-4, 7, 0.5, -110, 3, 12);
  const auto lidarInBase = pose6D(0.1, -0.2, 0.124, 180, 1, -2);

  const auto base = mola::GlimOdometry::worldFromBaseLink(worldFromLidar, lidarInBase);

  // T_w_lidar == T_w_base . T_base_lidar
  const mrpt::poses::CPose3D back = base + lidarInBase;
  EXPECT_NEAR((back.asVectorVal() - worldFromLidar.asVectorVal()).norm(), 0.0, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
