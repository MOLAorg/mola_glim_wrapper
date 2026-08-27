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
 * @file   GlimOdometry_Adapters.cpp
 * @brief  Conversions between MRPT/MOLA and glim_core plain types
 */
#include "GlimOdometry_Adapters.h"

#include <mrpt/maps/CSimplePointsMap.h>

#include <algorithm>
#include <cmath>

namespace mola
{
using glim_core::LidarPoint;
using glim_core::LidarPointVector;
using glim_core::XYZIPointVector;

/** Converts an incoming LiDAR observation into GLIM points, and computes the
 * scan's absolute [t_beg, t_end] time span.
 *
 * If the point cloud carries a per-point relative-time channel
 * (`CPointsMap::POINT_FIELD_TIMESTAMP`, seconds, in any convention -- centered
 * on the observation timestamp, start-relative, etc.), it is used directly:
 * each point's `t_offset_sec` is re-normalized against this scan's own
 * min/max, so no assumption about the dataset's time convention is needed.
 * This is what GLIM's deskewing runs on.
 *
 * Otherwise (no per-point timestamps, e.g. KITTI), the offset is synthesized
 * from each point's azimuth angle assuming constant angular velocity across
 * `fallback_scan_period` seconds, treating the observation timestamp as the
 * scan's end. Same construction as the Fast-LIO2 and DLIO wrappers use, so
 * the three see identical per-point clocks on such a dataset.
 */
LidarPointVector toGlimCloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end, double blind, bool * usedPerPointTimestamps)
{
  LidarPointVector out;
  const double blind2 = blind * blind;

  const auto & pts = obs.pointcloud;
  if (!pts) {
    t_beg = t_end = mrpt::Clock::toDouble(obs.timestamp);
    return out;
  }

  const auto & xs = pts->getPointsBufferRef_x();
  const auto & ys = pts->getPointsBufferRef_y();
  const auto & zs = pts->getPointsBufferRef_z();
  const auto n = xs.size();

  const auto * is =
    pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY);
  const auto * ts =
    pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);

  out.resize(n);

  const double obsTime = mrpt::Clock::toDouble(obs.timestamp);

  if (usedPerPointTimestamps) {
    *usedPerPointTimestamps = ts && ts->size() == n && n > 0;
  }

  if (ts && ts->size() == n && n > 0) {
    float tMin = (*ts)[0];
    float tMax = (*ts)[0];
    for (size_t i = 0; i < n; i++) {
      tMin = std::min(tMin, (*ts)[i]);
      tMax = std::max(tMax, (*ts)[i]);
    }
    t_beg = obsTime + tMin;
    t_end = obsTime + tMax;

    size_t k = 0;
    for (size_t i = 0; i < n; i++) {
      const double r2 = static_cast<double>(xs[i]) * xs[i] + static_cast<double>(ys[i]) * ys[i] +
                        static_cast<double>(zs[i]) * zs[i];
      if (r2 < blind2) {
        continue;
      }
      LidarPoint & p = out[k++];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;
      p.t_offset_sec = static_cast<double>((*ts)[i] - tMin);
    }
    out.resize(k);
  } else {
    t_beg = obsTime - fallback_scan_period;
    t_end = obsTime;

    size_t k = 0;
    for (size_t i = 0; i < n; i++) {
      const double r2 = static_cast<double>(xs[i]) * xs[i] + static_cast<double>(ys[i]) * ys[i] +
                        static_cast<double>(zs[i]) * zs[i];
      if (r2 < blind2) {
        continue;
      }
      LidarPoint & p = out[k++];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;

      // Azimuth in [0, 2*pi), 0 at the scan start.
      const double azimuth = std::atan2(static_cast<double>(ys[i]), static_cast<double>(xs[i]));
      const double azimuth01 = (azimuth + M_PI) / (2.0 * M_PI);
      p.t_offset_sec = azimuth01 * fallback_scan_period;
    }
    out.resize(k);
  }

  return out;
}

glim_core::ImuSample toGlimImu(const mrpt::obs::CObservationIMU & obs)
{
  glim_core::ImuSample s;
  s.t = mrpt::Clock::toDouble(obs.timestamp);
  s.acc = Eigen::Vector3d(
    obs.get(mrpt::obs::IMU_X_ACC), obs.get(mrpt::obs::IMU_Y_ACC), obs.get(mrpt::obs::IMU_Z_ACC));
  s.gyro = Eigen::Vector3d(
    obs.get(mrpt::obs::IMU_WX), obs.get(mrpt::obs::IMU_WY), obs.get(mrpt::obs::IMU_WZ));
  return s;
}

mrpt::maps::CPointsMap::Ptr toMrptPointsMap(const XYZIPointVector & points)
{
  auto out = mrpt::maps::CSimplePointsMap::Create();
  out->reserve(points.size());
  for (const auto & p : points) {
    out->insertPointFast(p.x, p.y, p.z);
  }
  return out;
}

}  // namespace mola
