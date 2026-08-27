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
 * @file   GlimCore.h
 * @brief  Thin, caller-driven facade over GLIM's odometry estimation.
 */
#pragma once

#include <glim_core/Types.hpp>
#include <memory>

namespace glim_core
{

/** Drives GLIM's LiDAR-inertial odometry front end synchronously.
 *
 * Unlike upstream's `AsyncOdometryEstimation`, nothing here runs on a thread
 * this class owns: `processScan()` returns only once the scan has been
 * registered, which is what gives the offline CLI its loss-free guarantee.
 *
 * Only the odometry stage is instantiated. GLIM's sub-mapping and global
 * mapping stages (which add loop closure and global bundle adjustment) are
 * deliberately left out, so that what this reports is odometry comparable
 * with the other LiDAR-inertial odometry wrappers, not a full SLAM result.
 */
class GlimCore
{
public:
  explicit GlimCore(const Config & cfg);
  ~GlimCore();

  GlimCore(const GlimCore &) = delete;
  GlimCore & operator=(const GlimCore &) = delete;

  /** Feeds one IMU sample. Must be called in non-decreasing time order.
   * Samples whose timestamp GLIM rejects as out of sync are dropped and
   * counted in droppedImuSamples(). */
  void addImuSample(const ImuSample & s);

  /** Registers one scan.
   *
   * @param points  Scan points in the LiDAR frame, with per-point time
   *                offsets relative to `t_beg`.
   * @param t_beg   Absolute time of the first point, seconds.
   * @param t_end   Absolute time of the last point, seconds.
   * @param out     [out] The solution. `out.valid` is false while GLIM is
   *                still bootstrapping its initial state.
   * @return True if the scan was consumed, false if it was rejected before
   *         reaching the estimator (empty, or an invalid timestamp).
   */
  bool processScan(
    const LidarPointVector & points, double t_beg, double t_end, OdometryOutput & out);

  /** Number of IMU samples GLIM refused as out of sync with the LiDAR. */
  std::size_t droppedImuSamples() const;

  /** Number of scans fed so far, including those that produced no pose. */
  std::size_t scanCount() const;

  const Config & config() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace glim_core
