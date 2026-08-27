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
 * @file   GlimOdometry_Adapters.h
 * @brief  Conversions between MRPT/MOLA and glim_core plain types
 *         (private to this module, not installed).
 */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>

#include <glim_core/Types.hpp>

namespace mola
{
/** See the .cpp file for the full documentation of the conversion logic. */
glim_core::LidarPointVector toGlimCloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end, double blind = 0.0, bool * usedPerPointTimestamps = nullptr);

glim_core::ImuSample toGlimImu(const mrpt::obs::CObservationIMU & obs);

mrpt::maps::CPointsMap::Ptr toMrptPointsMap(const glim_core::XYZIPointVector & points);

}  // namespace mola
