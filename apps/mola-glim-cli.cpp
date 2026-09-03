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
 * @file   mola-glim-cli.cpp
 * @brief  main() for the offline, loss-free GLIM CLI.
 *
 * Structurally mirrors mola_lidar_odometry's own
 * `mola-lidar-odometry-cli.cpp` (same dataset-source helpers, same
 * TCLAP flags where they overlap, same `while (isBusy()) sleep(1ms)`
 * no-drop idiom before advancing to the next dataset entry), minus the
 * companion state-estimator machinery: GLIM carries its own fixed-lag
 * factor-graph estimator and needs none.
 */

#include <mola_glim_wrapper/GlimOdometry.h>
#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/progress.h>

#include <CLI/CLI.hpp>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#if defined(HAVE_MOLA_INPUT_MULRAN)
#include <mola_input_mulran_dataset/MulranDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
#include <mola_input_rawlog/RawlogDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
#include <mola_input_rosbag2/Rosbag2Dataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
#include <mola_input_rosbag1/Rosbag1Dataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
#include <mola_input_kitti_dataset/KittiOdometryDataset.h>
#endif

namespace
{

struct Cli
{
  CLI::App cmd{"mola-glim-cli"};

  std::string argYAML;
  std::string arg_verbosity_level{"INFO"};
  bool arg_verbosity_level_set{false};
  std::string arg_plugins;
  bool arg_plugins_set{false};
  std::string arg_outPath{"output-trajectory.txt"};
  bool arg_outPath_set{false};
  int arg_firstN{0};
  bool arg_firstN_set{false};
  int arg_numThreads{1};
  bool arg_numThreads_set{false};

#if defined(HAVE_MOLA_INPUT_MULRAN)
  std::string argMulranSeq{"KAIST01"};
  bool argMulranSeq_set{false};
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
  std::string argRawlog{"dataset.rawlog"};
  bool argRawlog_set{false};
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  std::string argRosbag2{"dataset.mcap"};
  bool argRosbag2_set{false};
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
  std::string argRosbag1{"dataset.bag"};
  bool argRosbag1_set{false};
#endif

// Shared by both bag formats -- rosbag1 and rosbag2 read the same topic names
// into the same lidar/imu sensor entries, see dataset_from_rosbag1() and
// dataset_from_rosbag2() below.
#if defined(HAVE_MOLA_INPUT_ROSBAG2) || defined(HAVE_MOLA_INPUT_ROSBAG1)
  std::string arg_lidarTopic{"/lidar"};
  std::string arg_imuTopic{"/imu"};
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
  std::string argKittiSeq{"00"};
  bool argKittiSeq_set{false};
#endif

  CLI::Option * optVerbosity{nullptr};
  CLI::Option * optPlugins{nullptr};
  CLI::Option * optOutPath{nullptr};
  CLI::Option * optFirstN{nullptr};
  CLI::Option * optNumThreads{nullptr};
#if defined(HAVE_MOLA_INPUT_MULRAN)
  CLI::Option * optMulranSeq{nullptr};
#endif
#if defined(HAVE_MOLA_INPUT_RAWLOG)
  CLI::Option * optRawlog{nullptr};
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  CLI::Option * optRosbag2{nullptr};
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG1)
  CLI::Option * optRosbag1{nullptr};
#endif
#if defined(HAVE_MOLA_INPUT_KITTI)
  CLI::Option * optKittiSeq{nullptr};
#endif

  Cli()
  {
    cmd.add_option("-c,--config", argYAML, "Input GLIM pipeline YAML config file (required)")
      ->required();
    optVerbosity = cmd.add_option(
      "-v,--verbosity", arg_verbosity_level,
      "Verbosity level: ERROR|WARN|INFO|DEBUG {Default: INFO}");
    optPlugins = cmd.add_option(
      "-l,--load-plugins", arg_plugins,
      "One or more {comma separated} *.so files to load as plugins");
    optOutPath = cmd.add_option(
      "--output-tum-path", arg_outPath,
      "Save the estimated path as a TXT file using the TUM file format (see evo docs)");
    optFirstN = cmd.add_option(
      "--only-first-n", arg_firstN, "Run for the first N steps only (0=default, not used)");
    optNumThreads = cmd.add_option(
      "--num-threads", arg_numThreads,
      "Worker threads GLIM may use. Defaults to 1, which makes an offline run "
      "bit-for-bit reproducible; anything above 1 is faster and NOT "
      "reproducible (see the comment in main_odometry())");

#if defined(HAVE_MOLA_INPUT_MULRAN)
    optMulranSeq = cmd.add_option(
      "--input-mulran-seq", argMulranSeq,
      "INPUT DATASET: Use Mulran dataset sequence KAIST01|KAIST01|...");
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
    optRawlog = cmd.add_option(
      "--input-rawlog", argRawlog,
      "INPUT DATASET: rawlog. Input dataset in rawlog format (*.rawlog)");
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
    optRosbag2 = cmd.add_option(
      "--input-rosbag2", argRosbag2,
      "INPUT DATASET: rosbag2. Input dataset in rosbag2 format (*.mcap)");
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
    optRosbag1 = cmd.add_option(
      "--input-rosbag1", argRosbag1,
      "INPUT DATASET: rosbag1. Input dataset in ROS 1 bag format (*.bag)");
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2) || defined(HAVE_MOLA_INPUT_ROSBAG1)
    cmd.add_option(
      "--lidar-topic", arg_lidarTopic,
      "Only for rosbag1/rosbag2 input: the LiDAR point cloud topic name.");
    cmd.add_option(
      "--imu-topic", arg_imuTopic, "Only for rosbag1/rosbag2 input: the IMU topic name.");
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
    optKittiSeq = cmd.add_option(
      "--input-kitti-seq", argKittiSeq,
      "INPUT DATASET: Use KITTI dataset sequence number 00|01|...");
#endif
  }

  // Called from main() right after CLI11_PARSE, to fill in the *_set flags
  // (mirrors what TCLAP's own isSet() gave us for free).
  void afterParse()
  {
    arg_verbosity_level_set = (optVerbosity->count() > 0);
    arg_plugins_set = (optPlugins->count() > 0);
    arg_outPath_set = (optOutPath->count() > 0);
    arg_firstN_set = (optFirstN->count() > 0);
    arg_numThreads_set = (optNumThreads->count() > 0);
#if defined(HAVE_MOLA_INPUT_MULRAN)
    argMulranSeq_set = (optMulranSeq->count() > 0);
#endif
#if defined(HAVE_MOLA_INPUT_RAWLOG)
    argRawlog_set = (optRawlog->count() > 0);
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
    argRosbag2_set = (optRosbag2->count() > 0);
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG1)
    argRosbag1_set = (optRosbag1->count() > 0);
#endif
#if defined(HAVE_MOLA_INPUT_KITTI)
    argKittiSeq_set = (optKittiSeq->count() > 0);
#endif
  }
};  // end struct "Cli"

#if defined(HAVE_MOLA_INPUT_MULRAN)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_mulran(
  const std::string & mulranSequence, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::MulranDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      base_dir: ${MULRAN_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      publish_lidar: true
      publish_ground_truth: true
)"""",
    mulranSequence.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_RAWLOG)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rawlog(
  const std::string & rawlogFile, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::RawlogDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rawlog_filename: '%s'
      read_all_first: true
)"""",
    rawlogFile.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag2(
  Cli & cli, const std::string & rosbag2file, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::Rosbag2Dataset>();
  o->setMinLoggingLevel(logLevel);

  // A comma-separated value becomes a YAML sequence, so that a sequence split
  // across several bag directories (e.g. Oxford Spires keble-college-04, whose
  // two parts share a base timestamp and are two halves of one continuous
  // recording) can be replayed as the single sequence it is. Rosbag2Dataset
  // accepts either a scalar or a sequence for 'rosbag_filename'.
  std::string bagsYaml;
  {
    std::vector<std::string> parts;
    mrpt::system::tokenize(rosbag2file, ",", parts);
    ASSERT_(!parts.empty());
    if (parts.size() == 1) {
      bagsYaml = "'" + mrpt::system::trim(parts[0]) + "'";
    } else {
      for (const auto & p : parts) {
        bagsYaml += "\n        - '" + mrpt::system::trim(p) + "'";
      }
    }
  }

  // Fixed sensor poses (env vars), for bags with no /tf or /tf_static (e.g.
  // Oxford Spires): same env var names as mola-lidar-odometry-cli's own
  // dataset_from_rosbag2(), so the same override snippet works for both.
  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rosbag_filename: %s
      base_link_frame_id: "${MOLA_TF_BASE_LINK|base_link}"
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          sensorLabel: lidar
          fixed_sensor_pose: "${LIDAR_POSE_X|0} ${LIDAR_POSE_Y|0} ${LIDAR_POSE_Z|0} ${LIDAR_POSE_YAW|0} ${LIDAR_POSE_PITCH|0} ${LIDAR_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: '%s'
          type: CObservationIMU
          sensorLabel: imu
          fixed_sensor_pose: "${IMU_POSE_X|0} ${IMU_POSE_Y|0} ${IMU_POSE_Z|0} ${IMU_POSE_YAW|0} ${IMU_POSE_PITCH|0} ${IMU_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_IMU_POSE|false}
)"""",
    bagsYaml.c_str(), cli.arg_lidarTopic.c_str(), cli.arg_imuTopic.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag1(
  Cli & cli, const std::string & rosbag1file, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::Rosbag1Dataset>();
  o->setMinLoggingLevel(logLevel);

  // A comma-separated value becomes a YAML sequence, so a recording split
  // across several bag files (e.g. CitrusFarm) is replayed as the single
  // sequence it is. Rosbag1Dataset accepts a scalar or a sequence.
  std::string bagsYaml;
  {
    std::vector<std::string> parts;
    mrpt::system::tokenize(rosbag1file, ",", parts);
    ASSERT_(!parts.empty());
    if (parts.size() == 1) {
      bagsYaml = "'" + mrpt::system::trim(parts[0]) + "'";
    } else {
      for (const auto & p : parts) {
        bagsYaml += "\n        - '" + mrpt::system::trim(p) + "'";
      }
    }
  }

  // Same env var names as dataset_from_rosbag2() above and as
  // mola-lidar-odometry-cli's own dataset_from_rosbag1(), so the same
  // override snippet (and the same dataset profiles) work unchanged here.
  // Unlike that CLI's version, this carries only the lidar and imu entries:
  // GLIM has no GNSS/wheel-odometry fusion path, it only ever reads
  // lidar_sensor_label/imu_sensor_label.
  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rosbag_filename: %s
      base_link_frame_id: "${MOLA_TF_BASE_LINK|base_link}"
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          sensorLabel: lidar
          fixed_sensor_pose: "${LIDAR_POSE_X|0} ${LIDAR_POSE_Y|0} ${LIDAR_POSE_Z|0} ${LIDAR_POSE_YAW|0} ${LIDAR_POSE_PITCH|0} ${LIDAR_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: '%s'
          type: CObservationIMU
          sensorLabel: imu
          fixed_sensor_pose: "${IMU_POSE_X|0} ${IMU_POSE_Y|0} ${IMU_POSE_Z|0} ${IMU_POSE_YAW|0} ${IMU_POSE_PITCH|0} ${IMU_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_IMU_POSE|false}
)"""",
    bagsYaml.c_str(), cli.arg_lidarTopic.c_str(), cli.arg_imuTopic.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
// KittiOdometryDataset carries no IMU stream at all -- the standard KITTI
// odometry-benchmark sequences never shipped one, unlike KITTI raw/tracking.
// GLIM is IMU-mandatory (its IMU stream drives both the initial-attitude
// estimate and the preintegrated factor between consecutive states), so this
// wraps the real dataset and interleaves synthetic, constant-gravity,
// zero-angular-rate CObservationIMU entries immediately before each real
// LiDAR scan.
// That keeps the propagation clock ticking and lets the gravity/bias
// initialization converge trivially to "level, stationary" -- the actual
// motion estimate then comes entirely from LiDAR-only scan-to-map
// registration.
//
// Same technique, same rationale and same constant as mola-dlio-cli's
// KittiWithSyntheticImu, deliberately: the two wrappers have to feed KITTI
// identically for a cross-method comparison on it to mean anything.
//
// Emitting each synthetic sample as its own dataset timestep, rather than
// merging them into one CSensoryFrame per timestep, is deliberate:
// main_odometry()'s loop below only ever takes ONE observation per timestep
// (pointcloud, else IMU), so a merged frame would silently drop the rest.
//
// MOLA_KITTI_SYNTH_IMU_PER_SCAN raises the number of synthetic samples per
// scan interval above the one-per-scan default the other wrappers use.
// Leave it at 1 for any run whose numbers are meant to be compared with
// theirs; a higher rate only conditions GLIM's preintegration better, it
// carries no extra information.
class KittiWithSyntheticImu : public mola::OfflineDatasetSource
{
public:
  explicit KittiWithSyntheticImu(std::shared_ptr<mola::KittiOdometryDataset> inner, int imuPerScan)
  : imu_per_scan_(imuPerScan), inner_(std::move(inner))
  {
    ASSERT_(imu_per_scan_ >= 1);
  }

  size_t datasetSize() const override
  {
    return static_cast<size_t>(imu_per_scan_ + 1) * inner_->datasetSize();
  }

  mrpt::obs::CSensoryFrame::Ptr datasetGetObservations(size_t timestep) const override
  {
    const size_t stride = static_cast<size_t>(imu_per_scan_) + 1;
    const size_t realIdx = timestep / stride;
    const size_t sub = timestep % stride;
    if (sub == stride - 1) {
      return inner_->datasetGetObservations(realIdx);
    }

    // Synthetic IMU tick, timestamped just before the scan it precedes.
    const auto realSf = inner_->datasetGetObservations(realIdx);
    const auto lidarObs = realSf->getObservationByClass<mrpt::obs::CObservationPointCloud>();
    ASSERT_(lidarObs);

    // mrpt::Clock::fromDouble()/TTimeStamp's raw tick count has no epoch
    // offset baked in -- fromDouble(0.0) IS the representable minimum, and
    // KITTI's own sequence-relative timestamps (KittiOdometryDataset reads
    // them straight from times.txt) start at exactly 0.0. Subtracting a fixed
    // offset without this clamp underflows the underlying unsigned tick count
    // into a garbage multi-millennium timestamp for that first scan, which
    // then poisons the estimator's notion of its own start time. The clamp
    // costs at most the very first scan, which the core drops anyway for
    // having no IMU strictly before it.
    const double lidarT = mrpt::Clock::toDouble(lidarObs->timestamp);
    auto imu = mrpt::obs::CObservationIMU::Create();
    imu->sensorLabel = "imu";
    // The last tick sits 1 ms before its scan, exactly where the other
    // wrappers put their single one; any extra ticks are spread backwards
    // over the scan interval, so the default (one per scan) is bit-for-bit
    // the same input those wrappers get.
    const double dt =
      0.001 + static_cast<double>(imu_per_scan_ - 1 - sub) * (0.9 * kScanPeriod / imu_per_scan_);
    imu->timestamp = mrpt::Clock::fromDouble(std::max(0.0, lidarT - dt));
    imu->set(mrpt::obs::IMU_X_ACC, 0.0);
    imu->set(mrpt::obs::IMU_Y_ACC, 0.0);
    imu->set(mrpt::obs::IMU_Z_ACC, kGravity);
    imu->set(mrpt::obs::IMU_WX, 0.0);
    imu->set(mrpt::obs::IMU_WY, 0.0);
    imu->set(mrpt::obs::IMU_WZ, 0.0);

    auto sf = mrpt::obs::CSensoryFrame::Create();
    sf->insert(imu);
    return sf;
  }

  bool hasGroundTruthTrajectory() const override { return inner_->hasGroundTruthTrajectory(); }
  mola::trajectory_t getGroundTruthTrajectory() const override
  {
    return inner_->getGroundTruthTrajectory();
  }

private:
  // Standard gravity magnitude (m/s^2); the sign matches a stationary, level
  // accelerometer reading +g on its up axis, which is the convention
  // FastLio2Core's own gravity initialization expects.
  static constexpr double kGravity = 9.80665;

  /// KITTI's Velodyne runs at 10 Hz.
  static constexpr double kScanPeriod = 0.1;

  int imu_per_scan_ = 1;
  std::shared_ptr<mola::KittiOdometryDataset> inner_;
};

std::shared_ptr<mola::OfflineDatasetSource> dataset_from_kitti(
  const std::string & kittiSeqNumber, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::KittiOdometryDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      base_dir: ${KITTI_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      clouds_as_organized_points: false
      publish_lidar: true
      publish_image_0: false
      publish_image_1: false
      publish_ground_truth: true
)"""",
    kittiSeqNumber.c_str())));

  o->initialize(cfg);

  int imuPerScan = 1;
  if (const char * e = ::getenv("MOLA_KITTI_SYNTH_IMU_PER_SCAN"); e != nullptr) {
    imuPerScan = std::max(1, std::atoi(e));
  }

  return std::make_shared<KittiWithSyntheticImu>(o, imuPerScan);
}
#endif

void mola_signal_handler(int s)
{
  std::cerr << "Caught signal " << s << ". Shutting down...\n";
  exit(0);  // NOLINT
}

void mola_install_signal_handler()
{
  struct sigaction sigIntHandler
  {
  };
  sigIntHandler.sa_handler = &mola_signal_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, nullptr);
}

/** Pins the thread count the pipeline YAML reads, and says which way.
 *
 * GLIM's random-grid downsampling draws from a per-thread RNG inside an
 * OpenMP loop with a dynamic schedule, and appends the survivors through an
 * atomic cursor. Which thread draws for which voxel, and the order the
 * results land in, are therefore both decided by the scheduler rather than
 * by the RNG -- so two runs of one configuration sample different points in
 * a different order, which then moves k-NN tie-breaking and float summation
 * order. Measured on kitti-04: at 4 threads two runs differ by 0.27 m at
 * their worst pose; at 1 thread the two output files are byte-identical.
 * The cost is about 1.3x wall clock, because GLIM's time is dominated by
 * its serial fixed-lag smoother.
 *
 * The offline CLI already promises losslessness, so it promises
 * reproducibility too and defaults to one thread. Every OpenMP pragma in
 * the vendored libraries carries an explicit num_threads() clause, so this
 * one value is the whole control -- OMP_NUM_THREADS is not read anywhere.
 *
 * An explicit --num-threads wins over the environment; otherwise an
 * exported GLIM_NUM_THREADS still wins over the default, so a caller who
 * wants speed does not have to change the command line.
 */
void pin_thread_count(Cli & cli)
{
  const std::string n = std::to_string(cli.arg_numThreads);
  const bool explicitFlag = cli.arg_numThreads_set;

  ::setenv("GLIM_NUM_THREADS", n.c_str(), explicitFlag ? 1 : 0);

  const char * effective = ::getenv("GLIM_NUM_THREADS");
  const bool deterministic = effective != nullptr && std::string(effective) == "1";

  std::cout << "GLIM_NUM_THREADS=" << (effective ? effective : "?") << " ("
            << (deterministic ? "deterministic: identical inputs give an identical trajectory"
                              : "NOT deterministic: >1 thread makes the sampling order vary")
            << ")\n";
}

int main_odometry(Cli & cli)
{
  pin_thread_count(cli);

  auto glim = mola::GlimOdometry::Create();

  mrpt::system::VerbosityLevel logLevel = glim->getMinLoggingLevel();
  if (cli.arg_verbosity_level_set) {
    using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
    logLevel = vl::name2value(cli.arg_verbosity_level);
    glim->setVerbosityLevel(logLevel);
  }

  // Initialize GLIM (no 'raw_data_source': we feed it directly below):
  const auto cfg = mola::load_yaml_file(cli.argYAML);
  glim->initialize(cfg);

  // Select dataset input:
  std::shared_ptr<mola::OfflineDatasetSource> dataset;

#if defined(HAVE_MOLA_INPUT_RAWLOG)
  if (cli.argRawlog_set) {
    dataset = dataset_from_rawlog(cli.argRawlog, logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_MULRAN)
    if (cli.argMulranSeq_set) {
    dataset = dataset_from_mulran(cli.argMulranSeq, logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
    if (cli.argRosbag2_set) {
    dataset = dataset_from_rosbag2(cli, cli.argRosbag2, logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG1)
    if (cli.argRosbag1_set) {
    dataset = dataset_from_rosbag1(cli, cli.argRosbag1, logLevel);
  } else
#endif
#if defined(HAVE_MOLA_INPUT_KITTI)
    if (cli.argKittiSeq_set) {
    dataset = dataset_from_kitti(cli.argKittiSeq, logLevel);
  } else
#endif
  {
    THROW_EXCEPTION("At least one of the dataset input CLI flags must be defined. Use --help.");
  }
  ASSERT_(dataset);

  // Save GT, if available:
  if (cli.arg_outPath_set && dataset->hasGroundTruthTrajectory()) {
    using namespace std::string_literals;
    const auto gtPath = dataset->getGroundTruthTrajectory();
    const auto gtOutFile = mrpt::system::fileNameChangeExtension(cli.arg_outPath, "") + "_gt."s +
                           mrpt::system::extractFileExtension(cli.arg_outPath);
    std::cout << "Ground truth available. Saving it to: " << gtOutFile << "\n";
    gtPath.saveToTextFile_TUM(gtOutFile);
  }

  const double tStart = mrpt::Clock::nowDouble();

  size_t lastDatasetEntry = dataset->datasetSize();
  if (cli.arg_firstN_set) {
    lastDatasetEntry = static_cast<size_t>(cli.arg_firstN);
  }
  mrpt::keep_min(lastDatasetEntry, dataset->datasetSize());

  size_t nLidarFed = 0;

  std::cout << "\n";  // Needed for the VT100 codes below.

  for (size_t i = 0; i < lastDatasetEntry; i++) {
    const auto sf = dataset->datasetGetObservations(i);
    ASSERT_(sf);

    mrpt::obs::CObservation::Ptr obs =
      sf->getObservationByClass<mrpt::obs::CObservationPointCloud>();
    if (!obs) obs = sf->getObservationByClass<mrpt::obs::CObservationIMU>();
    if (!obs) continue;

    if (obs->GetRuntimeClass() == CLASS_ID(mrpt::obs::CObservationPointCloud)) nLidarFed++;

    glim->onNewObservation(obs);

    // No-drop guarantee: wait for the worker to fully drain this scan
    // before feeding the next one (mirrors mola-lidar-odometry-cli.cpp).
    while (glim->isBusy()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // No `cnt = 0` reset inside the branch: that makes `cnt++ % 100` fire on
    // every iteration, which on an IMU-heavy dataset is hundreds of thousands
    // of progress lines (tens of MB) in a CI log.
    static int cnt = 0;
    if (cnt++ % 100 == 0) {
      const size_t N = (dataset->datasetSize() - 1);
      const double pc = N > 0 ? static_cast<double>(i) / static_cast<double>(N) : 1.0;
      const double tNow = mrpt::Clock::nowDouble();
      const double ETA = pc > 0 ? (tNow - tStart) * (1.0 / pc - 1) : .0;
      const double totalTime = ETA + (tNow - tStart);

      std::cout << "\033[A\33[2KT\r" << mrpt::system::progress(pc, 30)
                << mrpt::format(
                     " %6zu/%6zu (%.02f%%) ETA=%s/T=%s | scans fed=%zu\n", i, N, 100 * pc,
                     mrpt::system::formatTimeInterval(ETA).c_str(),
                     mrpt::system::formatTimeInterval(totalTime).c_str(), nLidarFed);
      std::cout.flush();
    }
  }

  std::cout << "\nDone. Dataset entries processed: " << lastDatasetEntry
            << ", LiDAR scans fed: " << nLidarFed << "\n";

  if (cli.arg_outPath_set) {
    const auto fil = cli.arg_outPath;
    std::cout << "Saving estimated path in TUM format to: " << fil << "\n";
    glim->estimatedTrajectory().saveToTextFile_TUM(fil);
  }

  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    Cli cli;
    CLI11_PARSE(cli.cmd, argc, argv);
    cli.afterParse();

    if (cli.arg_plugins_set) {
      std::string errMsg;
      const auto plugins = cli.arg_plugins;
      std::cout << "Loading plugin(s): " << plugins << "\n";
      if (!mrpt::system::loadPluginModules(plugins, errMsg)) {
        std::cerr << errMsg << std::endl;
        return 1;
      }
    }

    mola_install_signal_handler();
    return main_odometry(cli);
  } catch (std::exception & e) {
    mola::pretty_print_exception(e, "Exit due to exception:");
    return 1;
  }
}
