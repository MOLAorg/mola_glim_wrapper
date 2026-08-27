# mola_glim_wrapper

MOLA wrapper for [GLIM](https://github.com/koide3/glim), Kenji Koide's
LiDAR-inertial mapping framework.

It provides:

- `mola::GlimOdometry`, a MOLA front-end module (`FrontEndBase` +
  `LocalizationSourceBase` + `MapSourceBase`), loadable from a `mola-cli`
  launch YAML.
- `mola-glim-cli`, an offline, loss-free CLI over the MOLA dataset sources:
  KITTI, rosbag1, rosbag2, MulRan and rawlog.
- Pipeline YAMLs for Oxford Spires, KITTI, BotanicGarden and Newer College.

## Odometry only, on purpose

Upstream GLIM is a full SLAM system: an odometry front end, a sub-mapping
stage, and a global-mapping back end with loop closure and global
optimization. **This wrapper instantiates only the odometry stage**
(`glim::OdometryEstimationCPU`, a fixed-lag factor-graph estimator over
GICP/VGICP scan-to-model factors and preintegrated IMU factors).

That is deliberate. The wrapper exists to sit next to `mola_dlio_wrapper`,
`mola_fast_lio2_wrapper` and `mola_kiss_icp_wrapper` in a cross-method
odometry comparison, and a loop-closed, globally-optimized trajectory is not
comparable with those. If you want GLIM's full SLAM output, run upstream
GLIM.

## Build

Everything it needs is vendored under `third_party/` (see
`THIRD_PARTY_NOTICES.md`), so there is nothing to install first beyond the
system packages below:

- GTSAM >= 4.2 with `gtsam_unstable` (on ROS distros: `ros-$ROS_DISTRO-gtsam`)
- Eigen 3, Boost (graph, filesystem, serialization), spdlog, OpenMP
- MRPT and the MOLA core packages

```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-select mola_glim_wrapper \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

The build compiles the vendored `gtsam_points` and `glim` as well, so a cold
build takes a couple of minutes. CUDA, the Iridescence viewer and OpenCV are
all switched off: none is available on the target hosts, and none is needed
for odometry.

## Run: offline CLI

```bash
# Oxford Spires
SEQ=/mnt/datasets/public/oxford-spires/2024-03-13-observatory-quarter-01
BAG=$SEQ/raw/ros2bag/1710338090_2024-03-13-13-54-51
env LIDAR_POSE_X=0 LIDAR_POSE_Y=0 LIDAR_POSE_Z=0.124 \
    LIDAR_POSE_YAW=180 LIDAR_POSE_PITCH=0 LIDAR_POSE_ROLL=0 \
    MOLA_USE_FIXED_LIDAR_POSE=true \
    IMU_POSE_X=-0.018771 IMU_POSE_Y=0.008218 IMU_POSE_Z=0.053526 \
    IMU_POSE_YAW=89.3737 IMU_POSE_PITCH=-0.1665 IMU_POSE_ROLL=-0.1287 \
    MOLA_USE_FIXED_IMU_POSE=true \
  mola-glim-cli \
    -c $(ros2 pkg prefix mola_glim_wrapper)/share/mola_glim_wrapper/pipelines/glim-oxford-spires.yaml \
    --input-rosbag2 "$BAG" \
    --lidar-topic /hesai/pandar --imu-topic /alphasense_driver_ros/imu \
    --output-tum-path /tmp/glim_oq01.tum

# KITTI odometry benchmark
KITTI_BASE_DIR=/mnt/datasets/public/kitti mola-glim-cli \
  -c .../pipelines/glim-kitti.yaml --input-kitti-seq 04 \
  --output-tum-path /tmp/glim_kitti04.tum

# A ROS 1 bag (BotanicGarden). A comma-separated list of bags is replayed as
# one continuous sequence.
mola-glim-cli -c .../pipelines/glim-botanic.yaml \
  --input-rosbag1 /path/to/1005_00_LIO.bag \
  --lidar-topic /velodyne_points --imu-topic /imu/data \
  --output-tum-path /tmp/glim_botanic.tum
```

`--output-tum-path` also writes `<base>_gt.tum` when the dataset module
provides ground truth.

## Run: online, through `mola-cli`

```bash
mola-glim-gui-oxford-spires /path/to/sequences/<sequence-name>/ [--headless]
```

which fills in the multipart bag slots and launches
`mola-cli-launchs/glim_from_oxford_spires.yaml`. Set
`MOLA_SAVE_TRAJECTORY=true` and `MOLA_TUM_TRAJECTORY_OUTPUT=<path>` to export
a trajectory from this path too.

Note the online path is **not** loss-free: scans go through a
`POLICY_DROP_OLD` worker pool, so an overloaded host drops them. The offline
CLI feeds one observation at a time and waits on `isBusy()`, which is what
makes it lossless.

## Parameters

The pipeline YAML is the source of truth for everything this wrapper
exposes; the JSON directory in `glim-config/` supplies the handful of GLIM
parameters that have no programmatic path in (the crop-box filter, the
initialization window, marginal covariances). YAML defaults match upstream's
JSON defaults, so an unset key changes nothing.

| Key | What it is |
|---|---|
| `imu_pose_in_lidar` | GLIM's `T_lidar_imu`: the IMU frame expressed in the LiDAR frame. Goes in **uninverted** — unlike the Fast-LIO2 wrapper, whose extrinsic is the inverse of this one. |
| `baselink2lidar_pose_str` | `T_base_lidar`. Identity when the LiDAR is the reference frame. |
| `registration_type` | `GICP` (iVox target) or `VGICP` (voxelmap target) |
| `ivox_resolution`, `ivox_min_dist` | GICP target-map resolution |
| `vgicp_resolution`, `vgicp_voxelmap_levels` | VGICP target-map resolution |
| `distance_near_thresh`, `distance_far_thresh` | Range crop |
| `downsample_resolution`, `downsample_target`, `downsample_rate` | Input downsampling |
| `imu_acc_noise`, `imu_gyro_noise`, `imu_int_noise`, `imu_bias_noise` | IMU preintegration noises |
| `initialization_mode` | `LOOSE` (scan-matching bootstrap over an initial window) or `NAIVE` |
| `initialization_window_size` | Seconds of LOOSE bootstrapping, and therefore how much trajectory is missing from the head of every run |
| `k_correspondences` | Neighbors used for the per-point covariance estimate |
| `smoother_lag` | Fixed-lag smoothing window, seconds |
| `num_threads` | Preprocessing and per-factor parallelism |
| `glim_config_path` | GLIM's JSON config dir; empty resolves to the installed `glim-config/` |

Every one is overridable from the environment through the `${VAR|default}`
placeholders in the shipped pipelines (`GLIM_IVOX_RESOLUTION`,
`GLIM_NUM_THREADS`, ...).

`MOLA_GLIM_DUMP_ESTIMATED_GRAV=<path>` writes `<time> <gx> <gy> <gz>` per
scan, the same format the other wrappers' gravity dumps use. GLIM does not
estimate gravity as a filter state, so what this reports is the world gravity
rotated by the estimated attitude.

## Things worth knowing

- **GLIM stamps each pose at the START of its scan**, where Fast-LIO2 stamps
  at the end and MOLA-LO and DLIO at the middle. Against a reference on a
  different convention this is a half- or whole-period shift, which an
  aligned APE mostly absorbs and every relative metric does not. Measure it
  per dataset (sweep the applied offset, or check the nearest-tick residual)
  rather than assuming.
- **The first few scans produce no pose at all.** `initialization_mode: LOOSE`
  spends `initialization_window_size` seconds bootstrapping the initial
  attitude and velocity; `OdometryEstimationBase::insert_frame()` returns null
  throughout. At the shipped 3.0 s that is ~30 scans missing from the head of
  the trajectory on a 10 Hz sensor.

  Shortening the window buys the head back and costs accuracy, monotonically.
  Measured on kitti-04 (271 scans):

  | `initialization_window_size` | poses | first stamp | APE RMSE |
  |---|---|---|---|
  | 0.2 | 269 | 0.108 s | 0.239 m |
  | 0.5 | 266 | 0.421 s | 0.207 m |
  | 1.0 | 261 | 0.942 s | 0.201 m |
  | 3.0 (shipped) | 242 | 2.921 s | 0.198 m |

  The shipped value is upstream's. It only costs meaningful coverage on short
  sequences -- 3 s is 11% of kitti-04 and under 1% of kitti-00 -- so a
  consumer that gates on trajectory coverage wants a floor for this method
  rather than a shorter window here.

- **`NAIVE` initialization is not a cheaper `LOOSE`.** It buys back exactly
  one scan (kitti-04 242 -> 241, oxford-spires 2863 -> 2864) and costs
  accuracy on both (kitti-04 0.182 -> 0.193 m, oxford-spires
  0.063 -> 0.096 m).
- **`k_correspondences` matters more than `ivox_resolution`, and upstream's
  default of 10 is too few for a sparse LiDAR.** On BotanicGarden 1005_00
  (16-ring VLP-16), scored the way the evaluation harness does (ground-truth
  body offset, +0.05 s onto the reference clock, all 5760 poses associated),
  one line per run:

  | | `k_correspondences` 10 | `k_correspondences` 20 |
  |---|---|---|
  | `ivox_resolution` 1.0 | 2.23 / 2.58 / 2.15 / 1.61 m | 2.26 m |
  | `ivox_resolution` 0.5 | **17.9** m | 1.19 m |
  | `ivox_resolution` 0.3 | 2.86 m | 1.82 / 0.69 / 0.98 m |
  | `ivox_resolution` 0.2 | -- | 0.36 m |

  Two things follow. A finer target map only helps once the covariances are
  well conditioned: at 10 neighbors, going from 1.0 m to 0.3 m makes things
  slightly worse and the single 0.5 m run diverged outright, while at 20
  neighbors the same change roughly halves the mean error and nothing
  diverged in seven runs. Ten neighbors is not enough to condition a GICP
  covariance on 16 rings, which is why `glim-botanic.yaml` ships 20 and
  0.3 m. (The 17.9 m cell is one run, not a measured divergence rate.)

  Read that table with the method's run-to-run spread in mind: GLIM
  randomizes its grid downsampling and evaluates factors under OpenMP, so
  repeated runs of one configuration differ by roughly a third here (and by
  a few percent on KITTI). The 1.0/10 and 0.3/20 replicate ranges above
  overlap slightly; the ~2x difference in their means is the claim, not any
  single pair of numbers.

- **None of that transfers to a dense or an automotive LiDAR**, which is why
  only the BotanicGarden pipeline moves off upstream's values. Oxford Spires
  observatory-quarter-01 (64-ring Hesai QT64) is best exactly where it ships:
  0.063 m at `ivox_resolution` 0.5 / `k_correspondences` 10, against 0.071 m
  at k 20 and 0.078 m at 0.3 m. On KITTI the same sweep contradicts itself
  between sequences and inside the noise -- kitti-08 (3.2 km) is best at the
  shipped 1.0 / 10 (2.89 m, against 3.19 m at 0.5/20 and 3.66 m at 0.3/20),
  while kitti-00 replicates of one configuration span 3.9-5.4 m -- so the
  KITTI pipeline keeps upstream's defaults and the first numbers it produces
  are unretuned ones.
- **KITTI has no IMU**, so `--input-kitti-seq` wraps the dataset in a
  decorator that synthesizes a constant-gravity, zero-rate IMU sample before
  each scan — the same technique, constant and rate as the DLIO and Fast-LIO2
  wrappers, so the methods are fed identically.
  `MOLA_KITTI_SYNTH_IMU_PER_SCAN=N` raises the rate; measured on kitti-04 it
  makes things slightly worse (0.197 m at N=10 against 0.182 m at N=1) and
  breaks that comparability, so leave it alone unless you are studying it.

## License

GPL-3.0. See `LICENSE`, and `THIRD_PARTY_NOTICES.md` for the vendored MIT /
BSD-2-Clause code under `third_party/`.
