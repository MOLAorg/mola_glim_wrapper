# Third-party notices

`mola_glim_wrapper` itself is BSD-3-Clause (see `LICENSE`). It vendors,
under `third_party/`, code from the projects below. Their own licenses apply
to their own files and are reproduced in each subdirectory's `LICENSE`.

---

## `third_party/glim`

- **Upstream:** https://github.com/koide3/glim
- **Commit copied:** `bba7bf407844cd7db36f4c9a54d58298c9b044f6` (2026-08-18,
  "Fix normal coloring for odometry scans and keyframes (#321)"), version 1.2.2
- **License:** MIT (`third_party/glim/LICENSE`)
- **Copyright:** (c) 2024 Kenji Koide (koide3), and the GLIM contributors.
  Bundles `thirdparty/json` (nlohmann/json, MIT, (c) 2013-2023 Niels
  Lohmann), kept verbatim.

Vendored by copy rather than as a submodule because it needed the edits
below. Every change is either a build-integration change or a portability
fix; none of them touches the estimation math.

1. **`src/glim/util/serialization.cpp`, `src/glim/util/export_factors.cpp`,
   `src/glim/mapping/global_mapping.cpp`, `include/glim/util/serialization.hpp`
   — factor-graph (de)serialization made optional.** The GTSAM this host
   provides (`ros-lyrical-gtsam` 4.3.0) is built with
   `GTSAM_ENABLE_BOOST_SERIALIZATION 0`, so `gtsam::serializeToBinaryFile` and
   friends do not exist and those translation units do not compile. Both files
   are now guarded on that macro, with warning/throwing stubs on the disabled
   side, and `global_mapping.cpp`'s two `gtsam::deserializeFromBinaryFile`
   calls go through new `glim::deserializeFromBinaryFile` wrappers so the
   guard lives in one place. Only GLIM's global-mapping `save()`/`load()` path
   uses any of this, and this wrapper does not instantiate global mapping.

2. **`include/glim/odometry/odometry_estimation_imu.hpp`,
   `src/glim/odometry/odometry_estimation_imu.cpp` — IMU preintegration noises
   exposed on the params struct.** `imu_acc_noise`, `imu_gyro_noise` and
   `imu_int_noise` were reachable only through `config_sensors.json`, because
   `OdometryEstimationIMU` constructed a default `IMUIntegrationParams` which
   read them itself. They are now fields of `OdometryEstimationIMUParams`,
   defaulted from the same JSON keys with the same fallbacks, and passed
   explicitly into `IMUIntegration`. Behavior is unchanged for a caller that
   does not set them; the point is that the MOLA pipeline YAML can.

3. **`include/glim/odometry/odometry_estimation_imu.hpp`,
   `src/glim/odometry/odometry_estimation_imu.cpp`,
   `include/glim/odometry/loose_initial_state_estimation.hpp` — the LOOSE
   initialization window exposed on the params struct.** Same shape and same
   reason as the change above: `initialization_window_size` was reachable only
   through `config_odometry.json`, because `LooseInitialStateEstimation` read
   it in its own constructor. It is now a field of
   `OdometryEstimationIMUParams`, defaulted from the same JSON key with the
   same fallback, and applied through a new
   `LooseInitialStateEstimation::set_window_size()`. It decides how much
   trajectory is missing from the head of every run, so it has to be settable
   from the pipeline YAML; behavior is unchanged for a caller that leaves it
   alone.

4. **`CMakeLists.txt` — consumable as an `add_subdirectory()`.**
   `find_package(gtsam_points ...)` is skipped when the target already exists
   (it does, since this package builds `gtsam_points` first), and the
   dlopen-able estimation-module `.so` loop, the install/export rules and the
   ROS/ament section are all gated on `GLIM_VENDORED_IN_MOLA`, which this
   package sets. Nothing is gated on it in a standalone build, so the tree
   still builds the way upstream intends.

5. **Deleted, not modified:** `src/glim/viewer/`, `include/glim/viewer/`
   (need Iridescence, which is not packaged for this distro and is not built),
   plus `docs/`, `docker/`, `mkdocs.yml` and `package.xml` (this package
   supplies its own).

## `third_party/gtsam_points`

- **Upstream:** https://github.com/koide3/gtsam_points
- **Commit copied:** `76437e4ddb03df98a4f8a875c8de28edb0394c7c` (2026-08-02,
  "Merge pull request #103 from koide3/has_index"), version 1.2.2
- **License:** MIT (`third_party/gtsam_points/LICENSE`)
- **Copyright:** (c) 2024 Kenji Koide (koide3), and the gtsam_points
  contributors.
  Bundles `thirdparty/nanoflann` (nanoflann, BSD-2-Clause, (c) Jose Luis
  Blanco-Claraco and contributors), kept verbatim.

1. **`include/gtsam_points/factors/linear_damping_factor.hpp`,
   `include/gtsam_points/factors/rotate_vector3_factor.hpp` — missing includes.**
   Both call `boost::serialization::make_nvp` / `base_object` without
   including `<boost/serialization/nvp.hpp>` / `<boost/serialization/base_object.hpp>`;
   Boost 1.90 no longer pulls them in transitively, so both headers fail to
   compile. The includes were added.

2. **`include/gtsam_points/config.hpp.in` — version macros.** The template
   expanded `@CMAKE_PROJECT_VERSION_*@`, which names the *top-level* project.
   In a standalone build that is `gtsam_points` itself; vendored, it is
   `mola_glim_wrapper`, whose `project()` declares no version, so every
   `GTSAM_POINTS_VERSION_*` macro expanded to nothing and `debug.cpp` failed
   to compile. Changed to the directory-scoped `@PROJECT_VERSION_*@`, which is
   correct in both cases.

3. **`CMakeLists.txt` — consumable as an `add_subdirectory()`.** Adds the
   `gtsam_points::gtsam_points` ALIAS that the exported package config would
   otherwise be the only source of, and gates the install/export and CPack
   sections on `GTSAM_POINTS_VENDORED_IN_MOLA`.

4. **Deleted, not modified:** `data/` (9.3 MB of test fixtures), `test/`,
   `demo/`, `example/`, `tools/`, `docs/`, `docker/` — none are built here.

## `third_party/glim_core`

Not vendored: original code written for this package (BSD-3-Clause, same as
the rest of it). It is the plain-data facade that keeps GTSAM, gtsam_points
and GLIM headers out of everything above it.

---

## License compatibility

Everything vendored here is permissive -- MIT (GLIM, gtsam_points,
nlohmann/json) or BSD-2-Clause (nanoflann) -- so none of it constrains this
package's own BSD-3-Clause terms, and redistribution needs only the copyright
notices reproduced above and in each subdirectory's `LICENSE`. The external
build dependencies are permissive too: MRPT and GTSAM are BSD-3-Clause,
spdlog is MIT, Boost is BSL-1.0, Eigen is MPL-2.0. This is unlike
`mola_fast_lio2_wrapper`, whose upstream is GPL-2.0-only; there is no
equivalent open question here.

The one restriction is on the **binaries**, and it comes from MOLA, not from
`third_party/`: `mola_kernel` and the `mola_input_*` dataset readers (except
`mola_input_rosbag1`) are GPL-3.0. BSD-3-Clause is GPL-compatible, so the
combination is fine, but it is a combined work that has to be distributed
under GPL-3.0. The sources in this repository remain BSD-3-Clause and are
reusable as such -- `glim_core/` in particular depends on neither MOLA nor
MRPT.
