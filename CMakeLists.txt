# ------------------------------------------------------------------------------
#                          mola_glim_wrapper
#
# Copyright (C) 2026, Jose Luis Blanco-Claraco
# All rights reserved.
# Released under GNU GPL v3. See LICENSE file.
# See THIRD_PARTY_NOTICES.md for the licensing terms of the vendored
# GLIM / gtsam_points code under third_party/.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.22...3.31)  # Ubuntu 22.04+

if("$ENV{ROS_VERSION}" STREQUAL "2")
    set(DETECTED_ROS2 TRUE)
endif()

project(mola_glim_wrapper LANGUAGES CXX)

# MOLA CMake scripts: "mola_xxx()"
find_package(mola_common REQUIRED)

# find dependencies:
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS graph filesystem serialization)
find_package(GTSAM 4.2 REQUIRED)
find_package(GTSAM_UNSTABLE 4.2 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(spdlog REQUIRED)
find_package(mrpt-maps REQUIRED)
find_package(mrpt-obs REQUIRED)
find_package(mrpt-tclap REQUIRED)
find_package(mola_kernel REQUIRED)
find_package(mola_yaml REQUIRED)

# ament_cmake / BUILD_TESTING must be set up before the add_subdirectory()
# calls below, since glim_core's CMakeLists.txt uses ament_add_gtest():
if(DETECTED_ROS2)
  find_package(ament_cmake REQUIRED)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

# -----------------------------------------------------------------------------
#  Vendored upstream: gtsam_points, then glim (which depends on it).
#
#  Both keep their own upstream CMakeLists.txt; the *_VENDORED_IN_MOLA flags
#  below switch off their install/export/packaging sections, which only make
#  sense for a standalone build. See THIRD_PARTY_NOTICES.md.
# -----------------------------------------------------------------------------
set(GTSAM_POINTS_VENDORED_IN_MOLA ON)
set(GLIM_VENDORED_IN_MOLA ON)

# There is no GPU on the target hosts, and the viewer needs Iridescence, which
# is not packaged for this distro. OpenCV is only used by GLIM's camera
# extension modules, which this wrapper does not build.
set(BUILD_WITH_CUDA OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_VIEWER OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_OPENCV OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_TBB OFF CACHE BOOL "" FORCE)
set(BUILD_WITH_OPENMP ON CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_DEMO OFF CACHE BOOL "" FORCE)
set(BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)

add_subdirectory(third_party/gtsam_points)
add_subdirectory(third_party/glim)

# Their own install rules are switched off above, so install just the two
# shared libraries here: everything in this package links against them at
# runtime, and colcon puts this prefix's lib/ on LD_LIBRARY_PATH.
install(TARGETS gtsam_points glim
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# -----------------------------------------------------------------------------
#  glim_core: the ROS-free, MOLA-free facade over GLIM's odometry.
# -----------------------------------------------------------------------------
add_subdirectory(third_party/glim_core)

# -----------------------------------------------------------------------------
#  mola_glim_wrapper: the online mola-cli module
# -----------------------------------------------------------------------------
mola_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES
    module/src/GlimOdometry.cpp
    module/src/GlimOdometry_Adapters.cpp
    # Last:
    module/src/register.cpp
  PUBLIC_LINK_LIBRARIES
    mrpt::maps
    mrpt::obs
    mola::mola_kernel
    mola::mola_yaml
  PRIVATE_LINK_LIBRARIES
    # glim_core is a plain (non ament/non exported) static lib: link it
    # PRIVATE so `install(EXPORT ...)` doesn't need to export it too. Only
    # GlimOdometry.cpp's own compilation needs its full API; the public
    # header only needs the light Types.hpp.
    glim_core
  CMAKE_DEPENDENCIES
    mola_kernel
    mrpt-maps
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/module/include>
  # module/include/.../GlimOdometry.h (installed, public) only needs the
  # light, GTSAM-free glim_core/Types.hpp, so its include dir is PUBLIC too:
  # needed by apps/mola-glim-cli.cpp (a separate target below).
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/glim_core/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
    module/src
)
# Where GLIM's own JSON config directory lands, so that a pipeline YAML that
# leaves `glim_config_path` empty still finds it.
target_compile_definitions(${PROJECT_NAME} PRIVATE
  MOLA_GLIM_DEFAULT_CONFIG_DIR="${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/glim-config")

install(
  DIRECTORY module/include/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
)

# Install pipeline / mola-cli launch YAMLs, plus GLIM's own JSON config dir:
install(
  DIRECTORY
    mola-cli-launchs
    pipelines
    glim-config
  DESTINATION
    share/${PROJECT_NAME}
)

# Install gui-* convenience scripts:
install(
  PROGRAMS
    scripts/mola-glim-gui-oxford-spires
  DESTINATION
    bin
)

# -----------------------------------------------------------------------------
#  Tests for the wrapper module itself (glim_core has its own, next to its
#  sources). Only the frame conventions so far: pure pose algebra that needs
#  no estimator, and getting it wrong is invisible in APE.
# -----------------------------------------------------------------------------
if(DETECTED_ROS2 AND BUILD_TESTING AND ament_cmake_gtest_FOUND)
  ament_add_gtest(test_glim_frame_conventions module/test/test_frame_conventions.cpp)
  if(TARGET test_glim_frame_conventions)
    target_link_libraries(test_glim_frame_conventions ${PROJECT_NAME})
  endif()
endif()

# -----------------------------------------------------------------------------
#  mola-glim-cli: the offline, lossless CLI app
# -----------------------------------------------------------------------------
find_package(mola_input_mulran_dataset) # optional
find_package(mola_input_rawlog) # optional
find_package(mola_input_rosbag2) # optional
find_package(mola_input_rosbag1) # optional
find_package(mola_input_kitti_dataset) # optional

mola_add_executable(
  TARGET  mola-glim-cli
  SOURCES apps/mola-glim-cli.cpp
  LINK_LIBRARIES
    ${PROJECT_NAME}
    mrpt::maps
    mrpt::tclap
)

if(mola_input_mulran_dataset_FOUND)
  message(STATUS "mola-glim-cli: With Mulran = YES")
  target_link_libraries(mola-glim-cli mola::mola_input_mulran_dataset)
  target_compile_definitions(mola-glim-cli PRIVATE HAVE_MOLA_INPUT_MULRAN)
else()
  message(STATUS "mola-glim-cli: With Mulran = NO")
endif()

if(mola_input_rawlog_FOUND)
  message(STATUS "mola-glim-cli: With Rawlog = YES")
  target_link_libraries(mola-glim-cli mola::mola_input_rawlog)
  target_compile_definitions(mola-glim-cli PRIVATE HAVE_MOLA_INPUT_RAWLOG)
else()
  message(STATUS "mola-glim-cli: With Rawlog = NO")
endif()

if(mola_input_rosbag2_FOUND)
  message(STATUS "mola-glim-cli: With rosbag2 = YES")
  target_link_libraries(mola-glim-cli mola::mola_input_rosbag2)
  target_compile_definitions(mola-glim-cli PRIVATE HAVE_MOLA_INPUT_ROSBAG2)
else()
  message(STATUS "mola-glim-cli: With rosbag2 = NO")
endif()

if(mola_input_rosbag1_FOUND)
  message(STATUS "mola-glim-cli: With rosbag1 = YES")
  target_link_libraries(mola-glim-cli mola::mola_input_rosbag1)
  target_compile_definitions(mola-glim-cli PRIVATE HAVE_MOLA_INPUT_ROSBAG1)
else()
  message(STATUS "mola-glim-cli: With rosbag1 = NO")
endif()

if(mola_input_kitti_dataset_FOUND)
  message(STATUS "mola-glim-cli: With KITTI = YES")
  target_link_libraries(mola-glim-cli mola::mola_input_kitti_dataset)
  target_compile_definitions(mola-glim-cli PRIVATE HAVE_MOLA_INPUT_KITTI)
else()
  message(STATUS "mola-glim-cli: With KITTI = NO")
endif()

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if(DETECTED_ROS2)
  ament_package()
endif()
