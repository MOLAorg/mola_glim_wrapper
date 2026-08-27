# ------------------------------------------------------------------------------
#  glim_core: the MOLA-free, MRPT-free facade over the vendored GLIM algorithm.
#
#  Copyright (C) 2026, Jose Luis Blanco-Claraco
#  Released under the BSD 3-Clause License. See the LICENSE file at the
#  repository root.
#  See THIRD_PARTY_NOTICES.md for the licensing terms of third_party/glim and
#  third_party/gtsam_points.
# ------------------------------------------------------------------------------

add_library(glim_core STATIC
  src/GlimCore.cpp
)

# Position-independent: this static library is absorbed into the wrapper's
# shared module library.
set_target_properties(glim_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(glim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# glim (and, through it, gtsam_points and GTSAM) is a PRIVATE dependency:
# include/glim_core/*.h deliberately exposes nothing from it, which is what
# keeps GTSAM headers out of every consumer of this wrapper.
target_link_libraries(glim_core
  PUBLIC  Eigen3::Eigen
  PRIVATE glim
)

target_compile_features(glim_core PUBLIC cxx_std_17)

if(DETECTED_ROS2 AND BUILD_TESTING AND ament_cmake_gtest_FOUND)
  ament_add_gtest(test_glim_core test/test_glim_core.cpp)
  if(TARGET test_glim_core)
    target_link_libraries(test_glim_core glim_core)
    target_compile_definitions(test_glim_core PRIVATE
      GLIM_CORE_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../glim-config")
  endif()
endif()
