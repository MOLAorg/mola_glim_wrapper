#include <gtsam/config.h>

// The factor-graph dump requires a GTSAM built with boost serialization
// enabled; some distributions ship it disabled. It is used only by the
// global-mapping save() path, so a warning stub keeps the library usable.
#if defined(GTSAM_ENABLE_BOOST_SERIALIZATION) && GTSAM_ENABLE_BOOST_SERIALIZATION

#include <glim/util/serialization.hpp>

#include <spdlog/spdlog.h>
#include <gtsam/base/serialization.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace glim {

void serializeToBinaryFile(const gtsam::NonlinearFactorGraph& graph, const std::string& path, bool only_serializable) {
  try {
    gtsam::serializeToBinaryFile(graph, path);
    return;
  } catch (boost::archive::archive_exception e) {
    spdlog::warn("failed to serialize factor graph!!");
    spdlog::warn(e.what());

    if (!only_serializable) {
      throw e;
    }
  }

  spdlog::warn("retrying to serialize factor graph with only serializable factor types");

  gtsam::NonlinearFactorGraph ser;
  for (const auto& factor : graph) {
    try {
      gtsam::serializeBinary(factor);
      ser.add(factor);
    } catch (boost::archive::archive_exception e) {
      factor->print();
    }
  }

  gtsam::serializeToBinaryFile(ser, path);
}

void serializeToBinaryFile(const gtsam::Values& values, const std::string& path, bool only_serializable) {
  try {
    gtsam::serializeToBinaryFile(values, path);
    return;
  } catch (boost::archive::archive_exception e) {
    spdlog::warn("failed to serialize values!!");
    spdlog::warn(e.what());

    if (!only_serializable) {
      throw e;
    }
  }

  spdlog::warn("retrying to serialize values with only serializable value types");

  gtsam::Values ser;
  for (const auto& value : values) {
    try {
      gtsam::serializeBinary(value.value);
      ser.insert(value.key, value.value);
    } catch (boost::archive::archive_exception e) {
      std::cout << "key=" << value.key << std::endl;
      value.value.print();
    }
  }

  gtsam::serializeToBinaryFile(ser, path);
}

void deserializeFromBinaryFile(const std::string& path, gtsam::NonlinearFactorGraph& graph) {
  gtsam::deserializeFromBinaryFile(path, graph);
}

void deserializeFromBinaryFile(const std::string& path, gtsam::Values& values) {
  gtsam::deserializeFromBinaryFile(path, values);
}

}  // namespace glim


#else

#include <glim/util/serialization.hpp>
#include <spdlog/spdlog.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <stdexcept>

namespace glim {

void serializeToBinaryFile(const gtsam::NonlinearFactorGraph&, const std::string& path, bool) {
  spdlog::warn("factor graph serialization is disabled in this GTSAM build; skipping {}", path);
}

void serializeToBinaryFile(const gtsam::Values&, const std::string& path, bool) {
  spdlog::warn("values serialization is disabled in this GTSAM build; skipping {}", path);
}

void deserializeFromBinaryFile(const std::string& path, gtsam::NonlinearFactorGraph&) {
  throw std::runtime_error("factor graph deserialization is disabled in this GTSAM build: " + path);
}

void deserializeFromBinaryFile(const std::string& path, gtsam::Values&) {
  throw std::runtime_error("values deserialization is disabled in this GTSAM build: " + path);
}

}  // namespace glim

#endif
