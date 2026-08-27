#pragma once

#include <string>

namespace gtsam {
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace glim {

void serializeToBinaryFile(const gtsam::NonlinearFactorGraph& graph, const std::string& path, bool only_serializable = true);
void serializeToBinaryFile(const gtsam::Values& values, const std::string& path, bool only_serializable = true);

void deserializeFromBinaryFile(const std::string& path, gtsam::NonlinearFactorGraph& graph);
void deserializeFromBinaryFile(const std::string& path, gtsam::Values& values);

}  // namespace glim
