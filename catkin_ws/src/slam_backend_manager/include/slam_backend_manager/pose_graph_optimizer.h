#pragma once

#include <map>
#include <string>
#include <vector>

namespace slam_backend_manager {

struct ChainageNode {
  std::string keyframe_id;
  double chainage_m = 0.0;
};

struct LoopConstraint {
  std::string from_keyframe_id;
  std::string to_keyframe_id;
  double relative_chainage_m = 0.0;
  double weight = 1.0;
  double stddev_m = -1.0;
  double robust_kernel_delta = 0.0;
};

struct Pose6D {
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double yaw_deg = 0.0;
};

struct PoseGraphNode6D {
  std::string keyframe_id;
  Pose6D pose;
};

struct PoseGraphConstraint6D {
  std::string from_keyframe_id;
  std::string to_keyframe_id;
  Pose6D relative_pose;
  double weight = 1.0;
  double translation_stddev_m = -1.0;
  double rotation_stddev_deg = -1.0;
  double robust_kernel_delta = 0.0;
};

struct PoseGraphOptimizerConfig {
  int max_iterations = 5;
  double implicit_odometry_translation_stddev_m = 10.0;
  double implicit_odometry_rotation_stddev_deg = 100.0;
  double smoothness_translation_stddev_m = 1.0;
  double smoothness_rotation_stddev_deg = 1.0;
  bool enable_ceres = true;
  bool enable_smoothness_prior = true;
};

std::map<std::string, double> optimizeChainageGraph(const std::vector<ChainageNode>& nodes,
                                                    const std::vector<LoopConstraint>& loop_constraints,
                                                    int iterations = 5);

std::map<std::string, Pose6D> optimizePoseGraph6D(
    const std::vector<PoseGraphNode6D>& nodes,
    const std::vector<PoseGraphConstraint6D>& constraints,
    int iterations = 5);

std::map<std::string, Pose6D> optimizePoseGraph6D(
    const std::vector<PoseGraphNode6D>& nodes,
    const std::vector<PoseGraphConstraint6D>& constraints,
    const PoseGraphOptimizerConfig& config);

}  // namespace slam_backend_manager
