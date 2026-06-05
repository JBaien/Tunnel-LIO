#include "slam_backend_manager/pose_graph_optimizer.h"

#include <ceres/ceres.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace slam_backend_manager {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

double clampWeight(double weight) {
  return std::max(0.0, std::min(1.0, weight));
}

bool validWeight(double weight) {
  return std::isfinite(weight) && weight >= 0.0 && weight <= 1.0;
}

bool validOptionalStddev(double stddev) {
  return std::isfinite(stddev) &&
         (stddev == -1.0 || stddev > 0.0);
}

bool validRobustKernelDelta(double delta) {
  return std::isfinite(delta) && delta >= 0.0;
}

bool validKeyframeId(const std::string& value) {
  if (value.empty() || value == "." || value == "..") {
    return false;
  }
  for (const char character : value) {
    const bool uppercase = character >= 'A' && character <= 'Z';
    const bool lowercase = character >= 'a' && character <= 'z';
    const bool digit = character >= '0' && character <= '9';
    if (!uppercase && !lowercase && !digit && character != '_' &&
        character != '-' && character != '.') {
      return false;
    }
  }
  return true;
}

double positiveOrDefault(const double value, const double fallback) {
  return value > 0.0 ? value : fallback;
}

double robustScale(const double normalized_residual, const double delta) {
  if (delta <= 0.0 || normalized_residual <= delta) {
    return 1.0;
  }
  return delta / normalized_residual;
}

double chainageConstraintWeight(const LoopConstraint& constraint,
                                const double error_m) {
  const double stddev = positiveOrDefault(constraint.stddev_m, 1.0);
  const double normalized_residual = std::abs(error_m) / stddev;
  const double covariance_weight =
      constraint.stddev_m > 0.0 ? 1.0 / (1.0 + stddev) : 1.0;
  return clampWeight(constraint.weight * covariance_weight *
                     robustScale(normalized_residual,
                                 constraint.robust_kernel_delta));
}

double normalizeAngleDeg(double value) {
  while (value > 180.0) {
    value -= 360.0;
  }
  while (value < -180.0) {
    value += 360.0;
  }
  return value;
}

double normalizeAngleRad(double value) {
  while (value > kPi) {
    value -= 2.0 * kPi;
  }
  while (value < -kPi) {
    value += 2.0 * kPi;
  }
  return value;
}

Pose6D addPose(const Pose6D& lhs, const Pose6D& rhs) {
  Pose6D result;
  result.x_m = lhs.x_m + rhs.x_m;
  result.y_m = lhs.y_m + rhs.y_m;
  result.z_m = lhs.z_m + rhs.z_m;
  result.roll_deg = normalizeAngleDeg(lhs.roll_deg + rhs.roll_deg);
  result.pitch_deg = normalizeAngleDeg(lhs.pitch_deg + rhs.pitch_deg);
  result.yaw_deg = normalizeAngleDeg(lhs.yaw_deg + rhs.yaw_deg);
  return result;
}

Pose6D poseFromParameters(const double* parameters) {
  Pose6D pose;
  pose.x_m = parameters[0];
  pose.y_m = parameters[1];
  pose.z_m = parameters[2];
  pose.roll_deg = normalizeAngleDeg(parameters[3] * kRadToDeg);
  pose.pitch_deg = normalizeAngleDeg(parameters[4] * kRadToDeg);
  pose.yaw_deg = normalizeAngleDeg(parameters[5] * kRadToDeg);
  return pose;
}

bool validPose(const Pose6D& pose) {
  return std::isfinite(pose.x_m) &&
         std::isfinite(pose.y_m) &&
         std::isfinite(pose.z_m) &&
         std::isfinite(pose.roll_deg) &&
         std::isfinite(pose.pitch_deg) &&
         std::isfinite(pose.yaw_deg);
}

bool validPoseGraphConfig(const PoseGraphOptimizerConfig& config) {
  return config.max_iterations > 0 &&
         std::isfinite(config.implicit_odometry_translation_stddev_m) &&
         config.implicit_odometry_translation_stddev_m > 0.0 &&
         std::isfinite(config.implicit_odometry_rotation_stddev_deg) &&
         config.implicit_odometry_rotation_stddev_deg > 0.0 &&
         std::isfinite(config.smoothness_translation_stddev_m) &&
         std::isfinite(config.smoothness_rotation_stddev_deg) &&
         (!config.enable_smoothness_prior ||
          (config.smoothness_translation_stddev_m > 0.0 &&
           config.smoothness_rotation_stddev_deg > 0.0));
}

std::array<double, 6> parametersFromPose(const Pose6D& pose) {
  return {pose.x_m,
          pose.y_m,
          pose.z_m,
          pose.roll_deg * kDegToRad,
          pose.pitch_deg * kDegToRad,
          pose.yaw_deg * kDegToRad};
}

template <typename T>
void rotateZyx(const T& roll,
               const T& pitch,
               const T& yaw,
               const T* vector,
               T* rotated) {
  const T cr = ceres::cos(roll);
  const T sr = ceres::sin(roll);
  const T cp = ceres::cos(pitch);
  const T sp = ceres::sin(pitch);
  const T cy = ceres::cos(yaw);
  const T sy = ceres::sin(yaw);

  rotated[0] = (cy * cp) * vector[0] +
               (cy * sp * sr - sy * cr) * vector[1] +
               (cy * sp * cr + sy * sr) * vector[2];
  rotated[1] = (sy * cp) * vector[0] +
               (sy * sp * sr + cy * cr) * vector[1] +
               (sy * sp * cr - cy * sr) * vector[2];
  rotated[2] = (-sp) * vector[0] + (cp * sr) * vector[1] +
               (cp * cr) * vector[2];
}

Pose6D relativePoseFromInitial(const Pose6D& from, const Pose6D& to) {
  const double delta_world[3] = {
      to.x_m - from.x_m,
      to.y_m - from.y_m,
      to.z_m - from.z_m,
  };
  const double roll = from.roll_deg * kDegToRad;
  const double pitch = from.pitch_deg * kDegToRad;
  const double yaw = from.yaw_deg * kDegToRad;
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  const double rotation[3][3] = {
      {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
      {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
      {-sp, cp * sr, cp * cr},
  };

  Pose6D relative;
  relative.x_m = rotation[0][0] * delta_world[0] +
                 rotation[1][0] * delta_world[1] +
                 rotation[2][0] * delta_world[2];
  relative.y_m = rotation[0][1] * delta_world[0] +
                 rotation[1][1] * delta_world[1] +
                 rotation[2][1] * delta_world[2];
  relative.z_m = rotation[0][2] * delta_world[0] +
                 rotation[1][2] * delta_world[1] +
                 rotation[2][2] * delta_world[2];
  relative.roll_deg = normalizeAngleDeg(to.roll_deg - from.roll_deg);
  relative.pitch_deg = normalizeAngleDeg(to.pitch_deg - from.pitch_deg);
  relative.yaw_deg = normalizeAngleDeg(to.yaw_deg - from.yaw_deg);
  return relative;
}

Pose6D subtractPose(const Pose6D& lhs, const Pose6D& rhs) {
  Pose6D result;
  result.x_m = lhs.x_m - rhs.x_m;
  result.y_m = lhs.y_m - rhs.y_m;
  result.z_m = lhs.z_m - rhs.z_m;
  result.roll_deg = normalizeAngleDeg(lhs.roll_deg - rhs.roll_deg);
  result.pitch_deg = normalizeAngleDeg(lhs.pitch_deg - rhs.pitch_deg);
  result.yaw_deg = normalizeAngleDeg(lhs.yaw_deg - rhs.yaw_deg);
  return result;
}

Pose6D applyErrorFraction(const Pose6D& pose,
                          const Pose6D& error,
                          double fraction,
                          double weight) {
  Pose6D result = pose;
  result.x_m -= error.x_m * fraction * weight;
  result.y_m -= error.y_m * fraction * weight;
  result.z_m -= error.z_m * fraction * weight;
  result.roll_deg = normalizeAngleDeg(result.roll_deg -
                                      error.roll_deg * fraction * weight);
  result.pitch_deg = normalizeAngleDeg(result.pitch_deg -
                                       error.pitch_deg * fraction * weight);
  result.yaw_deg = normalizeAngleDeg(result.yaw_deg -
                                     error.yaw_deg * fraction * weight);
  return result;
}

double poseConstraintWeight(const PoseGraphConstraint6D& constraint,
                            const Pose6D& error) {
  const double translation_stddev =
      positiveOrDefault(constraint.translation_stddev_m, 1.0);
  const double rotation_stddev =
      positiveOrDefault(constraint.rotation_stddev_deg, 1.0);
  const double translation_error =
      std::sqrt(error.x_m * error.x_m + error.y_m * error.y_m +
                error.z_m * error.z_m) /
      translation_stddev;
  const double rotation_error =
      std::sqrt(error.roll_deg * error.roll_deg +
                error.pitch_deg * error.pitch_deg +
                error.yaw_deg * error.yaw_deg) /
      rotation_stddev;
  const double normalized_residual =
      std::sqrt(translation_error * translation_error +
                rotation_error * rotation_error);
  const double covariance_weight =
      (constraint.translation_stddev_m > 0.0 ||
       constraint.rotation_stddev_deg > 0.0)
          ? 1.0 / (1.0 + 0.5 * (translation_stddev + rotation_stddev))
          : 1.0;
  return clampWeight(constraint.weight * covariance_weight *
                     robustScale(normalized_residual,
                                 constraint.robust_kernel_delta));
}

struct RelativePoseResidual {
  RelativePoseResidual(const Pose6D& relative_pose,
                       const double translation_stddev_m,
                       const double rotation_stddev_deg,
                       const double sqrt_weight)
      : relative_pose_(relative_pose),
        translation_stddev_m_(translation_stddev_m),
        rotation_stddev_rad_(rotation_stddev_deg * kDegToRad),
        sqrt_weight_(sqrt_weight) {}

  template <typename T>
  bool operator()(const T* const from, const T* const to, T* residuals) const {
    const T relative_translation[3] = {
        T(relative_pose_.x_m),
        T(relative_pose_.y_m),
        T(relative_pose_.z_m),
    };
    T rotated_translation[3];
    rotateZyx(from[3], from[4], from[5], relative_translation,
              rotated_translation);

    const T predicted_to[6] = {
        from[0] + rotated_translation[0],
        from[1] + rotated_translation[1],
        from[2] + rotated_translation[2],
        from[3] + T(relative_pose_.roll_deg * kDegToRad),
        from[4] + T(relative_pose_.pitch_deg * kDegToRad),
        from[5] + T(relative_pose_.yaw_deg * kDegToRad),
    };

    const T translation_scale =
        T(sqrt_weight_ / positiveOrDefault(translation_stddev_m_, 1.0));
    const T rotation_scale =
        T(sqrt_weight_ /
          positiveOrDefault(rotation_stddev_rad_, 1.0 * kDegToRad));
    residuals[0] = (to[0] - predicted_to[0]) * translation_scale;
    residuals[1] = (to[1] - predicted_to[1]) * translation_scale;
    residuals[2] = (to[2] - predicted_to[2]) * translation_scale;
    residuals[3] = (to[3] - predicted_to[3]) * rotation_scale;
    residuals[4] = (to[4] - predicted_to[4]) * rotation_scale;
    residuals[5] = (to[5] - predicted_to[5]) * rotation_scale;
    return true;
  }

  Pose6D relative_pose_;
  double translation_stddev_m_;
  double rotation_stddev_rad_;
  double sqrt_weight_;
};

struct SmoothnessResidual {
  SmoothnessResidual(const double translation_stddev_m,
                     const double rotation_stddev_deg)
      : translation_stddev_m_(translation_stddev_m),
        rotation_stddev_rad_(rotation_stddev_deg * kDegToRad) {}

  template <typename T>
  bool operator()(const T* const previous,
                  const T* const current,
                  const T* const next,
                  T* residuals) const {
    const T translation_scale =
        T(1.0 / positiveOrDefault(translation_stddev_m_, 1.0));
    const T rotation_scale =
        T(1.0 / positiveOrDefault(rotation_stddev_rad_, 1.0 * kDegToRad));
    for (int index = 0; index < 3; ++index) {
      residuals[index] =
          (current[index] - T(0.5) * (previous[index] + next[index])) *
          translation_scale;
    }
    for (int index = 3; index < 6; ++index) {
      residuals[index] =
          (current[index] - T(0.5) * (previous[index] + next[index])) *
          rotation_scale;
    }
    return true;
  }

  double translation_stddev_m_;
  double rotation_stddev_rad_;
};

bool buildPoseGraphNodeIndex(const std::vector<PoseGraphNode6D>& nodes,
                             std::map<std::string, int>* index_by_id) {
  index_by_id->clear();
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const PoseGraphNode6D& node = nodes[index];
    if (!validKeyframeId(node.keyframe_id) || !validPose(node.pose) ||
        index_by_id->find(node.keyframe_id) != index_by_id->end()) {
      return false;
    }
    (*index_by_id)[node.keyframe_id] = static_cast<int>(index);
  }
  return true;
}

bool validPoseGraphConstraint(const PoseGraphConstraint6D& constraint,
                              const std::map<std::string, int>& index_by_id) {
  return validKeyframeId(constraint.from_keyframe_id) &&
         validKeyframeId(constraint.to_keyframe_id) &&
         index_by_id.find(constraint.from_keyframe_id) != index_by_id.end() &&
         index_by_id.find(constraint.to_keyframe_id) != index_by_id.end() &&
         validPose(constraint.relative_pose) &&
         validWeight(constraint.weight) &&
         validOptionalStddev(constraint.translation_stddev_m) &&
         validOptionalStddev(constraint.rotation_stddev_deg) &&
         validRobustKernelDelta(constraint.robust_kernel_delta);
}

bool validPoseGraphInput(const std::vector<PoseGraphNode6D>& nodes,
                         const std::vector<PoseGraphConstraint6D>& constraints,
                         std::map<std::string, int>* index_by_id) {
  if (!buildPoseGraphNodeIndex(nodes, index_by_id)) {
    return false;
  }
  for (const PoseGraphConstraint6D& constraint : constraints) {
    if (!validPoseGraphConstraint(constraint, *index_by_id)) {
      return false;
    }
  }
  return true;
}

std::map<std::string, Pose6D> optimizePoseGraph6DLightweight(
    const std::vector<PoseGraphNode6D>& nodes,
    const std::vector<PoseGraphConstraint6D>& constraints,
    const int iterations) {
  std::map<std::string, Pose6D> poses;
  std::map<std::string, int> index_by_id;
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    poses[nodes[index].keyframe_id] = nodes[index].pose;
    index_by_id[nodes[index].keyframe_id] = static_cast<int>(index);
  }
  if (nodes.empty()) {
    return poses;
  }

  const std::string anchor_id = nodes.front().keyframe_id;
  const Pose6D anchor_pose = poses[anchor_id];
  for (int iteration = 0; iteration < std::max(1, iterations); ++iteration) {
    for (const PoseGraphConstraint6D& constraint : constraints) {
      if (index_by_id.find(constraint.from_keyframe_id) == index_by_id.end() ||
          index_by_id.find(constraint.to_keyframe_id) == index_by_id.end()) {
        continue;
      }

      int start_index = index_by_id[constraint.from_keyframe_id];
      int end_index = index_by_id[constraint.to_keyframe_id];
      std::string from_id = constraint.from_keyframe_id;
      Pose6D relative = constraint.relative_pose;
      if (start_index == end_index) {
        continue;
      }
      if (end_index < start_index) {
        std::swap(start_index, end_index);
        from_id = constraint.to_keyframe_id;
        relative.x_m = -relative.x_m;
        relative.y_m = -relative.y_m;
        relative.z_m = -relative.z_m;
        relative.roll_deg = normalizeAngleDeg(-relative.roll_deg);
        relative.pitch_deg = normalizeAngleDeg(-relative.pitch_deg);
        relative.yaw_deg = normalizeAngleDeg(-relative.yaw_deg);
      }

      const std::string end_id = nodes[end_index].keyframe_id;
      const Pose6D desired_end = addPose(poses[from_id], relative);
      const Pose6D error = subtractPose(poses[end_id], desired_end);
      const double weight = poseConstraintWeight(constraint, error);
      const double span = static_cast<double>(end_index - start_index);
      for (int index = start_index + 1; index <= end_index; ++index) {
        const std::string keyframe_id = nodes[index].keyframe_id;
        const double fraction = static_cast<double>(index - start_index) / span;
        poses[keyframe_id] =
            applyErrorFraction(poses[keyframe_id], error, fraction, weight);
      }
      poses[anchor_id] = anchor_pose;
    }
  }
  return poses;
}

}  // namespace

bool buildChainageNodeIndex(const std::vector<ChainageNode>& nodes,
                            std::map<std::string, int>* index_by_id) {
  index_by_id->clear();
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const ChainageNode& node = nodes[index];
    if (!validKeyframeId(node.keyframe_id) ||
        !std::isfinite(node.chainage_m) ||
        index_by_id->find(node.keyframe_id) != index_by_id->end()) {
      return false;
    }
    (*index_by_id)[node.keyframe_id] = static_cast<int>(index);
  }
  return true;
}

bool validChainageConstraint(const LoopConstraint& constraint,
                             const std::map<std::string, int>& index_by_id) {
  return validKeyframeId(constraint.from_keyframe_id) &&
         validKeyframeId(constraint.to_keyframe_id) &&
         index_by_id.find(constraint.from_keyframe_id) != index_by_id.end() &&
         index_by_id.find(constraint.to_keyframe_id) != index_by_id.end() &&
         std::isfinite(constraint.relative_chainage_m) &&
         validWeight(constraint.weight) &&
         validOptionalStddev(constraint.stddev_m) &&
         validRobustKernelDelta(constraint.robust_kernel_delta);
}

bool validChainageGraphInput(const std::vector<ChainageNode>& nodes,
                             const std::vector<LoopConstraint>& loop_constraints,
                             std::map<std::string, int>* index_by_id) {
  if (!buildChainageNodeIndex(nodes, index_by_id)) {
    return false;
  }
  for (const LoopConstraint& constraint : loop_constraints) {
    if (!validChainageConstraint(constraint, *index_by_id)) {
      return false;
    }
  }
  return true;
}

std::map<std::string, double> optimizeChainageGraph(
    const std::vector<ChainageNode>& nodes,
    const std::vector<LoopConstraint>& loop_constraints,
    const int iterations) {
  std::map<std::string, int> validated_index;
  if (iterations <= 0 ||
      !validChainageGraphInput(nodes, loop_constraints, &validated_index)) {
    return {};
  }
  std::map<std::string, double> positions;
  std::map<std::string, int> index_by_id;
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    positions[nodes[index].keyframe_id] = nodes[index].chainage_m;
    index_by_id[nodes[index].keyframe_id] = static_cast<int>(index);
  }
  if (nodes.empty()) {
    return positions;
  }

  const std::string anchor_id = nodes.front().keyframe_id;
  const double anchor_value = positions[anchor_id];
  for (int iteration = 0; iteration < std::max(1, iterations); ++iteration) {
    for (const LoopConstraint& constraint : loop_constraints) {
      if (index_by_id.find(constraint.from_keyframe_id) == index_by_id.end() ||
          index_by_id.find(constraint.to_keyframe_id) == index_by_id.end()) {
        continue;
      }
      int start_index = index_by_id[constraint.from_keyframe_id];
      int end_index = index_by_id[constraint.to_keyframe_id];
      std::string from_id = constraint.from_keyframe_id;
      double relative = constraint.relative_chainage_m;
      if (start_index == end_index) {
        continue;
      }
      if (end_index < start_index) {
        std::swap(start_index, end_index);
        from_id = constraint.to_keyframe_id;
        relative = -constraint.relative_chainage_m;
      }
      const std::string end_id = nodes[end_index].keyframe_id;
      const double desired_end = positions[from_id] + relative;
      const double error = positions[end_id] - desired_end;
      const double weight = chainageConstraintWeight(constraint, error);
      const double span = static_cast<double>(end_index - start_index);
      for (int index = start_index + 1; index <= end_index; ++index) {
        const std::string keyframe_id = nodes[index].keyframe_id;
        const double fraction = (index - start_index) / span;
        const double target = positions[keyframe_id] - error * fraction;
        positions[keyframe_id] = positions[keyframe_id] * (1.0 - weight) + target * weight;
      }
      positions[anchor_id] = anchor_value;
    }
  }
  return positions;
}

std::map<std::string, Pose6D> optimizePoseGraph6D(
    const std::vector<PoseGraphNode6D>& nodes,
    const std::vector<PoseGraphConstraint6D>& constraints,
    const int iterations) {
  PoseGraphOptimizerConfig config;
  config.max_iterations = iterations;
  return optimizePoseGraph6D(nodes, constraints, config);
}

std::map<std::string, Pose6D> optimizePoseGraph6D(
    const std::vector<PoseGraphNode6D>& nodes,
    const std::vector<PoseGraphConstraint6D>& constraints,
    const PoseGraphOptimizerConfig& config) {
  if (nodes.empty()) {
    return {};
  }
  std::map<std::string, int> validated_index;
  if (!validPoseGraphConfig(config) ||
      !validPoseGraphInput(nodes, constraints, &validated_index)) {
    return {};
  }
  if (!config.enable_ceres) {
    return optimizePoseGraph6DLightweight(
        nodes, constraints, config.max_iterations);
  }

  std::map<std::string, std::array<double, 6>> parameter_blocks;
  std::map<std::string, int> index_by_id;
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    parameter_blocks[nodes[index].keyframe_id] =
        parametersFromPose(nodes[index].pose);
    index_by_id[nodes[index].keyframe_id] = static_cast<int>(index);
  }

  ceres::Problem problem;
  for (PoseGraphNode6D const& node : nodes) {
    problem.AddParameterBlock(parameter_blocks[node.keyframe_id].data(), 6);
  }
  problem.SetParameterBlockConstant(
      parameter_blocks[nodes.front().keyframe_id].data());

  for (std::size_t index = 1; index < nodes.size(); ++index) {
    const PoseGraphNode6D& from = nodes[index - 1];
    const PoseGraphNode6D& to = nodes[index];
    const Pose6D relative = relativePoseFromInitial(from.pose, to.pose);
    ceres::CostFunction* cost =
        new ceres::AutoDiffCostFunction<RelativePoseResidual, 6, 6, 6>(
            new RelativePoseResidual(relative,
                                     positiveOrDefault(
                                         config.implicit_odometry_translation_stddev_m,
                                         10.0),
                                     positiveOrDefault(
                                         config.implicit_odometry_rotation_stddev_deg,
                                         100.0),
                                     1.0));
    problem.AddResidualBlock(cost,
                             nullptr,
                             parameter_blocks[from.keyframe_id].data(),
                             parameter_blocks[to.keyframe_id].data());
  }
  if (config.enable_smoothness_prior &&
      config.smoothness_translation_stddev_m > 0.0 &&
      config.smoothness_rotation_stddev_deg > 0.0) {
    for (std::size_t index = 1; index + 1 < nodes.size(); ++index) {
      ceres::CostFunction* cost =
          new ceres::AutoDiffCostFunction<SmoothnessResidual, 6, 6, 6, 6>(
              new SmoothnessResidual(config.smoothness_translation_stddev_m,
                                     config.smoothness_rotation_stddev_deg));
      problem.AddResidualBlock(
          cost,
          nullptr,
          parameter_blocks[nodes[index - 1].keyframe_id].data(),
          parameter_blocks[nodes[index].keyframe_id].data(),
          parameter_blocks[nodes[index + 1].keyframe_id].data());
    }
  }

  bool added_external_constraint = false;
  for (const PoseGraphConstraint6D& constraint : constraints) {
    if (index_by_id.find(constraint.from_keyframe_id) == index_by_id.end() ||
        index_by_id.find(constraint.to_keyframe_id) == index_by_id.end()) {
      continue;
    }
    const double sqrt_weight = std::sqrt(std::max(0.0, constraint.weight));
    if (sqrt_weight <= std::numeric_limits<double>::epsilon()) {
      continue;
    }
    ceres::CostFunction* cost =
        new ceres::AutoDiffCostFunction<RelativePoseResidual, 6, 6, 6>(
            new RelativePoseResidual(
                constraint.relative_pose,
                positiveOrDefault(constraint.translation_stddev_m, 1.0),
                positiveOrDefault(constraint.rotation_stddev_deg, 1.0),
                sqrt_weight));
    ceres::LossFunction* loss = nullptr;
    if (constraint.robust_kernel_delta > 0.0) {
      loss = new ceres::HuberLoss(constraint.robust_kernel_delta);
    }
    problem.AddResidualBlock(
        cost,
        loss,
        parameter_blocks[constraint.from_keyframe_id].data(),
        parameter_blocks[constraint.to_keyframe_id].data());
    added_external_constraint = true;
  }

  if (!added_external_constraint && constraints.empty()) {
    std::map<std::string, Pose6D> unchanged;
    for (const PoseGraphNode6D& node : nodes) {
      unchanged[node.keyframe_id] = node.pose;
    }
    return unchanged;
  }

  ceres::Solver::Options options;
  options.max_num_iterations = std::max(1, config.max_iterations) * 10;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = false;
  options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  if (!summary.IsSolutionUsable()) {
    return optimizePoseGraph6DLightweight(
        nodes, constraints, config.max_iterations);
  }

  std::map<std::string, Pose6D> poses;
  for (const PoseGraphNode6D& node : nodes) {
    poses[node.keyframe_id] =
        poseFromParameters(parameter_blocks[node.keyframe_id].data());
  }
  return poses;
}

}  // namespace slam_backend_manager
