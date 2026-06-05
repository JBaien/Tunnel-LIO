#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

namespace lio_local_odometry {

struct RegistrationGate {
  std::size_t min_points = 100;
  double max_fitness_score = 0.5;
  double min_geometry_score = 0.0;
};

struct SubmapQualityGate {
  std::size_t min_stable_points = 1000;
  double min_observability_score = 0.5;
  std::size_t max_consecutive_rejections = 2;
  std::size_t max_points = 200000;
};

struct SubmapQuality {
  bool can_promote = false;
  double quality_score = 0.0;
  double capacity_ratio = 0.0;
  std::string reason = "starting";
};

struct GeometryObservability {
  bool degenerate = true;
  double min_eigen_ratio = 0.0;
  double score = 0.0;
  std::string reason = "insufficient_points";
};

inline Eigen::Affine3d accumulatePose(
    const Eigen::Affine3d& previous_pose,
    const Eigen::Matrix4f& current_to_previous) {
  const Eigen::Affine3d delta_current_to_previous(
      current_to_previous.cast<double>());
  return previous_pose * delta_current_to_previous.inverse();
}

inline Eigen::Matrix4f scaledTranslationInitialGuess(
    const Eigen::Matrix4f& previous_current_to_previous,
    double scale,
    double max_translation) {
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  if (!std::isfinite(scale) || scale <= 0.0 ||
      !std::isfinite(max_translation) || max_translation <= 0.0) {
    return guess;
  }
  for (int row = 0; row < previous_current_to_previous.rows(); ++row) {
    for (int col = 0; col < previous_current_to_previous.cols(); ++col) {
      if (!std::isfinite(previous_current_to_previous(row, col))) {
        return guess;
      }
    }
  }

  Eigen::Vector3f translation =
      previous_current_to_previous.block<3, 1>(0, 3) *
      static_cast<float>(scale);
  const float norm = translation.norm();
  if (norm > static_cast<float>(max_translation) && norm > 0.0f) {
    translation *= static_cast<float>(max_translation) / norm;
  }
  guess.block<3, 1>(0, 3) = translation;
  return guess;
}

inline bool isRegistrationUsable(bool converged,
                                 double fitness_score,
                                 std::size_t point_count,
                                 double geometry_score,
                                 const RegistrationGate& gate) {
  if (gate.min_points == 0 ||
      !std::isfinite(gate.max_fitness_score) ||
      gate.max_fitness_score < 0.0 ||
      !std::isfinite(gate.min_geometry_score) ||
      gate.min_geometry_score < 0.0) {
    return false;
  }
  return converged && point_count >= gate.min_points &&
         std::isfinite(fitness_score) &&
         fitness_score <= gate.max_fitness_score &&
         std::isfinite(geometry_score) &&
         geometry_score >= gate.min_geometry_score;
}

inline double observabilityScore(double fitness_score,
                                 double max_fitness_score) {
  if (!std::isfinite(fitness_score) || max_fitness_score <= 0.0) {
    return 0.0;
  }
  const double score = 1.0 - fitness_score / max_fitness_score;
  return std::max(0.0, std::min(1.0, score));
}

inline bool shouldReseedAfterRejectedRegistration(
    std::size_t consecutive_rejections,
    double geometry_score,
    double min_geometry_score,
    std::size_t reseed_after_consecutive_rejections) {
  if (reseed_after_consecutive_rejections == 0 ||
      consecutive_rejections < reseed_after_consecutive_rejections ||
      !std::isfinite(geometry_score) ||
      !std::isfinite(min_geometry_score) ||
      min_geometry_score < 0.0) {
    return false;
  }
  return geometry_score >= min_geometry_score;
}

inline GeometryObservability evaluateGeometryObservability(
    const std::vector<Eigen::Vector3d>& points,
    std::size_t min_points,
    double min_eigen_ratio) {
  GeometryObservability result;
  if (min_points == 0 || !std::isfinite(min_eigen_ratio) ||
      min_eigen_ratio < 0.0) {
    result.reason = "invalid_geometry_gate";
    return result;
  }
  if (points.size() < min_points || points.empty()) {
    return result;
  }
  for (const auto& point : points) {
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        !std::isfinite(point.z())) {
      result.reason = "non_finite_geometry";
      return result;
    }
  }

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& point : points) {
    mean += point;
  }
  mean /= static_cast<double>(points.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& point : points) {
    const Eigen::Vector3d centered = point - mean;
    covariance += centered * centered.transpose();
  }
  covariance /= static_cast<double>(points.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    result.reason = "eigen_solver_failed";
    return result;
  }

  const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
  const double max_eigenvalue = eigenvalues.maxCoeff();
  if (max_eigenvalue <= std::numeric_limits<double>::epsilon()) {
    result.reason = "zero_spread";
    return result;
  }

  result.min_eigen_ratio = eigenvalues.minCoeff() / max_eigenvalue;
  const double required_ratio = std::max(0.0, min_eigen_ratio);
  result.score = required_ratio <= 0.0
                     ? 1.0
                     : std::max(0.0, std::min(1.0, result.min_eigen_ratio /
                                                       required_ratio));
  if (result.min_eigen_ratio < required_ratio) {
    result.degenerate = true;
    result.reason = "geometry_degenerate";
    return result;
  }

  result.degenerate = false;
  result.reason = "ok";
  return result;
}

inline SubmapQuality evaluateSubmapQuality(
    bool registration_usable,
    double observability_score,
    std::size_t submap_points,
    std::size_t consecutive_rejections,
    const SubmapQualityGate& gate) {
  SubmapQuality quality;
  if (gate.min_stable_points == 0 ||
      !std::isfinite(gate.min_observability_score) ||
      gate.min_observability_score < 0.0 ||
      gate.min_observability_score > 1.0) {
    quality.reason = "invalid_quality_gate";
    return quality;
  }
  if (!std::isfinite(observability_score)) {
    quality.reason = "invalid_observability";
    return quality;
  }

  if (gate.max_points > 0) {
    quality.capacity_ratio =
        std::min(1.0, static_cast<double>(submap_points) /
                          static_cast<double>(gate.max_points));
  }

  const double point_score =
      gate.min_stable_points == 0
          ? 1.0
          : std::min(1.0, static_cast<double>(submap_points) /
                              static_cast<double>(gate.min_stable_points));
  quality.quality_score =
      std::max(0.0, std::min(1.0, 0.6 * observability_score + 0.4 * point_score));

  if (!registration_usable) {
    quality.reason = "registration_rejected";
    return quality;
  }
  if (observability_score < gate.min_observability_score) {
    quality.reason = "weak_observability";
    return quality;
  }
  if (submap_points < gate.min_stable_points) {
    quality.reason = "insufficient_submap_points";
    return quality;
  }
  if (consecutive_rejections > gate.max_consecutive_rejections) {
    quality.reason = "registration_unstable";
    return quality;
  }

  quality.can_promote = true;
  quality.reason = "ok";
  return quality;
}

}  // namespace lio_local_odometry
