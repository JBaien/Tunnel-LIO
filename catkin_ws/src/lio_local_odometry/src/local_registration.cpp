#include "lio_local_odometry/local_registration.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

namespace lio_local_odometry {
namespace {

bool validLeafSizes(const std::vector<double>& leaf_sizes) {
  if (leaf_sizes.empty()) {
    return false;
  }
  for (const double leaf_size : leaf_sizes) {
    if (!std::isfinite(leaf_size) || leaf_size < 0.0) {
      return false;
    }
  }
  return true;
}

bool validIcpLikeConfig(const MultiScaleIcpConfig& config) {
  return validLeafSizes(config.voxel_leaf_sizes) &&
         std::isfinite(config.max_correspondence_distance) &&
         config.max_correspondence_distance > 0.0 &&
         std::isfinite(config.transformation_epsilon) &&
         config.transformation_epsilon >= 0.0 &&
         std::isfinite(config.euclidean_fitness_epsilon) &&
         config.euclidean_fitness_epsilon >= 0.0 &&
         config.max_iterations > 0 &&
         config.min_points > 0;
}

bool validNdtConfig(const MultiScaleNdtConfig& config) {
  if (config.resolutions.empty()) {
    return false;
  }
  for (const double resolution : config.resolutions) {
    if (!std::isfinite(resolution) || resolution <= 0.0) {
      return false;
    }
  }
  return std::isfinite(config.step_size) &&
         config.step_size > 0.0 &&
         std::isfinite(config.max_correspondence_distance) &&
         config.max_correspondence_distance > 0.0 &&
         std::isfinite(config.transformation_epsilon) &&
         config.transformation_epsilon >= 0.0 &&
         config.max_iterations > 0 &&
         config.refinement_iterations >= 0 &&
         config.min_points > 0;
}

RegistrationPointT firstFinitePointOrZero(
    const RegistrationCloudT::ConstPtr& first,
    const RegistrationCloudT::ConstPtr& second) {
  for (const RegistrationCloudT::ConstPtr& cloud : {first, second}) {
    if (!cloud) {
      continue;
    }
    for (const auto& point : cloud->points) {
      if (pcl::isFinite(point)) {
        return point;
      }
    }
  }
  RegistrationPointT origin;
  origin.x = 0.0f;
  origin.y = 0.0f;
  origin.z = 0.0f;
  origin.intensity = 0.0f;
  return origin;
}

RegistrationCloudT::Ptr voxelDownsampleFiniteCloudFromOrigin(
    const RegistrationCloudT::ConstPtr& cloud,
    double leaf_size,
    const RegistrationPointT& origin) {
  RegistrationCloudT::Ptr finite(new RegistrationCloudT);
  if (!cloud) {
    return finite;
  }

  finite->reserve(cloud->size());
  for (const auto& point : cloud->points) {
    if (pcl::isFinite(point)) {
      finite->push_back(point);
    }
  }
  finite->width = finite->size();
  finite->height = 1;
  finite->is_dense = false;

  if (leaf_size <= 0.0 || finite->empty()) {
    return finite;
  }

  RegistrationCloudT::Ptr local(new RegistrationCloudT);
  local->reserve(finite->size());
  for (const auto& point : finite->points) {
    RegistrationPointT shifted = point;
    shifted.x -= origin.x;
    shifted.y -= origin.y;
    shifted.z -= origin.z;
    local->push_back(shifted);
  }
  local->width = local->size();
  local->height = 1;
  local->is_dense = false;

  pcl::VoxelGrid<RegistrationPointT> voxel;
  voxel.setInputCloud(local);
  voxel.setLeafSize(static_cast<float>(leaf_size), static_cast<float>(leaf_size),
                    static_cast<float>(leaf_size));
  RegistrationCloudT::Ptr filtered_local(new RegistrationCloudT);
  voxel.filter(*filtered_local);

  if (filtered_local->empty()) {
    return finite;
  }

  RegistrationCloudT::Ptr filtered(new RegistrationCloudT);
  filtered->reserve(filtered_local->size());
  for (const auto& point : filtered_local->points) {
    RegistrationPointT restored = point;
    restored.x += origin.x;
    restored.y += origin.y;
    restored.z += origin.z;
    filtered->push_back(restored);
  }
  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;
  return filtered;
}

std::vector<double> uniqueLeafSizes(const std::vector<double>& leaf_sizes) {
  std::vector<double> unique;
  for (const double leaf_size : leaf_sizes) {
    if (std::find(unique.begin(), unique.end(), leaf_size) == unique.end()) {
      unique.push_back(leaf_size);
    }
  }
  return unique;
}

std::vector<VoxelPyramidLevel> buildVoxelPyramidFromOrigin(
    const RegistrationCloudT::ConstPtr& cloud,
    const std::vector<double>& leaf_sizes,
    const RegistrationPointT& origin) {
  std::vector<VoxelPyramidLevel> levels;
  if (!validLeafSizes(leaf_sizes)) {
    return levels;
  }
  const std::vector<double> unique_leaf_sizes = uniqueLeafSizes(leaf_sizes);
  levels.reserve(unique_leaf_sizes.size());
  for (const double leaf_size : unique_leaf_sizes) {
    VoxelPyramidLevel level;
    level.leaf_size = leaf_size;
    level.cloud = voxelDownsampleFiniteCloudFromOrigin(cloud, leaf_size, origin);
    levels.push_back(level);
  }
  return levels;
}

}  // namespace

RegistrationCloudT::Ptr voxelDownsampleFiniteCloud(
    const RegistrationCloudT::ConstPtr& cloud,
    double leaf_size) {
  return voxelDownsampleFiniteCloudFromOrigin(
      cloud, leaf_size, firstFinitePointOrZero(cloud, RegistrationCloudT::ConstPtr()));
}

std::vector<VoxelPyramidLevel> buildVoxelPyramid(
    const RegistrationCloudT::ConstPtr& cloud,
    const std::vector<double>& leaf_sizes) {
  return buildVoxelPyramidFromOrigin(
      cloud, leaf_sizes, firstFinitePointOrZero(cloud, RegistrationCloudT::ConstPtr()));
}

MultiScaleIcpRegistration::MultiScaleIcpRegistration(
    const MultiScaleIcpConfig& config)
    : config_(config) {
  if (config_.voxel_leaf_sizes.empty()) {
    config_.voxel_leaf_sizes.push_back(0.0);
  }
}

MultiScaleIcpResult MultiScaleIcpRegistration::align(
    const RegistrationCloudT::ConstPtr& source,
    const RegistrationCloudT::ConstPtr& target) const {
  return align(source, target, Eigen::Matrix4f::Identity());
}

MultiScaleIcpResult MultiScaleIcpRegistration::align(
    const RegistrationCloudT::ConstPtr& source,
    const RegistrationCloudT::ConstPtr& target,
    const Eigen::Matrix4f& initial_guess) const {
  MultiScaleIcpResult result;
  if (!validIcpLikeConfig(config_)) {
    result.reason = "invalid_config";
    return result;
  }
  if (!source || !target || source->size() < config_.min_points ||
      target->size() < config_.min_points) {
    result.reason = "insufficient_points";
    return result;
  }

  Eigen::Matrix4f stage_initial_guess = initial_guess;
  const RegistrationPointT voxel_origin =
      firstFinitePointOrZero(target, source);
  const std::vector<VoxelPyramidLevel> source_pyramid =
      buildVoxelPyramidFromOrigin(source, config_.voxel_leaf_sizes, voxel_origin);
  const std::vector<VoxelPyramidLevel> target_pyramid =
      buildVoxelPyramidFromOrigin(target, config_.voxel_leaf_sizes, voxel_origin);
  const std::size_t stage_count = std::min(source_pyramid.size(), target_pyramid.size());
  for (std::size_t stage = 0; stage < stage_count; ++stage) {
    const RegistrationCloudT::Ptr stage_source = source_pyramid[stage].cloud;
    const RegistrationCloudT::Ptr stage_target = target_pyramid[stage].cloud;
    if (stage_source->size() < config_.min_points ||
        stage_target->size() < config_.min_points) {
      continue;
    }

    pcl::IterativeClosestPoint<RegistrationPointT, RegistrationPointT> icp;
    icp.setInputSource(stage_source);
    icp.setInputTarget(stage_target);
    icp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
    icp.setTransformationEpsilon(config_.transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
    icp.setMaximumIterations(config_.max_iterations);

    RegistrationCloudT aligned;
    icp.align(aligned, stage_initial_guess);

    ++result.stages_used;
    result.converged = icp.hasConverged();
    result.fitness_score = icp.getFitnessScore();
    result.source_to_target = icp.getFinalTransformation();
    stage_initial_guess = result.source_to_target;
    result.reason = result.converged ? "ok" : "icp_not_converged";
  }

  if (result.stages_used == 0) {
    result.reason = "insufficient_stage_points";
  }
  return result;
}

RegistrationCloudT::Ptr MultiScaleIcpRegistration::downsample(
    const RegistrationCloudT::ConstPtr& cloud,
    double leaf_size,
    const RegistrationPointT& origin) const {
  return voxelDownsampleFiniteCloudFromOrigin(cloud, leaf_size, origin);
}

MultiScaleNdtRegistration::MultiScaleNdtRegistration(
    const MultiScaleNdtConfig& config)
    : config_(config) {
  if (config_.resolutions.empty()) {
    config_.resolutions.push_back(1.0);
  }
}

MultiScaleNdtResult MultiScaleNdtRegistration::align(
    const RegistrationCloudT::ConstPtr& source,
    const RegistrationCloudT::ConstPtr& target) const {
  return align(source, target, Eigen::Matrix4f::Identity());
}

MultiScaleNdtResult MultiScaleNdtRegistration::align(
    const RegistrationCloudT::ConstPtr& source,
    const RegistrationCloudT::ConstPtr& target,
    const Eigen::Matrix4f& initial_guess) const {
  MultiScaleNdtResult result;
  if (!validNdtConfig(config_)) {
    result.reason = "invalid_config";
    return result;
  }
  const RegistrationCloudT::Ptr finite_source = finiteCloud(source);
  const RegistrationCloudT::Ptr finite_target = finiteCloud(target);
  if (!finite_source || !finite_target ||
      finite_source->size() < config_.min_points ||
      finite_target->size() < config_.min_points) {
    result.reason = "insufficient_points";
    return result;
  }

  Eigen::Matrix4f stage_initial_guess = initial_guess;
  for (const double resolution : config_.resolutions) {
    if (resolution <= 0.0) {
      continue;
    }

    pcl::NormalDistributionsTransform<RegistrationPointT, RegistrationPointT> ndt;
    ndt.setInputSource(finite_source);
    ndt.setInputTarget(finite_target);
    ndt.setResolution(resolution);
    ndt.setStepSize(config_.step_size);
    ndt.setTransformationEpsilon(config_.transformation_epsilon);
    ndt.setMaximumIterations(config_.max_iterations);

    RegistrationCloudT aligned;
    ndt.align(aligned, stage_initial_guess);

    ++result.stages_used;
    result.converged = ndt.hasConverged();
    result.fitness_score = ndt.getFitnessScore();
    result.source_to_target = ndt.getFinalTransformation();
    stage_initial_guess = result.source_to_target;
    result.reason = result.converged ? "ok" : "ndt_not_converged";
  }

  if (result.stages_used == 0) {
    result.reason = "invalid_resolution";
    return result;
  }

  if (result.converged && config_.refinement_iterations > 0) {
    pcl::IterativeClosestPoint<RegistrationPointT, RegistrationPointT> icp;
    icp.setInputSource(finite_source);
    icp.setInputTarget(finite_target);
    icp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
    icp.setTransformationEpsilon(config_.transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(config_.transformation_epsilon);
    icp.setMaximumIterations(config_.refinement_iterations);

    RegistrationCloudT aligned;
    icp.align(aligned, result.source_to_target);
    result.converged = icp.hasConverged();
    result.fitness_score = icp.getFitnessScore();
    result.source_to_target = icp.getFinalTransformation();
    result.reason = result.converged ? "ok" : "icp_refinement_not_converged";
  }
  return result;
}

RegistrationCloudT::Ptr MultiScaleNdtRegistration::finiteCloud(
    const RegistrationCloudT::ConstPtr& cloud) const {
  RegistrationCloudT::Ptr finite(new RegistrationCloudT);
  if (!cloud) {
    return finite;
  }
  finite->reserve(cloud->size());
  for (const auto& point : cloud->points) {
    if (pcl::isFinite(point)) {
      finite->push_back(point);
    }
  }
  finite->width = finite->size();
  finite->height = 1;
  finite->is_dense = false;
  return finite;
}

MultiScaleGicpRegistration::MultiScaleGicpRegistration(
    const MultiScaleGicpConfig& config)
    : config_(config) {
  if (config_.voxel_leaf_sizes.empty()) {
    config_.voxel_leaf_sizes.push_back(0.0);
  }
}

MultiScaleGicpResult MultiScaleGicpRegistration::align(
    const RegistrationCloudT::ConstPtr& source,
    const RegistrationCloudT::ConstPtr& target) const {
  return align(source, target, Eigen::Matrix4f::Identity());
}

MultiScaleGicpResult MultiScaleGicpRegistration::align(
    const RegistrationCloudT::ConstPtr& source,
    const RegistrationCloudT::ConstPtr& target,
    const Eigen::Matrix4f& initial_guess) const {
  MultiScaleGicpResult result;
  if (!validIcpLikeConfig(config_)) {
    result.reason = "invalid_config";
    return result;
  }
  if (!source || !target || source->size() < config_.min_points ||
      target->size() < config_.min_points) {
    result.reason = "insufficient_points";
    return result;
  }

  Eigen::Matrix4f stage_initial_guess = initial_guess;
  const RegistrationPointT voxel_origin =
      firstFinitePointOrZero(target, source);
  const std::vector<VoxelPyramidLevel> source_pyramid =
      buildVoxelPyramidFromOrigin(source, config_.voxel_leaf_sizes, voxel_origin);
  const std::vector<VoxelPyramidLevel> target_pyramid =
      buildVoxelPyramidFromOrigin(target, config_.voxel_leaf_sizes, voxel_origin);
  const std::size_t stage_count = std::min(source_pyramid.size(), target_pyramid.size());
  for (std::size_t stage = 0; stage < stage_count; ++stage) {
    const RegistrationCloudT::Ptr stage_source = source_pyramid[stage].cloud;
    const RegistrationCloudT::Ptr stage_target = target_pyramid[stage].cloud;
    if (stage_source->size() < config_.min_points ||
        stage_target->size() < config_.min_points) {
      continue;
    }

    pcl::GeneralizedIterativeClosestPoint<RegistrationPointT, RegistrationPointT> gicp;
    gicp.setInputSource(stage_source);
    gicp.setInputTarget(stage_target);
    gicp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
    gicp.setTransformationEpsilon(config_.transformation_epsilon);
    gicp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
    gicp.setMaximumIterations(config_.max_iterations);

    RegistrationCloudT aligned;
    gicp.align(aligned, stage_initial_guess);

    ++result.stages_used;
    result.converged = gicp.hasConverged();
    result.fitness_score = gicp.getFitnessScore();
    result.source_to_target = gicp.getFinalTransformation();
    stage_initial_guess = result.source_to_target;
    result.reason = result.converged ? "ok" : "gicp_not_converged";
  }

  if (result.stages_used == 0) {
    result.reason = "insufficient_stage_points";
  }
  return result;
}

RegistrationCloudT::Ptr MultiScaleGicpRegistration::downsample(
    const RegistrationCloudT::ConstPtr& cloud,
    double leaf_size,
    const RegistrationPointT& origin) const {
  return voxelDownsampleFiniteCloudFromOrigin(cloud, leaf_size, origin);
}

}  // namespace lio_local_odometry
