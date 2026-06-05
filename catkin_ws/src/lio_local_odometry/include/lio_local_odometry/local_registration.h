#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lio_local_odometry {

using RegistrationPointT = pcl::PointXYZI;
using RegistrationCloudT = pcl::PointCloud<RegistrationPointT>;

RegistrationCloudT::Ptr voxelDownsampleFiniteCloud(
    const RegistrationCloudT::ConstPtr& cloud,
    double leaf_size);

struct VoxelPyramidLevel {
  double leaf_size = 0.0;
  RegistrationCloudT::Ptr cloud;
};

std::vector<VoxelPyramidLevel> buildVoxelPyramid(
    const RegistrationCloudT::ConstPtr& cloud,
    const std::vector<double>& leaf_sizes);

struct MultiScaleIcpConfig {
  std::vector<double> voxel_leaf_sizes;
  double max_correspondence_distance = 1.0;
  double transformation_epsilon = 1e-4;
  double euclidean_fitness_epsilon = 1e-3;
  int max_iterations = 50;
  std::size_t min_points = 10;
};

struct MultiScaleIcpResult {
  bool converged = false;
  double fitness_score = std::numeric_limits<double>::infinity();
  std::size_t stages_used = 0;
  Eigen::Matrix4f source_to_target = Eigen::Matrix4f::Identity();
  std::string reason = "not_started";
};

struct MultiScaleNdtConfig {
  std::vector<double> resolutions;
  double step_size = 0.1;
  double max_correspondence_distance = 1.0;
  double transformation_epsilon = 1e-4;
  int max_iterations = 50;
  int refinement_iterations = 30;
  std::size_t min_points = 10;
};

using MultiScaleNdtResult = MultiScaleIcpResult;
using MultiScaleGicpConfig = MultiScaleIcpConfig;
using MultiScaleGicpResult = MultiScaleIcpResult;

class MultiScaleIcpRegistration {
 public:
  explicit MultiScaleIcpRegistration(const MultiScaleIcpConfig& config);

  MultiScaleIcpResult align(const RegistrationCloudT::ConstPtr& source,
                            const RegistrationCloudT::ConstPtr& target) const;
  MultiScaleIcpResult align(const RegistrationCloudT::ConstPtr& source,
                            const RegistrationCloudT::ConstPtr& target,
                            const Eigen::Matrix4f& initial_guess) const;

 private:
  RegistrationCloudT::Ptr downsample(const RegistrationCloudT::ConstPtr& cloud,
                                     double leaf_size,
                                     const RegistrationPointT& origin) const;

  MultiScaleIcpConfig config_;
};

class MultiScaleNdtRegistration {
 public:
  explicit MultiScaleNdtRegistration(const MultiScaleNdtConfig& config);

  MultiScaleNdtResult align(const RegistrationCloudT::ConstPtr& source,
                            const RegistrationCloudT::ConstPtr& target) const;
  MultiScaleNdtResult align(const RegistrationCloudT::ConstPtr& source,
                            const RegistrationCloudT::ConstPtr& target,
                            const Eigen::Matrix4f& initial_guess) const;

 private:
  RegistrationCloudT::Ptr finiteCloud(const RegistrationCloudT::ConstPtr& cloud) const;

  MultiScaleNdtConfig config_;
};

class MultiScaleGicpRegistration {
 public:
  explicit MultiScaleGicpRegistration(const MultiScaleGicpConfig& config);

  MultiScaleGicpResult align(const RegistrationCloudT::ConstPtr& source,
                             const RegistrationCloudT::ConstPtr& target) const;
  MultiScaleGicpResult align(const RegistrationCloudT::ConstPtr& source,
                             const RegistrationCloudT::ConstPtr& target,
                             const Eigen::Matrix4f& initial_guess) const;

 private:
  RegistrationCloudT::Ptr downsample(const RegistrationCloudT::ConstPtr& cloud,
                                     double leaf_size,
                                     const RegistrationPointT& origin) const;

  MultiScaleGicpConfig config_;
};

}  // namespace lio_local_odometry
