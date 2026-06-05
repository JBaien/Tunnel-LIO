#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <limits>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lio_local_odometry/local_registration.h"

namespace {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

CloudT::Ptr makeAsymmetricCloud() {
  CloudT::Ptr cloud(new CloudT);
  for (int ix = 0; ix < 8; ++ix) {
    for (int iy = 0; iy < 5; ++iy) {
      for (int iz = 0; iz < 3; ++iz) {
        PointT point;
        point.x = 0.17f * static_cast<float>(ix);
        point.y = 0.23f * static_cast<float>(iy) + 0.03f * ix;
        point.z = 0.19f * static_cast<float>(iz) + 0.01f * iy;
        point.intensity = static_cast<float>(ix + 2 * iy + 3 * iz);
        cloud->push_back(point);
      }
    }
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

CloudT::Ptr makeLargeOffsetVoxelGridCloud() {
  CloudT::Ptr cloud(new CloudT);
  for (int ix = 0; ix < 8; ++ix) {
    for (int iy = 0; iy < 8; ++iy) {
      PointT point;
      point.x = 680000.0f + 0.02f * static_cast<float>(ix);
      point.y = 2450000.0f + 0.02f * static_cast<float>(iy);
      point.z = -38.0f;
      point.intensity = static_cast<float>(ix + iy);
      cloud->push_back(point);
    }
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

TEST(LocalRegistration, VoxelDownsampleHandlesLargeAbsoluteCoordinates) {
  const CloudT::Ptr cloud = makeLargeOffsetVoxelGridCloud();

  const CloudT::Ptr filtered =
      lio_local_odometry::voxelDownsampleFiniteCloud(cloud, 0.10);

  ASSERT_TRUE(filtered);
  EXPECT_LT(filtered->size(), cloud->size());
  EXPECT_GT(filtered->size(), 0u);
  EXPECT_NEAR(680000.0, filtered->front().x, 0.30);
  EXPECT_NEAR(2450000.0, filtered->front().y, 0.30);
}

TEST(LocalRegistration, VoxelDownsampleHandlesLargeCoordinateSpan) {
  CloudT::Ptr cloud(new CloudT);
  for (int i = 0; i < 100; ++i) {
    PointT point;
    point.x = 1000000.0f + 0.01f * static_cast<float>(i);
    point.y = -2000000.0f;
    point.z = 5.0f;
    point.intensity = static_cast<float>(i);
    cloud->push_back(point);
  }
  PointT outlier;
  outlier.x = 1000000000.0f;
  outlier.y = -1000000000.0f;
  outlier.z = 1000000000.0f;
  outlier.intensity = 1.0f;
  cloud->push_back(outlier);
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;

  const CloudT::Ptr filtered =
      lio_local_odometry::voxelDownsampleFiniteCloud(cloud, 0.10);

  ASSERT_TRUE(filtered);
  EXPECT_GT(filtered->size(), 0u);
  EXPECT_LT(filtered->size(), cloud->size());
}

TEST(LocalRegistration, MultiScaleIcpRecoversRigidTranslation) {
  const CloudT::Ptr target = makeAsymmetricCloud();

  Eigen::Matrix4f target_to_source = Eigen::Matrix4f::Identity();
  target_to_source(0, 3) = 0.34f;
  target_to_source(1, 3) = -0.18f;
  target_to_source(2, 3) = 0.07f;

  CloudT::Ptr source(new CloudT);
  pcl::transformPointCloud(*target, *source, target_to_source);

  lio_local_odometry::MultiScaleIcpConfig config;
  config.max_correspondence_distance = 1.0;
  config.transformation_epsilon = 1e-8;
  config.euclidean_fitness_epsilon = 1e-8;
  config.max_iterations = 80;
  config.voxel_leaf_sizes = {0.30, 0.15, 0.0};

  lio_local_odometry::MultiScaleIcpRegistration registration(config);
  const auto result = registration.align(source, target);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(3u, result.stages_used);
  EXPECT_LT(result.fitness_score, 1e-6);
  EXPECT_NEAR(-0.34, result.source_to_target(0, 3), 1e-3);
  EXPECT_NEAR(0.18, result.source_to_target(1, 3), 1e-3);
  EXPECT_NEAR(-0.07, result.source_to_target(2, 3), 1e-3);
}

TEST(LocalRegistration, MultiScaleIcpUsesProvidedInitialGuess) {
  const CloudT::Ptr target = makeAsymmetricCloud();

  Eigen::Matrix4f target_to_source = Eigen::Matrix4f::Identity();
  target_to_source(0, 3) = 2.0f;

  CloudT::Ptr source(new CloudT);
  pcl::transformPointCloud(*target, *source, target_to_source);

  lio_local_odometry::MultiScaleIcpConfig config;
  config.max_correspondence_distance = 0.25;
  config.transformation_epsilon = 1e-8;
  config.euclidean_fitness_epsilon = 1e-8;
  config.max_iterations = 80;
  config.voxel_leaf_sizes = {0.30, 0.0};

  Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
  initial_guess(0, 3) = -2.0f;

  lio_local_odometry::MultiScaleIcpRegistration registration(config);
  const auto result = registration.align(source, target, initial_guess);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(2u, result.stages_used);
  EXPECT_LT(result.fitness_score, 1e-6);
  EXPECT_NEAR(-2.0, result.source_to_target(0, 3), 1e-3);
}

TEST(LocalRegistration, MultiScaleIcpRejectsEmptyInputs) {
  lio_local_odometry::MultiScaleIcpConfig config;
  lio_local_odometry::MultiScaleIcpRegistration registration(config);

  const auto empty(new CloudT);
  const CloudT::ConstPtr cloud(empty);
  const auto result = registration.align(cloud, cloud);

  EXPECT_FALSE(result.converged);
  EXPECT_EQ("insufficient_points", result.reason);
  EXPECT_EQ(0u, result.stages_used);
}

TEST(LocalRegistration, MultiScaleIcpRejectsInvalidConfig) {
  const CloudT::Ptr cloud = makeAsymmetricCloud();

  lio_local_odometry::MultiScaleIcpConfig config;
  config.max_correspondence_distance = -1.0;
  config.max_iterations = 80;
  config.voxel_leaf_sizes = {0.20, 0.0};

  lio_local_odometry::MultiScaleIcpRegistration registration(config);
  const auto result = registration.align(cloud, cloud);

  EXPECT_FALSE(result.converged);
  EXPECT_EQ("invalid_config", result.reason);
  EXPECT_EQ(0u, result.stages_used);
}

TEST(LocalRegistration, MultiScaleNdtRecoversRigidTranslation) {
  const CloudT::Ptr target = makeAsymmetricCloud();

  Eigen::Matrix4f target_to_source = Eigen::Matrix4f::Identity();
  target_to_source(0, 3) = 0.30f;
  target_to_source(1, 3) = -0.16f;
  target_to_source(2, 3) = 0.05f;

  CloudT::Ptr source(new CloudT);
  pcl::transformPointCloud(*target, *source, target_to_source);

  lio_local_odometry::MultiScaleNdtConfig config;
  config.resolutions = {0.80, 0.40, 0.20};
  config.step_size = 0.20;
  config.transformation_epsilon = 1e-8;
  config.max_iterations = 80;
  config.min_points = 30;

  lio_local_odometry::MultiScaleNdtRegistration registration(config);
  const auto result = registration.align(source, target);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(3u, result.stages_used);
  EXPECT_LT(result.fitness_score, 1e-4);
  EXPECT_NEAR(-0.30, result.source_to_target(0, 3), 2e-2);
  EXPECT_NEAR(0.16, result.source_to_target(1, 3), 2e-2);
  EXPECT_NEAR(-0.05, result.source_to_target(2, 3), 2e-2);
}

TEST(LocalRegistration, MultiScaleNdtRejectsInvalidConfig) {
  const CloudT::Ptr cloud = makeAsymmetricCloud();

  lio_local_odometry::MultiScaleNdtConfig config;
  config.resolutions = {1.0, 0.5};
  config.step_size = std::numeric_limits<double>::quiet_NaN();
  config.max_iterations = 80;
  config.min_points = 30;

  lio_local_odometry::MultiScaleNdtRegistration registration(config);
  const auto result = registration.align(cloud, cloud);

  EXPECT_FALSE(result.converged);
  EXPECT_EQ("invalid_config", result.reason);
  EXPECT_EQ(0u, result.stages_used);
}

TEST(LocalRegistration, MultiScaleGicpRecoversRigidTranslation) {
  const CloudT::Ptr target = makeAsymmetricCloud();

  Eigen::Matrix4f target_to_source = Eigen::Matrix4f::Identity();
  target_to_source(0, 3) = 0.22f;
  target_to_source(1, 3) = -0.14f;
  target_to_source(2, 3) = 0.09f;

  CloudT::Ptr source(new CloudT);
  pcl::transformPointCloud(*target, *source, target_to_source);

  lio_local_odometry::MultiScaleGicpConfig config;
  config.voxel_leaf_sizes = {0.25, 0.10, 0.0};
  config.max_correspondence_distance = 1.0;
  config.transformation_epsilon = 1e-8;
  config.euclidean_fitness_epsilon = 1e-8;
  config.max_iterations = 80;
  config.min_points = 30;

  lio_local_odometry::MultiScaleGicpRegistration registration(config);
  const auto result = registration.align(source, target);

  ASSERT_TRUE(result.converged);
  EXPECT_EQ(3u, result.stages_used);
  EXPECT_LT(result.fitness_score, 1e-5);
  EXPECT_NEAR(-0.22, result.source_to_target(0, 3), 2e-3);
  EXPECT_NEAR(0.14, result.source_to_target(1, 3), 2e-3);
  EXPECT_NEAR(-0.09, result.source_to_target(2, 3), 2e-3);
}

TEST(LocalRegistration, MultiScaleGicpRejectsInvalidConfig) {
  const CloudT::Ptr cloud = makeAsymmetricCloud();

  lio_local_odometry::MultiScaleGicpConfig config;
  config.voxel_leaf_sizes = {0.20, 0.0};
  config.euclidean_fitness_epsilon = -1e-3;
  config.max_iterations = 80;
  config.min_points = 30;

  lio_local_odometry::MultiScaleGicpRegistration registration(config);
  const auto result = registration.align(cloud, cloud);

  EXPECT_FALSE(result.converged);
  EXPECT_EQ("invalid_config", result.reason);
  EXPECT_EQ(0u, result.stages_used);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
