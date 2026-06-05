#include <gtest/gtest.h>

#include <limits>

#include "slam_backend_manager/pose_graph_optimizer.h"

namespace slam_backend_manager {
namespace {

TEST(PoseGraphOptimizer, LoopConstraintDistributesChainageError) {
  const std::map<std::string, double> optimized =
      optimizeChainageGraph({ChainageNode{"kf_0", 0.0},
                             ChainageNode{"kf_1", 10.0},
                             ChainageNode{"kf_2", 20.0},
                             ChainageNode{"kf_3", 30.0}},
                            {LoopConstraint{"kf_0", "kf_3", 29.0, 1.0}}, 5);

  EXPECT_NEAR(0.0, optimized.at("kf_0"), 1e-6);
  EXPECT_NEAR(29.0, optimized.at("kf_3"), 1e-3);
  EXPECT_NEAR(9.666, optimized.at("kf_1"), 0.02);
  EXPECT_NEAR(19.333, optimized.at("kf_2"), 0.02);
}

TEST(PoseGraphOptimizer, LowWeightLoopChangesLessThanHighWeightLoop) {
  const std::vector<ChainageNode> nodes = {ChainageNode{"a", 0.0}, ChainageNode{"b", 10.0}};
  const std::map<std::string, double> weak =
      optimizeChainageGraph(nodes, {LoopConstraint{"a", "b", 8.0, 0.25}}, 3);
  const std::map<std::string, double> strong =
      optimizeChainageGraph(nodes, {LoopConstraint{"a", "b", 8.0, 1.0}}, 3);

  EXPECT_GT(weak.at("b"), strong.at("b"));
  EXPECT_GT(strong.at("b"), 7.9);
}

TEST(PoseGraphOptimizer, ChainageGraphRejectsInvalidInputs) {
  const std::vector<ChainageNode> nodes = {
      ChainageNode{"kf_0", 0.0},
      ChainageNode{"kf_1", 10.0},
  };

  EXPECT_TRUE(optimizeChainageGraph(
                  {ChainageNode{"kf_0", 0.0},
                   ChainageNode{"kf_bad", std::numeric_limits<double>::quiet_NaN()}},
                  {}, 3)
                  .empty());
  EXPECT_TRUE(optimizeChainageGraph(
                  {ChainageNode{"", 0.0}, ChainageNode{"kf_1", 10.0}},
                  {}, 3)
                  .empty());
  EXPECT_TRUE(optimizeChainageGraph(
                  nodes,
                  {LoopConstraint{"kf_0", "kf_1",
                                  std::numeric_limits<double>::quiet_NaN(),
                                  1.0}},
                  3)
                  .empty());
  EXPECT_TRUE(optimizeChainageGraph(
                  nodes,
                  {LoopConstraint{"kf_0", "missing", 8.0, 1.0}},
                  3)
                  .empty());

  EXPECT_FALSE(optimizeChainageGraph(
                   nodes,
                   {LoopConstraint{"kf_0", "kf_1", 8.0, 1.0, -1.0}},
                   3)
                   .empty());
  EXPECT_TRUE(optimizeChainageGraph(
                  nodes,
                  {LoopConstraint{"kf_0", "kf_1", 8.0, 1.0, 0.0}},
                  3)
                  .empty());
  EXPECT_TRUE(optimizeChainageGraph(
                  nodes,
                  {LoopConstraint{"kf_0", "kf_1", 8.0, 1.0, -0.5}},
                  3)
                  .empty());
}

TEST(PoseGraphOptimizer, ChainageGraphRejectsPollutedKeyframeIds) {
  EXPECT_TRUE(optimizeChainageGraph(
                  {ChainageNode{"kf_0", 0.0},
                   ChainageNode{"kf_bad;chainage=spoof", 10.0}},
                  {},
                  3)
                  .empty());
  EXPECT_TRUE(optimizeChainageGraph(
                  {ChainageNode{"..", 0.0}, ChainageNode{"kf_1", 10.0}},
                  {},
                  3)
                  .empty());
}

TEST(PoseGraphOptimizer, SixDofLoopConstraintDistributesPoseDrift) {
  std::vector<PoseGraphNode6D> nodes;
  nodes.push_back(PoseGraphNode6D{"kf_0", Pose6D{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  nodes.push_back(PoseGraphNode6D{"kf_1", Pose6D{10.0, 0.5, 0.0, 0.0, 0.0, 2.0}});
  nodes.push_back(PoseGraphNode6D{"kf_2", Pose6D{20.0, 1.0, 0.0, 0.0, 0.0, 4.0}});

  const std::map<std::string, Pose6D> optimized = optimizePoseGraph6D(
      nodes,
      {PoseGraphConstraint6D{"kf_0", "kf_2",
                             Pose6D{18.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                             1.0}},
      8);

  EXPECT_NEAR(0.0, optimized.at("kf_0").x_m, 1e-9);
  EXPECT_NEAR(0.0, optimized.at("kf_0").yaw_deg, 1e-9);
  EXPECT_NEAR(18.0, optimized.at("kf_2").x_m, 0.05);
  EXPECT_NEAR(0.0, optimized.at("kf_2").y_m, 0.05);
  EXPECT_NEAR(0.0, optimized.at("kf_2").yaw_deg, 0.1);
  EXPECT_NEAR(9.0, optimized.at("kf_1").x_m, 0.1);
  EXPECT_NEAR(0.0, optimized.at("kf_1").yaw_deg, 0.2);
}

TEST(PoseGraphOptimizer, RobustKernelSuppressesOutlierSixDofLoop) {
  std::vector<PoseGraphNode6D> nodes;
  nodes.push_back(PoseGraphNode6D{"kf_0", Pose6D{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  nodes.push_back(PoseGraphNode6D{"kf_1", Pose6D{10.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  nodes.push_back(PoseGraphNode6D{"kf_2", Pose6D{20.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

  PoseGraphConstraint6D trusted;
  trusted.from_keyframe_id = "kf_0";
  trusted.to_keyframe_id = "kf_2";
  trusted.relative_pose = Pose6D{20.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  trusted.weight = 1.0;
  trusted.translation_stddev_m = 0.05;
  trusted.rotation_stddev_deg = 0.5;
  trusted.robust_kernel_delta = 2.0;

  PoseGraphConstraint6D outlier = trusted;
  outlier.relative_pose = Pose6D{12.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  const std::map<std::string, Pose6D> optimized =
      optimizePoseGraph6D(nodes, {trusted, outlier}, 8);

  EXPECT_NEAR(20.0, optimized.at("kf_2").x_m, 0.5);
  EXPECT_NEAR(10.0, optimized.at("kf_1").x_m, 0.5);
}

TEST(PoseGraphOptimizer, SixDofRelativeTranslationUsesFromPoseHeading) {
  std::vector<PoseGraphNode6D> nodes;
  nodes.push_back(PoseGraphNode6D{"kf_0", Pose6D{0.0, 0.0, 0.0, 0.0, 0.0, 90.0}});
  nodes.push_back(PoseGraphNode6D{"kf_1", Pose6D{0.0, 10.0, 0.0, 0.0, 0.0, 90.0}});

  PoseGraphConstraint6D forward_constraint;
  forward_constraint.from_keyframe_id = "kf_0";
  forward_constraint.to_keyframe_id = "kf_1";
  forward_constraint.relative_pose = Pose6D{10.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  forward_constraint.weight = 1.0;
  forward_constraint.translation_stddev_m = 0.05;
  forward_constraint.rotation_stddev_deg = 0.5;

  const std::map<std::string, Pose6D> optimized =
      optimizePoseGraph6D(nodes, {forward_constraint}, 20);

  EXPECT_NEAR(0.0, optimized.at("kf_0").x_m, 1e-9);
  EXPECT_NEAR(90.0, optimized.at("kf_0").yaw_deg, 1e-9);
  EXPECT_NEAR(0.0, optimized.at("kf_1").x_m, 0.05);
  EXPECT_NEAR(10.0, optimized.at("kf_1").y_m, 0.05);
  EXPECT_NEAR(90.0, optimized.at("kf_1").yaw_deg, 0.05);
}

TEST(PoseGraphOptimizer, ConfigurableOdometryPriorControlsLoopCorrectionStrength) {
  std::vector<PoseGraphNode6D> nodes;
  nodes.push_back(PoseGraphNode6D{"kf_0", Pose6D{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  nodes.push_back(PoseGraphNode6D{"kf_1", Pose6D{10.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  nodes.push_back(PoseGraphNode6D{"kf_2", Pose6D{20.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

  PoseGraphConstraint6D loop;
  loop.from_keyframe_id = "kf_0";
  loop.to_keyframe_id = "kf_2";
  loop.relative_pose = Pose6D{16.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  loop.weight = 1.0;

  PoseGraphOptimizerConfig strong_odometry;
  strong_odometry.max_iterations = 20;
  strong_odometry.implicit_odometry_translation_stddev_m = 0.05;
  strong_odometry.implicit_odometry_rotation_stddev_deg = 0.5;

  const std::map<std::string, Pose6D> default_prior =
      optimizePoseGraph6D(nodes, {loop}, 20);
  const std::map<std::string, Pose6D> strong_prior =
      optimizePoseGraph6D(nodes, {loop}, strong_odometry);

  EXPECT_LT(default_prior.at("kf_2").x_m, 16.2);
  EXPECT_GT(strong_prior.at("kf_2").x_m, 19.0);
}

TEST(PoseGraphOptimizer, SixDofGraphRejectsInvalidInputsAndConfig) {
  std::vector<PoseGraphNode6D> nodes;
  nodes.push_back(PoseGraphNode6D{"kf_0", Pose6D{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  nodes.push_back(PoseGraphNode6D{"kf_1", Pose6D{10.0, 0.0, 0.0, 0.0, 0.0, 0.0}});

  PoseGraphConstraint6D valid;
  valid.from_keyframe_id = "kf_0";
  valid.to_keyframe_id = "kf_1";
  valid.relative_pose = Pose6D{10.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  valid.weight = 1.0;

  std::vector<PoseGraphNode6D> invalid_nodes = nodes;
  invalid_nodes[1].pose.x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(optimizePoseGraph6D(invalid_nodes, {valid}, 5).empty());

  PoseGraphConstraint6D invalid_relative = valid;
  invalid_relative.relative_pose.yaw_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(optimizePoseGraph6D(nodes, {invalid_relative}, 5).empty());

  PoseGraphConstraint6D unknown_endpoint = valid;
  unknown_endpoint.to_keyframe_id = "missing";
  EXPECT_TRUE(optimizePoseGraph6D(nodes, {unknown_endpoint}, 5).empty());

  PoseGraphConstraint6D sentinel_stddev = valid;
  sentinel_stddev.translation_stddev_m = -1.0;
  sentinel_stddev.rotation_stddev_deg = -1.0;
  EXPECT_FALSE(optimizePoseGraph6D(nodes, {sentinel_stddev}, 5).empty());

  PoseGraphConstraint6D zero_translation_stddev = valid;
  zero_translation_stddev.translation_stddev_m = 0.0;
  EXPECT_TRUE(optimizePoseGraph6D(nodes, {zero_translation_stddev}, 5).empty());

  PoseGraphConstraint6D negative_rotation_stddev = valid;
  negative_rotation_stddev.rotation_stddev_deg = -0.5;
  EXPECT_TRUE(optimizePoseGraph6D(nodes, {negative_rotation_stddev}, 5).empty());

  PoseGraphOptimizerConfig invalid_config;
  invalid_config.implicit_odometry_translation_stddev_m =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(optimizePoseGraph6D(nodes, {valid}, invalid_config).empty());
}

TEST(PoseGraphOptimizer, SixDofGraphRejectsPollutedKeyframeIds) {
  std::vector<PoseGraphNode6D> semicolon_nodes;
  semicolon_nodes.push_back(
      PoseGraphNode6D{"kf_0", Pose6D{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  semicolon_nodes.push_back(
      PoseGraphNode6D{"kf_bad;pose=spoof",
                      Pose6D{10.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  EXPECT_TRUE(optimizePoseGraph6D(semicolon_nodes, {}, 5).empty());

  std::vector<PoseGraphNode6D> path_nodes;
  path_nodes.push_back(
      PoseGraphNode6D{"..", Pose6D{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  path_nodes.push_back(
      PoseGraphNode6D{"kf_1", Pose6D{10.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  EXPECT_TRUE(optimizePoseGraph6D(path_nodes, {}, 5).empty());
}

}  // namespace
}  // namespace slam_backend_manager

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
