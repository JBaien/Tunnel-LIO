#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <limits>
#include <vector>

#include "lio_local_odometry/local_odometry_math.h"

namespace {

TEST(LocalOdometryMath, AccumulatesInverseRegistrationDelta) {
  Eigen::Affine3d previous_pose = Eigen::Affine3d::Identity();
  previous_pose.translation().x() = 10.0;

  Eigen::Matrix4f current_to_previous = Eigen::Matrix4f::Identity();
  current_to_previous(0, 3) = -0.25f;

  const Eigen::Affine3d current_pose =
      lio_local_odometry::accumulatePose(previous_pose, current_to_previous);

  EXPECT_NEAR(10.25, current_pose.translation().x(), 1e-9);
  EXPECT_NEAR(0.0, current_pose.translation().y(), 1e-9);
  EXPECT_NEAR(0.0, current_pose.translation().z(), 1e-9);
}

TEST(LocalOdometryMath, BuildsClampedTranslationInitialGuess) {
  Eigen::Matrix4f previous_delta = Eigen::Matrix4f::Identity();
  previous_delta(0, 3) = -0.4f;
  previous_delta(1, 3) = 0.2f;

  const Eigen::Matrix4f scaled =
      lio_local_odometry::scaledTranslationInitialGuess(previous_delta, 2.0,
                                                        2.0);
  EXPECT_NEAR(-0.8, scaled(0, 3), 1e-6);
  EXPECT_NEAR(0.4, scaled(1, 3), 1e-6);
  EXPECT_NEAR(0.0, scaled(2, 3), 1e-6);

  const Eigen::Matrix4f clamped =
      lio_local_odometry::scaledTranslationInitialGuess(previous_delta, 10.0,
                                                        1.0);
  const double norm = std::sqrt(clamped(0, 3) * clamped(0, 3) +
                                clamped(1, 3) * clamped(1, 3) +
                                clamped(2, 3) * clamped(2, 3));
  EXPECT_NEAR(1.0, norm, 1e-6);

  previous_delta(0, 3) = std::numeric_limits<float>::quiet_NaN();
  const Eigen::Matrix4f invalid =
      lio_local_odometry::scaledTranslationInitialGuess(previous_delta, 2.0,
                                                        2.0);
  EXPECT_TRUE(invalid.isIdentity(1e-6));
}

TEST(LocalOdometryMath, RejectsUnusableRegistrationResults) {
  lio_local_odometry::RegistrationGate gate;
  gate.min_points = 80;
  gate.max_fitness_score = 0.25;

  EXPECT_TRUE(
      lio_local_odometry::isRegistrationUsable(true, 0.1, 200, 1.0, gate));
  EXPECT_FALSE(
      lio_local_odometry::isRegistrationUsable(false, 0.1, 200, 1.0, gate));
  EXPECT_FALSE(
      lio_local_odometry::isRegistrationUsable(true, 0.5, 200, 1.0, gate));
  EXPECT_FALSE(
      lio_local_odometry::isRegistrationUsable(true, 0.1, 20, 1.0, gate));
}

TEST(LocalOdometryMath, RejectsRegistrationsWithWeakGeometry) {
  lio_local_odometry::RegistrationGate gate;
  gate.min_points = 80;
  gate.max_fitness_score = 0.25;
  gate.min_geometry_score = 0.4;

  EXPECT_TRUE(
      lio_local_odometry::isRegistrationUsable(true, 0.1, 200, 0.8, gate));
  EXPECT_FALSE(
      lio_local_odometry::isRegistrationUsable(true, 0.1, 200, 0.2, gate));
}

TEST(LocalOdometryMath, ComputesObservabilityFromFitness) {
  EXPECT_NEAR(1.0, lio_local_odometry::observabilityScore(0.0, 1.0), 1e-9);
  EXPECT_NEAR(0.5, lio_local_odometry::observabilityScore(0.5, 1.0), 1e-9);
  EXPECT_NEAR(0.0, lio_local_odometry::observabilityScore(2.0, 1.0), 1e-9);
}

TEST(LocalOdometryMath, ReseedsAfterRepeatedObservableRejections) {
  EXPECT_TRUE(lio_local_odometry::shouldReseedAfterRejectedRegistration(
      2, 0.8, 0.2, 2));
  EXPECT_FALSE(lio_local_odometry::shouldReseedAfterRejectedRegistration(
      1, 0.8, 0.2, 2));
  EXPECT_FALSE(lio_local_odometry::shouldReseedAfterRejectedRegistration(
      2, 0.1, 0.2, 2));
  EXPECT_FALSE(lio_local_odometry::shouldReseedAfterRejectedRegistration(
      2, 0.8, 0.2, 0));
  EXPECT_FALSE(lio_local_odometry::shouldReseedAfterRejectedRegistration(
      2, std::numeric_limits<double>::quiet_NaN(), 0.2, 2));
}

TEST(LocalOdometryMath, PromotesOnlyStableObservableSubmaps) {
  lio_local_odometry::SubmapQualityGate gate;
  gate.min_stable_points = 1000;
  gate.min_observability_score = 0.55;
  gate.max_consecutive_rejections = 2;
  gate.max_points = 4000;

  const auto good = lio_local_odometry::evaluateSubmapQuality(
      true, 0.8, 2000, 0, gate);
  EXPECT_TRUE(good.can_promote);
  EXPECT_EQ("ok", good.reason);
  EXPECT_GT(good.quality_score, 0.7);
  EXPECT_NEAR(0.5, good.capacity_ratio, 1e-9);

  const auto weak = lio_local_odometry::evaluateSubmapQuality(
      true, 0.2, 2000, 0, gate);
  EXPECT_FALSE(weak.can_promote);
  EXPECT_EQ("weak_observability", weak.reason);

  const auto unstable = lio_local_odometry::evaluateSubmapQuality(
      true, 0.8, 2000, 3, gate);
  EXPECT_FALSE(unstable.can_promote);
  EXPECT_EQ("registration_unstable", unstable.reason);
}

TEST(LocalOdometryMath, RejectsInvalidSubmapQualityInputs) {
  lio_local_odometry::SubmapQualityGate gate;
  gate.min_stable_points = 1000;
  gate.min_observability_score = 0.55;
  gate.max_consecutive_rejections = 2;
  gate.max_points = 4000;

  const auto non_finite = lio_local_odometry::evaluateSubmapQuality(
      true, std::numeric_limits<double>::quiet_NaN(), 2000, 0, gate);
  EXPECT_FALSE(non_finite.can_promote);
  EXPECT_EQ("invalid_observability", non_finite.reason);
  EXPECT_TRUE(std::isfinite(non_finite.quality_score));

  gate.min_observability_score = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_gate = lio_local_odometry::evaluateSubmapQuality(
      true, 0.8, 2000, 0, gate);
  EXPECT_FALSE(invalid_gate.can_promote);
  EXPECT_EQ("invalid_quality_gate", invalid_gate.reason);
}

TEST(LocalOdometryMath, DetectsGeometryDegeneracyFromPointDistribution) {
  std::vector<Eigen::Vector3d> volumetric_points;
  for (double x : {-1.0, 0.0, 1.0}) {
    for (double y : {-1.0, 0.0, 1.0}) {
      for (double z : {-0.5, 0.0, 0.5}) {
        volumetric_points.emplace_back(x, y, z);
      }
    }
  }

  const auto volumetric =
      lio_local_odometry::evaluateGeometryObservability(
          volumetric_points, 10, 0.05);
  EXPECT_FALSE(volumetric.degenerate);
  EXPECT_EQ("ok", volumetric.reason);
  EXPECT_GT(volumetric.min_eigen_ratio, 0.05);
  EXPECT_GT(volumetric.score, 0.8);

  std::vector<Eigen::Vector3d> line_points;
  for (int i = 0; i < 40; ++i) {
    line_points.emplace_back(static_cast<double>(i) * 0.2, 0.0, 0.0);
  }

  const auto line = lio_local_odometry::evaluateGeometryObservability(
      line_points, 10, 0.05);
  EXPECT_TRUE(line.degenerate);
  EXPECT_EQ("geometry_degenerate", line.reason);
  EXPECT_LT(line.min_eigen_ratio, 0.05);
  EXPECT_LT(line.score, 0.5);
}

TEST(LocalOdometryMath, RejectsNonFiniteGeometryPoints) {
  std::vector<Eigen::Vector3d> points;
  points.emplace_back(0.0, 0.0, 0.0);
  points.emplace_back(1.0, 0.0, 0.0);
  points.emplace_back(std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0);

  const auto result = lio_local_odometry::evaluateGeometryObservability(
      points, 3, 0.05);

  EXPECT_TRUE(result.degenerate);
  EXPECT_EQ("non_finite_geometry", result.reason);
  EXPECT_EQ(0.0, result.score);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
