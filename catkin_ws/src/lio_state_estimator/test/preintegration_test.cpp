#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "lio_state_estimator/preintegration.h"

namespace lio_state_estimator {
namespace {

TEST(ImuPreintegrator, IntegratesConstantLinearAcceleration) {
  ImuPreintegrator integrator(0.0, 0.2);
  integrator.observe(0.0, ImuVector{1.0, 0.0, 0.0}, ImuVector{0.0, 0.0, 0.0});
  const ImuPredictionState state =
      integrator.observe(1.0, ImuVector{1.0, 0.0, 0.0}, ImuVector{0.0, 0.0, 0.0});

  EXPECT_NEAR(1.0, state.velocity.x, 1e-9);
  EXPECT_NEAR(0.5, state.position.x, 1e-9);
  EXPECT_EQ(0, state.rejected_updates);
}

TEST(ImuPreintegrator, RejectsLargeTimeGap) {
  ImuPreintegrator integrator(0.0, 0.1);
  integrator.observe(0.0, ImuVector{1.0, 0.0, 0.0}, ImuVector{0.0, 0.0, 0.0});
  const ImuPredictionState state =
      integrator.observe(1.0, ImuVector{1.0, 0.0, 0.0}, ImuVector{0.0, 0.0, 0.0});

  EXPECT_NEAR(0.0, state.velocity.x, 1e-9);
  EXPECT_EQ(1, state.rejected_updates);
}

TEST(ImuPreintegrator, TimeRegressionResetsPrediction) {
  ImuPreintegrator integrator(0.0, 0.2);
  integrator.observe(1.0, ImuVector{1.0, 0.0, 0.0}, ImuVector{0.0, 0.0, 0.0});
  integrator.observe(1.1, ImuVector{1.0, 0.0, 0.0}, ImuVector{0.0, 0.0, 0.0});
  const ImuPredictionState state =
      integrator.observe(0.9, ImuVector{1.0, 0.0, 0.0}, ImuVector{0.0, 0.0, 0.0});

  EXPECT_NEAR(0.0, state.velocity.x, 1e-9);
  EXPECT_NEAR(0.0, state.position.x, 1e-9);
  EXPECT_EQ(1, state.reset_count);
}

TEST(ImuPreintegrator, RemovesGyroBiasFromAngularVelocityState) {
  ImuPreintegrator integrator(0.0, 0.2);
  integrator.setGyroBias(ImuVector{0.10, -0.05, 0.02});

  const ImuPredictionState biased =
      integrator.observe(0.0,
                         ImuVector{0.0, 0.0, 0.0},
                         ImuVector{0.12, -0.02, 0.03});

  EXPECT_NEAR(0.02, biased.angular_velocity.x, 1e-9);
  EXPECT_NEAR(0.03, biased.angular_velocity.y, 1e-9);
  EXPECT_NEAR(0.01, biased.angular_velocity.z, 1e-9);
  EXPECT_NEAR(0.10, biased.gyro_bias.x, 1e-9);

  integrator.clearGyroBias();
  const ImuPredictionState uncorrected =
      integrator.observe(0.1,
                         ImuVector{0.0, 0.0, 0.0},
                         ImuVector{0.12, -0.02, 0.03});
  EXPECT_NEAR(0.12, uncorrected.angular_velocity.x, 1e-9);
  EXPECT_NEAR(-0.02, uncorrected.angular_velocity.y, 1e-9);
  EXPECT_NEAR(0.03, uncorrected.angular_velocity.z, 1e-9);
}

TEST(ImuPreintegrator, RemovesGravityAlongConfiguredDirection) {
  ImuPreintegrator integrator(9.80665, 0.2);
  integrator.setGravityDirection(ImuVector{1.0, 0.0, 0.0});

  integrator.observe(0.0,
                     ImuVector{9.80665, 0.0, 0.0},
                     ImuVector{0.0, 0.0, 0.0});
  const ImuPredictionState state =
      integrator.observe(0.1,
                         ImuVector{9.80665, 0.0, 0.0},
                         ImuVector{0.0, 0.0, 0.0});

  EXPECT_NEAR(0.0, state.velocity.x, 1e-9);
  EXPECT_NEAR(0.0, state.velocity.y, 1e-9);
  EXPECT_NEAR(0.0, state.velocity.z, 1e-9);
  EXPECT_NEAR(1.0, state.gravity_direction.x, 1e-9);
}

TEST(ImuPreintegrator, RejectsNonFiniteObservationBeforeIntegration) {
  ImuPreintegrator integrator(0.0, 0.2);
  integrator.observe(0.0,
                     ImuVector{1.0, 0.0, 0.0},
                     ImuVector{0.0, 0.0, 0.0});

  const ImuPredictionState rejected =
      integrator.observe(0.1,
                         ImuVector{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
                         ImuVector{0.0, 0.0, 0.0});

  EXPECT_EQ(1, rejected.rejected_updates);
  EXPECT_TRUE(std::isfinite(rejected.position.x));
  EXPECT_TRUE(std::isfinite(rejected.velocity.x));
  EXPECT_NEAR(0.0, rejected.position.x, 1e-9);
  EXPECT_NEAR(0.0, rejected.velocity.x, 1e-9);
  EXPECT_NEAR(0.0, rejected.last_stamp, 1e-9);

  const ImuPredictionState recovered =
      integrator.observe(0.1,
                         ImuVector{1.0, 0.0, 0.0},
                         ImuVector{0.0, 0.0, 0.0});

  EXPECT_EQ(1, recovered.rejected_updates);
  EXPECT_NEAR(0.1, recovered.velocity.x, 1e-9);
}

TEST(ImuPreintegrator, IgnoresNonFiniteBiasAndGravityDirection) {
  ImuPreintegrator integrator(9.80665, 0.2);
  integrator.setGyroBias(ImuVector{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});
  integrator.setGravityDirection(ImuVector{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0});

  const ImuPredictionState state =
      integrator.observe(0.0,
                         ImuVector{0.0, 0.0, 9.80665},
                         ImuVector{0.1, 0.0, 0.0});

  EXPECT_TRUE(std::isfinite(state.angular_velocity.x));
  EXPECT_NEAR(0.1, state.angular_velocity.x, 1e-9);
  EXPECT_NEAR(0.0, state.gravity_direction.x, 1e-9);
  EXPECT_NEAR(0.0, state.gravity_direction.y, 1e-9);
  EXPECT_NEAR(1.0, state.gravity_direction.z, 1e-9);
}

}  // namespace
}  // namespace lio_state_estimator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
