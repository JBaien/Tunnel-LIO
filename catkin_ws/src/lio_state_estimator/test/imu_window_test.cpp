#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "lio_state_estimator/imu_window.h"

namespace lio_state_estimator {
namespace {

TEST(ImuWindow, ComputesFrequencyMeanValuesAndHealth) {
  ImuWindow window(5.0, 100.0);
  for (int index = 0; index < 5; ++index) {
    window.add(ImuSample{index * 0.01, 0.0, 0.0, 9.8, 0.1, 0.0, 0.0});
  }

  const ImuWindowStatus status = window.status();
  EXPECT_NEAR(100.0, status.frequency_hz, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, status.mean_acc.x);
  EXPECT_DOUBLE_EQ(0.0, status.mean_acc.y);
  EXPECT_DOUBLE_EQ(9.8, status.mean_acc.z);
  EXPECT_DOUBLE_EQ(0.1, status.mean_gyro.x);
  EXPECT_DOUBLE_EQ(0.0, status.mean_gyro.y);
  EXPECT_DOUBLE_EQ(0.0, status.mean_gyro.z);
  EXPECT_DOUBLE_EQ(1.0, status.health_score);
}

TEST(ImuWindow, TimestampRegressionReducesHealth) {
  ImuWindow window(5.0, 10.0);
  window.add(ImuSample{1.0, 0.0, 0.0, 9.8, 0.0, 0.0, 0.0});
  window.add(ImuSample{0.9, 0.0, 0.0, 9.8, 0.0, 0.0, 0.0});

  const ImuWindowStatus status = window.status();
  EXPECT_EQ(1, status.regression_count);
  EXPECT_LT(status.health_score, 1.0);
}

TEST(ImuWindow, DiscardsSamplesOutsideWindow) {
  ImuWindow window(1.0, 1.0);
  window.add(ImuSample{0.0, 0.0, 0.0, 9.8, 0.0, 0.0, 0.0});
  window.add(ImuSample{2.0, 0.0, 0.0, 9.8, 0.0, 0.0, 0.0});

  EXPECT_EQ(1, window.status().sample_count);
}

TEST(ImuWindow, EstimatesStationaryBiasAndGravityDirection) {
  ImuWindow window(5.0, 10.0);
  for (int index = 0; index < 20; ++index) {
    window.add(ImuSample{index * 0.05,
                         0.01,
                         -0.02,
                         9.80665,
                         0.012,
                         -0.004,
                         0.002});
  }

  const ImuWindowStatus status = window.status();
  ASSERT_TRUE(status.stationary);
  EXPECT_NEAR(0.012, status.gyro_bias.x, 1e-9);
  EXPECT_NEAR(-0.004, status.gyro_bias.y, 1e-9);
  EXPECT_NEAR(0.002, status.gyro_bias.z, 1e-9);
  EXPECT_NEAR(0.0, status.gravity_direction.x, 2e-3);
  EXPECT_NEAR(0.0, status.gravity_direction.y, 3e-3);
  EXPECT_NEAR(1.0, status.gravity_direction.z, 1e-5);
}

TEST(ImuWindow, RejectsBiasEstimateWhenWindowIsMoving) {
  ImuWindow window(5.0, 10.0);
  for (int index = 0; index < 20; ++index) {
    window.add(ImuSample{index * 0.05,
                         0.0,
                         0.0,
                         9.80665,
                         0.4,
                         0.0,
                         0.0});
  }

  const ImuWindowStatus status = window.status();
  EXPECT_FALSE(status.stationary);
  EXPECT_DOUBLE_EQ(0.0, status.gyro_bias.x);
  EXPECT_DOUBLE_EQ(0.0, status.gyro_bias.y);
  EXPECT_DOUBLE_EQ(0.0, status.gyro_bias.z);
}

TEST(ImuWindow, RejectsNonFiniteSamplesAndKeepsStatusFinite) {
  ImuWindow window(5.0, 10.0);
  window.add(ImuSample{0.0, 0.0, 0.0, 9.80665, 0.0, 0.0, 0.0});
  window.add(ImuSample{0.1,
                       std::numeric_limits<double>::quiet_NaN(),
                       0.0,
                       9.80665,
                       0.0,
                       0.0,
                       0.0});

  const ImuWindowStatus status = window.status();
  ASSERT_EQ(1, status.sample_count);
  EXPECT_TRUE(std::isfinite(status.mean_acc.x));
  EXPECT_TRUE(std::isfinite(status.mean_acc.y));
  EXPECT_TRUE(std::isfinite(status.mean_acc.z));
  EXPECT_TRUE(std::isfinite(status.health_score));
  ASSERT_NE(nullptr, window.lastSample());
  EXPECT_DOUBLE_EQ(0.0, window.lastSample()->stamp);
}

}  // namespace
}  // namespace lio_state_estimator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
