#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "lio_preprocess/deskewing.h"

TEST(ImuDeskewing, RotatesPointsToScanStartUsingPointTime) {
  std::vector<lio_preprocess::PointTuple> points;
  points.push_back(lio_preprocess::PointTuple{1.0, 0.0, 0.0, 0.0});
  points.push_back(lio_preprocess::PointTuple{1.0, 0.0, 0.0, 1.0});
  std::vector<lio_preprocess::ImuAngularSample> samples;
  samples.push_back(lio_preprocess::ImuAngularSample{0.0, 0.0, 0.0, M_PI / 2.0});
  samples.push_back(lio_preprocess::ImuAngularSample{1.0, 0.0, 0.0, M_PI / 2.0});

  lio_preprocess::DeskewConfig config;
  config.enabled = true;
  config.reference = "start";
  config.max_abs_point_time = 2.0;
  lio_preprocess::DeskewStats stats;
  const std::vector<lio_preprocess::PointTuple> corrected =
      lio_preprocess::deskewPointTuples(points, std::vector<std::string>{"x", "y", "z", "time"}, 0.0, samples, config, &stats);

  EXPECT_NEAR(1.0, corrected[0][0], 1e-6);
  EXPECT_NEAR(0.0, corrected[0][1], 1e-6);
  EXPECT_NEAR(0.0, corrected[1][0], 1e-6);
  EXPECT_NEAR(-1.0, corrected[1][1], 1e-6);
  EXPECT_EQ(2, stats.deskewed_points);
  EXPECT_EQ(0, stats.missing_time_field);
  EXPECT_TRUE(stats.used_imu);
}

TEST(ImuDeskewing, KeepsPointsWhenTimeFieldIsMissing) {
  std::vector<lio_preprocess::PointTuple> points;
  points.push_back(lio_preprocess::PointTuple{1.0, 0.0, 0.0, 20.0});
  std::vector<lio_preprocess::ImuAngularSample> samples;
  samples.push_back(lio_preprocess::ImuAngularSample{0.0, 0.0, 0.0, 1.0});

  lio_preprocess::DeskewConfig config;
  config.enabled = true;
  lio_preprocess::DeskewStats stats;
  const std::vector<lio_preprocess::PointTuple> corrected =
      lio_preprocess::deskewPointTuples(points, std::vector<std::string>{"x", "y", "z", "intensity"}, 0.0, samples, config, &stats);

  ASSERT_EQ(1u, corrected.size());
  EXPECT_DOUBLE_EQ(points[0][0], corrected[0][0]);
  EXPECT_EQ(1, stats.missing_time_field);
  EXPECT_FALSE(stats.used_imu);
}

TEST(ImuDeskewing, RejectsDuplicatePointTimeFieldNames) {
  EXPECT_EQ(
      -1,
      lio_preprocess::findTimeField(
          std::vector<std::string>{"x", "y", "z", "time", "time"},
          std::vector<std::string>{"time", "timestamp"}));
}

TEST(ImuDeskewing, DoesNotDeskewWhenPointTimeFieldIsAmbiguous) {
  std::vector<lio_preprocess::PointTuple> points;
  points.push_back(lio_preprocess::PointTuple{1.0, 0.0, 0.0, 0.0, 0.5});
  points.push_back(lio_preprocess::PointTuple{1.0, 0.0, 0.0, 1.0, 0.5});
  std::vector<lio_preprocess::ImuAngularSample> samples;
  samples.push_back(lio_preprocess::ImuAngularSample{0.0, 0.0, 0.0, M_PI / 2.0});
  samples.push_back(lio_preprocess::ImuAngularSample{1.0, 0.0, 0.0, M_PI / 2.0});

  lio_preprocess::DeskewConfig config;
  config.enabled = true;
  config.reference = "start";
  config.max_abs_point_time = 2.0;

  lio_preprocess::DeskewStats stats;
  const std::vector<lio_preprocess::PointTuple> corrected =
      lio_preprocess::deskewPointTuples(
          points,
          std::vector<std::string>{"x", "y", "z", "time", "time"},
          0.0,
          samples,
          config,
          &stats);

  ASSERT_EQ(points.size(), corrected.size());
  EXPECT_DOUBLE_EQ(points[1][0], corrected[1][0]);
  EXPECT_DOUBLE_EQ(points[1][1], corrected[1][1]);
  EXPECT_EQ(2, stats.missing_time_field);
  EXPECT_EQ(0, stats.deskewed_points);
  EXPECT_FALSE(stats.used_imu);
}

TEST(ImuDeskewing, RejectsPointTuplesWithoutXyzSlots) {
  std::vector<lio_preprocess::PointTuple> points;
  points.push_back(lio_preprocess::PointTuple{0.0});
  std::vector<lio_preprocess::ImuAngularSample> samples;
  samples.push_back(lio_preprocess::ImuAngularSample{0.0, 0.0, 0.0, M_PI / 2.0});
  samples.push_back(lio_preprocess::ImuAngularSample{1.0, 0.0, 0.0, M_PI / 2.0});

  lio_preprocess::DeskewConfig config;
  config.enabled = true;
  config.reference = "start";
  config.max_abs_point_time = 2.0;

  lio_preprocess::DeskewStats stats;
  const std::vector<lio_preprocess::PointTuple> corrected =
      lio_preprocess::deskewPointTuples(
          points,
          std::vector<std::string>{"time"},
          0.0,
          samples,
          config,
          &stats);

  ASSERT_EQ(1u, corrected.size());
  ASSERT_EQ(1u, corrected[0].size());
  EXPECT_DOUBLE_EQ(0.0, corrected[0][0]);
  EXPECT_EQ(1, stats.invalid_point_time);
  EXPECT_EQ(0, stats.deskewed_points);
  EXPECT_FALSE(stats.used_imu);
}

TEST(ImuDeskewing, RejectsNonFiniteImuSamples) {
  std::vector<lio_preprocess::PointTuple> points;
  points.push_back(lio_preprocess::PointTuple{1.0, 0.0, 0.0, 0.0});
  points.push_back(lio_preprocess::PointTuple{1.0, 0.0, 0.0, 1.0});

  std::vector<lio_preprocess::ImuAngularSample> samples;
  samples.push_back(lio_preprocess::ImuAngularSample{
      0.0,
      0.0,
      0.0,
      std::numeric_limits<double>::quiet_NaN()});

  lio_preprocess::DeskewConfig config;
  config.enabled = true;
  config.reference = "start";
  config.max_abs_point_time = 2.0;

  lio_preprocess::DeskewStats stats;
  const std::vector<lio_preprocess::PointTuple> corrected =
      lio_preprocess::deskewPointTuples(
          points,
          std::vector<std::string>{"x", "y", "z", "time"},
          0.0,
          samples,
          config,
          &stats);

  ASSERT_EQ(points.size(), corrected.size());
  EXPECT_DOUBLE_EQ(points[0][0], corrected[0][0]);
  EXPECT_DOUBLE_EQ(points[1][0], corrected[1][0]);
  EXPECT_DOUBLE_EQ(points[1][1], corrected[1][1]);
  EXPECT_EQ(0, stats.deskewed_points);
  EXPECT_EQ(2, stats.missing_imu);
  EXPECT_FALSE(stats.used_imu);
}

TEST(ImuDeskewing, RejectsInvalidDeskewConfig) {
  std::vector<lio_preprocess::PointTuple> points;
  points.push_back(lio_preprocess::PointTuple{1.0, 0.0, 0.0, 1.0});

  std::vector<lio_preprocess::ImuAngularSample> samples;
  samples.push_back(lio_preprocess::ImuAngularSample{0.0, 0.0, 0.0, M_PI / 2.0});

  lio_preprocess::DeskewConfig config;
  config.enabled = true;
  config.reference = "middle";
  config.max_abs_point_time = std::numeric_limits<double>::quiet_NaN();

  lio_preprocess::DeskewStats stats;
  const std::vector<lio_preprocess::PointTuple> corrected =
      lio_preprocess::deskewPointTuples(
          points,
          std::vector<std::string>{"x", "y", "z", "time"},
          0.0,
          samples,
          config,
          &stats);

  ASSERT_EQ(1u, corrected.size());
  EXPECT_DOUBLE_EQ(points[0][0], corrected[0][0]);
  EXPECT_DOUBLE_EQ(points[0][1], corrected[0][1]);
  EXPECT_EQ(0, stats.deskewed_points);
  EXPECT_FALSE(stats.used_imu);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
