#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "lio_preprocess/filtering.h"

TEST(PointFiltering, KeepsValidPointOutsideBodyCrop) {
  lio_preprocess::PointFilterConfig config;
  config.min_range = 0.4;
  config.max_range = 10.0;
  config.enable_body_crop = true;
  config.body_crop = lio_preprocess::CropBox{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

  const lio_preprocess::KeepDecision decision = lio_preprocess::shouldKeepXYZ(2.0, 0.0, 0.0, config);

  EXPECT_TRUE(decision.keep);
  EXPECT_EQ("", decision.reason);
}

TEST(PointFiltering, RejectsNanRangeAndBodyPoints) {
  lio_preprocess::PointFilterConfig config;
  config.min_range = 0.4;
  config.max_range = 10.0;
  config.enable_body_crop = true;
  config.body_crop = lio_preprocess::CropBox{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

  std::vector<lio_preprocess::PointTuple> points;
  points.push_back(lio_preprocess::PointTuple{2.0, 0.0, 0.0, 10.0});
  points.push_back(lio_preprocess::PointTuple{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 11.0});
  points.push_back(lio_preprocess::PointTuple{0.1, 0.0, 0.0, 12.0});
  points.push_back(lio_preprocess::PointTuple{0.5, 0.0, 0.0, 13.0});

  lio_preprocess::FilterStats stats;
  const std::vector<lio_preprocess::PointTuple> kept = lio_preprocess::filterPointTuples(points, config, &stats);

  ASSERT_EQ(1u, kept.size());
  EXPECT_DOUBLE_EQ(2.0, kept[0][0]);
  EXPECT_EQ(4, stats.input_points);
  EXPECT_EQ(1, stats.output_points);
  EXPECT_EQ(1, stats.dropped_nan);
  EXPECT_EQ(1, stats.dropped_range);
  EXPECT_EQ(1, stats.dropped_body);
}

TEST(PointFiltering, RejectsPointTuplesWithoutXyzSlots) {
  lio_preprocess::PointFilterConfig config;
  config.min_range = 0.4;
  config.max_range = 10.0;
  config.enable_body_crop = false;

  std::vector<lio_preprocess::PointTuple> points;
  points.push_back(lio_preprocess::PointTuple{2.0});
  points.push_back(lio_preprocess::PointTuple{2.0, 0.0});
  points.push_back(lio_preprocess::PointTuple{2.0, 0.0, 0.0});

  lio_preprocess::FilterStats stats;
  const std::vector<lio_preprocess::PointTuple> kept =
      lio_preprocess::filterPointTuples(points, config, &stats);

  ASSERT_EQ(1u, kept.size());
  EXPECT_DOUBLE_EQ(2.0, kept[0][0]);
  EXPECT_EQ(3, stats.input_points);
  EXPECT_EQ(1, stats.output_points);
  EXPECT_EQ(2, stats.dropped_nan);
}

TEST(PointFiltering, RejectsInvalidRangeConfig) {
  lio_preprocess::PointFilterConfig config;
  config.min_range = std::numeric_limits<double>::quiet_NaN();
  config.max_range = 10.0;
  config.enable_body_crop = false;

  const lio_preprocess::KeepDecision decision = lio_preprocess::shouldKeepXYZ(2.0, 0.0, 0.0, config);

  EXPECT_FALSE(decision.keep);
  EXPECT_EQ("range", decision.reason);
}

TEST(PointFiltering, RejectsInvalidBodyCropConfig) {
  lio_preprocess::PointFilterConfig config;
  config.min_range = 0.4;
  config.max_range = 10.0;
  config.enable_body_crop = true;
  config.body_crop = lio_preprocess::CropBox{std::numeric_limits<double>::quiet_NaN(), 1.0, -1.0, 1.0, -1.0, 1.0};

  const lio_preprocess::KeepDecision decision = lio_preprocess::shouldKeepXYZ(2.0, 0.0, 0.0, config);

  EXPECT_FALSE(decision.keep);
  EXPECT_EQ("body", decision.reason);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
