#include <gtest/gtest.h>

#include <limits>

#include "slam_backend_manager/backend_candidates.h"

namespace slam_backend_manager {
namespace {

TEST(BackendCandidates, DescriptorCountsPointsByRadiusRing) {
  const HistogramDescriptor descriptor =
      makeHistogramDescriptor({Point3{1.0, 0.0, 0.0}, Point3{3.0, 0.0, 0.0}, Point3{9.0, 0.0, 0.0}},
                              std::vector<double>{0.0, 2.0, 5.0, 10.0});
  ASSERT_EQ(3u, descriptor.bins.size());
  EXPECT_EQ(1, descriptor.bins[0]);
  EXPECT_EQ(1, descriptor.bins[1]);
  EXPECT_EQ(1, descriptor.bins[2]);
}

TEST(BackendCandidates, ScanContextSimilarityAlignsRotatedSectors) {
  const std::vector<double> ring_edges{0.0, 5.0, 10.0};
  const HistogramDescriptor left =
      makeScanContextDescriptor({Point3{3.0, 0.0, 0.0}, Point3{8.0, 0.0, 0.0}}, ring_edges, 4);
  const HistogramDescriptor rotated =
      makeScanContextDescriptor({Point3{0.0, 3.0, 0.0}, Point3{0.0, 8.0, 0.0}}, ring_edges, 4);

  EXPECT_LT(descriptorSimilarity(left.bins, rotated.bins), 0.6);
  EXPECT_NEAR(1.0, descriptorSimilarity(left.bins, rotated.bins, 4), 1e-6);

  BackendConfig config;
  config.ring_edges = ring_edges;
  config.sector_count = 4;
  config.min_loop_score = 0.9;
  config.min_top_score_ratio = 1.2;
  config.min_loop_chainage_separation_m = 5.0;
  OptionalLoopCandidate match =
      chooseLoopCandidate(Keyframe{"now", 20.0, left.bins}, {Keyframe{"rotated", 0.0, rotated.bins}}, config);
  ASSERT_TRUE(match.has_value);
  EXPECT_EQ("rotated", match.value.keyframe_id);
}

TEST(BackendCandidates, ConfigurableScanContextCapsDenseBins) {
  DescriptorConfig descriptor_config;
  descriptor_config.ring_edges = {0.0, 5.0};
  descriptor_config.sector_count = 4;
  descriptor_config.max_geometry_bin_count = 2;

  const HistogramDescriptor descriptor = makeScanContextDescriptor(
      {Point3{3.0, 0.0, 0.0}, Point3{3.2, 0.0, 0.0}, Point3{3.4, 0.0, 0.0}},
      descriptor_config);

  ASSERT_EQ(4u, descriptor.bins.size());
  EXPECT_EQ(2, descriptor.bins[0]);
}

TEST(BackendCandidates, ConfigurableIntensityContextQuantizesAndRequiresOccupancy) {
  DescriptorConfig descriptor_config;
  descriptor_config.ring_edges = {0.0, 5.0};
  descriptor_config.sector_count = 4;
  descriptor_config.intensity_quantization = 10.0;
  descriptor_config.min_intensity_bin_points = 2;

  const HistogramDescriptor descriptor = makeIntensityScanContextDescriptor(
      {Point3{3.0, 0.0, 0.0, 14.0}, Point3{3.2, 0.0, 0.0, 16.0},
       Point3{0.0, 3.0, 0.0, 90.0}},
      descriptor_config);

  ASSERT_EQ(4u, descriptor.bins.size());
  EXPECT_EQ(2, descriptor.bins[0]);
  EXPECT_EQ(0, descriptor.bins[1]);
}

TEST(BackendCandidates, InvalidDescriptorConfigProducesEmptyContexts) {
  DescriptorConfig descriptor_config;
  descriptor_config.ring_edges = {0.0, 5.0, 4.0};
  descriptor_config.sector_count = 4;

  EXPECT_TRUE(makeScanContextDescriptor({Point3{3.0, 0.0, 0.0}}, descriptor_config).bins.empty());
  EXPECT_TRUE(makeIntensityScanContextDescriptor({Point3{3.0, 0.0, 0.0, 42.0}}, descriptor_config)
                  .bins.empty());

  descriptor_config.ring_edges = {0.0, std::numeric_limits<double>::quiet_NaN(), 10.0};
  EXPECT_TRUE(makeScanContextDescriptor({Point3{3.0, 0.0, 0.0}}, descriptor_config).bins.empty());
  EXPECT_TRUE(makeIntensityScanContextDescriptor({Point3{3.0, 0.0, 0.0, 42.0}}, descriptor_config)
                  .bins.empty());

  descriptor_config.ring_edges = {0.0, 5.0};
  descriptor_config.sector_count = 0;
  EXPECT_TRUE(makeScanContextDescriptor({Point3{3.0, 0.0, 0.0}}, descriptor_config).bins.empty());
  EXPECT_TRUE(makeIntensityScanContextDescriptor({Point3{3.0, 0.0, 0.0, 42.0}}, descriptor_config)
                  .bins.empty());
}

TEST(BackendCandidates, ScanContextDescriptorRejectsNonFinitePoints) {
  DescriptorConfig descriptor_config;
  descriptor_config.ring_edges = {0.0, 5.0};
  descriptor_config.sector_count = 4;

  const HistogramDescriptor descriptor = makeScanContextDescriptor(
      {Point3{3.0, 0.0, 0.0}, Point3{std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0}},
      descriptor_config);

  EXPECT_TRUE(descriptor.bins.empty());
}

TEST(BackendCandidates, IntensityContextRejectsNonFiniteOrNegativeIntensity) {
  DescriptorConfig descriptor_config;
  descriptor_config.ring_edges = {0.0, 5.0};
  descriptor_config.sector_count = 4;
  descriptor_config.intensity_quantization = 10.0;

  EXPECT_TRUE(makeIntensityScanContextDescriptor(
                  {Point3{3.0, 0.0, 0.0, std::numeric_limits<double>::infinity()}},
                  descriptor_config)
                  .bins.empty());
  EXPECT_TRUE(makeIntensityScanContextDescriptor({Point3{3.0, 0.0, 0.0, -1.0}},
                                                descriptor_config)
                  .bins.empty());
}

TEST(BackendCandidates, RejectsAmbiguousLoopCandidate) {
  BackendConfig config;
  config.min_loop_score = 0.5;
  config.min_top_score_ratio = 1.5;
  const OptionalLoopCandidate match =
      chooseLoopCandidate(Keyframe{"now", 30.0, std::vector<int>{4, 2, 1}},
                          {Keyframe{"a", 0.0, std::vector<int>{4, 2, 1}},
                           Keyframe{"b", 10.0, std::vector<int>{4, 2, 1}}},
                          config);
  EXPECT_FALSE(match.has_value);
}

TEST(BackendCandidates, DescriptorSimilarityRejectsNegativeBins) {
  EXPECT_DOUBLE_EQ(0.0, descriptorSimilarity(std::vector<int>{-1, 2},
                                            std::vector<int>{-1, 2}));

  BackendConfig config;
  config.min_loop_score = 0.5;
  config.min_top_score_ratio = 1.0;
  config.min_loop_chainage_separation_m = 5.0;

  const OptionalLoopCandidate match =
      chooseLoopCandidate(Keyframe{"now", 30.0, std::vector<int>{-1, 2}},
                          {Keyframe{"candidate", 0.0, std::vector<int>{-1, 2}}},
                          config);
  EXPECT_FALSE(match.has_value);
}

TEST(BackendCandidates, CandidateSelectionRejectsNonFiniteChainage) {
  BackendConfig config;
  config.min_loop_score = 0.5;
  config.min_top_score_ratio = 1.0;
  config.min_loop_chainage_separation_m = 5.0;

  const Keyframe valid_candidate{"candidate", 0.0, std::vector<int>{4, 2, 1}};
  EXPECT_FALSE(chooseLoopCandidate(
                   Keyframe{"now", std::numeric_limits<double>::quiet_NaN(), std::vector<int>{4, 2, 1}},
                   {valid_candidate},
                   config)
                   .has_value);

  const Keyframe current{"now", 30.0, std::vector<int>{4, 2, 1}};
  EXPECT_FALSE(chooseLoopCandidate(
                   current,
                   {Keyframe{"candidate", std::numeric_limits<double>::infinity(), std::vector<int>{4, 2, 1}}},
                   config)
                   .has_value);
}

TEST(BackendCandidates, CandidateSelectionRejectsPollutedKeyframeIds) {
  BackendConfig config;
  config.min_loop_score = 0.5;
  config.min_top_score_ratio = 1.0;
  config.min_loop_chainage_separation_m = 5.0;

  const Keyframe valid_candidate{"candidate", 0.0, std::vector<int>{4, 2, 1}};
  EXPECT_FALSE(chooseLoopCandidate(
                   Keyframe{"now;candidate=spoof", 30.0, std::vector<int>{4, 2, 1}},
                   {valid_candidate},
                   config)
                   .has_value);

  const Keyframe current{"now", 30.0, std::vector<int>{4, 2, 1}};
  const OptionalLoopCandidate match =
      chooseLoopCandidate(current,
                          {Keyframe{"candidate;score=spoof", 0.0,
                                    std::vector<int>{4, 2, 1}}},
                          config);
  EXPECT_FALSE(match.has_value);
}

TEST(BackendCandidates, InvalidCandidateConfigFailsClosed) {
  BackendConfig config;
  config.min_loop_score = 0.95;
  config.min_top_score_ratio = 1.0;
  config.min_loop_chainage_separation_m = 5.0;

  const Keyframe identical_current{"now", 30.0, std::vector<int>{4, 2, 1}};
  const Keyframe identical_candidate{"candidate", 0.0, std::vector<int>{4, 2, 1}};

  config.sector_count = 0;
  EXPECT_FALSE(chooseLoopCandidate(identical_current, {identical_candidate}, config).has_value);

  config.sector_count = 4;
  Keyframe current{"now", 30.0, std::vector<int>{5, 0, 0, 0, 0, 3, 0, 0}};
  current.intensity_descriptor = std::vector<int>{9, 8, 7, 6, 5, 4, 3, 2};
  Keyframe candidate{"candidate", 0.0, std::vector<int>{5, 0, 0, 0, 0, 0, 3, 0}};
  candidate.intensity_descriptor = current.intensity_descriptor;

  config.intensity_descriptor_weight = 1.5;
  EXPECT_FALSE(chooseLoopCandidate(current, {candidate}, config).has_value);

  config.intensity_descriptor_weight = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(chooseLoopCandidate(current, {candidate}, config).has_value);
}

TEST(BackendCandidates, IntensityContextBreaksGeometricAmbiguity) {
  BackendConfig config;
  config.min_loop_score = 0.5;
  config.min_top_score_ratio = 1.2;
  config.intensity_descriptor_weight = 0.4;
  config.min_loop_chainage_separation_m = 5.0;

  Keyframe current{"now", 30.0, std::vector<int>{4, 2, 1}};
  current.intensity_descriptor = std::vector<int>{20, 80, 40};

  Keyframe matching_intensity{"matching", 0.0, std::vector<int>{4, 2, 1}};
  matching_intensity.intensity_descriptor = std::vector<int>{20, 80, 40};

  Keyframe wrong_intensity{"wrong", 10.0, std::vector<int>{4, 2, 1}};
  wrong_intensity.intensity_descriptor = std::vector<int>{90, 10, 90};

  const OptionalLoopCandidate match =
      chooseLoopCandidate(current, {matching_intensity, wrong_intensity}, config);

  ASSERT_TRUE(match.has_value);
  EXPECT_EQ("matching", match.value.keyframe_id);
  EXPECT_GT(match.value.score, 0.85);
}

TEST(BackendCandidates, AcceptsUniqueLoopCandidateAndStablePromotion) {
  BackendConfig config;
  config.min_loop_score = 0.5;
  config.min_top_score_ratio = 1.5;
  const OptionalLoopCandidate match =
      chooseLoopCandidate(Keyframe{"now", 30.0, std::vector<int>{4, 2, 1}},
                          {Keyframe{"a", 0.0, std::vector<int>{4, 2, 1}},
                           Keyframe{"b", 10.0, std::vector<int>{0, 0, 4}}},
                          config);
  ASSERT_TRUE(match.has_value);
  EXPECT_EQ("a", match.value.keyframe_id);

  StableMapPolicy policy("B", true);
  EXPECT_TRUE(policy.canPromote("FWD_MOVE", "A", false));
  EXPECT_FALSE(policy.canPromote("CONFLICT", "A", false));
  EXPECT_FALSE(policy.canPromote("FWD_MOVE", "C", false));
  EXPECT_FALSE(policy.canPromote("FWD_MOVE", "A", true));
}

TEST(BackendCandidates, StablePromotionRejectsInvalidMinimumQuality) {
  StableMapPolicy invalid_policy("invalid", false);

  EXPECT_FALSE(invalid_policy.canPromote("IDLE_STATIC", "A", false));
  EXPECT_FALSE(invalid_policy.canPromote("FWD_MOVE", "B", false));
  EXPECT_FALSE(invalid_policy.canPromote("REV_MOVE", "C", false));
}

TEST(BackendCandidates, GeometricVerificationRejectsShapeMismatch) {
  BackendConfig config;
  config.min_geometric_score = 0.75;
  config.max_centroid_distance_m = 0.5;

  Keyframe current{"now", 40.0, std::vector<int>{4, 2, 1}};
  current.geometry = makeGeometrySummary(
      {Point3{0.0, -2.0, -1.0}, Point3{0.0, 2.0, 1.0}, Point3{8.0, 0.0, 0.0}});
  Keyframe similar{"similar", 0.0, std::vector<int>{4, 2, 1}};
  similar.geometry = makeGeometrySummary(
      {Point3{0.1, -2.1, -1.0}, Point3{0.1, 2.0, 1.0}, Point3{8.1, 0.0, 0.0}});
  Keyframe wide{"wide", 0.0, std::vector<int>{4, 2, 1}};
  wide.geometry = makeGeometrySummary(
      {Point3{0.0, -8.0, -1.0}, Point3{0.0, 8.0, 1.0}, Point3{8.0, 0.0, 0.0}});

  EXPECT_TRUE(verifyLoopGeometry(current, similar, config).accepted);
  const LoopVerification rejected = verifyLoopGeometry(current, wide, config);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ("geometry_score_low", rejected.reason);
}

TEST(BackendCandidates, GeometrySummaryRejectsNonFinitePoints) {
  const GeometrySummary invalid = makeGeometrySummary(
      {Point3{0.0, 0.0, 0.0}, Point3{std::numeric_limits<double>::infinity(), 1.0, 0.0}});
  EXPECT_FALSE(invalid.valid);

  BackendConfig config;
  Keyframe current{"now", 40.0, std::vector<int>{4, 2, 1}};
  current.geometry = invalid;
  Keyframe candidate{"candidate", 0.0, std::vector<int>{4, 2, 1}};
  candidate.geometry = makeGeometrySummary(
      {Point3{0.0, -2.0, -1.0}, Point3{0.0, 2.0, 1.0}, Point3{8.0, 0.0, 0.0}});

  const LoopVerification result = verifyLoopGeometry(current, candidate, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ("missing_geometry", result.reason);
}

TEST(BackendCandidates, GeometricVerificationRejectsInvalidThresholds) {
  Keyframe current{"now", 40.0, std::vector<int>{4, 2, 1}};
  current.geometry = makeGeometrySummary(
      {Point3{0.0, -2.0, -1.0}, Point3{0.0, 2.0, 1.0}, Point3{8.0, 0.0, 0.0}});
  Keyframe candidate{"candidate", 0.0, std::vector<int>{4, 2, 1}};
  candidate.geometry = makeGeometrySummary(
      {Point3{0.1, -2.1, -1.0}, Point3{0.1, 2.0, 1.0}, Point3{8.1, 0.0, 0.0}});

  BackendConfig config;
  config.min_geometric_score = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(verifyLoopGeometry(current, candidate, config).accepted);

  config.min_geometric_score = 0.75;
  config.max_centroid_distance_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(verifyLoopGeometry(current, candidate, config).accepted);
}

TEST(BackendCandidates, IcpVerificationAcceptsRigidlyAlignedPoints) {
  BackendConfig config;
  config.max_icp_rmse_m = 0.05;
  config.min_icp_inlier_ratio = 0.9;
  config.icp_iterations = 20;
  const std::vector<Point3> candidate_points = {
      {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 1.0, 0.0}, {0.0, 3.0, 0.0}, {3.0, 4.0, 0.0}};
  const std::vector<Point3> current_points = {
      {2.0, -1.0, 0.0}, {2.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, -1.0, 0.0}, {-2.0, 2.0, 0.0}};

  const IcpVerification result = verifyLoopIcp(current_points, candidate_points, config);
  EXPECT_TRUE(result.accepted);
  EXPECT_LT(result.rmse_m, 0.05);
  EXPECT_GE(result.inlier_ratio, 0.9);
}

TEST(BackendCandidates, IcpVerificationRejectsNonFiniteInputPoints) {
  BackendConfig config;
  config.max_icp_rmse_m = 0.05;
  config.min_icp_inlier_ratio = 0.9;
  config.icp_iterations = 20;
  const std::vector<Point3> valid_points = {
      {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 1.0, 0.0}, {0.0, 3.0, 0.0}};
  const std::vector<Point3> invalid_points = {
      {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0},
      {0.0, 3.0, 0.0}};

  const IcpVerification current_invalid = verifyLoopIcp(invalid_points, valid_points, config);
  EXPECT_FALSE(current_invalid.accepted);
  EXPECT_EQ("invalid_icp_points", current_invalid.reason);

  const IcpVerification candidate_invalid = verifyLoopIcp(valid_points, invalid_points, config);
  EXPECT_FALSE(candidate_invalid.accepted);
  EXPECT_EQ("invalid_icp_points", candidate_invalid.reason);
}

TEST(BackendCandidates, IcpVerificationRejectsInvalidConfig) {
  const std::vector<Point3> points = {
      {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 1.0, 0.0}, {0.0, 3.0, 0.0}};

  BackendConfig config;
  config.max_icp_rmse_m = std::numeric_limits<double>::quiet_NaN();
  IcpVerification result = verifyLoopIcp(points, points, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ("invalid_icp_config", result.reason);

  config.max_icp_rmse_m = 0.05;
  config.min_icp_inlier_ratio = 1.5;
  result = verifyLoopIcp(points, points, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ("invalid_icp_config", result.reason);

  config.min_icp_inlier_ratio = 0.9;
  config.icp_inlier_threshold_m = -0.1;
  result = verifyLoopIcp(points, points, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ("invalid_icp_config", result.reason);
}

TEST(BackendCandidates, IcpVerificationRejectsPoorAlignment) {
  BackendConfig config;
  config.max_icp_rmse_m = 0.05;
  config.min_icp_inlier_ratio = 0.9;
  config.icp_iterations = 20;
  const std::vector<Point3> current_points = {
      {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}, {2.0, 2.0, 0.0}};
  const std::vector<Point3> candidate_points = {
      {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 10.0, 0.0}};

  const IcpVerification result = verifyLoopIcp(current_points, candidate_points, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ("icp_rmse_high", result.reason);
}

}  // namespace
}  // namespace slam_backend_manager

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
