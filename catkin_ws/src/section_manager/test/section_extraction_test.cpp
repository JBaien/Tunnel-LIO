#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "section_manager/section_extraction.h"

TEST(SectionExtraction, ExtractsPointsNearRequestedChainage) {
  std::vector<section_manager::PointXYZ> points;
  points.push_back(section_manager::PointXYZ{9.95, -1.0, 0.0});
  points.push_back(section_manager::PointXYZ{10.02, 1.0, 0.0});
  points.push_back(section_manager::PointXYZ{10.20, 0.0, 1.0});
  section_manager::SectionConfig config;
  config.slice_thickness_m = 0.1;

  const section_manager::SectionObservation section = section_manager::extractSectionPoints(points, 10.0, config);

  ASSERT_EQ(2u, section.points_yz.size());
  EXPECT_DOUBLE_EQ(-1.0, section.points_yz[0].y);
  EXPECT_DOUBLE_EQ(0.0, section.points_yz[0].z);
  EXPECT_DOUBLE_EQ(1.0, section.points_yz[1].y);
  EXPECT_DOUBLE_EQ(0.0, section.points_yz[1].z);
  EXPECT_EQ(2, section.pointCount());
}

TEST(SectionExtraction, EstimatesAngularCompleteness) {
  std::vector<section_manager::PointYZ> points;
  points.push_back(section_manager::PointYZ{1.0, 0.0});
  points.push_back(section_manager::PointYZ{0.0, 1.0});
  points.push_back(section_manager::PointYZ{-1.0, 0.0});
  points.push_back(section_manager::PointYZ{0.0, -1.0});

  EXPECT_NEAR(1.0, section_manager::estimateAngularCompleteness(points, 4), 1e-9);
  points.resize(2);
  EXPECT_NEAR(0.5, section_manager::estimateAngularCompleteness(points, 4), 1e-9);
}

TEST(SectionExtraction, RectangularRmseAndGrade) {
  std::vector<section_manager::PointYZ> points;
  points.push_back(section_manager::PointYZ{-2.0, -1.0});
  points.push_back(section_manager::PointYZ{2.0, -1.0});
  points.push_back(section_manager::PointYZ{-2.0, 1.0});
  points.push_back(section_manager::PointYZ{2.0, 1.0});

  EXPECT_NEAR(0.0, section_manager::rectangularSectionRmse(points, 4.0, 2.0), 1e-9);
  EXPECT_EQ("A", section_manager::gradeSection(0.95, 20.0));
  EXPECT_EQ("B", section_manager::gradeSection(0.80, 35.0));
  EXPECT_EQ("C", section_manager::gradeSection(0.40, 60.0));
}

TEST(SectionExtraction, ArchedRmseAndExtractionModel) {
  std::vector<section_manager::PointYZ> points;
  points.push_back(section_manager::PointYZ{-2.0, 0.0});
  points.push_back(section_manager::PointYZ{2.0, 0.0});
  points.push_back(section_manager::PointYZ{-2.0, 0.8});
  points.push_back(section_manager::PointYZ{2.0, 0.8});
  points.push_back(section_manager::PointYZ{0.0, 3.0});
  points.push_back(section_manager::PointYZ{std::sqrt(2.0), 1.0 + std::sqrt(2.0)});
  points.push_back(section_manager::PointYZ{-std::sqrt(2.0), 1.0 + std::sqrt(2.0)});

  EXPECT_NEAR(0.0, section_manager::archedSectionRmse(points, 4.0, 1.0, 2.0), 1e-9);
  EXPECT_TRUE(std::isinf(section_manager::archedSectionRmse(points, -4.0, 1.0, 2.0)));

  std::vector<section_manager::PointXYZ> points_xyz;
  for (const auto& point : points) {
    points_xyz.push_back(section_manager::PointXYZ{12.0, point.y, point.z});
  }
  section_manager::SectionConfig config;
  config.slice_thickness_m = 0.2;
  config.section_model = "arch";
  config.rectangle_width_m = 4.0;
  config.arch_wall_height_m = 1.0;
  config.arch_roof_radius_m = 2.0;

  const section_manager::SectionObservation section =
      section_manager::extractSectionPoints(points_xyz, 12.0, config);

  ASSERT_EQ(points.size(), section.points_yz.size());
  EXPECT_NEAR(0.0, section.rmse_mm, 1e-9);
}

TEST(SectionExtraction, RejectsInvalidExtractionConfigAndPoints) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<section_manager::PointXYZ> points;
  points.push_back(section_manager::PointXYZ{10.0, -2.0, 1.0});
  points.push_back(section_manager::PointXYZ{10.0, nan, 1.0});
  points.push_back(section_manager::PointXYZ{10.0, 2.0, nan});
  points.push_back(section_manager::PointXYZ{nan, 2.0, 1.0});

  section_manager::SectionConfig config;
  config.slice_thickness_m = 0.2;

  const section_manager::SectionObservation section =
      section_manager::extractSectionPoints(points, 10.0, config);

  ASSERT_EQ(1u, section.points_yz.size());
  EXPECT_DOUBLE_EQ(-2.0, section.points_yz[0].y);
  EXPECT_DOUBLE_EQ(1.0, section.points_yz[0].z);

  config.slice_thickness_m = nan;
  const section_manager::SectionObservation invalid_config =
      section_manager::extractSectionPoints(points, 10.0, config);

  EXPECT_EQ(0, invalid_config.pointCount());
  EXPECT_DOUBLE_EQ(0.0, invalid_config.completeness);
  EXPECT_DOUBLE_EQ(0.0, invalid_config.rmse_mm);
  EXPECT_EQ("C", invalid_config.quality);

  config.slice_thickness_m = 0.2;
  const section_manager::SectionObservation invalid_chainage =
      section_manager::extractSectionPoints(points, nan, config);

  EXPECT_EQ(0, invalid_chainage.pointCount());
  EXPECT_DOUBLE_EQ(0.0, invalid_chainage.completeness);
  EXPECT_DOUBLE_EQ(0.0, invalid_chainage.rmse_mm);
  EXPECT_EQ("C", invalid_chainage.quality);

  std::vector<section_manager::PointYZ> yz_points;
  yz_points.push_back(section_manager::PointYZ{nan, 1.0});
  yz_points.push_back(section_manager::PointYZ{1.0, nan});
  yz_points.push_back(section_manager::PointYZ{1.0, 0.0});

  EXPECT_NEAR(0.25, section_manager::estimateAngularCompleteness(yz_points, 4), 1e-9);
  EXPECT_DOUBLE_EQ(0.0, section_manager::rectangularSectionRmse(yz_points, 2.0, 2.0));
}

TEST(SectionExtraction, RejectsNonFiniteSectionQualityAndHistoryInputs) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ("C", section_manager::gradeSection(0.95, -1.0));
  EXPECT_EQ("C", section_manager::gradeSection(nan, 20.0));
  EXPECT_EQ("C", section_manager::gradeSection(0.95, nan));

  section_manager::SectionObservation valid;
  valid.session_id = "session_alpha";
  valid.state_source = "IDLE_STATIC";
  valid.chainage_m = 10.0;
  valid.completeness = 0.90;
  valid.rmse_mm = 25.0;
  valid.quality = "A";
  valid.points_yz.push_back(section_manager::PointYZ{-2.0, 1.0});

  section_manager::SectionObservation invalid = valid;
  invalid.chainage_m = nan;
  invalid.completeness = 1.0;
  invalid.rmse_mm = 0.0;
  invalid.quality = "A";

  std::vector<section_manager::SectionObservation> history;
  EXPECT_TRUE(section_manager::upsertSectionObservation(&history, valid, 1.0).inserted);
  const section_manager::SectionHistoryUpdate rejected =
      section_manager::upsertSectionObservation(&history, invalid, 1.0);

  EXPECT_FALSE(rejected.inserted);
  EXPECT_FALSE(rejected.replaced);
  ASSERT_EQ(1u, history.size());
  EXPECT_DOUBLE_EQ(10.0, history[0].chainage_m);

  section_manager::SectionObservation polluted = valid;
  polluted.chainage_m = 10.2;
  polluted.completeness = nan;
  polluted.rmse_mm = 0.0;
  EXPECT_FALSE(section_manager::isBetterSection(polluted, valid));

  polluted = valid;
  polluted.chainage_m = 10.2;
  polluted.rmse_mm = nan;
  EXPECT_FALSE(section_manager::isBetterSection(polluted, valid));
}

TEST(SectionExport, WritesCsvForRequestedChainageRangeAndQuality) {
  section_manager::SectionObservation low_quality;
  low_quality.session_id = "session_alpha";
  low_quality.state_source = "CUTTING_STATIC";
  low_quality.chainage_m = 8.0;
  low_quality.completeness = 0.40;
  low_quality.rmse_mm = 60.0;
  low_quality.quality = "C";
  low_quality.points_yz.push_back(section_manager::PointYZ{0.0, 0.0});

  section_manager::SectionObservation accepted;
  accepted.session_id = "session_alpha";
  accepted.state_source = "IDLE_STATIC";
  accepted.chainage_m = 10.0;
  accepted.completeness = 0.92;
  accepted.rmse_mm = 18.0;
  accepted.quality = "A";
  accepted.points_yz.push_back(section_manager::PointYZ{-2.0, 1.0});
  accepted.points_yz.push_back(section_manager::PointYZ{2.0, 1.0});

  section_manager::SectionObservation outside;
  outside.session_id = "session_beta";
  outside.state_source = "FWD_MOVE";
  outside.chainage_m = 13.0;
  outside.completeness = 0.80;
  outside.rmse_mm = 35.0;
  outside.quality = "B";
  outside.points_yz.push_back(section_manager::PointYZ{0.0, 1.0});

  const std::vector<section_manager::SectionObservation> sections = {low_quality, accepted, outside};

  section_manager::SectionExportResult result =
      section_manager::exportSectionsCsv(sections, 9.0, 11.0, "B");

  EXPECT_EQ(1u, result.exported_count);
  EXPECT_NE(std::string::npos,
            result.csv.find("session_id,chainage_m,state_source,quality,completeness,rmse_mm,points"));
  EXPECT_NE(std::string::npos,
            result.csv.find("session_alpha,10.000,IDLE_STATIC,A,0.920,18.000,2"));
  EXPECT_EQ(std::string::npos, result.csv.find("session_alpha,8.000"));
  EXPECT_EQ(std::string::npos, result.csv.find("session_beta,13.000"));
}

TEST(SectionExport, SkipsInvalidRowsAndInvalidRanges) {
  const double nan = std::numeric_limits<double>::quiet_NaN();

  section_manager::SectionObservation valid;
  valid.session_id = "session_alpha";
  valid.state_source = "IDLE_STATIC";
  valid.chainage_m = 10.0;
  valid.completeness = 0.92;
  valid.rmse_mm = 18.0;
  valid.quality = "A";
  valid.points_yz.push_back(section_manager::PointYZ{-2.0, 1.0});

  section_manager::SectionObservation invalid_chainage = valid;
  invalid_chainage.chainage_m = nan;

  section_manager::SectionObservation invalid_quality = valid;
  invalid_quality.chainage_m = 10.1;
  invalid_quality.quality = "Z";

  section_manager::SectionObservation invalid_text = valid;
  invalid_text.chainage_m = 10.2;
  invalid_text.session_id = "session,beta";

  section_manager::SectionObservation invalid_metric = valid;
  invalid_metric.chainage_m = 10.3;
  invalid_metric.rmse_mm = -1.0;

  const std::vector<section_manager::SectionObservation> sections = {
      valid, invalid_chainage, invalid_quality, invalid_text, invalid_metric};

  section_manager::SectionExportResult result =
      section_manager::exportSectionsCsv(sections, 9.0, 11.0, "B");

  EXPECT_EQ(1u, result.exported_count);
  EXPECT_NE(std::string::npos, result.csv.find("session_alpha,10.000,IDLE_STATIC,A,0.920,18.000,1"));
  EXPECT_EQ(std::string::npos, result.csv.find("nan"));
  EXPECT_EQ(std::string::npos, result.csv.find("session,beta"));
  EXPECT_EQ(std::string::npos, result.csv.find(",Z,"));
  EXPECT_EQ(std::string::npos, result.csv.find("-1.000"));

  const section_manager::SectionExportResult invalid_range =
      section_manager::exportSectionsCsv(sections, 11.0, 9.0, "B");
  EXPECT_EQ(0u, invalid_range.exported_count);
  EXPECT_NE(std::string::npos,
            invalid_range.csv.find("session_id,chainage_m,state_source,quality,completeness,rmse_mm,points"));
  EXPECT_EQ(std::string::npos, invalid_range.csv.find("session_alpha,10.000"));
}

TEST(SectionHistory, AppendsSectionsOutsideSpacingWindow) {
  std::vector<section_manager::SectionObservation> history;
  section_manager::SectionObservation first;
  first.chainage_m = 10.0;
  first.quality = "B";
  first.completeness = 0.80;
  first.rmse_mm = 35.0;

  section_manager::SectionObservation next = first;
  next.chainage_m = 11.1;

  const section_manager::SectionHistoryUpdate first_update =
      section_manager::upsertSectionObservation(&history, first, 1.0);
  const section_manager::SectionHistoryUpdate second_update =
      section_manager::upsertSectionObservation(&history, next, 1.0);

  EXPECT_TRUE(first_update.inserted);
  EXPECT_FALSE(first_update.replaced);
  EXPECT_TRUE(second_update.inserted);
  EXPECT_FALSE(second_update.replaced);
  ASSERT_EQ(2u, history.size());
  EXPECT_DOUBLE_EQ(10.0, history[0].chainage_m);
  EXPECT_DOUBLE_EQ(11.1, history[1].chainage_m);
}

TEST(SectionHistory, KeepsBestSectionInsideSpacingWindow) {
  std::vector<section_manager::SectionObservation> history;
  section_manager::SectionObservation weak;
  weak.chainage_m = 20.0;
  weak.quality = "C";
  weak.completeness = 0.50;
  weak.rmse_mm = 80.0;
  weak.points_yz.push_back(section_manager::PointYZ{0.0, 1.0});

  section_manager::SectionObservation better = weak;
  better.chainage_m = 20.4;
  better.quality = "A";
  better.completeness = 0.95;
  better.rmse_mm = 20.0;
  better.points_yz.push_back(section_manager::PointYZ{1.0, 0.0});

  section_manager::SectionObservation worse = weak;
  worse.chainage_m = 20.2;
  worse.quality = "C";
  worse.completeness = 0.40;
  worse.rmse_mm = 90.0;

  EXPECT_TRUE(section_manager::upsertSectionObservation(&history, weak, 1.0).inserted);
  const section_manager::SectionHistoryUpdate replacement =
      section_manager::upsertSectionObservation(&history, better, 1.0);
  const section_manager::SectionHistoryUpdate rejected =
      section_manager::upsertSectionObservation(&history, worse, 1.0);

  EXPECT_FALSE(replacement.inserted);
  EXPECT_TRUE(replacement.replaced);
  EXPECT_FALSE(rejected.inserted);
  EXPECT_FALSE(rejected.replaced);
  ASSERT_EQ(1u, history.size());
  EXPECT_DOUBLE_EQ(20.4, history[0].chainage_m);
  EXPECT_EQ("A", history[0].quality);
  EXPECT_EQ(2, history[0].pointCount());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
