#pragma once

#include <string>
#include <vector>

namespace section_manager {

struct PointXYZ {
  double x;
  double y;
  double z;
};

struct PointYZ {
  double y;
  double z;
};

struct SectionConfig {
  double slice_thickness_m = 0.2;
  int angle_bins = 36;
  int min_points = 30;
  std::string section_model = "rectangle";
  double rectangle_width_m = 4.0;
  double rectangle_height_m = 2.0;
  double arch_wall_height_m = 1.0;
  double arch_roof_radius_m = 2.0;
};

struct SectionObservation {
  std::string session_id = "unassigned";
  double chainage_m = 0.0;
  std::string state_source = "unknown";
  std::vector<PointYZ> points_yz;
  double completeness = 0.0;
  double rmse_mm = 0.0;
  std::string quality = "C";

  int pointCount() const {
    return static_cast<int>(points_yz.size());
  }
};

struct SectionExportResult {
  std::string csv;
  std::size_t exported_count = 0;
};

struct SectionHistoryUpdate {
  bool inserted = false;
  bool replaced = false;
  std::size_t index = 0;
};

SectionObservation extractSectionPoints(const std::vector<PointXYZ>& points_xyz, double chainage_m, const SectionConfig& config);
double estimateAngularCompleteness(const std::vector<PointYZ>& points_yz, int angle_bins = 36);
double rectangularSectionRmse(const std::vector<PointYZ>& points_yz, double width_m, double height_m);
double archedSectionRmse(const std::vector<PointYZ>& points_yz, double width_m, double wall_height_m, double roof_radius_m);
std::string gradeSection(double completeness, double rmse_mm);
bool isBetterSection(const SectionObservation& candidate,
                     const SectionObservation& current);
SectionHistoryUpdate upsertSectionObservation(std::vector<SectionObservation>* sections,
                                              const SectionObservation& observation,
                                              double section_spacing_m);
SectionExportResult exportSectionsCsv(const std::vector<SectionObservation>& sections,
                                      double min_chainage_m,
                                      double max_chainage_m,
                                      const std::string& min_quality);

}  // namespace section_manager
