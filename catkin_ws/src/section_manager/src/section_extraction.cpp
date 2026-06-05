#include "section_manager/section_extraction.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>

namespace section_manager {

namespace {

bool finiteDouble(double value) {
  return std::isfinite(value);
}

bool finitePoint(const PointXYZ& point) {
  return finiteDouble(point.x) && finiteDouble(point.y) && finiteDouble(point.z);
}

bool finitePoint(const PointYZ& point) {
  return finiteDouble(point.y) && finiteDouble(point.z);
}

bool validSectionConfig(const SectionConfig& config) {
  const bool common_valid =
      finiteDouble(config.slice_thickness_m) && config.slice_thickness_m > 0.0 &&
      config.angle_bins > 0 && config.min_points >= 0 &&
      finiteDouble(config.rectangle_width_m) && config.rectangle_width_m > 0.0 &&
      finiteDouble(config.rectangle_height_m) && config.rectangle_height_m > 0.0;
  if (!common_valid) {
    return false;
  }
  if (config.section_model == "rectangle") {
    return true;
  }
  if (config.section_model == "arch") {
    return finiteDouble(config.arch_wall_height_m) && config.arch_wall_height_m > 0.0 &&
           finiteDouble(config.arch_roof_radius_m) && config.arch_roof_radius_m > 0.0;
  }
  return false;
}

SectionObservation rejectedObservation(double chainage_m) {
  SectionObservation observation;
  if (finiteDouble(chainage_m)) {
    observation.chainage_m = chainage_m;
  }
  observation.completeness = 0.0;
  observation.rmse_mm = 0.0;
  observation.quality = "C";
  return observation;
}

int qualityRank(const std::string& quality) {
  if (quality == "A") {
    return 3;
  }
  if (quality == "B") {
    return 2;
  }
  if (quality == "C") {
    return 1;
  }
  return 0;
}

bool validQuality(const std::string& quality) {
  return qualityRank(quality) > 0;
}

bool validSectionMetrics(const SectionObservation& observation) {
  return finiteDouble(observation.chainage_m) &&
         finiteDouble(observation.completeness) && observation.completeness >= 0.0 &&
         observation.completeness <= 1.0 &&
         finiteDouble(observation.rmse_mm) && observation.rmse_mm >= 0.0 &&
         validQuality(observation.quality);
}

bool validCsvText(const std::string& value) {
  if (value.empty()) {
    return false;
  }
  for (std::string::const_iterator it = value.begin(); it != value.end(); ++it) {
    if (*it == ',' || *it == ';' || *it == '\n' || *it == '\r') {
      return false;
    }
  }
  return true;
}

bool validExportSection(const SectionObservation& observation) {
  if (!validSectionMetrics(observation) ||
      !validCsvText(observation.session_id) ||
      !validCsvText(observation.state_source) ||
      observation.pointCount() <= 0) {
    return false;
  }
  for (std::vector<PointYZ>::const_iterator it = observation.points_yz.begin();
       it != observation.points_yz.end(); ++it) {
    if (!finitePoint(*it)) {
      return false;
    }
  }
  return true;
}

}  // namespace

SectionObservation extractSectionPoints(const std::vector<PointXYZ>& points_xyz, double chainage_m, const SectionConfig& config) {
  SectionObservation observation = rejectedObservation(chainage_m);
  if (!finiteDouble(chainage_m) || !validSectionConfig(config)) {
    return observation;
  }
  const double half = config.slice_thickness_m * 0.5;
  const double epsilon = 1e-9;
  for (std::vector<PointXYZ>::const_iterator it = points_xyz.begin(); it != points_xyz.end(); ++it) {
    if (finitePoint(*it) && std::fabs(it->x - chainage_m) <= half + epsilon) {
      observation.points_yz.push_back(PointYZ{it->y, it->z});
    }
  }
  if (observation.points_yz.empty()) {
    return observation;
  }
  observation.completeness = estimateAngularCompleteness(observation.points_yz, config.angle_bins);
  if (config.section_model == "arch") {
    observation.rmse_mm = archedSectionRmse(
        observation.points_yz,
        config.rectangle_width_m,
        config.arch_wall_height_m,
        config.arch_roof_radius_m);
  } else {
    observation.rmse_mm = rectangularSectionRmse(
        observation.points_yz,
        config.rectangle_width_m,
        config.rectangle_height_m);
  }
  observation.quality = gradeSection(observation.completeness, observation.rmse_mm);
  return observation;
}

double estimateAngularCompleteness(const std::vector<PointYZ>& points_yz, int angle_bins) {
  if (angle_bins <= 0) {
    return 0.0;
  }
  std::set<int> occupied;
  for (std::vector<PointYZ>::const_iterator it = points_yz.begin(); it != points_yz.end(); ++it) {
    if (!finitePoint(*it)) {
      continue;
    }
    if (it->y == 0.0 && it->z == 0.0) {
      continue;
    }
    double angle = std::atan2(it->z, it->y);
    if (angle < 0.0) {
      angle += 2.0 * M_PI;
    }
    occupied.insert(static_cast<int>(angle / (2.0 * M_PI) * angle_bins) % angle_bins);
  }
  const double completeness = static_cast<double>(occupied.size()) / static_cast<double>(angle_bins);
  return completeness > 1.0 ? 1.0 : completeness;
}

double rectangularSectionRmse(const std::vector<PointYZ>& points_yz, double width_m, double height_m) {
  if (points_yz.empty() || !finiteDouble(width_m) || !finiteDouble(height_m) ||
      width_m <= 0.0 || height_m <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double half_w = width_m * 0.5;
  const double half_h = height_m * 0.5;
  double squared_sum = 0.0;
  std::size_t valid_points = 0;
  for (std::vector<PointYZ>::const_iterator it = points_yz.begin(); it != points_yz.end(); ++it) {
    if (!finitePoint(*it)) {
      continue;
    }
    const double distance_to_wall = std::min(std::fabs(std::fabs(it->y) - half_w), std::fabs(std::fabs(it->z) - half_h));
    squared_sum += distance_to_wall * distance_to_wall;
    ++valid_points;
  }
  if (valid_points == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(squared_sum / static_cast<double>(valid_points)) * 1000.0;
}

double archedSectionRmse(const std::vector<PointYZ>& points_yz, double width_m, double wall_height_m, double roof_radius_m) {
  if (points_yz.empty() || !finiteDouble(width_m) || !finiteDouble(wall_height_m) ||
      !finiteDouble(roof_radius_m) || width_m <= 0.0 || wall_height_m <= 0.0 ||
      roof_radius_m <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  const double half_w = width_m * 0.5;
  double squared_sum = 0.0;
  std::size_t valid_points = 0;
  for (std::vector<PointYZ>::const_iterator it = points_yz.begin(); it != points_yz.end(); ++it) {
    if (!finitePoint(*it)) {
      continue;
    }
    const double floor_distance = std::fabs(it->z);
    const double wall_distance = std::fabs(std::fabs(it->y) - half_w);
    const double roof_radius = std::sqrt(it->y * it->y + (it->z - wall_height_m) * (it->z - wall_height_m));
    const double roof_distance = std::fabs(roof_radius - roof_radius_m);
    double boundary_distance = floor_distance;
    if (it->z <= wall_height_m + 1e-9) {
      boundary_distance = std::min(boundary_distance, wall_distance);
    }
    if (it->z >= wall_height_m - 1e-9) {
      boundary_distance = std::min(boundary_distance, roof_distance);
    }
    squared_sum += boundary_distance * boundary_distance;
    ++valid_points;
  }
  if (valid_points == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(squared_sum / static_cast<double>(valid_points)) * 1000.0;
}

std::string gradeSection(double completeness, double rmse_mm) {
  if (!finiteDouble(completeness) || completeness < 0.0 || completeness > 1.0 ||
      !finiteDouble(rmse_mm) || rmse_mm < 0.0) {
    return "C";
  }
  if (completeness >= 0.90 && rmse_mm <= 25.0) {
    return "A";
  }
  if (completeness >= 0.75 && rmse_mm <= 40.0) {
    return "B";
  }
  return "C";
}

bool isBetterSection(const SectionObservation& candidate,
                     const SectionObservation& current) {
  if (!validSectionMetrics(candidate)) {
    return false;
  }
  if (!validSectionMetrics(current)) {
    return true;
  }
  const int candidate_rank = qualityRank(candidate.quality);
  const int current_rank = qualityRank(current.quality);
  if (candidate_rank != current_rank) {
    return candidate_rank > current_rank;
  }
  if (std::fabs(candidate.completeness - current.completeness) > 1e-9) {
    return candidate.completeness > current.completeness;
  }
  if (std::fabs(candidate.rmse_mm - current.rmse_mm) > 1e-9) {
    return candidate.rmse_mm < current.rmse_mm;
  }
  return candidate.pointCount() > current.pointCount();
}

SectionHistoryUpdate upsertSectionObservation(std::vector<SectionObservation>* sections,
                                              const SectionObservation& observation,
                                              double section_spacing_m) {
  SectionHistoryUpdate update;
  if (!sections) {
    return update;
  }
  if (!validSectionMetrics(observation) || !finiteDouble(section_spacing_m)) {
    return update;
  }

  if (section_spacing_m <= 0.0 || sections->empty()) {
    sections->push_back(observation);
    update.inserted = true;
    update.index = sections->size() - 1;
    return update;
  }

  std::size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < sections->size(); ++i) {
    if (!finiteDouble((*sections)[i].chainage_m)) {
      continue;
    }
    const double distance = std::fabs((*sections)[i].chainage_m - observation.chainage_m);
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }

  if (nearest_distance < section_spacing_m) {
    update.index = nearest_index;
    if (isBetterSection(observation, (*sections)[nearest_index])) {
      (*sections)[nearest_index] = observation;
      update.replaced = true;
    }
    return update;
  }

  sections->push_back(observation);
  update.inserted = true;
  update.index = sections->size() - 1;
  return update;
}

SectionExportResult exportSectionsCsv(const std::vector<SectionObservation>& sections,
                                      double min_chainage_m,
                                      double max_chainage_m,
                                      const std::string& min_quality) {
  SectionExportResult result;
  std::ostringstream csv;
  csv.setf(std::ios::fixed);
  csv.precision(3);
  csv << "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n";

  const int minimum_rank = qualityRank(min_quality);
  if (!finiteDouble(min_chainage_m) || !finiteDouble(max_chainage_m) ||
      min_chainage_m > max_chainage_m || minimum_rank == 0) {
    result.csv = csv.str();
    return result;
  }
  for (std::vector<SectionObservation>::const_iterator it = sections.begin(); it != sections.end(); ++it) {
    if (!validExportSection(*it)) {
      continue;
    }
    if (it->chainage_m < min_chainage_m || it->chainage_m > max_chainage_m) {
      continue;
    }
    if (qualityRank(it->quality) < minimum_rank) {
      continue;
    }
    csv << it->session_id << "," << it->chainage_m << "," << it->state_source << ","
        << it->quality << "," << it->completeness << "," << it->rmse_mm << ","
        << it->pointCount() << "\n";
    ++result.exported_count;
  }
  result.csv = csv.str();
  return result;
}

}  // namespace section_manager
