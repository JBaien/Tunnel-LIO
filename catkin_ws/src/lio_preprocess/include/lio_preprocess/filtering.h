#pragma once

#include <string>
#include <vector>

namespace lio_preprocess {

using PointTuple = std::vector<double>;

struct CropBox {
  double x_min;
  double x_max;
  double y_min;
  double y_max;
  double z_min;
  double z_max;

  bool contains(double x, double y, double z) const;
};

struct PointFilterConfig {
  double min_range = 0.4;
  double max_range = 120.0;
  bool enable_body_crop = true;
  CropBox body_crop{-1.8, 1.8, -1.2, 1.2, -0.8, 1.4};
};

struct FilterStats {
  int input_points = 0;
  int output_points = 0;
  int dropped_nan = 0;
  int dropped_range = 0;
  int dropped_body = 0;
};

struct KeepDecision {
  bool keep = false;
  std::string reason;
};

KeepDecision shouldKeepXYZ(double x, double y, double z, const PointFilterConfig& config);
std::vector<PointTuple> filterPointTuples(const std::vector<PointTuple>& points, const PointFilterConfig& config, FilterStats* stats);

}  // namespace lio_preprocess
