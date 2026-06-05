#pragma once

#include <string>
#include <vector>

#include "lio_preprocess/filtering.h"

namespace lio_preprocess {

struct ImuAngularSample {
  double stamp = 0.0;
  double wx = 0.0;
  double wy = 0.0;
  double wz = 0.0;
};

struct DeskewConfig {
  bool enabled = false;
  std::string reference = "start";
  std::vector<std::string> point_time_fields{"time", "timestamp", "t", "offset_time"};
  double point_time_scale = 1.0;
  double max_abs_point_time = 0.2;
};

struct DeskewStats {
  int input_points = 0;
  int deskewed_points = 0;
  int missing_time_field = 0;
  int invalid_point_time = 0;
  int missing_imu = 0;
  int invalid_imu = 0;
  int invalid_config = 0;
  bool used_imu = false;
  double reference_time = 0.0;
};

std::vector<PointTuple> deskewPointTuples(
    const std::vector<PointTuple>& points,
    const std::vector<std::string>& field_names,
    double cloud_stamp,
    const std::vector<ImuAngularSample>& imu_samples,
    const DeskewConfig& config,
    DeskewStats* stats);

int findTimeField(const std::vector<std::string>& field_names, const std::vector<std::string>& candidates);
bool pointRelativeTime(const PointTuple& point, int time_index, const DeskewConfig& config, double* relative_time);
void rotateXYZ(double x, double y, double z, double rx, double ry, double rz, double* out_x, double* out_y, double* out_z);

}  // namespace lio_preprocess
