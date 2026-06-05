#include "lio_preprocess/deskewing.h"

#include <algorithm>
#include <cmath>
#include <set>

namespace lio_preprocess {

namespace {

std::string lowerCopy(const std::string& value) {
  std::string result = value;
  std::transform(result.begin(), result.end(), result.begin(), ::tolower);
  return result;
}

bool validDeskewConfig(const DeskewConfig& config) {
  if (config.reference != "start" && config.reference != "end") {
    return false;
  }
  if (!std::isfinite(config.point_time_scale) ||
      !std::isfinite(config.max_abs_point_time) ||
      !std::isfinite(config.max_abs_acceleration) ||
      config.max_abs_point_time < 0.0 ||
      config.max_abs_acceleration < 0.0 ||
      config.point_time_fields.empty()) {
    return false;
  }
  for (std::vector<std::string>::const_iterator it = config.point_time_fields.begin();
       it != config.point_time_fields.end(); ++it) {
    if (it->empty()) {
      return false;
    }
  }
  return true;
}

bool validImuSample(const ImuAngularSample& sample, const DeskewConfig& config, DeskewStats* stats) {
  const bool angular_ok =
      std::isfinite(sample.stamp) &&
      std::isfinite(sample.wx) &&
      std::isfinite(sample.wy) &&
      std::isfinite(sample.wz);
  if (!angular_ok) {
    return false;
  }
  if (!config.enable_translation_compensation) {
    return true;
  }
  const bool linear_ok =
      std::isfinite(sample.ax) &&
      std::isfinite(sample.ay) &&
      std::isfinite(sample.az) &&
      std::fabs(sample.ax) <= config.max_abs_acceleration &&
      std::fabs(sample.ay) <= config.max_abs_acceleration &&
      std::fabs(sample.az) <= config.max_abs_acceleration;
  if (!linear_ok && stats != nullptr) {
    ++stats->invalid_linear_acceleration;
  }
  return linear_ok;
}

bool validImuAngularSample(const ImuAngularSample& sample) {
  return std::isfinite(sample.stamp) &&
         std::isfinite(sample.wx) &&
         std::isfinite(sample.wy) &&
         std::isfinite(sample.wz);
}

void meanAngularVelocity(
    const std::vector<ImuAngularSample>& imu_samples,
    double cloud_stamp,
    const std::vector<double>& valid_relative_times,
    double* wx,
    double* wy,
    double* wz) {
  const double min_time = *std::min_element(valid_relative_times.begin(), valid_relative_times.end());
  const double max_time = *std::max_element(valid_relative_times.begin(), valid_relative_times.end());
  const double start = cloud_stamp + min_time;
  const double end = cloud_stamp + max_time;
  std::vector<ImuAngularSample> selected;
  for (std::vector<ImuAngularSample>::const_iterator it = imu_samples.begin(); it != imu_samples.end(); ++it) {
    if (start - 0.02 <= it->stamp && it->stamp <= end + 0.02) {
      selected.push_back(*it);
    }
  }
  if (selected.empty()) {
    selected = imu_samples;
  }
  *wx = 0.0;
  *wy = 0.0;
  *wz = 0.0;
  for (std::vector<ImuAngularSample>::const_iterator it = selected.begin(); it != selected.end(); ++it) {
    *wx += it->wx;
    *wy += it->wy;
    *wz += it->wz;
  }
  const double count = static_cast<double>(selected.size());
  *wx /= count;
  *wy /= count;
  *wz /= count;
}

void meanLinearAcceleration(
    const std::vector<ImuAngularSample>& imu_samples,
    double cloud_stamp,
    const std::vector<double>& valid_relative_times,
    double* ax,
    double* ay,
    double* az) {
  const double min_time = *std::min_element(valid_relative_times.begin(), valid_relative_times.end());
  const double max_time = *std::max_element(valid_relative_times.begin(), valid_relative_times.end());
  const double start = cloud_stamp + min_time;
  const double end = cloud_stamp + max_time;
  std::vector<ImuAngularSample> selected;
  for (std::vector<ImuAngularSample>::const_iterator it = imu_samples.begin(); it != imu_samples.end(); ++it) {
    if (start - 0.02 <= it->stamp && it->stamp <= end + 0.02) {
      selected.push_back(*it);
    }
  }
  if (selected.empty()) {
    selected = imu_samples;
  }
  *ax = 0.0;
  *ay = 0.0;
  *az = 0.0;
  for (std::vector<ImuAngularSample>::const_iterator it = selected.begin(); it != selected.end(); ++it) {
    *ax += it->ax;
    *ay += it->ay;
    *az += it->az;
  }
  const double count = static_cast<double>(selected.size());
  *ax /= count;
  *ay /= count;
  *az /= count;
}

}  // namespace

std::vector<PointTuple> deskewPointTuples(
    const std::vector<PointTuple>& points,
    const std::vector<std::string>& field_names,
    double cloud_stamp,
    const std::vector<ImuAngularSample>& imu_samples,
    const DeskewConfig& config,
    DeskewStats* stats) {
  DeskewStats local;
  local.input_points = static_cast<int>(points.size());
  if (!config.enabled || points.empty()) {
    if (stats != nullptr) {
      *stats = local;
    }
    return points;
  }
  if (!std::isfinite(cloud_stamp) || !validDeskewConfig(config)) {
    local.invalid_config = static_cast<int>(points.size());
    if (stats != nullptr) {
      *stats = local;
    }
    return points;
  }

  const int time_index = findTimeField(field_names, config.point_time_fields);
  if (time_index < 0) {
    local.missing_time_field = static_cast<int>(points.size());
    if (stats != nullptr) {
      *stats = local;
    }
    return points;
  }
  std::vector<ImuAngularSample> valid_imu_samples;
  valid_imu_samples.reserve(imu_samples.size());
  for (std::vector<ImuAngularSample>::const_iterator it = imu_samples.begin();
       it != imu_samples.end(); ++it) {
    if (validImuSample(*it, config, &local)) {
      valid_imu_samples.push_back(*it);
    } else {
      ++local.invalid_imu;
    }
  }
  if (valid_imu_samples.empty()) {
    local.missing_imu = static_cast<int>(points.size());
    if (stats != nullptr) {
      *stats = local;
    }
    return points;
  }

  std::vector<double> relative_times(points.size(), 0.0);
  std::vector<bool> valid(points.size(), false);
  std::vector<double> valid_times;
  for (std::size_t i = 0; i < points.size(); ++i) {
    double relative_time = 0.0;
    if (points[i].size() >= 3 &&
        pointRelativeTime(points[i], time_index, config, &relative_time)) {
      relative_times[i] = relative_time;
      valid[i] = true;
      valid_times.push_back(relative_time);
    } else {
      ++local.invalid_point_time;
    }
  }
  if (valid_times.empty()) {
    if (stats != nullptr) {
      *stats = local;
    }
    return points;
  }

  const double reference_time = config.reference == "start" ? 0.0 : *std::max_element(valid_times.begin(), valid_times.end());
  local.reference_time = reference_time;
  double mean_wx = 0.0;
  double mean_wy = 0.0;
  double mean_wz = 0.0;
  meanAngularVelocity(valid_imu_samples, cloud_stamp, valid_times, &mean_wx, &mean_wy, &mean_wz);
  double mean_ax = 0.0;
  double mean_ay = 0.0;
  double mean_az = 0.0;
  if (config.enable_translation_compensation) {
    meanLinearAcceleration(valid_imu_samples, cloud_stamp, valid_times, &mean_ax, &mean_ay, &mean_az);
  }

  std::vector<PointTuple> corrected = points;
  for (std::size_t i = 0; i < corrected.size(); ++i) {
    if (!valid[i]) {
      continue;
    }
    const double dt = reference_time - relative_times[i];
    rotateXYZ(points[i][0], points[i][1], points[i][2], mean_wx * dt, mean_wy * dt, mean_wz * dt, &corrected[i][0], &corrected[i][1], &corrected[i][2]);
    if (config.enable_translation_compensation) {
      const double point_t2 = relative_times[i] * relative_times[i];
      const double reference_t2 = reference_time * reference_time;
      corrected[i][0] += 0.5 * mean_ax * (point_t2 - reference_t2);
      corrected[i][1] += 0.5 * mean_ay * (point_t2 - reference_t2);
      corrected[i][2] += 0.5 * mean_az * (point_t2 - reference_t2);
      ++local.translation_compensated_points;
    }
    ++local.deskewed_points;
  }
  local.used_imu = local.deskewed_points > 0;
  if (stats != nullptr) {
    *stats = local;
  }
  return corrected;
}

int findTimeField(const std::vector<std::string>& field_names, const std::vector<std::string>& candidates) {
  std::set<std::string> candidate_names;
  for (std::vector<std::string>::const_iterator candidate = candidates.begin(); candidate != candidates.end(); ++candidate) {
    if (!candidate->empty()) {
      candidate_names.insert(lowerCopy(*candidate));
    }
  }
  int result = -1;
  std::size_t matches = 0;
  for (std::size_t i = 0; i < field_names.size(); ++i) {
    if (candidate_names.find(lowerCopy(field_names[i])) != candidate_names.end()) {
      result = static_cast<int>(i);
      ++matches;
    }
  }
  return matches == 1 ? result : -1;
}

bool pointRelativeTime(const PointTuple& point, int time_index, const DeskewConfig& config, double* relative_time) {
  if (time_index < 0 || static_cast<std::size_t>(time_index) >= point.size()) {
    return false;
  }
  const double value = point[time_index] * config.point_time_scale;
  if (!std::isfinite(value) || std::fabs(value) > config.max_abs_point_time) {
    return false;
  }
  *relative_time = value;
  return true;
}

void rotateXYZ(double x, double y, double z, double rx, double ry, double rz, double* out_x, double* out_y, double* out_z) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(rx) || !std::isfinite(ry) || !std::isfinite(rz)) {
    *out_x = x;
    *out_y = y;
    *out_z = z;
    return;
  }
  const double angle = std::sqrt(rx * rx + ry * ry + rz * rz);
  if (angle < 1e-12) {
    *out_x = x;
    *out_y = y;
    *out_z = z;
    return;
  }
  const double ux = rx / angle;
  const double uy = ry / angle;
  const double uz = rz / angle;
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  const double one_c = 1.0 - c;
  const double dot = ux * x + uy * y + uz * z;
  const double cross_x = uy * z - uz * y;
  const double cross_y = uz * x - ux * z;
  const double cross_z = ux * y - uy * x;
  *out_x = x * c + cross_x * s + ux * dot * one_c;
  *out_y = y * c + cross_y * s + uy * dot * one_c;
  *out_z = z * c + cross_z * s + uz * dot * one_c;
}

}  // namespace lio_preprocess
