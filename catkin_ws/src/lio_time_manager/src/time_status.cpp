#include "lio_time_manager/time_status.h"

#include <cctype>
#include <cstdlib>
#include <cmath>
#include <limits>

namespace lio_time_manager {

SensorTimeTracker::SensorTimeTracker(const std::string& name, double stale_after_sec, std::size_t max_window_size)
    : name_(name), stale_after_sec_(stale_after_sec), max_window_size_(max_window_size) {}

void SensorTimeTracker::observe(double sensor_stamp, double receipt_time) {
  if (!std::isfinite(sensor_stamp) || !std::isfinite(receipt_time)) {
    return;
  }
  if (has_last_sensor_stamp_ && sensor_stamp < last_sensor_stamp_) {
    ++regression_count_;
    time_went_backwards_ = true;
  }
  has_last_sensor_stamp_ = true;
  last_sensor_stamp_ = sensor_stamp;
  last_latency_ms_ = (receipt_time - sensor_stamp) * 1000.0;
  samples_.push_back(std::make_pair(sensor_stamp, receipt_time));
  while (samples_.size() > max_window_size_) {
    samples_.pop_front();
  }
}

SensorTimeStatus SensorTimeTracker::status(double now) const {
  SensorTimeStatus result;
  result.name = name_;
  result.last_latency_ms = last_latency_ms_;
  result.time_went_backwards = time_went_backwards_;
  result.regression_count = regression_count_;
  result.sample_count = static_cast<int>(samples_.size());

  if (samples_.size() >= 2) {
    const double duration = samples_.back().first - samples_.front().first;
    if (duration > 0.0) {
      result.frequency_hz = static_cast<double>(samples_.size() - 1) / duration;
    }
  }
  if (!samples_.empty()) {
    result.stale = !std::isfinite(now) ||
                   !std::isfinite(stale_after_sec_) ||
                   stale_after_sec_ < 0.0 ||
                   now - samples_.back().second > stale_after_sec_;
  }
  return result;
}

ClockOffsetEstimator::ClockOffsetEstimator(const std::string& name,
                                           const std::size_t max_window_size,
                                           const std::size_t min_valid_samples)
    : name_(name), max_window_size_(max_window_size), min_valid_samples_(min_valid_samples) {}

void ClockOffsetEstimator::observe(const double device_time, const double host_time) {
  if (!std::isfinite(device_time) || !std::isfinite(host_time)) {
    return;
  }
  if (has_last_device_time_ && device_time < last_device_time_) {
    ++regression_count_;
    device_time_went_backwards_ = true;
  }
  has_last_device_time_ = true;
  last_device_time_ = device_time;
  samples_.push_back(std::make_pair(device_time, host_time));
  while (samples_.size() > max_window_size_) {
    samples_.pop_front();
  }
}

ClockOffsetStatus ClockOffsetEstimator::status() const {
  ClockOffsetStatus result;
  result.name = name_;
  result.device_time_went_backwards = device_time_went_backwards_;
  result.regression_count = regression_count_;
  result.sample_count = static_cast<int>(samples_.size());
  if (samples_.empty()) {
    return result;
  }

  result.latest_device_time = samples_.back().first;
  result.latest_host_time = samples_.back().second;
  double offset_sum = 0.0;
  for (std::deque<std::pair<double, double> >::const_iterator it = samples_.begin(); it != samples_.end(); ++it) {
    offset_sum += it->second - it->first;
  }
  const double mean_offset_sec = offset_sum / static_cast<double>(samples_.size());
  result.mean_offset_ms = mean_offset_sec * 1000.0;

  double variance = 0.0;
  for (std::deque<std::pair<double, double> >::const_iterator it = samples_.begin(); it != samples_.end(); ++it) {
    const double offset = it->second - it->first;
    const double error = offset - mean_offset_sec;
    variance += error * error;
  }
  result.jitter_ms = std::sqrt(variance / static_cast<double>(samples_.size())) * 1000.0;
  result.valid = samples_.size() >= min_valid_samples_ && !device_time_went_backwards_;
  return result;
}

PpsEventTracker::PpsEventTracker(const std::string& name,
                                 const double stale_after_sec,
                                 const std::size_t max_window_size)
    : name_(name),
      stale_after_sec_(stale_after_sec),
      max_window_size_(max_window_size) {}

void PpsEventTracker::observe(const double event_stamp,
                              const double receipt_time) {
  if (!std::isfinite(event_stamp) || !std::isfinite(receipt_time)) {
    return;
  }
  if (has_last_event_stamp_ && event_stamp < last_event_stamp_) {
    ++regression_count_;
    time_went_backwards_ = true;
  }
  has_last_event_stamp_ = true;
  last_event_stamp_ = event_stamp;
  samples_.push_back(std::make_pair(event_stamp, receipt_time));
  while (samples_.size() > max_window_size_) {
    samples_.pop_front();
  }
}

PpsEventStatus PpsEventTracker::status(const double now) const {
  PpsEventStatus result;
  result.name = name_;
  result.time_went_backwards = time_went_backwards_;
  result.regression_count = regression_count_;
  result.sample_count = static_cast<int>(samples_.size());

  if (!samples_.empty()) {
    result.stale = !std::isfinite(now) ||
                   !std::isfinite(stale_after_sec_) ||
                   stale_after_sec_ < 0.0 ||
                   now - samples_.back().second > stale_after_sec_;
  }
  if (samples_.size() < 2) {
    return result;
  }

  const double duration = samples_.back().first - samples_.front().first;
  if (duration > 0.0) {
    result.frequency_hz = static_cast<double>(samples_.size() - 1) / duration;
  }

  std::vector<double> intervals_sec;
  intervals_sec.reserve(samples_.size() - 1);
  for (std::size_t i = 1; i < samples_.size(); ++i) {
    intervals_sec.push_back(samples_[i].first - samples_[i - 1].first);
  }
  result.latest_interval_ms = intervals_sec.back() * 1000.0;

  double mean_interval_sec = 0.0;
  for (std::vector<double>::const_iterator it = intervals_sec.begin();
       it != intervals_sec.end(); ++it) {
    mean_interval_sec += *it;
  }
  mean_interval_sec /= static_cast<double>(intervals_sec.size());

  double variance = 0.0;
  for (std::vector<double>::const_iterator it = intervals_sec.begin();
       it != intervals_sec.end(); ++it) {
    const double error = *it - mean_interval_sec;
    variance += error * error;
  }
  result.interval_jitter_ms =
      std::sqrt(variance / static_cast<double>(intervals_sec.size())) * 1000.0;
  return result;
}

DiagnosticFeedTracker::DiagnosticFeedTracker(const std::string& name,
                                             const double stale_after_sec)
    : name_(name), stale_after_sec_(stale_after_sec) {
  latest_.name = name_;
}

void DiagnosticFeedTracker::observe(const double receipt_time,
                                    const DiagnosticLevel level,
                                    const std::string& message,
                                    const std::vector<DiagnosticKeyValue>& values) {
  if (!std::isfinite(receipt_time)) {
    return;
  }
  latest_.name = name_;
  latest_.received = true;
  latest_.stale = false;
  latest_.level = level;
  latest_.message = message;
  latest_.last_observed_time = receipt_time;

  for (std::vector<DiagnosticKeyValue>::const_iterator it = values.begin();
       it != values.end(); ++it) {
    double double_value = 0.0;
    int int_value = 0;
    if (it->key == "publish_rate_hz" && parseDouble(it->value, &double_value)) {
      latest_.publish_rate_hz = double_value;
    } else if (it->key == "last_read_latency_ms" &&
               parseDouble(it->value, &double_value)) {
      latest_.last_read_latency_ms = double_value;
    } else if (it->key == "mean_read_latency_ms" &&
               parseDouble(it->value, &double_value)) {
      latest_.mean_read_latency_ms = double_value;
    } else if (it->key == "max_read_latency_ms" &&
               parseDouble(it->value, &double_value)) {
      latest_.max_read_latency_ms = double_value;
    } else if (it->key == "read_error_count" && parseInt(it->value, &int_value)) {
      latest_.read_error_count = int_value;
    } else if (it->key == "invalid_frame_count" && parseInt(it->value, &int_value)) {
      latest_.invalid_frame_count = int_value;
    } else if (it->key == "saturation_count" && parseInt(it->value, &int_value)) {
      latest_.saturation_count = int_value;
    } else if (it->key == "temperature_sample_count" && parseInt(it->value, &int_value)) {
      latest_.temperature_sample_count = int_value;
    } else if (it->key == "temperature_warning_count" && parseInt(it->value, &int_value)) {
      latest_.temperature_warning_count = int_value;
    } else if (it->key == "latest_temperature_c" &&
               parseDouble(it->value, &double_value)) {
      latest_.latest_temperature_c = double_value;
    } else if (it->key == "min_temperature_c" &&
               parseDouble(it->value, &double_value)) {
      latest_.min_temperature_c = double_value;
    } else if (it->key == "max_temperature_c" &&
               parseDouble(it->value, &double_value)) {
      latest_.max_temperature_c = double_value;
    } else if (it->key == "orientation_covariance_x" &&
               parseDouble(it->value, &double_value)) {
      latest_.orientation_covariance_x = double_value;
    } else if (it->key == "orientation_covariance_y" &&
               parseDouble(it->value, &double_value)) {
      latest_.orientation_covariance_y = double_value;
    } else if (it->key == "orientation_covariance_z" &&
               parseDouble(it->value, &double_value)) {
      latest_.orientation_covariance_z = double_value;
    } else if (it->key == "angular_velocity_covariance_x" &&
               parseDouble(it->value, &double_value)) {
      latest_.angular_velocity_covariance_x = double_value;
    } else if (it->key == "angular_velocity_covariance_y" &&
               parseDouble(it->value, &double_value)) {
      latest_.angular_velocity_covariance_y = double_value;
    } else if (it->key == "angular_velocity_covariance_z" &&
               parseDouble(it->value, &double_value)) {
      latest_.angular_velocity_covariance_z = double_value;
    } else if (it->key == "linear_acceleration_covariance_x" &&
               parseDouble(it->value, &double_value)) {
      latest_.linear_acceleration_covariance_x = double_value;
    } else if (it->key == "linear_acceleration_covariance_y" &&
               parseDouble(it->value, &double_value)) {
      latest_.linear_acceleration_covariance_y = double_value;
    } else if (it->key == "linear_acceleration_covariance_z" &&
               parseDouble(it->value, &double_value)) {
      latest_.linear_acceleration_covariance_z = double_value;
    } else if (it->key == "reconnect_attempt_count" && parseInt(it->value, &int_value)) {
      latest_.reconnect_attempt_count = int_value;
    } else if (it->key == "reconnect_success_count" && parseInt(it->value, &int_value)) {
      latest_.reconnect_success_count = int_value;
    } else if (it->key == "timestamp_source") {
      latest_.timestamp_source = it->value;
    } else if (it->key == "hardware_time_status") {
      latest_.hardware_time_status = it->value;
    } else if (it->key == "pps_status") {
      latest_.pps_status = it->value;
    }
  }
}

DiagnosticFeedStatus DiagnosticFeedTracker::status(const double now) const {
  DiagnosticFeedStatus result = latest_;
  result.name = name_;
  result.stale = !result.received ||
                 !std::isfinite(now) ||
                 !std::isfinite(result.last_observed_time) ||
                 !std::isfinite(stale_after_sec_) ||
                 stale_after_sec_ < 0.0 ||
                 now - result.last_observed_time > stale_after_sec_;
  if (result.stale && result.received && result.message == "ok") {
    result.message = "diagnostics stale";
  }
  return result;
}

bool DiagnosticFeedTracker::parseDouble(const std::string& value, double* output) {
  if (value.empty() ||
      std::isspace(static_cast<unsigned char>(value[0])) != 0) {
    return false;
  }
  char* end = NULL;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  *output = parsed;
  return true;
}

bool DiagnosticFeedTracker::parseInt(const std::string& value, int* output) {
  if (value.empty() ||
      std::isspace(static_cast<unsigned char>(value[0])) != 0) {
    return false;
  }
  char* end = NULL;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0' ||
      parsed < std::numeric_limits<int>::min() ||
      parsed > std::numeric_limits<int>::max()) {
    return false;
  }
  *output = static_cast<int>(parsed);
  return true;
}

}  // namespace lio_time_manager
