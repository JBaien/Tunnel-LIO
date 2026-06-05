#pragma once

#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace lio_time_manager {

struct SensorTimeStatus {
  std::string name;
  double frequency_hz = 0.0;
  double last_latency_ms = 0.0;
  bool stale = true;
  bool time_went_backwards = false;
  int regression_count = 0;
  int sample_count = 0;
};

struct ClockOffsetStatus {
  std::string name;
  double mean_offset_ms = 0.0;
  double jitter_ms = 0.0;
  double latest_device_time = 0.0;
  double latest_host_time = 0.0;
  bool valid = false;
  bool device_time_went_backwards = false;
  int regression_count = 0;
  int sample_count = 0;
};

struct PpsEventStatus {
  std::string name;
  double frequency_hz = 0.0;
  double latest_interval_ms = 0.0;
  double interval_jitter_ms = 0.0;
  bool stale = true;
  bool time_went_backwards = false;
  int regression_count = 0;
  int sample_count = 0;
};

enum class DiagnosticLevel {
  OK = 0,
  WARN = 1,
  ERROR = 2,
};

struct DiagnosticKeyValue {
  std::string key;
  std::string value;
};

struct DiagnosticFeedStatus {
  std::string name;
  bool received = false;
  bool stale = true;
  DiagnosticLevel level = DiagnosticLevel::WARN;
  std::string message = "waiting for diagnostics";
  double last_observed_time = 0.0;
  double publish_rate_hz = 0.0;
  double last_read_latency_ms = 0.0;
  double mean_read_latency_ms = 0.0;
  double max_read_latency_ms = 0.0;
  int read_error_count = 0;
  int invalid_frame_count = 0;
  int saturation_count = 0;
  int temperature_sample_count = 0;
  int temperature_warning_count = 0;
  double latest_temperature_c = 0.0;
  double min_temperature_c = 0.0;
  double max_temperature_c = 0.0;
  double orientation_covariance_x = 0.0;
  double orientation_covariance_y = 0.0;
  double orientation_covariance_z = 0.0;
  double angular_velocity_covariance_x = 0.0;
  double angular_velocity_covariance_y = 0.0;
  double angular_velocity_covariance_z = 0.0;
  double linear_acceleration_covariance_x = 0.0;
  double linear_acceleration_covariance_y = 0.0;
  double linear_acceleration_covariance_z = 0.0;
  int reconnect_attempt_count = 0;
  int reconnect_success_count = 0;
  std::string timestamp_source = "unknown";
  std::string hardware_time_status = "unknown";
  std::string pps_status = "unknown";

  DiagnosticLevel effectiveLevel() const {
    if (!received) {
      return DiagnosticLevel::WARN;
    }
    if (stale && level == DiagnosticLevel::OK) {
      return DiagnosticLevel::WARN;
    }
    return level;
  }
};

class DiagnosticFeedTracker {
 public:
  DiagnosticFeedTracker(const std::string& name, double stale_after_sec);

  void observe(double receipt_time,
               DiagnosticLevel level,
               const std::string& message,
               const std::vector<DiagnosticKeyValue>& values);
  DiagnosticFeedStatus status(double now) const;

 private:
  static bool parseDouble(const std::string& value, double* output);
  static bool parseInt(const std::string& value, int* output);

  std::string name_;
  double stale_after_sec_ = 1.0;
  DiagnosticFeedStatus latest_;
};

class SensorTimeTracker {
 public:
  SensorTimeTracker(const std::string& name, double stale_after_sec, std::size_t max_window_size = 100);

  void observe(double sensor_stamp, double receipt_time);
  SensorTimeStatus status(double now) const;

 private:
  std::string name_;
  double stale_after_sec_ = 1.0;
  std::size_t max_window_size_ = 100;
  std::deque<std::pair<double, double> > samples_;
  bool has_last_sensor_stamp_ = false;
  double last_sensor_stamp_ = 0.0;
  double last_latency_ms_ = 0.0;
  int regression_count_ = 0;
  bool time_went_backwards_ = false;
};

class ClockOffsetEstimator {
 public:
  ClockOffsetEstimator(const std::string& name, std::size_t max_window_size = 100, std::size_t min_valid_samples = 2);

  void observe(double device_time, double host_time);
  ClockOffsetStatus status() const;

 private:
  std::string name_;
  std::size_t max_window_size_ = 100;
  std::size_t min_valid_samples_ = 2;
  std::deque<std::pair<double, double> > samples_;
  bool has_last_device_time_ = false;
  double last_device_time_ = 0.0;
  int regression_count_ = 0;
  bool device_time_went_backwards_ = false;
};

class PpsEventTracker {
 public:
  PpsEventTracker(const std::string& name,
                  double stale_after_sec,
                  std::size_t max_window_size = 100);

  void observe(double event_stamp, double receipt_time);
  PpsEventStatus status(double now) const;

 private:
  std::string name_;
  double stale_after_sec_ = 1.0;
  std::size_t max_window_size_ = 100;
  std::deque<std::pair<double, double> > samples_;
  bool has_last_event_stamp_ = false;
  double last_event_stamp_ = 0.0;
  int regression_count_ = 0;
  bool time_went_backwards_ = false;
};

}  // namespace lio_time_manager
