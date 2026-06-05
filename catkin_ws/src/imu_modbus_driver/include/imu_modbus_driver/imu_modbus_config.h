#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace imu_modbus_driver {

struct ImuModbusConfig {
  std::string ip_address = "192.168.188.105";
  int port = 502;
  int start_register = 30;
  int num_registers = 56;
  int max_reconnect_attempts = 10;
  int reconnect_delay_ms = 1000;
  std::string output_topic = "imu_raw";
  std::string frame_id = "imu_link";
  double poll_rate_hz = 400.0;
  double stats_period_sec = 5.0;
  double warn_min_publish_rate_ratio = 0.8;
  double warn_read_latency_ms = 50.0;
  double warn_acc_saturation_mps2 = 150.0;
  double warn_gyro_saturation_radps = 1800.0 * M_PI / 180.0;
  double orientation_covariance_x = 0.001;
  double orientation_covariance_y = 0.001;
  double orientation_covariance_z = 0.001;
  double angular_velocity_covariance_x =
      1.2968807717194922e-05 * 1.2968807717194922e-05;
  double angular_velocity_covariance_y =
      6.0873535629375565e-06 * 6.0873535629375565e-06;
  double angular_velocity_covariance_z =
      1.5389301887171400e-05 * 1.5389301887171400e-05;
  double linear_acceleration_covariance_x =
      3.0335331998782933e-03 * 3.0335331998782933e-03;
  double linear_acceleration_covariance_y =
      2.7319572898178790e-03 * 2.7319572898178790e-03;
  double linear_acceleration_covariance_z =
      3.2680420545776476e-03 * 3.2680420545776476e-03;
  bool enable_temperature_diagnostics = false;
  int temperature_register = -1;
  std::string temperature_register_type = "float32";
  double temperature_scale = 1.0;
  double temperature_offset_c = 0.0;
  double warn_temperature_abs_c = 85.0;
  double warn_temperature_delta_c = 15.0;
  std::string diagnostics_topic = "imu_diagnostics";
  std::string timestamp_source = "host_now";
  std::string hardware_time_status = "host_time_only";
  std::string pps_status = "unconfigured";
};

enum class ImuDiagnosticLevel {
  OK = 0,
  WARN = 1,
  ERROR = 2,
};

struct ImuRuntimeStatsSnapshot {
  int publish_count = 0;
  int read_error_count = 0;
  int invalid_frame_count = 0;
  int saturation_count = 0;
  int reconnect_attempt_count = 0;
  int reconnect_success_count = 0;
  int read_count = 0;
  double window_duration_sec = 0.0;
  double publish_rate_hz = 0.0;
  double mean_read_latency_ms = 0.0;
  double max_read_latency_ms = 0.0;
  double last_read_latency_ms = 0.0;
  int temperature_sample_count = 0;
  int temperature_warning_count = 0;
  double latest_temperature_c = 0.0;
  double min_temperature_c = 0.0;
  double max_temperature_c = 0.0;

  ImuDiagnosticLevel diagnosticLevel(bool connected,
                                     double expected_rate_hz,
                                     double min_rate_ratio,
                                     double max_latency_ms) const {
    if (!connected) {
      return ImuDiagnosticLevel::ERROR;
    }
    const double min_rate_hz = expected_rate_hz * min_rate_ratio;
    if (read_error_count > 0 || invalid_frame_count > 0 || saturation_count > 0 ||
        temperature_warning_count > 0 ||
        reconnect_attempt_count > 0 || publish_rate_hz < min_rate_hz ||
        max_read_latency_ms > max_latency_ms) {
      return ImuDiagnosticLevel::WARN;
    }
    return ImuDiagnosticLevel::OK;
  }
};

class ImuRuntimeStats {
 public:
  void reset(double window_start_sec) {
    window_start_sec_ = window_start_sec;
    publish_count_ = 0;
    read_count_ = 0;
    read_error_count_ = 0;
    invalid_frame_count_ = 0;
    saturation_count_ = 0;
    reconnect_attempt_count_ = 0;
    reconnect_success_count_ = 0;
    latency_sum_ms_ = 0.0;
    max_read_latency_ms_ = 0.0;
    last_read_latency_ms_ = 0.0;
    temperature_sample_count_ = 0;
    temperature_warning_count_ = 0;
    latest_temperature_c_ = 0.0;
    min_temperature_c_ = std::numeric_limits<double>::infinity();
    max_temperature_c_ = -std::numeric_limits<double>::infinity();
  }

  void observeRead(double start_sec, double end_sec, bool success) {
    ++read_count_;
    const double latency_ms = std::max(0.0, (end_sec - start_sec) * 1000.0);
    last_read_latency_ms_ = latency_ms;
    latency_sum_ms_ += latency_ms;
    max_read_latency_ms_ = std::max(max_read_latency_ms_, latency_ms);
    if (!success) {
      ++read_error_count_;
    }
  }

  void observePublish(double stamp_sec) {
    if (window_start_sec_ <= 0.0) {
      window_start_sec_ = stamp_sec;
    }
    ++publish_count_;
  }

  void observeInvalidFrame() {
    ++invalid_frame_count_;
  }

  void observeSaturation() {
    ++saturation_count_;
  }

  void observeReconnectAttempt() {
    ++reconnect_attempt_count_;
  }

  void observeReconnectSuccess() {
    ++reconnect_success_count_;
  }

  void observeTemperature(double temperature_c,
                          double warn_abs_c,
                          double warn_delta_c) {
    if (!std::isfinite(temperature_c)) {
      return;
    }
    ++temperature_sample_count_;
    latest_temperature_c_ = temperature_c;
    min_temperature_c_ = std::min(min_temperature_c_, temperature_c);
    max_temperature_c_ = std::max(max_temperature_c_, temperature_c);
    const bool absolute_warn =
        warn_abs_c > 0.0 && std::abs(temperature_c) >= warn_abs_c;
    const bool delta_warn =
        warn_delta_c > 0.0 && max_temperature_c_ - min_temperature_c_ >= warn_delta_c;
    if (absolute_warn || delta_warn) {
      ++temperature_warning_count_;
    }
  }

  bool due(double now_sec, double period_sec) const {
    return period_sec > 0.0 && window_start_sec_ > 0.0 &&
           now_sec - window_start_sec_ >= period_sec;
  }

  ImuRuntimeStatsSnapshot snapshot(double now_sec) const {
    ImuRuntimeStatsSnapshot result;
    result.publish_count = publish_count_;
    result.read_error_count = read_error_count_;
    result.invalid_frame_count = invalid_frame_count_;
    result.saturation_count = saturation_count_;
    result.reconnect_attempt_count = reconnect_attempt_count_;
    result.reconnect_success_count = reconnect_success_count_;
    result.read_count = read_count_;
    result.window_duration_sec = std::max(0.0, now_sec - window_start_sec_);
    if (result.window_duration_sec > 1e-9) {
      result.publish_rate_hz = publish_count_ / result.window_duration_sec;
    }
    if (read_count_ > 0) {
      result.mean_read_latency_ms = latency_sum_ms_ / read_count_;
    }
    result.max_read_latency_ms = max_read_latency_ms_;
    result.last_read_latency_ms = last_read_latency_ms_;
    result.temperature_sample_count = temperature_sample_count_;
    result.temperature_warning_count = temperature_warning_count_;
    if (temperature_sample_count_ > 0) {
      result.latest_temperature_c = latest_temperature_c_;
      result.min_temperature_c = min_temperature_c_;
      result.max_temperature_c = max_temperature_c_;
    }
    return result;
  }

 private:
  double window_start_sec_ = 0.0;
  int publish_count_ = 0;
  int read_count_ = 0;
  int read_error_count_ = 0;
  int invalid_frame_count_ = 0;
  int saturation_count_ = 0;
  int reconnect_attempt_count_ = 0;
  int reconnect_success_count_ = 0;
  double latency_sum_ms_ = 0.0;
  double max_read_latency_ms_ = 0.0;
  double last_read_latency_ms_ = 0.0;
  int temperature_sample_count_ = 0;
  int temperature_warning_count_ = 0;
  double latest_temperature_c_ = 0.0;
  double min_temperature_c_ = std::numeric_limits<double>::infinity();
  double max_temperature_c_ = -std::numeric_limits<double>::infinity();
};

inline float registersToIEEEFloat(uint16_t low, uint16_t high) {
  const uint32_t combined = (static_cast<uint32_t>(high) << 16) | low;
  float result = 0.0f;
  std::memcpy(&result, &combined, sizeof(result));
  return result;
}

inline int32_t registersToInt32(uint16_t low, uint16_t high) {
  const uint32_t combined = (static_cast<uint32_t>(high) << 16) | low;
  return static_cast<int32_t>(combined);
}

inline bool decodeTemperatureRegisters(const std::vector<uint16_t>& registers,
                                       int register_offset,
                                       const std::string& register_type,
                                       double scale,
                                       double offset_c,
                                       double* temperature_c) {
  if (!temperature_c || register_offset < 0 ||
      register_offset >= static_cast<int>(registers.size()) ||
      !std::isfinite(scale) || !std::isfinite(offset_c)) {
    return false;
  }

  double raw = 0.0;
  if (register_type == "float32") {
    if (register_offset + 1 >= static_cast<int>(registers.size())) {
      return false;
    }
    raw = registersToIEEEFloat(registers[register_offset], registers[register_offset + 1]);
  } else if (register_type == "int32") {
    if (register_offset + 1 >= static_cast<int>(registers.size())) {
      return false;
    }
    raw = registersToInt32(registers[register_offset], registers[register_offset + 1]);
  } else if (register_type == "int16") {
    raw = static_cast<int16_t>(registers[register_offset]);
  } else if (register_type == "uint16") {
    raw = registers[register_offset];
  } else {
    return false;
  }

  const double converted = raw * scale + offset_c;
  if (!std::isfinite(converted)) {
    return false;
  }
  *temperature_c = converted;
  return true;
}

inline bool isDataValid(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
  if (!std::isfinite(acc(0)) || !std::isfinite(acc(1)) || !std::isfinite(acc(2)) ||
      !std::isfinite(gyro(0)) || !std::isfinite(gyro(1)) || !std::isfinite(gyro(2))) {
    return false;
  }

  const double acc_sq_norm = acc.squaredNorm();
  if (acc_sq_norm < 0.9604 || acc_sq_norm > 240100.0) {
    return false;
  }

  constexpr double max_gyro = 2000.0 * M_PI / 180.0;
  return gyro.squaredNorm() <= max_gyro * max_gyro;
}

inline bool isDataNearSaturation(const Eigen::Vector3d& acc,
                                 const Eigen::Vector3d& gyro,
                                 double acc_threshold_mps2,
                                 double gyro_threshold_radps) {
  if (acc_threshold_mps2 <= 0.0 && gyro_threshold_radps <= 0.0) {
    return false;
  }
  const double max_abs_acc = acc.cwiseAbs().maxCoeff();
  const double max_abs_gyro = gyro.cwiseAbs().maxCoeff();
  return (acc_threshold_mps2 > 0.0 && max_abs_acc >= acc_threshold_mps2) ||
         (gyro_threshold_radps > 0.0 && max_abs_gyro >= gyro_threshold_radps);
}

inline Eigen::Matrix<double, 9, 1> makeDiagonalCovariance(double x,
                                                          double y,
                                                          double z) {
  Eigen::Matrix<double, 9, 1> covariance;
  covariance.setZero();
  covariance(0) = x;
  covariance(4) = y;
  covariance(8) = z;
  return covariance;
}

}  // namespace imu_modbus_driver
