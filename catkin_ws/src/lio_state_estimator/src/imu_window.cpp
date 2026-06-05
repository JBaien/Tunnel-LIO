#include "lio_state_estimator/imu_window.h"

#include <algorithm>
#include <cmath>

namespace lio_state_estimator {
namespace {

constexpr double kDefaultWindowSec = 5.0;

bool validSample(const ImuSample& sample) {
  return std::isfinite(sample.stamp) &&
         std::isfinite(sample.ax) &&
         std::isfinite(sample.ay) &&
         std::isfinite(sample.az) &&
         std::isfinite(sample.gx) &&
         std::isfinite(sample.gy) &&
         std::isfinite(sample.gz);
}

double norm3(const ImuVector& value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

ImuVector scale3(const ImuVector& value, const double scale) {
  ImuVector result;
  result.x = value.x * scale;
  result.y = value.y * scale;
  result.z = value.z * scale;
  return result;
}

}  // namespace

ImuWindow::ImuWindow(const double window_sec, const double min_frequency_hz)
    : window_sec_(std::isfinite(window_sec) && window_sec > 0.0 ? window_sec : kDefaultWindowSec),
      min_frequency_hz_(std::isfinite(min_frequency_hz) && min_frequency_hz > 0.0 ? min_frequency_hz : 0.0) {}

void ImuWindow::add(const ImuSample& sample) {
  if (!validSample(sample)) {
    ++invalid_sample_count_;
    return;
  }
  if (!samples_.empty() && sample.stamp < samples_.back().stamp) {
    ++regression_count_;
  }

  samples_.push_back(sample);
  const double cutoff = sample.stamp - window_sec_;
  samples_.erase(std::remove_if(samples_.begin(), samples_.end(),
                                [cutoff](const ImuSample& item) { return item.stamp < cutoff; }),
                 samples_.end());
}

ImuWindowStatus ImuWindow::status() const {
  ImuWindowStatus result;
  result.sample_count = static_cast<int>(samples_.size());
  result.regression_count = regression_count_;
  result.invalid_sample_count = invalid_sample_count_;

  if (samples_.empty()) {
    return result;
  }

  const double first = samples_.front().stamp;
  const double last = samples_.back().stamp;
  result.duration_sec = std::max(0.0, last - first);
  if (samples_.size() >= 2 && result.duration_sec > 0.0) {
    result.frequency_hz = static_cast<double>(samples_.size() - 1) / result.duration_sec;
  }

  for (const ImuSample& sample : samples_) {
    result.mean_acc.x += sample.ax;
    result.mean_acc.y += sample.ay;
    result.mean_acc.z += sample.az;
    result.mean_gyro.x += sample.gx;
    result.mean_gyro.y += sample.gy;
    result.mean_gyro.z += sample.gz;
  }

  const double count = static_cast<double>(samples_.size());
  result.mean_acc.x /= count;
  result.mean_acc.y /= count;
  result.mean_acc.z /= count;
  result.mean_gyro.x /= count;
  result.mean_gyro.y /= count;
  result.mean_gyro.z /= count;

  double acc_deviation_sum = 0.0;
  double gyro_norm_sum = 0.0;
  for (const ImuSample& sample : samples_) {
    const double dax = sample.ax - result.mean_acc.x;
    const double day = sample.ay - result.mean_acc.y;
    const double daz = sample.az - result.mean_acc.z;
    acc_deviation_sum += dax * dax + day * day + daz * daz;
    gyro_norm_sum += sample.gx * sample.gx + sample.gy * sample.gy + sample.gz * sample.gz;
  }

  result.acc_rms_deviation = std::sqrt(acc_deviation_sum / count);
  result.gyro_rms = std::sqrt(gyro_norm_sum / count);
  result.health_score =
      min_frequency_hz_ > 0.0 ? std::min(1.0, result.frequency_hz / min_frequency_hz_) : 1.0;
  if (regression_count_ > 0) {
    result.health_score *= 0.5;
  }

  constexpr double kGravity = 9.80665;
  constexpr double kMaxStationaryAccRms = 0.05;
  constexpr double kMaxStationaryGyroRms = 0.05;
  constexpr double kMaxGravityMagnitudeError = 0.25;
  const double acc_norm = norm3(result.mean_acc);
  result.stationary =
      samples_.size() >= 2 &&
      result.acc_rms_deviation <= kMaxStationaryAccRms &&
      result.gyro_rms <= kMaxStationaryGyroRms &&
      std::fabs(acc_norm - kGravity) <= kMaxGravityMagnitudeError &&
      regression_count_ == 0;
  if (result.stationary && acc_norm > 1e-9) {
    result.gyro_bias = result.mean_gyro;
    result.gravity_direction = scale3(result.mean_acc, 1.0 / acc_norm);
  }

  return result;
}

const ImuSample* ImuWindow::lastSample() const {
  if (samples_.empty()) {
    return nullptr;
  }
  return &samples_.back();
}

}  // namespace lio_state_estimator
