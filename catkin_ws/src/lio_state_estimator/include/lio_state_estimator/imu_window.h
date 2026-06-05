#pragma once

#include <cstddef>
#include <vector>

namespace lio_state_estimator {

struct ImuVector {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct ImuSample {
  double stamp = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
  double gx = 0.0;
  double gy = 0.0;
  double gz = 0.0;
};

struct ImuWindowStatus {
  int sample_count = 0;
  double duration_sec = 0.0;
  double frequency_hz = 0.0;
  ImuVector mean_acc;
  ImuVector mean_gyro;
  ImuVector gyro_bias;
  ImuVector gravity_direction;
  double acc_rms_deviation = 0.0;
  double gyro_rms = 0.0;
  int regression_count = 0;
  int invalid_sample_count = 0;
  double health_score = 0.0;
  bool stationary = false;
};

class ImuWindow {
 public:
  ImuWindow(double window_sec, double min_frequency_hz);

  void add(const ImuSample& sample);
  ImuWindowStatus status() const;
  const ImuSample* lastSample() const;

  double minFrequencyHz() const { return min_frequency_hz_; }

 private:
  double window_sec_ = 0.0;
  double min_frequency_hz_ = 0.0;
  std::vector<ImuSample> samples_;
  int regression_count_ = 0;
  int invalid_sample_count_ = 0;
};

}  // namespace lio_state_estimator
