#pragma once

#include "lio_state_estimator/imu_window.h"

namespace lio_state_estimator {

struct ImuPredictionState {
  ImuVector position;
  ImuVector velocity;
  ImuVector angular_velocity;
  ImuVector gyro_bias;
  ImuVector gravity_direction;
  double last_stamp = 0.0;
  int rejected_updates = 0;
  int reset_count = 0;
};

class ImuPreintegrator {
 public:
  ImuPreintegrator(double gravity_mps2 = 9.80665,
                   double max_dt_sec = 0.2,
                   double max_gap_factor = 5.0);

  ImuPredictionState observe(double stamp,
                             const ImuVector& acceleration,
                             const ImuVector& angular_velocity);
  const ImuPredictionState& state() const { return state_; }
  void setGyroBias(const ImuVector& bias);
  void clearGyroBias();
  void setGravityDirection(const ImuVector& direction);
  void clearGravityDirection();

 private:
  ImuVector correctAcceleration(const ImuVector& acceleration) const;
  ImuVector correctGyro(const ImuVector& angular_velocity) const;
  void setLast(double stamp,
               const ImuVector& acceleration,
               const ImuVector& angular_velocity);

  double gravity_mps2_ = 9.80665;
  double max_dt_sec_ = 0.2;
  double max_gap_factor_ = 5.0;
  ImuPredictionState state_;
  ImuVector last_acc_;
  ImuVector last_gyro_;
  ImuVector gyro_bias_;
  ImuVector gravity_direction_{0.0, 0.0, 1.0};
  bool has_gyro_bias_ = false;
  bool has_last_ = false;
};

ImuVector add(const ImuVector& lhs, const ImuVector& rhs);
ImuVector scale(const ImuVector& value, double factor);

}  // namespace lio_state_estimator
