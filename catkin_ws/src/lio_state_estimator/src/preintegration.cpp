#include "lio_state_estimator/preintegration.h"

#include <cmath>

namespace lio_state_estimator {

ImuVector add(const ImuVector& lhs, const ImuVector& rhs) {
  return ImuVector{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

ImuVector scale(const ImuVector& value, const double factor) {
  return ImuVector{value.x * factor, value.y * factor, value.z * factor};
}

namespace {

constexpr double kDefaultGravityMps2 = 9.80665;
constexpr double kDefaultMaxDtSec = 0.2;
constexpr double kDefaultMaxGapFactor = 5.0;

ImuVector subtract(const ImuVector& lhs, const ImuVector& rhs) {
  return ImuVector{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

bool finiteVector(const ImuVector& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double norm(const ImuVector& value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

}  // namespace

ImuPreintegrator::ImuPreintegrator(const double gravity_mps2,
                                   const double max_dt_sec,
                                   const double max_gap_factor)
    : gravity_mps2_(std::isfinite(gravity_mps2) && gravity_mps2 >= 0.0 ? gravity_mps2 : kDefaultGravityMps2),
      max_dt_sec_(std::isfinite(max_dt_sec) && max_dt_sec > 0.0 ? max_dt_sec : kDefaultMaxDtSec),
      max_gap_factor_(std::isfinite(max_gap_factor) && max_gap_factor > 0.0 ? max_gap_factor : kDefaultMaxGapFactor) {}

ImuPredictionState ImuPreintegrator::observe(const double stamp,
                                             const ImuVector& acceleration,
                                             const ImuVector& angular_velocity) {
  if (!std::isfinite(stamp) || !finiteVector(acceleration) || !finiteVector(angular_velocity)) {
    ++state_.rejected_updates;
    return state_;
  }

  const ImuVector corrected_acc = correctAcceleration(acceleration);
  const ImuVector corrected_gyro = correctGyro(angular_velocity);
  if (!finiteVector(corrected_acc) || !finiteVector(corrected_gyro)) {
    ++state_.rejected_updates;
    return state_;
  }

  if (!has_last_) {
    setLast(stamp, corrected_acc, corrected_gyro);
    return state_;
  }

  const double dt = stamp - state_.last_stamp;
  if (dt < 0.0) {
    const int next_reset_count = state_.reset_count + 1;
    state_ = ImuPredictionState{};
    state_.reset_count = next_reset_count;
    setLast(stamp, corrected_acc, corrected_gyro);
    return state_;
  }

  if (dt > max_dt_sec_ * max_gap_factor_) {
    ++state_.rejected_updates;
    setLast(stamp, corrected_acc, corrected_gyro);
    return state_;
  }

  const ImuVector avg_acc = scale(add(last_acc_, corrected_acc), 0.5);
  state_.position = add(add(state_.position, scale(state_.velocity, dt)),
                        scale(avg_acc, 0.5 * dt * dt));
  state_.velocity = add(state_.velocity, scale(avg_acc, dt));
  setLast(stamp, corrected_acc, corrected_gyro);
  return state_;
}

void ImuPreintegrator::setGyroBias(const ImuVector& bias) {
  if (!finiteVector(bias)) {
    return;
  }
  gyro_bias_ = bias;
  has_gyro_bias_ = true;
  state_.gyro_bias = gyro_bias_;
}

void ImuPreintegrator::clearGyroBias() {
  gyro_bias_ = ImuVector{};
  has_gyro_bias_ = false;
  state_.gyro_bias = ImuVector{};
}

void ImuPreintegrator::setGravityDirection(const ImuVector& direction) {
  if (!finiteVector(direction)) {
    return;
  }
  const double length = norm(direction);
  if (!std::isfinite(length) || length <= 1e-9) {
    return;
  }
  gravity_direction_ = scale(direction, 1.0 / length);
  state_.gravity_direction = gravity_direction_;
}

void ImuPreintegrator::clearGravityDirection() {
  gravity_direction_ = ImuVector{0.0, 0.0, 1.0};
  state_.gravity_direction = gravity_direction_;
}

ImuVector ImuPreintegrator::correctAcceleration(const ImuVector& acceleration) const {
  return subtract(acceleration, scale(gravity_direction_, gravity_mps2_));
}

ImuVector ImuPreintegrator::correctGyro(const ImuVector& angular_velocity) const {
  return has_gyro_bias_ ? subtract(angular_velocity, gyro_bias_) : angular_velocity;
}

void ImuPreintegrator::setLast(const double stamp,
                               const ImuVector& acceleration,
                               const ImuVector& angular_velocity) {
  state_.last_stamp = stamp;
  state_.angular_velocity = angular_velocity;
  state_.gyro_bias = has_gyro_bias_ ? gyro_bias_ : ImuVector{};
  state_.gravity_direction = gravity_direction_;
  last_acc_ = acceleration;
  last_gyro_ = angular_velocity;
  has_last_ = true;
}

}  // namespace lio_state_estimator
