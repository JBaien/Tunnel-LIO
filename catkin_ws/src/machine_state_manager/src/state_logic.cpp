#include "machine_state_manager/state_logic.h"

#include <cmath>

namespace machine_state_manager {
namespace {

bool finiteSignals(const MachineSignals& signals) {
  return std::isfinite(signals.left_track_speed) &&
         std::isfinite(signals.right_track_speed) &&
         std::isfinite(signals.lidar_speed) &&
         std::isfinite(signals.imu_vibration);
}

bool validThresholds(const MachineStateThresholds& thresholds) {
  return std::isfinite(thresholds.static_track_threshold) &&
         std::isfinite(thresholds.move_track_threshold) &&
         std::isfinite(thresholds.static_lidar_speed_threshold) &&
         std::isfinite(thresholds.move_lidar_speed_threshold) &&
         std::isfinite(thresholds.turn_track_delta_threshold) &&
         std::isfinite(thresholds.imu_static_vibration_threshold) &&
         thresholds.static_track_threshold >= 0.0 &&
         thresholds.move_track_threshold >= thresholds.static_track_threshold &&
         thresholds.static_lidar_speed_threshold >= 0.0 &&
         thresholds.move_lidar_speed_threshold >= thresholds.static_lidar_speed_threshold &&
         thresholds.turn_track_delta_threshold >= 0.0 &&
         thresholds.imu_static_vibration_threshold >= 0.0;
}

}  // namespace

const char* const IDLE_STATIC = "IDLE_STATIC";
const char* const CUTTING_STATIC = "CUTTING_STATIC";
const char* const FWD_MOVE = "FWD_MOVE";
const char* const REV_MOVE = "REV_MOVE";
const char* const TURNING = "TURNING";
const char* const CMD_MOVE_NO_DISP = "CMD_MOVE_NO_DISP";
const char* const CONFLICT = "CONFLICT";
const char* const RELOCALIZING = "RELOCALIZING";
const char* const UNKNOWN = "UNKNOWN";

std::string classifyMachineState(const MachineSignals& signals, const MachineStateThresholds& thresholds) {
  if (!finiteSignals(signals) || !validThresholds(thresholds)) {
    return CONFLICT;
  }
  if (signals.relocalizing) {
    return RELOCALIZING;
  }

  const double left = signals.left_track_speed;
  const double right = signals.right_track_speed;
  const bool track_static =
      std::fabs(left) <= thresholds.static_track_threshold && std::fabs(right) <= thresholds.static_track_threshold;
  const bool lidar_static = std::fabs(signals.lidar_speed) <= thresholds.static_lidar_speed_threshold;
  const bool lidar_moving = std::fabs(signals.lidar_speed) >= thresholds.move_lidar_speed_threshold;
  const bool imu_static = signals.imu_ok && signals.imu_vibration <= thresholds.imu_static_vibration_threshold;

  if (track_static && !lidar_static) {
    return CONFLICT;
  }
  if (track_static && signals.cutting_on && lidar_static) {
    return CUTTING_STATIC;
  }
  if (track_static && !signals.cutting_on && lidar_static && imu_static) {
    return IDLE_STATIC;
  }
  if (std::fabs(left - right) >= thresholds.turn_track_delta_threshold || left * right < 0.0) {
    return lidar_moving ? std::string(TURNING) : std::string(CMD_MOVE_NO_DISP);
  }

  const bool both_forward = left >= thresholds.move_track_threshold && right >= thresholds.move_track_threshold;
  const bool both_reverse = left <= -thresholds.move_track_threshold && right <= -thresholds.move_track_threshold;
  if (both_forward) {
    return lidar_moving ? std::string(FWD_MOVE) : std::string(CMD_MOVE_NO_DISP);
  }
  if (both_reverse) {
    return lidar_moving ? std::string(REV_MOVE) : std::string(CMD_MOVE_NO_DISP);
  }
  return UNKNOWN;
}

bool allowsStableMapWrite(const std::string& state) {
  return state == IDLE_STATIC || state == FWD_MOVE;
}

bool freezesPose(const std::string& state) {
  return state == IDLE_STATIC ||
         state == CUTTING_STATIC ||
         state == CMD_MOVE_NO_DISP ||
         state == CONFLICT ||
         state == RELOCALIZING;
}

}  // namespace machine_state_manager
