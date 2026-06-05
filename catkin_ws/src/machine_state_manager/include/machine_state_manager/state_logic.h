#pragma once

#include <string>

namespace machine_state_manager {

extern const char* const IDLE_STATIC;
extern const char* const CUTTING_STATIC;
extern const char* const FWD_MOVE;
extern const char* const REV_MOVE;
extern const char* const TURNING;
extern const char* const CMD_MOVE_NO_DISP;
extern const char* const CONFLICT;
extern const char* const RELOCALIZING;
extern const char* const UNKNOWN;

struct MachineSignals {
  double left_track_speed = 0.0;
  double right_track_speed = 0.0;
  bool cutting_on = false;
  double lidar_speed = 0.0;
  double imu_vibration = 0.0;
  bool imu_ok = true;
  bool relocalizing = false;
};

struct MachineStateThresholds {
  double static_track_threshold = 0.05;
  double move_track_threshold = 0.1;
  double static_lidar_speed_threshold = 0.03;
  double move_lidar_speed_threshold = 0.08;
  double turn_track_delta_threshold = 0.12;
  double imu_static_vibration_threshold = 0.2;
};

std::string classifyMachineState(const MachineSignals& signals, const MachineStateThresholds& thresholds);
bool allowsStableMapWrite(const std::string& state);
bool freezesPose(const std::string& state);

}  // namespace machine_state_manager
