#pragma once

#include <string>

namespace mine_slam_bringup {

struct EventDumpRequest {
  std::string session_dir;
  std::string output_root;
  std::string event_id;
  double event_time_s = 0.0;
  double window_before_s = 10.0;
  double window_after_s = 10.0;
  std::string reason;
};

struct EventDumpResult {
  bool success = false;
  std::string message;
  std::string event_id;
  std::string event_dir;
};

EventDumpResult createEventDump(const EventDumpRequest& request);

}  // namespace mine_slam_bringup
