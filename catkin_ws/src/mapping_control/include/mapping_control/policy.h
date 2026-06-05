#pragma once

#include <string>

namespace mapping_control {

extern const char* const ACCEPT;
extern const char* const FREEZE;
extern const char* const LIMIT;
extern const char* const REJECT;

extern const char* const IDLE_STATIC;
extern const char* const CUTTING_STATIC;
extern const char* const FWD_MOVE;
extern const char* const REV_MOVE;
extern const char* const TURNING;
extern const char* const CMD_MOVE_NO_DISP;
extern const char* const CONFLICT;
extern const char* const RELOCALIZING;

struct MappingPolicyConfig {
  double section_spacing_m = 1.0;
  double control_anchor_spacing_m = 50.0;
  double weak_observability_threshold = 0.3;
  double max_weak_delta_m = 0.05;
  double max_nominal_delta_m = 0.5;
};

struct ControlDecision {
  std::string action;
  double accepted_delta_m = 0.0;
  bool stable_map_write = false;
  bool section_sample = false;
  std::string reason;
};

ControlDecision decideMappingControl(
    const std::string& machine_state,
    double requested_delta_m,
    double observability_score,
    double chainage_m,
    double last_section_chainage_m,
    const MappingPolicyConfig& config);

bool needsControlAnchor(double chainage_m, double last_anchor_chainage_m, const MappingPolicyConfig& config);
std::string gradeSection(double completeness, double rmse_mm);

}  // namespace mapping_control
