#include "mapping_control/policy.h"

#include <cmath>

namespace mapping_control {

const char* const ACCEPT = "ACCEPT";
const char* const FREEZE = "FREEZE";
const char* const LIMIT = "LIMIT";
const char* const REJECT = "REJECT";

const char* const IDLE_STATIC = "IDLE_STATIC";
const char* const CUTTING_STATIC = "CUTTING_STATIC";
const char* const FWD_MOVE = "FWD_MOVE";
const char* const REV_MOVE = "REV_MOVE";
const char* const TURNING = "TURNING";
const char* const CMD_MOVE_NO_DISP = "CMD_MOVE_NO_DISP";
const char* const CONFLICT = "CONFLICT";
const char* const RELOCALIZING = "RELOCALIZING";

namespace {

bool validPolicyConfig(const MappingPolicyConfig& config) {
  return std::isfinite(config.section_spacing_m) &&
         std::isfinite(config.control_anchor_spacing_m) &&
         std::isfinite(config.weak_observability_threshold) &&
         std::isfinite(config.max_weak_delta_m) &&
         std::isfinite(config.max_nominal_delta_m) &&
         config.section_spacing_m > 0.0 &&
         config.control_anchor_spacing_m > 0.0 &&
         config.weak_observability_threshold >= 0.0 &&
         config.weak_observability_threshold <= 1.0 &&
         config.max_weak_delta_m >= 0.0 &&
         config.max_nominal_delta_m >= config.max_weak_delta_m;
}

bool validControlInputs(double requested_delta_m,
                        double observability_score,
                        double chainage_m,
                        double last_section_chainage_m) {
  return std::isfinite(requested_delta_m) &&
         std::isfinite(observability_score) &&
         std::isfinite(chainage_m) &&
         std::isfinite(last_section_chainage_m) &&
         observability_score >= 0.0 &&
         observability_score <= 1.0;
}

bool knownMachineState(const std::string& machine_state) {
  return machine_state == IDLE_STATIC ||
         machine_state == CUTTING_STATIC ||
         machine_state == FWD_MOVE ||
         machine_state == REV_MOVE ||
         machine_state == TURNING ||
         machine_state == CMD_MOVE_NO_DISP ||
         machine_state == CONFLICT ||
         machine_state == RELOCALIZING;
}

double clamp(double value, double limit) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

bool shouldSampleSection(double chainage_m, double last_section_chainage_m, const MappingPolicyConfig& config) {
  return chainage_m - last_section_chainage_m >= config.section_spacing_m;
}

ControlDecision decision(
    const std::string& action,
    double accepted_delta_m,
    bool stable_map_write,
    bool section_sample,
    const std::string& reason) {
  ControlDecision item;
  item.action = action;
  item.accepted_delta_m = accepted_delta_m;
  item.stable_map_write = stable_map_write;
  item.section_sample = section_sample;
  item.reason = reason;
  return item;
}

}  // namespace

ControlDecision decideMappingControl(
    const std::string& machine_state,
    double requested_delta_m,
    double observability_score,
    double chainage_m,
    double last_section_chainage_m,
    const MappingPolicyConfig& config) {
  if (!validPolicyConfig(config)) {
    return decision(REJECT, 0.0, false, false, "invalid_config");
  }
  if (!validControlInputs(requested_delta_m, observability_score, chainage_m, last_section_chainage_m)) {
    return decision(REJECT, 0.0, false, false, "invalid_input");
  }
  if (!knownMachineState(machine_state)) {
    return decision(REJECT, 0.0, false, false, "unknown_state");
  }

  if (machine_state == CONFLICT || machine_state == RELOCALIZING) {
    return decision(REJECT, 0.0, false, false, "unsafe_state");
  }
  if (machine_state == CMD_MOVE_NO_DISP) {
    return decision(REJECT, 0.0, false, false, "commanded_motion_without_displacement");
  }

  if (machine_state == IDLE_STATIC || machine_state == CUTTING_STATIC) {
    const bool sample = shouldSampleSection(chainage_m, last_section_chainage_m, config);
    return decision(FREEZE, 0.0, machine_state == IDLE_STATIC, sample, "static_freeze");
  }

  if (std::fabs(requested_delta_m) > config.max_nominal_delta_m) {
    requested_delta_m = clamp(requested_delta_m, config.max_nominal_delta_m);
  }

  if (observability_score < config.weak_observability_threshold) {
    const double limited = clamp(requested_delta_m, config.max_weak_delta_m);
    const bool sample = shouldSampleSection(chainage_m + limited, last_section_chainage_m, config);
    return decision(LIMIT, limited, machine_state == FWD_MOVE, sample, "weak_observability");
  }

  const bool stable_write = machine_state == FWD_MOVE;
  const bool sample = shouldSampleSection(chainage_m + requested_delta_m, last_section_chainage_m, config);
  return decision(ACCEPT, requested_delta_m, stable_write, sample, "nominal");
}

bool needsControlAnchor(double chainage_m, double last_anchor_chainage_m, const MappingPolicyConfig& config) {
  if (!validPolicyConfig(config) ||
      !std::isfinite(chainage_m) ||
      !std::isfinite(last_anchor_chainage_m)) {
    return false;
  }
  return chainage_m - last_anchor_chainage_m >= config.control_anchor_spacing_m;
}

std::string gradeSection(double completeness, double rmse_mm) {
  if (!std::isfinite(completeness) ||
      !std::isfinite(rmse_mm) ||
      completeness < 0.0 ||
      completeness > 1.0 ||
      rmse_mm < 0.0) {
    return "C";
  }
  if (completeness >= 0.90 && rmse_mm <= 25.0) {
    return "A";
  }
  if (completeness >= 0.75 && rmse_mm <= 40.0) {
    return "B";
  }
  return "C";
}

}  // namespace mapping_control
