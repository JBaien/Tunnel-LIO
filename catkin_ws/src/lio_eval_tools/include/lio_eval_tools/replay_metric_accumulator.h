#pragma once

#include <map>
#include <string>
#include <vector>

#include "lio_eval_tools/validation_metrics.h"

namespace lio_eval_tools {

struct ReplayEvent {
  std::string event;
  std::string scenario;
  std::string session_id;
  double stamp_s = 0.0;
  bool stamp_valid = true;
  bool record_valid = true;
  std::map<std::string, std::string> fields;
};

std::vector<ReplayEvent> parseReplayEventRecords(const std::string& text);

ValidationMetrics aggregateReplayMetrics(const std::vector<ReplayEvent>& events);

std::string formatMetricRecord(const ValidationMetrics& metrics);

}  // namespace lio_eval_tools
