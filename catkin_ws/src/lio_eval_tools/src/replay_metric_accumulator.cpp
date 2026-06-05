#include "lio_eval_tools/replay_metric_accumulator.h"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace lio_eval_tools {
namespace {

const char kDuplicateKeyValue[] = "__DUPLICATE_KEY__";

std::string trim(const std::string& value) {
  const std::string whitespace = " \t\r\n";
  const std::size_t first = value.find_first_not_of(whitespace);
  if (first == std::string::npos) {
    return "";
  }
  const std::size_t last = value.find_last_not_of(whitespace);
  return value.substr(first, last - first + 1);
}

std::map<std::string, std::string> parseKeyValuePairs(const std::string& line,
                                                      bool* record_valid) {
  std::map<std::string, std::string> values;
  if (record_valid != nullptr &&
      (line.empty() || line.front() == ';' || line.back() == ';' ||
       line.find(";;") != std::string::npos)) {
    *record_valid = false;
  }
  std::stringstream stream(line);
  std::string token;
  while (std::getline(stream, token, ';')) {
    const std::size_t split = token.find('=');
    if (split == std::string::npos) {
      if (record_valid != nullptr) {
        *record_valid = false;
      }
      continue;
    }
    const std::string key = trim(token.substr(0, split));
    const std::string value = token.substr(split + 1);
    if (key.empty()) {
      if (record_valid != nullptr) {
        *record_valid = false;
      }
      continue;
    }
    const auto inserted = values.emplace(key, value);
    if (!inserted.second) {
      inserted.first->second = kDuplicateKeyValue;
    }
  }
  return values;
}

bool hasDuplicateKey(const std::map<std::string, std::string>& values) {
  for (const auto& value : values) {
    if (value.second == kDuplicateKeyValue) {
      return true;
    }
  }
  return false;
}

bool hasField(const std::map<std::string, std::string>& values,
              const std::string& key) {
  return values.find(key) != values.end();
}

bool validReplayTextValue(const std::string& value) {
  return !value.empty() && value == trim(value) && value != "missing" &&
         value != kDuplicateKeyValue && value.find(';') == std::string::npos &&
         value.find('\n') == std::string::npos &&
         value.find('\r') == std::string::npos;
}

bool validScenarioToken(const std::string& value) {
  if (!validReplayTextValue(value)) {
    return false;
  }
  for (const char character : value) {
    const bool uppercase = character >= 'A' && character <= 'Z';
    const bool digit = character >= '0' && character <= '9';
    if (!uppercase && !digit && character != '_') {
      return false;
    }
  }
  return true;
}

bool validSessionToken(const std::string& value) {
  if (!validReplayTextValue(value) || value == "." || value == "..") {
    return false;
  }
  for (const char character : value) {
    const bool uppercase = character >= 'A' && character <= 'Z';
    const bool lowercase = character >= 'a' && character <= 'z';
    const bool digit = character >= '0' && character <= '9';
    if (!uppercase && !lowercase && !digit && character != '_' &&
        character != '.' && character != '-') {
      return false;
    }
  }
  return true;
}

bool hasReplayEventStructureError(const ReplayEvent& event) {
  return !event.record_valid || hasDuplicateKey(event.fields) ||
         event.fields.find("event") == event.fields.end() ||
         event.fields.find("session_id") == event.fields.end() ||
         event.fields.find("scenario") == event.fields.end() ||
         event.fields.find("t") == event.fields.end() ||
         !validReplayTextValue(event.event) ||
         !validSessionToken(event.session_id) ||
         !validScenarioToken(event.scenario);
}

bool parseStrictDoubleValue(const std::string& value, double* parsed_value) {
  if (value.empty()) {
    return false;
  }
  if (std::isspace(static_cast<unsigned char>(value[0])) != 0) {
    return false;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  if (parsed_value != nullptr) {
    *parsed_value = parsed;
  }
  return true;
}

bool parseStrictIntValue(const std::string& value, int* parsed_value) {
  if (value.empty()) {
    return false;
  }
  if (std::isspace(static_cast<unsigned char>(value[0])) != 0) {
    return false;
  }
  char* end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0' ||
      parsed < std::numeric_limits<int>::min() ||
      parsed > std::numeric_limits<int>::max()) {
    return false;
  }
  if (parsed_value != nullptr) {
    *parsed_value = static_cast<int>(parsed);
  }
  return true;
}

std::string stringField(const std::map<std::string, std::string>& values,
                        const std::string& key) {
  const auto iter = values.find(key);
  return iter == values.end() ? "" : iter->second;
}

double doubleField(const std::map<std::string, std::string>& values,
                   const std::string& key,
                   const double fallback) {
  const auto iter = values.find(key);
  if (iter == values.end() || iter->second.empty()) {
    return fallback;
  }
  double parsed = 0.0;
  return parseStrictDoubleValue(iter->second, &parsed) ? parsed : fallback;
}

int intField(const std::map<std::string, std::string>& values,
             const std::string& key,
             const int fallback) {
  const auto iter = values.find(key);
  if (iter == values.end() || iter->second.empty()) {
    return fallback;
  }
  int parsed = 0;
  return parseStrictIntValue(iter->second, &parsed) ? parsed : fallback;
}

bool readOptionalDouble(const std::map<std::string, std::string>& values,
                        const std::string& key,
                        double* parsed_value) {
  const auto iter = values.find(key);
  return iter != values.end() &&
         parseStrictDoubleValue(iter->second, parsed_value);
}

bool readOptionalInt(const std::map<std::string, std::string>& values,
                     const std::string& key,
                     int* parsed_value) {
  const auto iter = values.find(key);
  return iter != values.end() && parseStrictIntValue(iter->second, parsed_value);
}

double invalidMetricValue() {
  return std::numeric_limits<double>::quiet_NaN();
}

void updateScenarioAndSession(const ReplayEvent& event,
                              ValidationMetrics* metrics) {
  if (metrics->scenario.empty() && !event.scenario.empty()) {
    metrics->scenario = event.scenario;
  }
  if (metrics->session_id.empty() && !event.session_id.empty()) {
    metrics->session_id = event.session_id;
  }
}

}  // namespace

std::vector<ReplayEvent> parseReplayEventRecords(const std::string& text) {
  std::vector<ReplayEvent> events;
  std::stringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    const std::string stripped = trim(line);
    if (stripped.empty() || stripped[0] == '#') {
      continue;
    }
    ReplayEvent event;
    event.fields = parseKeyValuePairs(line, &event.record_valid);
    event.event = stringField(event.fields, "event");
    event.scenario = stringField(event.fields, "scenario");
    event.session_id = stringField(event.fields, "session_id");
    const auto timestamp = event.fields.find("t");
    if (timestamp != event.fields.end() && !timestamp->second.empty()) {
      event.stamp_valid =
          parseStrictDoubleValue(timestamp->second, &event.stamp_s) &&
          event.stamp_s >= 0.0;
    }
    events.push_back(event);
  }
  return events;
}

ValidationMetrics aggregateReplayMetrics(const std::vector<ReplayEvent>& events) {
  ValidationMetrics metrics;
  metrics.static_drift_m = 0.0;
  metrics.length_error_percent = 0.0;
  metrics.recovery_time_s = 0.0;
  metrics.wrong_loop_count = 0;
  metrics.queue_backlog_max = 0;
  metrics.pps_jitter_ms = 0.0;

  double last_power_loss_time = std::numeric_limits<double>::quiet_NaN();
  for (const ReplayEvent& event : events) {
    updateScenarioAndSession(event, &metrics);
    if (hasReplayEventStructureError(event)) {
      metrics.static_drift_m = invalidMetricValue();
      continue;
    }
    if (!event.stamp_valid) {
      metrics.recovery_time_s = invalidMetricValue();
      last_power_loss_time = invalidMetricValue();
      continue;
    }

    double static_drift = 0.0;
    if (hasField(event.fields, "static_drift_m") &&
        (!readOptionalDouble(event.fields, "static_drift_m", &static_drift) ||
         static_drift < 0.0)) {
      metrics.static_drift_m = invalidMetricValue();
    } else if (static_drift >= 0.0 && hasField(event.fields, "static_drift_m")) {
      metrics.static_drift_m =
          std::max(metrics.static_drift_m, std::abs(static_drift));
    }

    double direct_length_error = 0.0;
    if (hasField(event.fields, "length_error_percent") &&
        !readOptionalDouble(event.fields, "length_error_percent",
                            &direct_length_error)) {
      metrics.length_error_percent = invalidMetricValue();
    } else if (hasField(event.fields, "length_error_percent") &&
               direct_length_error < 0.0) {
      metrics.length_error_percent = invalidMetricValue();
    } else if (hasField(event.fields, "length_error_percent") &&
               direct_length_error >= 0.0) {
      metrics.length_error_percent =
          std::max(metrics.length_error_percent, std::abs(direct_length_error));
    } else {
      double chainage = 0.0;
      double reference = 0.0;
      const bool has_chainage = hasField(event.fields, "chainage_m");
      const bool has_reference = hasField(event.fields, "reference_chainage_m");
      if ((has_chainage || has_reference) &&
          (!readOptionalDouble(event.fields, "chainage_m", &chainage) ||
           !readOptionalDouble(event.fields, "reference_chainage_m",
                               &reference))) {
        metrics.length_error_percent = invalidMetricValue();
      } else if (has_chainage && has_reference) {
        const double denominator = std::max(std::abs(reference), 1.0);
        const double percent = std::abs(chainage - reference) / denominator * 100.0;
        metrics.length_error_percent =
            std::max(metrics.length_error_percent, percent);
      }
    }

    if (event.event == "power_loss" && !event.stamp_valid) {
      metrics.recovery_time_s = invalidMetricValue();
      last_power_loss_time = invalidMetricValue();
    } else if (event.event == "power_loss") {
      last_power_loss_time = event.stamp_s;
    } else if ((event.event == "recovered" ||
                event.event == "recovery_complete") &&
               !event.stamp_valid) {
      metrics.recovery_time_s = invalidMetricValue();
    } else if ((event.event == "recovered" ||
                event.event == "recovery_complete") &&
               !std::isnan(last_power_loss_time)) {
      metrics.recovery_time_s =
          std::max(metrics.recovery_time_s,
                   std::max(0.0, event.stamp_s - last_power_loss_time));
    }

    int wrong_loop = 0;
    if (hasField(event.fields, "wrong_loop") &&
        (!readOptionalInt(event.fields, "wrong_loop", &wrong_loop) ||
         wrong_loop < 0)) {
      metrics.wrong_loop_count = -1;
    } else if (metrics.wrong_loop_count >= 0) {
      metrics.wrong_loop_count += wrong_loop;
    }

    int queue_backlog = 0;
    if (hasField(event.fields, "queue_backlog") &&
        (!readOptionalInt(event.fields, "queue_backlog", &queue_backlog) ||
         queue_backlog < 0)) {
      metrics.queue_backlog_max = -1;
    } else if (metrics.queue_backlog_max >= 0) {
      metrics.queue_backlog_max =
          std::max(metrics.queue_backlog_max, queue_backlog);
    }

    double pps_jitter = 0.0;
    if (hasField(event.fields, "pps_jitter_ms") &&
        (!readOptionalDouble(event.fields, "pps_jitter_ms", &pps_jitter) ||
         pps_jitter < 0.0)) {
      metrics.pps_jitter_ms = invalidMetricValue();
    } else if (hasField(event.fields, "pps_jitter_ms") && pps_jitter >= 0.0) {
      metrics.pps_jitter_ms =
          std::max(metrics.pps_jitter_ms, std::abs(pps_jitter));
    }
  }

  return metrics;
}

std::string formatMetricRecord(const ValidationMetrics& metrics) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  stream << "scenario=" << metrics.scenario
         << ";session_id=" << metrics.session_id
         << ";static_drift_m=" << metrics.static_drift_m
         << ";length_error_percent=" << metrics.length_error_percent
         << ";recovery_time_s=" << metrics.recovery_time_s
         << ";wrong_loop_count=" << metrics.wrong_loop_count
         << ";queue_backlog_max=" << metrics.queue_backlog_max
         << ";pps_jitter_ms=" << metrics.pps_jitter_ms;
  return stream.str();
}

}  // namespace lio_eval_tools
