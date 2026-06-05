#include "lio_eval_tools/validation_metrics.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <map>
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

void markRecordInvalid(bool* record_valid) {
  if (record_valid != nullptr) {
    *record_valid = false;
  }
}

std::map<std::string, std::string> parseKeyValuePairs(
    const std::string& line,
    bool* record_valid = nullptr) {
  std::map<std::string, std::string> values;
  if (record_valid != nullptr) {
    *record_valid = true;
  }
  if (line.empty()) {
    markRecordInvalid(record_valid);
    return values;
  }
  if (line.front() == ';' || line.back() == ';' ||
      line.find(";;") != std::string::npos) {
    markRecordInvalid(record_valid);
  }
  std::stringstream stream(line);
  std::string token;
  while (std::getline(stream, token, ';')) {
    const std::size_t split = token.find('=');
    if (split == std::string::npos) {
      markRecordInvalid(record_valid);
      continue;
    }
    const std::string key = trim(token.substr(0, split));
    const std::string value = token.substr(split + 1);
    if (key.empty()) {
      markRecordInvalid(record_valid);
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

double parseDouble(const std::map<std::string, std::string>& values,
                   const std::string& key,
                   const double fallback) {
  const auto iter = values.find(key);
  if (iter == values.end() || iter->second.empty()) {
    return fallback;
  }
  double parsed = 0.0;
  return parseStrictDoubleValue(iter->second, &parsed) ? parsed : fallback;
}

int parseInt(const std::map<std::string, std::string>& values,
             const std::string& key,
             const int fallback) {
  const auto iter = values.find(key);
  if (iter == values.end() || iter->second.empty()) {
    return fallback;
  }
  int parsed = 0;
  return parseStrictIntValue(iter->second, &parsed) ? parsed : fallback;
}

std::string parseString(const std::map<std::string, std::string>& values,
                        const std::string& key) {
  const auto iter = values.find(key);
  return iter == values.end() ? "" : iter->second;
}

bool hasKey(const std::map<std::string, std::string>& values,
            const std::string& key) {
  return values.find(key) != values.end();
}

double parseThresholdDouble(const std::map<std::string, std::string>& values,
                            const std::string& key) {
  const auto iter = values.find(key);
  double parsed = 0.0;
  if (iter != values.end() &&
      parseStrictDoubleValue(iter->second, &parsed)) {
    return parsed;
  }
  return -1.0;
}

int parseThresholdInt(const std::map<std::string, std::string>& values,
                      const std::string& key) {
  const auto iter = values.find(key);
  int parsed = 0;
  if (iter != values.end() &&
      parseStrictIntValue(iter->second, &parsed)) {
    return parsed;
  }
  return -1;
}

void parseThresholdOverrides(
    const std::map<std::string, std::string>& values,
    ScenarioValidationThresholds* record) {
  if (hasKey(values, "max_static_drift_m")) {
    record->thresholds.max_static_drift_m =
        parseThresholdDouble(values, "max_static_drift_m");
    record->has_max_static_drift_m = true;
  }
  if (hasKey(values, "max_length_error_percent")) {
    record->thresholds.max_length_error_percent =
        parseThresholdDouble(values, "max_length_error_percent");
    record->has_max_length_error_percent = true;
  }
  if (hasKey(values, "max_recovery_time_s")) {
    record->thresholds.max_recovery_time_s =
        parseThresholdDouble(values, "max_recovery_time_s");
    record->has_max_recovery_time_s = true;
  }
  if (hasKey(values, "max_wrong_loop_count")) {
    record->thresholds.max_wrong_loop_count =
        parseThresholdInt(values, "max_wrong_loop_count");
    record->has_max_wrong_loop_count = true;
  }
  if (hasKey(values, "max_queue_backlog")) {
    record->thresholds.max_queue_backlog =
        parseThresholdInt(values, "max_queue_backlog");
    record->has_max_queue_backlog = true;
  }
  if (hasKey(values, "max_pps_jitter_ms")) {
    record->thresholds.max_pps_jitter_ms =
        parseThresholdDouble(values, "max_pps_jitter_ms");
    record->has_max_pps_jitter_ms = true;
  }
}

void markAllThresholdOverridesInvalid(ScenarioValidationThresholds* record) {
  record->thresholds.max_static_drift_m = -1.0;
  record->thresholds.max_length_error_percent = -1.0;
  record->thresholds.max_recovery_time_s = -1.0;
  record->thresholds.max_wrong_loop_count = -1;
  record->thresholds.max_queue_backlog = -1;
  record->thresholds.max_pps_jitter_ms = -1.0;
  record->has_max_static_drift_m = true;
  record->has_max_length_error_percent = true;
  record->has_max_recovery_time_s = true;
  record->has_max_wrong_loop_count = true;
  record->has_max_queue_backlog = true;
  record->has_max_pps_jitter_ms = true;
}

ValidationThresholds mergeScenarioThresholds(
    const ValidationThresholds& default_thresholds,
    const ScenarioValidationThresholds& scenario_thresholds) {
  ValidationThresholds thresholds = default_thresholds;
  if (scenario_thresholds.has_max_static_drift_m) {
    thresholds.max_static_drift_m =
        scenario_thresholds.thresholds.max_static_drift_m;
  }
  if (scenario_thresholds.has_max_length_error_percent) {
    thresholds.max_length_error_percent =
        scenario_thresholds.thresholds.max_length_error_percent;
  }
  if (scenario_thresholds.has_max_recovery_time_s) {
    thresholds.max_recovery_time_s =
        scenario_thresholds.thresholds.max_recovery_time_s;
  }
  if (scenario_thresholds.has_max_wrong_loop_count) {
    thresholds.max_wrong_loop_count =
        scenario_thresholds.thresholds.max_wrong_loop_count;
  }
  if (scenario_thresholds.has_max_queue_backlog) {
    thresholds.max_queue_backlog =
        scenario_thresholds.thresholds.max_queue_backlog;
  }
  if (scenario_thresholds.has_max_pps_jitter_ms) {
    thresholds.max_pps_jitter_ms =
        scenario_thresholds.thresholds.max_pps_jitter_ms;
  }
  return thresholds;
}

ValidationThresholds thresholdsForScenario(
    const std::string& scenario,
    const ValidationThresholds& default_thresholds,
    const std::vector<ScenarioValidationThresholds>& scenario_thresholds) {
  for (const ScenarioValidationThresholds& candidate : scenario_thresholds) {
    if (candidate.scenario == kDuplicateKeyValue) {
      return mergeScenarioThresholds(default_thresholds, candidate);
    }
  }
  for (const ScenarioValidationThresholds& candidate : scenario_thresholds) {
    if (candidate.scenario == scenario) {
      return mergeScenarioThresholds(default_thresholds, candidate);
    }
  }
  return default_thresholds;
}

void addFailedCheck(std::vector<FailedCheck>* failed_checks,
                    const std::string& name,
                    const double value,
                    const double limit) {
  FailedCheck check;
  check.name = name;
  check.value = value;
  check.limit = limit;
  std::ostringstream message;
  message << name << "=" << std::fixed << std::setprecision(3) << value
          << " > limit " << limit;
  check.message = message.str();
  failed_checks->push_back(check);
}

void checkMax(std::vector<FailedCheck>* failed_checks,
              const std::string& name,
              const double value,
              const double limit) {
  if (!std::isfinite(value) || value < 0.0 ||
      !std::isfinite(limit) || limit < 0.0 || value > limit) {
    addFailedCheck(failed_checks, name, value, limit);
  }
}

void checkMax(std::vector<FailedCheck>* failed_checks,
              const std::string& name,
              const int value,
              const int limit) {
  if (value < 0 || limit < 0 || value > limit) {
    addFailedCheck(failed_checks, name, static_cast<double>(value),
                   static_cast<double>(limit));
  }
}

}  // namespace

ValidationMetrics parseMetricRecord(const std::string& line) {
  bool record_valid = true;
  const std::map<std::string, std::string> values =
      parseKeyValuePairs(line, &record_valid);
  ValidationMetrics metrics;
  metrics.scenario = parseString(values, "scenario");
  metrics.session_id = parseString(values, "session_id");
  metrics.static_drift_m = parseDouble(values, "static_drift_m", -1.0);
  metrics.length_error_percent =
      parseDouble(values, "length_error_percent", -1.0);
  metrics.recovery_time_s = parseDouble(values, "recovery_time_s", -1.0);
  metrics.wrong_loop_count = parseInt(values, "wrong_loop_count", -1);
  metrics.queue_backlog_max = parseInt(values, "queue_backlog_max", -1);
  metrics.pps_jitter_ms = parseDouble(values, "pps_jitter_ms", -1.0);
  if (!record_valid || hasDuplicateKey(values)) {
    metrics.static_drift_m = -1.0;
  }
  return metrics;
}

std::vector<ValidationMetrics> parseMetricRecords(const std::string& text) {
  std::vector<ValidationMetrics> records;
  std::stringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    const std::string stripped = trim(line);
    if (stripped.empty() || stripped[0] == '#') {
      continue;
    }
    records.push_back(parseMetricRecord(line));
  }
  return records;
}

std::vector<ScenarioValidationThresholds> parseScenarioThresholdRecords(
    const std::string& text) {
  std::vector<ScenarioValidationThresholds> records;
  std::stringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    const std::string stripped = trim(line);
    if (stripped.empty() || stripped[0] == '#') {
      continue;
    }
    bool record_valid = true;
    const std::map<std::string, std::string> values =
        parseKeyValuePairs(line, &record_valid);
    ScenarioValidationThresholds record;
    record.scenario = parseString(values, "scenario");
    if (!record_valid || hasDuplicateKey(values)) {
      record.scenario = kDuplicateKeyValue;
      markAllThresholdOverridesInvalid(&record);
      records.push_back(record);
      continue;
    }
    if (record.scenario.empty()) {
      continue;
    }
    parseThresholdOverrides(values, &record);
    records.push_back(record);
  }
  return records;
}

ValidationResult evaluateValidationMetrics(const ValidationMetrics& metrics,
                                           const ValidationThresholds& thresholds) {
  ValidationResult result;
  checkMax(&result.failed_checks, "static_drift_m", metrics.static_drift_m,
           thresholds.max_static_drift_m);
  checkMax(&result.failed_checks, "length_error_percent",
           metrics.length_error_percent, thresholds.max_length_error_percent);
  checkMax(&result.failed_checks, "recovery_time_s", metrics.recovery_time_s,
           thresholds.max_recovery_time_s);
  checkMax(&result.failed_checks, "wrong_loop_count", metrics.wrong_loop_count,
           thresholds.max_wrong_loop_count);
  checkMax(&result.failed_checks, "queue_backlog_max", metrics.queue_backlog_max,
           thresholds.max_queue_backlog);
  checkMax(&result.failed_checks, "pps_jitter_ms", metrics.pps_jitter_ms,
           thresholds.max_pps_jitter_ms);

  result.passed = result.failed_checks.empty();
  std::ostringstream summary;
  summary << "session=" << metrics.session_id << ";scenario=" << metrics.scenario
          << ";status=" << (result.passed ? "PASS" : "FAIL")
          << ";failed_checks=" << result.failed_checks.size();
  result.summary = summary.str();
  return result;
}

ValidationBatchReport evaluateValidationBatch(
    const std::vector<ValidationMetrics>& records,
    const ValidationThresholds& thresholds) {
  return evaluateValidationBatch(records, thresholds, {});
}

ValidationBatchReport evaluateValidationBatch(
    const std::vector<ValidationMetrics>& records,
    const ValidationThresholds& default_thresholds,
    const std::vector<ScenarioValidationThresholds>& scenario_thresholds) {
  ValidationBatchReport batch;
  batch.total_records = records.size();

  std::ostringstream text;
  text << "overall=";
  std::vector<std::string> per_record_reports;
  for (const ValidationMetrics& metrics : records) {
    const ValidationThresholds thresholds =
        thresholdsForScenario(metrics.scenario, default_thresholds,
                              scenario_thresholds);
    const ValidationResult result =
        evaluateValidationMetrics(metrics, thresholds);
    if (!result.passed) {
      ++batch.failed_records;
    }
    per_record_reports.push_back(formatValidationReport(metrics, result));
  }

  batch.passed = batch.failed_records == 0 && batch.total_records > 0;
  text << (batch.passed ? "PASS" : "FAIL")
       << ";total_records=" << batch.total_records
       << ";failed_records=" << batch.failed_records << "\n";
  for (const std::string& report : per_record_reports) {
    text << "---\n" << report;
  }
  batch.text = text.str();
  return batch;
}

std::string formatValidationReport(const ValidationMetrics& metrics,
                                   const ValidationResult& result) {
  std::ostringstream report;
  report << result.summary << "\n";
  report << "static_drift_m=" << metrics.static_drift_m << "\n";
  report << "length_error_percent=" << metrics.length_error_percent << "\n";
  report << "recovery_time_s=" << metrics.recovery_time_s << "\n";
  report << "wrong_loop_count=" << metrics.wrong_loop_count << "\n";
  report << "queue_backlog_max=" << metrics.queue_backlog_max << "\n";
  report << "pps_jitter_ms=" << metrics.pps_jitter_ms << "\n";
  for (const FailedCheck& check : result.failed_checks) {
    report << "failed_check=" << check.message << "\n";
  }
  return report.str();
}

}  // namespace lio_eval_tools
