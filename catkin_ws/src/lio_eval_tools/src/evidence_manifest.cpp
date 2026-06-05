#include "lio_eval_tools/evidence_manifest.h"

#include "lio_eval_tools/replay_metric_accumulator.h"

#include <cmath>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <sys/stat.h>

namespace lio_eval_tools {
namespace {

const char kDuplicateKeyValue[] = "__DUPLICATE_KEY__";

struct RuntimeStabilityRunLogFields {
  std::string started_at;
  std::string finished_at;
  std::string runtime_dir;
  std::string samples;
  std::string interval;
  std::string exit_status;
  std::string capture_exit_status;
};

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
    if (values.find(key) == values.end()) {
      values[key] = value;
    } else {
      values[key] = kDuplicateKeyValue;
    }
  }
  return values;
}

std::map<std::string, std::string> parseKeyValueLines(
    const std::string& text,
    bool* record_valid = nullptr) {
  std::map<std::string, std::string> values;
  if (record_valid != nullptr) {
    *record_valid = true;
  }
  std::stringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    if (trim(line).empty()) {
      continue;
    }
    const std::size_t split = line.find('=');
    if (split == std::string::npos) {
      markRecordInvalid(record_valid);
      continue;
    }
    const std::string key = trim(line.substr(0, split));
    const std::string value = line.substr(split + 1);
    if (key.empty()) {
      markRecordInvalid(record_valid);
      continue;
    }
    if (values.find(key) == values.end()) {
      values[key] = value;
    } else {
      values[key] = kDuplicateKeyValue;
    }
  }
  return values;
}

bool hasDuplicateKeyValue(const std::map<std::string, std::string>& values) {
  for (const auto& value : values) {
    if (value.second == kDuplicateKeyValue) {
      return true;
    }
  }
  return false;
}

std::string lookup(const std::map<std::string, std::string>& values,
                   const std::string& key) {
  const auto iter = values.find(key);
  return iter == values.end() ? "" : iter->second;
}

bool fileExists(const std::string& path) {
  struct stat status;
  return stat(path.c_str(), &status) == 0 && S_ISREG(status.st_mode) &&
         status.st_size > 0;
}

std::string joinPath(const std::string& base_dir, const std::string& path) {
  if (path.empty() || (!path.empty() && path[0] == '/')) {
    return path;
  }
  if (base_dir.empty()) {
    return path;
  }
  return base_dir[base_dir.size() - 1] == '/' ? base_dir + path
                                               : base_dir + "/" + path;
}

std::string readTextFile(const std::string& path) {
  std::ifstream input(path.c_str());
  if (!input) {
    return "";
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

double lookupDouble(const std::map<std::string, std::string>& values,
                    const std::string& key);
bool lookupStrictDouble(const std::map<std::string, std::string>& values,
                        const std::string& key,
                        double* parsed_value);
bool lookupStrictPositiveInteger(const std::map<std::string, std::string>& values,
                                 const std::string& key);
bool lookupStrictZeroInteger(const std::map<std::string, std::string>& values,
                             const std::string& key);
bool summaryFailureCounterClear(const std::map<std::string, std::string>& values,
                                const std::string& key);
bool parseStrictDoubleValue(const std::string& value, double* parsed_value);
bool parseStrictPositiveIntegerValue(const std::string& value);
bool parseStrictPositiveIntegerValue(const std::string& value, long* parsed_value);
bool parseStrictNonnegativeIntegerValue(const std::string& value,
                                        long long* parsed_value);
bool validSectionStateSource(const std::string& state_source);

bool validEvidenceTextValue(const std::string& value) {
  return !value.empty() && value == trim(value) && value != "missing" &&
         value != kDuplicateKeyValue && value.find(';') == std::string::npos &&
         value.find('\n') == std::string::npos &&
         value.find('\r') == std::string::npos;
}

bool validRosTopicSegment(const std::string& value) {
  if (value.empty()) {
    return false;
  }
  const char first = value[0];
  if (!((first >= 'A' && first <= 'Z') || (first >= 'a' && first <= 'z'))) {
    return false;
  }
  for (std::size_t i = 1; i < value.size(); ++i) {
    const char character = value[i];
    const bool uppercase = character >= 'A' && character <= 'Z';
    const bool lowercase = character >= 'a' && character <= 'z';
    const bool digit = character >= '0' && character <= '9';
    if (!uppercase && !lowercase && !digit && character != '_') {
      return false;
    }
  }
  return true;
}

bool validRosTopicName(const std::string& value) {
  if (!validEvidenceTextValue(value)) {
    return false;
  }
  std::size_t segment_begin = 0;
  if (value[0] == '/' || value[0] == '~') {
    segment_begin = 1;
  }
  if (segment_begin >= value.size()) {
    return false;
  }
  while (segment_begin < value.size()) {
    const std::size_t slash = value.find('/', segment_begin);
    const std::size_t segment_end =
        slash == std::string::npos ? value.size() : slash;
    if (!validRosTopicSegment(
            value.substr(segment_begin, segment_end - segment_begin))) {
      return false;
    }
    if (slash == std::string::npos) {
      break;
    }
    segment_begin = slash + 1;
  }
  return true;
}

bool allDigits(const std::string& value,
               const std::size_t first,
               const std::size_t last_exclusive) {
  if (first >= last_exclusive || last_exclusive > value.size()) {
    return false;
  }
  for (std::size_t i = first; i < last_exclusive; ++i) {
    if (value[i] < '0' || value[i] > '9') {
      return false;
    }
  }
  return true;
}

int parseFixedWidthInteger(const std::string& value,
                           const std::size_t first,
                           const std::size_t last_exclusive) {
  int parsed = 0;
  for (std::size_t i = first; i < last_exclusive; ++i) {
    parsed = parsed * 10 + (value[i] - '0');
  }
  return parsed;
}

bool leapYear(const int year) {
  return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

int daysInMonth(const int year, const int month) {
  switch (month) {
    case 1:
    case 3:
    case 5:
    case 7:
    case 8:
    case 10:
    case 12:
      return 31;
    case 4:
    case 6:
    case 9:
    case 11:
      return 30;
    case 2:
      return leapYear(year) ? 29 : 28;
    default:
      return 0;
  }
}

long long daysFromCivil(int year, const int month, const int day) {
  year -= month <= 2 ? 1 : 0;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned year_of_era =
      static_cast<unsigned>(year - era * 400);
  const unsigned month_for_year =
      static_cast<unsigned>(month + (month > 2 ? -3 : 9));
  const unsigned day_of_year =
      (153 * month_for_year + 2) / 5 + static_cast<unsigned>(day) - 1;
  const unsigned day_of_era =
      year_of_era * 365 + year_of_era / 4 - year_of_era / 100 +
      day_of_year;
  return static_cast<long long>(era) * 146097 +
         static_cast<long long>(day_of_era) - 719468;
}

bool validIso8601SecondsTimestamp(const std::string& value) {
  if (!validEvidenceTextValue(value)) {
    return false;
  }
  if (value.size() != 20 && value.size() != 25) {
    return false;
  }
  if (value[4] != '-' || value[7] != '-' || value[10] != 'T' ||
      value[13] != ':' || value[16] != ':') {
    return false;
  }
  if (!allDigits(value, 0, 4) || !allDigits(value, 5, 7) ||
      !allDigits(value, 8, 10) || !allDigits(value, 11, 13) ||
      !allDigits(value, 14, 16) || !allDigits(value, 17, 19)) {
    return false;
  }
  const int year = parseFixedWidthInteger(value, 0, 4);
  const int month = parseFixedWidthInteger(value, 5, 7);
  const int day = parseFixedWidthInteger(value, 8, 10);
  const int hour = parseFixedWidthInteger(value, 11, 13);
  const int minute = parseFixedWidthInteger(value, 14, 16);
  const int second = parseFixedWidthInteger(value, 17, 19);
  if (year <= 0 || month < 1 || month > 12 || day < 1 ||
      day > daysInMonth(year, month) || hour > 23 || minute > 59 ||
      second > 59) {
    return false;
  }
  if (value.size() == 20) {
    return value[19] == 'Z';
  }
  if ((value[19] != '+' && value[19] != '-') || value[22] != ':' ||
      !allDigits(value, 20, 22) || !allDigits(value, 23, 25)) {
    return false;
  }
  const int offset_hour = parseFixedWidthInteger(value, 20, 22);
  const int offset_minute = parseFixedWidthInteger(value, 23, 25);
  return offset_hour <= 23 && offset_minute <= 59;
}

bool parseIso8601SecondsTimestampToUnixSeconds(const std::string& value,
                                               long long* seconds) {
  if (seconds == nullptr || !validIso8601SecondsTimestamp(value)) {
    return false;
  }
  const int year = parseFixedWidthInteger(value, 0, 4);
  const int month = parseFixedWidthInteger(value, 5, 7);
  const int day = parseFixedWidthInteger(value, 8, 10);
  const int hour = parseFixedWidthInteger(value, 11, 13);
  const int minute = parseFixedWidthInteger(value, 14, 16);
  const int second = parseFixedWidthInteger(value, 17, 19);
  long long offset_seconds = 0;
  if (value.size() == 25) {
    const int offset_hour = parseFixedWidthInteger(value, 20, 22);
    const int offset_minute = parseFixedWidthInteger(value, 23, 25);
    offset_seconds =
        static_cast<long long>(offset_hour) * 3600LL +
        static_cast<long long>(offset_minute) * 60LL;
    if (value[19] == '-') {
      offset_seconds = -offset_seconds;
    }
  }
  const long long local_seconds =
      daysFromCivil(year, month, day) * 86400LL +
      static_cast<long long>(hour) * 3600LL +
      static_cast<long long>(minute) * 60LL + second;
  *seconds = local_seconds - offset_seconds;
  return true;
}

bool iso8601SecondsFinishedNotBeforeStarted(const std::string& started_at,
                                            const std::string& finished_at) {
  long long started_seconds = 0;
  long long finished_seconds = 0;
  return parseIso8601SecondsTimestampToUnixSeconds(started_at,
                                                   &started_seconds) &&
         parseIso8601SecondsTimestampToUnixSeconds(finished_at,
                                                   &finished_seconds) &&
         finished_seconds >= started_seconds;
}

bool iso8601SecondsNotBefore(const std::string& timestamp,
                             const std::string& evidence_timestamp) {
  long long timestamp_seconds = 0;
  long long evidence_seconds = 0;
  return parseIso8601SecondsTimestampToUnixSeconds(timestamp,
                                                   &timestamp_seconds) &&
         parseIso8601SecondsTimestampToUnixSeconds(evidence_timestamp,
                                                   &evidence_seconds) &&
         timestamp_seconds >= evidence_seconds;
}

bool fieldAcceptanceTimestampCoversEvidence(
    const std::map<std::string, std::string>& values,
    const std::string& expected_power_loss_source) {
  const std::string timestamp = lookup(values, "timestamp");
  if (!iso8601SecondsNotBefore(timestamp,
                               lookup(values, "time_sync_timestamp")) ||
      !iso8601SecondsNotBefore(
          timestamp,
          lookup(values, "runtime_deployment_timestamp")) ||
      !iso8601SecondsNotBefore(
          timestamp,
          lookup(values, "runtime_health_timestamp")) ||
      !iso8601SecondsNotBefore(
          timestamp,
          lookup(values, "runtime_stability_summary_timestamp")) ||
      !iso8601SecondsNotBefore(
          timestamp,
          lookup(values, "runtime_stability_csv_last_timestamp")) ||
      !iso8601SecondsNotBefore(
          timestamp,
          lookup(values, "runtime_stability_run_log_finished_at")) ||
      !iso8601SecondsNotBefore(timestamp,
                               lookup(values, "pps_ptp_wiring_timestamp")) ||
      !iso8601SecondsNotBefore(
          timestamp,
          lookup(values, "power_loss_resume_timestamp")) ||
      !iso8601SecondsNotBefore(timestamp,
                               lookup(values, "wiring_verified_at"))) {
    return false;
  }
  return expected_power_loss_source != "manual_file" ||
         iso8601SecondsNotBefore(timestamp, lookup(values, "resume_verified_at"));
}

long long runtimeStabilityRunLogElapsedToleranceSeconds(const long interval_s) {
  if (interval_s <= 0) {
    return 0;
  }
  return interval_s < 60 ? interval_s : 60;
}

bool runtimeStabilityRunLogElapsedAndRequired(
    const std::string& started_at,
    const std::string& finished_at,
    const long samples,
    const long interval_s,
    long long* elapsed_s,
    long long* required_elapsed_s) {
  if (elapsed_s == nullptr || required_elapsed_s == nullptr) {
    return false;
  }
  if (samples <= 0 || interval_s <= 0) {
    return false;
  }
  long long started_seconds = 0;
  long long finished_seconds = 0;
  if (!parseIso8601SecondsTimestampToUnixSeconds(started_at,
                                                 &started_seconds) ||
      !parseIso8601SecondsTimestampToUnixSeconds(finished_at,
                                                 &finished_seconds)) {
    return false;
  }
  const long long elapsed = finished_seconds - started_seconds;
  if (elapsed < 0) {
    return false;
  }
  const long long sample_count = static_cast<long long>(samples);
  const long long interval_seconds = static_cast<long long>(interval_s);
  if (sample_count > std::numeric_limits<long long>::max() / interval_seconds) {
    return false;
  }
  const long long configured_duration_s = sample_count * interval_seconds;
  const long long tolerance_s =
      runtimeStabilityRunLogElapsedToleranceSeconds(interval_s);
  const long long required =
      configured_duration_s > tolerance_s ? configured_duration_s - tolerance_s
                                          : 0;
  *elapsed_s = elapsed;
  *required_elapsed_s = required;
  return true;
}

bool runtimeStabilityRunLogElapsedCoversDuration(
    const std::string& started_at,
    const std::string& finished_at,
    const long samples,
    const long interval_s) {
  long long elapsed_s = 0;
  long long required_elapsed_s = 0;
  return runtimeStabilityRunLogElapsedAndRequired(started_at,
                                                  finished_at,
                                                  samples,
                                                  interval_s,
                                                  &elapsed_s,
                                                  &required_elapsed_s) &&
         elapsed_s >= required_elapsed_s;
}

bool runtimeStabilityDurationMatchesSummary(const double duration_h,
                                            const long samples,
                                            const long interval_s) {
  if (samples <= 0 || interval_s <= 0) {
    return false;
  }
  const double expected_duration_h =
      (static_cast<double>(samples) * static_cast<double>(interval_s)) / 3600.0;
  const double expected_reported_duration_h =
      std::round(expected_duration_h * 100.0) / 100.0;
  return std::fabs(duration_h - expected_reported_duration_h) <= 1.0e-9;
}

bool validScenarioToken(const std::string& value) {
  if (!validEvidenceTextValue(value)) {
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
  if (!validEvidenceTextValue(value) || value == "." || value == "..") {
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

bool hasDotPathSegment(const std::string& value) {
  std::stringstream stream(value);
  std::string segment;
  while (std::getline(stream, segment, '/')) {
    if (segment == "." || segment == "..") {
      return true;
    }
  }
  return false;
}

bool validAbsolutePath(const std::string& value) {
  return validEvidenceTextValue(value) && value.size() > 1 &&
         value[0] == '/' && !hasDotPathSegment(value);
}

bool validRuntimeDirPath(const std::string& value) {
  return validAbsolutePath(value) && value.find(',') == std::string::npos;
}

bool validBundleRelativePath(const std::string& value) {
  if (!validEvidenceTextValue(value) || value[0] == '/' ||
      value.find('\\') != std::string::npos) {
    return false;
  }
  std::stringstream stream(value);
  std::string segment;
  while (std::getline(stream, segment, '/')) {
    if (segment.empty() || segment == "." || segment == "..") {
      return false;
    }
  }
  return true;
}

bool evidencePathReferenceMatches(const std::string& value,
                                  const std::string& expected_path,
                                  const std::string& base_dir) {
  return (validAbsolutePath(value) || validBundleRelativePath(value)) &&
         joinPath(base_dir, value) == expected_path;
}

bool pathWithinBaseDir(const std::string& path, const std::string& base_dir) {
  if (path.empty() || base_dir.empty()) {
    return false;
  }
  std::string prefix = base_dir;
  if (prefix[prefix.size() - 1] != '/') {
    prefix += "/";
  }
  return path.compare(0, prefix.size(), prefix) == 0;
}

bool evidenceFileReferenceExistsInBundle(const std::string& value,
                                         const std::string& base_dir) {
  if (validBundleRelativePath(value)) {
    return fileExists(joinPath(base_dir, value));
  }
  if (!validAbsolutePath(value)) {
    return false;
  }
  const std::string resolved_path = joinPath(base_dir, value);
  return pathWithinBaseDir(resolved_path, base_dir) && fileExists(resolved_path);
}

std::string parentDir(const std::string& path) {
  const std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos == std::string::npos) {
    return "";
  }
  if (slash_pos == 0) {
    return "/";
  }
  return path.substr(0, slash_pos);
}

bool hasReplayMetricEvidence(const std::vector<ReplayEvent>& events) {
  bool has_power_loss = false;
  for (const ReplayEvent& event : events) {
    if (event.fields.find("static_drift_m") != event.fields.end() ||
        event.fields.find("length_error_percent") != event.fields.end() ||
        (event.fields.find("chainage_m") != event.fields.end() &&
         event.fields.find("reference_chainage_m") != event.fields.end()) ||
        event.fields.find("wrong_loop") != event.fields.end() ||
        event.fields.find("queue_backlog") != event.fields.end() ||
        event.fields.find("pps_jitter_ms") != event.fields.end()) {
      return true;
    }
    if (event.event == "power_loss") {
      has_power_loss = true;
    } else if ((event.event == "recovered" ||
                event.event == "recovery_complete") &&
               has_power_loss) {
      return true;
    }
  }
  return false;
}

bool eventFilePassed(const std::string& text,
                     const std::string& expected_session_id,
                     const std::string& expected_scenario) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(text);
  if (events.empty()) {
    return false;
  }
  for (const ReplayEvent& event : events) {
    if (!event.record_valid || hasDuplicateKeyValue(event.fields) ||
        event.fields.find("event") == event.fields.end() ||
        event.fields.find("session_id") == event.fields.end() ||
        event.fields.find("scenario") == event.fields.end() ||
        event.fields.find("t") == event.fields.end() ||
        !validEvidenceTextValue(event.event) ||
        !validSessionToken(event.session_id) ||
        !validScenarioToken(event.scenario) ||
        !event.stamp_valid) {
      return false;
    }
  }
  std::vector<ReplayEvent> matching_events;
  for (const ReplayEvent& event : events) {
    if (event.session_id == expected_session_id &&
        event.scenario == expected_scenario) {
      matching_events.push_back(event);
    }
  }
  if (matching_events.empty() || !hasReplayMetricEvidence(matching_events)) {
    return false;
  }
  const ValidationMetrics metrics = aggregateReplayMetrics(matching_events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});
  return result.passed && metrics.session_id == expected_session_id &&
         metrics.scenario == expected_scenario;
}

bool metricsReportPassed(const std::string& text,
                         const std::string& expected_session_id,
                         const std::string& expected_scenario) {
  std::stringstream stream(text);
  std::string line;
  bool summary_seen = false;
  bool matching_record_seen = false;
  while (std::getline(stream, line)) {
    if (trim(line).empty() || line == "---") {
      continue;
    }
    bool record_valid = true;
    const std::map<std::string, std::string> values =
        parseKeyValuePairs(line, &record_valid);
    if (!record_valid || hasDuplicateKeyValue(values)) {
      return false;
    }
    if (!summary_seen) {
      if (lookup(values, "overall") != "PASS" ||
          !lookupStrictPositiveInteger(values, "total_records") ||
          !lookupStrictZeroInteger(values, "failed_records")) {
        return false;
      }
      summary_seen = true;
      continue;
    }
    if (lookup(values, "session") == expected_session_id &&
        lookup(values, "scenario") == expected_scenario &&
        lookup(values, "status") == "PASS" &&
        lookupStrictZeroInteger(values, "failed_checks")) {
      matching_record_seen = true;
    }
  }
  return summary_seen && matching_record_seen;
}

bool metricsReportRecoveryTimeMatches(const std::string& text,
                                      const std::string& expected_session_id,
                                      const std::string& expected_scenario,
                                      const double expected_recovery_time_s) {
  std::stringstream stream(text);
  std::string line;
  bool summary_seen = false;
  bool matching_record = false;
  bool matching_record_seen = false;
  double record_recovery_time_s = 0.0;
  int recovery_time_count = 0;

  const auto recoveryMatches = [&](const double recovery_time_s) {
    return std::fabs(recovery_time_s - expected_recovery_time_s) <= 1.0e-9;
  };
  const auto finishMatchingBlock = [&]() {
    if (!matching_record) {
      return true;
    }
    matching_record_seen = true;
    return recovery_time_count == 1 &&
           recoveryMatches(record_recovery_time_s);
  };

  while (std::getline(stream, line)) {
    if (trim(line).empty()) {
      continue;
    }
    if (!summary_seen) {
      summary_seen = true;
      continue;
    }
    if (line == "---") {
      if (!finishMatchingBlock()) {
        return false;
      }
      matching_record = false;
      recovery_time_count = 0;
      record_recovery_time_s = 0.0;
      continue;
    }
    if (line.find(';') != std::string::npos) {
      if (!finishMatchingBlock()) {
        return false;
      }
      bool record_valid = true;
      const std::map<std::string, std::string> values =
          parseKeyValuePairs(line, &record_valid);
      if (!record_valid || hasDuplicateKeyValue(values)) {
        return false;
      }
      matching_record =
          lookup(values, "session") == expected_session_id &&
          lookup(values, "scenario") == expected_scenario &&
          lookup(values, "status") == "PASS" &&
          lookupStrictZeroInteger(values, "failed_checks");
      recovery_time_count = 0;
      record_recovery_time_s = 0.0;
      const std::string inline_recovery_time = lookup(values, "recovery_time_s");
      if (matching_record && !inline_recovery_time.empty()) {
        if (!parseStrictDoubleValue(inline_recovery_time,
                                    &record_recovery_time_s)) {
          return false;
        }
        ++recovery_time_count;
      }
      continue;
    }
    if (matching_record) {
      bool record_valid = true;
      const std::map<std::string, std::string> values =
          parseKeyValuePairs(line, &record_valid);
      if (!record_valid || hasDuplicateKeyValue(values)) {
        return false;
      }
      const std::string recovery_time = lookup(values, "recovery_time_s");
      if (!recovery_time.empty()) {
        if (!parseStrictDoubleValue(recovery_time, &record_recovery_time_s)) {
          return false;
        }
        ++recovery_time_count;
      }
    }
  }
  return finishMatchingBlock() && summary_seen && matching_record_seen;
}

bool timeSyncPassed(const std::string& text,
                    const std::string& base_dir,
                    std::string* timestamp,
                    std::string* time_status_topic,
                    std::string* pps_topic,
                    std::string* pps_jitter_ms,
                    std::string* mean_offset_ms,
                    std::string* time_sync_raw) {
  bool record_valid = true;
  const std::map<std::string, std::string> values =
      parseKeyValueLines(text, &record_valid);
  if (!record_valid || hasDuplicateKeyValue(values)) {
    return false;
  }
  const std::string parsed_timestamp = lookup(values, "timestamp");
  const std::string parsed_time_status_topic =
      lookup(values, "time_status_topic");
  const std::string parsed_pps_topic = lookup(values, "pps_topic");
  const std::string parsed_pps_jitter_ms = lookup(values, "pps_jitter_ms");
  const std::string parsed_mean_offset_ms = lookup(values, "mean_offset_ms");
  const std::string parsed_time_sync_raw = lookup(values, "raw");
  if (timestamp != nullptr) {
    *timestamp = parsed_timestamp;
  }
  if (time_status_topic != nullptr) {
    *time_status_topic = parsed_time_status_topic;
  }
  if (pps_topic != nullptr) {
    *pps_topic = parsed_pps_topic;
  }
  if (pps_jitter_ms != nullptr) {
    *pps_jitter_ms = parsed_pps_jitter_ms;
  }
  if (mean_offset_ms != nullptr) {
    *mean_offset_ms = parsed_mean_offset_ms;
  }
  if (time_sync_raw != nullptr) {
    *time_sync_raw = parsed_time_sync_raw;
  }
  double pps_jitter_ms_value = 0.0;
  double mean_offset_ms_value = 0.0;
  return lookup(values, "time_sync_status") == "PASS" &&
         validIso8601SecondsTimestamp(parsed_timestamp) &&
         validRosTopicName(parsed_time_status_topic) &&
         validRosTopicName(parsed_pps_topic) &&
         lookup(values, "capture_status") == "CAPTURED" &&
         lookup(values, "pps_status") == "PASS" &&
         lookup(values, "clock_offset_status") == "PASS" &&
         lookupStrictDouble(values, "pps_jitter_ms", &pps_jitter_ms_value) &&
         lookupStrictDouble(values, "mean_offset_ms", &mean_offset_ms_value) &&
         evidenceFileReferenceExistsInBundle(parsed_time_sync_raw, base_dir) &&
         pps_jitter_ms_value >= 0.0;
}

bool ppsPtpWiringPassed(const std::string& text,
                        const bool time_sync_passed,
                        const std::string& expected_pps_jitter_ms,
                        const std::string& expected_mean_offset_ms,
                        const std::string& expected_time_status_topic,
                        const std::string& expected_pps_topic,
                        const std::string& expected_time_sync_report_path,
                        const std::string& expected_time_sync_raw,
                        const std::string& expected_time_sync_timestamp,
                        const std::string& base_dir,
                        std::string* pps_ptp_wiring_timestamp,
                        std::string* wiring_verified_by,
                        std::string* wiring_verified_at) {
  bool record_valid = true;
  const std::map<std::string, std::string> values =
      parseKeyValueLines(text, &record_valid);
  if (!record_valid || hasDuplicateKeyValue(values)) {
    return false;
  }
  const std::string parsed_wiring_verified_by =
      lookup(values, "wiring_verified_by");
  const std::string parsed_wiring_verified_at =
      lookup(values, "wiring_verified_at");
  const std::string parsed_pps_ptp_wiring_timestamp =
      lookup(values, "timestamp");
  if (pps_ptp_wiring_timestamp != nullptr) {
    *pps_ptp_wiring_timestamp = parsed_pps_ptp_wiring_timestamp;
  }
  if (wiring_verified_by != nullptr) {
    *wiring_verified_by = parsed_wiring_verified_by;
  }
  if (wiring_verified_at != nullptr) {
    *wiring_verified_at = parsed_wiring_verified_at;
  }
  double pps_jitter_ms = 0.0;
  double mean_offset_ms = 0.0;
  return time_sync_passed &&
         validIso8601SecondsTimestamp(parsed_pps_ptp_wiring_timestamp) &&
         lookup(values, "pps_ptp_wiring_verified") == "PASS" &&
         evidencePathReferenceMatches(lookup(values, "time_sync_report"),
                                      expected_time_sync_report_path,
                                      base_dir) &&
         validEvidenceTextValue(lookup(values, "time_sync_raw")) &&
         lookup(values, "time_sync_raw") == expected_time_sync_raw &&
         lookup(values, "time_sync_raw_status") == "PASS" &&
         validIso8601SecondsTimestamp(lookup(values, "time_sync_timestamp")) &&
         lookup(values, "time_sync_timestamp") ==
             expected_time_sync_timestamp &&
         lookup(values, "time_sync_status") == "PASS" &&
         lookup(values, "capture_status") == "CAPTURED" &&
         validRosTopicName(lookup(values, "time_status_topic")) &&
         lookup(values, "time_status_topic") == expected_time_status_topic &&
         validRosTopicName(lookup(values, "pps_topic")) &&
         lookup(values, "pps_topic") == expected_pps_topic &&
         lookup(values, "pps_status") == "PASS" &&
         lookup(values, "clock_offset_status") == "PASS" &&
         lookupStrictDouble(values, "pps_jitter_ms", &pps_jitter_ms) &&
         lookup(values, "pps_jitter_ms") == expected_pps_jitter_ms &&
         lookupStrictDouble(values, "mean_offset_ms", &mean_offset_ms) &&
         lookup(values, "mean_offset_ms") == expected_mean_offset_ms &&
         pps_jitter_ms >= 0.0 &&
         lookup(values, "wiring_confirmation") == "PASS" &&
         lookup(values, "wiring_confirmation_overall") == "PASS" &&
         lookup(values, "wiring_confirmation_keys_status") == "PASS" &&
         lookup(values, "wiring_confirmation_source") == "manual_file" &&
         lookup(values, "pps_wiring_verified") == "PASS" &&
         lookup(values, "ptp_wiring_verified") == "PASS" &&
         validEvidenceTextValue(parsed_wiring_verified_by) &&
         validIso8601SecondsTimestamp(parsed_wiring_verified_at);
}

bool runtimeHealthPassed(const std::string& text,
                         const std::string& expected_runtime_dir,
                         std::string* timestamp,
                         std::string* disk_available_gb,
                         std::string* runtime_pid,
                         std::string* systemd_active,
                         std::string* docker_container_status) {
  bool record_valid = true;
  const std::map<std::string, std::string> values =
      parseKeyValueLines(text, &record_valid);
  if (!record_valid || hasDuplicateKeyValue(values)) {
    return false;
  }
  const std::string parsed_disk_available_gb =
      lookup(values, "disk_available_gb");
  const std::string parsed_timestamp = lookup(values, "timestamp");
  const std::string parsed_runtime_pid = lookup(values, "runtime_pid");
  const std::string parsed_systemd_active = lookup(values, "systemd_active");
  const std::string parsed_systemd_active_source =
      lookup(values, "systemd_active_source");
  const std::string parsed_docker_container_status =
      lookup(values, "docker_container_status");
  const std::string parsed_docker_container_status_source =
      lookup(values, "docker_container_status_source");
  if (timestamp != nullptr) {
    *timestamp = parsed_timestamp;
  }
  if (disk_available_gb != nullptr) {
    *disk_available_gb = parsed_disk_available_gb;
  }
  if (runtime_pid != nullptr) {
    *runtime_pid = parsed_runtime_pid;
  }
  if (systemd_active != nullptr) {
    *systemd_active = parsed_systemd_active;
  }
  if (docker_container_status != nullptr) {
    *docker_container_status = parsed_docker_container_status;
  }
  double disk_available_gb_value = 0.0;
  long runtime_pid_value = 0;
  return validIso8601SecondsTimestamp(parsed_timestamp) &&
         validRuntimeDirPath(lookup(values, "runtime_dir")) &&
         (expected_runtime_dir.empty() ||
          lookup(values, "runtime_dir") == expected_runtime_dir) &&
         lookupStrictDouble(values, "disk_available_gb",
                            &disk_available_gb_value) &&
         disk_available_gb_value >= 0.0 &&
         parseStrictPositiveIntegerValue(lookup(values, "runtime_pid"),
                                         &runtime_pid_value) &&
         lookup(values, "systemd_active") == "active" &&
         parsed_systemd_active_source == "systemctl" &&
         lookup(values, "docker_container_status") == "running" &&
         parsed_docker_container_status_source == "docker_inspect";
}

bool runtimeHealthReportReferencePassedNearCsv(
    const std::string& value,
    const std::string& csv_path,
    const std::string& base_dir,
    const std::string& expected_runtime_dir) {
  std::string resolved_path;
  if (validBundleRelativePath(value)) {
    resolved_path = joinPath(parentDir(csv_path), value);
  } else if (validAbsolutePath(value)) {
    resolved_path = value;
  } else {
    return false;
  }
  return pathWithinBaseDir(resolved_path, base_dir) &&
         fileExists(resolved_path) &&
         runtimeHealthPassed(readTextFile(resolved_path),
                             expected_runtime_dir,
                             nullptr,
                             nullptr,
                             nullptr,
                             nullptr,
                             nullptr);
}

bool runtimeDeploymentPassed(const std::string& text,
                             const std::string& expected_runtime_dir,
                             std::string* timestamp) {
  bool record_valid = true;
  const std::map<std::string, std::string> values =
      parseKeyValueLines(text, &record_valid);
  if (!record_valid || hasDuplicateKeyValue(values)) {
    return false;
  }
  const std::string parsed_timestamp = lookup(values, "timestamp");
  if (timestamp != nullptr) {
    *timestamp = parsed_timestamp;
  }
  return validIso8601SecondsTimestamp(parsed_timestamp) &&
         validRuntimeDirPath(lookup(values, "runtime_dir")) &&
         (expected_runtime_dir.empty() ||
          lookup(values, "runtime_dir") == expected_runtime_dir) &&
         lookup(values, "systemd_unit_file") == "PASS" &&
         lookup(values, "systemd_env_file") == "PASS" &&
         lookup(values, "systemd_active") == "active" &&
         lookup(values, "systemd_active_source") == "systemctl" &&
         lookup(values, "docker_compose_file") == "PASS" &&
         lookup(values, "docker_env_file") == "PASS" &&
         lookup(values, "docker_container_status") == "running" &&
         lookup(values, "docker_container_status_source") == "docker_inspect" &&
         lookup(values, "start_command") == "PASS" &&
         lookup(values, "runtime_process_status") == "PASS" &&
         lookup(values, "deployment_status") == "PASS";
}

bool runtimeStabilityPassed(const std::string& text,
                            const long* expected_sample_count,
                            long* summary_sample_count,
                            long* summary_interval_s,
                            std::string* summary_timestamp = nullptr) {
  bool record_valid = true;
  const std::map<std::string, std::string> values =
      parseKeyValueLines(text, &record_valid);
  if (!record_valid || hasDuplicateKeyValue(values)) {
    return false;
  }
  long samples = 0;
  long interval_s = 0;
  const std::string timestamp = lookup(values, "timestamp");
  const bool passed =
      validIso8601SecondsTimestamp(timestamp) &&
      lookup(values, "overall") == "PASS" &&
      parseStrictPositiveIntegerValue(lookup(values, "samples"), &samples) &&
      parseStrictPositiveIntegerValue(lookup(values, "interval_s"),
                                      &interval_s) &&
      (expected_sample_count == nullptr || samples == *expected_sample_count) &&
      summaryFailureCounterClear(values, "disk_failures") &&
      summaryFailureCounterClear(values, "watchdog_failures") &&
      summaryFailureCounterClear(values, "watchdog_skipped") &&
      summaryFailureCounterClear(values, "health_failures");
  if (passed && summary_sample_count != nullptr) {
    *summary_sample_count = samples;
  }
  if (passed && summary_interval_s != nullptr) {
    *summary_interval_s = interval_s;
  }
  if (passed && summary_timestamp != nullptr) {
    *summary_timestamp = timestamp;
  }
  return passed;
}

bool runtimeStabilityRunLogPassed(const std::string& text,
                                  const std::string& expected_runtime_dir,
                                  const long expected_sample_count,
                                  const long expected_interval_s,
                                  RuntimeStabilityRunLogFields* fields = nullptr) {
  bool record_valid = true;
  const std::map<std::string, std::string> values =
      parseKeyValueLines(text, &record_valid);
  if (!record_valid || hasDuplicateKeyValue(values)) {
    return false;
  }
  const std::string started_at = lookup(values, "started_at");
  const std::string finished_at = lookup(values, "finished_at");
  const std::string runtime_dir = lookup(values, "runtime_dir");
  const std::string samples_text = lookup(values, "samples");
  const std::string interval_text = lookup(values, "interval");
  const std::string exit_status = lookup(values, "exit_status");
  const std::string capture_exit_status = lookup(values, "capture_exit_status");
  long samples = 0;
  long interval_s = 0;
  const bool passed =
      validIso8601SecondsTimestamp(started_at) &&
      validIso8601SecondsTimestamp(finished_at) &&
      iso8601SecondsFinishedNotBeforeStarted(started_at, finished_at) &&
      validRuntimeDirPath(runtime_dir) &&
         (expected_runtime_dir.empty() ||
          runtime_dir == expected_runtime_dir) &&
         parseStrictPositiveIntegerValue(samples_text, &samples) &&
         samples == expected_sample_count &&
         parseStrictPositiveIntegerValue(interval_text, &interval_s) &&
         interval_s == expected_interval_s &&
         runtimeStabilityRunLogElapsedCoversDuration(started_at,
                                                     finished_at,
                                                     samples,
                                                     interval_s) &&
         lookupStrictZeroInteger(values, "exit_status") &&
         lookupStrictZeroInteger(values, "capture_exit_status");
  if (passed && fields != nullptr) {
    fields->started_at = started_at;
    fields->finished_at = finished_at;
    fields->runtime_dir = runtime_dir;
    fields->samples = samples_text;
    fields->interval = interval_text;
    fields->exit_status = exit_status;
    fields->capture_exit_status = capture_exit_status;
  }
  return passed;
}

bool powerLossResumePassed(const std::string& text,
                           const std::string& expected_metrics_report_path,
                           const std::string& expected_session_id,
                           const std::string& expected_scenario,
                           const std::string& base_dir,
                           std::string* power_loss_resume_source = nullptr,
                           std::string* recovery_time_s_text = nullptr,
                           std::string* max_recovery_time_s_text = nullptr,
                           std::string* confirmation_overall = nullptr,
                           std::string* resume_verified_by = nullptr,
                           std::string* resume_verified_at = nullptr,
                           const bool require_confirmation_keys_status = false,
                           std::string* power_loss_resume_timestamp = nullptr) {
  bool record_valid = true;
  const std::map<std::string, std::string> values =
      parseKeyValueLines(text, &record_valid);
  if (!record_valid || hasDuplicateKeyValue(values)) {
    return false;
  }
  const std::string parsed_source = lookup(values, "power_loss_resume_source");
  const std::string parsed_timestamp = lookup(values, "timestamp");
  const std::string parsed_recovery_time_s = lookup(values, "recovery_time_s");
  const std::string parsed_max_recovery_time_s =
      lookup(values, "max_recovery_time_s");
  const std::string parsed_confirmation_overall =
      lookup(values, "power_loss_resume_confirmation_overall").empty()
          ? "missing"
          : lookup(values, "power_loss_resume_confirmation_overall");
  const std::string parsed_resume_verified_by =
      lookup(values, "resume_verified_by");
  const std::string parsed_resume_verified_at =
      lookup(values, "resume_verified_at");
  if (power_loss_resume_source != nullptr) {
    *power_loss_resume_source = parsed_source;
  }
  if (recovery_time_s_text != nullptr) {
    *recovery_time_s_text = parsed_recovery_time_s;
  }
  if (max_recovery_time_s_text != nullptr) {
    *max_recovery_time_s_text = parsed_max_recovery_time_s;
  }
  if (confirmation_overall != nullptr) {
    *confirmation_overall = parsed_confirmation_overall;
  }
  if (resume_verified_by != nullptr) {
    *resume_verified_by = parsed_resume_verified_by;
  }
  if (resume_verified_at != nullptr) {
    *resume_verified_at = parsed_resume_verified_at;
  }
  if (power_loss_resume_timestamp != nullptr) {
    *power_loss_resume_timestamp = parsed_timestamp;
  }
  if (!validIso8601SecondsTimestamp(parsed_timestamp)) {
    return false;
  }
  if (lookup(values, "power_loss_resume_status") != "PASS") {
    return false;
  }
  if (parsed_source != "manual_file" && parsed_source != "metrics_report") {
    return false;
  }
  if (parsed_source == "metrics_report") {
    const std::string metrics_report = lookup(values, "metrics_report");
    if (metrics_report.empty() || metrics_report == "missing") {
      return false;
    }
    if (expected_metrics_report_path.empty() ||
        joinPath(base_dir, metrics_report) != expected_metrics_report_path) {
      return false;
    }
  }
  if (parsed_source == "manual_file" &&
      parsed_confirmation_overall != "PASS") {
    return false;
  }
  if (parsed_source == "manual_file" &&
      require_confirmation_keys_status &&
      lookup(values, "power_loss_resume_confirmation_keys_status") != "PASS") {
    return false;
  }
  double recovery_time_s = 0.0;
  double max_recovery_time_s = 0.0;
  if (!parseStrictDoubleValue(parsed_recovery_time_s, &recovery_time_s) ||
      !parseStrictDoubleValue(parsed_max_recovery_time_s,
                              &max_recovery_time_s)) {
    return false;
  }
  if (parsed_source == "metrics_report") {
    const std::string metrics_report_text =
        readTextFile(expected_metrics_report_path);
    if (!metricsReportPassed(metrics_report_text,
                             expected_session_id,
                             expected_scenario) ||
        !metricsReportRecoveryTimeMatches(metrics_report_text,
                                          expected_session_id,
                                          expected_scenario,
                                          recovery_time_s)) {
      return false;
    }
  }
  if (parsed_source == "manual_file" &&
      (!validEvidenceTextValue(parsed_resume_verified_by) ||
       !validIso8601SecondsTimestamp(parsed_resume_verified_at))) {
    return false;
  }
  return max_recovery_time_s > 0.0 && recovery_time_s >= 0.0 &&
         recovery_time_s <= max_recovery_time_s;
}

bool runtimeStabilityCsvPassed(const std::string& text,
                               const std::string& csv_path,
                               const std::string& base_dir,
                               const std::string& expected_runtime_dir,
                               long* sample_count,
                               std::string* first_timestamp = nullptr,
                               std::string* last_timestamp = nullptr) {
  std::stringstream stream(text);
  std::string line;
  bool header_seen = false;
  bool data_row_seen = false;
  long rows = 0;
  long long previous_sample_seconds = 0;
  bool previous_sample_seen = false;
  std::string parsed_first_timestamp;
  std::string parsed_last_timestamp;
  while (std::getline(stream, line)) {
    if (trim(line).empty()) {
      continue;
    }
    if (!header_seen) {
      header_seen =
          line == "sample,timestamp,disk_guard_status,watchdog_status,health_report";
      if (!header_seen) {
        return false;
      }
      continue;
    }
    std::stringstream row(line);
    std::string field;
    std::vector<std::string> fields;
    while (std::getline(row, field, ',')) {
      if (field.empty() || field != trim(field)) {
        return false;
      }
      fields.push_back(field);
    }
    if (fields.size() != 5) {
      return false;
    }
    long sample_index = 0;
    long long sample_seconds = 0;
    if (!parseStrictPositiveIntegerValue(fields[0], &sample_index) ||
        sample_index != rows + 1 ||
        !parseIso8601SecondsTimestampToUnixSeconds(fields[1],
                                                   &sample_seconds) ||
        (previous_sample_seen &&
         sample_seconds < previous_sample_seconds) ||
        fields[2] != "PASS" || fields[3] != "PASS" ||
        !validEvidenceTextValue(fields[4]) ||
        !runtimeHealthReportReferencePassedNearCsv(fields[4],
                                                   csv_path,
                                                   base_dir,
                                                   expected_runtime_dir)) {
      return false;
    }
    if (parsed_first_timestamp.empty()) {
      parsed_first_timestamp = fields[1];
    }
    parsed_last_timestamp = fields[1];
    previous_sample_seconds = sample_seconds;
    previous_sample_seen = true;
    data_row_seen = true;
    ++rows;
  }
  if (data_row_seen && sample_count != nullptr) {
    *sample_count = rows;
  }
  if (data_row_seen && first_timestamp != nullptr) {
    *first_timestamp = parsed_first_timestamp;
  }
  if (data_row_seen && last_timestamp != nullptr) {
    *last_timestamp = parsed_last_timestamp;
  }
  return data_row_seen;
}

bool runtimeStabilityCsvTimestampsWithinRunLog(
    const std::string& text,
    const RuntimeStabilityRunLogFields& run_log_fields) {
  long long started_seconds = 0;
  long long finished_seconds = 0;
  if (!parseIso8601SecondsTimestampToUnixSeconds(run_log_fields.started_at,
                                                 &started_seconds) ||
      !parseIso8601SecondsTimestampToUnixSeconds(run_log_fields.finished_at,
                                                 &finished_seconds) ||
      finished_seconds < started_seconds) {
    return false;
  }
  std::stringstream stream(text);
  std::string line;
  bool header_seen = false;
  bool data_row_seen = false;
  while (std::getline(stream, line)) {
    if (trim(line).empty()) {
      continue;
    }
    if (!header_seen) {
      header_seen =
          line == "sample,timestamp,disk_guard_status,watchdog_status,health_report";
      if (!header_seen) {
        return false;
      }
      continue;
    }
    std::stringstream row(line);
    std::string field;
    std::vector<std::string> fields;
    while (std::getline(row, field, ',')) {
      fields.push_back(field);
    }
    if (fields.size() != 5) {
      return false;
    }
    long long sample_seconds = 0;
    if (!parseIso8601SecondsTimestampToUnixSeconds(fields[1],
                                                   &sample_seconds) ||
        sample_seconds < started_seconds ||
        sample_seconds > finished_seconds) {
      return false;
    }
    data_row_seen = true;
  }
  return data_row_seen;
}

bool sectionExportPassed(const std::string& text,
                         const std::string& expected_session_id) {
  std::stringstream stream(text);
  std::string line;
  bool header_seen = false;
  bool data_row_seen = false;
  while (std::getline(stream, line)) {
    if (trim(line).empty()) {
      continue;
    }
    if (!header_seen) {
      header_seen =
          line == "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points";
      if (!header_seen) {
        return false;
      }
      continue;
    }
    std::stringstream row(line);
    std::string field;
    std::vector<std::string> fields;
    while (std::getline(row, field, ',')) {
      if (field.empty() || field != trim(field)) {
        return false;
      }
      fields.push_back(field);
    }
    if (fields.size() != 7) {
      return false;
    }
    if (expected_session_id.empty() || fields[0] != expected_session_id) {
      return false;
    }
    if (!validSectionStateSource(fields[2])) {
      return false;
    }
    if (fields[3] != "A" && fields[3] != "B" && fields[3] != "C") {
      return false;
    }
    double chainage_m = 0.0;
    double completeness = 0.0;
    double rmse_mm = 0.0;
    if (!parseStrictDoubleValue(fields[1], &chainage_m) ||
        !parseStrictDoubleValue(fields[4], &completeness) ||
        !parseStrictDoubleValue(fields[5], &rmse_mm) ||
        !parseStrictPositiveIntegerValue(fields[6])) {
      return false;
    }
    if (completeness < 0.0 || completeness > 1.0 || rmse_mm < 0.0) {
      return false;
    }
    data_row_seen = true;
  }
  return data_row_seen;
}

bool validSectionStateSource(const std::string& state_source) {
  return state_source == "IDLE_STATIC" ||
         state_source == "CUTTING_STATIC" ||
         state_source == "FWD_MOVE" ||
         state_source == "REV_MOVE" ||
         state_source == "TURNING" ||
         state_source == "CMD_MOVE_NO_DISP" ||
         state_source == "CONFLICT" ||
         state_source == "RELOCALIZING";
}

double lookupDouble(const std::map<std::string, std::string>& values,
                    const std::string& key) {
  const std::string value = lookup(values, key);
  if (value.empty()) {
    return 0.0;
  }
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  return end == value.c_str() ? 0.0 : parsed;
}

bool lookupStrictDouble(const std::map<std::string, std::string>& values,
                        const std::string& key,
                        double* parsed_value) {
  return parseStrictDoubleValue(lookup(values, key), parsed_value);
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

bool parseStrictPositiveIntegerValue(const std::string& value,
                                     long* parsed_value) {
  if (value.empty() || value[0] < '1' || value[0] > '9') {
    return false;
  }
  for (std::size_t i = 1; i < value.size(); ++i) {
    if (value[i] < '0' || value[i] > '9') {
      return false;
    }
  }
  char* end = nullptr;
  errno = 0;
  const long parsed = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0' || errno == ERANGE ||
      parsed <= 0) {
    return false;
  }
  if (parsed_value != nullptr) {
    *parsed_value = parsed;
  }
  return true;
}

bool parseStrictPositiveIntegerValue(const std::string& value) {
  return parseStrictPositiveIntegerValue(value, nullptr);
}

bool parseStrictNonnegativeIntegerValue(const std::string& value,
                                        long long* parsed_value) {
  if (value.empty()) {
    return false;
  }
  if (value == "0") {
    if (parsed_value != nullptr) {
      *parsed_value = 0;
    }
    return true;
  }
  if (value[0] < '1' || value[0] > '9') {
    return false;
  }
  for (std::size_t i = 1; i < value.size(); ++i) {
    if (value[i] < '0' || value[i] > '9') {
      return false;
    }
  }
  char* end = nullptr;
  errno = 0;
  const long long parsed = std::strtoll(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0' || errno == ERANGE ||
      parsed < 0) {
    return false;
  }
  if (parsed_value != nullptr) {
    *parsed_value = parsed;
  }
  return true;
}

bool lookupStrictPositiveInteger(const std::map<std::string, std::string>& values,
                                 const std::string& key) {
  return parseStrictPositiveIntegerValue(lookup(values, key));
}

bool summaryFailureCounterClear(const std::map<std::string, std::string>& values,
                                const std::string& key) {
  return lookup(values, key) == "0";
}

bool lookupStrictZeroInteger(const std::map<std::string, std::string>& values,
                             const std::string& key) {
  return lookup(values, key) == "0";
}

bool fieldAcceptancePassed(const std::string& text,
                           const long* expected_csv_sample_count,
                           const long* expected_summary_sample_count,
                           const long* expected_summary_interval_s,
                           const std::string& expected_event_file_path,
                           const std::string& expected_metrics_report_path,
                           const std::string& expected_section_export_path,
                           const std::string& expected_runtime_health_path,
                           const std::string& expected_runtime_deployment_path,
                           const std::string& expected_runtime_stability_csv_path,
                           const std::string& expected_runtime_stability_csv_first_timestamp,
                           const std::string& expected_runtime_stability_csv_last_timestamp,
                           const std::string& expected_runtime_stability_summary_path,
                           const std::string& expected_runtime_stability_run_log_path,
                           const std::string& expected_runtime_stability_summary_timestamp,
                           const std::string& expected_power_loss_resume_path,
                           const std::string& expected_pps_ptp_wiring_path,
                           const std::string& expected_session_id,
                           const std::string& expected_scenario,
                           const std::string& expected_runtime_dir,
                           const std::string& expected_time_sync_report_path,
                           const std::string& expected_time_sync_raw,
                           const std::string& expected_time_status_topic,
                           const std::string& expected_pps_topic,
                           const std::string& expected_pps_jitter_ms,
                           const std::string& expected_mean_offset_ms,
                           const std::string& expected_time_sync_timestamp,
                           const std::string& expected_pps_ptp_wiring_timestamp,
                           const std::string& expected_power_loss_resume_timestamp,
                           const std::string& expected_runtime_deployment_timestamp,
                           const std::string& expected_runtime_health_timestamp,
                           const std::string& expected_runtime_health_disk_gb,
                           const std::string& expected_runtime_health_pid,
                           const std::string& expected_runtime_health_systemd,
                           const std::string& expected_runtime_health_docker,
                           const std::string& expected_power_loss_source,
                           const std::string& expected_recovery_time_s,
                           const std::string& expected_max_recovery_time_s,
                           const std::string& expected_resume_confirmation,
                           const std::string& expected_resume_verified_by,
                           const std::string& expected_resume_verified_at,
                           const std::string& expected_wiring_verified_by,
                           const std::string& expected_wiring_verified_at,
                           const RuntimeStabilityRunLogFields* expected_run_log,
                           const std::string& base_dir) {
  bool record_valid = true;
  const std::map<std::string, std::string> values =
      parseKeyValueLines(text, &record_valid);
  if (!record_valid || hasDuplicateKeyValue(values)) {
    return false;
  }
  const std::string field_resume_confirmation =
      lookup(values, "power_loss_resume_confirmation_overall").empty()
          ? "missing"
          : lookup(values, "power_loss_resume_confirmation_overall");
  double pps_jitter_ms = 0.0;
  double mean_offset_ms = 0.0;
  double pps_ptp_wiring_pps_jitter_ms = 0.0;
  double pps_ptp_wiring_mean_offset_ms = 0.0;
  double runtime_stability_duration_h = 0.0;
  double runtime_stability_min_duration_h = 0.0;
  double runtime_health_disk_available_gb = 0.0;
  long long expected_run_log_elapsed_s = 0;
  long long expected_run_log_required_elapsed_s = 0;
  long long runtime_stability_run_log_elapsed_s = 0;
  long long runtime_stability_run_log_required_elapsed_s = 0;
  long runtime_stability_csv_samples = 0;
  long runtime_stability_run_log_interval_s = 0;
  long runtime_stability_run_log_samples = 0;
  long runtime_stability_samples = 0;
  long runtime_stability_interval_s = 0;
  long runtime_health_pid = 0;
  return validIso8601SecondsTimestamp(lookup(values, "timestamp")) &&
         lookup(values, "field_acceptance_timestamp_status") == "PASS" &&
         fieldAcceptanceTimestampCoversEvidence(values,
                                                expected_power_loss_source) &&
         lookup(values, "field_acceptance_status") == "PASS" &&
         lookup(values, "session_id") == expected_session_id &&
         lookup(values, "scenario") == expected_scenario &&
         lookup(values, "event_file_status") == "PASS" &&
         evidencePathReferenceMatches(lookup(values, "event_file_report"),
                                      expected_event_file_path,
                                      base_dir) &&
         lookup(values, "time_sync_status") == "PASS" &&
         lookup(values, "time_sync_keys_status") == "PASS" &&
         evidencePathReferenceMatches(lookup(values, "time_sync_report"),
                                      expected_time_sync_report_path,
                                      base_dir) &&
         validIso8601SecondsTimestamp(
             lookup(values, "time_sync_timestamp")) &&
         lookup(values, "time_sync_timestamp") ==
             expected_time_sync_timestamp &&
         validEvidenceTextValue(lookup(values, "time_sync_raw")) &&
         lookup(values, "time_sync_raw") == expected_time_sync_raw &&
         lookup(values, "time_sync_raw_status") == "PASS" &&
         validRosTopicName(lookup(values, "time_status_topic")) &&
         lookup(values, "time_status_topic") == expected_time_status_topic &&
         validRosTopicName(lookup(values, "pps_topic")) &&
         lookup(values, "pps_topic") == expected_pps_topic &&
         lookup(values, "time_capture_status") == "CAPTURED" &&
         lookup(values, "time_pps_status") == "PASS" &&
         lookup(values, "time_clock_offset_status") == "PASS" &&
         lookupStrictDouble(values, "pps_jitter_ms", &pps_jitter_ms) &&
         lookup(values, "pps_jitter_ms") == expected_pps_jitter_ms &&
         lookupStrictDouble(values, "mean_offset_ms", &mean_offset_ms) &&
         lookup(values, "mean_offset_ms") == expected_mean_offset_ms &&
         pps_jitter_ms >= 0.0 &&
         lookupStrictDouble(values, "runtime_stability_duration_h",
                            &runtime_stability_duration_h) &&
         runtime_stability_duration_h >= 24.0 &&
         lookup(values, "runtime_stability_min_duration_status") == "PASS" &&
         lookupStrictDouble(values, "runtime_stability_min_duration_h",
                            &runtime_stability_min_duration_h) &&
         runtime_stability_min_duration_h >= 24.0 &&
         runtime_stability_duration_h >= runtime_stability_min_duration_h &&
         lookup(values, "runtime_health_status") == "PASS" &&
         lookup(values, "runtime_health_keys_status") == "PASS" &&
         evidencePathReferenceMatches(lookup(values, "runtime_health_report"),
                                      expected_runtime_health_path,
                                      base_dir) &&
         validIso8601SecondsTimestamp(
             lookup(values, "runtime_health_timestamp")) &&
         lookup(values, "runtime_health_timestamp") ==
             expected_runtime_health_timestamp &&
         validRuntimeDirPath(lookup(values, "runtime_health_runtime_dir")) &&
         (expected_runtime_dir.empty() ||
          lookup(values, "runtime_health_runtime_dir") == expected_runtime_dir) &&
         lookupStrictDouble(values, "runtime_health_disk_available_gb",
                            &runtime_health_disk_available_gb) &&
         lookup(values, "runtime_health_disk_available_gb") ==
             expected_runtime_health_disk_gb &&
         runtime_health_disk_available_gb >= 0.0 &&
         parseStrictPositiveIntegerValue(
             lookup(values, "runtime_health_pid"), &runtime_health_pid) &&
         lookup(values, "runtime_health_pid") == expected_runtime_health_pid &&
         lookup(values, "runtime_health_systemd_active") == "active" &&
         lookup(values, "runtime_health_systemd_active") ==
             expected_runtime_health_systemd &&
         lookup(values, "runtime_health_systemd_active_source") ==
             "systemctl" &&
         lookup(values, "runtime_health_docker_container_status") ==
             "running" &&
         lookup(values, "runtime_health_docker_container_status") ==
             expected_runtime_health_docker &&
         lookup(values, "runtime_health_docker_container_status_source") ==
             "docker_inspect" &&
         lookup(values, "section_export_status") == "PASS" &&
         evidencePathReferenceMatches(lookup(values, "section_export_report"),
                                      expected_section_export_path,
                                      base_dir) &&
         !lookup(values, "metrics_report").empty() &&
         lookup(values, "metrics_report") != "missing" &&
         joinPath(base_dir, lookup(values, "metrics_report")) ==
             expected_metrics_report_path &&
         lookup(values, "metrics_status") == "PASS" &&
         lookup(values, "runtime_deployment_status") == "PASS" &&
         lookup(values, "runtime_deployment_keys_status") == "PASS" &&
         evidencePathReferenceMatches(
             lookup(values, "runtime_deployment_report"),
             expected_runtime_deployment_path,
             base_dir) &&
         validIso8601SecondsTimestamp(
             lookup(values, "runtime_deployment_timestamp")) &&
         lookup(values, "runtime_deployment_timestamp") ==
             expected_runtime_deployment_timestamp &&
         lookup(values, "deployment_overall") == "PASS" &&
         lookup(values, "deployment_status") == "PASS" &&
         lookup(values, "systemd_unit_file") == "PASS" &&
         lookup(values, "systemd_env_file") == "PASS" &&
         lookup(values, "docker_compose_file") == "PASS" &&
         lookup(values, "docker_env_file") == "PASS" &&
         lookup(values, "start_command") == "PASS" &&
         lookup(values, "runtime_process_status") == "PASS" &&
         validRuntimeDirPath(lookup(values, "deployment_runtime_dir")) &&
         (expected_runtime_dir.empty() ||
          lookup(values, "deployment_runtime_dir") == expected_runtime_dir) &&
         lookup(values, "runtime_stability_status") == "PASS" &&
         evidencePathReferenceMatches(
             lookup(values, "runtime_stability_csv_report"),
             expected_runtime_stability_csv_path,
             base_dir) &&
         validIso8601SecondsTimestamp(
             lookup(values, "runtime_stability_csv_first_timestamp")) &&
         lookup(values, "runtime_stability_csv_first_timestamp") ==
             expected_runtime_stability_csv_first_timestamp &&
         validIso8601SecondsTimestamp(
             lookup(values, "runtime_stability_csv_last_timestamp")) &&
         lookup(values, "runtime_stability_csv_last_timestamp") ==
             expected_runtime_stability_csv_last_timestamp &&
         iso8601SecondsFinishedNotBeforeStarted(
             lookup(values, "runtime_stability_csv_first_timestamp"),
             lookup(values, "runtime_stability_csv_last_timestamp")) &&
         evidencePathReferenceMatches(
             lookup(values, "runtime_stability_summary_report"),
             expected_runtime_stability_summary_path,
             base_dir) &&
         evidencePathReferenceMatches(
             lookup(values, "runtime_stability_run_log_report"),
             expected_runtime_stability_run_log_path,
             base_dir) &&
         validIso8601SecondsTimestamp(
             lookup(values, "runtime_stability_summary_timestamp")) &&
         lookup(values, "runtime_stability_summary_timestamp") ==
             expected_runtime_stability_summary_timestamp &&
         lookup(values, "runtime_stability_overall") == "PASS" &&
         lookup(values, "runtime_stability_summary_keys_status") == "PASS" &&
         lookup(values, "runtime_stability_csv_status") == "PASS" &&
         parseStrictPositiveIntegerValue(
             lookup(values, "runtime_stability_csv_samples"),
             &runtime_stability_csv_samples) &&
         (expected_csv_sample_count == nullptr ||
          runtime_stability_csv_samples == *expected_csv_sample_count) &&
         lookup(values, "runtime_stability_sample_count_match") == "PASS" &&
         lookup(values, "runtime_stability_run_log_status") == "PASS" &&
         lookup(values, "runtime_stability_run_log_keys_status") == "PASS" &&
         validIso8601SecondsTimestamp(
             lookup(values, "runtime_stability_run_log_started_at")) &&
         (expected_run_log == nullptr ||
          lookup(values, "runtime_stability_run_log_started_at") ==
              expected_run_log->started_at) &&
         validIso8601SecondsTimestamp(
             lookup(values, "runtime_stability_run_log_finished_at")) &&
         iso8601SecondsFinishedNotBeforeStarted(
             lookup(values, "runtime_stability_run_log_started_at"),
             lookup(values, "runtime_stability_run_log_finished_at")) &&
         (expected_run_log == nullptr ||
         lookup(values, "runtime_stability_run_log_finished_at") ==
             expected_run_log->finished_at) &&
         iso8601SecondsFinishedNotBeforeStarted(
             lookup(values, "runtime_stability_run_log_started_at"),
             lookup(values, "runtime_stability_csv_first_timestamp")) &&
         iso8601SecondsFinishedNotBeforeStarted(
             lookup(values, "runtime_stability_csv_last_timestamp"),
             lookup(values, "runtime_stability_run_log_finished_at")) &&
         iso8601SecondsFinishedNotBeforeStarted(
             lookup(values, "runtime_stability_run_log_started_at"),
             lookup(values, "runtime_stability_summary_timestamp")) &&
         iso8601SecondsFinishedNotBeforeStarted(
             lookup(values, "runtime_stability_summary_timestamp"),
             lookup(values, "runtime_stability_run_log_finished_at")) &&
         validRuntimeDirPath(
             lookup(values, "runtime_stability_run_log_runtime_dir")) &&
         (expected_runtime_dir.empty() ||
          lookup(values, "runtime_stability_run_log_runtime_dir") ==
              expected_runtime_dir) &&
         (expected_run_log == nullptr ||
          lookup(values, "runtime_stability_run_log_runtime_dir") ==
              expected_run_log->runtime_dir) &&
         parseStrictPositiveIntegerValue(
             lookup(values, "runtime_stability_run_log_samples"),
             &runtime_stability_run_log_samples) &&
         (expected_summary_sample_count == nullptr ||
          runtime_stability_run_log_samples == *expected_summary_sample_count) &&
         (expected_run_log == nullptr ||
          lookup(values, "runtime_stability_run_log_samples") ==
              expected_run_log->samples) &&
         parseStrictPositiveIntegerValue(
             lookup(values, "runtime_stability_run_log_interval"),
             &runtime_stability_run_log_interval_s) &&
         (expected_summary_interval_s == nullptr ||
          runtime_stability_run_log_interval_s ==
              *expected_summary_interval_s) &&
         (expected_run_log == nullptr ||
          lookup(values, "runtime_stability_run_log_interval") ==
              expected_run_log->interval) &&
         runtimeStabilityRunLogElapsedAndRequired(
             lookup(values, "runtime_stability_run_log_started_at"),
             lookup(values, "runtime_stability_run_log_finished_at"),
             runtime_stability_run_log_samples,
             runtime_stability_run_log_interval_s,
             &expected_run_log_elapsed_s,
             &expected_run_log_required_elapsed_s) &&
         parseStrictNonnegativeIntegerValue(
             lookup(values, "runtime_stability_run_log_elapsed_s"),
             &runtime_stability_run_log_elapsed_s) &&
         runtime_stability_run_log_elapsed_s == expected_run_log_elapsed_s &&
         parseStrictNonnegativeIntegerValue(
             lookup(values, "runtime_stability_run_log_required_elapsed_s"),
             &runtime_stability_run_log_required_elapsed_s) &&
         runtime_stability_run_log_required_elapsed_s ==
             expected_run_log_required_elapsed_s &&
         lookup(values, "runtime_stability_run_log_duration_status") == "PASS" &&
         runtime_stability_run_log_elapsed_s >=
             runtime_stability_run_log_required_elapsed_s &&
         lookupStrictZeroInteger(values,
                                 "runtime_stability_run_log_exit_status") &&
         (expected_run_log == nullptr ||
          lookup(values, "runtime_stability_run_log_exit_status") ==
              expected_run_log->exit_status) &&
         lookupStrictZeroInteger(
             values, "runtime_stability_run_log_capture_exit_status") &&
         (expected_run_log == nullptr ||
          lookup(values, "runtime_stability_run_log_capture_exit_status") ==
              expected_run_log->capture_exit_status) &&
         lookup(values, "power_loss_resume_overall") == "PASS" &&
         lookup(values, "power_loss_resume_keys_status") == "PASS" &&
         evidencePathReferenceMatches(lookup(values, "power_loss_resume_report"),
                                      expected_power_loss_resume_path,
                                      base_dir) &&
         validIso8601SecondsTimestamp(
             lookup(values, "power_loss_resume_timestamp")) &&
         lookup(values, "power_loss_resume_timestamp") ==
             expected_power_loss_resume_timestamp &&
         lookup(values, "power_loss_resume_source") ==
             expected_power_loss_source &&
         lookup(values, "recovery_time_s") == expected_recovery_time_s &&
         lookup(values, "max_recovery_time_s") ==
             expected_max_recovery_time_s &&
         field_resume_confirmation == expected_resume_confirmation &&
         (expected_power_loss_source != "manual_file" ||
          lookup(values, "power_loss_resume_confirmation_keys_status") ==
              "PASS") &&
         (expected_power_loss_source != "manual_file" ||
          (lookup(values, "resume_verified_by") == expected_resume_verified_by &&
           lookup(values, "resume_verified_at") ==
               expected_resume_verified_at)) &&
         powerLossResumePassed(text,
                               expected_metrics_report_path,
                               expected_session_id,
                               expected_scenario,
                               base_dir) &&
         lookup(values, "pps_ptp_wiring_verified") == "PASS" &&
         lookup(values, "pps_ptp_wiring_keys_status") == "PASS" &&
         evidencePathReferenceMatches(lookup(values, "pps_ptp_wiring_report"),
                                      expected_pps_ptp_wiring_path,
                                      base_dir) &&
         validIso8601SecondsTimestamp(
             lookup(values, "pps_ptp_wiring_timestamp")) &&
         lookup(values, "pps_ptp_wiring_timestamp") ==
             expected_pps_ptp_wiring_timestamp &&
         lookup(values, "pps_ptp_wiring_overall") == "PASS" &&
         lookup(values, "pps_ptp_wiring_time_sync_status") == "PASS" &&
         lookup(values, "pps_ptp_wiring_capture_status") == "CAPTURED" &&
         validRosTopicName(lookup(values, "pps_ptp_wiring_time_status_topic")) &&
         lookup(values, "pps_ptp_wiring_time_status_topic") ==
             expected_time_status_topic &&
         validRosTopicName(lookup(values, "pps_ptp_wiring_pps_topic")) &&
         lookup(values, "pps_ptp_wiring_pps_topic") == expected_pps_topic &&
         lookup(values, "pps_ptp_wiring_pps_status") == "PASS" &&
         lookup(values, "pps_ptp_wiring_clock_offset_status") == "PASS" &&
         lookupStrictDouble(values,
                            "pps_ptp_wiring_pps_jitter_ms",
                            &pps_ptp_wiring_pps_jitter_ms) &&
         lookup(values, "pps_ptp_wiring_pps_jitter_ms") ==
             expected_pps_jitter_ms &&
         pps_ptp_wiring_pps_jitter_ms >= 0.0 &&
         lookupStrictDouble(values,
                            "pps_ptp_wiring_mean_offset_ms",
                            &pps_ptp_wiring_mean_offset_ms) &&
         lookup(values, "pps_ptp_wiring_mean_offset_ms") ==
             expected_mean_offset_ms &&
         evidencePathReferenceMatches(
             lookup(values, "pps_ptp_wiring_time_sync_report"),
             expected_time_sync_report_path,
             base_dir) &&
         validEvidenceTextValue(
             lookup(values, "pps_ptp_wiring_time_sync_raw")) &&
         lookup(values, "pps_ptp_wiring_time_sync_raw") ==
             expected_time_sync_raw &&
         lookup(values, "pps_ptp_wiring_time_sync_raw_status") == "PASS" &&
         validIso8601SecondsTimestamp(
             lookup(values, "pps_ptp_wiring_time_sync_timestamp")) &&
         lookup(values, "pps_ptp_wiring_time_sync_timestamp") ==
             expected_time_sync_timestamp &&
         lookup(values, "wiring_confirmation") == "PASS" &&
         lookup(values, "wiring_confirmation_overall") == "PASS" &&
         lookup(values, "wiring_confirmation_keys_status") == "PASS" &&
         lookup(values, "wiring_confirmation_source") == "manual_file" &&
         lookup(values, "pps_wiring_verified") == "PASS" &&
         lookup(values, "ptp_wiring_verified") == "PASS" &&
         validEvidenceTextValue(lookup(values, "wiring_verified_by")) &&
         lookup(values, "wiring_verified_by") == expected_wiring_verified_by &&
         validIso8601SecondsTimestamp(lookup(values, "wiring_verified_at")) &&
         lookup(values, "wiring_verified_at") == expected_wiring_verified_at &&
         lookup(values, "systemd_active") == "active" &&
         lookup(values, "systemd_active_source") == "systemctl" &&
         lookup(values, "docker_container_status") == "running" &&
         lookup(values, "docker_container_status_source") == "docker_inspect" &&
         parseStrictPositiveIntegerValue(
             lookup(values, "runtime_stability_samples"),
             &runtime_stability_samples) &&
         (expected_summary_sample_count == nullptr ||
          runtime_stability_samples == *expected_summary_sample_count) &&
         parseStrictPositiveIntegerValue(
             lookup(values, "runtime_stability_interval_s"),
             &runtime_stability_interval_s) &&
         (expected_summary_interval_s == nullptr ||
          runtime_stability_interval_s == *expected_summary_interval_s) &&
         runtimeStabilityDurationMatchesSummary(runtime_stability_duration_h,
                                                runtime_stability_samples,
                                                runtime_stability_interval_s) &&
         (static_cast<double>(runtime_stability_samples) *
          static_cast<double>(runtime_stability_interval_s) / 3600.0) >=
             runtime_stability_duration_h &&
         lookupStrictZeroInteger(values, "runtime_stability_disk_failures") &&
         lookupStrictZeroInteger(values, "runtime_stability_watchdog_failures") &&
         lookupStrictZeroInteger(values, "runtime_stability_watchdog_skipped") &&
         lookupStrictZeroInteger(values, "runtime_stability_health_failures");
}

}  // namespace

EvidenceManifest parseEvidenceManifestRecord(const std::string& line) {
  bool record_valid = true;
  const std::map<std::string, std::string> values =
      parseKeyValuePairs(line, &record_valid);
  EvidenceManifest manifest;
  manifest.session_id = lookup(values, "session_id");
  manifest.scenario = lookup(values, "scenario");
  manifest.runtime_dir = lookup(values, "runtime_dir");
  manifest.has_duplicate_keys = hasDuplicateKeyValue(values);
  manifest.has_malformed_tokens = !record_valid;

  const char* required_keys[] = {
      "metrics_report", "event_file", "bag_file", "pcap_file",
      "tf_snapshot", "params_snapshot", "runtime_log", "time_sync",
      "pps_ptp_wiring", "runtime_health", "runtime_deployment",
      "runtime_stability_csv", "runtime_stability_summary",
      "runtime_stability_run_log", "power_loss_resume", "section_export",
      "field_acceptance"};
  for (const char* key : required_keys) {
    EvidenceFileCheck file;
    file.key = key;
    file.path = lookup(values, key);
    manifest.files.push_back(file);
  }
  return manifest;
}

EvidenceBundleReport evaluateEvidenceBundle(const EvidenceManifest& manifest,
                                             const std::string& base_dir) {
  EvidenceBundleReport report;
  report.session_id = manifest.session_id;
  report.scenario = manifest.scenario;
  report.runtime_dir = manifest.runtime_dir;
  report.checked_files = manifest.files.size();

  std::string metrics_report_path;
  std::string event_file_path;
  std::string time_sync_path;
  std::string pps_ptp_wiring_path;
  std::string runtime_health_path;
  std::string runtime_deployment_path;
  std::string runtime_stability_csv_path;
  std::string runtime_stability_summary_path;
  std::string runtime_stability_run_log_path;
  std::string power_loss_resume_path;
  std::string field_acceptance_path;
  std::string section_export_path;
  for (const EvidenceFileCheck& file : manifest.files) {
    const std::string resolved_path = joinPath(base_dir, file.path);
    const bool valid_path = validBundleRelativePath(file.path);
    if (file.key == "metrics_report") {
      if (valid_path) {
        metrics_report_path = resolved_path;
      }
    }
    if (file.key == "event_file") {
      if (valid_path) {
        event_file_path = resolved_path;
      }
    }
    if (file.key == "time_sync") {
      if (valid_path) {
        time_sync_path = resolved_path;
      }
    }
    if (file.key == "pps_ptp_wiring") {
      if (valid_path) {
        pps_ptp_wiring_path = resolved_path;
      }
    }
    if (file.key == "runtime_health") {
      if (valid_path) {
        runtime_health_path = resolved_path;
      }
    }
    if (file.key == "runtime_deployment") {
      if (valid_path) {
        runtime_deployment_path = resolved_path;
      }
    }
    if (file.key == "runtime_stability_csv") {
      if (valid_path) {
        runtime_stability_csv_path = resolved_path;
      }
    }
    if (file.key == "runtime_stability_summary") {
      if (valid_path) {
        runtime_stability_summary_path = resolved_path;
      }
    }
    if (file.key == "runtime_stability_run_log") {
      if (valid_path) {
        runtime_stability_run_log_path = resolved_path;
      }
    }
    if (file.key == "power_loss_resume") {
      if (valid_path) {
        power_loss_resume_path = resolved_path;
      }
    }
    if (file.key == "field_acceptance") {
      if (valid_path) {
        field_acceptance_path = resolved_path;
      }
    }
    if (file.key == "section_export") {
      if (valid_path) {
        section_export_path = resolved_path;
      }
      report.section_export_checked = true;
    }
    if (!valid_path || !fileExists(resolved_path)) {
      EvidenceFileCheck missing = file;
      missing.path = resolved_path;
      report.missing_files.push_back(missing);
    }
  }

  if (!metrics_report_path.empty() && fileExists(metrics_report_path)) {
    report.metrics_passed = metricsReportPassed(readTextFile(metrics_report_path),
                                                manifest.session_id,
                                                manifest.scenario);
  }
  if (!event_file_path.empty() && fileExists(event_file_path)) {
    report.event_file_passed = eventFilePassed(readTextFile(event_file_path),
                                               manifest.session_id,
                                               manifest.scenario);
  }
  std::string expected_time_status_topic;
  std::string expected_pps_topic;
  std::string expected_pps_jitter_ms;
  std::string expected_mean_offset_ms;
  std::string expected_time_sync_timestamp;
  std::string expected_time_sync_raw;
  std::string expected_pps_ptp_wiring_timestamp;
  std::string expected_wiring_verified_by;
  std::string expected_wiring_verified_at;
  if (!time_sync_path.empty() && fileExists(time_sync_path)) {
    report.time_sync_passed = timeSyncPassed(readTextFile(time_sync_path),
                                             base_dir,
                                             &expected_time_sync_timestamp,
                                             &expected_time_status_topic,
                                             &expected_pps_topic,
                                             &expected_pps_jitter_ms,
                                             &expected_mean_offset_ms,
                                             &expected_time_sync_raw);
  }
  if (!pps_ptp_wiring_path.empty() && fileExists(pps_ptp_wiring_path)) {
    report.pps_ptp_wiring_passed =
        ppsPtpWiringPassed(readTextFile(pps_ptp_wiring_path),
                            report.time_sync_passed,
                            expected_pps_jitter_ms,
                            expected_mean_offset_ms,
                            expected_time_status_topic,
                            expected_pps_topic,
                            time_sync_path,
                            expected_time_sync_raw,
                            expected_time_sync_timestamp,
                            base_dir,
                            &expected_pps_ptp_wiring_timestamp,
                            &expected_wiring_verified_by,
                            &expected_wiring_verified_at);
  }
  const std::string expected_runtime_dir =
      validRuntimeDirPath(manifest.runtime_dir) ? manifest.runtime_dir : "";
  std::string expected_runtime_health_timestamp;
  std::string expected_runtime_health_disk_gb;
  std::string expected_runtime_health_pid;
  std::string expected_runtime_health_systemd;
  std::string expected_runtime_health_docker;
  std::string expected_runtime_deployment_timestamp;
  std::string expected_power_loss_source;
  std::string expected_power_loss_recovery_time_s;
  std::string expected_power_loss_max_recovery_time_s;
  std::string expected_power_loss_confirmation;
  std::string expected_power_loss_resume_timestamp;
  std::string expected_resume_verified_by;
  std::string expected_resume_verified_at;

  if (!runtime_health_path.empty() && fileExists(runtime_health_path)) {
    report.runtime_health_passed =
        runtimeHealthPassed(readTextFile(runtime_health_path),
                            expected_runtime_dir,
                            &expected_runtime_health_timestamp,
                            &expected_runtime_health_disk_gb,
                            &expected_runtime_health_pid,
                            &expected_runtime_health_systemd,
                            &expected_runtime_health_docker);
  }
  if (!runtime_deployment_path.empty() && fileExists(runtime_deployment_path)) {
    report.runtime_deployment_passed =
        runtimeDeploymentPassed(readTextFile(runtime_deployment_path),
                                expected_runtime_dir,
                                &expected_runtime_deployment_timestamp);
  }
  long runtime_stability_sample_count = 0;
  std::string runtime_stability_csv_text;
  std::string expected_runtime_stability_csv_first_timestamp;
  std::string expected_runtime_stability_csv_last_timestamp;
  if (!runtime_stability_csv_path.empty() &&
      fileExists(runtime_stability_csv_path)) {
    runtime_stability_csv_text = readTextFile(runtime_stability_csv_path);
    report.runtime_stability_csv_passed =
        runtimeStabilityCsvPassed(runtime_stability_csv_text,
                                  runtime_stability_csv_path,
                                  base_dir,
                                  expected_runtime_dir,
                                  &runtime_stability_sample_count,
                                  &expected_runtime_stability_csv_first_timestamp,
                                  &expected_runtime_stability_csv_last_timestamp);
  }
  long runtime_stability_summary_sample_count = 0;
  long runtime_stability_summary_interval_s = 0;
  std::string expected_runtime_stability_summary_timestamp;
  RuntimeStabilityRunLogFields runtime_stability_run_log_fields;
  if (!runtime_stability_summary_path.empty() &&
      fileExists(runtime_stability_summary_path)) {
    const long* expected_sample_count =
        report.runtime_stability_csv_passed ? &runtime_stability_sample_count
                                            : nullptr;
    const bool runtime_stability_summary_passed =
        runtimeStabilityPassed(readTextFile(runtime_stability_summary_path),
                               expected_sample_count,
                               &runtime_stability_summary_sample_count,
                               &runtime_stability_summary_interval_s,
                               &expected_runtime_stability_summary_timestamp);
    if (runtime_stability_summary_passed &&
        !runtime_stability_run_log_path.empty() &&
        fileExists(runtime_stability_run_log_path)) {
      report.runtime_stability_run_log_passed =
          runtimeStabilityRunLogPassed(
              readTextFile(runtime_stability_run_log_path),
              expected_runtime_dir,
              runtime_stability_summary_sample_count,
              runtime_stability_summary_interval_s,
              &runtime_stability_run_log_fields);
    }
    if (report.runtime_stability_csv_passed &&
        report.runtime_stability_run_log_passed &&
        !runtimeStabilityCsvTimestampsWithinRunLog(
            runtime_stability_csv_text,
            runtime_stability_run_log_fields)) {
      report.runtime_stability_csv_passed = false;
    }
    report.runtime_stability_passed =
        runtime_stability_summary_passed &&
        report.runtime_stability_run_log_passed;
  }
  if (!power_loss_resume_path.empty() && fileExists(power_loss_resume_path)) {
    report.power_loss_resume_passed =
        powerLossResumePassed(readTextFile(power_loss_resume_path),
                              metrics_report_path,
                              manifest.session_id,
                              manifest.scenario,
                              base_dir,
                              &expected_power_loss_source,
                              &expected_power_loss_recovery_time_s,
                              &expected_power_loss_max_recovery_time_s,
                              &expected_power_loss_confirmation,
                              &expected_resume_verified_by,
                              &expected_resume_verified_at,
                              true,
                              &expected_power_loss_resume_timestamp);
  }
  if (!section_export_path.empty() && fileExists(section_export_path)) {
    report.section_export_passed =
        sectionExportPassed(readTextFile(section_export_path),
                            manifest.session_id);
  }
  if (!field_acceptance_path.empty() && fileExists(field_acceptance_path)) {
    const long* expected_field_csv_sample_count =
        report.runtime_stability_csv_passed ? &runtime_stability_sample_count
                                            : nullptr;
    const long* expected_field_summary_sample_count =
        report.runtime_stability_passed ? &runtime_stability_summary_sample_count
                                        : nullptr;
    const long* expected_field_summary_interval_s =
        report.runtime_stability_passed ? &runtime_stability_summary_interval_s
                                        : nullptr;
    const RuntimeStabilityRunLogFields* expected_field_run_log =
        report.runtime_stability_run_log_passed
            ? &runtime_stability_run_log_fields
            : nullptr;
    report.field_acceptance_passed =
        report.metrics_passed &&
        report.event_file_passed &&
        report.time_sync_passed &&
        report.pps_ptp_wiring_passed &&
        report.runtime_health_passed &&
        report.runtime_deployment_passed &&
        report.runtime_stability_csv_passed &&
        report.runtime_stability_passed &&
        report.power_loss_resume_passed &&
        report.section_export_checked &&
        report.section_export_passed &&
        fieldAcceptancePassed(readTextFile(field_acceptance_path),
                              expected_field_csv_sample_count,
                              expected_field_summary_sample_count,
                              expected_field_summary_interval_s,
                              event_file_path,
                              metrics_report_path,
                              section_export_path,
                              runtime_health_path,
                              runtime_deployment_path,
                              runtime_stability_csv_path,
                              expected_runtime_stability_csv_first_timestamp,
                              expected_runtime_stability_csv_last_timestamp,
                              runtime_stability_summary_path,
                              runtime_stability_run_log_path,
                              expected_runtime_stability_summary_timestamp,
                              power_loss_resume_path,
                              pps_ptp_wiring_path,
                              manifest.session_id,
                              manifest.scenario,
                              expected_runtime_dir,
                              time_sync_path,
                              expected_time_sync_raw,
                              expected_time_status_topic,
                              expected_pps_topic,
                              expected_pps_jitter_ms,
                              expected_mean_offset_ms,
                              expected_time_sync_timestamp,
                              expected_pps_ptp_wiring_timestamp,
                              expected_power_loss_resume_timestamp,
                              expected_runtime_deployment_timestamp,
                              expected_runtime_health_timestamp,
                              expected_runtime_health_disk_gb,
                              expected_runtime_health_pid,
                              expected_runtime_health_systemd,
                              expected_runtime_health_docker,
                              expected_power_loss_source,
                              expected_power_loss_recovery_time_s,
                              expected_power_loss_max_recovery_time_s,
                              expected_power_loss_confirmation,
                              expected_resume_verified_by,
                              expected_resume_verified_at,
                              expected_wiring_verified_by,
                              expected_wiring_verified_at,
                              expected_field_run_log,
                              base_dir);
  }

  report.passed = report.missing_files.empty() && report.metrics_passed &&
                  report.event_file_passed &&
                  report.time_sync_passed &&
                  report.pps_ptp_wiring_passed &&
                  report.runtime_health_passed &&
                  report.runtime_deployment_passed &&
                  report.runtime_stability_csv_passed &&
                  report.runtime_stability_run_log_passed &&
                  report.runtime_stability_passed &&
                  report.power_loss_resume_passed &&
                  report.field_acceptance_passed &&
                  report.section_export_checked &&
                  report.section_export_passed &&
                  !manifest.has_duplicate_keys &&
                  !manifest.has_malformed_tokens &&
                  validSessionToken(report.session_id) &&
                  validScenarioToken(report.scenario) &&
                  validRuntimeDirPath(report.runtime_dir);

  std::ostringstream text;
  text << "evidence_status=" << (report.passed ? "PASS" : "FAIL")
       << ";session_id=" << report.session_id
       << ";scenario=" << report.scenario
       << ";runtime_dir=" << report.runtime_dir
       << ";manifest_duplicate_keys="
       << (manifest.has_duplicate_keys ? "true" : "false")
       << ";manifest_malformed_tokens="
       << (manifest.has_malformed_tokens ? "true" : "false")
       << ";checked_files=" << report.checked_files
       << ";missing_files=" << report.missing_files.size()
       << ";metrics_status=" << (report.metrics_passed ? "PASS" : "FAIL")
       << ";event_file_status="
       << (report.event_file_passed ? "PASS" : "FAIL")
       << ";time_sync_status="
       << (report.time_sync_passed ? "PASS" : "FAIL")
       << ";pps_ptp_wiring_status="
       << (report.pps_ptp_wiring_passed ? "PASS" : "FAIL")
       << ";runtime_health_status="
       << (report.runtime_health_passed ? "PASS" : "FAIL")
       << ";runtime_deployment_status="
       << (report.runtime_deployment_passed ? "PASS" : "FAIL")
       << ";runtime_stability_csv_status="
       << (report.runtime_stability_csv_passed ? "PASS" : "FAIL")
       << ";runtime_stability_run_log_status="
       << (report.runtime_stability_run_log_passed ? "PASS" : "FAIL")
       << ";runtime_stability_status="
       << (report.runtime_stability_passed ? "PASS" : "FAIL")
       << ";power_loss_resume_status="
       << (report.power_loss_resume_passed ? "PASS" : "FAIL")
       << ";field_acceptance_status="
       << (report.field_acceptance_passed ? "PASS" : "FAIL")
       << ";section_export_status="
       << (report.section_export_passed ? "PASS" : "FAIL")
       << "\n";
  for (const EvidenceFileCheck& missing : report.missing_files) {
    text << "missing_file=" << missing.key << ";path=" << missing.path << "\n";
  }
  report.text = text.str();
  return report;
}

}  // namespace lio_eval_tools
