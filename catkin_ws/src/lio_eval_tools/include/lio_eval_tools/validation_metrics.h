#pragma once

#include <string>
#include <vector>

namespace lio_eval_tools {

struct ValidationMetrics {
  std::string scenario;
  std::string session_id;
  double static_drift_m = -1.0;
  double length_error_percent = -1.0;
  double recovery_time_s = -1.0;
  int wrong_loop_count = -1;
  int queue_backlog_max = -1;
  double pps_jitter_ms = -1.0;
};

struct ValidationThresholds {
  double max_static_drift_m = 0.05;
  double max_length_error_percent = 0.5;
  double max_recovery_time_s = 45.0;
  int max_wrong_loop_count = 0;
  int max_queue_backlog = 10;
  double max_pps_jitter_ms = 2.0;
};

struct ScenarioValidationThresholds {
  std::string scenario;
  ValidationThresholds thresholds;
  bool has_max_static_drift_m = false;
  bool has_max_length_error_percent = false;
  bool has_max_recovery_time_s = false;
  bool has_max_wrong_loop_count = false;
  bool has_max_queue_backlog = false;
  bool has_max_pps_jitter_ms = false;
};

struct FailedCheck {
  std::string name;
  double value = 0.0;
  double limit = 0.0;
  std::string message;
};

struct ValidationResult {
  bool passed = false;
  std::vector<FailedCheck> failed_checks;
  std::string summary;
};

struct ValidationBatchReport {
  bool passed = false;
  std::size_t total_records = 0;
  std::size_t failed_records = 0;
  std::string text;
};

ValidationMetrics parseMetricRecord(const std::string& line);

std::vector<ValidationMetrics> parseMetricRecords(const std::string& text);

std::vector<ScenarioValidationThresholds> parseScenarioThresholdRecords(
    const std::string& text);

ValidationResult evaluateValidationMetrics(const ValidationMetrics& metrics,
                                           const ValidationThresholds& thresholds);

ValidationBatchReport evaluateValidationBatch(const std::vector<ValidationMetrics>& records,
                                              const ValidationThresholds& thresholds);

ValidationBatchReport evaluateValidationBatch(
    const std::vector<ValidationMetrics>& records,
    const ValidationThresholds& default_thresholds,
    const std::vector<ScenarioValidationThresholds>& scenario_thresholds);

std::string formatValidationReport(const ValidationMetrics& metrics,
                                   const ValidationResult& result);

}  // namespace lio_eval_tools
