#include <gtest/gtest.h>

#include "lio_eval_tools/validation_metrics.h"

namespace lio_eval_tools {
namespace {

TEST(ValidationMetrics, ParsesSemicolonMetricRecord) {
  const ValidationMetrics metrics = parseMetricRecord(
      "scenario=POWER_LOSS_MICRO_MOVE;session_id=s42;static_drift_m=0.018;"
      "length_error_percent=0.27;recovery_time_s=31.5;wrong_loop_count=0;"
      "queue_backlog_max=2;pps_jitter_ms=0.6");

  EXPECT_EQ("POWER_LOSS_MICRO_MOVE", metrics.scenario);
  EXPECT_EQ("s42", metrics.session_id);
  EXPECT_NEAR(0.018, metrics.static_drift_m, 1e-9);
  EXPECT_NEAR(0.27, metrics.length_error_percent, 1e-9);
  EXPECT_NEAR(31.5, metrics.recovery_time_s, 1e-9);
  EXPECT_EQ(0, metrics.wrong_loop_count);
  EXPECT_EQ(2, metrics.queue_backlog_max);
  EXPECT_NEAR(0.6, metrics.pps_jitter_ms, 1e-9);
}

TEST(ValidationMetrics, AcceptsReplayWhenAllThresholdsAreMet) {
  ValidationMetrics metrics;
  metrics.scenario = "POWER_LOSS_ORIGIN";
  metrics.session_id = "ok_run";
  metrics.static_drift_m = 0.01;
  metrics.length_error_percent = 0.2;
  metrics.recovery_time_s = 22.0;
  metrics.wrong_loop_count = 0;
  metrics.queue_backlog_max = 4;
  metrics.pps_jitter_ms = 0.8;

  const ValidationResult result = evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_TRUE(result.passed);
  EXPECT_TRUE(result.failed_checks.empty());
  EXPECT_NE(result.summary.find("ok_run"), std::string::npos);
}

TEST(ValidationMetrics, ReportsEveryFailedGate) {
  ValidationMetrics metrics;
  metrics.scenario = "LONG_STRAIGHT";
  metrics.session_id = "bad_run";
  metrics.static_drift_m = 0.12;
  metrics.length_error_percent = 1.4;
  metrics.recovery_time_s = 70.0;
  metrics.wrong_loop_count = 1;
  metrics.queue_backlog_max = 21;
  metrics.pps_jitter_ms = 4.5;

  ValidationThresholds thresholds;
  thresholds.max_static_drift_m = 0.05;
  thresholds.max_length_error_percent = 0.5;
  thresholds.max_recovery_time_s = 45.0;
  thresholds.max_wrong_loop_count = 0;
  thresholds.max_queue_backlog = 10;
  thresholds.max_pps_jitter_ms = 2.0;

  const ValidationResult result = evaluateValidationMetrics(metrics, thresholds);

  EXPECT_FALSE(result.passed);
  EXPECT_EQ(6u, result.failed_checks.size());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
  EXPECT_EQ("pps_jitter_ms", result.failed_checks[5].name);
}

TEST(ValidationMetrics, RejectsMalformedAndNonFiniteMetricNumbers) {
  const ValidationMetrics metrics = parseMetricRecord(
      "scenario=STATIC;session_id=bad_numbers;static_drift_m=0.01abc;"
      "length_error_percent=nan;recovery_time_s=10s;"
      "wrong_loop_count=0abc;queue_backlog_max=1abc;pps_jitter_ms=inf");

  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_EQ(6u, result.failed_checks.size());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
  EXPECT_EQ("length_error_percent", result.failed_checks[1].name);
  EXPECT_EQ("recovery_time_s", result.failed_checks[2].name);
  EXPECT_EQ("wrong_loop_count", result.failed_checks[3].name);
  EXPECT_EQ("queue_backlog_max", result.failed_checks[4].name);
  EXPECT_EQ("pps_jitter_ms", result.failed_checks[5].name);
}

TEST(ValidationMetrics, RejectsMetricNumbersWithWhitespacePollution) {
  const ValidationMetrics metrics = parseMetricRecord(
      "scenario=STATIC;session_id=bad_space;static_drift_m= 0.01;"
      "length_error_percent=0.2\r;recovery_time_s= 10;"
      "wrong_loop_count= 0;queue_backlog_max=1\t;pps_jitter_ms=0.4\t");

  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_EQ(6u, result.failed_checks.size());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
  EXPECT_EQ("length_error_percent", result.failed_checks[1].name);
  EXPECT_EQ("recovery_time_s", result.failed_checks[2].name);
  EXPECT_EQ("wrong_loop_count", result.failed_checks[3].name);
  EXPECT_EQ("queue_backlog_max", result.failed_checks[4].name);
  EXPECT_EQ("pps_jitter_ms", result.failed_checks[5].name);
}

TEST(ValidationMetrics, RejectsBatchMetricLineEndWhitespacePollution) {
  const std::vector<ValidationMetrics> records = parseMetricRecords(
      "scenario=STATIC;session_id=batch_bad_space;static_drift_m=0.01;"
      "length_error_percent=0.1;recovery_time_s=10;wrong_loop_count=0;"
      "queue_backlog_max=1;pps_jitter_ms=0.4 \n");

  ASSERT_EQ(1u, records.size());
  const ValidationResult result =
      evaluateValidationMetrics(records[0], ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("pps_jitter_ms", result.failed_checks.back().name);
}

TEST(ValidationMetrics, DuplicateMetricRecordKeysFailClosed) {
  const ValidationMetrics metrics = parseMetricRecord(
      "scenario=STATIC;session_id=dup;static_drift_m=0.20;"
      "static_drift_m=0.01;length_error_percent=0.1;recovery_time_s=10;"
      "wrong_loop_count=0;queue_backlog_max=1;pps_jitter_ms=0.4");

  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
}

TEST(ValidationMetrics, DuplicateMetricRecordTextKeysFailClosed) {
  const ValidationMetrics metrics = parseMetricRecord(
      "scenario=STATIC;session_id=dup;session_id=shadow;"
      "static_drift_m=0.01;length_error_percent=0.1;recovery_time_s=10;"
      "wrong_loop_count=0;queue_backlog_max=1;pps_jitter_ms=0.4");

  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
}

TEST(ValidationMetrics, MalformedMetricRecordTokensFailClosed) {
  const ValidationMetrics metrics = parseMetricRecord(
      "scenario=STATIC;session_id=bad_token;static_drift_m=0.01;"
      "length_error_percent=0.1;recovery_time_s=10;wrong_loop_count=0;"
      "queue_backlog_max=1;pps_jitter_ms=0.4;not_a_key_value_pair");

  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
}

TEST(ValidationMetrics, EmptyMetricRecordTokensFailClosed) {
  const ValidationMetrics metrics = parseMetricRecord(
      "scenario=STATIC;session_id=empty_token;static_drift_m=0.01;"
      "length_error_percent=0.1;recovery_time_s=10;wrong_loop_count=0;"
      "queue_backlog_max=1;pps_jitter_ms=0.4;");

  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
}

TEST(ValidationMetrics, BuildsBatchReportFromMultipleRecords) {
  const std::vector<ValidationMetrics> records = parseMetricRecords(
      "scenario=STATIC;session_id=ok;static_drift_m=0.01;"
      "length_error_percent=0.1;recovery_time_s=10;wrong_loop_count=0;"
      "queue_backlog_max=1;pps_jitter_ms=0.4\n"
      "scenario=WRONG_LOOP_TRAP;session_id=bad;static_drift_m=0.01;"
      "length_error_percent=0.1;recovery_time_s=10;wrong_loop_count=2;"
      "queue_backlog_max=1;pps_jitter_ms=0.4\n");

  const ValidationBatchReport report =
      evaluateValidationBatch(records, ValidationThresholds{});

  EXPECT_FALSE(report.passed);
  EXPECT_EQ(2u, report.total_records);
  EXPECT_EQ(1u, report.failed_records);
  EXPECT_NE(report.text.find("overall=FAIL"), std::string::npos);
  EXPECT_NE(report.text.find("session=bad"), std::string::npos);
}

TEST(ValidationMetrics, AppliesScenarioSpecificThresholds) {
  const std::vector<ValidationMetrics> records = parseMetricRecords(
      "scenario=STATIC;session_id=static_ok;static_drift_m=0.01;"
      "length_error_percent=0.4;recovery_time_s=10;wrong_loop_count=0;"
      "queue_backlog_max=1;pps_jitter_ms=0.4\n"
      "scenario=LONG_STRAIGHT;session_id=long_fail;static_drift_m=0.01;"
      "length_error_percent=0.4;recovery_time_s=10;wrong_loop_count=0;"
      "queue_backlog_max=1;pps_jitter_ms=0.4\n");
  const std::vector<ScenarioValidationThresholds> scenario_thresholds =
      parseScenarioThresholdRecords(
          "scenario=LONG_STRAIGHT;max_length_error_percent=0.3\n");

  const ValidationBatchReport report =
      evaluateValidationBatch(records, ValidationThresholds{}, scenario_thresholds);

  EXPECT_FALSE(report.passed);
  EXPECT_EQ(2u, report.total_records);
  EXPECT_EQ(1u, report.failed_records);
  EXPECT_NE(report.text.find("session=static_ok;scenario=STATIC;status=PASS"),
            std::string::npos);
  EXPECT_NE(report.text.find("session=long_fail;scenario=LONG_STRAIGHT;status=FAIL"),
            std::string::npos);
  EXPECT_NE(report.text.find("length_error_percent"), std::string::npos);
}

TEST(ValidationMetrics, ScenarioThresholdMissingFieldsUseRuntimeDefaults) {
  const std::vector<ValidationMetrics> records = parseMetricRecords(
      "scenario=LONG_STRAIGHT;session_id=drift_should_fail;"
      "static_drift_m=0.03;length_error_percent=0.2;recovery_time_s=10;"
      "wrong_loop_count=0;queue_backlog_max=1;pps_jitter_ms=0.4\n");
  ValidationThresholds runtime_defaults;
  runtime_defaults.max_static_drift_m = 0.02;
  const std::vector<ScenarioValidationThresholds> scenario_thresholds =
      parseScenarioThresholdRecords(
          "scenario=LONG_STRAIGHT;max_length_error_percent=0.3\n");

  const ValidationBatchReport report =
      evaluateValidationBatch(records, runtime_defaults, scenario_thresholds);

  EXPECT_FALSE(report.passed);
  EXPECT_EQ(1u, report.failed_records);
  EXPECT_NE(report.text.find("static_drift_m"), std::string::npos);
}

TEST(ValidationMetrics, MalformedScenarioThresholdFieldsFailClosed) {
  const std::vector<ValidationMetrics> records = parseMetricRecords(
      "scenario=LONG_STRAIGHT;session_id=bad_threshold;"
      "static_drift_m=0.03;length_error_percent=0.2;recovery_time_s=10;"
      "wrong_loop_count=0;queue_backlog_max=1;pps_jitter_ms=0.4\n");
  ValidationThresholds runtime_defaults;
  runtime_defaults.max_static_drift_m = 0.05;
  const std::vector<ScenarioValidationThresholds> scenario_thresholds =
      parseScenarioThresholdRecords(
          "scenario=LONG_STRAIGHT;max_static_drift_m=bad\n");

  const ValidationBatchReport report =
      evaluateValidationBatch(records, runtime_defaults, scenario_thresholds);

  EXPECT_FALSE(report.passed);
  EXPECT_EQ(1u, report.failed_records);
  EXPECT_NE(report.text.find("static_drift_m"), std::string::npos);
}

TEST(ValidationMetrics, ScenarioThresholdLineEndWhitespacePollutionFailsClosed) {
  const std::vector<ScenarioValidationThresholds> scenario_thresholds =
      parseScenarioThresholdRecords(
          "scenario=LONG_STRAIGHT;max_length_error_percent=0.3 \n");

  ASSERT_EQ(1u, scenario_thresholds.size());
  EXPECT_TRUE(scenario_thresholds[0].has_max_length_error_percent);
  EXPECT_DOUBLE_EQ(-1.0,
                   scenario_thresholds[0].thresholds.max_length_error_percent);
}

TEST(ValidationMetrics, DuplicateScenarioThresholdTextKeysFailClosed) {
  const std::vector<ValidationMetrics> records = parseMetricRecords(
      "scenario=LONG_STRAIGHT;session_id=duplicate_threshold_scenario;"
      "static_drift_m=0.01;length_error_percent=0.4;recovery_time_s=10;"
      "wrong_loop_count=0;queue_backlog_max=1;pps_jitter_ms=0.4\n");
  ValidationThresholds runtime_defaults;
  runtime_defaults.max_length_error_percent = 0.5;
  const std::vector<ScenarioValidationThresholds> scenario_thresholds =
      parseScenarioThresholdRecords(
          "scenario=LONG_STRAIGHT;scenario=STATIC;"
          "max_length_error_percent=0.3\n");

  const ValidationBatchReport report =
      evaluateValidationBatch(records, runtime_defaults, scenario_thresholds);

  EXPECT_FALSE(report.passed);
  EXPECT_EQ(1u, report.failed_records);
  EXPECT_NE(report.text.find("failed_check"), std::string::npos);
}

TEST(ValidationMetrics, MalformedScenarioThresholdTokensFailClosed) {
  const std::vector<ValidationMetrics> records = parseMetricRecords(
      "scenario=LONG_STRAIGHT;session_id=bad_threshold_token;"
      "static_drift_m=0.01;length_error_percent=0.2;recovery_time_s=10;"
      "wrong_loop_count=0;queue_backlog_max=1;pps_jitter_ms=0.4\n");
  ValidationThresholds runtime_defaults;
  runtime_defaults.max_length_error_percent = 0.5;
  const std::vector<ScenarioValidationThresholds> scenario_thresholds =
      parseScenarioThresholdRecords(
          "scenario=LONG_STRAIGHT;max_length_error_percent=0.3;"
          "not_a_key_value_pair\n");

  const ValidationBatchReport report =
      evaluateValidationBatch(records, runtime_defaults, scenario_thresholds);

  EXPECT_FALSE(report.passed);
  EXPECT_EQ(1u, report.failed_records);
  EXPECT_NE(report.text.find("failed_check"), std::string::npos);
}

TEST(ValidationMetrics, EmptyScenarioThresholdTokensFailClosed) {
  const std::vector<ValidationMetrics> records = parseMetricRecords(
      "scenario=LONG_STRAIGHT;session_id=empty_threshold_token;"
      "static_drift_m=0.01;length_error_percent=0.2;recovery_time_s=10;"
      "wrong_loop_count=0;queue_backlog_max=1;pps_jitter_ms=0.4\n");
  ValidationThresholds runtime_defaults;
  runtime_defaults.max_length_error_percent = 0.5;
  const std::vector<ScenarioValidationThresholds> scenario_thresholds =
      parseScenarioThresholdRecords(
          "scenario=LONG_STRAIGHT;max_length_error_percent=0.3;\n");

  const ValidationBatchReport report =
      evaluateValidationBatch(records, runtime_defaults, scenario_thresholds);

  EXPECT_FALSE(report.passed);
  EXPECT_EQ(1u, report.failed_records);
  EXPECT_NE(report.text.find("failed_check"), std::string::npos);
}

}  // namespace
}  // namespace lio_eval_tools

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
