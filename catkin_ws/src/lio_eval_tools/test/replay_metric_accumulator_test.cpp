#include <gtest/gtest.h>

#include "lio_eval_tools/replay_metric_accumulator.h"

namespace lio_eval_tools {
namespace {

TEST(ReplayMetricAccumulator, AggregatesReplayEventsIntoValidationMetrics) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=POWER_LOSS_MICRO_MOVE;session_id=s77;t=0.0\n"
      "event=static_sample;scenario=POWER_LOSS_MICRO_MOVE;session_id=s77;"
      "t=1.0;static_drift_m=0.015\n"
      "event=static_sample;scenario=POWER_LOSS_MICRO_MOVE;session_id=s77;"
      "t=2.0;static_drift_m=0.022\n"
      "event=chainage_sample;scenario=POWER_LOSS_MICRO_MOVE;session_id=s77;"
      "t=3.0;chainage_m=50.25;reference_chainage_m=50.0\n"
      "event=power_loss;scenario=POWER_LOSS_MICRO_MOVE;session_id=s77;t=4.0\n"
      "event=recovered;scenario=POWER_LOSS_MICRO_MOVE;session_id=s77;t=34.5\n"
      "event=loop_verified;scenario=POWER_LOSS_MICRO_MOVE;session_id=s77;"
      "t=35.0;wrong_loop=1\n"
      "event=queue_sample;scenario=POWER_LOSS_MICRO_MOVE;session_id=s77;"
      "t=36.0;queue_backlog=7\n"
      "event=pps_sample;scenario=POWER_LOSS_MICRO_MOVE;session_id=s77;"
      "t=37.0;pps_jitter_ms=1.4\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);

  EXPECT_EQ("POWER_LOSS_MICRO_MOVE", metrics.scenario);
  EXPECT_EQ("s77", metrics.session_id);
  EXPECT_NEAR(0.022, metrics.static_drift_m, 1e-9);
  EXPECT_NEAR(0.5, metrics.length_error_percent, 1e-9);
  EXPECT_NEAR(30.5, metrics.recovery_time_s, 1e-9);
  EXPECT_EQ(1, metrics.wrong_loop_count);
  EXPECT_EQ(7, metrics.queue_backlog_max);
  EXPECT_NEAR(1.4, metrics.pps_jitter_ms, 1e-9);
}

TEST(ReplayMetricAccumulator, MissingEventsUseZeroSafeDefaults) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=STATIC_IDLE;session_id=safe;t=0.0\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);

  EXPECT_EQ("STATIC_IDLE", metrics.scenario);
  EXPECT_EQ("safe", metrics.session_id);
  EXPECT_DOUBLE_EQ(0.0, metrics.static_drift_m);
  EXPECT_DOUBLE_EQ(0.0, metrics.length_error_percent);
  EXPECT_DOUBLE_EQ(0.0, metrics.recovery_time_s);
  EXPECT_EQ(0, metrics.wrong_loop_count);
  EXPECT_EQ(0, metrics.queue_backlog_max);
  EXPECT_DOUBLE_EQ(0.0, metrics.pps_jitter_ms);
}

TEST(ReplayMetricAccumulator, MalformedNumericEventsProduceFailingMetrics) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=STATIC_IDLE;session_id=bad;t=0.0\n"
      "event=static_sample;scenario=STATIC_IDLE;session_id=bad;"
      "t=1.0;static_drift_m=nan\n"
      "event=chainage_sample;scenario=STATIC_IDLE;session_id=bad;"
      "t=2.0;chainage_m=50x;reference_chainage_m=50.0\n"
      "event=loop_verified;scenario=STATIC_IDLE;session_id=bad;"
      "t=3.0;wrong_loop=0abc\n"
      "event=queue_sample;scenario=STATIC_IDLE;session_id=bad;"
      "t=4.0;queue_backlog=1abc\n"
      "event=pps_sample;scenario=STATIC_IDLE;session_id=bad;"
      "t=5.0;pps_jitter_ms=inf\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_EQ(5u, result.failed_checks.size());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
  EXPECT_EQ("length_error_percent", result.failed_checks[1].name);
  EXPECT_EQ("wrong_loop_count", result.failed_checks[2].name);
  EXPECT_EQ("queue_backlog_max", result.failed_checks[3].name);
  EXPECT_EQ("pps_jitter_ms", result.failed_checks[4].name);
}

TEST(ReplayMetricAccumulator, NegativeReplayMetricEventsProduceFailingMetrics) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=STATIC_IDLE;session_id=negative;t=0.0\n"
      "event=static_sample;scenario=STATIC_IDLE;session_id=negative;"
      "t=1.0;static_drift_m=-0.01\n"
      "event=length_sample;scenario=STATIC_IDLE;session_id=negative;"
      "t=2.0;length_error_percent=-0.1\n"
      "event=loop_verified;scenario=STATIC_IDLE;session_id=negative;"
      "t=3.0;wrong_loop=-1\n"
      "event=queue_sample;scenario=STATIC_IDLE;session_id=negative;"
      "t=4.0;queue_backlog=-5\n"
      "event=pps_sample;scenario=STATIC_IDLE;session_id=negative;"
      "t=5.0;pps_jitter_ms=-0.5\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_EQ(5u, result.failed_checks.size());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
  EXPECT_EQ("length_error_percent", result.failed_checks[1].name);
  EXPECT_EQ("wrong_loop_count", result.failed_checks[2].name);
  EXPECT_EQ("queue_backlog_max", result.failed_checks[3].name);
  EXPECT_EQ("pps_jitter_ms", result.failed_checks[4].name);
}

TEST(ReplayMetricAccumulator,
     MalformedNumericEventsWithWhitespacePollutionProduceFailingMetrics) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=POWER_LOSS_ORIGIN;session_id=bad_space;t=0.0\n"
      "event=static_sample;scenario=POWER_LOSS_ORIGIN;session_id=bad_space;"
      "t=1.0;static_drift_m= 0.015\n"
      "event=chainage_sample;scenario=POWER_LOSS_ORIGIN;session_id=bad_space;"
      "t=2.0;chainage_m=50.0\r;reference_chainage_m=50.0\n"
      "event=power_loss;scenario=POWER_LOSS_ORIGIN;session_id=bad_space;t=4.0\n"
      "event=recovered;scenario=POWER_LOSS_ORIGIN;session_id=bad_space;t= 34.5\n"
      "event=loop_verified;scenario=POWER_LOSS_ORIGIN;session_id=bad_space;"
      "t=35.0;wrong_loop= 0\n"
      "event=queue_sample;scenario=POWER_LOSS_ORIGIN;session_id=bad_space;"
      "t=36.0;queue_backlog=7\t;note=field\n"
      "event=pps_sample;scenario=POWER_LOSS_ORIGIN;session_id=bad_space;"
      "t=37.0;pps_jitter_ms= 1.4\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
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

TEST(ReplayMetricAccumulator,
     LineEndTimestampWhitespacePollutionProducesFailingMetrics) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=POWER_LOSS_ORIGIN;session_id=bad_time;t=0.0\n"
      "event=power_loss;scenario=POWER_LOSS_ORIGIN;session_id=bad_time;t=4.0\n"
      "event=recovered;scenario=POWER_LOSS_ORIGIN;session_id=bad_time;t=34.5 \n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("recovery_time_s", result.failed_checks[0].name);
}

TEST(ReplayMetricAccumulator, NegativeReplayTimestampsProduceFailingMetric) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=POWER_LOSS_ORIGIN;session_id=bad_time;t=0.0\n"
      "event=static_sample;scenario=POWER_LOSS_ORIGIN;session_id=bad_time;"
      "t=-1.0;static_drift_m=0.01\n");

  ASSERT_EQ(2u, events.size());
  EXPECT_FALSE(events[1].stamp_valid);

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("recovery_time_s", result.failed_checks[0].name);
}

TEST(ReplayMetricAccumulator, MissingReplayEventStructureProducesFailingMetric) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=STATIC_IDLE;session_id=bad_event;t=0.0\n"
      "event=static_sample;scenario=STATIC_IDLE;t=1.0;static_drift_m=0.01\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
}

TEST(ReplayMetricAccumulator, MalformedRecoveryTimestampsProduceFailingMetric) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=POWER_LOSS_ORIGIN;session_id=bad_time;t=0.0\n"
      "event=power_loss;scenario=POWER_LOSS_ORIGIN;session_id=bad_time;t=4.0\n"
      "event=recovered;scenario=POWER_LOSS_ORIGIN;session_id=bad_time;t=bad_time\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_EQ(1u, result.failed_checks.size());
  EXPECT_EQ("recovery_time_s", result.failed_checks[0].name);
}

TEST(ReplayMetricAccumulator, DuplicateEventMetricKeysProduceFailingMetrics) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=STATIC_IDLE;session_id=dup;t=0.0\n"
      "event=static_sample;scenario=STATIC_IDLE;session_id=dup;"
      "t=1.0;static_drift_m=0.20;"
      "static_drift_m=0.01\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
}

TEST(ReplayMetricAccumulator, DuplicateEventTextKeysProduceFailingMetrics) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=STATIC_IDLE;session_id=dup;t=0.0\n"
      "event=static_sample;event=queue_sample;scenario=STATIC_IDLE;"
      "session_id=dup;t=1.0\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
}

TEST(ReplayMetricAccumulator, MalformedEventTokensProduceFailingMetrics) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=STATIC_IDLE;session_id=bad_token;t=0.0\n"
      "event=static_sample;scenario=STATIC_IDLE;session_id=bad_token;"
      "t=1.0;static_drift_m=0.01;not_a_key_value_pair\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
}

TEST(ReplayMetricAccumulator, EmptyEventTokensProduceFailingMetrics) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=STATIC_IDLE;session_id=empty_token;t=0.0\n"
      "event=static_sample;scenario=STATIC_IDLE;session_id=empty_token;"
      "t=1.0;static_drift_m=0.01;\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
}

TEST(ReplayMetricAccumulator, SentinelTextValuesProduceFailingMetrics) {
  const std::vector<ReplayEvent> events = parseReplayEventRecords(
      "event=session_start;scenario=STATIC_IDLE;session_id=missing;t=0.0\n"
      "event=missing;scenario=STATIC_IDLE;session_id=missing;"
      "t=1.0;static_drift_m=0.01\n");

  const ValidationMetrics metrics = aggregateReplayMetrics(events);
  const ValidationResult result =
      evaluateValidationMetrics(metrics, ValidationThresholds{});

  EXPECT_FALSE(result.passed);
  ASSERT_FALSE(result.failed_checks.empty());
  EXPECT_EQ("static_drift_m", result.failed_checks[0].name);
}

}  // namespace
}  // namespace lio_eval_tools

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
