#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "lio_time_manager/time_status.h"

TEST(SensorTimeTracker, ReportsFrequencyFromObservationWindow) {
  lio_time_manager::SensorTimeTracker tracker("imu", 1.0);

  tracker.observe(10.0, 10.01);
  tracker.observe(10.1, 10.11);
  tracker.observe(10.2, 10.21);

  const lio_time_manager::SensorTimeStatus status = tracker.status(10.25);

  EXPECT_NEAR(10.0, status.frequency_hz, 1e-9);
  EXPECT_FALSE(status.time_went_backwards);
  EXPECT_FALSE(status.stale);
}

TEST(SensorTimeTracker, DetectsTimeRegression) {
  lio_time_manager::SensorTimeTracker tracker("points", 1.0);

  tracker.observe(100.0, 100.02);
  tracker.observe(99.9, 100.12);

  const lio_time_manager::SensorTimeStatus status = tracker.status(100.2);

  EXPECT_TRUE(status.time_went_backwards);
  EXPECT_EQ(1, status.regression_count);
}

TEST(SensorTimeTracker, ReportsStaleSensorAndLatency) {
  lio_time_manager::SensorTimeTracker tracker("lidar", 0.5);

  tracker.observe(2.0, 2.2);
  const lio_time_manager::SensorTimeStatus status = tracker.status(3.0);

  EXPECT_TRUE(status.stale);
  EXPECT_NEAR(200.0, status.last_latency_ms, 1e-9);
}

TEST(SensorTimeTracker, RejectsNonFiniteSamples) {
  lio_time_manager::SensorTimeTracker tracker("imu", 1.0);

  tracker.observe(10.0, 10.01);
  tracker.observe(std::numeric_limits<double>::quiet_NaN(), 10.02);
  tracker.observe(10.1, std::numeric_limits<double>::infinity());

  const lio_time_manager::SensorTimeStatus status = tracker.status(10.2);

  EXPECT_EQ(1, status.sample_count);
  EXPECT_FALSE(status.time_went_backwards);
  EXPECT_EQ(0, status.regression_count);
  EXPECT_TRUE(std::isfinite(status.last_latency_ms));
  EXPECT_NEAR(10.0, status.last_latency_ms, 1e-9);
  EXPECT_TRUE(std::isfinite(status.frequency_hz));
}

TEST(SensorTimeTracker, TreatsNonFiniteStatusTimeAsStale) {
  lio_time_manager::SensorTimeTracker tracker("imu", 1.0);

  tracker.observe(10.0, 10.01);
  const lio_time_manager::SensorTimeStatus status =
      tracker.status(std::numeric_limits<double>::quiet_NaN());

  EXPECT_EQ(1, status.sample_count);
  EXPECT_TRUE(status.stale);
  EXPECT_TRUE(std::isfinite(status.last_latency_ms));
}

TEST(ClockOffsetEstimator, EstimatesMeanOffsetAndJitter) {
  lio_time_manager::ClockOffsetEstimator estimator("pps", 4);

  estimator.observe(100.0, 100.012);
  estimator.observe(101.0, 101.014);
  estimator.observe(102.0, 102.016);

  const lio_time_manager::ClockOffsetStatus status = estimator.status();

  EXPECT_EQ("pps", status.name);
  EXPECT_EQ(3, status.sample_count);
  EXPECT_TRUE(status.valid);
  EXPECT_NEAR(14.0, status.mean_offset_ms, 1e-9);
  EXPECT_GT(status.jitter_ms, 0.0);
  EXPECT_FALSE(status.device_time_went_backwards);
}

TEST(ClockOffsetEstimator, DetectsDeviceTimeRegressionAndLimitsWindow) {
  lio_time_manager::ClockOffsetEstimator estimator("imu_device", 2);

  estimator.observe(10.0, 10.010);
  estimator.observe(11.0, 11.011);
  estimator.observe(10.5, 10.512);

  const lio_time_manager::ClockOffsetStatus status = estimator.status();

  EXPECT_EQ(2, status.sample_count);
  EXPECT_EQ(1, status.regression_count);
  EXPECT_TRUE(status.device_time_went_backwards);
  EXPECT_FALSE(status.valid);
}

TEST(ClockOffsetEstimator, RejectsNonFiniteSamples) {
  lio_time_manager::ClockOffsetEstimator estimator("pps", 4);

  estimator.observe(100.0, 100.012);
  estimator.observe(std::numeric_limits<double>::quiet_NaN(), 101.014);
  estimator.observe(102.0, std::numeric_limits<double>::infinity());
  estimator.observe(101.0, 101.016);

  const lio_time_manager::ClockOffsetStatus status = estimator.status();

  EXPECT_EQ(2, status.sample_count);
  EXPECT_TRUE(status.valid);
  EXPECT_FALSE(status.device_time_went_backwards);
  EXPECT_EQ(0, status.regression_count);
  EXPECT_TRUE(std::isfinite(status.mean_offset_ms));
  EXPECT_TRUE(std::isfinite(status.jitter_ms));
  EXPECT_TRUE(std::isfinite(status.latest_device_time));
  EXPECT_TRUE(std::isfinite(status.latest_host_time));
}

TEST(PpsEventTracker, ReportsFrequencyIntervalJitterAndFreshness) {
  lio_time_manager::PpsEventTracker tracker("/time/pps_event", 1.5, 8);

  tracker.observe(100.000, 100.010);
  tracker.observe(101.000, 101.011);
  tracker.observe(102.002, 102.012);

  const lio_time_manager::PpsEventStatus status = tracker.status(102.2);

  EXPECT_EQ("/time/pps_event", status.name);
  EXPECT_EQ(3, status.sample_count);
  EXPECT_FALSE(status.stale);
  EXPECT_FALSE(status.time_went_backwards);
  EXPECT_NEAR(2.0 / 2.002, status.frequency_hz, 1e-9);
  EXPECT_NEAR(1002.0, status.latest_interval_ms, 1e-9);
  EXPECT_GT(status.interval_jitter_ms, 0.0);
}

TEST(PpsEventTracker, ReportsStaleAndTimestampRegression) {
  lio_time_manager::PpsEventTracker tracker("/time/pps_event", 0.5, 4);

  tracker.observe(10.0, 10.01);
  tracker.observe(9.5, 10.20);

  const lio_time_manager::PpsEventStatus status = tracker.status(11.0);

  EXPECT_TRUE(status.stale);
  EXPECT_TRUE(status.time_went_backwards);
  EXPECT_EQ(1, status.regression_count);
}

TEST(PpsEventTracker, RejectsNonFiniteEvents) {
  lio_time_manager::PpsEventTracker tracker("/time/pps_event", 1.5, 8);

  tracker.observe(100.000, 100.010);
  tracker.observe(std::numeric_limits<double>::quiet_NaN(), 101.011);
  tracker.observe(102.000, std::numeric_limits<double>::infinity());
  tracker.observe(101.000, 101.012);

  const lio_time_manager::PpsEventStatus status = tracker.status(101.2);

  EXPECT_EQ(2, status.sample_count);
  EXPECT_FALSE(status.stale);
  EXPECT_FALSE(status.time_went_backwards);
  EXPECT_EQ(0, status.regression_count);
  EXPECT_TRUE(std::isfinite(status.frequency_hz));
  EXPECT_TRUE(std::isfinite(status.latest_interval_ms));
  EXPECT_TRUE(std::isfinite(status.interval_jitter_ms));
  EXPECT_NEAR(1.0, status.frequency_hz, 1e-9);
  EXPECT_NEAR(1000.0, status.latest_interval_ms, 1e-9);
}

TEST(PpsEventTracker, TreatsNonFiniteStatusTimeAsStale) {
  lio_time_manager::PpsEventTracker tracker("/time/pps_event", 1.5, 8);

  tracker.observe(100.000, 100.010);
  const lio_time_manager::PpsEventStatus status =
      tracker.status(std::numeric_limits<double>::quiet_NaN());

  EXPECT_EQ(1, status.sample_count);
  EXPECT_TRUE(status.stale);
  EXPECT_FALSE(status.time_went_backwards);
}

TEST(DiagnosticFeedTracker, ParsesImuModbusTimingAndErrorValues) {
  lio_time_manager::DiagnosticFeedTracker tracker("imu_modbus", 2.0);
  std::vector<lio_time_manager::DiagnosticKeyValue> values;
  values.push_back({"publish_rate_hz", "395.500"});
  values.push_back({"last_read_latency_ms", "2.500"});
  values.push_back({"mean_read_latency_ms", "1.250"});
  values.push_back({"max_read_latency_ms", "4.750"});
  values.push_back({"read_error_count", "1"});
  values.push_back({"saturation_count", "4"});
  values.push_back({"temperature_sample_count", "5"});
  values.push_back({"temperature_warning_count", "1"});
  values.push_back({"latest_temperature_c", "42.500"});
  values.push_back({"min_temperature_c", "31.250"});
  values.push_back({"max_temperature_c", "47.750"});
  values.push_back({"orientation_covariance_x", "0.001"});
  values.push_back({"orientation_covariance_y", "0.002"});
  values.push_back({"orientation_covariance_z", "0.003"});
  values.push_back({"angular_velocity_covariance_x", "0.000010"});
  values.push_back({"angular_velocity_covariance_y", "0.000020"});
  values.push_back({"angular_velocity_covariance_z", "0.000030"});
  values.push_back({"linear_acceleration_covariance_x", "0.010"});
  values.push_back({"linear_acceleration_covariance_y", "0.020"});
  values.push_back({"linear_acceleration_covariance_z", "0.030"});
  values.push_back({"reconnect_attempt_count", "2"});
  values.push_back({"reconnect_success_count", "1"});
  values.push_back({"timestamp_source", "host_now"});
  values.push_back({"hardware_time_status", "host_time_only"});
  values.push_back({"pps_status", "unconfigured"});

  tracker.observe(10.0, lio_time_manager::DiagnosticLevel::WARN, "degraded", values);
  const lio_time_manager::DiagnosticFeedStatus status = tracker.status(11.0);

  EXPECT_EQ("imu_modbus", status.name);
  EXPECT_TRUE(status.received);
  EXPECT_FALSE(status.stale);
  EXPECT_EQ(lio_time_manager::DiagnosticLevel::WARN, status.level);
  EXPECT_EQ("degraded", status.message);
  EXPECT_NEAR(395.5, status.publish_rate_hz, 1e-9);
  EXPECT_NEAR(2.5, status.last_read_latency_ms, 1e-9);
  EXPECT_NEAR(1.25, status.mean_read_latency_ms, 1e-9);
  EXPECT_NEAR(4.75, status.max_read_latency_ms, 1e-9);
  EXPECT_EQ(1, status.read_error_count);
  EXPECT_EQ(4, status.saturation_count);
  EXPECT_EQ(5, status.temperature_sample_count);
  EXPECT_EQ(1, status.temperature_warning_count);
  EXPECT_NEAR(42.5, status.latest_temperature_c, 1e-9);
  EXPECT_NEAR(31.25, status.min_temperature_c, 1e-9);
  EXPECT_NEAR(47.75, status.max_temperature_c, 1e-9);
  EXPECT_NEAR(0.001, status.orientation_covariance_x, 1e-9);
  EXPECT_NEAR(0.002, status.orientation_covariance_y, 1e-9);
  EXPECT_NEAR(0.003, status.orientation_covariance_z, 1e-9);
  EXPECT_NEAR(0.000010, status.angular_velocity_covariance_x, 1e-12);
  EXPECT_NEAR(0.000020, status.angular_velocity_covariance_y, 1e-12);
  EXPECT_NEAR(0.000030, status.angular_velocity_covariance_z, 1e-12);
  EXPECT_NEAR(0.010, status.linear_acceleration_covariance_x, 1e-9);
  EXPECT_NEAR(0.020, status.linear_acceleration_covariance_y, 1e-9);
  EXPECT_NEAR(0.030, status.linear_acceleration_covariance_z, 1e-9);
  EXPECT_EQ(2, status.reconnect_attempt_count);
  EXPECT_EQ(1, status.reconnect_success_count);
  EXPECT_EQ("host_now", status.timestamp_source);
  EXPECT_EQ("host_time_only", status.hardware_time_status);
  EXPECT_EQ("unconfigured", status.pps_status);
}

TEST(DiagnosticFeedTracker, ParsesPlcLatencyInvalidFramesAndStaleState) {
  lio_time_manager::DiagnosticFeedTracker tracker("plc_modbus", 1.0);
  std::vector<lio_time_manager::DiagnosticKeyValue> values;
  values.push_back({"last_read_latency_ms", "18.250"});
  values.push_back({"read_error_count", "2"});
  values.push_back({"invalid_frame_count", "3"});

  tracker.observe(20.0, lio_time_manager::DiagnosticLevel::OK, "ok", values);
  const lio_time_manager::DiagnosticFeedStatus status = tracker.status(21.5);

  EXPECT_TRUE(status.received);
  EXPECT_TRUE(status.stale);
  EXPECT_EQ(lio_time_manager::DiagnosticLevel::WARN, status.effectiveLevel());
  EXPECT_NEAR(18.25, status.last_read_latency_ms, 1e-9);
  EXPECT_EQ(2, status.read_error_count);
  EXPECT_EQ(3, status.invalid_frame_count);
}

TEST(DiagnosticFeedTracker, RejectsMalformedDiagnosticNumericValues) {
  lio_time_manager::DiagnosticFeedTracker tracker("imu_modbus", 2.0);
  std::vector<lio_time_manager::DiagnosticKeyValue> values;
  values.push_back({"publish_rate_hz", " 395.500"});
  values.push_back({"last_read_latency_ms", "2.500\r"});
  values.push_back({"mean_read_latency_ms", "nan"});
  values.push_back({"max_read_latency_ms", "inf"});
  values.push_back({"read_error_count", " 1"});
  values.push_back({"invalid_frame_count", "2\t"});
  values.push_back({"reconnect_attempt_count", "2147483648"});

  tracker.observe(30.0, lio_time_manager::DiagnosticLevel::WARN, "degraded", values);
  const lio_time_manager::DiagnosticFeedStatus status = tracker.status(30.5);

  EXPECT_TRUE(status.received);
  EXPECT_FALSE(status.stale);
  EXPECT_NEAR(0.0, status.publish_rate_hz, 1e-9);
  EXPECT_NEAR(0.0, status.last_read_latency_ms, 1e-9);
  EXPECT_NEAR(0.0, status.mean_read_latency_ms, 1e-9);
  EXPECT_NEAR(0.0, status.max_read_latency_ms, 1e-9);
  EXPECT_EQ(0, status.read_error_count);
  EXPECT_EQ(0, status.invalid_frame_count);
  EXPECT_EQ(0, status.reconnect_attempt_count);
}

TEST(DiagnosticFeedTracker, RejectsNonFiniteReceiptTime) {
  lio_time_manager::DiagnosticFeedTracker tracker("imu_modbus", 2.0);
  std::vector<lio_time_manager::DiagnosticKeyValue> values;
  values.push_back({"publish_rate_hz", "400.000"});

  tracker.observe(std::numeric_limits<double>::quiet_NaN(),
                  lio_time_manager::DiagnosticLevel::OK,
                  "ok",
                  values);
  const lio_time_manager::DiagnosticFeedStatus status = tracker.status(10.0);

  EXPECT_FALSE(status.received);
  EXPECT_TRUE(status.stale);
  EXPECT_EQ(lio_time_manager::DiagnosticLevel::WARN, status.effectiveLevel());
  EXPECT_NEAR(0.0, status.publish_rate_hz, 1e-9);
}

TEST(DiagnosticFeedTracker, TreatsNonFiniteStatusTimeAsStale) {
  lio_time_manager::DiagnosticFeedTracker tracker("plc_modbus", 1.0);

  tracker.observe(20.0, lio_time_manager::DiagnosticLevel::OK, "ok",
                  std::vector<lio_time_manager::DiagnosticKeyValue>());
  const lio_time_manager::DiagnosticFeedStatus status =
      tracker.status(std::numeric_limits<double>::quiet_NaN());

  EXPECT_TRUE(status.received);
  EXPECT_TRUE(status.stale);
  EXPECT_EQ(lio_time_manager::DiagnosticLevel::WARN, status.effectiveLevel());
  EXPECT_EQ("diagnostics stale", status.message);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
