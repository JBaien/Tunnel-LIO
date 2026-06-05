#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "imu_modbus_driver/imu_modbus_config.h"

namespace {

uint32_t floatBits(float value) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

}  // namespace

TEST(ImuModbusConfig, ProvidesProductionDefaults) {
  const imu_modbus_driver::ImuModbusConfig config;

  EXPECT_EQ("192.168.188.105", config.ip_address);
  EXPECT_EQ(502, config.port);
  EXPECT_EQ(30, config.start_register);
  EXPECT_EQ(56, config.num_registers);
  EXPECT_EQ("imu_raw", config.output_topic);
  EXPECT_EQ("imu_link", config.frame_id);
  EXPECT_DOUBLE_EQ(400.0, config.poll_rate_hz);
  EXPECT_DOUBLE_EQ(5.0, config.stats_period_sec);
  EXPECT_DOUBLE_EQ(0.8, config.warn_min_publish_rate_ratio);
  EXPECT_DOUBLE_EQ(50.0, config.warn_read_latency_ms);
  EXPECT_DOUBLE_EQ(150.0, config.warn_acc_saturation_mps2);
  EXPECT_DOUBLE_EQ(1800.0 * M_PI / 180.0, config.warn_gyro_saturation_radps);
  EXPECT_DOUBLE_EQ(0.001, config.orientation_covariance_x);
  EXPECT_DOUBLE_EQ(0.001, config.orientation_covariance_y);
  EXPECT_DOUBLE_EQ(0.001, config.orientation_covariance_z);
  EXPECT_DOUBLE_EQ(1.2968807717194922e-05 * 1.2968807717194922e-05,
                   config.angular_velocity_covariance_x);
  EXPECT_DOUBLE_EQ(6.0873535629375565e-06 * 6.0873535629375565e-06,
                   config.angular_velocity_covariance_y);
  EXPECT_DOUBLE_EQ(1.5389301887171400e-05 * 1.5389301887171400e-05,
                   config.angular_velocity_covariance_z);
  EXPECT_DOUBLE_EQ(3.0335331998782933e-03 * 3.0335331998782933e-03,
                   config.linear_acceleration_covariance_x);
  EXPECT_DOUBLE_EQ(2.7319572898178790e-03 * 2.7319572898178790e-03,
                   config.linear_acceleration_covariance_y);
  EXPECT_DOUBLE_EQ(3.2680420545776476e-03 * 3.2680420545776476e-03,
                   config.linear_acceleration_covariance_z);
  EXPECT_FALSE(config.enable_temperature_diagnostics);
  EXPECT_EQ(-1, config.temperature_register);
  EXPECT_EQ("float32", config.temperature_register_type);
  EXPECT_DOUBLE_EQ(1.0, config.temperature_scale);
  EXPECT_DOUBLE_EQ(0.0, config.temperature_offset_c);
  EXPECT_DOUBLE_EQ(85.0, config.warn_temperature_abs_c);
  EXPECT_DOUBLE_EQ(15.0, config.warn_temperature_delta_c);
  EXPECT_EQ("host_now", config.timestamp_source);
  EXPECT_EQ("host_time_only", config.hardware_time_status);
  EXPECT_EQ("unconfigured", config.pps_status);
}

TEST(ImuModbusRegisters, ConvertsLowHighWordsToSignedInt32) {
  EXPECT_EQ(1, imu_modbus_driver::registersToInt32(0x0001, 0x0000));
  EXPECT_EQ(65536, imu_modbus_driver::registersToInt32(0x0000, 0x0001));
  EXPECT_EQ(-1, imu_modbus_driver::registersToInt32(0xffff, 0xffff));
}

TEST(ImuModbusRegisters, ConvertsLowHighWordsToFloat) {
  const uint32_t bits = floatBits(12.5f);
  const uint16_t low = static_cast<uint16_t>(bits & 0xffff);
  const uint16_t high = static_cast<uint16_t>((bits >> 16) & 0xffff);

  EXPECT_FLOAT_EQ(12.5f, imu_modbus_driver::registersToIEEEFloat(low, high));
}

TEST(ImuModbusRegisters, DecodesTemperatureRegistersByType) {
  std::vector<uint16_t> registers(8, 0);
  const uint32_t float_bits = floatBits(36.5f);
  registers[2] = static_cast<uint16_t>(float_bits & 0xffff);
  registers[3] = static_cast<uint16_t>((float_bits >> 16) & 0xffff);
  registers[4] = 0xff9c;
  registers[5] = 0xffff;
  registers[6] = 0xffd8;
  registers[7] = 215;

  double temperature_c = 0.0;
  EXPECT_TRUE(imu_modbus_driver::decodeTemperatureRegisters(
      registers, 2, "float32", 1.0, 0.0, &temperature_c));
  EXPECT_FLOAT_EQ(36.5f, temperature_c);

  EXPECT_TRUE(imu_modbus_driver::decodeTemperatureRegisters(
      registers, 4, "int32", 0.1, 10.0, &temperature_c));
  EXPECT_DOUBLE_EQ(0.0, temperature_c);

  EXPECT_TRUE(imu_modbus_driver::decodeTemperatureRegisters(
      registers, 6, "int16", 1.0, 0.0, &temperature_c));
  EXPECT_DOUBLE_EQ(-40.0, temperature_c);

  EXPECT_TRUE(imu_modbus_driver::decodeTemperatureRegisters(
      registers, 7, "uint16", 0.1, -10.0, &temperature_c));
  EXPECT_DOUBLE_EQ(11.5, temperature_c);
}

TEST(ImuModbusRegisters, RejectsInvalidTemperatureRegisterRequests) {
  const std::vector<uint16_t> registers{0x0001};
  double temperature_c = 12.0;

  EXPECT_FALSE(imu_modbus_driver::decodeTemperatureRegisters(
      registers, -1, "uint16", 1.0, 0.0, &temperature_c));
  EXPECT_DOUBLE_EQ(12.0, temperature_c);

  EXPECT_FALSE(imu_modbus_driver::decodeTemperatureRegisters(
      registers, 0, "float32", 1.0, 0.0, &temperature_c));
  EXPECT_FALSE(imu_modbus_driver::decodeTemperatureRegisters(
      registers, 0, "raw16", 1.0, 0.0, &temperature_c));
  EXPECT_FALSE(imu_modbus_driver::decodeTemperatureRegisters(
      registers, 0, "uint16", std::numeric_limits<double>::infinity(), 0.0,
      &temperature_c));
  EXPECT_FALSE(imu_modbus_driver::decodeTemperatureRegisters(
      registers, 0, "uint16", 1.0, std::numeric_limits<double>::quiet_NaN(),
      &temperature_c));
  EXPECT_FALSE(imu_modbus_driver::decodeTemperatureRegisters(
      registers, 0, "uint16", 1.0, 0.0, nullptr));
}

TEST(ImuModbusValidation, RejectsInvalidSamples) {
  EXPECT_TRUE(imu_modbus_driver::isDataValid({0.0, 0.0, 9.81}, {0.0, 0.0, 0.0}));
  EXPECT_FALSE(imu_modbus_driver::isDataValid({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}));
  EXPECT_FALSE(imu_modbus_driver::isDataValid({0.0, 0.0, 9.81}, {100.0, 0.0, 0.0}));
  EXPECT_FALSE(imu_modbus_driver::isDataValid({NAN, 0.0, 9.81}, {0.0, 0.0, 0.0}));
}

TEST(ImuModbusValidation, DetectsNearSaturationSamples) {
  const double gyro_warn = 1800.0 * M_PI / 180.0;

  EXPECT_FALSE(imu_modbus_driver::isDataNearSaturation(
      {0.0, 0.0, 9.81}, {0.0, 0.0, 0.0}, 150.0, gyro_warn));
  EXPECT_TRUE(imu_modbus_driver::isDataNearSaturation(
      {151.0, 0.0, 9.81}, {0.0, 0.0, 0.0}, 150.0, gyro_warn));
  EXPECT_TRUE(imu_modbus_driver::isDataNearSaturation(
      {0.0, 0.0, 9.81}, {gyro_warn + 0.01, 0.0, 0.0}, 150.0, gyro_warn));
}

TEST(ImuModbusCovariance, BuildsDiagonalCovarianceArray) {
  const Eigen::Matrix<double, 9, 1> covariance =
      imu_modbus_driver::makeDiagonalCovariance(1.0, 2.0, 3.0);

  EXPECT_DOUBLE_EQ(1.0, covariance(0));
  EXPECT_DOUBLE_EQ(0.0, covariance(1));
  EXPECT_DOUBLE_EQ(0.0, covariance(2));
  EXPECT_DOUBLE_EQ(0.0, covariance(3));
  EXPECT_DOUBLE_EQ(2.0, covariance(4));
  EXPECT_DOUBLE_EQ(0.0, covariance(5));
  EXPECT_DOUBLE_EQ(0.0, covariance(6));
  EXPECT_DOUBLE_EQ(0.0, covariance(7));
  EXPECT_DOUBLE_EQ(3.0, covariance(8));
}

TEST(ImuRuntimeStats, TracksFrequencyLatencyErrorsAndDiagnosticLevel) {
  imu_modbus_driver::ImuRuntimeStats stats;
  stats.reset(10.0);

  stats.observeRead(10.00, 10.01, true);
  stats.observePublish(10.01);
  stats.observeRead(10.10, 10.13, true);
  stats.observePublish(10.13);
  stats.observeRead(10.20, 10.26, false);
  stats.observeInvalidFrame();
  stats.observeSaturation();
  stats.observeReconnectAttempt();
  stats.observeReconnectAttempt();
  stats.observeReconnectSuccess();

  const imu_modbus_driver::ImuRuntimeStatsSnapshot snapshot = stats.snapshot(12.0);

  EXPECT_EQ(2, snapshot.publish_count);
  EXPECT_EQ(1, snapshot.read_error_count);
  EXPECT_EQ(1, snapshot.invalid_frame_count);
  EXPECT_EQ(1, snapshot.saturation_count);
  EXPECT_EQ(2, snapshot.reconnect_attempt_count);
  EXPECT_EQ(1, snapshot.reconnect_success_count);
  EXPECT_NEAR(2.0, snapshot.window_duration_sec, 1e-9);
  EXPECT_NEAR(1.0, snapshot.publish_rate_hz, 1e-9);
  EXPECT_NEAR(100.0 / 3.0, snapshot.mean_read_latency_ms, 1e-9);
  EXPECT_NEAR(60.0, snapshot.max_read_latency_ms, 1e-9);
  EXPECT_NEAR(60.0, snapshot.last_read_latency_ms, 1e-9);
  EXPECT_EQ(imu_modbus_driver::ImuDiagnosticLevel::WARN,
            snapshot.diagnosticLevel(true, 400.0, 0.8, 50.0));
  EXPECT_EQ(imu_modbus_driver::ImuDiagnosticLevel::ERROR,
            snapshot.diagnosticLevel(false, 400.0, 0.8, 50.0));
}

TEST(ImuRuntimeStats, ReconnectAttemptsDegradeDiagnostics) {
  imu_modbus_driver::ImuRuntimeStats stats;
  stats.reset(20.0);

  stats.observeReconnectAttempt();

  const imu_modbus_driver::ImuRuntimeStatsSnapshot snapshot = stats.snapshot(21.0);

  EXPECT_EQ(1, snapshot.reconnect_attempt_count);
  EXPECT_EQ(imu_modbus_driver::ImuDiagnosticLevel::WARN,
            snapshot.diagnosticLevel(true, 400.0, 0.8, 50.0));
}

TEST(ImuRuntimeStats, SaturationSamplesDegradeDiagnostics) {
  imu_modbus_driver::ImuRuntimeStats stats;
  stats.reset(30.0);

  stats.observePublish(30.01);
  stats.observeSaturation();

  const imu_modbus_driver::ImuRuntimeStatsSnapshot snapshot = stats.snapshot(31.0);

  EXPECT_EQ(1, snapshot.saturation_count);
  EXPECT_EQ(imu_modbus_driver::ImuDiagnosticLevel::WARN,
            snapshot.diagnosticLevel(true, 1.0, 0.8, 50.0));
}

TEST(ImuRuntimeStats, TracksTemperatureAndWarnsOnHighAbsoluteValue) {
  imu_modbus_driver::ImuRuntimeStats stats;
  stats.reset(40.0);

  stats.observeTemperature(42.0, 85.0, 15.0);
  stats.observeTemperature(88.0, 85.0, 15.0);

  const imu_modbus_driver::ImuRuntimeStatsSnapshot snapshot = stats.snapshot(41.0);

  EXPECT_EQ(2, snapshot.temperature_sample_count);
  EXPECT_DOUBLE_EQ(88.0, snapshot.latest_temperature_c);
  EXPECT_DOUBLE_EQ(42.0, snapshot.min_temperature_c);
  EXPECT_DOUBLE_EQ(88.0, snapshot.max_temperature_c);
  EXPECT_EQ(1, snapshot.temperature_warning_count);
  EXPECT_EQ(imu_modbus_driver::ImuDiagnosticLevel::WARN,
            snapshot.diagnosticLevel(true, 1.0, 0.8, 50.0));
}

TEST(ImuRuntimeStats, WarnsOnTemperatureDeltaWithinWindow) {
  imu_modbus_driver::ImuRuntimeStats stats;
  stats.reset(50.0);

  stats.observeTemperature(30.0, 85.0, 15.0);
  stats.observeTemperature(46.0, 85.0, 15.0);

  const imu_modbus_driver::ImuRuntimeStatsSnapshot snapshot = stats.snapshot(51.0);

  EXPECT_EQ(1, snapshot.temperature_warning_count);
  EXPECT_EQ(imu_modbus_driver::ImuDiagnosticLevel::WARN,
            snapshot.diagnosticLevel(true, 1.0, 0.8, 50.0));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
