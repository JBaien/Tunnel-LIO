#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "machine_state_manager/plc_modbus_parser.h"

namespace machine_state_manager {
namespace {

TEST(PlcModbusParser, ParsesSignedTrackSpeedsAndStatusBits) {
  PlcModbusConfig config;
  config.left_track_register_index = 0;
  config.right_track_register_index = 1;
  config.status_register_index = 2;
  config.track_speed_scale = 0.001;
  config.cutting_on_bit = 0;
  config.valid_bit = 1;

  const std::vector<uint16_t> registers = {
      1200U,
      static_cast<uint16_t>(-850),
      0x0003U,
  };

  const PlcSignals signals = parsePlcRegisters(registers, config);

  ASSERT_TRUE(signals.enough_registers);
  ASSERT_TRUE(signals.data_valid);
  EXPECT_NEAR(1.2, signals.left_track_speed, 1e-9);
  EXPECT_NEAR(-0.85, signals.right_track_speed, 1e-9);
  EXPECT_TRUE(signals.cutting_on);
}

TEST(PlcModbusParser, MarksShortOrInvalidFramesUnusable) {
  PlcModbusConfig config;
  config.left_track_register_index = 0;
  config.right_track_register_index = 1;
  config.status_register_index = 2;
  config.valid_bit = 1;

  EXPECT_FALSE(parsePlcRegisters({100U, 100U}, config).enough_registers);

  const PlcSignals invalid = parsePlcRegisters({100U, 100U, 0x0001U}, config);
  ASSERT_TRUE(invalid.enough_registers);
  EXPECT_FALSE(invalid.data_valid);
}

TEST(PlcModbusParser, RejectsInvalidRegisterConfigAndScale) {
  PlcModbusConfig config;
  config.left_track_register_index = 0;
  config.right_track_register_index = 1;
  config.status_register_index = 2;
  config.valid_bit = 1;
  config.track_speed_scale = std::numeric_limits<double>::quiet_NaN();

  const PlcSignals invalid_scale = parsePlcRegisters({100U, 100U, 0x0002U}, config);
  EXPECT_FALSE(invalid_scale.enough_registers);
  EXPECT_FALSE(invalid_scale.data_valid);
  EXPECT_TRUE(std::isfinite(invalid_scale.left_track_speed));
  EXPECT_TRUE(std::isfinite(invalid_scale.right_track_speed));

  config.track_speed_scale = 0.001;
  config.left_track_register_index = -1;
  const PlcSignals invalid_index = parsePlcRegisters({100U, 100U, 0x0002U}, config);
  EXPECT_FALSE(invalid_index.enough_registers);
  EXPECT_FALSE(invalid_index.data_valid);
}

}  // namespace
}  // namespace machine_state_manager

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
