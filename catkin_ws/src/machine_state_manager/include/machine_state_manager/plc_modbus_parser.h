#pragma once

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <vector>

namespace machine_state_manager {

struct PlcModbusConfig {
  int left_track_register_index = 0;
  int right_track_register_index = 1;
  int status_register_index = 2;
  int cutting_on_bit = 0;
  int valid_bit = 1;
  double track_speed_scale = 0.001;
};

struct PlcSignals {
  double left_track_speed = 0.0;
  double right_track_speed = 0.0;
  bool cutting_on = false;
  bool data_valid = false;
  bool enough_registers = false;
  bool config_valid = true;
};

inline int16_t registerToSignedInt16(const uint16_t value) {
  return value <= 0x7FFFU ? static_cast<int16_t>(value)
                          : static_cast<int16_t>(static_cast<int32_t>(value) - 0x10000);
}

inline bool registerBitSet(const uint16_t value, const int bit) {
  return bit >= 0 && bit < 16 && ((value & (1U << bit)) != 0U);
}

inline bool validRegisterIndex(const int index) {
  return index >= 0;
}

inline bool validStatusBit(const int bit) {
  return bit >= 0 && bit < 16;
}

inline bool validOptionalStatusBit(const int bit) {
  return bit < 0 || validStatusBit(bit);
}

inline bool validPlcModbusConfig(const PlcModbusConfig& config) {
  return validRegisterIndex(config.left_track_register_index) &&
         validRegisterIndex(config.right_track_register_index) &&
         validRegisterIndex(config.status_register_index) &&
         validStatusBit(config.cutting_on_bit) &&
         validOptionalStatusBit(config.valid_bit) &&
         std::isfinite(config.track_speed_scale) &&
         config.track_speed_scale > 0.0;
}

inline PlcSignals parsePlcRegisters(const std::vector<uint16_t>& registers,
                                    const PlcModbusConfig& config) {
  PlcSignals signals;
  signals.config_valid = validPlcModbusConfig(config);
  if (!signals.config_valid) {
    signals.enough_registers = false;
    signals.data_valid = false;
    return signals;
  }

  const int max_index = std::max(config.status_register_index,
                                 std::max(config.left_track_register_index,
                                          config.right_track_register_index));
  signals.enough_registers =
      max_index >= 0 && static_cast<std::size_t>(max_index) < registers.size();
  if (!signals.enough_registers) {
    return signals;
  }

  signals.left_track_speed =
      static_cast<double>(registerToSignedInt16(
          registers[config.left_track_register_index])) *
      config.track_speed_scale;
  signals.right_track_speed =
      static_cast<double>(registerToSignedInt16(
          registers[config.right_track_register_index])) *
      config.track_speed_scale;

  const uint16_t status = registers[config.status_register_index];
  signals.cutting_on = registerBitSet(status, config.cutting_on_bit);
  signals.data_valid =
      config.valid_bit < 0 || registerBitSet(status, config.valid_bit);
  return signals;
}

}  // namespace machine_state_manager
