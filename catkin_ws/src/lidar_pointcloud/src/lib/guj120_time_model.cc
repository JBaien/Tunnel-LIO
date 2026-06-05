#include "lidar_pointcloud/guj120_time_model.h"

#include <algorithm>
#include <cmath>

namespace lidar_pointcloud {
namespace {

uint16_t readLe16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8u);
}

uint32_t readLe32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8u) |
         (static_cast<uint32_t>(data[2]) << 16u) |
         (static_cast<uint32_t>(data[3]) << 24u);
}

int forwardAzimuthDelta(uint16_t start_azimuth, uint16_t end_azimuth) {
  int delta = static_cast<int>(end_azimuth) - static_cast<int>(start_azimuth);
  if (delta < 0) {
    delta += GUJ120_ROTATION_UNITS;
  }
  return delta;
}

}  // namespace

bool parseGuj120PacketTimeInfo(const uint8_t* packet, std::size_t packet_size, Guj120PacketTimeInfo* info) {
  if (info != nullptr) {
    *info = Guj120PacketTimeInfo();
  }
  if (packet == nullptr || info == nullptr || packet_size < GUJ120_PACKET_SIZE) {
    return false;
  }

  Guj120PacketTimeInfo parsed;
  parsed.valid = true;
  parsed.device_timestamp_us = readLe32(packet + 1200);
  parsed.block_count = GUJ120_BLOCK_COUNT;
  parsed.block_azimuths.reserve(GUJ120_BLOCK_COUNT);

  bool saw_wrap = false;
  bool monotonic = true;
  uint16_t previous = 0;
  for (int block = 0; block < GUJ120_BLOCK_COUNT; ++block) {
    const std::size_t offset = static_cast<std::size_t>(block) * GUJ120_BLOCK_SIZE;
    const uint16_t azimuth = readLe16(packet + offset + 2u);
    if (azimuth >= GUJ120_ROTATION_UNITS) {
      parsed.valid = false;
      monotonic = false;
    }
    if (block > 0) {
      const int raw_delta = static_cast<int>(azimuth) - static_cast<int>(previous);
      const int forward_delta = forwardAzimuthDelta(previous, azimuth);
      if (raw_delta < 0) {
        saw_wrap = true;
      }
      if (forward_delta > 1000) {
        monotonic = false;
      }
    }
    parsed.block_azimuths.push_back(azimuth);
    previous = azimuth;
  }

  parsed.azimuth_wrapped = saw_wrap;
  parsed.azimuths_monotonic = parsed.valid && monotonic;
  *info = parsed;
  return parsed.valid;
}

void Guj120TimingTracker::reset() {
  stats_ = Guj120TimingStats();
}

void Guj120TimingTracker::observe(const Guj120PacketTimeInfo& info) {
  ++stats_.packet_count;
  if (!info.valid) {
    ++stats_.invalid_packet_count;
    return;
  }

  ++stats_.valid_packet_count;
  if (!info.azimuths_monotonic) {
    ++stats_.azimuth_nonmonotonic_packet_count;
  }

  if (stats_.has_latest_device_timestamp) {
    const int64_t delta =
        static_cast<int64_t>(info.device_timestamp_us) -
        static_cast<int64_t>(stats_.latest_device_timestamp_us);
    stats_.latest_device_timestamp_delta_us = delta;
    if (delta < 0) {
      ++stats_.device_time_regression_count;
    }
  }
  stats_.has_latest_device_timestamp = true;
  stats_.latest_device_timestamp_us = info.device_timestamp_us;
}

Guj120TimingStats Guj120TimingTracker::stats() const {
  return stats_;
}

uint16_t interpolateGuj120Azimuth(uint16_t start_azimuth, uint16_t end_azimuth, double ratio) {
  if (!std::isfinite(ratio)) {
    ratio = 0.0;
  }
  ratio = std::max(0.0, std::min(1.0, ratio));
  const int delta = forwardAzimuthDelta(start_azimuth, end_azimuth);
  const int interpolated = static_cast<int>(std::llround(static_cast<double>(start_azimuth) + ratio * delta));
  return static_cast<uint16_t>((interpolated % GUJ120_ROTATION_UNITS + GUJ120_ROTATION_UNITS) % GUJ120_ROTATION_UNITS);
}

Guj120PointTiming computeGuj120PointTiming(
    const Guj120PacketTimeInfo& info,
    int block_index,
    int firing_index,
    int channel_index) {
  Guj120PointTiming timing;
  if (!info.valid || static_cast<int>(info.block_azimuths.size()) < GUJ120_BLOCK_COUNT) {
    return timing;
  }
  if (block_index < 0 || block_index >= GUJ120_BLOCK_COUNT ||
      firing_index < 0 || firing_index >= GUJ120_FIRINGS_PER_BLOCK ||
      channel_index < 0 || channel_index >= GUJ120_CHANNELS_PER_FIRING) {
    return timing;
  }

  static const double kFiringOffsetUs = 51.0;
  static const double kChannelOffsetUs = 3.0;
  const double offset_us =
      kFiringOffsetUs * static_cast<double>(block_index * GUJ120_FIRINGS_PER_BLOCK + firing_index) +
      kChannelOffsetUs * static_cast<double>(channel_index);

  const uint16_t start_azimuth = info.block_azimuths[static_cast<std::size_t>(block_index)];
  const uint16_t end_azimuth =
      info.block_azimuths[static_cast<std::size_t>((block_index + 1) % GUJ120_BLOCK_COUNT)];
  const double ratio =
      static_cast<double>(firing_index * GUJ120_CHANNELS_PER_FIRING + channel_index) /
      static_cast<double>(GUJ120_FIRINGS_PER_BLOCK * GUJ120_CHANNELS_PER_FIRING);

  timing.valid = true;
  timing.offset_s = offset_us / 1000000.0;
  timing.azimuth = interpolateGuj120Azimuth(start_azimuth, end_azimuth, ratio);
  timing.ring = static_cast<uint16_t>(channel_index);
  return timing;
}

}  // namespace lidar_pointcloud
