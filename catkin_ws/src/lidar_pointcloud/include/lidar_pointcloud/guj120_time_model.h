#ifndef LIDAR_POINTCLOUD_GUJ120_TIME_MODEL_H
#define LIDAR_POINTCLOUD_GUJ120_TIME_MODEL_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace lidar_pointcloud {

static const int GUJ120_PACKET_SIZE = 1206;
static const int GUJ120_BLOCK_COUNT = 12;
static const int GUJ120_BLOCK_SIZE = 100;
static const int GUJ120_FIRINGS_PER_BLOCK = 2;
static const int GUJ120_CHANNELS_PER_FIRING = 16;
static const int GUJ120_ROTATION_UNITS = 36000;

struct Guj120PacketTimeInfo {
  bool valid = false;
  uint32_t device_timestamp_us = 0;
  int block_count = 0;
  bool azimuths_monotonic = false;
  bool azimuth_wrapped = false;
  std::vector<uint16_t> block_azimuths;
};

struct Guj120PointTiming {
  bool valid = false;
  double offset_s = 0.0;
  uint16_t azimuth = 0;
  uint16_t ring = 0;
};

struct Guj120TimingStats {
  int packet_count = 0;
  int valid_packet_count = 0;
  int invalid_packet_count = 0;
  int azimuth_nonmonotonic_packet_count = 0;
  int device_time_regression_count = 0;
  bool has_latest_device_timestamp = false;
  uint32_t latest_device_timestamp_us = 0;
  int64_t latest_device_timestamp_delta_us = 0;
};

class Guj120TimingTracker {
public:
  void reset();
  void observe(const Guj120PacketTimeInfo& info);
  Guj120TimingStats stats() const;

private:
  Guj120TimingStats stats_;
};

bool parseGuj120PacketTimeInfo(const uint8_t* packet, std::size_t packet_size, Guj120PacketTimeInfo* info);

uint16_t interpolateGuj120Azimuth(uint16_t start_azimuth, uint16_t end_azimuth, double ratio);

Guj120PointTiming computeGuj120PointTiming(
    const Guj120PacketTimeInfo& info,
    int block_index,
    int firing_index,
    int channel_index);

}  // namespace lidar_pointcloud

#endif  // LIDAR_POINTCLOUD_GUJ120_TIME_MODEL_H
