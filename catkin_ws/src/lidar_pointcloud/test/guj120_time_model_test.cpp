#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "lidar_pointcloud/guj120_time_model.h"

namespace {

std::array<uint8_t, 1206> makePacket() {
  std::array<uint8_t, 1206> packet{};
  for (int block = 0; block < 12; ++block) {
    const uint16_t azimuth = static_cast<uint16_t>((35900 + block * 20) % 36000);
    const std::size_t offset = static_cast<std::size_t>(block) * 100u;
    packet[offset + 2] = static_cast<uint8_t>(azimuth & 0xffu);
    packet[offset + 3] = static_cast<uint8_t>((azimuth >> 8u) & 0xffu);
  }
  packet[1200] = 0x78;
  packet[1201] = 0x56;
  packet[1202] = 0x34;
  packet[1203] = 0x12;
  return packet;
}

}  // namespace

TEST(Guj120TimeModel, ParsesPacketTailDeviceTimestampLittleEndian) {
  const std::array<uint8_t, 1206> packet = makePacket();

  lidar_pointcloud::Guj120PacketTimeInfo info;
  ASSERT_TRUE(lidar_pointcloud::parseGuj120PacketTimeInfo(packet.data(), packet.size(), &info));

  EXPECT_TRUE(info.valid);
  EXPECT_EQ(0x12345678u, info.device_timestamp_us);
  EXPECT_EQ(12, info.block_count);
  EXPECT_TRUE(info.azimuths_monotonic);
}

TEST(Guj120TimeModel, InterpolatesAzimuthAcrossZeroWithoutLargeJump) {
  EXPECT_EQ(0u, lidar_pointcloud::interpolateGuj120Azimuth(35990u, 10u, 0.5));
  EXPECT_EQ(35995u, lidar_pointcloud::interpolateGuj120Azimuth(35990u, 10u, 0.25));
  EXPECT_EQ(5u, lidar_pointcloud::interpolateGuj120Azimuth(35990u, 10u, 0.75));
}

TEST(Guj120TimeModel, ComputesMonotonicPointOffsetsWithinPacket) {
  const std::array<uint8_t, 1206> packet = makePacket();
  lidar_pointcloud::Guj120PacketTimeInfo info;
  ASSERT_TRUE(lidar_pointcloud::parseGuj120PacketTimeInfo(packet.data(), packet.size(), &info));

  const lidar_pointcloud::Guj120PointTiming first =
      lidar_pointcloud::computeGuj120PointTiming(info, 0, 0, 0);
  const lidar_pointcloud::Guj120PointTiming second_channel =
      lidar_pointcloud::computeGuj120PointTiming(info, 0, 0, 1);
  const lidar_pointcloud::Guj120PointTiming second_firing =
      lidar_pointcloud::computeGuj120PointTiming(info, 0, 1, 0);
  const lidar_pointcloud::Guj120PointTiming next_block =
      lidar_pointcloud::computeGuj120PointTiming(info, 1, 0, 0);

  EXPECT_TRUE(first.valid);
  EXPECT_DOUBLE_EQ(0.0, first.offset_s);
  EXPECT_NEAR(3.0e-6, second_channel.offset_s, 1e-12);
  EXPECT_NEAR(51.0e-6, second_firing.offset_s, 1e-12);
  EXPECT_NEAR(102.0e-6, next_block.offset_s, 1e-12);
  EXPECT_LT(first.offset_s, second_channel.offset_s);
  EXPECT_LT(second_channel.offset_s, second_firing.offset_s);
  EXPECT_LT(second_firing.offset_s, next_block.offset_s);
}

TEST(Guj120TimeModel, RejectsInvalidPacketAndPointIndexesFailClosed) {
  lidar_pointcloud::Guj120PacketTimeInfo info;
  EXPECT_FALSE(lidar_pointcloud::parseGuj120PacketTimeInfo(nullptr, 1206, &info));
  EXPECT_FALSE(lidar_pointcloud::parseGuj120PacketTimeInfo(makePacket().data(), 1205, &info));

  const std::array<uint8_t, 1206> packet = makePacket();
  ASSERT_TRUE(lidar_pointcloud::parseGuj120PacketTimeInfo(packet.data(), packet.size(), &info));
  EXPECT_FALSE(lidar_pointcloud::computeGuj120PointTiming(info, -1, 0, 0).valid);
  EXPECT_FALSE(lidar_pointcloud::computeGuj120PointTiming(info, 12, 0, 0).valid);
  EXPECT_FALSE(lidar_pointcloud::computeGuj120PointTiming(info, 0, 2, 0).valid);
  EXPECT_FALSE(lidar_pointcloud::computeGuj120PointTiming(info, 0, 0, 16).valid);
}

TEST(Guj120TimeModel, TrackerCountsInvalidAzimuthAndDeviceTimeRegression) {
  std::array<uint8_t, 1206> first = makePacket();
  std::array<uint8_t, 1206> second = makePacket();
  first[1200] = 0xe8;
  first[1201] = 0x03;
  first[1202] = 0x00;
  first[1203] = 0x00;
  second[1200] = 0x84;
  second[1201] = 0x03;
  second[1202] = 0x00;
  second[1203] = 0x00;
  second[102] = 0x20;
  second[103] = 0x4e;

  lidar_pointcloud::Guj120PacketTimeInfo first_info;
  lidar_pointcloud::Guj120PacketTimeInfo second_info;
  ASSERT_TRUE(lidar_pointcloud::parseGuj120PacketTimeInfo(first.data(), first.size(), &first_info));
  ASSERT_TRUE(lidar_pointcloud::parseGuj120PacketTimeInfo(second.data(), second.size(), &second_info));

  lidar_pointcloud::Guj120TimingTracker tracker;
  tracker.observe(first_info);
  tracker.observe(second_info);
  tracker.observe(lidar_pointcloud::Guj120PacketTimeInfo());

  const lidar_pointcloud::Guj120TimingStats stats = tracker.stats();
  EXPECT_EQ(3, stats.packet_count);
  EXPECT_EQ(2, stats.valid_packet_count);
  EXPECT_EQ(1, stats.invalid_packet_count);
  EXPECT_EQ(1, stats.azimuth_nonmonotonic_packet_count);
  EXPECT_EQ(1, stats.device_time_regression_count);
  EXPECT_EQ(900u, stats.latest_device_timestamp_us);
  EXPECT_EQ(-100, stats.latest_device_timestamp_delta_us);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
