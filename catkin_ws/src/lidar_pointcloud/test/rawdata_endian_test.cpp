#include <gtest/gtest.h>
#include <lidar_pointcloud/rawdata.h>

#include <cstdint>
#include <cstring>

namespace {

lidar_rawdata::raw_block_t makeBlockFromWireBytes(uint16_t header,
                                                  uint16_t rotation,
                                                  uint16_t distance)
{
  lidar_rawdata::raw_block_t block;
  std::memset(&block, 0, sizeof(block));
  uint8_t* bytes = reinterpret_cast<uint8_t*>(&block);
  bytes[0] = static_cast<uint8_t>(header & 0xffu);
  bytes[1] = static_cast<uint8_t>((header >> 8) & 0xffu);
  bytes[2] = static_cast<uint8_t>(rotation & 0xffu);
  bytes[3] = static_cast<uint8_t>((rotation >> 8) & 0xffu);
  block.data[0] = static_cast<uint8_t>(distance & 0xffu);
  block.data[1] = static_cast<uint8_t>((distance >> 8) & 0xffu);
  return block;
}

}  // namespace

TEST(LidarRawDataEndianTest, ReadsLittleEndianPacketWordsFromWireBytes)
{
  const uint8_t bytes[] = {0x34, 0x12};

  EXPECT_EQ(0x1234u, lidar_rawdata::readLittleEndianUint16(bytes));
}

TEST(LidarRawDataEndianTest, ReadsBlockHeaderRotationAndDistanceFromWireBytes)
{
  const lidar_rawdata::raw_block_t upper =
      makeBlockFromWireBytes(lidar_rawdata::UPPER_BANK, 0x1234u, 0x5678u);
  const lidar_rawdata::raw_block_t lower =
      makeBlockFromWireBytes(lidar_rawdata::LOWER_BANK, 0x0102u, 0x0304u);

  EXPECT_EQ(lidar_rawdata::UPPER_BANK, lidar_rawdata::rawBlockHeader(upper));
  EXPECT_EQ(0x1234u, lidar_rawdata::rawBlockRotation(upper));
  EXPECT_EQ(0x5678u, lidar_rawdata::rawBlockDistance(upper, 0));
  EXPECT_EQ(lidar_rawdata::LOWER_BANK, lidar_rawdata::rawBlockHeader(lower));
  EXPECT_EQ(0x0102u, lidar_rawdata::rawBlockRotation(lower));
  EXPECT_EQ(0x0304u, lidar_rawdata::rawBlockDistance(lower, 0));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
