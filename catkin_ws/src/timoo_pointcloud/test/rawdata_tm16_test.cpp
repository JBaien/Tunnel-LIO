#include <gtest/gtest.h>
#include <timoo_pointcloud/rawdata.h>

#include <cstdint>
#include <cstring>

namespace {

timoo_rawdata::raw_block_t makeBlockFromWireBytes(uint16_t header,
                                                  uint16_t rotation,
                                                  uint16_t distance)
{
    timoo_rawdata::raw_block_t block;
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

} // namespace

TEST(TimooRawDataEndian, ReadsLittleEndianPacketWordsFromWireBytes)
{
    const uint8_t bytes[] = {0x34, 0x12};

    EXPECT_EQ(0x1234u, timoo_rawdata::readLittleEndianUint16(bytes));
}

TEST(TimooRawDataEndian, ReadsBlockHeaderRotationAndDistanceFromWireBytes)
{
    const timoo_rawdata::raw_block_t upper =
        makeBlockFromWireBytes(timoo_rawdata::UPPER_BANK, 0x1234u, 0x5678u);
    const timoo_rawdata::raw_block_t lower =
        makeBlockFromWireBytes(timoo_rawdata::LOWER_BANK, 0x0102u, 0x0304u);

    EXPECT_EQ(timoo_rawdata::UPPER_BANK, timoo_rawdata::rawBlockHeader(upper));
    EXPECT_EQ(0x1234u, timoo_rawdata::rawBlockRotation(upper));
    EXPECT_EQ(0x5678u, timoo_rawdata::rawBlockDistance(upper, 0));
    EXPECT_EQ(timoo_rawdata::LOWER_BANK, timoo_rawdata::rawBlockHeader(lower));
    EXPECT_EQ(0x0102u, timoo_rawdata::rawBlockRotation(lower));
    EXPECT_EQ(0x0304u, timoo_rawdata::rawBlockDistance(lower, 0));
}

TEST(TimooRawDataTm16, AcceptsPlausibleModuloRolloverWithoutPreviousDiff)
{
    float azimuth_diff = 0.0f;

    EXPECT_TRUE(timoo_rawdata::resolveTm16AzimuthDiff(35990, 10, 0.0f,
                                                      azimuth_diff));
    EXPECT_FLOAT_EQ(20.0f, azimuth_diff);
}

TEST(TimooRawDataTm16, FallsBackToPreviousDiffForImplausibleReverseJump)
{
    float azimuth_diff = 0.0f;

    EXPECT_TRUE(timoo_rawdata::resolveTm16AzimuthDiff(2000, 1000, 35.0f,
                                                      azimuth_diff));
    EXPECT_FLOAT_EQ(35.0f, azimuth_diff);
}

TEST(TimooRawDataTm16, RejectsImplausibleReverseJumpWithoutPreviousDiff)
{
    float azimuth_diff = 0.0f;

    EXPECT_FALSE(timoo_rawdata::resolveTm16AzimuthDiff(2000, 1000, 0.0f,
                                                       azimuth_diff));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
