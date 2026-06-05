#include <gtest/gtest.h>
#include <timoo_pointcloud/rawdata.h>

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
