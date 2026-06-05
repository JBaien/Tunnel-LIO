#include <gtest/gtest.h>

#include <cstring>
#include <limits>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include "slam_backend_manager/backend_point_cloud_reader.h"

namespace slam_backend_manager {
namespace {

void writeFloat(sensor_msgs::PointCloud2* cloud,
                const std::size_t point_index,
                const std::size_t offset,
                const float value) {
  std::memcpy(&cloud->data[point_index * cloud->point_step + offset],
              &value,
              sizeof(value));
}

sensor_msgs::PointField floatField(const std::string& name,
                                   const std::uint32_t offset) {
  sensor_msgs::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.count = 1;
  return field;
}

sensor_msgs::PointCloud2 makeXyziCloud(const std::uint32_t width,
                                       const std::uint32_t height) {
  sensor_msgs::PointCloud2 cloud;
  cloud.width = width;
  cloud.height = height;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 16;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.push_back(floatField("x", 0));
  cloud.fields.push_back(floatField("y", 4));
  cloud.fields.push_back(floatField("z", 8));
  cloud.fields.push_back(floatField("intensity", 12));
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step) * cloud.height);
  return cloud;
}

TEST(BackendPointCloudReader, ReadsFiniteXyziCloud) {
  sensor_msgs::PointCloud2 cloud = makeXyziCloud(2, 1);
  writeFloat(&cloud, 0, 0, 1.0F);
  writeFloat(&cloud, 0, 4, 2.0F);
  writeFloat(&cloud, 0, 8, 3.0F);
  writeFloat(&cloud, 0, 12, 4.0F);
  writeFloat(&cloud, 1, 0, 5.0F);
  writeFloat(&cloud, 1, 4, 6.0F);
  writeFloat(&cloud, 1, 8, 7.0F);
  writeFloat(&cloud, 1, 12, 8.0F);

  const BackendPointCloudReadResult result = readBackendPointCloud2(cloud);

  ASSERT_TRUE(result.ok);
  ASSERT_EQ(2u, result.points.size());
  EXPECT_DOUBLE_EQ(1.0, result.points[0].x);
  EXPECT_DOUBLE_EQ(4.0, result.points[0].intensity);
  EXPECT_DOUBLE_EQ(7.0, result.points[1].z);
}

TEST(BackendPointCloudReader, RejectsTruncatedPointData) {
  sensor_msgs::PointCloud2 cloud = makeXyziCloud(2, 1);
  cloud.data.resize(cloud.data.size() - 1);

  const BackendPointCloudReadResult result = readBackendPointCloud2(cloud);

  EXPECT_FALSE(result.ok);
  EXPECT_EQ("truncated_data", result.reason);
}

TEST(BackendPointCloudReader, RejectsUnreadableFieldLayout) {
  sensor_msgs::PointCloud2 cloud = makeXyziCloud(1, 1);
  cloud.fields[0].offset = 20;

  const BackendPointCloudReadResult result = readBackendPointCloud2(cloud);

  EXPECT_FALSE(result.ok);
  EXPECT_EQ("invalid_field_layout", result.reason);
}

TEST(BackendPointCloudReader, RejectsDuplicateFieldNames) {
  sensor_msgs::PointCloud2 cloud = makeXyziCloud(1, 1);
  cloud.fields.push_back(floatField("intensity", 16));

  const BackendPointCloudReadResult result = readBackendPointCloud2(cloud);

  EXPECT_FALSE(result.ok);
  EXPECT_EQ("duplicate_field", result.reason);
}

TEST(BackendPointCloudReader, RejectsNonFiniteOrNegativeIntensity) {
  sensor_msgs::PointCloud2 cloud = makeXyziCloud(1, 1);
  writeFloat(&cloud, 0, 0, 1.0F);
  writeFloat(&cloud, 0, 4, 2.0F);
  writeFloat(&cloud, 0, 8, 3.0F);
  writeFloat(&cloud, 0, 12, std::numeric_limits<float>::quiet_NaN());

  BackendPointCloudReadResult result = readBackendPointCloud2(cloud);

  EXPECT_FALSE(result.ok);
  EXPECT_EQ("invalid_intensity", result.reason);

  writeFloat(&cloud, 0, 12, -1.0F);
  result = readBackendPointCloud2(cloud);

  EXPECT_FALSE(result.ok);
  EXPECT_EQ("invalid_intensity", result.reason);
}

}  // namespace
}  // namespace slam_backend_manager

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
