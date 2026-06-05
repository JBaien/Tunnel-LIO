#include <gtest/gtest.h>

#include <cstring>
#include <limits>
#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include "lio_preprocess/point_cloud2_reader.h"

namespace {

sensor_msgs::PointField makeField(const std::string& name, uint32_t offset) {
  sensor_msgs::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::PointField::FLOAT32;
  field.count = 1;
  return field;
}

void writeFloat(sensor_msgs::PointCloud2* cloud, std::size_t offset, float value) {
  std::memcpy(&cloud->data[offset], &value, sizeof(float));
}

sensor_msgs::PointCloud2 makeXyzCloud(uint32_t width, uint32_t height) {
  sensor_msgs::PointCloud2 cloud;
  cloud.height = height;
  cloud.width = width;
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.push_back(makeField("x", 0));
  cloud.fields.push_back(makeField("y", 4));
  cloud.fields.push_back(makeField("z", 8));
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step) * cloud.height);
  return cloud;
}

}  // namespace

TEST(PointCloud2Reader, ReadsFiniteCloudAndAppliesFilter) {
  sensor_msgs::PointCloud2 cloud = makeXyzCloud(2, 1);
  writeFloat(&cloud, 0, 2.0f);
  writeFloat(&cloud, 4, 0.0f);
  writeFloat(&cloud, 8, 0.0f);
  writeFloat(&cloud, 12, 0.1f);
  writeFloat(&cloud, 16, 0.0f);
  writeFloat(&cloud, 20, 0.0f);

  lio_preprocess::PointFilterConfig config;
  config.min_range = 0.4;
  config.max_range = 10.0;
  config.enable_body_crop = false;

  lio_preprocess::PointCloud2ReadResult result;
  ASSERT_TRUE(lio_preprocess::readFilteredPointCloud2(cloud, config, &result));

  EXPECT_TRUE(result.ok);
  EXPECT_EQ("", result.reason);
  ASSERT_EQ(1u, result.points.size());
  ASSERT_EQ(3u, result.points[0].size());
  EXPECT_DOUBLE_EQ(2.0, result.points[0][0]);
  EXPECT_EQ(2, result.stats.input_points);
  EXPECT_EQ(1, result.stats.output_points);
  EXPECT_EQ(1, result.stats.dropped_range);
  EXPECT_EQ(0, result.x_index);
  EXPECT_EQ(1, result.y_index);
  EXPECT_EQ(2, result.z_index);
}

TEST(PointCloud2Reader, RejectsDuplicateRequiredFieldNames) {
  sensor_msgs::PointCloud2 cloud = makeXyzCloud(1, 1);
  cloud.point_step = 16;
  cloud.row_step = 16;
  cloud.fields.push_back(makeField("x", 12));
  cloud.data.resize(cloud.row_step);

  lio_preprocess::PointFilterConfig config;
  lio_preprocess::PointCloud2ReadResult result;

  EXPECT_FALSE(lio_preprocess::readFilteredPointCloud2(cloud, config, &result));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ("duplicate_required_field", result.reason);
  EXPECT_TRUE(result.points.empty());
}

TEST(PointCloud2Reader, RejectsTruncatedPointData) {
  sensor_msgs::PointCloud2 cloud = makeXyzCloud(2, 1);
  cloud.data.resize(12);

  lio_preprocess::PointFilterConfig config;
  lio_preprocess::PointCloud2ReadResult result;

  EXPECT_FALSE(lio_preprocess::readFilteredPointCloud2(cloud, config, &result));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ("truncated_data", result.reason);
  EXPECT_TRUE(result.points.empty());
}

TEST(PointCloud2Reader, RejectsNonFloatingRequiredCoordinates) {
  sensor_msgs::PointCloud2 cloud = makeXyzCloud(1, 1);
  cloud.fields[0].datatype = sensor_msgs::PointField::INT32;

  lio_preprocess::PointFilterConfig config;
  lio_preprocess::PointCloud2ReadResult result;

  EXPECT_FALSE(lio_preprocess::readFilteredPointCloud2(cloud, config, &result));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ("unwritable_required_field", result.reason);
  EXPECT_TRUE(result.points.empty());
}

TEST(PointCloud2Reader, RejectsNonFiniteAuxiliaryFields) {
  sensor_msgs::PointCloud2 cloud = makeXyzCloud(1, 1);
  cloud.point_step = 16;
  cloud.row_step = 16;
  cloud.fields.push_back(makeField("intensity", 12));
  cloud.data.resize(cloud.row_step);
  writeFloat(&cloud, 0, 2.0f);
  writeFloat(&cloud, 4, 0.0f);
  writeFloat(&cloud, 8, 0.0f);
  writeFloat(&cloud, 12, std::numeric_limits<float>::quiet_NaN());

  lio_preprocess::PointFilterConfig config;
  lio_preprocess::PointCloud2ReadResult result;

  EXPECT_FALSE(lio_preprocess::readFilteredPointCloud2(cloud, config, &result));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ("nonfinite_field", result.reason);
  EXPECT_TRUE(result.points.empty());
}

TEST(PointCloud2Reader, RejectsDuplicateAuxiliaryFieldNames) {
  sensor_msgs::PointCloud2 cloud = makeXyzCloud(1, 1);
  cloud.point_step = 20;
  cloud.row_step = 20;
  cloud.fields.push_back(makeField("intensity", 12));
  cloud.fields.push_back(makeField("intensity", 16));
  cloud.data.resize(cloud.row_step);
  writeFloat(&cloud, 0, 2.0f);
  writeFloat(&cloud, 4, 0.0f);
  writeFloat(&cloud, 8, 0.0f);
  writeFloat(&cloud, 12, 42.0f);
  writeFloat(&cloud, 16, 43.0f);

  lio_preprocess::PointFilterConfig config;
  lio_preprocess::PointCloud2ReadResult result;

  EXPECT_FALSE(lio_preprocess::readFilteredPointCloud2(cloud, config, &result));
  EXPECT_FALSE(result.ok);
  EXPECT_EQ("duplicate_field", result.reason);
  EXPECT_TRUE(result.points.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
