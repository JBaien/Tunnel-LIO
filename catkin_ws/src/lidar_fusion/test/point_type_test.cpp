#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "lidar_fusion/lidar_fusion.h"

namespace {

bool hasField(const sensor_msgs::PointCloud2& cloud, const std::string& name) {
  return std::find_if(cloud.fields.begin(), cloud.fields.end(),
                      [&name](const sensor_msgs::PointField& field) {
                        return field.name == name;
                      }) != cloud.fields.end();
}

sensor_msgs::PointCloud2 makeXyziCloud() {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.stamp = ros::Time(10.0);
  cloud.header.frame_id = "laser_link";
  cloud.height = 1;
  cloud.width = 2;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(2);

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
  for (size_t i = 0; i < 2; ++i, ++x, ++y, ++z, ++intensity) {
    *x = static_cast<float>(i + 1);
    *y = static_cast<float>(i + 2);
    *z = static_cast<float>(i + 3);
    *intensity = static_cast<float>(40 + i);
  }
  return cloud;
}

}  // namespace

TEST(MultiLidarFusionPointType, FusedPointCloudExportsSensorIdRingAndTime) {
  pcl::PointCloud<lidar_fusion::PointXYZIRTS> cloud;
  lidar_fusion::PointXYZIRTS point;
  point.x = 1.0f;
  point.y = 2.0f;
  point.z = 3.0f;
  point.intensity = 42.0f;
  point.ring = 7;
  point.time = 0.012f;
  point.sensor_id = 2;
  cloud.push_back(point);
  cloud.width = 1;
  cloud.height = 1;

  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);

  EXPECT_TRUE(hasField(message, "x"));
  EXPECT_TRUE(hasField(message, "y"));
  EXPECT_TRUE(hasField(message, "z"));
  EXPECT_TRUE(hasField(message, "intensity"));
  EXPECT_TRUE(hasField(message, "ring"));
  EXPECT_TRUE(hasField(message, "time"));
  EXPECT_TRUE(hasField(message, "sensor_id"));
}

TEST(MultiLidarFusionPointType, MakeFusedPointPreservesTimingAndSource) {
  lidar_fusion::PointXYZIRT source;
  source.x = 1.0f;
  source.y = 2.0f;
  source.z = 3.0f;
  source.intensity = 17.0f;
  source.ring = 9;
  source.time = 0.020f;

  const lidar_fusion::PointXYZIRTS fused =
      lidar_fusion::makeFusedPoint(source, Eigen::Vector3f(4.0f, 5.0f, 6.0f),
                                   0.003f, 1);

  EXPECT_FLOAT_EQ(4.0f, fused.x);
  EXPECT_FLOAT_EQ(5.0f, fused.y);
  EXPECT_FLOAT_EQ(6.0f, fused.z);
  EXPECT_FLOAT_EQ(17.0f, fused.intensity);
  EXPECT_EQ(9, fused.ring);
  EXPECT_FLOAT_EQ(0.023f, fused.time);
  EXPECT_EQ(1, fused.sensor_id);
}

TEST(MultiLidarFusionPointType, RejectsXyziCloudWhenLegacyCompatibilityDisabled) {
  const sensor_msgs::PointCloud2 cloud = makeXyziCloud();
  pcl::PointCloud<lidar_fusion::PointXYZIRT> converted;
  bool used_legacy_defaults = true;

  EXPECT_FALSE(lidar_fusion::convertToFusionInputCloud(
      cloud, false, converted, &used_legacy_defaults));
  EXPECT_FALSE(used_legacy_defaults);
}

TEST(MultiLidarFusionPointType, ConvertsXyziCloudWithLegacyDefaultsWhenEnabled) {
  const sensor_msgs::PointCloud2 cloud = makeXyziCloud();
  pcl::PointCloud<lidar_fusion::PointXYZIRT> converted;
  bool used_legacy_defaults = false;

  ASSERT_TRUE(lidar_fusion::convertToFusionInputCloud(
      cloud, true, converted, &used_legacy_defaults));

  EXPECT_TRUE(used_legacy_defaults);
  ASSERT_EQ(2u, converted.points.size());
  EXPECT_EQ(2u, converted.width);
  EXPECT_EQ(1u, converted.height);
  EXPECT_FLOAT_EQ(1.0f, converted.points[0].x);
  EXPECT_FLOAT_EQ(2.0f, converted.points[0].y);
  EXPECT_FLOAT_EQ(3.0f, converted.points[0].z);
  EXPECT_FLOAT_EQ(40.0f, converted.points[0].intensity);
  EXPECT_EQ(0u, converted.points[0].ring);
  EXPECT_FLOAT_EQ(0.0f, converted.points[0].time);
}

TEST(MultiLidarFusionDiagnostics, FormatsReplayFriendlyKeyValuePayload) {
  lidar_fusion::FusionDiagnosticSnapshot snapshot;
  snapshot.sync_callbacks = 12;
  snapshot.published = 10;
  snapshot.last_output_points = 345;
  snapshot.last_sync_span = 0.024;
  snapshot.overlap_pairs = 8;
  snapshot.overlap_rmse = 0.035;
  snapshot.overlap_max = 0.08;
  snapshot.overlap_status = "warn";
  snapshot.last_input_points = {100, 110, 105};
  snapshot.last_input_frames = {"lidar_center", "lidar_left", "lidar_right"};
  snapshot.dropped_empty = 1;
  snapshot.dropped_min_points = 2;
  snapshot.dropped_field = 3;
  snapshot.dropped_tf = 4;
  snapshot.dropped_conversion = 5;
  snapshot.dropped_exception = 6;
  snapshot.legacy_xyzi_clouds = 7;
  snapshot.legacy_xyzi_points = 890;

  const std::string payload = lidar_fusion::formatFusionDiagnostics(snapshot);

  EXPECT_NE(std::string::npos, payload.find("callbacks=12"));
  EXPECT_NE(std::string::npos, payload.find("published=10"));
  EXPECT_NE(std::string::npos, payload.find("last_output_points=345"));
  EXPECT_NE(std::string::npos, payload.find("last_sync_span=0.024"));
  EXPECT_NE(std::string::npos, payload.find("overlap_pairs=8"));
  EXPECT_NE(std::string::npos, payload.find("overlap_rmse=0.035"));
  EXPECT_NE(std::string::npos, payload.find("overlap_max=0.08"));
  EXPECT_NE(std::string::npos, payload.find("overlap_status=warn"));
  EXPECT_NE(std::string::npos, payload.find("input0_points=100"));
  EXPECT_NE(std::string::npos, payload.find("input1_frame=lidar_left"));
  EXPECT_NE(std::string::npos, payload.find("dropped_tf=4"));
  EXPECT_NE(std::string::npos, payload.find("dropped_exception=6"));
  EXPECT_NE(std::string::npos, payload.find("legacy_xyzi_clouds=7"));
  EXPECT_NE(std::string::npos, payload.find("legacy_xyzi_points=890"));
}

TEST(MultiLidarFusionDiagnostics, ComputesCrossSensorOverlapResidual) {
  pcl::PointCloud<lidar_fusion::PointXYZIRTS> cloud;
  lidar_fusion::PointXYZIRTS center;
  center.x = 0.0f;
  center.y = 0.0f;
  center.z = 0.0f;
  center.sensor_id = 0;

  lidar_fusion::PointXYZIRTS left = center;
  left.x = 0.03f;
  left.sensor_id = 1;

  lidar_fusion::PointXYZIRTS right = center;
  right.y = 0.04f;
  right.sensor_id = 2;

  lidar_fusion::PointXYZIRTS far = center;
  far.x = 5.0f;
  far.sensor_id = 1;

  cloud.push_back(center);
  cloud.push_back(left);
  cloud.push_back(right);
  cloud.push_back(far);

  const lidar_fusion::FusionOverlapResidual residual =
      lidar_fusion::computeOverlapResidual(cloud, 0.10);

  EXPECT_EQ(2u, residual.pairs);
  EXPECT_NEAR(std::sqrt((0.03 * 0.03 + 0.04 * 0.04) / 2.0),
              residual.rmse, 1e-6);
  EXPECT_NEAR(0.04, residual.max_distance, 1e-6);
}

TEST(MultiLidarFusionDiagnostics, BoundsOverlapResidualWithStridedSample) {
  pcl::PointCloud<lidar_fusion::PointXYZIRTS> cloud;
  for (int i = 0; i < 100; ++i) {
    lidar_fusion::PointXYZIRTS center;
    center.x = 0.05f * static_cast<float>(i);
    center.y = 0.0f;
    center.z = 0.0f;
    center.sensor_id = 0;
    cloud.push_back(center);
  }
  for (int i = 0; i < 100; ++i) {
    lidar_fusion::PointXYZIRTS left;
    left.x = 0.05f * static_cast<float>(i) + 0.02f;
    left.y = 0.0f;
    left.z = 0.0f;
    left.sensor_id = 1;
    cloud.push_back(left);
  }

  const lidar_fusion::FusionOverlapResidual full =
      lidar_fusion::computeOverlapResidual(cloud, 0.05);
  const lidar_fusion::FusionOverlapResidual sampled =
      lidar_fusion::computeOverlapResidual(cloud, 0.05, 20);
  const lidar_fusion::FusionOverlapResidual too_small =
      lidar_fusion::computeOverlapResidual(cloud, 0.05, 1);

  EXPECT_GT(full.pairs, 0u);
  EXPECT_GT(sampled.pairs, 0u);
  EXPECT_LT(sampled.pairs, full.pairs);
  EXPECT_EQ(0u, too_small.pairs);
}

TEST(MultiLidarFusionDiagnostics, RejectsInvalidOverlapDistance) {
  pcl::PointCloud<lidar_fusion::PointXYZIRTS> cloud;
  lidar_fusion::PointXYZIRTS center;
  center.x = 0.0f;
  center.y = 0.0f;
  center.z = 0.0f;
  center.sensor_id = 0;

  lidar_fusion::PointXYZIRTS far = center;
  far.x = 100.0f;
  far.sensor_id = 1;

  cloud.push_back(center);
  cloud.push_back(far);

  const lidar_fusion::FusionOverlapResidual inf_distance =
      lidar_fusion::computeOverlapResidual(
          cloud, std::numeric_limits<double>::infinity());
  EXPECT_EQ(0u, inf_distance.pairs);
  EXPECT_DOUBLE_EQ(0.0, inf_distance.rmse);
  EXPECT_DOUBLE_EQ(0.0, inf_distance.max_distance);

  const lidar_fusion::FusionOverlapResidual nan_distance =
      lidar_fusion::computeOverlapResidual(
          cloud, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(0u, nan_distance.pairs);
  EXPECT_DOUBLE_EQ(0.0, nan_distance.rmse);
  EXPECT_DOUBLE_EQ(0.0, nan_distance.max_distance);
}

TEST(MultiLidarFusionDiagnostics, FormatsDiagnosticsFailClosedForPollutedValues) {
  lidar_fusion::FusionDiagnosticSnapshot snapshot;
  snapshot.last_sync_span = std::numeric_limits<double>::quiet_NaN();
  snapshot.overlap_rmse = std::numeric_limits<double>::infinity();
  snapshot.overlap_max = -1.0;
  snapshot.overlap_status = "ok;published=999";
  snapshot.last_input_points = {10};
  snapshot.last_input_frames = {"lidar_center\npublished=999"};

  const std::string payload = lidar_fusion::formatFusionDiagnostics(snapshot);

  EXPECT_EQ(std::string::npos, payload.find("nan"));
  EXPECT_EQ(std::string::npos, payload.find("inf"));
  EXPECT_EQ(std::string::npos, payload.find("\n"));
  EXPECT_EQ(std::string::npos, payload.find(";published=999"));
  EXPECT_NE(std::string::npos, payload.find("last_sync_span=0"));
  EXPECT_NE(std::string::npos, payload.find("overlap_rmse=0"));
  EXPECT_NE(std::string::npos, payload.find("overlap_max=0"));
  EXPECT_NE(std::string::npos, payload.find("overlap_status=invalid"));
  EXPECT_NE(std::string::npos, payload.find("input0_frame=invalid"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
