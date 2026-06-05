#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "mine_slam_calibration/extrinsics.h"

namespace {

geometry_msgs::TransformStamped toTransformStamped(
    const mine_slam_calibration::TransformSpec& transform) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = transform.parent;
  msg.child_frame_id = transform.child;
  msg.transform.translation.x = transform.translation[0];
  msg.transform.translation.y = transform.translation[1];
  msg.transform.translation.z = transform.translation[2];
  tf2::Quaternion q;
  q.setRPY(transform.rotation_rpy[0], transform.rotation_rpy[1], transform.rotation_rpy[2]);
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  return msg;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_extrinsics");
  ros::NodeHandle private_nh("~");

  std::string extrinsics_file =
      "/home/bai/Desktop/Tunnel-LIO/catkin_ws/src/mine_slam_calibration/config/extrinsics.yaml";
  private_nh.param<std::string>("extrinsics_file", extrinsics_file, extrinsics_file);

  const mine_slam_calibration::ExtrinsicsData data =
      mine_slam_calibration::loadExtrinsics(extrinsics_file);
  const std::vector<std::string> issues = mine_slam_calibration::auditExtrinsics(data);
  if (!issues.empty()) {
    for (const std::string& issue : issues) {
      ROS_ERROR_STREAM("Extrinsics audit failed: " << issue);
    }
    return 1;
  }

  std::vector<geometry_msgs::TransformStamped> transforms;
  transforms.reserve(data.transforms.size());
  for (const mine_slam_calibration::TransformSpec& transform : data.transforms) {
    transforms.push_back(toTransformStamped(transform));
  }

  tf2_ros::StaticTransformBroadcaster broadcaster;
  broadcaster.sendTransform(transforms);
  ROS_INFO_STREAM("Published " << transforms.size() << " static extrinsics from " << extrinsics_file);
  ros::spin();
  return 0;
}
