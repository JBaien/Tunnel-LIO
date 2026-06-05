#include <cmath>
#include <sstream>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "machine_state_manager/state_logic.h"

namespace machine_state_manager {

class MachineStateNode {
 public:
  MachineStateNode() : private_nh_("~") {
    private_nh_.param("static_track_threshold", thresholds_.static_track_threshold, 0.05);
    private_nh_.param("move_track_threshold", thresholds_.move_track_threshold, 0.1);
    private_nh_.param("static_lidar_speed_threshold", thresholds_.static_lidar_speed_threshold, 0.03);
    private_nh_.param("move_lidar_speed_threshold", thresholds_.move_lidar_speed_threshold, 0.08);
    private_nh_.param("turn_track_delta_threshold", thresholds_.turn_track_delta_threshold, 0.12);
    private_nh_.param("imu_static_vibration_threshold", thresholds_.imu_static_vibration_threshold, 0.2);

    std::string state_topic;
    std::string diagnostics_topic;
    std::string left_track_speed_topic;
    std::string right_track_speed_topic;
    std::string cutting_on_topic;
    std::string odom_topic;
    std::string relocalizing_topic;
    private_nh_.param("state_topic", state_topic, std::string("/machine/state"));
    private_nh_.param("diagnostics_topic", diagnostics_topic, std::string("/diagnostics/machine_state"));
    private_nh_.param("left_track_speed_topic", left_track_speed_topic, std::string("/plc/left_track_speed"));
    private_nh_.param("right_track_speed_topic", right_track_speed_topic, std::string("/plc/right_track_speed"));
    private_nh_.param("cutting_on_topic", cutting_on_topic, std::string("/plc/cutting_on"));
    private_nh_.param("odom_topic", odom_topic, std::string("/lio/odom_local"));
    private_nh_.param("relocalizing_topic", relocalizing_topic, std::string("/session/relocalizing"));

    state_pub_ = nh_.advertise<std_msgs::String>(state_topic, 10);
    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic, 5);
    left_sub_ = nh_.subscribe(left_track_speed_topic, 10, &MachineStateNode::leftCallback, this);
    right_sub_ = nh_.subscribe(right_track_speed_topic, 10, &MachineStateNode::rightCallback, this);
    cutting_sub_ = nh_.subscribe(cutting_on_topic, 10, &MachineStateNode::cuttingCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic, 20, &MachineStateNode::odomCallback, this);
    relocalizing_sub_ = nh_.subscribe(relocalizing_topic, 10, &MachineStateNode::relocalizingCallback, this);

    double period = 0.1;
    private_nh_.param("publish_period_sec", period, 0.1);
    timer_ = nh_.createTimer(ros::Duration(period), &MachineStateNode::timerCallback, this);
  }

 private:
  void leftCallback(const std_msgs::Float64ConstPtr& message) {
    signals_.left_track_speed = message->data;
  }

  void rightCallback(const std_msgs::Float64ConstPtr& message) {
    signals_.right_track_speed = message->data;
  }

  void cuttingCallback(const std_msgs::BoolConstPtr& message) {
    signals_.cutting_on = message->data;
  }

  void relocalizingCallback(const std_msgs::BoolConstPtr& message) {
    signals_.relocalizing = message->data;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    const geometry_msgs::Vector3& linear = message->twist.twist.linear;
    signals_.lidar_speed = std::sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z);
  }

  void timerCallback(const ros::TimerEvent&) {
    state_ = classifyMachineState(signals_, thresholds_);
    std_msgs::String state_msg;
    state_msg.data = state_;
    state_pub_.publish(state_msg);
    publishDiagnostics();
  }

  void publishDiagnostics() {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "machine_state_manager";
    status.hardware_id = "crawler";
    status.level = (state_ == CONFLICT || state_ == UNKNOWN) ? diagnostic_msgs::DiagnosticStatus::WARN
                                                             : diagnostic_msgs::DiagnosticStatus::OK;
    status.message = state_;
    status.values.push_back(keyValue("left_track_speed", signals_.left_track_speed));
    status.values.push_back(keyValue("right_track_speed", signals_.right_track_speed));
    status.values.push_back(keyValue("cutting_on", signals_.cutting_on ? "true" : "false"));
    status.values.push_back(keyValue("relocalizing", signals_.relocalizing ? "true" : "false"));
    status.values.push_back(keyValue("lidar_speed", signals_.lidar_speed));
    status.values.push_back(keyValue("stable_map_write", allowsStableMapWrite(state_) ? "true" : "false"));
    status.values.push_back(keyValue("freeze_pose", freezesPose(state_) ? "true" : "false"));
    array.status.push_back(status);
    diag_pub_.publish(array);
  }

  diagnostic_msgs::KeyValue keyValue(const std::string& key, const std::string& value) const {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
  }

  diagnostic_msgs::KeyValue keyValue(const std::string& key, double value) const {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(3);
    stream << value;
    return keyValue(key, stream.str());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher state_pub_;
  ros::Publisher diag_pub_;
  ros::Subscriber left_sub_;
  ros::Subscriber right_sub_;
  ros::Subscriber cutting_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber relocalizing_sub_;
  ros::Timer timer_;
  MachineSignals signals_;
  MachineStateThresholds thresholds_;
  std::string state_ = UNKNOWN;
};

}  // namespace machine_state_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "machine_state_manager");
  machine_state_manager::MachineStateNode node;
  ros::spin();
  return 0;
}
