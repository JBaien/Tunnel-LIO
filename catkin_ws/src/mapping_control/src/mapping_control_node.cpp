#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "mapping_control/policy.h"

namespace mapping_control {

class MappingControlNode {
 public:
  MappingControlNode() : private_nh_("~") {
    private_nh_.param("section_spacing_m", config_.section_spacing_m, 1.0);
    private_nh_.param("control_anchor_spacing_m", config_.control_anchor_spacing_m, 50.0);
    private_nh_.param("weak_observability_threshold", config_.weak_observability_threshold, 0.3);
    private_nh_.param("max_weak_delta_m", config_.max_weak_delta_m, 0.05);
    private_nh_.param("max_nominal_delta_m", config_.max_nominal_delta_m, 0.5);

    std::string machine_state_topic;
    std::string odom_topic;
    std::string control_topic;
    std::string diagnostics_topic;
    private_nh_.param("machine_state_topic", machine_state_topic, std::string("/machine/state"));
    private_nh_.param("odom_topic", odom_topic, std::string("/lio/odom_local"));
    private_nh_.param("control_topic", control_topic, std::string("/mapping/control"));
    private_nh_.param("diagnostics_topic", diagnostics_topic, std::string("/diagnostics/mapping_control"));

    control_pub_ = nh_.advertise<std_msgs::String>(control_topic, 10);
    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic, 5);
    state_sub_ = nh_.subscribe(machine_state_topic, 10, &MappingControlNode::stateCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic, 20, &MappingControlNode::odomCallback, this);

    double period = 0.1;
    private_nh_.param("publish_period_sec", period, 0.1);
    timer_ = nh_.createTimer(ros::Duration(period), &MappingControlNode::timerCallback, this);
  }

 private:
  void stateCallback(const std_msgs::StringConstPtr& message) {
    machine_state_ = message->data;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    const double x = message->pose.pose.position.x;
    if (!has_last_odom_x_) {
      last_odom_x_ = x;
      has_last_odom_x_ = true;
      return;
    }
    const double requested_delta = x - last_odom_x_;
    last_odom_x_ = x;
    last_decision_ = decideMappingControl(machine_state_, requested_delta, 1.0, chainage_m_, last_section_chainage_m_, config_);
    has_decision_ = true;
    chainage_m_ += std::max(0.0, last_decision_.accepted_delta_m);
    if (last_decision_.section_sample) {
      last_section_chainage_m_ = chainage_m_;
    }
    if (needsControlAnchor(chainage_m_, last_anchor_chainage_m_, config_)) {
      last_anchor_chainage_m_ = chainage_m_;
    }
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!has_decision_) {
      return;
    }
    std_msgs::String message;
    std::ostringstream stream;
    stream << "action=" << last_decision_.action
           << ";machine_state=" << machine_state_
           << ";stable_map_write=" << (last_decision_.stable_map_write ? "true" : "false")
           << ";section_sample=" << (last_decision_.section_sample ? "true" : "false")
           << ";chainage_m=";
    stream.setf(std::ios::fixed);
    stream.precision(3);
    stream << chainage_m_ << ";reason=" << last_decision_.reason;
    message.data = stream.str();
    control_pub_.publish(message);
    publishDiagnostics();
  }

  void publishDiagnostics() {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "mapping_control";
    status.hardware_id = "mapping_policy";
    status.level = (last_decision_.action == REJECT || last_decision_.action == LIMIT) ? diagnostic_msgs::DiagnosticStatus::WARN
                                                                                       : diagnostic_msgs::DiagnosticStatus::OK;
    status.message = last_decision_.action;
    status.values.push_back(keyValue("machine_state", machine_state_));
    status.values.push_back(keyValue("accepted_delta_m", last_decision_.accepted_delta_m));
    status.values.push_back(keyValue("chainage_m", chainage_m_));
    status.values.push_back(keyValue("stable_map_write", last_decision_.stable_map_write ? "true" : "false"));
    status.values.push_back(keyValue("section_sample", last_decision_.section_sample ? "true" : "false"));
    status.values.push_back(keyValue("reason", last_decision_.reason));
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
  ros::Publisher control_pub_;
  ros::Publisher diag_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber odom_sub_;
  ros::Timer timer_;
  MappingPolicyConfig config_;
  std::string machine_state_ = RELOCALIZING;
  double chainage_m_ = 0.0;
  double last_odom_x_ = 0.0;
  bool has_last_odom_x_ = false;
  double last_section_chainage_m_ = -1.0;
  double last_anchor_chainage_m_ = 0.0;
  ControlDecision last_decision_;
  bool has_decision_ = false;
};

}  // namespace mapping_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "mapping_control");
  mapping_control::MappingControlNode node;
  ros::spin();
  return 0;
}
