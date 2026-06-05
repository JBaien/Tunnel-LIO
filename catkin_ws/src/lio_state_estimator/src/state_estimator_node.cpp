#include <algorithm>
#include <sstream>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "lio_state_estimator/imu_window.h"
#include "lio_state_estimator/preintegration.h"

namespace {

diagnostic_msgs::KeyValue kv(const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string format3(const lio_state_estimator::ImuVector& value) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(6);
  stream << value.x << "," << value.y << "," << value.z;
  return stream.str();
}

std::string formatDouble(const double value, const int precision) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(precision);
  stream << value;
  return stream.str();
}

}  // namespace

namespace lio_state_estimator {

class StateEstimatorNode {
 public:
  StateEstimatorNode()
      : private_nh_("~"),
        window_(paramDouble("window_sec", 5.0), paramDouble("min_frequency_hz", 100.0)),
        preintegrator_(paramDouble("gravity_mps2", 9.80665),
                       paramDouble("max_imu_dt_sec", 0.2)) {
    private_nh_.param<std::string>("imu_topic", imu_topic_, "/sensors/imu/raw");
    private_nh_.param<std::string>("state_topic", state_topic_, "/lio/state_predict");
    private_nh_.param<std::string>("diagnostics_topic", diagnostics_topic_,
                                   "/diagnostics/lio_state_estimator");
    private_nh_.param<std::string>("frame_id", frame_id_, "odom");
    private_nh_.param<std::string>("child_frame_id", child_frame_id_, "base_link");

    const double publish_period = paramDouble("publish_period_sec", 0.02);
    const double diagnostics_period = paramDouble("diagnostics_period_sec", 1.0);

    state_pub_ = nh_.advertise<nav_msgs::Odometry>(state_topic_, 20);
    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 5);
    imu_sub_ = nh_.subscribe(imu_topic_, 200, &StateEstimatorNode::imuCallback, this);
    publish_timer_ = nh_.createTimer(ros::Duration(publish_period),
                                     &StateEstimatorNode::publishState, this);
    diagnostics_timer_ = nh_.createTimer(ros::Duration(diagnostics_period),
                                         &StateEstimatorNode::publishDiagnostics, this);
  }

 private:
  double paramDouble(const std::string& name, const double fallback) const {
    double value = fallback;
    private_nh_.param<double>(name, value, fallback);
    return value;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
    last_imu_ = msg;
    const double stamp = msg->header.stamp.toSec();
    window_.add(ImuSample{stamp,
                          msg->linear_acceleration.x,
                          msg->linear_acceleration.y,
                          msg->linear_acceleration.z,
                          msg->angular_velocity.x,
                          msg->angular_velocity.y,
                          msg->angular_velocity.z});
    const ImuWindowStatus status = window_.status();
    if (status.stationary) {
      preintegrator_.setGyroBias(status.gyro_bias);
      preintegrator_.setGravityDirection(status.gravity_direction);
    }
    preintegrator_.observe(stamp,
                           ImuVector{msg->linear_acceleration.x,
                                     msg->linear_acceleration.y,
                                     msg->linear_acceleration.z},
                           ImuVector{msg->angular_velocity.x,
                                     msg->angular_velocity.y,
                                     msg->angular_velocity.z});
  }

  void publishState(const ros::TimerEvent&) {
    if (!last_imu_) {
      return;
    }

    const ImuWindowStatus status = window_.status();
    const ImuPredictionState& prediction = preintegrator_.state();

    nav_msgs::Odometry msg;
    msg.header.stamp = last_imu_->header.stamp;
    msg.header.frame_id = frame_id_;
    msg.child_frame_id = child_frame_id_;
    msg.pose.pose.position.x = prediction.position.x;
    msg.pose.pose.position.y = prediction.position.y;
    msg.pose.pose.position.z = prediction.position.z;
    msg.pose.pose.orientation = last_imu_->orientation;
    msg.twist.twist.linear.x = prediction.velocity.x;
    msg.twist.twist.linear.y = prediction.velocity.y;
    msg.twist.twist.linear.z = prediction.velocity.z;
    msg.twist.twist.angular.x = prediction.angular_velocity.x;
    msg.twist.twist.angular.y = prediction.angular_velocity.y;
    msg.twist.twist.angular.z = prediction.angular_velocity.z;
    msg.pose.covariance[0] = std::max(1e-6, 1.0 - status.health_score);
    msg.twist.covariance[0] = status.gyro_rms;
    state_pub_.publish(msg);
  }

  void publishDiagnostics(const ros::TimerEvent&) {
    const ImuWindowStatus status = window_.status();

    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus item;
    item.name = "lio_state_estimator";
    item.hardware_id = imu_topic_;

    if (status.sample_count == 0) {
      item.level = diagnostic_msgs::DiagnosticStatus::WARN;
      item.message = "waiting for imu";
    } else if (status.regression_count > 0) {
      item.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      item.message = "imu timestamp regression";
    } else if (status.invalid_sample_count > 0) {
      item.level = diagnostic_msgs::DiagnosticStatus::WARN;
      item.message = "invalid imu sample";
    } else if (status.frequency_hz < window_.minFrequencyHz()) {
      item.level = diagnostic_msgs::DiagnosticStatus::WARN;
      item.message = "imu frequency below threshold";
    } else {
      item.level = diagnostic_msgs::DiagnosticStatus::OK;
      item.message = "ok";
    }

    const ImuPredictionState& prediction = preintegrator_.state();
    item.values.push_back(kv("sample_count", std::to_string(status.sample_count)));
    item.values.push_back(kv("duration_sec", formatDouble(status.duration_sec, 3)));
    item.values.push_back(kv("frequency_hz", formatDouble(status.frequency_hz, 3)));
    item.values.push_back(kv("mean_acc", format3(status.mean_acc)));
    item.values.push_back(kv("mean_gyro", format3(status.mean_gyro)));
    item.values.push_back(kv("stationary", status.stationary ? "true" : "false"));
    item.values.push_back(kv("gyro_bias", format3(status.gyro_bias)));
    item.values.push_back(kv("gravity_direction", format3(status.gravity_direction)));
    item.values.push_back(kv("acc_rms_deviation", formatDouble(status.acc_rms_deviation, 6)));
    item.values.push_back(kv("gyro_rms", formatDouble(status.gyro_rms, 6)));
    item.values.push_back(kv("health_score", formatDouble(status.health_score, 3)));
    item.values.push_back(kv("regression_count", std::to_string(status.regression_count)));
    item.values.push_back(kv("invalid_sample_count", std::to_string(status.invalid_sample_count)));
    item.values.push_back(kv("prediction_position", format3(prediction.position)));
    item.values.push_back(kv("prediction_velocity", format3(prediction.velocity)));
    item.values.push_back(kv("prediction_angular_velocity",
                             format3(prediction.angular_velocity)));
    item.values.push_back(kv("preintegration_gyro_bias",
                             format3(prediction.gyro_bias)));
    item.values.push_back(kv("preintegration_gravity_direction",
                             format3(prediction.gravity_direction)));
    item.values.push_back(kv("preintegration_rejected_updates",
                             std::to_string(prediction.rejected_updates)));
    item.values.push_back(kv("preintegration_reset_count", std::to_string(prediction.reset_count)));

    array.status.push_back(item);
    diag_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::string imu_topic_;
  std::string state_topic_;
  std::string diagnostics_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  ImuWindow window_;
  ImuPreintegrator preintegrator_;
  sensor_msgs::ImuConstPtr last_imu_;
  ros::Subscriber imu_sub_;
  ros::Publisher state_pub_;
  ros::Publisher diag_pub_;
  ros::Timer publish_timer_;
  ros::Timer diagnostics_timer_;
};

}  // namespace lio_state_estimator

int main(int argc, char** argv) {
  ros::init(argc, argv, "lio_state_estimator");
  lio_state_estimator::StateEstimatorNode node;
  ros::spin();
  return 0;
}
