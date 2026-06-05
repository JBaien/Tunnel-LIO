#include <algorithm>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include "lio_time_manager/time_status.h"

namespace lio_time_manager {

class TimeStatusNode {
 public:
  TimeStatusNode() : private_nh_("~") {
    private_nh_.param("publish_period_sec", publish_period_sec_, 1.0);
    private_nh_.param("stale_after_sec", stale_after_sec_, 1.0);
    private_nh_.param("clock_offset_window_size", clock_offset_window_size_, 100);
    private_nh_.param("enable_pps", enable_pps_, true);
    private_nh_.param("imu_topic", imu_topic_, std::string("/sensors/imu/raw"));
    private_nh_.param("imu_diagnostics_topic",
                      imu_diagnostics_topic_,
                      std::string("/diagnostics/imu_modbus"));
    private_nh_.param("plc_diagnostics_topic",
                      plc_diagnostics_topic_,
                      std::string("/diagnostics/plc_modbus"));
    private_nh_.param("pps_topic", pps_topic_, std::string("/time/pps_event"));
    private_nh_.param("output_topic", output_topic_, std::string("/time/status"));
    cloud_topics_ = readCloudTopics();

    publisher_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(output_topic_, 10);
    for (std::vector<std::string>::const_iterator it = cloud_topics_.begin(); it != cloud_topics_.end(); ++it) {
      addTracker(*it);
      addClockEstimator(*it);
      cloud_subscribers_.push_back(
          nh_.subscribe<sensor_msgs::PointCloud2>(*it, 10, boost::bind(&TimeStatusNode::cloudCallback, this, _1, *it)));
    }
    addTracker(imu_topic_);
    addClockEstimator(imu_topic_);
    imu_subscriber_ = nh_.subscribe<sensor_msgs::Imu>(imu_topic_, 50, boost::bind(&TimeStatusNode::imuCallback, this, _1, imu_topic_));
    addDiagnosticTracker("imu_modbus");
    addDiagnosticTracker("plc_modbus");
    imu_diagnostics_subscriber_ = nh_.subscribe<diagnostic_msgs::DiagnosticArray>(
        imu_diagnostics_topic_, 10,
        boost::bind(&TimeStatusNode::diagnosticsCallback, this, _1, std::string("imu_modbus")));
    plc_diagnostics_subscriber_ = nh_.subscribe<diagnostic_msgs::DiagnosticArray>(
        plc_diagnostics_topic_, 10,
        boost::bind(&TimeStatusNode::diagnosticsCallback, this, _1, std::string("plc_modbus")));
    if (enable_pps_) {
      addClockEstimator(pps_topic_);
      addPpsTracker(pps_topic_);
      pps_subscriber_ = nh_.subscribe<std_msgs::Header>(pps_topic_, 10, &TimeStatusNode::ppsCallback, this);
    }

    timer_ = nh_.createTimer(ros::Duration(publish_period_sec_), &TimeStatusNode::timerCallback, this);
  }

 private:
  std::vector<std::string> readCloudTopics() const {
    std::vector<std::string> topics;
    XmlRpc::XmlRpcValue value;
    if (private_nh_.getParam("cloud_topics", value) && value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < value.size(); ++i) {
        if (value[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
          topics.push_back(static_cast<std::string>(value[i]));
        }
      }
    }
    if (topics.empty()) {
      topics.push_back("/points_raw");
    }
    std::sort(topics.begin(), topics.end());
    topics.erase(std::unique(topics.begin(), topics.end()), topics.end());
    return topics;
  }

  void addTracker(const std::string& name) {
    trackers_.insert(std::make_pair(name, SensorTimeTracker(name, stale_after_sec_)));
  }

  void addClockEstimator(const std::string& name) {
    clock_estimators_.insert(std::make_pair(name, ClockOffsetEstimator(name, static_cast<std::size_t>(clock_offset_window_size_))));
  }

  void addDiagnosticTracker(const std::string& name) {
    diagnostic_trackers_.insert(
        std::make_pair(name, DiagnosticFeedTracker(name, stale_after_sec_)));
  }

  void addPpsTracker(const std::string& name) {
    pps_trackers_.insert(
        std::make_pair(name, PpsEventTracker(name, stale_after_sec_)));
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message, const std::string& name) {
    observe(name, message->header.stamp.toSec());
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& message, const std::string& name) {
    observe(name, message->header.stamp.toSec());
  }

  void ppsCallback(const std_msgs::HeaderConstPtr& message) {
    const double receipt_time = ros::Time::now().toSec();
    const double event_time = message->stamp.toSec();
    std::map<std::string, PpsEventTracker>::iterator it =
        pps_trackers_.find(pps_topic_);
    if (it != pps_trackers_.end()) {
      it->second.observe(event_time, receipt_time);
    }
    observeClock(pps_topic_, event_time, receipt_time);
  }

  void diagnosticsCallback(const diagnostic_msgs::DiagnosticArrayConstPtr& message,
                           const std::string& name) {
    if (message->status.empty()) {
      return;
    }
    std::map<std::string, DiagnosticFeedTracker>::iterator tracker =
        diagnostic_trackers_.find(name);
    if (tracker == diagnostic_trackers_.end()) {
      return;
    }
    const diagnostic_msgs::DiagnosticStatus& source = message->status.front();
    std::vector<DiagnosticKeyValue> values;
    for (std::vector<diagnostic_msgs::KeyValue>::const_iterator it =
             source.values.begin();
         it != source.values.end(); ++it) {
      DiagnosticKeyValue item;
      item.key = it->key;
      item.value = it->value;
      values.push_back(item);
    }
    tracker->second.observe(ros::Time::now().toSec(),
                            fromDiagnosticLevel(source.level),
                            source.message,
                            values);
  }

  void observe(const std::string& name, double sensor_stamp) {
    const double receipt_time = ros::Time::now().toSec();
    std::map<std::string, SensorTimeTracker>::iterator it = trackers_.find(name);
    if (it != trackers_.end()) {
      it->second.observe(sensor_stamp, receipt_time);
    }
    observeClock(name, sensor_stamp, receipt_time);
  }

  void observeClock(const std::string& name, double device_time, double host_time) {
    std::map<std::string, ClockOffsetEstimator>::iterator it = clock_estimators_.find(name);
    if (it != clock_estimators_.end()) {
      it->second.observe(device_time, host_time);
    }
  }

  void timerCallback(const ros::TimerEvent&) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    const double now = array.header.stamp.toSec();
    for (std::map<std::string, SensorTimeTracker>::const_iterator it = trackers_.begin(); it != trackers_.end(); ++it) {
      array.status.push_back(toDiagnosticStatus(it->second.status(now)));
    }
    for (std::map<std::string, ClockOffsetEstimator>::const_iterator it = clock_estimators_.begin();
         it != clock_estimators_.end(); ++it) {
      array.status.push_back(toDiagnosticStatus(it->second.status()));
    }
    for (std::map<std::string, PpsEventTracker>::const_iterator it =
             pps_trackers_.begin();
         it != pps_trackers_.end(); ++it) {
      array.status.push_back(toDiagnosticStatus(it->second.status(now)));
    }
    for (std::map<std::string, DiagnosticFeedTracker>::const_iterator it =
             diagnostic_trackers_.begin();
         it != diagnostic_trackers_.end(); ++it) {
      array.status.push_back(toDiagnosticStatus(it->second.status(now)));
    }
    publisher_.publish(array);
  }

  diagnostic_msgs::DiagnosticStatus toDiagnosticStatus(const SensorTimeStatus& state) const {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "lio_time_manager: " + state.name;
    status.hardware_id = state.name;
    if (state.time_went_backwards) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "sensor time went backwards";
    } else if (state.stale) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "sensor stale";
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "ok";
    }
    status.values.push_back(keyValue("frequency_hz", state.frequency_hz));
    status.values.push_back(keyValue("last_latency_ms", state.last_latency_ms));
    status.values.push_back(keyValue("sample_count", state.sample_count));
    status.values.push_back(keyValue("regression_count", state.regression_count));
    return status;
  }

  diagnostic_msgs::DiagnosticStatus toDiagnosticStatus(const PpsEventStatus& state) const {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "lio_time_manager pps: " + state.name;
    status.hardware_id = state.name;
    if (state.time_went_backwards) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "pps event time went backwards";
    } else if (state.stale) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "pps stale";
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "ok";
    }
    status.values.push_back(keyValue("frequency_hz", state.frequency_hz));
    status.values.push_back(keyValue("latest_interval_ms", state.latest_interval_ms));
    status.values.push_back(keyValue("interval_jitter_ms", state.interval_jitter_ms));
    status.values.push_back(keyValue("sample_count", state.sample_count));
    status.values.push_back(keyValue("regression_count", state.regression_count));
    return status;
  }

  diagnostic_msgs::DiagnosticStatus toDiagnosticStatus(const DiagnosticFeedStatus& state) const {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "lio_time_manager diagnostic: " + state.name;
    status.hardware_id = state.name;
    const DiagnosticLevel level = state.effectiveLevel();
    status.level = toRosDiagnosticLevel(level);
    if (!state.received) {
      status.message = "waiting for diagnostics";
    } else if (state.stale) {
      status.message = "diagnostics stale";
    } else {
      status.message = state.message;
    }
    status.values.push_back(keyValue("received", state.received ? "true" : "false"));
    status.values.push_back(keyValue("stale", state.stale ? "true" : "false"));
    status.values.push_back(keyValue("publish_rate_hz", state.publish_rate_hz));
    status.values.push_back(keyValue("last_read_latency_ms", state.last_read_latency_ms));
    status.values.push_back(keyValue("mean_read_latency_ms", state.mean_read_latency_ms));
    status.values.push_back(keyValue("max_read_latency_ms", state.max_read_latency_ms));
    status.values.push_back(keyValue("read_error_count", state.read_error_count));
    status.values.push_back(keyValue("invalid_frame_count", state.invalid_frame_count));
    status.values.push_back(keyValue("saturation_count", state.saturation_count));
    status.values.push_back(keyValue("temperature_sample_count", state.temperature_sample_count));
    status.values.push_back(keyValue("temperature_warning_count", state.temperature_warning_count));
    status.values.push_back(keyValue("latest_temperature_c", state.latest_temperature_c));
    status.values.push_back(keyValue("min_temperature_c", state.min_temperature_c));
    status.values.push_back(keyValue("max_temperature_c", state.max_temperature_c));
    status.values.push_back(keyValue("orientation_covariance_x", state.orientation_covariance_x));
    status.values.push_back(keyValue("orientation_covariance_y", state.orientation_covariance_y));
    status.values.push_back(keyValue("orientation_covariance_z", state.orientation_covariance_z));
    status.values.push_back(keyValue("angular_velocity_covariance_x", state.angular_velocity_covariance_x));
    status.values.push_back(keyValue("angular_velocity_covariance_y", state.angular_velocity_covariance_y));
    status.values.push_back(keyValue("angular_velocity_covariance_z", state.angular_velocity_covariance_z));
    status.values.push_back(keyValue("linear_acceleration_covariance_x", state.linear_acceleration_covariance_x));
    status.values.push_back(keyValue("linear_acceleration_covariance_y", state.linear_acceleration_covariance_y));
    status.values.push_back(keyValue("linear_acceleration_covariance_z", state.linear_acceleration_covariance_z));
    status.values.push_back(keyValue("reconnect_attempt_count", state.reconnect_attempt_count));
    status.values.push_back(keyValue("reconnect_success_count", state.reconnect_success_count));
    status.values.push_back(keyValue("timestamp_source", state.timestamp_source));
    status.values.push_back(keyValue("hardware_time_status", state.hardware_time_status));
    status.values.push_back(keyValue("pps_status", state.pps_status));
    return status;
  }

  diagnostic_msgs::DiagnosticStatus toDiagnosticStatus(const ClockOffsetStatus& state) const {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "lio_time_manager clock: " + state.name;
    status.hardware_id = state.name;
    if (state.device_time_went_backwards) {
      status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      status.message = "device time went backwards";
    } else if (!state.valid) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "waiting for clock samples";
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "ok";
    }
    status.values.push_back(keyValue("mean_offset_ms", state.mean_offset_ms));
    status.values.push_back(keyValue("jitter_ms", state.jitter_ms));
    status.values.push_back(keyValue("latest_device_time", state.latest_device_time));
    status.values.push_back(keyValue("latest_host_time", state.latest_host_time));
    status.values.push_back(keyValue("sample_count", state.sample_count));
    status.values.push_back(keyValue("regression_count", state.regression_count));
    return status;
  }

  diagnostic_msgs::KeyValue keyValue(const std::string& key, double value) const {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(3);
    stream << value;
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = stream.str();
    return kv;
  }

  diagnostic_msgs::KeyValue keyValue(const std::string& key, int value) const {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = std::to_string(value);
    return kv;
  }

  diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                     const std::string& value) const {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
  }

  DiagnosticLevel fromDiagnosticLevel(const int level) const {
    if (level >= diagnostic_msgs::DiagnosticStatus::ERROR) {
      return DiagnosticLevel::ERROR;
    }
    if (level >= diagnostic_msgs::DiagnosticStatus::WARN) {
      return DiagnosticLevel::WARN;
    }
    return DiagnosticLevel::OK;
  }

  int toRosDiagnosticLevel(const DiagnosticLevel level) const {
    if (level == DiagnosticLevel::ERROR) {
      return diagnostic_msgs::DiagnosticStatus::ERROR;
    }
    if (level == DiagnosticLevel::WARN) {
      return diagnostic_msgs::DiagnosticStatus::WARN;
    }
    return diagnostic_msgs::DiagnosticStatus::OK;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher publisher_;
  std::vector<ros::Subscriber> cloud_subscribers_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber imu_diagnostics_subscriber_;
  ros::Subscriber plc_diagnostics_subscriber_;
  ros::Subscriber pps_subscriber_;
  ros::Timer timer_;
  std::map<std::string, SensorTimeTracker> trackers_;
  std::map<std::string, ClockOffsetEstimator> clock_estimators_;
  std::map<std::string, DiagnosticFeedTracker> diagnostic_trackers_;
  std::map<std::string, PpsEventTracker> pps_trackers_;
  std::vector<std::string> cloud_topics_;
  std::string imu_topic_;
  std::string imu_diagnostics_topic_;
  std::string plc_diagnostics_topic_;
  std::string pps_topic_;
  std::string output_topic_;
  double publish_period_sec_ = 1.0;
  double stale_after_sec_ = 1.0;
  int clock_offset_window_size_ = 100;
  bool enable_pps_ = true;
};

}  // namespace lio_time_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "time_status_node");
  lio_time_manager::TimeStatusNode node;
  ros::spin();
  return 0;
}
