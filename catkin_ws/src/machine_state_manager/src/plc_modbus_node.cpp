#include <cerrno>
#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <modbus/modbus.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "machine_state_manager/plc_modbus_parser.h"

namespace machine_state_manager {
namespace {

diagnostic_msgs::KeyValue keyValue(const std::string& key,
                                   const std::string& value) {
  diagnostic_msgs::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

diagnostic_msgs::KeyValue keyValue(const std::string& key, const double value) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(3);
  stream << value;
  return keyValue(key, stream.str());
}

diagnostic_msgs::KeyValue keyValue(const std::string& key, const int value) {
  return keyValue(key, std::to_string(value));
}

}  // namespace

class PlcModbusNode {
 public:
  PlcModbusNode() : private_nh_("~") {
    loadParameters();
    left_pub_ = nh_.advertise<std_msgs::Float64>(left_track_speed_topic_, 10);
    right_pub_ = nh_.advertise<std_msgs::Float64>(right_track_speed_topic_, 10);
    cutting_pub_ = nh_.advertise<std_msgs::Bool>(cutting_on_topic_, 10);
    diagnostics_pub_ =
        nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 10);
    register_data_.resize(num_registers_, 0U);
  }

  ~PlcModbusNode() {
    closeConnection();
  }

  void spin() {
    ros::Rate rate(poll_rate_hz_);
    while (ros::ok()) {
      if (!connected_ && !connect()) {
        publishDiagnostics("disconnected", diagnostic_msgs::DiagnosticStatus::ERROR);
        ros::spinOnce();
        sleepForReconnect();
        continue;
      }

      if (readRegisters()) {
        const PlcSignals signals = parsePlcRegisters(register_data_, parser_config_);
        if (signals.enough_registers && signals.data_valid) {
          publishSignals(signals);
          last_valid_read_ = ros::Time::now();
          publishDiagnostics("ok", diagnostic_msgs::DiagnosticStatus::OK);
        } else {
          ++invalid_frame_count_;
          publishDiagnostics(!signals.config_valid ? "plc_config_invalid"
                             : signals.enough_registers ? "plc_data_invalid"
                                                        : "plc_frame_too_short",
                             diagnostic_msgs::DiagnosticStatus::WARN);
        }
      } else {
        ++read_error_count_;
        connected_ = false;
        publishDiagnostics("read_error", diagnostic_msgs::DiagnosticStatus::ERROR);
      }

      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  void loadParameters() {
    private_nh_.param<std::string>("plc/ip_address", ip_address_, "192.168.188.20");
    private_nh_.param("plc/port", port_, 502);
    private_nh_.param("plc/start_register", start_register_, 0);
    private_nh_.param("plc/num_registers", num_registers_, 3);
    private_nh_.param("plc/poll_rate_hz", poll_rate_hz_, 20.0);
    private_nh_.param("plc/response_timeout_ms", response_timeout_ms_, 200);
    private_nh_.param("plc/reconnect_delay_ms", reconnect_delay_ms_, 1000);
    private_nh_.param<std::string>("left_track_speed_topic",
                                   left_track_speed_topic_,
                                   "/plc/left_track_speed");
    private_nh_.param<std::string>("right_track_speed_topic",
                                   right_track_speed_topic_,
                                   "/plc/right_track_speed");
    private_nh_.param<std::string>("cutting_on_topic",
                                   cutting_on_topic_,
                                   "/plc/cutting_on");
    private_nh_.param<std::string>("plc/diagnostics_topic",
                                   diagnostics_topic_,
                                   "/diagnostics/plc_modbus");
    private_nh_.param("plc/left_track_register_index",
                      parser_config_.left_track_register_index,
                      parser_config_.left_track_register_index);
    private_nh_.param("plc/right_track_register_index",
                      parser_config_.right_track_register_index,
                      parser_config_.right_track_register_index);
    private_nh_.param("plc/status_register_index",
                      parser_config_.status_register_index,
                      parser_config_.status_register_index);
    private_nh_.param("plc/cutting_on_bit",
                      parser_config_.cutting_on_bit,
                      parser_config_.cutting_on_bit);
    private_nh_.param("plc/valid_bit", parser_config_.valid_bit, parser_config_.valid_bit);
    private_nh_.param("plc/track_speed_scale",
                      parser_config_.track_speed_scale,
                      parser_config_.track_speed_scale);

    if (num_registers_ <= 0) {
      ROS_WARN("num_registers <= 0, reset to 3");
      num_registers_ = 3;
    }
    if (poll_rate_hz_ <= 0.0) {
      ROS_WARN("poll_rate_hz <= 0, reset to 20 Hz");
      poll_rate_hz_ = 20.0;
    }
  }

  bool connect() {
    closeConnection();
    ctx_ = modbus_new_tcp(ip_address_.c_str(), port_);
    if (!ctx_) {
      ROS_ERROR("failed to allocate PLC Modbus context");
      return false;
    }
    modbus_set_response_timeout(ctx_, response_timeout_ms_ / 1000,
                                (response_timeout_ms_ % 1000) * 1000);
    modbus_set_byte_timeout(ctx_, response_timeout_ms_ / 1000,
                            (response_timeout_ms_ % 1000) * 1000);
    if (modbus_connect(ctx_) == -1) {
      ROS_ERROR("PLC Modbus connection failed: %s", modbus_strerror(errno));
      closeConnection();
      return false;
    }
    connected_ = true;
    ROS_INFO("connected to PLC Modbus at %s:%d", ip_address_.c_str(), port_);
    return true;
  }

  void closeConnection() {
    if (ctx_) {
      modbus_close(ctx_);
      modbus_free(ctx_);
      ctx_ = nullptr;
    }
    connected_ = false;
  }

  bool readRegisters() {
    if (!ctx_ || !connected_) {
      return false;
    }
    const ros::WallTime started = ros::WallTime::now();
    const int ret = modbus_read_registers(ctx_, start_register_, num_registers_,
                                          register_data_.data());
    last_read_latency_ms_ = (ros::WallTime::now() - started).toSec() * 1000.0;
    return ret == num_registers_;
  }

  void publishSignals(const PlcSignals& signals) {
    std_msgs::Float64 left;
    left.data = signals.left_track_speed;
    left_pub_.publish(left);

    std_msgs::Float64 right;
    right.data = signals.right_track_speed;
    right_pub_.publish(right);

    std_msgs::Bool cutting;
    cutting.data = signals.cutting_on;
    cutting_pub_.publish(cutting);
  }

  void publishDiagnostics(const std::string& message, const int level) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus status;
    status.name = "plc_modbus";
    status.hardware_id = ip_address_;
    status.level = level;
    status.message = message;
    status.values.push_back(keyValue("connected", connected_ ? "true" : "false"));
    status.values.push_back(keyValue("read_error_count", read_error_count_));
    status.values.push_back(keyValue("invalid_frame_count", invalid_frame_count_));
    status.values.push_back(keyValue("last_read_latency_ms", last_read_latency_ms_));
    status.values.push_back(keyValue("start_register", start_register_));
    status.values.push_back(keyValue("num_registers", num_registers_));
    status.values.push_back(keyValue("parser_config_valid",
                                     validPlcModbusConfig(parser_config_) ? "true" : "false"));
    status.values.push_back(keyValue("last_valid_read_age_s", lastValidReadAge()));
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  double lastValidReadAge() const {
    if (last_valid_read_.isZero()) {
      return -1.0;
    }
    return (ros::Time::now() - last_valid_read_).toSec();
  }

  void sleepForReconnect() const {
    std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms_));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher left_pub_;
  ros::Publisher right_pub_;
  ros::Publisher cutting_pub_;
  ros::Publisher diagnostics_pub_;

  modbus_t* ctx_ = nullptr;
  bool connected_ = false;
  std::vector<uint16_t> register_data_;
  PlcModbusConfig parser_config_;

  std::string ip_address_;
  int port_ = 502;
  int start_register_ = 0;
  int num_registers_ = 3;
  double poll_rate_hz_ = 20.0;
  int response_timeout_ms_ = 200;
  int reconnect_delay_ms_ = 1000;
  std::string left_track_speed_topic_;
  std::string right_track_speed_topic_;
  std::string cutting_on_topic_;
  std::string diagnostics_topic_;

  int read_error_count_ = 0;
  int invalid_frame_count_ = 0;
  double last_read_latency_ms_ = 0.0;
  ros::Time last_valid_read_;
};

}  // namespace machine_state_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "plc_modbus_node");
  machine_state_manager::PlcModbusNode node;
  node.spin();
  return 0;
}
