#include <algorithm>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include <XmlRpcValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include "lio_preprocess/deskewing.h"
#include "lio_preprocess/filtering.h"
#include "lio_preprocess/point_cloud2_reader.h"

namespace lio_preprocess {

class PreprocessNode {
 public:
  PreprocessNode() : private_nh_("~") {
    loadConfig();
    private_nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/points_raw"));
    private_nh_.param("imu_topic", imu_topic_, std::string("/sensors/imu/raw"));
    private_nh_.param("output_cloud_topic", output_cloud_topic_, std::string("/lio/points_deskewed"));
    private_nh_.param("diagnostics_topic", diagnostics_topic_, std::string("/diagnostics/lio_preprocess"));
    private_nh_.param("output_frame_id", output_frame_id_, std::string("base_link"));
    private_nh_.param("require_recent_imu", require_recent_imu_, false);
    private_nh_.param("max_imu_age_sec", max_imu_age_sec_, 0.2);
    private_nh_.param("imu_buffer_size", imu_buffer_size_, 400);
    double diagnostics_period = 1.0;
    private_nh_.param("diagnostics_period_sec", diagnostics_period, 1.0);

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 5);
    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 5);
    imu_sub_ = nh_.subscribe(imu_topic_, 100, &PreprocessNode::imuCallback, this);
    cloud_sub_ = nh_.subscribe(input_cloud_topic_, 5, &PreprocessNode::cloudCallback, this);
    diag_timer_ = nh_.createTimer(ros::Duration(diagnostics_period), &PreprocessNode::diagnosticsCallback, this);
  }

 private:
  void loadConfig() {
    private_nh_.param("min_range", filter_config_.min_range, 0.4);
    private_nh_.param("max_range", filter_config_.max_range, 120.0);
    private_nh_.param("enable_body_crop", filter_config_.enable_body_crop, true);
    private_nh_.param("body_crop/x_min", filter_config_.body_crop.x_min, -1.8);
    private_nh_.param("body_crop/x_max", filter_config_.body_crop.x_max, 1.8);
    private_nh_.param("body_crop/y_min", filter_config_.body_crop.y_min, -1.2);
    private_nh_.param("body_crop/y_max", filter_config_.body_crop.y_max, 1.2);
    private_nh_.param("body_crop/z_min", filter_config_.body_crop.z_min, -0.8);
    private_nh_.param("body_crop/z_max", filter_config_.body_crop.z_max, 1.4);

    private_nh_.param("enable_imu_deskew", deskew_config_.enabled, false);
    private_nh_.param("deskew_reference", deskew_config_.reference, std::string("start"));
    private_nh_.param("point_time_scale", deskew_config_.point_time_scale, 1.0);
    private_nh_.param("max_abs_point_time", deskew_config_.max_abs_point_time, 0.2);
    deskew_config_.point_time_fields.clear();
    XmlRpc::XmlRpcValue fields;
    if (private_nh_.getParam("point_time_fields", fields) && fields.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < fields.size(); ++i) {
        if (fields[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
          deskew_config_.point_time_fields.push_back(static_cast<std::string>(fields[i]));
        }
      }
    }
    if (deskew_config_.point_time_fields.empty()) {
      deskew_config_.point_time_fields.push_back("time");
      deskew_config_.point_time_fields.push_back("timestamp");
      deskew_config_.point_time_fields.push_back("t");
      deskew_config_.point_time_fields.push_back("offset_time");
    }
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& message) {
    last_imu_stamp_ = message->header.stamp;
    has_last_imu_ = true;
    ImuAngularSample sample;
    sample.stamp = message->header.stamp.toSec();
    sample.wx = message->angular_velocity.x;
    sample.wy = message->angular_velocity.y;
    sample.wz = message->angular_velocity.z;
    imu_samples_.push_back(sample);
    while (static_cast<int>(imu_samples_.size()) > std::max(1, imu_buffer_size_)) {
      imu_samples_.pop_front();
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    last_cloud_stamp_ = message->header.stamp;
    if (require_recent_imu_ && !hasRecentImu(message->header.stamp)) {
      ++dropped_no_imu_;
      return;
    }

    PointCloud2ReadResult read_result;
    if (!readFilteredPointCloud2(*message, filter_config_, &read_result)) {
      last_stats_ = read_result.stats;
      last_deskew_stats_ = DeskewStats();
      last_cloud_reject_reason_ = read_result.reason;
      ++invalid_clouds_;
      return;
    }
    last_stats_ = read_result.stats;
    last_cloud_reject_reason_.clear();

    std::vector<ImuAngularSample> imu_samples(imu_samples_.begin(), imu_samples_.end());
    last_deskew_stats_ = DeskewStats();
    std::vector<PointTuple> corrected =
        deskewPointTuples(read_result.points, read_result.field_names, message->header.stamp.toSec(), imu_samples, deskew_config_, &last_deskew_stats_);

    sensor_msgs::PointCloud2 output =
        buildOutputCloud(*message, read_result.point_bytes, corrected, read_result.x_index, read_result.y_index, read_result.z_index);
    output.header.frame_id = output_frame_id_.empty() ? message->header.frame_id : output_frame_id_;
    cloud_pub_.publish(output);
  }

  sensor_msgs::PointCloud2 buildOutputCloud(
      const sensor_msgs::PointCloud2& source,
      const std::vector<std::vector<uint8_t> >& bytes,
      const std::vector<PointTuple>& corrected,
      int x_index,
      int y_index,
      int z_index) const {
    sensor_msgs::PointCloud2 output = source;
    output.height = 1;
    output.width = static_cast<uint32_t>(bytes.size());
    output.row_step = output.point_step * output.width;
    output.data.resize(static_cast<std::size_t>(output.row_step));
    for (std::size_t i = 0; i < bytes.size(); ++i) {
      uint8_t* target = &output.data[i * output.point_step];
      std::memcpy(target, &bytes[i][0], output.point_step);
      if (i < corrected.size()) {
        writeDoubleToPointField(target, output.fields[x_index], corrected[i][0]);
        writeDoubleToPointField(target, output.fields[y_index], corrected[i][1]);
        writeDoubleToPointField(target, output.fields[z_index], corrected[i][2]);
      }
    }
    return output;
  }

  bool hasRecentImu(const ros::Time& cloud_stamp) const {
    if (!has_last_imu_) {
      return false;
    }
    return std::fabs((cloud_stamp - last_imu_stamp_).toSec()) <= max_imu_age_sec_;
  }

  void diagnosticsCallback(const ros::TimerEvent&) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "lio_preprocess";
    status.hardware_id = input_cloud_topic_;
    if (require_recent_imu_ && !has_last_imu_) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "waiting for imu";
    } else if (dropped_no_imu_ > 0) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "clouds dropped due to stale imu";
    } else if (invalid_clouds_ > 0) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "invalid point cloud2";
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "ok";
    }
    status.values.push_back(keyValue("input_cloud_topic", input_cloud_topic_));
    status.values.push_back(keyValue("output_cloud_topic", output_cloud_topic_));
    status.values.push_back(keyValue("input_points", last_stats_.input_points));
    status.values.push_back(keyValue("output_points", last_stats_.output_points));
    status.values.push_back(keyValue("dropped_nan", last_stats_.dropped_nan));
    status.values.push_back(keyValue("dropped_range", last_stats_.dropped_range));
    status.values.push_back(keyValue("dropped_body", last_stats_.dropped_body));
    status.values.push_back(keyValue("dropped_no_imu", dropped_no_imu_));
    status.values.push_back(keyValue("invalid_clouds", invalid_clouds_));
    status.values.push_back(keyValue("last_cloud_reject_reason", last_cloud_reject_reason_));
    status.values.push_back(keyValue("deskew_enabled", deskew_config_.enabled ? "true" : "false"));
    status.values.push_back(keyValue("deskewed_points", last_deskew_stats_.deskewed_points));
    status.values.push_back(keyValue("deskew_missing_time_field", last_deskew_stats_.missing_time_field));
    status.values.push_back(keyValue("deskew_invalid_point_time", last_deskew_stats_.invalid_point_time));
    status.values.push_back(keyValue("deskew_missing_imu", last_deskew_stats_.missing_imu));
    status.values.push_back(keyValue("deskew_invalid_imu", last_deskew_stats_.invalid_imu));
    status.values.push_back(keyValue("deskew_invalid_config", last_deskew_stats_.invalid_config));
    status.values.push_back(keyValue("deskew_used_imu", last_deskew_stats_.used_imu ? "true" : "false"));
    array.status.push_back(status);
    diag_pub_.publish(array);
  }

  diagnostic_msgs::KeyValue keyValue(const std::string& key, const std::string& value) const {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
  }

  diagnostic_msgs::KeyValue keyValue(const std::string& key, int value) const {
    return keyValue(key, std::to_string(value));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher cloud_pub_;
  ros::Publisher diag_pub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber cloud_sub_;
  ros::Timer diag_timer_;
  PointFilterConfig filter_config_;
  DeskewConfig deskew_config_;
  std::string input_cloud_topic_;
  std::string imu_topic_;
  std::string output_cloud_topic_;
  std::string diagnostics_topic_;
  std::string output_frame_id_;
  bool require_recent_imu_ = false;
  double max_imu_age_sec_ = 0.2;
  int imu_buffer_size_ = 400;
  bool has_last_imu_ = false;
  ros::Time last_imu_stamp_;
  ros::Time last_cloud_stamp_;
  std::deque<ImuAngularSample> imu_samples_;
  FilterStats last_stats_;
  DeskewStats last_deskew_stats_;
  int dropped_no_imu_ = 0;
  int invalid_clouds_ = 0;
  std::string last_cloud_reject_reason_;
};

}  // namespace lio_preprocess

int main(int argc, char** argv) {
  ros::init(argc, argv, "lio_preprocess");
  lio_preprocess::PreprocessNode node;
  ros::spin();
  return 0;
}
