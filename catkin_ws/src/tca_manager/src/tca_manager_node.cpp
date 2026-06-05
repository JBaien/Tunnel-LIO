#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "tca_manager/tca_detection.h"
#include "tca_manager/tca_point_cloud2.h"

namespace {

diagnostic_msgs::KeyValue kv(const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string formatDouble(const double value, const int precision) {
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(precision);
  stream << value;
  return stream.str();
}

std::vector<double> readDoubleArrayParam(const ros::NodeHandle& nh,
                                         const std::string& name,
                                         const std::vector<double>& fallback) {
  XmlRpc::XmlRpcValue value;
  if (!nh.getParam(name, value) || value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return fallback;
  }

  std::vector<double> result;
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      result.push_back(static_cast<int>(value[index]));
    } else if (value[index].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      result.push_back(static_cast<double>(value[index]));
    }
  }
  return result.empty() ? fallback : result;
}

}  // namespace

namespace tca_manager {

class TcaManagerNode {
 public:
  TcaManagerNode()
      : private_nh_("~"),
        ledger_(paramDouble("min_match_score", 0.5), paramDouble("min_top_score_ratio", 1.5)) {
    config_.intensity_threshold = paramDouble("intensity_threshold", 200.0);
    config_.min_reflective_points = paramInt("min_reflective_points", 5);
    config_.cluster_radius_m = paramDouble("cluster_radius_m", 0.35);
    config_.context_ring_edges =
        readDoubleArrayParam(private_nh_, "context_ring_edges", std::vector<double>{0.0, 3.0, 5.0, 8.0});

    private_nh_.param<std::string>("ledger_path", ledger_path_, "/tmp/tunnel_lio_tca/anchors.json");
    ledger_.load(ledger_path_);

    std::string detection_topic;
    std::string match_topic;
    std::string diagnostics_topic;
    std::string input_cloud_topic;
    private_nh_.param<std::string>("detection_topic", detection_topic, "/tca/detection");
    private_nh_.param<std::string>("match_topic", match_topic, "/tca/match");
    private_nh_.param<std::string>("diagnostics_topic", diagnostics_topic, "/diagnostics/tca_manager");
    private_nh_.param<std::string>("input_cloud_topic", input_cloud_topic, "/lio/points_deskewed");

    detection_pub_ = nh_.advertise<std_msgs::String>(detection_topic, 10);
    match_pub_ = nh_.advertise<std_msgs::String>(match_topic, 10);
    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic, 5);
    cloud_sub_ = nh_.subscribe(input_cloud_topic, 5, &TcaManagerNode::cloudCallback, this);
    register_service_ = nh_.advertiseService("/tca/register", &TcaManagerNode::registerService, this);
  }

 private:
  double paramDouble(const std::string& name, const double fallback) const {
    double value = fallback;
    private_nh_.param<double>(name, value, fallback);
    return value;
  }

  int paramInt(const std::string& name, const int fallback) const {
    int value = fallback;
    private_nh_.param<int>(name, value, fallback);
    return value;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    const TcaPointCloudReadResult cloud = readTcaPointCloud2(*msg);
    if (!cloud.valid) {
      detection_cache_.clear();
      has_last_match_ = false;
      publishDiagnostics(cloud.error, diagnostic_msgs::DiagnosticStatus::WARN);
      return;
    }

    const std::vector<TcaDetection> detections = detectReflectiveTargets(cloud.points, config_);
    detection_cache_.update(detections);
    has_last_match_ = false;
    if (!detection_cache_.hasDetection()) {
      publishDiagnostics("no tca detection", diagnostic_msgs::DiagnosticStatus::OK);
      return;
    }

    const TcaDetection& detection = detection_cache_.detection();
    std_msgs::String detection_msg;
    detection_msg.data = formatDetection(detection);
    detection_pub_.publish(detection_msg);

    const OptionalTcaMatch match =
        ledger_.match(detection.center, detection.context_signature.ring_counts);
    has_last_match_ = match.has_value;
    if (match.has_value) {
      std_msgs::String match_msg;
      match_msg.data = "anchor_id=" + match.value.anchor_id + ";score=" +
                       formatDouble(match.value.score, 3) + ";chainage_m=" +
                       formatDouble(match.value.chainage_m, 3);
      match_pub_.publish(match_msg);
    }
    publishDiagnostics("ok", diagnostic_msgs::DiagnosticStatus::OK);
  }

  bool registerService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (!detection_cache_.hasDetection()) {
      response.success = false;
      response.message = "no detection to register";
      return true;
    }

    const std::string anchor_id = "tca_" + std::to_string(ros::Time::now().sec);
    TcaAnchor anchor;
    if (!detection_cache_.buildAnchor(anchor_id, &anchor)) {
      response.success = false;
      response.message = "no valid detection to register";
      return true;
    }
    ledger_.addAnchor(anchor);
    ledger_.save(ledger_path_);

    response.success = true;
    response.message = anchor_id;
    return true;
  }

  void publishDiagnostics(const std::string& message, const unsigned char level) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "tca_manager";
    status.hardware_id = ledger_path_;
    status.level = level;
    status.message = message;
    status.values.push_back(kv("anchors", std::to_string(ledger_.anchors().size())));
    status.values.push_back(
        kv("last_detection_count", std::to_string(detection_cache_.lastDetectionCount())));
    status.values.push_back(kv("has_match", has_last_match_ ? "True" : "False"));
    array.status.push_back(status);
    diag_pub_.publish(array);
  }

  static std::string formatDetection(const TcaDetection& detection) {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(3);
    stream << "center=" << detection.center.x << "," << detection.center.y << ","
           << detection.center.z << ";reflective_points=" << detection.reflective_points
           << ";context=";
    for (std::size_t index = 0; index < detection.context_signature.ring_counts.size(); ++index) {
      if (index > 0) {
        stream << ",";
      }
      stream << detection.context_signature.ring_counts[index];
    }
    return stream.str();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  TcaDetectionConfig config_;
  std::string ledger_path_;
  TcaLedger ledger_;
  TcaDetectionCache detection_cache_;
  bool has_last_match_ = false;
  ros::Publisher detection_pub_;
  ros::Publisher match_pub_;
  ros::Publisher diag_pub_;
  ros::Subscriber cloud_sub_;
  ros::ServiceServer register_service_;
};

}  // namespace tca_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "tca_manager");
  tca_manager::TcaManagerNode node;
  ros::spin();
  return 0;
}
