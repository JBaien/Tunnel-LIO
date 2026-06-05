#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "slam_backend_manager/backend_candidates.h"
#include "slam_backend_manager/backend_point_cloud_reader.h"
#include "slam_backend_manager/control_field_parser.h"
#include "slam_backend_manager/stable_map_store.h"

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

namespace slam_backend_manager {

class SlamBackendNode {
 public:
  SlamBackendNode()
      : private_nh_("~"),
        stable_ledger_(paramString("stable_map_ledger_path",
                                   "/tmp/tunnel_lio_stable_map/stable_map.json")),
        policy_(paramString("min_stable_quality", "B")) {
    config_.ring_edges =
        readDoubleArrayParam(private_nh_, "ring_edges", std::vector<double>{0.0, 5.0, 10.0, 20.0, 40.0});
    config_.sector_count = paramInt("sector_count", 12);
    config_.max_descriptor_bin_count = paramInt("max_descriptor_bin_count", 0);
    config_.intensity_quantization = paramDouble("intensity_quantization", 1.0);
    config_.min_intensity_bin_points = paramInt("min_intensity_bin_points", 1);
    config_.min_loop_score = paramDouble("min_loop_score", 0.5);
    config_.min_top_score_ratio = paramDouble("min_top_score_ratio", 1.5);
    config_.min_rotation_uniqueness_ratio =
        paramDouble("min_rotation_uniqueness_ratio", 1.0);
    config_.min_loop_chainage_separation_m =
        paramDouble("min_loop_chainage_separation_m", 10.0);
    config_.min_geometric_score = paramDouble("min_geometric_score", 0.75);
    config_.max_centroid_distance_m = paramDouble("max_centroid_distance_m", 0.5);
    config_.max_icp_rmse_m = paramDouble("max_icp_rmse_m", 0.15);
    config_.min_icp_inlier_ratio = paramDouble("min_icp_inlier_ratio", 0.8);
    config_.icp_inlier_threshold_m = paramDouble("icp_inlier_threshold_m", 0.25);
    config_.icp_iterations = paramInt("icp_iterations", 20);
    config_.max_icp_points = paramInt("max_icp_points", 300);
    config_.intensity_descriptor_weight =
        paramDouble("intensity_descriptor_weight", 0.0);
    keyframe_spacing_m_ = paramDouble("keyframe_spacing_m", 5.0);
    last_keyframe_chainage_ = -keyframe_spacing_m_;

    loop_pub_ = nh_.advertise<std_msgs::String>(
        paramString("loop_candidate_topic", "/backend/loop_candidate"), 5);
    loop_verified_pub_ = nh_.advertise<std_msgs::String>(
        paramString("loop_verified_topic", "/backend/loop_verified"), 5);
    promotion_pub_ = nh_.advertise<std_msgs::String>(
        paramString("stable_promotion_topic", "/backend/stable_promotion"), 5);
    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        paramString("diagnostics_topic", "/diagnostics/slam_backend"), 5);

    submap_sub_ = nh_.subscribe(paramString("submap_topic", "/map/local_submap"), 2,
                                &SlamBackendNode::submapCallback, this);
    control_sub_ = nh_.subscribe(paramString("mapping_control_topic", "/mapping/control"), 10,
                                 &SlamBackendNode::controlCallback, this);
    section_sub_ = nh_.subscribe(paramString("section_structural_topic", "/section/structural"), 10,
                                 &SlamBackendNode::sectionCallback, this);
    machine_state_sub_ = nh_.subscribe(paramString("machine_state_topic", "/machine/state"), 10,
                                       &SlamBackendNode::machineStateCallback, this);
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

  std::string paramString(const std::string& name, const std::string& fallback) const {
    std::string value = fallback;
    private_nh_.param<std::string>(name, value, fallback);
    return value;
  }

  void submapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    last_submap_ = msg;
  }

  void controlCallback(const std_msgs::StringConstPtr& msg) {
    current_chainage_ = parseStrictDoubleFieldOr(
        msg->data, "chainage_m", current_chainage_);
    if (!last_submap_) {
      return;
    }
    if (current_chainage_ - last_keyframe_chainage_ < keyframe_spacing_m_) {
      return;
    }

    const BackendPointCloudReadResult read_result =
        readBackendPointCloud2(*last_submap_);
    if (!read_result.ok) {
      publishDiagnostics("invalid_submap:" + read_result.reason,
                         diagnostic_msgs::DiagnosticStatus::WARN);
      return;
    }
    const std::vector<Point3>& points = read_result.points;
    DescriptorConfig descriptor_config;
    descriptor_config.ring_edges = config_.ring_edges;
    descriptor_config.sector_count = config_.sector_count;
    descriptor_config.max_geometry_bin_count = config_.max_descriptor_bin_count;
    descriptor_config.intensity_quantization = config_.intensity_quantization;
    descriptor_config.min_intensity_bin_points = config_.min_intensity_bin_points;
    const HistogramDescriptor descriptor =
        makeScanContextDescriptor(points, descriptor_config);
    const HistogramDescriptor intensity_descriptor =
        makeIntensityScanContextDescriptor(points, descriptor_config);
    std::vector<Point3> samples;
    const std::size_t sample_count =
        config_.max_icp_points > 0 ? std::min<std::size_t>(points.size(), config_.max_icp_points)
                                   : points.size();
    samples.insert(samples.end(), points.begin(), points.begin() + sample_count);

    Keyframe current;
    current.keyframe_id = "kf_" + std::to_string(keyframes_.size());
    current.chainage_m = current_chainage_;
    current.descriptor = descriptor.bins;
    current.geometry = makeGeometrySummary(points);
    current.sample_points = samples;
    current.intensity_descriptor = intensity_descriptor.bins;

    const OptionalLoopCandidate candidate = chooseLoopCandidate(current, keyframes_, config_);
    if (candidate.has_value) {
      handleLoopCandidate(current, candidate.value);
    }

    keyframes_.push_back(current);
    last_keyframe_chainage_ = current_chainage_;
    maybePromote(current.keyframe_id);
    publishDiagnostics("ok", diagnostic_msgs::DiagnosticStatus::OK);
  }

  void handleLoopCandidate(const Keyframe& current, const LoopCandidate& candidate) {
    has_unresolved_loop_ = true;
    const Keyframe* candidate_keyframe = findKeyframe(candidate.keyframe_id);
    LoopVerification verification;
    IcpVerification icp_verification;
    if (candidate_keyframe != nullptr) {
      verification = verifyLoopGeometry(current, *candidate_keyframe, config_);
      if (verification.accepted) {
        icp_verification = verifyLoopIcp(current.sample_points, candidate_keyframe->sample_points, config_);
      }
    } else {
      verification.reason = "candidate_missing";
    }

    if (candidate_keyframe != nullptr && verification.accepted && icp_verification.accepted) {
      has_unresolved_loop_ = false;
      last_loop_verification_ =
          "accepted:" + formatDouble(verification.score, 3) + ";icp_rmse=" +
          formatDouble(icp_verification.rmse_m, 3);
      std_msgs::String text;
      text.data = "current=" + current.keyframe_id + ";candidate=" + candidate.keyframe_id +
                  ";score=" + formatDouble(candidate.score, 3) + ";geometry_score=" +
                  formatDouble(verification.score, 3) + ";icp_rmse_m=" +
                  formatDouble(icp_verification.rmse_m, 3) + ";icp_inlier_ratio=" +
                  formatDouble(icp_verification.inlier_ratio, 3);
      loop_verified_pub_.publish(text);
      return;
    }

    std::string reason = verification.reason.empty() ? "candidate_missing" : verification.reason;
    if (verification.accepted && !icp_verification.reason.empty()) {
      reason = icp_verification.reason;
    }
    last_loop_verification_ = "rejected:" + reason;
    std_msgs::String text;
    text.data = "current=" + current.keyframe_id + ";candidate=" + candidate.keyframe_id +
                ";score=" + formatDouble(candidate.score, 3) + ";candidate_chainage_m=" +
                formatDouble(candidate.chainage_m, 3) + ";verification=" + reason;
    loop_pub_.publish(text);
  }

  void sectionCallback(const std_msgs::StringConstPtr& msg) {
    section_quality_ = parseTextFieldOr(msg->data, "quality", section_quality_);
  }

  void machineStateCallback(const std_msgs::StringConstPtr& msg) {
    machine_state_ = msg->data;
  }

  void maybePromote(const std::string& keyframe_id) {
    if (!policy_.canPromote(machine_state_, section_quality_, has_unresolved_loop_)) {
      return;
    }

    stable_ledger_.promote(StableMapEntry{keyframe_id,
                                          current_chainage_,
                                          section_quality_,
                                          ros::Time::now().toSec(),
                                          last_loop_verification_.find("accepted") == 0});
    std_msgs::String text;
    text.data = "keyframe=" + keyframe_id + ";chainage_m=" + formatDouble(current_chainage_, 3) +
                ";quality=" + section_quality_;
    promotion_pub_.publish(text);
  }

  const Keyframe* findKeyframe(const std::string& keyframe_id) const {
    for (const Keyframe& keyframe : keyframes_) {
      if (keyframe.keyframe_id == keyframe_id) {
        return &keyframe;
      }
    }
    return nullptr;
  }

  void publishDiagnostics(const std::string& message, const unsigned char level) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "slam_backend_manager";
    status.hardware_id = "conservative_backend";
    status.level = level;
    status.message = message;
    status.values.push_back(kv("keyframes", std::to_string(keyframes_.size())));
    status.values.push_back(kv("chainage_m", formatDouble(current_chainage_, 3)));
    status.values.push_back(kv("section_quality", section_quality_));
    status.values.push_back(kv("machine_state", machine_state_));
    status.values.push_back(kv("has_unresolved_loop", has_unresolved_loop_ ? "True" : "False"));
    status.values.push_back(kv("last_loop_verification", last_loop_verification_));
    status.values.push_back(kv("stable_entries", std::to_string(stable_ledger_.entries().size())));
    status.values.push_back(kv("descriptor_type", "geometry_intensity_context"));
    status.values.push_back(kv("sector_count", std::to_string(config_.sector_count)));
    status.values.push_back(kv("min_rotation_uniqueness_ratio",
                               formatDouble(config_.min_rotation_uniqueness_ratio, 3)));
    array.status.push_back(status);
    diag_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  BackendConfig config_;
  double keyframe_spacing_m_ = 5.0;
  StableMapLedger stable_ledger_;
  StableMapPolicy policy_;
  std::vector<Keyframe> keyframes_;
  sensor_msgs::PointCloud2ConstPtr last_submap_;
  double last_keyframe_chainage_ = -5.0;
  double current_chainage_ = 0.0;
  std::string machine_state_ = "RELOCALIZING";
  std::string section_quality_ = "C";
  bool has_unresolved_loop_ = false;
  std::string last_loop_verification_ = "none";
  ros::Publisher loop_pub_;
  ros::Publisher loop_verified_pub_;
  ros::Publisher promotion_pub_;
  ros::Publisher diag_pub_;
  ros::Subscriber submap_sub_;
  ros::Subscriber control_sub_;
  ros::Subscriber section_sub_;
  ros::Subscriber machine_state_sub_;
};

}  // namespace slam_backend_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "slam_backend_manager");
  slam_backend_manager::SlamBackendNode node;
  ros::spin();
  return 0;
}
