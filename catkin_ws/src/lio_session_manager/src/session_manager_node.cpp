#include <sstream>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <XmlRpcValue.h>

#include "lio_session_manager/session_store.h"

namespace lio_session_manager {

namespace {

std::vector<double> doubleVectorParam(ros::NodeHandle& nh,
                                      const std::string& name,
                                      const std::vector<double>& defaults) {
  XmlRpc::XmlRpcValue raw;
  if (!nh.getParam(name, raw) || raw.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return defaults;
  }
  std::vector<double> values;
  values.reserve(raw.size());
  for (int i = 0; i < raw.size(); ++i) {
    if (raw[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      values.push_back(static_cast<double>(raw[i]));
    } else if (raw[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      values.push_back(static_cast<int>(raw[i]));
    }
  }
  return values.empty() ? defaults : values;
}

}  // namespace

class SessionManagerNode {
 public:
  SessionManagerNode() : private_nh_("~") {
    private_nh_.param("session_root", root_, std::string("/tmp/tunnel_lio_sessions"));
    private_nh_.param("max_resume_age_sec", max_resume_age_sec_, 86400.0);
    private_nh_.param("stable_map_ledger_path", stable_map_ledger_path_, std::string("/tmp/tunnel_lio_stable_map/stable_map.json"));
    private_nh_.param("stable_anchor_max_distance_m", stable_anchor_max_distance_m_, 5.0);
    private_nh_.param("stable_anchor_min_quality", stable_anchor_min_quality_, std::string("B"));
    private_nh_.param("require_loop_verified_anchor", require_loop_verified_anchor_, false);

    private_nh_.param("local_resume_score", recovery_thresholds_.local_resume, 0.8);
    private_nh_.param("local_relocalize_score", recovery_thresholds_.local_relocalize, 0.5);
    private_nh_.param("tca_resume_score", recovery_thresholds_.tca_resume, 0.75);
    private_nh_.param("global_resume_score", recovery_thresholds_.global_resume, 0.7);

    private_nh_.param("initial_local_match_score", recovery_evidence_.local_match_score, 0.0);
    private_nh_.param("initial_tca_match_score", recovery_evidence_.tca_match_score, 0.0);
    private_nh_.param("initial_global_match_score", recovery_evidence_.global_match_score, 0.0);

    private_nh_.param("recovery_alignment_max_rmse_m", recovery_alignment_gate_.max_rmse_m, 0.08);
    private_nh_.param("recovery_alignment_min_inlier_ratio", recovery_alignment_gate_.min_inlier_ratio, 0.7);
    private_nh_.param("recovery_alignment_max_translation_m", recovery_alignment_gate_.max_translation_m, 0.25);
    private_nh_.param("recovery_alignment_max_yaw_deg", recovery_alignment_gate_.max_yaw_deg, 3.0);

    private_nh_.param("initial_recovery_alignment_converged", recovery_alignment_.converged, false);
    private_nh_.param("initial_recovery_alignment_rmse_m", recovery_alignment_.rmse_m, 999.0);
    private_nh_.param("initial_recovery_alignment_inlier_ratio", recovery_alignment_.inlier_ratio, 0.0);
    private_nh_.param("initial_recovery_alignment_dx_m", recovery_alignment_.dx_m, 0.0);
    private_nh_.param("initial_recovery_alignment_dy_m", recovery_alignment_.dy_m, 0.0);
    private_nh_.param("initial_recovery_alignment_dz_m", recovery_alignment_.dz_m, 0.0);
    private_nh_.param("initial_recovery_alignment_yaw_deg", recovery_alignment_.yaw_deg, 0.0);

    private_nh_.param("enable_recovery_cloud_alignment", enable_recovery_cloud_alignment_, true);
    private_nh_.param("current_recovery_cloud_topic", current_recovery_cloud_topic_, std::string("/session/current_recovery_cloud"));
    private_nh_.param("stable_anchor_cloud_topic", stable_anchor_cloud_topic_, std::string("/session/stable_anchor_cloud"));
    private_nh_.param("recovery_icp_max_correspondence_distance_m", recovery_alignment_options_.max_correspondence_distance_m, 1.0);
    private_nh_.param("recovery_icp_transformation_epsilon", recovery_alignment_options_.transformation_epsilon, 1e-4);
    private_nh_.param("recovery_icp_euclidean_fitness_epsilon", recovery_alignment_options_.euclidean_fitness_epsilon, 1e-3);
    private_nh_.param("recovery_icp_max_iterations", recovery_alignment_options_.max_iterations, 50);
    private_nh_.param("recovery_icp_min_points", recovery_alignment_options_.min_points, 50);
    private_nh_.param("recovery_icp_inlier_threshold_m", recovery_alignment_options_.inlier_threshold_m, 0.10);
    recovery_alignment_options_.voxel_leaf_sizes =
        doubleVectorParam(private_nh_, "recovery_icp_voxel_leaf_sizes",
                          std::vector<double>{0.30, 0.15, 0.0});

    if (!latestSession(root_, &manifest_)) {
      manifest_ = createSession(root_);
    }

    std::string status_topic;
    private_nh_.param("status_topic", status_topic, std::string("/session/status"));
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic, 5);
    if (enable_recovery_cloud_alignment_) {
      current_recovery_cloud_sub_ = nh_.subscribe(
          current_recovery_cloud_topic_, 1,
          &SessionManagerNode::currentRecoveryCloudCallback, this);
      stable_anchor_cloud_sub_ = nh_.subscribe(
          stable_anchor_cloud_topic_, 1,
          &SessionManagerNode::stableAnchorCloudCallback, this);
    }
    snapshot_srv_ = nh_.advertiseService("/session/snapshot", &SessionManagerNode::snapshotService, this);
    recover_srv_ = nh_.advertiseService("/session/recover", &SessionManagerNode::recoverService, this);

    double publish_period_sec = 1.0;
    double snapshot_period_sec = 5.0;
    private_nh_.param("publish_period_sec", publish_period_sec, 1.0);
    private_nh_.param("snapshot_period_sec", snapshot_period_sec, 5.0);
    status_timer_ = nh_.createTimer(ros::Duration(publish_period_sec), &SessionManagerNode::statusTimer, this);
    snapshot_timer_ = nh_.createTimer(ros::Duration(snapshot_period_sec), &SessionManagerNode::snapshotTimer, this);
  }

 private:
  bool snapshotService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    snapshot("manual_snapshot");
    response.success = true;
    response.message = manifest_.session_id;
    return true;
  }

  bool recoverService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    SessionManifest candidate;
    const bool has_candidate = latestSession(root_, &candidate);
    const std::string base_action = decideTieredRecovery(
        has_candidate ? &candidate : nullptr, ros::Time::now().toSec(), max_resume_age_sec_, recovery_evidence_, recovery_thresholds_);

    OptionalStableRecoveryAnchor anchor = loadStableRecoveryAnchor(
        stable_map_ledger_path_,
        has_candidate ? candidate.chainage_m : 0.0,
        stable_anchor_max_distance_m_,
        stable_anchor_min_quality_,
        require_loop_verified_anchor_);
    StableAnchorRecoveryDecision stable_decision = decideStableAnchorRecovery(
        base_action,
        anchor.has_value ? &anchor.value : nullptr,
        &latestRecoveryAlignment(),
        recovery_alignment_gate_);

    const std::string action = stable_decision.action;
    if (action == "CREATE_NEW_SESSION") {
      manifest_ = createSession(root_);
    } else if (action == "CREATE_TEMP_SESSION") {
      std::ostringstream id;
      id << "temp_" << static_cast<long long>(ros::Time::now().toSec());
      manifest_ = createSession(root_, id.str());
      manifest_.state = "TEMP";
      commitManifestSnapshot(root_, manifest_, "mark_temp_session");
    } else if (has_candidate) {
      manifest_ = candidate;
    }

    appendWal(root_, manifest_.session_id, recoverRecord(base_action, action, stable_decision, anchor));
    response.success = true;
    response.message = action;
    return true;
  }

  void snapshotTimer(const ros::TimerEvent&) {
    snapshot("periodic_snapshot");
  }

  void statusTimer(const ros::TimerEvent&) {
    std_msgs::String message;
    message.data = manifest_.session_id + ":" + manifest_.state;
    status_pub_.publish(message);
  }

  void currentRecoveryCloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*message, *cloud);
    current_recovery_cloud_ = cloud;
  }

  void stableAnchorCloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*message, *cloud);
    stable_anchor_cloud_ = cloud;
  }

  const RecoveryAlignment& latestRecoveryAlignment() {
    if (enable_recovery_cloud_alignment_ && current_recovery_cloud_ &&
        stable_anchor_cloud_) {
      recovery_alignment_ = estimateStableAnchorAlignment(
          current_recovery_cloud_, stable_anchor_cloud_,
          recovery_alignment_options_);
    }
    return recovery_alignment_;
  }

  void snapshot(const std::string& event) {
    manifest_.updated_at = nextManifestUpdateStamp(
        manifest_, ros::Time::now().toSec());
    ++manifest_.wal_seq;
    commitManifestSnapshot(root_, manifest_, event);
  }

  std::string recoverRecord(
      const std::string& base_action,
      const std::string& action,
      const StableAnchorRecoveryDecision& stable_decision,
      const OptionalStableRecoveryAnchor& anchor) const {
    std::ostringstream record;
    record << "{\"event\":\"recover\",\"stamp\":" << ros::Time::now().toSec()
           << ",\"base_action\":\"" << jsonEscape(base_action)
           << "\",\"action\":\"" << jsonEscape(action)
           << "\",\"stable_anchor_decision\":{\"accepted\":" << (stable_decision.accepted ? "true" : "false")
           << ",\"reason\":\"" << jsonEscape(stable_decision.reason)
           << "\",\"translation_m\":" << stable_decision.translation_m << "}";
    if (anchor.has_value) {
      record << ",\"stable_anchor\":{\"keyframe_id\":\"" << jsonEscape(anchor.value.keyframe_id)
             << "\",\"chainage_m\":" << anchor.value.chainage_m
             << ",\"section_quality\":\"" << jsonEscape(anchor.value.section_quality)
             << "\",\"loop_verified\":" << (anchor.value.loop_verified ? "true" : "false") << "}";
    }
    record << "}";
    return record.str();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher status_pub_;
  ros::Subscriber current_recovery_cloud_sub_;
  ros::Subscriber stable_anchor_cloud_sub_;
  ros::ServiceServer snapshot_srv_;
  ros::ServiceServer recover_srv_;
  ros::Timer status_timer_;
  ros::Timer snapshot_timer_;

  std::string root_;
  std::string stable_map_ledger_path_;
  std::string stable_anchor_min_quality_;
  std::string current_recovery_cloud_topic_;
  std::string stable_anchor_cloud_topic_;
  bool require_loop_verified_anchor_ = false;
  bool enable_recovery_cloud_alignment_ = true;
  double max_resume_age_sec_ = 86400.0;
  double stable_anchor_max_distance_m_ = 5.0;
  RecoveryThresholds recovery_thresholds_;
  RecoveryEvidence recovery_evidence_;
  RecoveryAlignmentGate recovery_alignment_gate_;
  RecoveryAlignmentOptions recovery_alignment_options_;
  RecoveryAlignment recovery_alignment_;
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr current_recovery_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr stable_anchor_cloud_;
  SessionManifest manifest_;
};

}  // namespace lio_session_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "lio_session_manager");
  lio_session_manager::SessionManagerNode node;
  ros::spin();
  return 0;
}
