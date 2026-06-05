#pragma once

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lio_session_manager {

struct SessionManifest {
  std::string session_id;
  double created_at = 0.0;
  double updated_at = 0.0;
  std::string state = "ACTIVE";
  double chainage_m = 0.0;
  double last_stable_x_m = 0.0;
  double last_stable_y_m = 0.0;
  double last_stable_z_m = 0.0;
  double last_stable_yaw_deg = 0.0;
  int wal_seq = 0;
};

struct RecoveryEvidence {
  double local_match_score = 0.0;
  double tca_match_score = 0.0;
  double global_match_score = 0.0;
};

struct RecoveryThresholds {
  double local_resume = 0.8;
  double local_relocalize = 0.5;
  double tca_resume = 0.75;
  double global_resume = 0.7;
};

struct StableRecoveryAnchor {
  std::string keyframe_id;
  double chainage_m = 0.0;
  std::string section_quality = "C";
  bool loop_verified = false;
};

struct OptionalStableRecoveryAnchor {
  bool has_value = false;
  StableRecoveryAnchor value;
};

struct RecoveryAlignment {
  bool converged = false;
  double rmse_m = 999.0;
  double inlier_ratio = 0.0;
  double dx_m = 0.0;
  double dy_m = 0.0;
  double dz_m = 0.0;
  double yaw_deg = 0.0;
};

struct RecoveryAlignmentGate {
  double max_rmse_m = 0.08;
  double min_inlier_ratio = 0.7;
  double max_translation_m = 0.25;
  double max_yaw_deg = 3.0;
};

struct RecoveryAlignmentOptions {
  double max_correspondence_distance_m = 1.0;
  double transformation_epsilon = 1e-4;
  double euclidean_fitness_epsilon = 1e-3;
  int max_iterations = 50;
  int min_points = 10;
  double inlier_threshold_m = 0.10;
  std::vector<double> voxel_leaf_sizes;
};

struct StableAnchorRecoveryDecision {
  std::string action = "CREATE_TEMP_SESSION";
  bool accepted = false;
  std::string reason = "not_evaluated";
  double translation_m = 0.0;
};

SessionManifest createSession(const std::string& root, const std::string& session_id = std::string(), double now = 0.0);
void writeManifest(const std::string& root, const SessionManifest& manifest);
SessionManifest loadManifest(const std::string& root, const std::string& session_id);
void commitManifestSnapshot(const std::string& root, const SessionManifest& manifest, const std::string& event);
bool recoverManifest(const std::string& root, const std::string& session_id, SessionManifest* manifest);
void appendWal(const std::string& root, const std::string& session_id, const std::string& json_record);
std::vector<SessionManifest> listSessions(const std::string& root);
bool latestSession(const std::string& root, SessionManifest* manifest);
double nextManifestUpdateStamp(const SessionManifest& manifest, double observed_time);

std::string decideTieredRecovery(
    const SessionManifest* manifest,
    double now,
    double max_resume_age_sec,
    const RecoveryEvidence& evidence,
    const RecoveryThresholds& thresholds = RecoveryThresholds());

OptionalStableRecoveryAnchor loadStableRecoveryAnchor(
    const std::string& stable_map_ledger_path,
    double chainage_m,
    double max_distance_m = 5.0,
    const std::string& min_quality = "B",
    bool require_loop_verified = false);

StableAnchorRecoveryDecision decideStableAnchorRecovery(
    const std::string& base_action,
    const StableRecoveryAnchor* anchor,
    const RecoveryAlignment* alignment,
    const RecoveryAlignmentGate& gate = RecoveryAlignmentGate());

RecoveryAlignment estimateStableAnchorAlignment(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& current_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& stable_anchor_cloud,
    const RecoveryAlignmentOptions& options = RecoveryAlignmentOptions());

std::string jsonEscape(const std::string& value);

}  // namespace lio_session_manager
