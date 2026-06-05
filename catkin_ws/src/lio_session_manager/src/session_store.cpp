#include "lio_session_manager/session_store.h"

#include <cmath>
#include <ctime>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "lio_local_odometry/local_registration.h"

namespace fs = boost::filesystem;
namespace pt = boost::property_tree;

namespace lio_session_manager {

namespace {

double wallTimeNow() {
  return static_cast<double>(std::time(nullptr));
}

std::string makeSessionId(double now) {
  std::time_t raw = static_cast<std::time_t>(now);
  std::tm local_time;
  localtime_r(&raw, &local_time);
  char buffer[64];
  std::strftime(buffer, sizeof(buffer), "session_%Y%m%d_%H%M%S", &local_time);
  return std::string(buffer);
}

int qualityRank(const std::string& value) {
  if (value == "A") {
    return 0;
  }
  if (value == "B") {
    return 1;
  }
  if (value == "C") {
    return 2;
  }
  return 99;
}

bool validQuality(const std::string& value) {
  return qualityRank(value) < 99;
}

bool validKeyframeId(const std::string& value) {
  if (value.empty() || value == "." || value == "..") {
    return false;
  }
  for (const char character : value) {
    const bool uppercase = character >= 'A' && character <= 'Z';
    const bool lowercase = character >= 'a' && character <= 'z';
    const bool digit = character >= '0' && character <= '9';
    if (!uppercase && !lowercase && !digit && character != '_' &&
        character != '-' && character != '.') {
      return false;
    }
  }
  return true;
}

bool validSessionId(const std::string& value) {
  return validKeyframeId(value);
}

bool validSessionState(const std::string& value) {
  return value == "ACTIVE" || value == "TEMP";
}

bool validSnapshotEvent(const std::string& value) {
  return validKeyframeId(value);
}

bool validScore(double value) {
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

bool validRecoveryThresholds(const RecoveryThresholds& thresholds) {
  return validScore(thresholds.local_resume) &&
         validScore(thresholds.local_relocalize) &&
         validScore(thresholds.tca_resume) &&
         validScore(thresholds.global_resume) &&
         thresholds.local_relocalize <= thresholds.local_resume;
}

bool validRecoveryEvidence(const RecoveryEvidence& evidence) {
  return validScore(evidence.local_match_score) &&
         validScore(evidence.tca_match_score) &&
         validScore(evidence.global_match_score);
}

bool validRecoveryAction(const std::string& value) {
  return value == "CREATE_NEW_SESSION" ||
         value == "RESUME_LAST_STABLE" ||
         value == "RECOVER_WITH_TCA" ||
         value == "RECOVER_WITH_GLOBAL_CANDIDATE" ||
         value == "RELOCALIZE_REQUIRED" ||
         value == "CREATE_TEMP_SESSION" ||
         value == "RECOVER_WITH_STABLE_ANCHOR";
}

bool validStableAnchorRecoveryReason(const std::string& value) {
  return value == "base_action_requires_new_session" ||
         value == "stable_anchor_missing" ||
         value == "invalid_stable_anchor" ||
         value == "stable_anchor_alignment_missing" ||
         value == "invalid_alignment_gate" ||
         value == "invalid_alignment" ||
         value == "alignment_not_converged" ||
         value == "rmse_exceeds_gate" ||
         value == "inlier_ratio_below_gate" ||
         value == "translation_exceeds_gate" ||
         value == "yaw_exceeds_gate" ||
         value == "stable_anchor_alignment_accepted";
}

bool validStableAnchor(const StableRecoveryAnchor& anchor) {
  return validKeyframeId(anchor.keyframe_id) &&
         std::isfinite(anchor.chainage_m) &&
         validQuality(anchor.section_quality);
}

bool validAlignmentGate(const RecoveryAlignmentGate& gate) {
  return std::isfinite(gate.max_rmse_m) && gate.max_rmse_m >= 0.0 &&
         validScore(gate.min_inlier_ratio) &&
         std::isfinite(gate.max_translation_m) && gate.max_translation_m >= 0.0 &&
         std::isfinite(gate.max_yaw_deg) && gate.max_yaw_deg >= 0.0;
}

bool validAlignment(const RecoveryAlignment& alignment) {
  return std::isfinite(alignment.rmse_m) && alignment.rmse_m >= 0.0 &&
         validScore(alignment.inlier_ratio) &&
         std::isfinite(alignment.dx_m) &&
         std::isfinite(alignment.dy_m) &&
         std::isfinite(alignment.dz_m) &&
         std::isfinite(alignment.yaw_deg);
}

bool validAlignmentOptions(const RecoveryAlignmentOptions& options) {
  if (!std::isfinite(options.max_correspondence_distance_m) ||
      options.max_correspondence_distance_m <= 0.0 ||
      !std::isfinite(options.transformation_epsilon) ||
      options.transformation_epsilon < 0.0 ||
      !std::isfinite(options.euclidean_fitness_epsilon) ||
      options.euclidean_fitness_epsilon < 0.0 ||
      options.max_iterations <= 0 ||
      options.min_points <= 0 ||
      !std::isfinite(options.inlier_threshold_m) ||
      options.inlier_threshold_m <= 0.0) {
    return false;
  }
  for (std::vector<double>::const_iterator it = options.voxel_leaf_sizes.begin();
       it != options.voxel_leaf_sizes.end(); ++it) {
    if (!std::isfinite(*it) || *it < 0.0) {
      return false;
    }
  }
  return true;
}

bool finiteRecoveryCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) {
  if (!cloud) {
    return false;
  }
  for (const auto& point : cloud->points) {
    if (!pcl::isFinite(point)) {
      return false;
    }
  }
  return true;
}

fs::path sessionDir(const std::string& root, const std::string& session_id) {
  if (!validSessionId(session_id)) {
    throw std::invalid_argument("invalid session_id");
  }
  return fs::path(root) / session_id;
}

void manifestToTree(const SessionManifest& manifest, pt::ptree* tree) {
  tree->put("session_id", manifest.session_id);
  tree->put("created_at", manifest.created_at);
  tree->put("updated_at", manifest.updated_at);
  tree->put("state", manifest.state);
  tree->put("chainage_m", manifest.chainage_m);
  tree->put("last_stable_pose.x_m", manifest.last_stable_x_m);
  tree->put("last_stable_pose.y_m", manifest.last_stable_y_m);
  tree->put("last_stable_pose.z_m", manifest.last_stable_z_m);
  tree->put("last_stable_pose.yaw_deg", manifest.last_stable_yaw_deg);
  tree->put("wal_seq", manifest.wal_seq);
}

SessionManifest treeToManifest(const pt::ptree& tree) {
  SessionManifest manifest;
  manifest.session_id = tree.get<std::string>("session_id");
  manifest.created_at = tree.get<double>("created_at");
  manifest.updated_at = tree.get<double>("updated_at");
  manifest.state = tree.get<std::string>("state", "ACTIVE");
  manifest.chainage_m = tree.get<double>("chainage_m", 0.0);
  manifest.last_stable_x_m = tree.get<double>("last_stable_pose.x_m", 0.0);
  manifest.last_stable_y_m = tree.get<double>("last_stable_pose.y_m", 0.0);
  manifest.last_stable_z_m = tree.get<double>("last_stable_pose.z_m", 0.0);
  manifest.last_stable_yaw_deg = tree.get<double>("last_stable_pose.yaw_deg", 0.0);
  manifest.wal_seq = tree.get<int>("wal_seq", 0);
  return manifest;
}

bool validManifestForSession(const SessionManifest& manifest,
                             const std::string& session_id) {
  return manifest.session_id == session_id &&
         validSessionId(session_id) &&
         validSessionId(manifest.session_id) &&
         validSessionState(manifest.state) &&
         std::isfinite(manifest.created_at) &&
         std::isfinite(manifest.updated_at) &&
         manifest.created_at >= 0.0 &&
         manifest.updated_at >= manifest.created_at &&
         std::isfinite(manifest.chainage_m) &&
         std::isfinite(manifest.last_stable_x_m) &&
         std::isfinite(manifest.last_stable_y_m) &&
         std::isfinite(manifest.last_stable_z_m) &&
         std::isfinite(manifest.last_stable_yaw_deg) &&
         manifest.wal_seq >= 0;
}

void requireValidManifestForWrite(const SessionManifest& manifest) {
  if (!validManifestForSession(manifest, manifest.session_id)) {
    throw std::invalid_argument("invalid session manifest");
  }
}

void requireSingleLineWalRecord(const std::string& json_record) {
  if (json_record.find('\n') != std::string::npos ||
      json_record.find('\r') != std::string::npos) {
    throw std::invalid_argument("invalid wal record");
  }
}

bool hasDuplicateJsonKeys(const pt::ptree& tree) {
  std::set<std::string> keys;
  for (pt::ptree::const_iterator it = tree.begin(); it != tree.end(); ++it) {
    if (!it->first.empty() && !keys.insert(it->first).second) {
      return true;
    }
    if (hasDuplicateJsonKeys(it->second)) {
      return true;
    }
  }
  return false;
}

bool validWalStamp(double stamp) {
  return std::isfinite(stamp) && stamp >= 0.0;
}

bool sameWalStamp(double lhs, double rhs) {
  return validWalStamp(lhs) && validWalStamp(rhs) && std::fabs(lhs - rhs) <= 1e-9;
}

bool validManifestSnapshotWalRecord(const pt::ptree& record,
                                    const std::string& session_id) {
  try {
    const std::string snapshot_event =
        record.get<std::string>("snapshot_event");
    const double stamp = record.get<double>("stamp");
    const SessionManifest manifest =
        treeToManifest(record.get_child("manifest"));
    return validSnapshotEvent(snapshot_event) &&
           validWalStamp(stamp) &&
           sameWalStamp(stamp, manifest.updated_at) &&
           validManifestForSession(manifest, session_id);
  } catch (const std::exception&) {
    return false;
  }
}

bool validRecoverWalRecord(const pt::ptree& record) {
  try {
    const double stamp = record.get<double>("stamp");
    const std::string base_action = record.get<std::string>("base_action");
    const std::string action = record.get<std::string>("action");
    const pt::ptree& decision = record.get_child("stable_anchor_decision");
    const bool accepted = decision.get<bool>("accepted");
    const std::string reason = decision.get<std::string>("reason");
    const double translation_m = decision.get<double>("translation_m");
    if (!validWalStamp(stamp) ||
        !validRecoveryAction(base_action) ||
        !validRecoveryAction(action) ||
        !validStableAnchorRecoveryReason(reason) ||
        !std::isfinite(translation_m) ||
        translation_m < 0.0) {
      return false;
    }
    const bool stable_anchor_recovery =
        action == "RECOVER_WITH_STABLE_ANCHOR";
    if (stable_anchor_recovery != accepted) {
      return false;
    }
    if (accepted &&
        (base_action == "CREATE_NEW_SESSION" ||
         reason != "stable_anchor_alignment_accepted")) {
      return false;
    }
    if (!accepted &&
        (action != base_action ||
         reason == "stable_anchor_alignment_accepted")) {
      return false;
    }

    const pt::ptree::const_assoc_iterator anchor_it =
        record.find("stable_anchor");
    if (anchor_it == record.not_found()) {
      return action != "RECOVER_WITH_STABLE_ANCHOR";
    }

    StableRecoveryAnchor anchor;
    anchor.keyframe_id = anchor_it->second.get<std::string>("keyframe_id");
    anchor.chainage_m = anchor_it->second.get<double>("chainage_m");
    anchor.section_quality =
        anchor_it->second.get<std::string>("section_quality");
    anchor.loop_verified = anchor_it->second.get<bool>("loop_verified");
    return validStableAnchor(anchor);
  } catch (const std::exception&) {
    return false;
  }
}

void requireValidWalRecord(const std::string& session_id,
                           const std::string& json_record) {
  requireSingleLineWalRecord(json_record);
  try {
    std::istringstream stream(json_record);
    pt::ptree record;
    pt::read_json(stream, record);
    if (hasDuplicateJsonKeys(record)) {
      throw std::invalid_argument("duplicate wal key");
    }
    const std::string event = record.get<std::string>("event");
    if (event != "manifest_snapshot" && event != "recover") {
      throw std::invalid_argument("invalid wal event");
    }
    if (event == "manifest_snapshot" &&
        !validManifestSnapshotWalRecord(record, session_id)) {
      throw std::invalid_argument("invalid manifest snapshot wal record");
    }
    if (event == "recover" && !validRecoverWalRecord(record)) {
      throw std::invalid_argument("invalid recover wal record");
    }
  } catch (const std::invalid_argument&) {
    throw;
  } catch (const std::exception&) {
    throw std::invalid_argument("invalid wal record");
  }
}

}  // namespace

SessionManifest createSession(const std::string& root, const std::string& session_id, double now) {
  const double stamp = now > 0.0 ? now : wallTimeNow();
  SessionManifest manifest;
  manifest.session_id = session_id.empty() ? makeSessionId(stamp) : session_id;
  manifest.created_at = stamp;
  manifest.updated_at = stamp;
  fs::create_directories(sessionDir(root, manifest.session_id));
  commitManifestSnapshot(root, manifest, "create_session");
  return manifest;
}

void writeManifest(const std::string& root, const SessionManifest& manifest) {
  requireValidManifestForWrite(manifest);
  const fs::path dir = sessionDir(root, manifest.session_id);
  fs::create_directories(dir);
  const fs::path path = dir / "manifest.json";
  const fs::path tmp_path = dir / "manifest.json.tmp";
  pt::ptree tree;
  manifestToTree(manifest, &tree);
  pt::write_json(tmp_path.string(), tree);
  fs::rename(tmp_path, path);
}

SessionManifest loadManifest(const std::string& root, const std::string& session_id) {
  pt::ptree tree;
  pt::read_json((sessionDir(root, session_id) / "manifest.json").string(), tree);
  if (hasDuplicateJsonKeys(tree)) {
    throw std::invalid_argument("duplicate manifest key");
  }
  const SessionManifest manifest = treeToManifest(tree);
  if (!validManifestForSession(manifest, session_id)) {
    throw std::invalid_argument("invalid session manifest");
  }
  return manifest;
}

void commitManifestSnapshot(const std::string& root, const SessionManifest& manifest, const std::string& event) {
  requireValidManifestForWrite(manifest);
  if (!validSnapshotEvent(event)) {
    throw std::invalid_argument("invalid snapshot event");
  }
  pt::ptree tree;
  tree.put("event", "manifest_snapshot");
  tree.put("snapshot_event", event);
  tree.put("stamp", manifest.updated_at);
  pt::ptree manifest_tree;
  manifestToTree(manifest, &manifest_tree);
  tree.put_child("manifest", manifest_tree);

  std::ostringstream record;
  pt::write_json(record, tree, false);
  std::string line = record.str();
  if (!line.empty() && line[line.size() - 1] == '\n') {
    line.erase(line.size() - 1);
  }
  appendWal(root, manifest.session_id, line);
  writeManifest(root, manifest);
}

bool recoverManifest(const std::string& root, const std::string& session_id, SessionManifest* manifest) {
  if (manifest == nullptr) {
    return false;
  }
  try {
    const SessionManifest candidate = loadManifest(root, session_id);
    if (validManifestForSession(candidate, session_id)) {
      *manifest = candidate;
      return true;
    }
  } catch (const std::exception&) {
  }

  const fs::path wal_path = sessionDir(root, session_id) / "session.wal";
  if (!fs::is_regular_file(wal_path)) {
    return false;
  }

  std::ifstream stream(wal_path.string());
  std::string line;
  bool recovered = false;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    try {
      std::istringstream record_stream(line);
      pt::ptree record;
      pt::read_json(record_stream, record);
      if (hasDuplicateJsonKeys(record)) {
        continue;
      }
      if (record.get<std::string>("event", "") != "manifest_snapshot") {
        continue;
      }
      const SessionManifest candidate = treeToManifest(record.get_child("manifest"));
      if (!validManifestForSession(candidate, session_id)) {
        continue;
      }
      *manifest = candidate;
      recovered = true;
    } catch (const std::exception&) {
    }
  }
  return recovered;
}

void appendWal(const std::string& root, const std::string& session_id, const std::string& json_record) {
  requireValidWalRecord(session_id, json_record);
  const fs::path dir = sessionDir(root, session_id);
  fs::create_directories(dir);
  std::ofstream stream((dir / "session.wal").string(), std::ios::app);
  stream << json_record << "\n";
  stream.flush();
}

std::vector<SessionManifest> listSessions(const std::string& root) {
  std::vector<SessionManifest> sessions;
  if (!fs::is_directory(root)) {
    return sessions;
  }
  for (fs::directory_iterator it(root), end; it != end; ++it) {
    if (!fs::is_directory(it->path())) {
      continue;
    }
    const fs::path manifest_path = it->path() / "manifest.json";
    const fs::path wal_path = it->path() / "session.wal";
    if (!fs::is_regular_file(manifest_path) && !fs::is_regular_file(wal_path)) {
      continue;
    }
    try {
      SessionManifest manifest;
      if (recoverManifest(root, it->path().filename().string(), &manifest)) {
        sessions.push_back(manifest);
      }
    } catch (const std::exception&) {
    }
  }
  return sessions;
}

bool latestSession(const std::string& root, SessionManifest* manifest) {
  const std::vector<SessionManifest> sessions = listSessions(root);
  if (sessions.empty() || manifest == nullptr) {
    return false;
  }
  *manifest = sessions.front();
  for (std::vector<SessionManifest>::const_iterator it = sessions.begin(); it != sessions.end(); ++it) {
    if (it->updated_at > manifest->updated_at) {
      *manifest = *it;
    }
  }
  return true;
}

double nextManifestUpdateStamp(const SessionManifest& manifest,
                               const double observed_time) {
  if (!std::isfinite(observed_time) ||
      observed_time < manifest.updated_at) {
    return manifest.updated_at;
  }
  return observed_time;
}

std::string decideTieredRecovery(
    const SessionManifest* manifest,
    double now,
    double max_resume_age_sec,
    const RecoveryEvidence& evidence,
    const RecoveryThresholds& thresholds) {
  if (manifest == nullptr) {
    return "CREATE_NEW_SESSION";
  }
  if (!std::isfinite(now) ||
      !std::isfinite(max_resume_age_sec) ||
      max_resume_age_sec < 0.0 ||
      !validManifestForSession(*manifest, manifest->session_id) ||
      now < manifest->updated_at ||
      !validRecoveryThresholds(thresholds) ||
      !validRecoveryEvidence(evidence)) {
    return "CREATE_TEMP_SESSION";
  }
  if (now - manifest->updated_at > max_resume_age_sec) {
    return "CREATE_NEW_SESSION";
  }
  if (evidence.local_match_score >= thresholds.local_resume) {
    return "RESUME_LAST_STABLE";
  }
  if (evidence.tca_match_score >= thresholds.tca_resume) {
    return "RECOVER_WITH_TCA";
  }
  if (evidence.global_match_score >= thresholds.global_resume) {
    return "RECOVER_WITH_GLOBAL_CANDIDATE";
  }
  if (evidence.local_match_score >= thresholds.local_relocalize) {
    return "RELOCALIZE_REQUIRED";
  }
  return "CREATE_TEMP_SESSION";
}

OptionalStableRecoveryAnchor loadStableRecoveryAnchor(
    const std::string& stable_map_ledger_path,
    double chainage_m,
    double max_distance_m,
    const std::string& min_quality,
    bool require_loop_verified) {
  OptionalStableRecoveryAnchor result;
  if (!std::isfinite(chainage_m) ||
      !std::isfinite(max_distance_m) ||
      max_distance_m < 0.0 ||
      !validQuality(min_quality)) {
    return result;
  }
  if (!fs::is_regular_file(stable_map_ledger_path)) {
    return result;
  }

  pt::ptree tree;
  try {
    pt::read_json(stable_map_ledger_path, tree);
  } catch (const std::exception&) {
    return result;
  }

  double best_distance = std::numeric_limits<double>::infinity();
  for (pt::ptree::const_assoc_iterator it = tree.find("entries"); it != tree.not_found(); ++it) {
    for (pt::ptree::const_iterator item = it->second.begin(); item != it->second.end(); ++item) {
      try {
        const std::string quality = item->second.get<std::string>("section_quality", "C");
        const bool loop_verified = item->second.get<bool>("loop_verified", false);
        const double entry_chainage = item->second.get<double>("chainage_m");
        const std::string keyframe_id = item->second.get<std::string>("keyframe_id");
        if (!validKeyframeId(keyframe_id) || !std::isfinite(entry_chainage) ||
            !validQuality(quality)) {
          continue;
        }
        if (qualityRank(quality) > qualityRank(min_quality)) {
          continue;
        }
        if (require_loop_verified && !loop_verified) {
          continue;
        }
        const double distance = std::fabs(entry_chainage - chainage_m);
        if (distance > max_distance_m || distance >= best_distance) {
          continue;
        }
        result.has_value = true;
        result.value.keyframe_id = keyframe_id;
        result.value.chainage_m = entry_chainage;
        result.value.section_quality = quality;
        result.value.loop_verified = loop_verified;
        best_distance = distance;
      } catch (const std::exception&) {
      }
    }
    break;
  }
  return result;
}

StableAnchorRecoveryDecision decideStableAnchorRecovery(
    const std::string& base_action,
    const StableRecoveryAnchor* anchor,
    const RecoveryAlignment* alignment,
    const RecoveryAlignmentGate& gate) {
  StableAnchorRecoveryDecision decision;
  decision.action = base_action;
  if (base_action == "CREATE_NEW_SESSION") {
    decision.reason = "base_action_requires_new_session";
    return decision;
  }
  if (anchor == nullptr) {
    decision.reason = "stable_anchor_missing";
    return decision;
  }
  if (!validStableAnchor(*anchor)) {
    decision.reason = "invalid_stable_anchor";
    return decision;
  }
  if (alignment == nullptr) {
    decision.reason = "stable_anchor_alignment_missing";
    return decision;
  }
  if (!validAlignmentGate(gate)) {
    decision.reason = "invalid_alignment_gate";
    return decision;
  }
  if (!validAlignment(*alignment)) {
    decision.reason = "invalid_alignment";
    return decision;
  }

  decision.translation_m = std::sqrt(
      alignment->dx_m * alignment->dx_m + alignment->dy_m * alignment->dy_m + alignment->dz_m * alignment->dz_m);
  if (!alignment->converged) {
    decision.reason = "alignment_not_converged";
    return decision;
  }
  if (alignment->rmse_m > gate.max_rmse_m) {
    decision.reason = "rmse_exceeds_gate";
    return decision;
  }
  if (alignment->inlier_ratio < gate.min_inlier_ratio) {
    decision.reason = "inlier_ratio_below_gate";
    return decision;
  }
  if (decision.translation_m > gate.max_translation_m) {
    decision.reason = "translation_exceeds_gate";
    return decision;
  }
  if (std::fabs(alignment->yaw_deg) > gate.max_yaw_deg) {
    decision.reason = "yaw_exceeds_gate";
    return decision;
  }
  decision.action = "RECOVER_WITH_STABLE_ANCHOR";
  decision.accepted = true;
  decision.reason = "stable_anchor_alignment_accepted";
  return decision;
}

RecoveryAlignment estimateStableAnchorAlignment(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& current_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& stable_anchor_cloud,
    const RecoveryAlignmentOptions& options) {
  RecoveryAlignment alignment;
  if (!validAlignmentOptions(options)) {
    return alignment;
  }
  const std::size_t min_points =
      static_cast<std::size_t>(std::max(1, options.min_points));
  if (!finiteRecoveryCloud(current_cloud) ||
      !finiteRecoveryCloud(stable_anchor_cloud) ||
      current_cloud->size() < min_points ||
      stable_anchor_cloud->size() < min_points) {
    return alignment;
  }

  lio_local_odometry::MultiScaleIcpConfig config;
  config.voxel_leaf_sizes = options.voxel_leaf_sizes;
  config.max_correspondence_distance = options.max_correspondence_distance_m;
  config.transformation_epsilon = options.transformation_epsilon;
  config.euclidean_fitness_epsilon = options.euclidean_fitness_epsilon;
  config.max_iterations = options.max_iterations;
  config.min_points = min_points;

  const lio_local_odometry::MultiScaleIcpRegistration registration(config);
  const lio_local_odometry::MultiScaleIcpResult result =
      registration.align(current_cloud, stable_anchor_cloud);
  alignment.converged = result.converged;
  alignment.dx_m = result.source_to_target(0, 3);
  alignment.dy_m = result.source_to_target(1, 3);
  alignment.dz_m = result.source_to_target(2, 3);
  alignment.yaw_deg = std::atan2(result.source_to_target(1, 0),
                                 result.source_to_target(0, 0)) *
                      180.0 / M_PI;

  if (!result.converged) {
    return alignment;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*current_cloud, *aligned, result.source_to_target);

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(stable_anchor_cloud);
  const double inlier_threshold_sq =
      options.inlier_threshold_m * options.inlier_threshold_m;
  double sum_sq = 0.0;
  std::size_t matched = 0;
  std::size_t inliers = 0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (const auto& point : aligned->points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    if (kdtree.nearestKSearch(point, 1, indices, distances) <= 0) {
      continue;
    }
    ++matched;
    sum_sq += distances.front();
    if (distances.front() <= inlier_threshold_sq) {
      ++inliers;
    }
  }

  if (matched > 0) {
    alignment.rmse_m = std::sqrt(sum_sq / static_cast<double>(matched));
    alignment.inlier_ratio =
        static_cast<double>(inliers) / static_cast<double>(matched);
  }
  return alignment;
}

std::string jsonEscape(const std::string& value) {
  std::ostringstream escaped;
  for (std::string::const_iterator it = value.begin(); it != value.end(); ++it) {
    if (*it == '"' || *it == '\\') {
      escaped << '\\';
    }
    escaped << *it;
  }
  return escaped.str();
}

}  // namespace lio_session_manager
