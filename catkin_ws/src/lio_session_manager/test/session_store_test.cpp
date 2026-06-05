#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lio_session_manager/session_store.h"

namespace fs = boost::filesystem;
using namespace lio_session_manager;

namespace {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

CloudT::Ptr makeRecoveryCloud() {
  CloudT::Ptr cloud(new CloudT);
  for (int i = 0; i < 140; ++i) {
    const float t = static_cast<float>(i);
    PointT point;
    point.x = 0.035f * t + 0.020f * std::sin(0.37f * t);
    point.y = 0.45f * std::sin(0.19f * t) + 0.013f * t;
    point.z = 0.22f * std::cos(0.23f * t) + 0.004f * t;
    point.intensity = static_cast<float>(i % 31);
    cloud->push_back(point);
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

}  // namespace

class SessionStoreTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() / fs::unique_path("session_store_test_%%%%%%");
    fs::create_directories(root_);
  }

  void TearDown() override {
    fs::remove_all(root_);
  }

  fs::path root_;
};

TEST_F(SessionStoreTest, WritesManifestAndWalAtomically) {
  SessionManifest manifest = createSession(root_.string(), "s1", 100.0);
  manifest.chainage_m = 12.5;
  writeManifest(root_.string(), manifest);

  SessionManifest loaded = loadManifest(root_.string(), "s1");

  EXPECT_EQ("s1", loaded.session_id);
  EXPECT_DOUBLE_EQ(12.5, loaded.chainage_m);
  EXPECT_TRUE(fs::is_regular_file(root_ / "s1" / "session.wal"));
}

TEST_F(SessionStoreTest, NextManifestUpdateStampNeverRegressesBelowCurrentManifestTime) {
  SessionManifest manifest = createSession(root_.string(), "sim_replay", 200.0);

  EXPECT_DOUBLE_EQ(200.0, nextManifestUpdateStamp(manifest, 150.0));
  EXPECT_DOUBLE_EQ(250.0, nextManifestUpdateStamp(manifest, 250.0));
  EXPECT_DOUBLE_EQ(200.0, nextManifestUpdateStamp(
                              manifest,
                              std::numeric_limits<double>::quiet_NaN()));
}

TEST_F(SessionStoreTest, RecoversLatestCommittedManifestFromWalWhenManifestIsMissing) {
  SessionManifest manifest = createSession(root_.string(), "s1", 100.0);
  manifest.updated_at = 125.0;
  manifest.state = "ACTIVE";
  manifest.chainage_m = 18.75;
  manifest.last_stable_x_m = 1.2;
  manifest.last_stable_y_m = -0.4;
  manifest.last_stable_z_m = 0.1;
  manifest.last_stable_yaw_deg = 2.5;
  manifest.wal_seq = 7;
  commitManifestSnapshot(root_.string(), manifest, "stable_pose");
  fs::remove(root_ / "s1" / "manifest.json");

  SessionManifest recovered;
  ASSERT_TRUE(recoverManifest(root_.string(), "s1", &recovered));

  EXPECT_EQ("s1", recovered.session_id);
  EXPECT_EQ("ACTIVE", recovered.state);
  EXPECT_DOUBLE_EQ(18.75, recovered.chainage_m);
  EXPECT_DOUBLE_EQ(1.2, recovered.last_stable_x_m);
  EXPECT_DOUBLE_EQ(-0.4, recovered.last_stable_y_m);
  EXPECT_DOUBLE_EQ(0.1, recovered.last_stable_z_m);
  EXPECT_DOUBLE_EQ(2.5, recovered.last_stable_yaw_deg);
  EXPECT_EQ(7, recovered.wal_seq);
}

TEST_F(SessionStoreTest, WalRecoveryIgnoresSnapshotsForDifferentSession) {
  SessionManifest manifest = createSession(root_.string(), "s1", 100.0);
  manifest.updated_at = 125.0;
  manifest.chainage_m = 18.75;
  manifest.wal_seq = 7;
  commitManifestSnapshot(root_.string(), manifest, "stable_pose");
  std::ofstream wal((root_ / "s1" / "session.wal").string(), std::ios::app);
  wal << R"json({"event":"manifest_snapshot","snapshot_event":"wrong_session","stamp":300.0,"manifest":{"session_id":"other","created_at":10.0,"updated_at":300.0,"state":"ACTIVE","chainage_m":99.0,"last_stable_pose":{"x_m":9.0,"y_m":9.0,"z_m":9.0,"yaw_deg":90.0},"wal_seq":99}})json"
      << "\n";
  wal.close();
  fs::remove(root_ / "s1" / "manifest.json");

  SessionManifest recovered;
  ASSERT_TRUE(recoverManifest(root_.string(), "s1", &recovered));

  EXPECT_EQ("s1", recovered.session_id);
  EXPECT_DOUBLE_EQ(18.75, recovered.chainage_m);
  EXPECT_EQ(7, recovered.wal_seq);
}

TEST_F(SessionStoreTest, WalRecoverySkipsDuplicateKeySnapshots) {
  SessionManifest manifest = createSession(root_.string(), "s1", 100.0);
  manifest.updated_at = 125.0;
  manifest.chainage_m = 18.75;
  manifest.wal_seq = 7;
  commitManifestSnapshot(root_.string(), manifest, "stable_pose");

  std::ofstream wal((root_ / "s1" / "session.wal").string(), std::ios::app);
  wal << R"json({"event":"manifest_snapshot","event":"recover","snapshot_event":"duplicate_event","stamp":300.0,"manifest":{"session_id":"s1","session_id":"other","created_at":100.0,"updated_at":300.0,"state":"ACTIVE","chainage_m":99.0,"last_stable_pose":{"x_m":9.0,"y_m":9.0,"z_m":9.0,"yaw_deg":90.0},"wal_seq":99}})json"
      << "\n";
  wal.close();
  fs::remove(root_ / "s1" / "manifest.json");

  SessionManifest recovered;
  ASSERT_TRUE(recoverManifest(root_.string(), "s1", &recovered));

  EXPECT_EQ("s1", recovered.session_id);
  EXPECT_DOUBLE_EQ(18.75, recovered.chainage_m);
  EXPECT_EQ(7, recovered.wal_seq);
}

TEST_F(SessionStoreTest, SessionPathApisRejectPollutedSessionIds) {
  SessionManifest recovered;

  EXPECT_THROW(
      createSession(root_.string(), "bad;state=ACTIVE", 100.0),
      std::invalid_argument);
  EXPECT_THROW(
      appendWal(root_.string(), "bad state", R"json({"event":"test"})json"),
      std::invalid_argument);
  EXPECT_THROW(loadManifest(root_.string(), "bad/session"), std::invalid_argument);
  EXPECT_THROW(
      recoverManifest(root_.string(), "bad state", &recovered),
      std::invalid_argument);
}

TEST_F(SessionStoreTest, AppendWalRejectsMultilineRecordsBeforeWriting) {
  createSession(root_.string(), "s1", 100.0);
  fs::remove(root_ / "s1" / "session.wal");

  const std::string injected_snapshot =
      R"json({"event":"recover"})json"
      "\n"
      R"json({"event":"manifest_snapshot","snapshot_event":"injected","stamp":999.0,"manifest":{"session_id":"s1","created_at":100.0,"updated_at":999.0,"state":"ACTIVE","chainage_m":99.0,"last_stable_pose":{"x_m":9.0,"y_m":9.0,"z_m":9.0,"yaw_deg":90.0},"wal_seq":99}})json";

  EXPECT_THROW(
      appendWal(root_.string(), "s1", injected_snapshot),
      std::invalid_argument);
  EXPECT_THROW(
      appendWal(root_.string(), "s1", R"json({"event":"recover"})json" "\r"),
      std::invalid_argument);
  EXPECT_FALSE(fs::is_regular_file(root_ / "s1" / "session.wal"));
}

TEST_F(SessionStoreTest, AppendWalRejectsMalformedOrUnknownEventRecordsBeforeWriting) {
  createSession(root_.string(), "s1", 100.0);
  fs::remove(root_ / "s1" / "session.wal");

  EXPECT_THROW(appendWal(root_.string(), "s1", "{not-json"), std::invalid_argument);
  EXPECT_THROW(
      appendWal(root_.string(), "s1", R"json({"stamp":100.0})json"),
      std::invalid_argument);
  EXPECT_THROW(
      appendWal(root_.string(), "s1", R"json({"event":"recover;status=PASS"})json"),
      std::invalid_argument);
  EXPECT_THROW(
      appendWal(root_.string(), "s1", R"json({"event":"unknown"})json"),
      std::invalid_argument);
  EXPECT_FALSE(fs::is_regular_file(root_ / "s1" / "session.wal"));
}

TEST_F(SessionStoreTest, AppendWalRejectsDuplicateJsonKeysBeforeWriting) {
  createSession(root_.string(), "s1", 100.0);
  fs::remove(root_ / "s1" / "session.wal");

  EXPECT_THROW(
      appendWal(root_.string(), "s1", R"json({"event":"recover","event":"manifest_snapshot"})json"),
      std::invalid_argument);
  EXPECT_THROW(
      appendWal(
          root_.string(),
          "s1",
          R"json({"event":"manifest_snapshot","snapshot_event":"duplicate_manifest_session","stamp":200.0,"manifest":{"session_id":"s1","session_id":"other","created_at":100.0,"updated_at":200.0,"state":"ACTIVE","chainage_m":12.0,"last_stable_pose":{"x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_deg":0.0},"wal_seq":1}})json"),
      std::invalid_argument);
  EXPECT_FALSE(fs::is_regular_file(root_ / "s1" / "session.wal"));
}

TEST_F(SessionStoreTest, AppendWalRejectsPollutedManifestSnapshotPayloadBeforeWriting) {
  createSession(root_.string(), "s1", 100.0);
  fs::remove(root_ / "s1" / "session.wal");

  EXPECT_THROW(
      appendWal(
          root_.string(),
          "s1",
          R"json({"event":"manifest_snapshot","snapshot_event":"stable_pose;field_acceptance_status=PASS","stamp":200.0,"manifest":{"session_id":"s1","created_at":100.0,"updated_at":200.0,"state":"ACTIVE","chainage_m":12.0,"last_stable_pose":{"x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_deg":0.0},"wal_seq":1}})json"),
      std::invalid_argument);
  EXPECT_THROW(
      appendWal(
          root_.string(),
          "s1",
          R"json({"event":"manifest_snapshot","snapshot_event":"stable_pose","stamp":200.0,"manifest":{"session_id":"s1","created_at":100.0,"updated_at":200.0,"state":"ACTIVE;field_acceptance_status=PASS","chainage_m":12.0,"last_stable_pose":{"x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_deg":0.0},"wal_seq":1}})json"),
      std::invalid_argument);

  EXPECT_FALSE(fs::is_regular_file(root_ / "s1" / "session.wal"));
}

TEST_F(SessionStoreTest, AppendWalRejectsPollutedRecoverPayloadBeforeWriting) {
  createSession(root_.string(), "s1", 100.0);
  fs::remove(root_ / "s1" / "session.wal");

  EXPECT_THROW(
      appendWal(
          root_.string(),
          "s1",
          R"json({"event":"recover","stamp":200.0,"base_action":"RESUME_LAST_STABLE;field_acceptance_status=PASS","action":"RESUME_LAST_STABLE","stable_anchor_decision":{"accepted":false,"reason":"stable_anchor_missing","translation_m":0.0}})json"),
      std::invalid_argument);
  EXPECT_THROW(
      appendWal(
          root_.string(),
          "s1",
          R"json({"event":"recover","stamp":200.0,"base_action":"CREATE_TEMP_SESSION","action":"RECOVER_WITH_STABLE_ANCHOR","stable_anchor_decision":{"accepted":true,"reason":"stable_anchor_alignment_accepted","translation_m":0.1},"stable_anchor":{"keyframe_id":"kf_01;field_acceptance_status=PASS","chainage_m":10.0,"section_quality":"A","loop_verified":true}})json"),
      std::invalid_argument);

  EXPECT_FALSE(fs::is_regular_file(root_ / "s1" / "session.wal"));
}

TEST_F(SessionStoreTest, AppendWalRejectsInconsistentWalEvidenceBeforeWriting) {
  createSession(root_.string(), "s1", 100.0);
  fs::remove(root_ / "s1" / "session.wal");

  EXPECT_THROW(
      appendWal(
          root_.string(),
          "s1",
          R"json({"event":"manifest_snapshot","snapshot_event":"stable_pose","stamp":201.0,"manifest":{"session_id":"s1","created_at":100.0,"updated_at":200.0,"state":"ACTIVE","chainage_m":12.0,"last_stable_pose":{"x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_deg":0.0},"wal_seq":1}})json"),
      std::invalid_argument);
  EXPECT_THROW(
      appendWal(
          root_.string(),
          "s1",
          R"json({"event":"recover","stamp":200.0,"base_action":"CREATE_TEMP_SESSION","action":"RESUME_LAST_STABLE","stable_anchor_decision":{"accepted":false,"reason":"stable_anchor_missing","translation_m":0.0}})json"),
      std::invalid_argument);
  EXPECT_THROW(
      appendWal(
          root_.string(),
          "s1",
          R"json({"event":"recover","stamp":200.0,"base_action":"CREATE_TEMP_SESSION","action":"CREATE_TEMP_SESSION","stable_anchor_decision":{"accepted":false,"reason":"stable_anchor_alignment_accepted","translation_m":0.0}})json"),
      std::invalid_argument);

  EXPECT_FALSE(fs::is_regular_file(root_ / "s1" / "session.wal"));
}

TEST_F(SessionStoreTest, LatestSessionSkipsPollutedSessionDirectories) {
  SessionManifest safe = createSession(root_.string(), "safe", 100.0);
  safe.updated_at = 110.0;
  safe.chainage_m = 1.25;
  commitManifestSnapshot(root_.string(), safe, "safe_snapshot");

  const fs::path polluted_dir = root_ / "bad;state=ACTIVE";
  fs::create_directories(polluted_dir);
  std::ofstream polluted((polluted_dir / "manifest.json").string());
  polluted << R"json({
    "session_id": "bad;state=ACTIVE",
    "created_at": 100.0,
    "updated_at": 999.0,
    "state": "ACTIVE",
    "chainage_m": 99.0,
    "last_stable_pose": {
      "x_m": 0.0,
      "y_m": 0.0,
      "z_m": 0.0,
      "yaw_deg": 0.0
    },
    "wal_seq": 0
  })json";
  polluted.close();

  SessionManifest latest;
  ASSERT_TRUE(latestSession(root_.string(), &latest));

  EXPECT_EQ("safe", latest.session_id);
  EXPECT_DOUBLE_EQ(1.25, latest.chainage_m);
}

TEST_F(SessionStoreTest, RecoveryRejectsPollutedManifestState) {
  const fs::path dir = root_ / "s1";
  fs::create_directories(dir);
  std::ofstream manifest((dir / "manifest.json").string());
  manifest << R"json({
    "session_id": "s1",
    "created_at": 100.0,
    "updated_at": 110.0,
    "state": "ACTIVE;field_acceptance_status=PASS",
    "chainage_m": 12.5,
    "last_stable_pose": {
      "x_m": 0.0,
      "y_m": 0.0,
      "z_m": 0.0,
      "yaw_deg": 0.0
    },
    "wal_seq": 1
  })json";
  manifest.close();

  SessionManifest recovered;
  EXPECT_FALSE(recoverManifest(root_.string(), "s1", &recovered));
}

TEST_F(SessionStoreTest, LoadManifestRejectsPollutedManifestBeforeReturning) {
  const fs::path dir = root_ / "s1";
  fs::create_directories(dir);
  std::ofstream manifest((dir / "manifest.json").string());
  manifest << R"json({
    "session_id": "other",
    "created_at": 100.0,
    "updated_at": 110.0,
    "state": "ACTIVE;field_acceptance_status=PASS",
    "chainage_m": 12.5,
    "last_stable_pose": {
      "x_m": 0.0,
      "y_m": 0.0,
      "z_m": 0.0,
      "yaw_deg": 0.0
    },
    "wal_seq": 1
  })json";
  manifest.close();

  EXPECT_THROW(loadManifest(root_.string(), "s1"), std::invalid_argument);
}

TEST_F(SessionStoreTest, LoadManifestRejectsDuplicateJsonKeysBeforeReturning) {
  const fs::path dir = root_ / "s1";
  fs::create_directories(dir);
  std::ofstream manifest((dir / "manifest.json").string());
  manifest << R"json({
    "session_id": "s1",
    "session_id": "other",
    "created_at": 100.0,
    "updated_at": 110.0,
    "state": "ACTIVE",
    "chainage_m": 12.5,
    "last_stable_pose": {
      "x_m": 0.0,
      "y_m": 0.0,
      "z_m": 0.0,
      "yaw_deg": 0.0
    },
    "wal_seq": 1
  })json";
  manifest.close();

  EXPECT_THROW(loadManifest(root_.string(), "s1"), std::invalid_argument);
}

TEST_F(SessionStoreTest, WriteManifestRejectsPollutedStateWithoutOverwritingExistingManifest) {
  SessionManifest manifest = createSession(root_.string(), "s1", 100.0);
  manifest.state = "ACTIVE;field_acceptance_status=PASS";
  manifest.chainage_m = 99.0;

  EXPECT_THROW(writeManifest(root_.string(), manifest), std::invalid_argument);

  const SessionManifest loaded = loadManifest(root_.string(), "s1");
  EXPECT_EQ("ACTIVE", loaded.state);
  EXPECT_DOUBLE_EQ(0.0, loaded.chainage_m);
}

TEST_F(SessionStoreTest, ManifestTimeGateRejectsNegativeAndRegressiveTimesBeforeWriting) {
  SessionManifest manifest = createSession(root_.string(), "s1", 100.0);

  SessionManifest negative_time = manifest;
  negative_time.created_at = -1.0;
  negative_time.updated_at = 200.0;
  EXPECT_THROW(writeManifest(root_.string(), negative_time), std::invalid_argument);

  SessionManifest regressive_time = manifest;
  regressive_time.updated_at = 99.0;
  EXPECT_THROW(writeManifest(root_.string(), regressive_time), std::invalid_argument);

  fs::remove(root_ / "s1" / "session.wal");
  EXPECT_THROW(
      appendWal(
          root_.string(),
          "s1",
          R"json({"event":"manifest_snapshot","snapshot_event":"stable_pose","stamp":200.0,"manifest":{"session_id":"s1","created_at":300.0,"updated_at":200.0,"state":"ACTIVE","chainage_m":12.0,"last_stable_pose":{"x_m":0.0,"y_m":0.0,"z_m":0.0,"yaw_deg":0.0},"wal_seq":1}})json"),
      std::invalid_argument);

  EXPECT_FALSE(fs::is_regular_file(root_ / "s1" / "session.wal"));
  const SessionManifest loaded = loadManifest(root_.string(), "s1");
  EXPECT_DOUBLE_EQ(100.0, loaded.created_at);
  EXPECT_DOUBLE_EQ(100.0, loaded.updated_at);
}

TEST_F(SessionStoreTest, CommitSnapshotRejectsPollutedStateBeforeWalAppend) {
  SessionManifest manifest = createSession(root_.string(), "s1", 100.0);
  fs::remove(root_ / "s1" / "session.wal");
  manifest.state = "ACTIVE;field_acceptance_status=PASS";
  manifest.updated_at = 200.0;
  manifest.wal_seq = 10;

  EXPECT_THROW(
      commitManifestSnapshot(root_.string(), manifest, "polluted_snapshot"),
      std::invalid_argument);

  EXPECT_FALSE(fs::is_regular_file(root_ / "s1" / "session.wal"));
  const SessionManifest loaded = loadManifest(root_.string(), "s1");
  EXPECT_EQ("ACTIVE", loaded.state);
  EXPECT_EQ(0, loaded.wal_seq);
}

TEST_F(SessionStoreTest, CommitSnapshotRejectsPollutedSnapshotEventBeforeWalAppend) {
  SessionManifest manifest = createSession(root_.string(), "s1", 100.0);
  fs::remove(root_ / "s1" / "session.wal");
  manifest.updated_at = 200.0;
  manifest.chainage_m = 99.0;
  manifest.wal_seq = 10;

  EXPECT_THROW(
      commitManifestSnapshot(root_.string(), manifest, "stable_pose;field_acceptance_status=PASS"),
      std::invalid_argument);
  EXPECT_THROW(
      commitManifestSnapshot(root_.string(), manifest, "stable_pose\ninjected"),
      std::invalid_argument);

  EXPECT_FALSE(fs::is_regular_file(root_ / "s1" / "session.wal"));
  const SessionManifest loaded = loadManifest(root_.string(), "s1");
  EXPECT_DOUBLE_EQ(0.0, loaded.chainage_m);
  EXPECT_EQ(0, loaded.wal_seq);
}

TEST_F(SessionStoreTest, WalRecoverySkipsPollutedStateSnapshots) {
  SessionManifest valid = createSession(root_.string(), "s1", 100.0);
  valid.updated_at = 125.0;
  valid.state = "ACTIVE";
  valid.chainage_m = 18.75;
  valid.wal_seq = 7;
  commitManifestSnapshot(root_.string(), valid, "valid_snapshot");
  std::ofstream wal((root_ / "s1" / "session.wal").string(), std::ios::app);
  wal << R"json({"event":"manifest_snapshot","snapshot_event":"polluted_state","stamp":300.0,"manifest":{"session_id":"s1","created_at":100.0,"updated_at":300.0,"state":"ACTIVE;field_acceptance_status=PASS","chainage_m":99.0,"last_stable_pose":{"x_m":9.0,"y_m":9.0,"z_m":9.0,"yaw_deg":90.0},"wal_seq":99}})json"
      << "\n";
  wal.close();
  fs::remove(root_ / "s1" / "manifest.json");

  SessionManifest recovered;
  ASSERT_TRUE(recoverManifest(root_.string(), "s1", &recovered));

  EXPECT_EQ("ACTIVE", recovered.state);
  EXPECT_DOUBLE_EQ(18.75, recovered.chainage_m);
  EXPECT_EQ(7, recovered.wal_seq);
}

TEST_F(SessionStoreTest, LatestSessionUsesWalRecoveryWhenManifestIsCorrupt) {
  SessionManifest older = createSession(root_.string(), "older", 100.0);
  older.updated_at = 110.0;
  commitManifestSnapshot(root_.string(), older, "older_snapshot");

  SessionManifest newer = createSession(root_.string(), "newer", 100.0);
  newer.updated_at = 180.0;
  newer.chainage_m = 42.0;
  newer.wal_seq = 3;
  commitManifestSnapshot(root_.string(), newer, "newer_snapshot");

  std::ofstream corrupt((root_ / "newer" / "manifest.json").string());
  corrupt << "{not-json";
  corrupt.close();

  SessionManifest latest;
  ASSERT_TRUE(latestSession(root_.string(), &latest));

  EXPECT_EQ("newer", latest.session_id);
  EXPECT_DOUBLE_EQ(42.0, latest.chainage_m);
  EXPECT_EQ(3, latest.wal_seq);
}

TEST_F(SessionStoreTest, TieredRecoveryPrefersLocalTcaThenGlobal) {
  SessionManifest manifest = createSession(root_.string(), "s1", 100.0);
  manifest.updated_at = 150.0;
  RecoveryThresholds thresholds;
  thresholds.local_resume = 0.8;
  thresholds.tca_resume = 0.75;
  thresholds.global_resume = 0.7;

  EXPECT_EQ("RESUME_LAST_STABLE",
            decideTieredRecovery(&manifest, 151.0, 100.0, RecoveryEvidence{0.9, 0.9, 0.9}, thresholds));
  EXPECT_EQ("RECOVER_WITH_TCA",
            decideTieredRecovery(&manifest, 151.0, 100.0, RecoveryEvidence{0.2, 0.8, 0.9}, thresholds));
  EXPECT_EQ("RECOVER_WITH_GLOBAL_CANDIDATE",
            decideTieredRecovery(&manifest, 151.0, 100.0, RecoveryEvidence{0.2, 0.1, 0.8}, thresholds));
  EXPECT_EQ("CREATE_TEMP_SESSION",
            decideTieredRecovery(&manifest, 151.0, 100.0, RecoveryEvidence{0.2, 0.1, 0.1}, thresholds));
}

TEST_F(SessionStoreTest, TieredRecoveryRejectsInvalidScoresAndThresholds) {
  SessionManifest manifest = createSession(root_.string(), "s1", 100.0);
  manifest.updated_at = 150.0;
  RecoveryThresholds thresholds;
  thresholds.local_resume = 0.8;
  thresholds.local_relocalize = 0.5;
  thresholds.tca_resume = 0.75;
  thresholds.global_resume = 0.7;

  RecoveryEvidence invalid_evidence{std::numeric_limits<double>::quiet_NaN(), 0.95, 0.95};
  EXPECT_EQ("CREATE_TEMP_SESSION",
            decideTieredRecovery(&manifest, 151.0, 100.0, invalid_evidence, thresholds));

  RecoveryThresholds invalid_thresholds = thresholds;
  invalid_thresholds.tca_resume = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ("CREATE_TEMP_SESSION",
            decideTieredRecovery(&manifest, 151.0, 100.0, RecoveryEvidence{0.1, 0.95, 0.95}, invalid_thresholds));
}

TEST_F(SessionStoreTest, TieredRecoveryRejectsInvalidManifestAndFutureSnapshotTime) {
  SessionManifest manifest = createSession(root_.string(), "s1", 100.0);
  manifest.updated_at = 150.0;
  RecoveryThresholds thresholds;
  thresholds.local_resume = 0.8;
  thresholds.local_relocalize = 0.5;
  thresholds.tca_resume = 0.75;
  thresholds.global_resume = 0.7;
  const RecoveryEvidence strong_evidence{0.95, 0.95, 0.95};

  SessionManifest polluted_state = manifest;
  polluted_state.state = "ACTIVE;field_acceptance_status=PASS";
  EXPECT_EQ("CREATE_TEMP_SESSION",
            decideTieredRecovery(&polluted_state, 151.0, 100.0, strong_evidence, thresholds));

  SessionManifest regressive_time = manifest;
  regressive_time.updated_at = 99.0;
  EXPECT_EQ("CREATE_TEMP_SESSION",
            decideTieredRecovery(&regressive_time, 151.0, 100.0, strong_evidence, thresholds));

  SessionManifest future_snapshot = manifest;
  future_snapshot.updated_at = 200.0;
  EXPECT_EQ("CREATE_TEMP_SESSION",
            decideTieredRecovery(&future_snapshot, 151.0, 100.0, strong_evidence, thresholds));
}

TEST_F(SessionStoreTest, LoadsNearestStableRecoveryAnchorFromBackendLedger) {
  const fs::path ledger = root_ / "stable_map.json";
  std::ofstream stream(ledger.string());
  stream << R"json({
    "entries": [
      {"keyframe_id": "bad_quality", "chainage_m": 8.0, "section_quality": "C", "promoted_at": 100.0, "loop_verified": true},
      {"keyframe_id": "near", "chainage_m": 10.0, "section_quality": "B", "promoted_at": 101.0, "loop_verified": false},
      {"keyframe_id": "verified", "chainage_m": 11.0, "section_quality": "A", "promoted_at": 102.0, "loop_verified": true}
    ]
  })json";
  stream.close();

  OptionalStableRecoveryAnchor anchor = loadStableRecoveryAnchor(ledger.string(), 10.7, 2.0, "B", true);

  ASSERT_TRUE(anchor.has_value);
  EXPECT_EQ("verified", anchor.value.keyframe_id);
  EXPECT_DOUBLE_EQ(11.0, anchor.value.chainage_m);
}

TEST_F(SessionStoreTest, StableRecoveryAnchorLookupRejectsInvalidQueryAndGate) {
  const fs::path ledger = root_ / "stable_map.json";
  std::ofstream stream(ledger.string());
  stream << R"json({
    "entries": [
      {"keyframe_id": "near", "chainage_m": 10.0, "section_quality": "A", "promoted_at": 101.0, "loop_verified": true}
    ]
  })json";
  stream.close();

  EXPECT_FALSE(loadStableRecoveryAnchor(
                   ledger.string(),
                   std::numeric_limits<double>::quiet_NaN(),
                   2.0,
                   "B",
                   true)
                   .has_value);
  EXPECT_FALSE(loadStableRecoveryAnchor(
                   ledger.string(),
                   10.0,
                   std::numeric_limits<double>::quiet_NaN(),
                   "B",
                   true)
                   .has_value);
  EXPECT_FALSE(loadStableRecoveryAnchor(ledger.string(), 10.0, 2.0, "Z", true).has_value);
}

TEST_F(SessionStoreTest, StableRecoveryAnchorLookupSkipsPollutedKeyframeIds) {
  const fs::path ledger = root_ / "stable_map.json";
  std::ofstream stream(ledger.string());
  stream << R"json({
    "entries": [
      {"keyframe_id": "bad;quality=A", "chainage_m": 10.0, "section_quality": "A", "promoted_at": 101.0, "loop_verified": true},
      {"keyframe_id": "..", "chainage_m": 10.1, "section_quality": "A", "promoted_at": 102.0, "loop_verified": true},
      {"keyframe_id": "valid_anchor", "chainage_m": 10.2, "section_quality": "A", "promoted_at": 103.0, "loop_verified": true}
    ]
  })json";
  stream.close();

  const OptionalStableRecoveryAnchor anchor =
      loadStableRecoveryAnchor(ledger.string(), 10.0, 1.0, "B", true);

  ASSERT_TRUE(anchor.has_value);
  EXPECT_EQ("valid_anchor", anchor.value.keyframe_id);
  EXPECT_DOUBLE_EQ(10.2, anchor.value.chainage_m);
}

TEST_F(SessionStoreTest, StableAnchorRecoveryRequiresGoodLocalAlignment) {
  StableRecoveryAnchor anchor{"kf_12", 12.0, "A", true};
  RecoveryAlignment alignment{true, 0.04, 0.82, 0.08, -0.04, 0.01, 1.2};
  RecoveryAlignmentGate gate{0.08, 0.7, 0.25, 3.0};

  StableAnchorRecoveryDecision decision =
      decideStableAnchorRecovery("CREATE_TEMP_SESSION", &anchor, &alignment, gate);

  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ("RECOVER_WITH_STABLE_ANCHOR", decision.action);
  EXPECT_EQ("stable_anchor_alignment_accepted", decision.reason);
}

TEST_F(SessionStoreTest, StableAnchorRecoveryRejectsPollutedAnchorKeyframeId) {
  StableRecoveryAnchor anchor{"kf_12;chainage=spoof", 12.0, "A", true};
  RecoveryAlignment alignment{true, 0.04, 0.82, 0.08, -0.04, 0.01, 1.2};
  RecoveryAlignmentGate gate{0.08, 0.7, 0.25, 3.0};

  const StableAnchorRecoveryDecision decision =
      decideStableAnchorRecovery("CREATE_TEMP_SESSION", &anchor, &alignment, gate);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ("CREATE_TEMP_SESSION", decision.action);
  EXPECT_EQ("invalid_stable_anchor", decision.reason);
}

TEST_F(SessionStoreTest, StableAnchorRecoveryRejectsLargePowerLossDisplacement) {
  StableRecoveryAnchor anchor{"kf_12", 12.0, "A", true};
  RecoveryAlignment alignment{true, 0.03, 0.9, 0.4, 0.0, 0.0, 0.5};
  RecoveryAlignmentGate gate{0.08, 0.7, 0.25, 3.0};

  StableAnchorRecoveryDecision decision =
      decideStableAnchorRecovery("CREATE_TEMP_SESSION", &anchor, &alignment, gate);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ("CREATE_TEMP_SESSION", decision.action);
  EXPECT_EQ("translation_exceeds_gate", decision.reason);
}

TEST_F(SessionStoreTest, StableAnchorRecoveryRejectsInvalidAlignmentAndGate) {
  StableRecoveryAnchor anchor{"kf_12", 12.0, "A", true};
  RecoveryAlignment invalid_alignment{
      true,
      std::numeric_limits<double>::quiet_NaN(),
      0.9,
      0.02,
      0.0,
      0.0,
      0.5};
  RecoveryAlignmentGate gate{0.08, 0.7, 0.25, 3.0};

  StableAnchorRecoveryDecision decision =
      decideStableAnchorRecovery("CREATE_TEMP_SESSION", &anchor, &invalid_alignment, gate);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ("CREATE_TEMP_SESSION", decision.action);
  EXPECT_EQ("invalid_alignment", decision.reason);

  RecoveryAlignment valid_alignment{true, 0.03, 0.9, 0.02, 0.0, 0.0, 0.5};
  RecoveryAlignmentGate invalid_gate{
      std::numeric_limits<double>::quiet_NaN(),
      0.7,
      0.25,
      3.0};

  decision = decideStableAnchorRecovery("CREATE_TEMP_SESSION", &anchor, &valid_alignment, invalid_gate);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ("CREATE_TEMP_SESSION", decision.action);
  EXPECT_EQ("invalid_alignment_gate", decision.reason);
}

TEST_F(SessionStoreTest, EstimatesStableAnchorAlignmentFromPointClouds) {
  const CloudT::Ptr stable_anchor_cloud = makeRecoveryCloud();

  Eigen::Matrix4f stable_to_current = Eigen::Matrix4f::Identity();
  stable_to_current(0, 3) = 0.12f;
  stable_to_current(1, 3) = -0.05f;
  stable_to_current(2, 3) = 0.02f;

  CloudT::Ptr current_cloud(new CloudT);
  pcl::transformPointCloud(*stable_anchor_cloud, *current_cloud, stable_to_current);

  RecoveryAlignmentOptions options;
  options.max_correspondence_distance_m = 0.5;
  options.inlier_threshold_m = 0.05;
  options.voxel_leaf_sizes = {0.25, 0.10, 0.0};

  const RecoveryAlignment alignment =
      estimateStableAnchorAlignment(current_cloud, stable_anchor_cloud, options);

  EXPECT_TRUE(alignment.converged);
  EXPECT_LT(alignment.rmse_m, 0.01);
  EXPECT_GT(alignment.inlier_ratio, 0.95);
  EXPECT_NEAR(-0.12, alignment.dx_m, 1e-3);
  EXPECT_NEAR(0.05, alignment.dy_m, 1e-3);
  EXPECT_NEAR(-0.02, alignment.dz_m, 1e-3);
}

TEST_F(SessionStoreTest, StableAnchorAlignmentRejectsNonFiniteRecoveryClouds) {
  const CloudT::Ptr stable_anchor_cloud = makeRecoveryCloud();

  Eigen::Matrix4f stable_to_current = Eigen::Matrix4f::Identity();
  stable_to_current(0, 3) = 0.12f;
  stable_to_current(1, 3) = -0.05f;
  stable_to_current(2, 3) = 0.02f;

  CloudT::Ptr current_cloud(new CloudT);
  pcl::transformPointCloud(*stable_anchor_cloud, *current_cloud, stable_to_current);

  PointT corrupt;
  corrupt.x = std::numeric_limits<float>::quiet_NaN();
  corrupt.y = 0.0f;
  corrupt.z = 0.0f;
  corrupt.intensity = 1.0f;
  current_cloud->push_back(corrupt);

  RecoveryAlignmentOptions options;
  options.max_correspondence_distance_m = 0.5;
  options.inlier_threshold_m = 0.05;
  options.voxel_leaf_sizes = {0.25, 0.10, 0.0};

  const RecoveryAlignment alignment =
      estimateStableAnchorAlignment(current_cloud, stable_anchor_cloud, options);

  EXPECT_FALSE(alignment.converged);
  EXPECT_EQ(999.0, alignment.rmse_m);
  EXPECT_EQ(0.0, alignment.inlier_ratio);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
