#include <gtest/gtest.h>

#include <sys/stat.h>
#include <unistd.h>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string readFile(const std::string& path) {
  std::ifstream input(path.c_str());
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

void writeFile(const std::string& path, const std::string& text) {
  std::ofstream output(path.c_str());
  output << text;
}

bool exists(const std::string& path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

std::string makeTempDir(const std::string& prefix) {
  std::string pattern = "/tmp/" + prefix + "_XXXXXX";
  std::vector<char> buffer(pattern.begin(), pattern.end());
  buffer.push_back('\0');
  char* result = mkdtemp(buffer.data());
  return result == nullptr ? std::string() : std::string(result);
}

std::string scriptPath() {
  return std::string(ACTUAL_BAG_REPLAY_SCRIPT_PATH);
}

}  // namespace

TEST(ActualBagReplayScript, DryRunKeepsVelocityReferenceOutOfPlayback) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_replay");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/evidence";
  writeFile(bag, "not a real bag");

  const std::string command = scriptPath() + " --bag " + bag + " --out " + out +
                              " --duration 5 --start 0 --skip-bag-inspect";
  ASSERT_EQ(0, std::system(command.c_str()));

  ASSERT_TRUE(exists(out + "/commands/play_selected_topics.sh"));
  ASSERT_TRUE(exists(out + "/reports/actual_bag_replay_plan.txt"));
  ASSERT_TRUE(exists(out + "/reports/initial_velocity_reference.txt"));
  ASSERT_TRUE(exists(out + "/reports/actual_bag_replay_metrics_report.txt"));
  ASSERT_TRUE(exists(out + "/reports/actual_bag_replay_events.txt"));
  ASSERT_TRUE(exists(out + "/commands/validate_actual_bag_events.sh"));

  const std::string play = readFile(out + "/commands/play_selected_topics.sh");
  EXPECT_NE(std::string::npos, play.find("--topics"));
  EXPECT_NE(std::string::npos, play.find("/velodyne_points"));
  EXPECT_NE(std::string::npos, play.find("/left/lslidar_point_cloud"));
  EXPECT_NE(std::string::npos, play.find("/right/velodyne_points"));
  EXPECT_NE(std::string::npos, play.find("/imu/data"));
  EXPECT_NE(std::string::npos, play.find("/time_reference"));
  EXPECT_EQ(std::string::npos, play.find("/novatel_data/inspvax"));

  const std::string replay = readFile(out + "/commands/run_replay.sh");
  EXPECT_NE(std::string::npos, replay.find("/points_raw"));
  EXPECT_NE(std::string::npos, replay.find("/lio/points_deskewed"));
  EXPECT_EQ(std::string::npos,
            replay.find("rostopic echo -n 5 /diagnostics/lidar_fusion"));
  EXPECT_EQ(std::string::npos,
            replay.find("rostopic echo -n 5 /diagnostics/lio_preprocess"));
  EXPECT_NE(std::string::npos,
            replay.find("rostopic echo /diagnostics/lidar_fusion"));
  EXPECT_NE(std::string::npos,
            replay.find("rostopic echo /diagnostics/lio_preprocess"));
  EXPECT_NE(std::string::npos,
            replay.find("rostopic echo /diagnostics/lio_local_odometry"));
  EXPECT_NE(std::string::npos, replay.find("points_raw_captured="));
  EXPECT_NE(std::string::npos, replay.find("deskewed_cloud_captured="));
  EXPECT_NE(std::string::npos, replay.find("fusion_published="));
  EXPECT_NE(std::string::npos,
            replay.find("lio_local_odometry_diag_captured="));
  EXPECT_NE(std::string::npos,
            replay.find("local_odometry_published="));
  EXPECT_NE(std::string::npos,
            replay.find("minimum_local_odometry_published="));
  EXPECT_NE(std::string::npos,
            replay.find("local_odometry_duration_coverage_status="));
  EXPECT_NE(std::string::npos,
            replay.find("\"$local_odometry_duration_coverage_status\" == \"PASS\""));
  EXPECT_NE(std::string::npos,
            replay.find("diagnostic_key_value_max"));
  EXPECT_NE(std::string::npos,
            replay.find("[^A-Za-z0-9_]"));
  EXPECT_NE(std::string::npos,
            replay.find("diagnostic_key_value_max published_odometry"));
  EXPECT_NE(std::string::npos,
            replay.find("local_odometry_rejected_registrations="));
  EXPECT_NE(std::string::npos,
            replay.find("diagnostic_key_value_max keyframe_reseeds"));
  EXPECT_NE(std::string::npos,
            replay.find("local_odometry_keyframe_reseeds="));
  EXPECT_NE(std::string::npos, replay.find("minimum_fusion_published="));
  EXPECT_NE(std::string::npos,
            replay.find("fusion_duration_coverage_status="));
  EXPECT_NE(std::string::npos,
            replay.find("\"$fusion_duration_coverage_status\" == \"PASS\""));
  EXPECT_NE(std::string::npos, replay.find("legacy_xyzi_clouds="));
  EXPECT_NE(std::string::npos, replay.find("pipeline_error_status="));
  EXPECT_NE(std::string::npos,
            replay.find("actual_bag_replay_metrics_report.txt"));
  EXPECT_NE(std::string::npos,
            replay.find("actual_bag_replay_events.txt"));
  EXPECT_NE(std::string::npos,
            replay.find("validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY"));
  EXPECT_NE(std::string::npos,
            replay.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos,
            replay.find("scenario=ACTUAL_TUNNEL_LIDAR_IMU_INITIAL"));

  const std::string validate_events =
      readFile(out + "/commands/validate_actual_bag_events.sh");
  EXPECT_NE(std::string::npos,
            validate_events.find("validation_report.launch"));
  EXPECT_NE(std::string::npos,
            validate_events.find("event_file:=" + out +
                                 "/reports/actual_bag_replay_events.txt"));
  EXPECT_NE(std::string::npos,
            validate_events.find("report_file:=" + out +
                                 "/reports/actual_bag_replay_hil_validation_report.txt"));
  EXPECT_EQ(std::string::npos,
            validate_events.find("evidence_manifest_file:="));

  const std::string launch_pipeline =
      readFile(out + "/commands/launch_pipeline.sh");
  EXPECT_NE(std::string::npos, launch_pipeline.find("session_root:="));
  EXPECT_NE(std::string::npos, launch_pipeline.find("session_state"));
  EXPECT_NE(std::string::npos,
            launch_pipeline.find("start_machine_state:=false"));
  EXPECT_NE(std::string::npos,
            launch_pipeline.find("start_mapping_control:=false"));
  EXPECT_NE(std::string::npos,
            launch_pipeline.find("start_section_manager:=false"));

  const std::string plan = readFile(out + "/reports/actual_bag_replay_plan.txt");
  EXPECT_NE(std::string::npos,
            plan.find("actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY"));
  EXPECT_NE(std::string::npos,
            plan.find("bag_sensor_set=LIDAR_IMU_ONLY"));
  EXPECT_NE(std::string::npos,
            plan.find("plc_feedback_status=NOT_PRESENT_NA"));
  EXPECT_NE(std::string::npos,
            plan.find("machine_motion_assumption=CONTINUOUS_MOTION"));
  EXPECT_NE(std::string::npos,
            plan.find("vibration_profile=NORMAL"));
  EXPECT_NE(std::string::npos,
            plan.find("field_acceptance_requires_plc_feedback=YES"));
  EXPECT_NE(std::string::npos,
            plan.find("initial_velocity_reference_policy=START_ONLY_AUDIT"));
  EXPECT_NE(std::string::npos,
            plan.find("continuous_velocity_reference_used=NO"));
  EXPECT_NE(std::string::npos,
            plan.find("velocity_reference_played_to_slam=NO"));
  EXPECT_NE(std::string::npos,
            plan.find("actual_bag_metrics_report=reports/actual_bag_replay_metrics_report.txt"));
  EXPECT_NE(std::string::npos,
            plan.find("actual_bag_event_file=reports/actual_bag_replay_events.txt"));
  EXPECT_NE(std::string::npos,
            plan.find("actual_bag_event_validation_command=" + out +
                      "/commands/validate_actual_bag_events.sh"));

  const std::string summary =
      readFile(out + "/reports/actual_bag_replay_summary.txt");
  EXPECT_NE(std::string::npos,
            summary.find("actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY"));
  EXPECT_NE(std::string::npos,
            summary.find("bag_sensor_set=LIDAR_IMU_ONLY"));
  EXPECT_NE(std::string::npos,
            summary.find("plc_feedback_status=NOT_PRESENT_NA"));
  EXPECT_NE(std::string::npos,
            summary.find("machine_motion_assumption=CONTINUOUS_MOTION"));
  EXPECT_NE(std::string::npos,
            summary.find("field_acceptance_requires_plc_feedback=YES"));
  EXPECT_NE(std::string::npos,
            summary.find("actual_bag_metrics_report=reports/actual_bag_replay_metrics_report.txt"));
  EXPECT_NE(std::string::npos,
            summary.find("actual_bag_event_file=reports/actual_bag_replay_events.txt"));

  const std::string inspection =
      readFile(out + "/reports/actual_bag_inspection.txt");
  EXPECT_NE(std::string::npos,
            inspection.find("plc_feedback_status=NOT_PRESENT_NA"));
  EXPECT_NE(std::string::npos,
            inspection.find("plc_feedback_gate_status=NA_INITIAL_TEST"));

  const std::string initial =
      readFile(out + "/reports/initial_velocity_reference.txt");
  EXPECT_NE(std::string::npos,
            initial.find("initial_velocity_reference_policy=START_ONLY_AUDIT"));
  EXPECT_NE(std::string::npos,
            initial.find("continuous_velocity_reference_used=NO"));
  EXPECT_NE(std::string::npos,
            initial.find("velocity_reference_played_to_slam=NO"));

  const std::string metrics =
      readFile(out + "/reports/actual_bag_replay_metrics_report.txt");
  EXPECT_NE(std::string::npos,
            metrics.find("overall=FAIL;total_records=1;failed_records=1"));
  EXPECT_NE(std::string::npos,
            metrics.find("validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY"));
  EXPECT_NE(std::string::npos,
            metrics.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos,
            metrics.find("status=DRY_RUN"));
  EXPECT_EQ(std::string::npos,
            metrics.find("overall=PASS;total_records=1;failed_records=0"));

  const std::string events =
      readFile(out + "/reports/actual_bag_replay_events.txt");
  EXPECT_NE(std::string::npos,
            events.find("event=session_start;scenario=ACTUAL_TUNNEL_LIDAR_IMU_INITIAL"));
  EXPECT_NE(std::string::npos,
            events.find("event=actual_bag_replay;scenario=ACTUAL_TUNNEL_LIDAR_IMU_INITIAL"));
  EXPECT_NE(std::string::npos,
            events.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos,
            events.find("plc_feedback_status=NOT_PRESENT_NA"));
  EXPECT_NE(std::string::npos,
            events.find("velocity_reference_played_to_slam=NO"));
  EXPECT_NE(std::string::npos,
            events.find("local_odometry_keyframe_reseeds=0"));
  EXPECT_NE(std::string::npos,
            events.find("queue_backlog=-1"));
}

TEST(ActualBagReplayScript, CustomInputTopicsAreRemappedToReplayProfile) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_topics");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/evidence";
  writeFile(bag, "not a real bag");

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out +
      " --duration 5 --skip-bag-inspect"
      " --center-topic /raw/center_points"
      " --left-topic /raw/left_points"
      " --right-topic /raw/right_points"
      " --imu-topic /sensors/imu_raw"
      " --time-reference-topic /clock/time_reference"
      " --initial-velocity-topic /nav/inspvax";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string play = readFile(out + "/commands/play_selected_topics.sh");
  EXPECT_NE(std::string::npos, play.find("/raw/center_points:=/velodyne_points"));
  EXPECT_NE(std::string::npos,
            play.find("/raw/left_points:=/left/lslidar_point_cloud"));
  EXPECT_NE(std::string::npos,
            play.find("/raw/right_points:=/right/velodyne_points"));
  EXPECT_NE(std::string::npos, play.find("/sensors/imu_raw:=/imu/data"));
  EXPECT_NE(std::string::npos,
            play.find("/clock/time_reference:=/time_reference"));
  EXPECT_NE(std::string::npos, play.find("/raw/center_points"));
  EXPECT_NE(std::string::npos, play.find("/raw/left_points"));
  EXPECT_NE(std::string::npos, play.find("/raw/right_points"));
  EXPECT_NE(std::string::npos, play.find("/sensors/imu_raw"));
  EXPECT_NE(std::string::npos, play.find("/clock/time_reference"));
  EXPECT_EQ(std::string::npos, play.find("/nav/inspvax"));

  const std::string plan = readFile(out + "/reports/actual_bag_replay_plan.txt");
  EXPECT_NE(std::string::npos, plan.find("center_lidar_topic=/raw/center_points"));
  EXPECT_NE(std::string::npos, plan.find("left_lidar_topic=/raw/left_points"));
  EXPECT_NE(std::string::npos, plan.find("right_lidar_topic=/raw/right_points"));
  EXPECT_NE(std::string::npos, plan.find("imu_topic=/sensors/imu_raw"));
  EXPECT_NE(std::string::npos,
            plan.find("time_reference_topic=/clock/time_reference"));
  EXPECT_NE(std::string::npos,
            plan.find("initial_velocity_reference_topic=/nav/inspvax"));
  EXPECT_NE(std::string::npos,
            plan.find("canonical_center_lidar_topic=/velodyne_points"));
  EXPECT_NE(std::string::npos,
            plan.find("play_topics=/raw/center_points,/raw/left_points,"
                      "/raw/right_points,/sensors/imu_raw,/clock/time_reference"));
  EXPECT_NE(std::string::npos,
            plan.find("excluded_velocity_reference_topic=/nav/inspvax"));

  const std::string inspection =
      readFile(out + "/reports/actual_bag_inspection.txt");
  EXPECT_NE(std::string::npos, inspection.find("center_lidar_topic=/raw/center_points"));
  EXPECT_NE(std::string::npos,
            inspection.find("canonical_center_lidar_topic=/velodyne_points"));
  EXPECT_NE(std::string::npos,
            inspection.find("initial_velocity_reference_topic=/nav/inspvax"));

  const std::string summary =
      readFile(out + "/reports/actual_bag_replay_summary.txt");
  EXPECT_NE(std::string::npos,
            summary.find("center_lidar_topic=/raw/center_points"));
  EXPECT_NE(std::string::npos,
            summary.find("canonical_center_lidar_topic=/velodyne_points"));
  EXPECT_NE(std::string::npos,
            summary.find("play_topics=/raw/center_points,/raw/left_points,"
                         "/raw/right_points,/sensors/imu_raw,"
                         "/clock/time_reference"));
  EXPECT_NE(std::string::npos,
            summary.find("excluded_velocity_reference_topic=/nav/inspvax"));
}

TEST(ActualBagReplayScript, NoTimeReferenceModeOmitsReplayTopicAndRecordsScope) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_no_time_ref");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/evidence";
  writeFile(bag, "not a real bag");

  const std::string command = scriptPath() + " --bag " + bag + " --out " + out +
                              " --duration 5 --skip-bag-inspect"
                              " --no-time-reference";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string play = readFile(out + "/commands/play_selected_topics.sh");
  EXPECT_NE(std::string::npos, play.find("--topics"));
  EXPECT_NE(std::string::npos, play.find("/velodyne_points"));
  EXPECT_NE(std::string::npos, play.find("/left/lslidar_point_cloud"));
  EXPECT_NE(std::string::npos, play.find("/right/velodyne_points"));
  EXPECT_NE(std::string::npos, play.find("/imu/data"));
  EXPECT_EQ(std::string::npos, play.find("/time_reference"));

  const std::string plan = readFile(out + "/reports/actual_bag_replay_plan.txt");
  EXPECT_NE(std::string::npos, plan.find("time_reference_topic=NONE"));
  EXPECT_NE(std::string::npos, plan.find("canonical_time_reference_topic=NONE"));
  EXPECT_NE(std::string::npos,
            plan.find("time_reference_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            plan.find("time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            plan.find("play_topics=/velodyne_points,/left/lslidar_point_cloud,"
                      "/right/velodyne_points,/imu/data"));

  const std::string inspection =
      readFile(out + "/reports/actual_bag_inspection.txt");
  EXPECT_NE(std::string::npos, inspection.find("time_reference_topic=NONE"));
  EXPECT_NE(std::string::npos,
            inspection.find("time_reference_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            inspection.find("time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST"));

  const std::string summary =
      readFile(out + "/reports/actual_bag_replay_summary.txt");
  EXPECT_NE(std::string::npos, summary.find("time_reference_topic=NONE"));
  EXPECT_NE(std::string::npos,
            summary.find("time_reference_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            summary.find("time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            summary.find("play_topics=/velodyne_points,/left/lslidar_point_cloud,"
                         "/right/velodyne_points,/imu/data"));
}

TEST(ActualBagReplayScript, NoInitialVelocityReferenceModeRecordsNotPresent) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_no_initial_velocity");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/evidence";
  writeFile(bag, "not a real bag");

  const std::string command = scriptPath() + " --bag " + bag + " --out " + out +
                              " --duration 5 --skip-bag-inspect"
                              " --no-initial-velocity-reference";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string play = readFile(out + "/commands/play_selected_topics.sh");
  EXPECT_EQ(std::string::npos, play.find("/novatel_data/inspvax"));

  const std::string plan = readFile(out + "/reports/actual_bag_replay_plan.txt");
  EXPECT_NE(std::string::npos,
            plan.find("initial_velocity_reference_topic=NONE"));
  EXPECT_NE(std::string::npos,
            plan.find("initial_velocity_reference_required=NO"));
  EXPECT_NE(std::string::npos,
            plan.find("initial_velocity_reference_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            plan.find("initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            plan.find("velocity_reference_played_to_slam=NO"));
  EXPECT_NE(std::string::npos,
            plan.find("continuous_velocity_reference_used=NO"));

  const std::string inspection =
      readFile(out + "/reports/actual_bag_inspection.txt");
  EXPECT_NE(std::string::npos,
            inspection.find("initial_velocity_reference_topic=NONE"));
  EXPECT_NE(std::string::npos,
            inspection.find("initial_velocity_reference_required=NO"));
  EXPECT_NE(std::string::npos,
            inspection.find("initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST"));

  const std::string initial =
      readFile(out + "/reports/initial_velocity_reference.txt");
  EXPECT_NE(std::string::npos,
            initial.find("initial_velocity_reference_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            initial.find("initial_velocity_reference_topic=NONE"));
  EXPECT_NE(std::string::npos,
            initial.find("initial_velocity_reference_required=NO"));
  EXPECT_NE(std::string::npos,
            initial.find("initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            initial.find("velocity_reference_played_to_slam=NO"));
  EXPECT_NE(std::string::npos,
            initial.find("continuous_velocity_reference_used=NO"));
}

TEST(ActualBagReplayScript, RejectsConflictingNoTimeReferenceOverride) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_conflict_time_ref");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  writeFile(bag, "not a real bag");

  const std::string command = scriptPath() + " --bag " + bag +
                              " --duration 5 --skip-bag-inspect"
                              " --no-time-reference"
                              " --time-reference-topic /clock/time_reference";
  EXPECT_NE(0, std::system(command.c_str()));
}

TEST(ActualBagReplayScript, RejectsMalformedInputTopicOverride) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_bad_topic");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  writeFile(bag, "not a real bag");

  const std::string command = scriptPath() + " --bag " + bag +
                              " --duration 5 --skip-bag-inspect"
                              " --center-topic center_points";
  EXPECT_NE(0, std::system(command.c_str()));
}

TEST(ActualBagReplayScript, RejectsInvalidReplayDuration) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_bad_duration");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  writeFile(bag, "not a real bag");

  const std::string command = scriptPath() + " --bag " + bag +
                              " --duration 0 --skip-bag-inspect";
  EXPECT_NE(0, std::system(command.c_str()));
}

TEST(ActualBagReplayScript, CaptureTimeoutScalesWithReplayRate) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_rate_timeout");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/evidence";
  writeFile(bag, "not a real bag");

  const std::string command = scriptPath() + " --bag " + bag + " --out " + out +
                              " --duration 5 --rate 0.5 --skip-bag-inspect";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string replay = readFile(out + "/commands/run_replay.sh");
  EXPECT_NE(std::string::npos,
            replay.find("capture_timeout_s=40"));
  EXPECT_NE(std::string::npos,
            replay.find("timeout $capture_timeout_s rostopic echo /diagnostics/lidar_fusion"));
}

TEST(ActualBagReplayScript, CanOverrideLocalOdometryConfigForReplayTuning) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_local_config");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string config = root + "/local_icp_tuning.yaml";
  const std::string out = root + "/evidence";
  writeFile(bag, "not a real bag");
  writeFile(config, "voxel_leaf_size: 0.10\n");

  const std::string command = scriptPath() + " --bag " + bag + " --out " + out +
                              " --duration 5 --local-odometry-config " +
                              config + " --skip-bag-inspect";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string launch_pipeline =
      readFile(out + "/commands/launch_pipeline.sh");
  EXPECT_NE(std::string::npos,
            launch_pipeline.find("local_odometry_config:=" + config));

  const std::string plan = readFile(out + "/reports/actual_bag_replay_plan.txt");
  EXPECT_NE(std::string::npos,
            plan.find("local_odometry_config=" + config));
}

TEST(ActualBagReplayScript, GeneratedReplayCleansPipelineProcessGroup) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_cleanup");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/evidence";
  writeFile(bag, "not a real bag");

  const std::string command = scriptPath() + " --bag " + bag + " --out " + out +
                              " --duration 5 --skip-bag-inspect";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string replay = readFile(out + "/commands/run_replay.sh");
  EXPECT_NE(std::string::npos,
            replay.find("setsid \"$commands_dir/launch_pipeline.sh\""));
  EXPECT_NE(std::string::npos, replay.find("pipeline_pgid=\"$pipeline_pid\""));
  EXPECT_NE(std::string::npos, replay.find("kill -- \"-$pipeline_pgid\""));
}

TEST(ActualBagReplayScript, GeneratedReplayUsesOwnedRosMaster) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_rosmaster");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/evidence";
  writeFile(bag, "not a real bag");

  const std::string command = scriptPath() + " --bag " + bag + " --out " + out +
                              " --duration 5 --skip-bag-inspect";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string replay = readFile(out + "/commands/run_replay.sh");
  EXPECT_NE(std::string::npos, replay.find("ros_master_port="));
  EXPECT_NE(std::string::npos,
            replay.find("export ROS_MASTER_URI=\"http://127.0.0.1:$ros_master_port\""));
  EXPECT_NE(std::string::npos,
            replay.find("setsid roscore -p \"$ros_master_port\""));
  EXPECT_NE(std::string::npos, replay.find("kill -- \"-$roscore_pgid\""));
  EXPECT_NE(std::string::npos, replay.find("ros_master_cleanup_status="));
  EXPECT_NE(std::string::npos,
            replay.find("\"$ros_master_cleanup_status\" == \"PASS\""));
  EXPECT_NE(std::string::npos, replay.find("ros_master_uri=$ROS_MASTER_URI"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
