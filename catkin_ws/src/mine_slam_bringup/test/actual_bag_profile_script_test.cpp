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
  return std::string(ACTUAL_BAG_PROFILE_SCRIPT_PATH);
}

void createSyntheticBag(const std::string& bag_path,
                        bool include_time_reference,
                        bool include_velocity_reference = false,
                        bool include_plc_feedback = false) {
  const std::string script_path = bag_path + ".py";
  std::ostringstream script;
  script
      << "import rosbag\n"
      << "import rospy\n"
      << "from geometry_msgs.msg import TwistStamped\n"
      << "from sensor_msgs.msg import Imu, PointCloud2, PointField, TimeReference\n"
      << "from std_msgs.msg import Bool, Float64, String\n"
      << "\n"
      << "def cloud(frame, full):\n"
      << "    msg = PointCloud2()\n"
      << "    msg.header.stamp = rospy.Time.from_sec(1.0)\n"
      << "    msg.header.frame_id = frame\n"
      << "    names = ['x', 'y', 'z', 'intensity']\n"
      << "    if full:\n"
      << "        names += ['ring', 'time']\n"
      << "    msg.fields = []\n"
      << "    for i, name in enumerate(names):\n"
      << "        field = PointField()\n"
      << "        field.name = name\n"
      << "        field.offset = i * 4\n"
      << "        field.datatype = PointField.FLOAT32\n"
      << "        field.count = 1\n"
      << "        msg.fields.append(field)\n"
      << "    msg.height = 1\n"
      << "    msg.width = 1\n"
      << "    msg.is_bigendian = False\n"
      << "    msg.point_step = len(names) * 4\n"
      << "    msg.row_step = msg.point_step\n"
      << "    msg.data = bytes(msg.point_step)\n"
      << "    msg.is_dense = True\n"
      << "    return msg\n"
      << "\n"
      << "bag = rosbag.Bag('" << bag_path << "', 'w')\n"
      << "try:\n"
      << "    stamp = rospy.Time.from_sec(1.0)\n"
      << "    bag.write('/raw/center_points', cloud('center', True), stamp)\n"
      << "    bag.write('/raw/left_points', cloud('left', False), stamp)\n"
      << "    bag.write('/raw/right_points', cloud('right', True), stamp)\n"
      << "    imu = Imu()\n"
      << "    imu.header.stamp = stamp\n"
      << "    imu.header.frame_id = 'imu'\n"
      << "    bag.write('/sensors/imu_raw', imu, stamp)\n";
  if (include_time_reference) {
    script
        << "    time_ref = TimeReference()\n"
        << "    time_ref.header.stamp = stamp\n"
        << "    time_ref.header.frame_id = 'time_ref'\n"
        << "    time_ref.time_ref = stamp\n"
        << "    bag.write('/clock/time_reference', time_ref, stamp)\n";
  }
  if (include_velocity_reference) {
    script
        << "    velocity = TwistStamped()\n"
        << "    velocity.header.stamp = stamp\n"
        << "    velocity.twist.linear.x = 3.0\n"
        << "    velocity.twist.linear.y = 4.0\n"
        << "    velocity.twist.linear.z = 12.0\n"
        << "    bag.write('/nav/velocity', velocity, stamp)\n";
  }
  if (include_plc_feedback) {
    script
        << "    left_track = Float64()\n"
        << "    left_track.data = 1.2\n"
        << "    bag.write('/plc/left_track_speed', left_track, stamp)\n"
        << "    right_track = Float64()\n"
        << "    right_track.data = 1.1\n"
        << "    bag.write('/plc/right_track_speed', right_track, stamp)\n"
        << "    cutting = Bool()\n"
        << "    cutting.data = False\n"
        << "    bag.write('/plc/cutting_on', cutting, stamp)\n"
        << "    machine_state = String()\n"
        << "    machine_state.data = 'moving'\n"
        << "    bag.write('/machine/state', machine_state, stamp)\n";
  }
  script << "finally:\n"
         << "    bag.close()\n";
  writeFile(script_path, script.str());
  ASSERT_EQ(0, std::system(("python3 " + script_path).c_str()));
}

TEST(ActualBagProfileScript,
     AllowsPlcFeedbackProfileOnlyWithoutFieldAcceptanceEligibility) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_plc_profile_only");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  EXPECT_NE(std::string::npos, profile.find("actual_bag_profile_status=PASS"));
  EXPECT_NE(std::string::npos, profile.find("plc_feedback_topic_count=4"));
  EXPECT_NE(std::string::npos,
            profile.find("plc_feedback_status=PRESENT_PROFILE_ONLY"));
  EXPECT_NE(std::string::npos,
            profile.find("plc_feedback_gate_status="
                         "PRESENT_PROFILE_ONLY_FIELD_VALIDATION_REQUIRED"));
  EXPECT_NE(std::string::npos, profile.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos,
            profile.find("field_acceptance_requires_plc_feedback=YES"));
  EXPECT_NE(std::string::npos,
            profile.find("machine_motion_assumption=CONTINUOUS_MOTION"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_reference_policy=START_ONLY_AUDIT"));
  EXPECT_NE(std::string::npos,
            profile.find("velocity_reference_played_to_slam=NO"));
  EXPECT_NE(std::string::npos,
            profile.find("continuous_velocity_reference_used=NO"));

  ASSERT_EQ(0, std::system((out + "/commands/validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=PASS"));
  EXPECT_NE(std::string::npos,
            validation.find("plc_feedback_topic_count_status=PASS"));
  EXPECT_NE(std::string::npos,
            validation.find("plc_feedback_status_status=PASS"));
  EXPECT_NE(std::string::npos,
            validation.find("plc_feedback_gate_status_status=PASS"));
}

}  // namespace

TEST(ActualBagProfileScript, SuggestsNoTimeReferenceSuiteCommand) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_profile");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  ASSERT_TRUE(exists(out + "/reports/actual_bag_profile.txt"));
  ASSERT_TRUE(exists(out + "/commands/run_recommended_actual_bag_test_suite.sh"));
  ASSERT_TRUE(exists(out + "/commands/validate_actual_bag_profile.sh"));

  const std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  EXPECT_NE(std::string::npos,
            profile.find("actual_bag_profile_status=PASS"));
  EXPECT_NE(std::string::npos, profile.find("center_lidar_topic=/raw/center_points"));
  EXPECT_NE(std::string::npos, profile.find("left_lidar_topic=/raw/left_points"));
  EXPECT_NE(std::string::npos, profile.find("right_lidar_topic=/raw/right_points"));
  EXPECT_NE(std::string::npos, profile.find("imu_topic=/sensors/imu_raw"));
  EXPECT_NE(std::string::npos,
            profile.find("time_reference_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            profile.find("recommended_time_reference_arg=--no-time-reference"));
  EXPECT_NE(std::string::npos, profile.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos, profile.find("plc_feedback_topic_count=0"));
  EXPECT_NE(std::string::npos, profile.find("plc_feedback_status=NOT_PRESENT_NA"));
  EXPECT_NE(std::string::npos,
            profile.find("plc_feedback_gate_status=NA_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            profile.find("machine_motion_assumption=CONTINUOUS_MOTION"));
  EXPECT_NE(std::string::npos, profile.find("vibration_profile=NORMAL"));
  EXPECT_NE(std::string::npos,
            profile.find("field_acceptance_requires_plc_feedback=YES"));
  EXPECT_NE(std::string::npos,
            profile.find("recommended_suite_verified_execute_required=YES"));
  EXPECT_NE(std::string::npos,
            profile.find("recommended_suite_initial_readiness_required=YES"));
  EXPECT_NE(std::string::npos,
            profile.find(
                "recommended_suite_field_acceptance_handoff_required=YES"));
  EXPECT_NE(
      std::string::npos,
      profile.find(
          "recommended_suite_field_acceptance_handoff_manifest_required=YES"));
  EXPECT_NE(std::string::npos,
            profile.find(
                "recommended_suite_field_acceptance_collection_plan_required=YES"));
  EXPECT_NE(
      std::string::npos,
      profile.find(
          "recommended_suite_collection_plan_manifest_revalidation_required=YES"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_reference_status=CAPTURED"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_reference_policy=START_ONLY_AUDIT"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_reference_topic=/nav/velocity"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_first_sample_stamp_s=1.000000"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_north_mps=3.000000"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_east_mps=4.000000"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_up_mps=12.000000"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_speed_mps=13.000000"));

  const std::string suite =
      readFile(out + "/commands/run_recommended_actual_bag_test_suite.sh");
  const std::size_t validator_position =
      suite.find("validate_actual_bag_profile.sh");
  const std::size_t suite_position = suite.find("actual_bag_test_suite.sh");
  const std::size_t readiness_validator_position =
      suite.find("validate_actual_bag_initial_test_readiness.sh");
  const std::size_t handoff_validator_position =
      suite.find("validate_field_acceptance_handoff.sh");
  const std::size_t handoff_manifest_validator_position =
      suite.find("validate_field_acceptance_handoff_manifest.sh");
  const std::size_t collection_plan_validator_position =
      suite.find("validate_field_acceptance_collection_plan.sh");
  const std::size_t collection_plan_manifest_validator_position =
      suite.find("validate_actual_bag_test_suite.sh",
                 collection_plan_validator_position);
  EXPECT_NE(std::string::npos, validator_position);
  EXPECT_NE(std::string::npos, suite_position);
  EXPECT_NE(std::string::npos, readiness_validator_position);
  EXPECT_NE(std::string::npos, handoff_validator_position);
  EXPECT_NE(std::string::npos, handoff_manifest_validator_position);
  EXPECT_NE(std::string::npos, collection_plan_validator_position);
  EXPECT_NE(std::string::npos, collection_plan_manifest_validator_position);
  EXPECT_LT(validator_position, suite_position);
  EXPECT_LT(suite_position, readiness_validator_position);
  EXPECT_LT(readiness_validator_position, handoff_validator_position);
  EXPECT_LT(handoff_validator_position, handoff_manifest_validator_position);
  EXPECT_LT(handoff_manifest_validator_position, collection_plan_validator_position);
  EXPECT_LT(collection_plan_validator_position,
            collection_plan_manifest_validator_position);
  EXPECT_NE(std::string::npos, suite.find("--center-topic /raw/center_points"));
  EXPECT_NE(std::string::npos, suite.find("--left-topic /raw/left_points"));
  EXPECT_NE(std::string::npos, suite.find("--right-topic /raw/right_points"));
  EXPECT_NE(std::string::npos, suite.find("--imu-topic /sensors/imu_raw"));
  EXPECT_NE(std::string::npos, suite.find("--no-time-reference"));
  EXPECT_NE(std::string::npos, suite.find("--initial-velocity-topic /nav/velocity"));
  EXPECT_EQ(std::string::npos, suite.find("--time-reference-topic"));

  ASSERT_EQ(0, std::system((out + "/commands/validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=PASS"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_verified_execute_required_status=PASS"));
  EXPECT_NE(std::string::npos,
            validation.find(
                "recommended_suite_initial_readiness_required_status=PASS"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_field_acceptance_handoff_required_status=PASS"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_field_acceptance_handoff_manifest_required_status=PASS"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_field_acceptance_collection_plan_required_status=PASS"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_collection_plan_manifest_revalidation_required_status=PASS"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_initial_readiness_entry_status=PASS"));
  EXPECT_NE(std::string::npos,
            validation.find(
                "recommended_suite_field_acceptance_handoff_entry_status=PASS"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_field_acceptance_collection_plan_entry_status=PASS"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS"));
  EXPECT_NE(std::string::npos,
            validation.find("velocity_reference_played_to_slam=NO"));
  EXPECT_NE(std::string::npos,
            validation.find("continuous_velocity_reference_used=NO"));
}

TEST(ActualBagProfileScript, SuggestsNoInitialVelocityReferenceWhenMissing) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_no_velocity");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, true, false);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_reference_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_reference_required=NO"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            profile.find("initial_velocity_reference_topic=NONE"));
  EXPECT_NE(std::string::npos,
            profile.find("velocity_reference_played_to_slam=NO"));
  EXPECT_NE(std::string::npos,
            profile.find("continuous_velocity_reference_used=NO"));

  const std::string suite =
      readFile(out + "/commands/run_recommended_actual_bag_test_suite.sh");
  EXPECT_NE(std::string::npos,
            suite.find("--no-initial-velocity-reference"));
  EXPECT_EQ(std::string::npos, suite.find("--initial-velocity-topic"));

  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=PASS"));
  EXPECT_NE(std::string::npos,
            validation.find("initial_velocity_reference_status_status=PASS"));
}

TEST(ActualBagProfileScript, ValidatorRejectsContinuousVelocityUse) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_profile_validate");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  const std::string expected = "continuous_velocity_reference_used=NO";
  const std::size_t position = profile.find(expected);
  ASSERT_NE(std::string::npos, position);
  profile.replace(position, expected.size(), "continuous_velocity_reference_used=YES");
  writeFile(out + "/reports/actual_bag_profile.txt", profile);

  EXPECT_NE(0, std::system((out + "/commands/validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("continuous_velocity_reference_used_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsDisabledVerifiedExecuteRequirement) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_verified_execute");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  const std::string expected = "recommended_suite_verified_execute_required=YES";
  const std::size_t position = profile.find(expected);
  ASSERT_NE(std::string::npos, position);
  profile.replace(position, expected.size(),
                  "recommended_suite_verified_execute_required=NO");
  writeFile(out + "/reports/actual_bag_profile.txt", profile);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find(
                "recommended_suite_verified_execute_required_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsDisabledInitialReadinessRequirement) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_initial_readiness");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  const std::string expected = "recommended_suite_initial_readiness_required=YES";
  const std::size_t position = profile.find(expected);
  ASSERT_NE(std::string::npos, position);
  profile.replace(position, expected.size(),
                  "recommended_suite_initial_readiness_required=NO");
  writeFile(out + "/reports/actual_bag_profile.txt", profile);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find(
                "recommended_suite_initial_readiness_required_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsDisabledFieldAcceptanceHandoffRequirement) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_field_handoff");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  const std::string expected =
      "recommended_suite_field_acceptance_handoff_required=YES";
  const std::size_t position = profile.find(expected);
  ASSERT_NE(std::string::npos, position);
  profile.replace(position, expected.size(),
                  "recommended_suite_field_acceptance_handoff_required=NO");
  writeFile(out + "/reports/actual_bag_profile.txt", profile);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_field_acceptance_handoff_required_status=FAIL"));
}

TEST(ActualBagProfileScript,
     ValidatorRejectsDisabledFieldAcceptanceHandoffManifestRequirement) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_handoff_manifest");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  const std::string expected =
      "recommended_suite_field_acceptance_handoff_manifest_required=YES";
  const std::size_t position = profile.find(expected);
  ASSERT_NE(std::string::npos, position);
  profile.replace(
      position, expected.size(),
      "recommended_suite_field_acceptance_handoff_manifest_required=NO");
  writeFile(out + "/reports/actual_bag_profile.txt", profile);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_field_acceptance_handoff_manifest_required_status=FAIL"));
}

TEST(ActualBagProfileScript,
     ValidatorRejectsDisabledFieldAcceptanceCollectionPlanRequirement) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_collection_plan");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  const std::string expected =
      "recommended_suite_field_acceptance_collection_plan_required=YES";
  const std::size_t position = profile.find(expected);
  ASSERT_NE(std::string::npos, position);
  profile.replace(
      position, expected.size(),
      "recommended_suite_field_acceptance_collection_plan_required=NO");
  writeFile(out + "/reports/actual_bag_profile.txt", profile);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_field_acceptance_collection_plan_required_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsUngatedRecommendedSuiteEntry) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_profile_entry_gate");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string validator_call =
      "\"$script_dir/validate_actual_bag_profile.sh\"\n";
  const std::size_t validator_position = entry.find(validator_call);
  ASSERT_NE(std::string::npos, validator_position);
  entry.erase(validator_position, validator_call.size());
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_entry_gate_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsRecommendedSuiteEntryWithoutExecute) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_execute");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string execute_arg = " --execute";
  const std::size_t execute_position = entry.find(execute_arg);
  ASSERT_NE(std::string::npos, execute_position);
  entry.erase(execute_position, execute_arg.size());
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_entry_gate_status=FAIL"));
}

TEST(ActualBagProfileScript,
     ValidatorRejectsRecommendedSuiteEntryEarlyTerminationBeforeSuite) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_early_exit");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  const std::string command_key = "recommended_suite_command=";
  const std::size_t command_position = profile.find(command_key);
  ASSERT_NE(std::string::npos, command_position);
  const std::size_t command_value_start = command_position + command_key.size();
  const std::size_t command_value_end =
      profile.find('\n', command_value_start);
  ASSERT_NE(std::string::npos, command_value_end);
  const std::string expected_suite_command =
      profile.substr(command_value_start,
                     command_value_end - command_value_start);

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::size_t suite_command_position =
      entry.find(expected_suite_command);
  ASSERT_NE(std::string::npos, suite_command_position);
  entry.insert(suite_command_position, "exit 0\n");
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_entry_gate_status=FAIL"));
}

TEST(ActualBagProfileScript,
     ValidatorRejectsRecommendedSuiteEntryWrappedEarlyTerminationBeforeSuite) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_wrapped_early_exit");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  const std::string command_key = "recommended_suite_command=";
  const std::size_t command_position = profile.find(command_key);
  ASSERT_NE(std::string::npos, command_position);
  const std::size_t command_value_start = command_position + command_key.size();
  const std::size_t command_value_end =
      profile.find('\n', command_value_start);
  ASSERT_NE(std::string::npos, command_value_end);
  const std::string expected_suite_command =
      profile.substr(command_value_start,
                     command_value_end - command_value_start);

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::size_t suite_command_position =
      entry.find(expected_suite_command);
  ASSERT_NE(std::string::npos, suite_command_position);
  entry.insert(suite_command_position, "if true; then exit 0; fi\n");
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_entry_gate_status=FAIL"));
}

TEST(ActualBagProfileScript,
     ValidatorRejectsRecommendedSuiteEntryStrictModeRelaxationBeforeSuite) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_set_plus_e");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string validator_call =
      "\"$script_dir/validate_actual_bag_profile.sh\"\n";
  const std::size_t validator_position = entry.find(validator_call);
  ASSERT_NE(std::string::npos, validator_position);
  entry.insert(validator_position + validator_call.size(), "set +e\n");
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_entry_gate_status=FAIL"));
}

TEST(ActualBagProfileScript,
     ValidatorRejectsRecommendedSuiteEntryWrappedStrictModeRelaxationBeforeSuite) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_wrapped_set_plus_e");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string validator_call =
      "\"$script_dir/validate_actual_bag_profile.sh\"\n";
  const std::size_t validator_position = entry.find(validator_call);
  ASSERT_NE(std::string::npos, validator_position);
  entry.insert(validator_position + validator_call.size(),
               "if true; then set +e; fi\n");
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_entry_gate_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsRecommendedSuiteEntryWithoutReadinessValidation) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_readiness");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string readiness_validator_call =
      "\"$suite_out/commands/validate_actual_bag_initial_test_readiness.sh\"\n";
  const std::size_t readiness_position = entry.find(readiness_validator_call);
  ASSERT_NE(std::string::npos, readiness_position);
  entry.erase(readiness_position, readiness_validator_call.size());
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_initial_readiness_entry_status=FAIL"));
}

TEST(ActualBagProfileScript,
     ValidatorRejectsReadinessBeforeExactRecommendedSuiteCommand) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_readiness_before_suite");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  const std::string command_key = "recommended_suite_command=";
  const std::size_t command_position = profile.find(command_key);
  ASSERT_NE(std::string::npos, command_position);
  const std::size_t command_value_start = command_position + command_key.size();
  const std::size_t command_value_end =
      profile.find('\n', command_value_start);
  ASSERT_NE(std::string::npos, command_value_end);
  const std::string expected_suite_command =
      profile.substr(command_value_start,
                     command_value_end - command_value_start);

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string readiness_validator_call =
      "\"$suite_out/commands/validate_actual_bag_initial_test_readiness.sh\"\n";
  const std::size_t original_readiness_position =
      entry.find(readiness_validator_call);
  ASSERT_NE(std::string::npos, original_readiness_position);
  entry.erase(original_readiness_position, readiness_validator_call.size());
  const std::size_t suite_command_position =
      entry.find(expected_suite_command);
  ASSERT_NE(std::string::npos, suite_command_position);
  entry.insert(suite_command_position,
               "echo actual_bag_test_suite.sh\n" + readiness_validator_call);
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_initial_readiness_entry_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsRecommendedSuiteEntryWithoutFieldAcceptanceHandoffValidation) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_handoff");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string handoff_validator_call =
      "\"$suite_out/commands/validate_field_acceptance_handoff.sh\"\n";
  const std::size_t handoff_position = entry.find(handoff_validator_call);
  ASSERT_NE(std::string::npos, handoff_position);
  entry.erase(handoff_position, handoff_validator_call.size());
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(
      std::string::npos,
      validation.find("recommended_suite_field_acceptance_handoff_entry_status=FAIL"));
}

TEST(ActualBagProfileScript,
     ValidatorRejectsRecommendedSuiteEntryWithoutFieldAcceptanceHandoffManifestValidation) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_handoff_manifest");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string handoff_manifest_validator_call =
      "\"$suite_out/commands/validate_field_acceptance_handoff_manifest.sh\"\n";
  const std::size_t handoff_manifest_position =
      entry.find(handoff_manifest_validator_call);
  ASSERT_NE(std::string::npos, handoff_manifest_position);
  entry.erase(handoff_manifest_position, handoff_manifest_validator_call.size());
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_field_acceptance_handoff_manifest_entry_status=FAIL"));
}

TEST(ActualBagProfileScript,
     ValidatorRejectsRecommendedSuiteEntryWithoutFieldAcceptanceCollectionPlanValidation) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_collection_plan");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string collection_plan_validator_call =
      "\"$suite_out/commands/validate_field_acceptance_collection_plan.sh\"\n";
  const std::size_t collection_plan_position =
      entry.find(collection_plan_validator_call);
  ASSERT_NE(std::string::npos, collection_plan_position);
  entry.erase(collection_plan_position, collection_plan_validator_call.size());
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_field_acceptance_collection_plan_entry_status=FAIL"));
}

TEST(ActualBagProfileScript,
     ValidatorRejectsRecommendedSuiteEntryWithoutPostCollectionPlanManifestValidation) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_collection_manifest");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string collection_plan_validator_call =
      "\"$suite_out/commands/validate_field_acceptance_collection_plan.sh\"\n";
  const std::size_t collection_plan_position =
      entry.find(collection_plan_validator_call);
  ASSERT_NE(std::string::npos, collection_plan_position);
  const std::string suite_manifest_validator_call =
      "\"$suite_out/commands/validate_actual_bag_test_suite.sh\"\n";
  const std::size_t suite_manifest_position =
      entry.find(suite_manifest_validator_call,
                 collection_plan_position + collection_plan_validator_call.size());
  ASSERT_NE(std::string::npos, suite_manifest_position);
  entry.erase(suite_manifest_position, suite_manifest_validator_call.size());
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_collection_plan_manifest_revalidation_entry_status=FAIL"));
}

TEST(ActualBagProfileScript,
     ValidatorRejectsCommentedPostCollectionPlanManifestValidation) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_comment_manifest");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string collection_plan_validator_call =
      "\"$suite_out/commands/validate_field_acceptance_collection_plan.sh\"\n";
  const std::size_t collection_plan_position =
      entry.find(collection_plan_validator_call);
  ASSERT_NE(std::string::npos, collection_plan_position);
  const std::string suite_manifest_validator_call =
      "\"$suite_out/commands/validate_actual_bag_test_suite.sh\"\n";
  const std::size_t suite_manifest_position =
      entry.find(suite_manifest_validator_call,
                 collection_plan_position + collection_plan_validator_call.size());
  ASSERT_NE(std::string::npos, suite_manifest_position);
  entry.replace(suite_manifest_position, suite_manifest_validator_call.size(),
                "# " + suite_manifest_validator_call);
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(
      std::string::npos,
      validation.find(
          "recommended_suite_collection_plan_manifest_revalidation_entry_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsRecommendedSuiteEntrySuiteOutMismatch) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_suite_out");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string expected_suite_out = "suite_out=" + out + "/recommended_suite";
  const std::size_t suite_out_position = entry.find(expected_suite_out);
  ASSERT_NE(std::string::npos, suite_out_position);
  entry.replace(suite_out_position, expected_suite_out.size(),
                "suite_out=" + root + "/other_suite");
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_entry_gate_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsRecommendedSuiteEntrySuiteOutReassignment) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_suite_out_reassign");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string expected_suite_out_line =
      "suite_out=" + out + "/recommended_suite\n";
  const std::size_t suite_out_position = entry.find(expected_suite_out_line);
  ASSERT_NE(std::string::npos, suite_out_position);
  entry.insert(suite_out_position + expected_suite_out_line.size(),
               "suite_out=" + root + "/other_suite\n");
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_entry_gate_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsEchoedRecommendedSuiteCommand) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_echo_suite");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  const std::string command_key = "recommended_suite_command=";
  const std::size_t command_position = profile.find(command_key);
  ASSERT_NE(std::string::npos, command_position);
  const std::size_t command_value_start = command_position + command_key.size();
  const std::size_t command_value_end =
      profile.find('\n', command_value_start);
  ASSERT_NE(std::string::npos, command_value_end);
  const std::string expected_suite_command =
      profile.substr(command_value_start,
                     command_value_end - command_value_start);

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::size_t suite_command_position =
      entry.find(expected_suite_command);
  ASSERT_NE(std::string::npos, suite_command_position);
  entry.replace(suite_command_position, expected_suite_command.size(),
                "echo " + expected_suite_command);
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_entry_gate_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsRecommendedSuiteOutOutsideProfileRoot) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_suite_out_anchor");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string expected_suite_out = out + "/recommended_suite";
  const std::string tampered_suite_out = root + "/other_suite";

  std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  std::size_t profile_position = profile.find(expected_suite_out);
  ASSERT_NE(std::string::npos, profile_position);
  while (profile_position != std::string::npos) {
    profile.replace(profile_position, expected_suite_out.size(),
                    tampered_suite_out);
    profile_position = profile.find(expected_suite_out,
                                    profile_position + tampered_suite_out.size());
  }
  writeFile(out + "/reports/actual_bag_profile.txt", profile);

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  std::size_t entry_position = entry.find(expected_suite_out);
  ASSERT_NE(std::string::npos, entry_position);
  while (entry_position != std::string::npos) {
    entry.replace(entry_position, expected_suite_out.size(),
                  tampered_suite_out);
    entry_position = entry.find(expected_suite_out,
                                entry_position + tampered_suite_out.size());
  }
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_out_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsRecommendedSuiteEntryCommandMismatch) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_entry_mismatch");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::string rate_arg = "--rate 1.0";
  const std::size_t rate_position = entry.find(rate_arg);
  ASSERT_NE(std::string::npos, rate_position);
  entry.replace(rate_position, rate_arg.size(), "--rate 0.5");
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_entry_gate_status=FAIL"));
}

TEST(ActualBagProfileScript, ValidatorRejectsRecommendedCommandMissingInitialVelocityAudit) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_profile_missing_velocity_audit");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, false, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string velocity_arg = " --initial-velocity-topic /nav/velocity";
  std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  const std::size_t profile_position = profile.find(velocity_arg);
  ASSERT_NE(std::string::npos, profile_position);
  profile.erase(profile_position, velocity_arg.size());
  writeFile(out + "/reports/actual_bag_profile.txt", profile);

  const std::string entry_path =
      out + "/commands/run_recommended_actual_bag_test_suite.sh";
  std::string entry = readFile(entry_path);
  const std::size_t entry_position = entry.find(velocity_arg);
  ASSERT_NE(std::string::npos, entry_position);
  entry.erase(entry_position, velocity_arg.size());
  writeFile(entry_path, entry);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_profile.sh").c_str()));
  const std::string validation =
      readFile(out + "/reports/actual_bag_profile_validation.txt");
  EXPECT_NE(std::string::npos,
            validation.find("actual_bag_profile_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            validation.find("recommended_suite_command_status=FAIL"));
}

TEST(ActualBagProfileScript, SuggestsTimeReferenceTopicWhenPresent) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_profile_time");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/user.bag";
  const std::string out = root + "/profile";
  createSyntheticBag(bag, true);

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out;
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string profile = readFile(out + "/reports/actual_bag_profile.txt");
  EXPECT_NE(std::string::npos,
            profile.find("time_reference_status=PRESENT_REQUIRED"));
  EXPECT_NE(std::string::npos,
            profile.find("time_reference_topic=/clock/time_reference"));
  EXPECT_NE(std::string::npos,
            profile.find("recommended_time_reference_arg=--time-reference-topic /clock/time_reference"));

  const std::string suite =
      readFile(out + "/commands/run_recommended_actual_bag_test_suite.sh");
  EXPECT_NE(std::string::npos,
            suite.find("--time-reference-topic /clock/time_reference"));
  EXPECT_EQ(std::string::npos, suite.find("--no-time-reference"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
