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

void makeDir(const std::string& path) {
  ASSERT_EQ(0, std::system(("mkdir -p " + path).c_str()));
}

void makeExecutable(const std::string& path) {
  ASSERT_EQ(0, std::system(("chmod +x " + path).c_str()));
}

std::string makeTempDir(const std::string& prefix) {
  std::string pattern = "/tmp/" + prefix + "_XXXXXX";
  std::vector<char> buffer(pattern.begin(), pattern.end());
  buffer.push_back('\0');
  char* result = mkdtemp(buffer.data());
  return result == nullptr ? std::string() : std::string(result);
}

std::string scriptPath() {
  return std::string(ACTUAL_BAG_TEST_SUITE_SCRIPT_PATH);
}

void writeReplayStub(const std::string& script_path,
                     const std::string& replay_dir) {
  writeFile(script_path,
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            "mkdir -p \"" +
                replay_dir +
                "/reports\" \"" + replay_dir +
                "/commands\"\n"
                "cat > \"" +
                replay_dir +
                "/reports/actual_bag_replay_summary.txt\" <<'EOF'\n"
                "actual_bag_replay_status=PASS\n"
                "EOF\n"
                "cat > \"" +
                replay_dir +
                "/reports/actual_bag_replay_hil_validation_report.txt\" <<'EOF'\n"
                "overall=PASS;total_records=1;failed_records=0\n"
                "EOF\n"
                "cat > \"" +
                replay_dir +
                "/commands/validate_actual_bag_events.sh\" <<'EOF'\n"
                "#!/usr/bin/env bash\n"
                "exit 0\n"
                "EOF\n"
                "chmod +x \"" +
                replay_dir +
                "/commands/validate_actual_bag_events.sh\"\n");
  makeExecutable(script_path);
}

}  // namespace

TEST(ActualBagTestSuiteScript, DryRunCreatesSmokeFullAndValidationCommands) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_suite");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/suite";
  writeFile(bag, "not a real bag");

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out +
      " --smoke-duration 5 --full-duration 7 --rate 1.0 --skip-bag-inspect";
  ASSERT_EQ(0, std::system(command.c_str()));

  ASSERT_TRUE(exists(out + "/commands/run_suite.sh"));
  ASSERT_TRUE(exists(out + "/commands/run_verified_suite.sh"));
  ASSERT_TRUE(exists(out + "/commands/run_smoke_replay.sh"));
  ASSERT_TRUE(exists(out + "/commands/run_full_replay.sh"));
  ASSERT_TRUE(exists(out + "/commands/validate_actual_bag_test_suite.sh"));
  ASSERT_TRUE(exists(out + "/commands/audit_field_acceptance_gap.sh"));
  ASSERT_TRUE(exists(out + "/commands/validate_field_acceptance_gap.sh"));
  ASSERT_TRUE(exists(out + "/commands/audit_actual_bag_initial_test_readiness.sh"));
  ASSERT_TRUE(exists(out + "/commands/validate_actual_bag_initial_test_readiness.sh"));
  ASSERT_TRUE(exists(out + "/commands/generate_field_acceptance_handoff.sh"));
  ASSERT_TRUE(exists(out + "/commands/validate_field_acceptance_handoff.sh"));
  ASSERT_TRUE(exists(out + "/commands/validate_field_acceptance_handoff_manifest.sh"));
  ASSERT_TRUE(exists(out + "/commands/generate_field_acceptance_collection_plan.sh"));
  ASSERT_TRUE(exists(out + "/commands/validate_field_acceptance_collection_plan.sh"));
  ASSERT_TRUE(exists(out + "/reports/actual_bag_test_suite_plan.txt"));
  ASSERT_TRUE(exists(out + "/reports/actual_bag_test_suite_summary.txt"));
  ASSERT_TRUE(exists(out + "/reports/actual_bag_test_suite_manifest.txt"));
  ASSERT_TRUE(exists(out + "/reports/field_acceptance_gap_report.txt"));
  ASSERT_TRUE(exists(out + "/reports/actual_bag_initial_test_readiness.txt"));
  ASSERT_TRUE(exists(out + "/reports/field_acceptance_handoff.txt"));
  ASSERT_TRUE(exists(out + "/reports/field_acceptance_handoff_manifest.txt"));
  ASSERT_TRUE(exists(out + "/reports/field_acceptance_collection_plan.txt"));

  const std::string smoke = readFile(out + "/commands/run_smoke_replay.sh");
  EXPECT_NE(std::string::npos, smoke.find("actual_bag_replay.sh"));
  EXPECT_NE(std::string::npos, smoke.find("--duration 5"));
  EXPECT_NE(std::string::npos, smoke.find("--rate 1.0"));
  EXPECT_NE(std::string::npos, smoke.find("--execute"));
  EXPECT_NE(std::string::npos, smoke.find("--skip-bag-inspect"));
  EXPECT_NE(std::string::npos, smoke.find("smoke_5s_rate1_0"));

  const std::string full = readFile(out + "/commands/run_full_replay.sh");
  EXPECT_NE(std::string::npos, full.find("actual_bag_replay.sh"));
  EXPECT_NE(std::string::npos, full.find("--duration 7"));
  EXPECT_NE(std::string::npos, full.find("--rate 1.0"));
  EXPECT_NE(std::string::npos, full.find("--execute"));
  EXPECT_NE(std::string::npos, full.find("--skip-bag-inspect"));
  EXPECT_NE(std::string::npos, full.find("full_7s_rate1_0"));

  const std::string suite = readFile(out + "/commands/run_suite.sh");
  EXPECT_NE(std::string::npos, suite.find("run_smoke_replay.sh"));
  EXPECT_NE(std::string::npos, suite.find("run_full_replay.sh"));
  EXPECT_NE(std::string::npos, suite.find("validate_actual_bag_events.sh"));
  EXPECT_NE(std::string::npos,
            suite.find("actual_bag_test_suite_summary.txt"));
  EXPECT_NE(std::string::npos,
            suite.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos,
            suite.find("smoke_event_validation_status="));
  EXPECT_NE(std::string::npos,
            suite.find("full_event_validation_status="));
  EXPECT_NE(std::string::npos, suite.find("pgrep -af"));

  const std::string verified_suite =
      readFile(out + "/commands/run_verified_suite.sh");
  const std::size_t run_suite_position =
      verified_suite.find("run_suite.sh");
  const std::size_t manifest_validation_position =
      verified_suite.find("validate_actual_bag_test_suite.sh");
  const std::size_t final_manifest_validation_position =
      verified_suite.find("validate_actual_bag_test_suite.sh",
                          manifest_validation_position + 1);
  const std::size_t gap_audit_position =
      verified_suite.find("audit_field_acceptance_gap.sh");
  const std::size_t gap_validation_position =
      verified_suite.find("validate_field_acceptance_gap.sh");
  const std::size_t readiness_audit_position =
      verified_suite.find("audit_actual_bag_initial_test_readiness.sh");
  const std::size_t readiness_validation_position =
      verified_suite.find("validate_actual_bag_initial_test_readiness.sh");
  const std::size_t handoff_generation_position =
      verified_suite.find("generate_field_acceptance_handoff.sh");
  const std::size_t handoff_validation_position =
      verified_suite.find("validate_field_acceptance_handoff.sh");
  const std::size_t handoff_manifest_validation_position =
      verified_suite.find("validate_field_acceptance_handoff_manifest.sh");
  const std::size_t collection_plan_generation_position =
      verified_suite.find("generate_field_acceptance_collection_plan.sh");
  const std::size_t collection_plan_validation_position =
      verified_suite.find("validate_field_acceptance_collection_plan.sh");
  EXPECT_NE(std::string::npos, run_suite_position);
  EXPECT_NE(std::string::npos, manifest_validation_position);
  EXPECT_NE(std::string::npos, final_manifest_validation_position);
  EXPECT_NE(std::string::npos, gap_audit_position);
  EXPECT_NE(std::string::npos, gap_validation_position);
  EXPECT_NE(std::string::npos, readiness_audit_position);
  EXPECT_NE(std::string::npos, readiness_validation_position);
  EXPECT_NE(std::string::npos, handoff_generation_position);
  EXPECT_NE(std::string::npos, handoff_validation_position);
  EXPECT_NE(std::string::npos, handoff_manifest_validation_position);
  EXPECT_NE(std::string::npos, collection_plan_generation_position);
  EXPECT_NE(std::string::npos, collection_plan_validation_position);
  EXPECT_LT(run_suite_position, manifest_validation_position);
  EXPECT_LT(manifest_validation_position, gap_audit_position);
  EXPECT_LT(gap_audit_position, gap_validation_position);
  EXPECT_LT(gap_validation_position, readiness_audit_position);
  EXPECT_LT(readiness_audit_position, readiness_validation_position);
  EXPECT_LT(readiness_validation_position, handoff_generation_position);
  EXPECT_LT(handoff_generation_position, handoff_validation_position);
  EXPECT_LT(handoff_validation_position, handoff_manifest_validation_position);
  EXPECT_LT(handoff_manifest_validation_position, collection_plan_generation_position);
  EXPECT_LT(collection_plan_generation_position, collection_plan_validation_position);
  EXPECT_LT(collection_plan_validation_position, final_manifest_validation_position);
  EXPECT_NE(std::string::npos,
            verified_suite.find("field_acceptance_gap_audit_exit=1"));
  EXPECT_NE(std::string::npos,
            verified_suite.find(
                "actual_bag_initial_test_readiness_after_execute=PASS"));
  EXPECT_NE(std::string::npos,
            verified_suite.find("field_acceptance_handoff_after_execute=PASS"));
  EXPECT_NE(std::string::npos,
            verified_suite.find(
                "field_acceptance_handoff_manifest_after_execute=PASS"));
  EXPECT_NE(std::string::npos,
            verified_suite.find(
                "field_acceptance_collection_plan_after_execute=PASS"));

  const std::string validate =
      readFile(out + "/commands/validate_actual_bag_test_suite.sh");
  EXPECT_NE(std::string::npos,
            validate.find("actual_bag_test_suite_manifest.txt"));
  EXPECT_NE(std::string::npos,
            validate.find("actual_bag_test_suite_manifest_validation.txt"));
  EXPECT_NE(std::string::npos,
            validate.find("actual_bag_test_suite_manifest_validation_status="));
  EXPECT_NE(std::string::npos,
            validate.find("velocity_reference_played_to_slam"));
  EXPECT_NE(std::string::npos, validate.find("plc_feedback_status"));

  const std::string gap_audit =
      readFile(out + "/commands/audit_field_acceptance_gap.sh");
  EXPECT_NE(std::string::npos,
            gap_audit.find("validate_actual_bag_test_suite.sh"));
  EXPECT_NE(std::string::npos,
            gap_audit.find("field_acceptance_gap_report.txt"));
  EXPECT_NE(std::string::npos, gap_audit.find("field_acceptance_ready=NO"));
  EXPECT_NE(std::string::npos,
            gap_audit.find("plc_feedback_evidence_status=MISSING"));
  EXPECT_NE(std::string::npos,
            gap_audit.find("plc_feedback_collection_command="));
  EXPECT_NE(std::string::npos,
            gap_audit.find("section_export_collection_command="));
  EXPECT_NE(std::string::npos,
            gap_audit.find("pps_ptp_wiring_collection_command="));
  EXPECT_NE(std::string::npos,
            gap_audit.find("runtime_deployment_collection_command="));
  EXPECT_NE(std::string::npos,
            gap_audit.find("runtime_stability_24h_collection_command="));
  EXPECT_NE(std::string::npos,
            gap_audit.find("pps_ptp_wiring_evidence_status=MISSING"));
  EXPECT_NE(std::string::npos,
            gap_audit.find("runtime_stability_24h_evidence_status=MISSING"));

  const std::string gap_validator =
      readFile(out + "/commands/validate_field_acceptance_gap.sh");
  EXPECT_NE(std::string::npos,
            gap_validator.find("field_acceptance_gap_validation_status="));
  EXPECT_NE(std::string::npos,
            gap_validator.find("field_acceptance_ready"));
  EXPECT_NE(std::string::npos,
            gap_validator.find("required_next_evidence"));
  EXPECT_NE(std::string::npos,
            gap_validator.find("plc_feedback_collection_command"));
  EXPECT_NE(std::string::npos,
            gap_validator.find("field_acceptance_collection_command"));

  const std::string readiness_audit =
      readFile(out + "/commands/audit_actual_bag_initial_test_readiness.sh");
  EXPECT_NE(std::string::npos,
            readiness_audit.find("actual_bag_initial_test_readiness.txt"));
  EXPECT_NE(std::string::npos,
            readiness_audit.find("validate_actual_bag_test_suite.sh"));
  EXPECT_NE(std::string::npos,
            readiness_audit.find("validate_field_acceptance_gap.sh"));

  const std::string readiness_validator =
      readFile(out + "/commands/validate_actual_bag_initial_test_readiness.sh");
  EXPECT_NE(std::string::npos,
            readiness_validator.find(
                "actual_bag_initial_test_readiness_status"));
  EXPECT_NE(std::string::npos,
            readiness_validator.find("actual_bag_user_bag_test_ready"));
  EXPECT_NE(std::string::npos,
            readiness_validator.find("field_acceptance_eligible"));

  const std::string handoff_generator =
      readFile(out + "/commands/generate_field_acceptance_handoff.sh");
  EXPECT_NE(std::string::npos,
            handoff_generator.find("field_acceptance_handoff.txt"));
  EXPECT_NE(std::string::npos,
            handoff_generator.find("validate_actual_bag_initial_test_readiness.sh"));
  EXPECT_NE(std::string::npos,
            handoff_generator.find("validate_field_acceptance_gap.sh"));

  const std::string handoff_validator =
      readFile(out + "/commands/validate_field_acceptance_handoff.sh");
  EXPECT_NE(std::string::npos,
            handoff_validator.find("field_acceptance_handoff_validation_status"));
  EXPECT_NE(std::string::npos,
            handoff_validator.find("field_acceptance_handoff_status"));
  EXPECT_NE(std::string::npos,
            handoff_validator.find("plc_feedback_collection_command"));

  const std::string handoff_manifest_validator =
      readFile(out + "/commands/validate_field_acceptance_handoff_manifest.sh");
  EXPECT_NE(std::string::npos,
            handoff_manifest_validator.find(
                "field_acceptance_handoff_manifest_validation_status"));
  EXPECT_NE(std::string::npos,
            handoff_manifest_validator.find("field_acceptance_handoff_manifest.txt"));
  EXPECT_NE(std::string::npos,
            handoff_manifest_validator.find("field_acceptance_handoff_validation"));

  const std::string collection_plan_generator =
      readFile(out + "/commands/generate_field_acceptance_collection_plan.sh");
  EXPECT_NE(std::string::npos,
            collection_plan_generator.find("field_acceptance_collection_plan.txt"));
  EXPECT_NE(std::string::npos,
            collection_plan_generator.find("validate_field_acceptance_handoff_manifest.sh"));
  EXPECT_NE(std::string::npos,
            collection_plan_generator.find("validate_field_acceptance_handoff.sh"));

  const std::string collection_plan_validator =
      readFile(out + "/commands/validate_field_acceptance_collection_plan.sh");
  EXPECT_NE(std::string::npos,
            collection_plan_validator.find(
                "field_acceptance_collection_plan_validation_status"));
  EXPECT_NE(std::string::npos,
            collection_plan_validator.find("field_acceptance_collection_plan.txt"));
  EXPECT_NE(std::string::npos,
            collection_plan_validator.find("final_success_gate"));

  const std::string plan =
      readFile(out + "/reports/actual_bag_test_suite_plan.txt");
  EXPECT_NE(std::string::npos,
            plan.find("actual_bag_test_suite_plan_status=READY"));
  EXPECT_NE(std::string::npos, plan.find("smoke_duration_s=5"));
  EXPECT_NE(std::string::npos, plan.find("full_duration_s=7"));
  EXPECT_NE(std::string::npos, plan.find("rate=1.0"));
  EXPECT_NE(std::string::npos,
            plan.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos,
            plan.find("actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY"));
  EXPECT_NE(std::string::npos,
            plan.find("run_command=" + out + "/commands/run_suite.sh"));
  EXPECT_NE(std::string::npos,
            plan.find("run_verified_command=" + out +
                      "/commands/run_verified_suite.sh"));
  EXPECT_NE(std::string::npos,
            plan.find("validate_command=" + out +
                      "/commands/validate_actual_bag_test_suite.sh"));
  EXPECT_NE(std::string::npos,
            plan.find("field_acceptance_gap_audit_command=" + out +
                      "/commands/audit_field_acceptance_gap.sh"));
  EXPECT_NE(std::string::npos,
            plan.find("field_acceptance_gap_validation_command=" + out +
                      "/commands/validate_field_acceptance_gap.sh"));
  EXPECT_NE(std::string::npos,
            plan.find("actual_bag_initial_test_readiness_audit_command=" +
                      out +
                      "/commands/audit_actual_bag_initial_test_readiness.sh"));
  EXPECT_NE(std::string::npos,
            plan.find("actual_bag_initial_test_readiness_validation_command=" +
                      out +
                      "/commands/validate_actual_bag_initial_test_readiness.sh"));
  EXPECT_NE(std::string::npos,
            plan.find("field_acceptance_handoff_command=" + out +
                      "/commands/generate_field_acceptance_handoff.sh"));
  EXPECT_NE(std::string::npos,
            plan.find("field_acceptance_handoff_validation_command=" + out +
                      "/commands/validate_field_acceptance_handoff.sh"));
  EXPECT_NE(std::string::npos,
            plan.find("field_acceptance_handoff_manifest=reports/"
                      "field_acceptance_handoff_manifest.txt"));
  EXPECT_NE(std::string::npos,
            plan.find("field_acceptance_handoff_manifest_validation_command=" +
                      out +
                      "/commands/validate_field_acceptance_handoff_manifest.sh"));
  EXPECT_NE(std::string::npos,
            plan.find("field_acceptance_collection_plan=reports/"
                      "field_acceptance_collection_plan.txt"));
  EXPECT_NE(std::string::npos,
            plan.find("field_acceptance_collection_plan_command=" + out +
                      "/commands/generate_field_acceptance_collection_plan.sh"));
  EXPECT_NE(std::string::npos,
            plan.find("field_acceptance_collection_plan_validation_command=" +
                      out +
                      "/commands/validate_field_acceptance_collection_plan.sh"));

  const std::string manifest =
      readFile(out + "/reports/actual_bag_test_suite_manifest.txt");
  EXPECT_NE(std::string::npos,
            manifest.find("actual_bag_test_suite_manifest_status=READY"));
  EXPECT_NE(std::string::npos,
            manifest.find("summary=reports/actual_bag_test_suite_summary.txt"));
  EXPECT_NE(std::string::npos,
            manifest.find("metrics_report=reports/"
                          "actual_bag_test_suite_metrics_report.txt"));
  EXPECT_NE(std::string::npos,
            manifest.find("event_file=reports/actual_bag_test_suite_events.txt"));
  EXPECT_NE(std::string::npos,
            manifest.find("smoke_summary=smoke_5s_rate1_0/reports/"
                          "actual_bag_replay_summary.txt"));
  EXPECT_NE(std::string::npos,
            manifest.find("full_summary=full_7s_rate1_0/reports/"
                          "actual_bag_replay_summary.txt"));
  EXPECT_NE(std::string::npos,
            manifest.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos,
            manifest.find("validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY"));
  EXPECT_NE(std::string::npos,
            manifest.find("field_acceptance_gap_report=reports/"
                          "field_acceptance_gap_report.txt"));
  EXPECT_NE(std::string::npos,
            manifest.find("run_verified_command=" + out +
                          "/commands/run_verified_suite.sh"));
  EXPECT_NE(std::string::npos,
            manifest.find("field_acceptance_gap_validation_command=" + out +
                          "/commands/validate_field_acceptance_gap.sh"));
  EXPECT_NE(std::string::npos,
            manifest.find("actual_bag_initial_test_readiness=reports/"
                          "actual_bag_initial_test_readiness.txt"));
  EXPECT_NE(std::string::npos,
            manifest.find(
                "actual_bag_initial_test_readiness_audit_command=" + out +
                "/commands/audit_actual_bag_initial_test_readiness.sh"));
  EXPECT_NE(std::string::npos,
            manifest.find(
                "actual_bag_initial_test_readiness_validation_command=" + out +
                "/commands/validate_actual_bag_initial_test_readiness.sh"));
  EXPECT_NE(std::string::npos,
            manifest.find("field_acceptance_handoff=reports/"
                          "field_acceptance_handoff.txt"));
  EXPECT_NE(std::string::npos,
            manifest.find("field_acceptance_handoff_command=" + out +
                          "/commands/generate_field_acceptance_handoff.sh"));
  EXPECT_NE(std::string::npos,
            manifest.find("field_acceptance_handoff_validation_command=" + out +
                          "/commands/validate_field_acceptance_handoff.sh"));
  EXPECT_NE(std::string::npos,
            manifest.find("field_acceptance_handoff_manifest=reports/"
                          "field_acceptance_handoff_manifest.txt"));
  EXPECT_NE(std::string::npos,
            manifest.find(
                "field_acceptance_handoff_manifest_validation_command=" + out +
                "/commands/validate_field_acceptance_handoff_manifest.sh"));
  EXPECT_NE(std::string::npos,
            manifest.find("field_acceptance_collection_plan=reports/"
                          "field_acceptance_collection_plan.txt"));
  EXPECT_NE(std::string::npos,
            manifest.find("field_acceptance_collection_plan_validation=reports/"
                          "field_acceptance_collection_plan_validation.txt"));
  EXPECT_NE(std::string::npos,
            manifest.find("field_acceptance_collection_plan_command=" + out +
                          "/commands/generate_field_acceptance_collection_plan.sh"));
  EXPECT_NE(std::string::npos,
            manifest.find(
                "field_acceptance_collection_plan_validation_command=" + out +
                "/commands/validate_field_acceptance_collection_plan.sh"));

  const std::string summary =
      readFile(out + "/reports/actual_bag_test_suite_summary.txt");
  EXPECT_NE(std::string::npos,
            summary.find("actual_bag_test_suite_status=DRY_RUN"));
  EXPECT_NE(std::string::npos,
            summary.find("run_verified_command=" + out +
                         "/commands/run_verified_suite.sh"));
  EXPECT_NE(std::string::npos,
            summary.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos,
            summary.find("generated_event_validation=YES"));
  EXPECT_NE(std::string::npos,
            summary.find("field_acceptance_handoff_manifest=reports/"
                         "field_acceptance_handoff_manifest.txt"));
  EXPECT_NE(std::string::npos,
            summary.find("field_acceptance_collection_plan=reports/"
                         "field_acceptance_collection_plan.txt"));

  const std::string readiness =
      readFile(out + "/reports/actual_bag_initial_test_readiness.txt");
  EXPECT_NE(std::string::npos,
            readiness.find(
                "actual_bag_initial_test_readiness_status=DRY_RUN"));
  EXPECT_NE(std::string::npos,
            readiness.find("actual_bag_user_bag_test_ready=NO"));
  EXPECT_NE(std::string::npos,
            readiness.find("field_acceptance_eligible=NO"));

  const std::string handoff =
      readFile(out + "/reports/field_acceptance_handoff.txt");
  EXPECT_NE(std::string::npos,
            handoff.find("field_acceptance_handoff_status=DRY_RUN"));
  EXPECT_NE(std::string::npos,
            handoff.find("field_acceptance_handoff_ready=NO"));
  EXPECT_NE(std::string::npos,
            handoff.find(
                "field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"));

  const std::string handoff_manifest =
      readFile(out + "/reports/field_acceptance_handoff_manifest.txt");
  EXPECT_NE(std::string::npos,
            handoff_manifest.find(
                "field_acceptance_handoff_manifest_status=READY"));
  EXPECT_NE(std::string::npos,
            handoff_manifest.find(
                "handoff_bundle_scope=FIELD_ACCEPTANCE_EVIDENCE_COLLECTION"));
  EXPECT_NE(std::string::npos,
            handoff_manifest.find("field_acceptance_handoff=reports/"
                                  "field_acceptance_handoff.txt"));
  EXPECT_NE(std::string::npos,
            handoff_manifest.find("field_acceptance_handoff_validation=reports/"
                                  "field_acceptance_handoff_validation.txt"));

  const std::string collection_plan =
      readFile(out + "/reports/field_acceptance_collection_plan.txt");
  EXPECT_NE(std::string::npos,
            collection_plan.find(
                "field_acceptance_collection_plan_status=DRY_RUN"));
  EXPECT_NE(std::string::npos,
            collection_plan.find(
                "collection_plan_scope=FIELD_ACCEPTANCE_EVIDENCE_COLLECTION"));
  EXPECT_NE(std::string::npos,
            collection_plan.find(
                "final_success_gate=record_session.sh generated commands/validate_evidence.sh => field_acceptance_status=PASS"));

  const std::string gap =
      readFile(out + "/reports/field_acceptance_gap_report.txt");
  EXPECT_NE(std::string::npos,
            gap.find("field_acceptance_gap_audit_status=DRY_RUN"));
  EXPECT_NE(std::string::npos, gap.find("field_acceptance_ready=NO"));
  EXPECT_NE(std::string::npos,
            gap.find("field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"));
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_gap.sh").c_str()));
}

TEST(ActualBagTestSuiteScript, DryRunPropagatesReplayTopicOverrides) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_suite_topics");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/suite";
  writeFile(bag, "not a real bag");

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out +
      " --smoke-duration 5 --full-duration 7 --rate 1.0 --skip-bag-inspect"
      " --center-topic /raw/center_points"
      " --left-topic /raw/left_points"
      " --right-topic /raw/right_points"
      " --imu-topic /sensors/imu_raw"
      " --time-reference-topic /clock/time_reference"
      " --initial-velocity-topic /nav/inspvax";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string smoke = readFile(out + "/commands/run_smoke_replay.sh");
  EXPECT_NE(std::string::npos, smoke.find("--center-topic /raw/center_points"));
  EXPECT_NE(std::string::npos, smoke.find("--left-topic /raw/left_points"));
  EXPECT_NE(std::string::npos, smoke.find("--right-topic /raw/right_points"));
  EXPECT_NE(std::string::npos, smoke.find("--imu-topic /sensors/imu_raw"));
  EXPECT_NE(std::string::npos,
            smoke.find("--time-reference-topic /clock/time_reference"));
  EXPECT_NE(std::string::npos,
            smoke.find("--initial-velocity-topic /nav/inspvax"));

  const std::string full = readFile(out + "/commands/run_full_replay.sh");
  EXPECT_NE(std::string::npos, full.find("--center-topic /raw/center_points"));
  EXPECT_NE(std::string::npos, full.find("--left-topic /raw/left_points"));
  EXPECT_NE(std::string::npos, full.find("--right-topic /raw/right_points"));
  EXPECT_NE(std::string::npos, full.find("--imu-topic /sensors/imu_raw"));
  EXPECT_NE(std::string::npos,
            full.find("--time-reference-topic /clock/time_reference"));
  EXPECT_NE(std::string::npos,
            full.find("--initial-velocity-topic /nav/inspvax"));

  const std::string plan =
      readFile(out + "/reports/actual_bag_test_suite_plan.txt");
  EXPECT_NE(std::string::npos, plan.find("center_lidar_topic=/raw/center_points"));
  EXPECT_NE(std::string::npos, plan.find("left_lidar_topic=/raw/left_points"));
  EXPECT_NE(std::string::npos, plan.find("right_lidar_topic=/raw/right_points"));
  EXPECT_NE(std::string::npos, plan.find("imu_topic=/sensors/imu_raw"));
  EXPECT_NE(std::string::npos,
            plan.find("time_reference_topic=/clock/time_reference"));
  EXPECT_NE(std::string::npos,
            plan.find("initial_velocity_reference_topic=/nav/inspvax"));
}

TEST(ActualBagTestSuiteScript, DryRunPropagatesNoTimeReferenceMode) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_suite_no_time_ref");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/suite";
  writeFile(bag, "not a real bag");

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out +
      " --smoke-duration 5 --full-duration 7 --rate 1.0 --skip-bag-inspect"
      " --no-time-reference";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string smoke = readFile(out + "/commands/run_smoke_replay.sh");
  EXPECT_NE(std::string::npos, smoke.find("--no-time-reference"));
  EXPECT_EQ(std::string::npos, smoke.find("--time-reference-topic"));

  const std::string full = readFile(out + "/commands/run_full_replay.sh");
  EXPECT_NE(std::string::npos, full.find("--no-time-reference"));
  EXPECT_EQ(std::string::npos, full.find("--time-reference-topic"));

  const std::string plan =
      readFile(out + "/reports/actual_bag_test_suite_plan.txt");
  EXPECT_NE(std::string::npos, plan.find("time_reference_topic=NONE"));
  EXPECT_NE(std::string::npos,
            plan.find("time_reference_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            plan.find("time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST"));
}

TEST(ActualBagTestSuiteScript, DryRunPropagatesNoInitialVelocityReferenceMode) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_suite_no_initial_velocity");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/suite";
  writeFile(bag, "not a real bag");

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out +
      " --smoke-duration 5 --full-duration 7 --rate 1.0 --skip-bag-inspect"
      " --no-initial-velocity-reference";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string smoke = readFile(out + "/commands/run_smoke_replay.sh");
  EXPECT_NE(std::string::npos,
            smoke.find("--no-initial-velocity-reference"));
  EXPECT_EQ(std::string::npos, smoke.find("--initial-velocity-topic"));

  const std::string full = readFile(out + "/commands/run_full_replay.sh");
  EXPECT_NE(std::string::npos,
            full.find("--no-initial-velocity-reference"));
  EXPECT_EQ(std::string::npos, full.find("--initial-velocity-topic"));

  const std::string plan =
      readFile(out + "/reports/actual_bag_test_suite_plan.txt");
  EXPECT_NE(std::string::npos,
            plan.find("initial_velocity_reference_topic=NONE"));
  EXPECT_NE(std::string::npos,
            plan.find("initial_velocity_reference_required=NO"));
  EXPECT_NE(std::string::npos,
            plan.find("initial_velocity_reference_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            plan.find("initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST"));

  const std::string manifest =
      readFile(out + "/reports/actual_bag_test_suite_manifest.txt");
  EXPECT_NE(std::string::npos,
            manifest.find("initial_velocity_reference_topic=NONE"));
  EXPECT_NE(std::string::npos,
            manifest.find("initial_velocity_reference_required=NO"));
}

TEST(ActualBagTestSuiteScript, RunSuiteWritesNoInitialVelocitySummaryFields) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_suite_run_no_velocity");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/suite";
  writeFile(bag, "not a real bag");

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out +
      " --smoke-duration 5 --full-duration 7 --rate 1.0 --skip-bag-inspect"
      " --no-initial-velocity-reference";
  ASSERT_EQ(0, std::system(command.c_str()));

  writeReplayStub(out + "/commands/run_smoke_replay.sh",
                  out + "/smoke_5s_rate1_0");
  writeReplayStub(out + "/commands/run_full_replay.sh",
                  out + "/full_7s_rate1_0");

  ASSERT_EQ(0, std::system((out + "/commands/run_suite.sh").c_str()));
  const std::string summary =
      readFile(out + "/reports/actual_bag_test_suite_summary.txt");
  EXPECT_NE(std::string::npos,
            summary.find("initial_velocity_reference_status=NOT_PRESENT_INITIAL_TEST"));
  EXPECT_NE(std::string::npos,
            summary.find("initial_velocity_reference_required=NO"));
  EXPECT_NE(std::string::npos,
            summary.find("initial_velocity_reference_topic=NONE"));
  EXPECT_NE(std::string::npos,
            summary.find("initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST"));
}

TEST(ActualBagTestSuiteScript, ManifestValidatorPassesSyntheticExecutedSuite) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_suite_validate");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/suite";
  writeFile(bag, "not a real bag");

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out +
      " --smoke-duration 5 --full-duration 7 --rate 1.0 --skip-bag-inspect";
  ASSERT_EQ(0, std::system(command.c_str()));

  makeDir(out + "/smoke_5s_rate1_0/reports");
  makeDir(out + "/full_7s_rate1_0/reports");
  writeFile(out + "/reports/actual_bag_test_suite_summary.txt",
            "actual_bag_test_suite_status=PASS\n"
            "smoke_replay_status=PASS\n"
            "full_replay_status=PASS\n"
            "smoke_event_validation_status=PASS\n"
            "full_event_validation_status=PASS\n"
            "ros_residual_status=PASS\n"
	            "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n"
	            "bag_sensor_set=LIDAR_IMU_ONLY\n"
	            "plc_feedback_status=NOT_PRESENT_NA\n"
	            "plc_feedback_gate_status=NA_INITIAL_TEST\n"
	            "machine_motion_assumption=CONTINUOUS_MOTION\n"
	            "vibration_profile=NORMAL\n"
	            "time_reference_status=PRESENT_REQUIRED\n"
	            "time_sync_evidence_status=INITIAL_TIME_STATUS_CAPTURE_REQUIRED\n"
	            "initial_velocity_reference_required=YES\n"
	            "initial_velocity_reference_topic=/novatel_data/inspvax\n"
	            "initial_velocity_reference_policy=START_ONLY_AUDIT\n"
	            "velocity_reference_played_to_slam=NO\n"
	            "continuous_velocity_reference_used=NO\n"
	            "field_acceptance_eligible=NO\n"
	            "field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY\n");
  writeFile(out + "/reports/actual_bag_test_suite_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "session=suite;scenario=ACTUAL_BAG_LIDAR_IMU_SUITE;status=PASS;"
            "failed_checks=0;validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY;"
            "field_acceptance_eligible=NO;"
            "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n");
  writeFile(out + "/reports/actual_bag_test_suite_events.txt",
            "event=actual_bag_test_suite;scenario=ACTUAL_BAG_LIDAR_IMU_SUITE;"
            "session_id=suite;t=7;actual_bag_test_suite_status=PASS;"
            "field_acceptance_eligible=NO;velocity_reference_played_to_slam=NO;"
            "continuous_velocity_reference_used=NO\n");
  writeFile(out + "/reports/ros_residual_processes.txt", "");

  const std::string replay_summary =
      "actual_bag_replay_status=PASS\n"
      "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n"
      "bag_sensor_set=LIDAR_IMU_ONLY\n"
      "plc_feedback_status=NOT_PRESENT_NA\n"
      "time_reference_status=PRESENT_REQUIRED\n"
      "time_sync_evidence_status=INITIAL_TIME_STATUS_CAPTURE_REQUIRED\n"
      "velocity_reference_played_to_slam=NO\n"
      "continuous_velocity_reference_used=NO\n"
      "field_acceptance_requires_plc_feedback=YES\n";
  writeFile(out + "/smoke_5s_rate1_0/reports/actual_bag_replay_summary.txt",
            replay_summary);
  writeFile(out + "/full_7s_rate1_0/reports/actual_bag_replay_summary.txt",
            replay_summary);
  writeFile(out + "/smoke_5s_rate1_0/reports/"
                    "actual_bag_replay_hil_validation_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(out + "/full_7s_rate1_0/reports/"
                    "actual_bag_replay_hil_validation_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n");

  const std::string velocity =
      "initial_velocity_reference_status=CAPTURED\n"
      "initial_velocity_reference_policy=START_ONLY_AUDIT\n"
      "continuous_velocity_reference_used=NO\n"
      "velocity_reference_played_to_slam=NO\n";
  writeFile(out + "/smoke_5s_rate1_0/reports/initial_velocity_reference.txt",
            velocity);
  writeFile(out + "/full_7s_rate1_0/reports/initial_velocity_reference.txt",
            velocity);

  const std::string inspection =
      "actual_bag_status=PASS\n"
      "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n"
      "bag_sensor_set=LIDAR_IMU_ONLY\n"
      "plc_feedback_status=NOT_PRESENT_NA\n"
      "plc_feedback_gate_status=NA_INITIAL_TEST\n"
      "field_acceptance_requires_plc_feedback=YES\n"
      "time_reference_status=PRESENT_REQUIRED\n"
      "time_sync_evidence_status=INITIAL_TIME_STATUS_CAPTURE_REQUIRED\n"
      "topic_count[/plc/left_track_speed]=0\n"
      "topic_count[/plc/right_track_speed]=0\n"
      "topic_count[/plc/cutting_on]=0\n"
      "topic_count[/machine/state]=0\n"
      "continuous_velocity_reference_used=NO\n"
      "velocity_reference_played_to_slam=NO\n";
  writeFile(out + "/smoke_5s_rate1_0/reports/actual_bag_inspection.txt",
            inspection);
  writeFile(out + "/full_7s_rate1_0/reports/actual_bag_inspection.txt",
            inspection);

  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));
  const std::string smoke_velocity_path =
      out + "/smoke_5s_rate1_0/reports/initial_velocity_reference.txt";
  const std::string smoke_velocity = readFile(smoke_velocity_path);
  std::string tampered_smoke_velocity = smoke_velocity;
  const std::string captured_status = "initial_velocity_reference_status=CAPTURED";
  const std::size_t captured_position =
      tampered_smoke_velocity.find(captured_status);
  ASSERT_NE(std::string::npos, captured_position);
  tampered_smoke_velocity.replace(
      captured_position, captured_status.size(),
      "initial_velocity_reference_status=MISSING");
  writeFile(smoke_velocity_path, tampered_smoke_velocity);
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));
  const std::string missing_velocity_report = readFile(
      out + "/reports/actual_bag_test_suite_manifest_validation.txt");
  EXPECT_NE(std::string::npos,
            missing_velocity_report.find(
                "smoke_initial_velocity_initial_velocity_reference_status_status=FAIL"));
  writeFile(smoke_velocity_path, smoke_velocity);
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "audit_field_acceptance_gap.sh").c_str()));
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_gap.sh").c_str()));
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "audit_actual_bag_initial_test_readiness.sh").c_str()));
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_actual_bag_initial_test_readiness.sh").c_str()));
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "generate_field_acceptance_handoff.sh").c_str()));
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_handoff.sh").c_str()));
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_handoff_manifest.sh").c_str()));
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "generate_field_acceptance_collection_plan.sh").c_str()));
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_collection_plan.sh").c_str()));

  const std::string report = readFile(
      out + "/reports/actual_bag_test_suite_manifest_validation.txt");
  EXPECT_NE(std::string::npos,
            report.find(
                "actual_bag_test_suite_manifest_validation_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos,
            report.find("velocity_reference_played_to_slam=NO"));

  const std::string readiness =
      readFile(out + "/reports/actual_bag_initial_test_readiness.txt");
  EXPECT_NE(std::string::npos,
            readiness.find(
                "actual_bag_initial_test_readiness_status=PASS"));
  EXPECT_NE(std::string::npos,
            readiness.find("actual_bag_user_bag_test_ready=YES"));
  EXPECT_NE(std::string::npos,
            readiness.find("actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY"));
  EXPECT_NE(std::string::npos,
            readiness.find("validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY"));
  EXPECT_NE(std::string::npos,
            readiness.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos,
            readiness.find(
                "field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"));
  EXPECT_NE(std::string::npos,
            readiness.find("velocity_reference_played_to_slam=NO"));
  EXPECT_NE(std::string::npos,
            readiness.find("continuous_velocity_reference_used=NO"));
  EXPECT_NE(std::string::npos,
            readiness.find(
                "required_next_evidence=PLC_FEEDBACK_BAG,SECTION_EXPORT,"
                "PPS_PTP_WIRING,POWER_LOSS_RESUME,RUNTIME_DEPLOYMENT,"
                "RUNTIME_STABILITY_24H,FIELD_ACCEPTANCE"));

  const std::string handoff =
      readFile(out + "/reports/field_acceptance_handoff.txt");
  EXPECT_NE(std::string::npos,
            handoff.find("field_acceptance_handoff_status=PASS"));
  EXPECT_NE(std::string::npos,
            handoff.find("field_acceptance_handoff_scope=FIELD_ACCEPTANCE_EVIDENCE_COLLECTION"));
  EXPECT_NE(std::string::npos,
            handoff.find("actual_bag_initial_test_readiness_status=PASS"));
  EXPECT_NE(std::string::npos,
            handoff.find("actual_bag_user_bag_test_ready=YES"));
  EXPECT_NE(std::string::npos,
            handoff.find("field_acceptance_gap_validation_status=PASS"));
  EXPECT_NE(std::string::npos,
            handoff.find("field_acceptance_ready=NO"));
  EXPECT_NE(std::string::npos,
            handoff.find("field_acceptance_eligible=NO"));
  EXPECT_NE(std::string::npos,
            handoff.find("field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"));
  EXPECT_NE(std::string::npos,
            handoff.find("plc_feedback_collection_command=record_session.sh --topics"));
  EXPECT_NE(std::string::npos,
            handoff.find("section_export_collection_command=record_session.sh generated commands/capture_section_export.sh"));
  EXPECT_NE(std::string::npos,
            handoff.find("pps_ptp_wiring_collection_command=record_session.sh generated commands/capture_time_sync.sh && commands/capture_pps_ptp_wiring.sh"));
  EXPECT_NE(std::string::npos,
            handoff.find("runtime_stability_24h_collection_command=record_session.sh generated commands/run_runtime_stability.sh"));
  EXPECT_NE(std::string::npos,
            handoff.find("field_acceptance_collection_command=record_session.sh generated commands/capture_field_acceptance.sh && commands/validate_evidence.sh"));
  EXPECT_NE(std::string::npos,
            handoff.find("final_gate_command=record_session.sh generated commands/validate_evidence.sh"));

  const std::string handoff_validation =
      readFile(out + "/reports/field_acceptance_handoff_validation.txt");
  EXPECT_NE(std::string::npos,
            handoff_validation.find("field_acceptance_handoff_validation_status=PASS"));
  EXPECT_NE(std::string::npos,
            handoff_validation.find("field_acceptance_handoff_status_status=PASS"));
  EXPECT_NE(std::string::npos,
            handoff_validation.find("field_acceptance_collection_command_status=PASS"));

  const std::string handoff_manifest_validation =
      readFile(out + "/reports/field_acceptance_handoff_manifest_validation.txt");
  EXPECT_NE(std::string::npos,
            handoff_manifest_validation.find(
                "field_acceptance_handoff_manifest_validation_status=PASS"));
  EXPECT_NE(std::string::npos,
            handoff_manifest_validation.find(
                "field_acceptance_handoff_validation_status_status=PASS"));
  EXPECT_NE(std::string::npos,
            handoff_manifest_validation.find("field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"));

  const std::string original_handoff_manifest =
      readFile(out + "/reports/field_acceptance_handoff_manifest.txt");
  const std::string external_handoff_reports =
      root + "/external_handoff/reports";
  makeDir(external_handoff_reports);
  writeFile(external_handoff_reports + "/field_acceptance_handoff_validation.txt",
            handoff_validation);
  std::string external_handoff_manifest = original_handoff_manifest;
  const std::string internal_handoff_validation =
      "field_acceptance_handoff_validation=reports/"
      "field_acceptance_handoff_validation.txt";
  const std::size_t internal_handoff_validation_position =
      external_handoff_manifest.find(internal_handoff_validation);
  ASSERT_NE(std::string::npos, internal_handoff_validation_position);
  external_handoff_manifest.replace(
      internal_handoff_validation_position, internal_handoff_validation.size(),
      "field_acceptance_handoff_validation=" + external_handoff_reports +
          "/field_acceptance_handoff_validation.txt");
  writeFile(out + "/reports/field_acceptance_handoff_manifest.txt",
            external_handoff_manifest);
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_handoff_manifest.sh").c_str()));
  const std::string external_handoff_validation =
      readFile(out + "/reports/field_acceptance_handoff_manifest_validation.txt");
  EXPECT_NE(std::string::npos,
            external_handoff_validation.find(
                "field_acceptance_handoff_validation_path_status=FAIL"));
  writeFile(out + "/reports/field_acceptance_handoff_manifest.txt",
            original_handoff_manifest);
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_handoff_manifest.sh").c_str()));

  const std::string collection_plan =
      readFile(out + "/reports/field_acceptance_collection_plan.txt");
  EXPECT_NE(std::string::npos,
            collection_plan.find("field_acceptance_collection_plan_status=PASS"));
  EXPECT_NE(std::string::npos,
            collection_plan.find("collection_plan_ready=YES"));
  EXPECT_NE(std::string::npos,
            collection_plan.find("field_acceptance_handoff_manifest_validation_status=PASS"));
  EXPECT_NE(std::string::npos,
            collection_plan.find("plc_feedback_collection_step=1"));
  EXPECT_NE(std::string::npos,
            collection_plan.find("field_acceptance_collection_step=7"));
  EXPECT_NE(std::string::npos,
            collection_plan.find(
                "final_success_gate=record_session.sh generated commands/validate_evidence.sh => field_acceptance_status=PASS"));

  const std::string collection_plan_validation =
      readFile(out + "/reports/field_acceptance_collection_plan_validation.txt");
  EXPECT_NE(std::string::npos,
            collection_plan_validation.find(
                "field_acceptance_collection_plan_validation_status=PASS"));
  EXPECT_NE(std::string::npos,
            collection_plan_validation.find(
                "final_success_gate_status=PASS"));

  const std::string external_collection_source_reports =
      root + "/external_collection_source/reports";
  makeDir(external_collection_source_reports);
  writeFile(external_collection_source_reports +
                "/field_acceptance_handoff_manifest_validation.txt",
            handoff_manifest_validation);
  std::string external_source_collection_plan = collection_plan;
  const std::string internal_handoff_manifest_validation_source =
      "source_field_acceptance_handoff_manifest_validation=" + out +
      "/reports/field_acceptance_handoff_manifest_validation.txt";
  const std::size_t internal_handoff_manifest_validation_source_position =
      external_source_collection_plan.find(
          internal_handoff_manifest_validation_source);
  ASSERT_NE(std::string::npos,
            internal_handoff_manifest_validation_source_position);
  external_source_collection_plan.replace(
      internal_handoff_manifest_validation_source_position,
      internal_handoff_manifest_validation_source.size(),
      "source_field_acceptance_handoff_manifest_validation=" +
          external_collection_source_reports +
          "/field_acceptance_handoff_manifest_validation.txt");
  writeFile(out + "/reports/field_acceptance_collection_plan.txt",
            external_source_collection_plan);
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_collection_plan.sh").c_str()));
  const std::string external_collection_source_validation =
      readFile(out + "/reports/field_acceptance_collection_plan_validation.txt");
  EXPECT_NE(std::string::npos,
            external_collection_source_validation.find(
                "source_field_acceptance_handoff_manifest_validation_path_status=FAIL"));
  writeFile(out + "/reports/field_acceptance_collection_plan.txt",
            collection_plan);
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_collection_plan.sh").c_str()));

  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));
  const std::string post_collection_manifest_validation =
      readFile(out + "/reports/actual_bag_test_suite_manifest_validation.txt");
  EXPECT_NE(std::string::npos,
            post_collection_manifest_validation.find(
                "field_acceptance_collection_plan_validation_field_acceptance_collection_plan_validation_status_status=PASS"));
  EXPECT_NE(std::string::npos,
            post_collection_manifest_validation.find(
                "field_acceptance_collection_plan_final_success_gate_status=PASS"));

  std::string stale_validation_collection_plan = collection_plan;
  const std::string stale_internal_handoff_manifest_validation_source =
      "source_field_acceptance_handoff_manifest_validation=" + out +
      "/reports/field_acceptance_handoff_manifest_validation.txt";
  const std::size_t stale_internal_handoff_manifest_validation_source_position =
      stale_validation_collection_plan.find(
          stale_internal_handoff_manifest_validation_source);
  ASSERT_NE(std::string::npos,
            stale_internal_handoff_manifest_validation_source_position);
  stale_validation_collection_plan.replace(
      stale_internal_handoff_manifest_validation_source_position,
      stale_internal_handoff_manifest_validation_source.size(),
      "source_field_acceptance_handoff_manifest_validation=" +
          external_collection_source_reports +
          "/field_acceptance_handoff_manifest_validation.txt");
  writeFile(out + "/reports/field_acceptance_collection_plan.txt",
            stale_validation_collection_plan);
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));
  const std::string stale_collection_manifest_validation =
      readFile(out + "/reports/actual_bag_test_suite_manifest_validation.txt");
  EXPECT_NE(std::string::npos,
            stale_collection_manifest_validation.find(
                "field_acceptance_collection_plan_source_field_acceptance_handoff_manifest_validation_path_status=FAIL"));
  writeFile(out + "/reports/field_acceptance_collection_plan.txt",
            collection_plan);
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_collection_plan.sh").c_str()));
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));

  std::string tampered_readiness = readiness;
  const std::string ready_yes = "actual_bag_user_bag_test_ready=YES";
  const std::size_t ready_yes_position = tampered_readiness.find(ready_yes);
  ASSERT_NE(std::string::npos, ready_yes_position);
  tampered_readiness.replace(ready_yes_position, ready_yes.size(),
                             "actual_bag_user_bag_test_ready=NO");
  writeFile(out + "/reports/actual_bag_initial_test_readiness.txt",
            tampered_readiness);
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_initial_test_readiness.sh").c_str()));
  const std::string readiness_validation =
      readFile(out + "/reports/actual_bag_initial_test_readiness_validation.txt");
  EXPECT_NE(std::string::npos,
            readiness_validation.find(
                "actual_bag_user_bag_test_ready_status=FAIL"));
  writeFile(out + "/reports/actual_bag_initial_test_readiness.txt", readiness);
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_actual_bag_initial_test_readiness.sh").c_str()));

  std::string tampered_handoff = handoff;
  const std::string not_eligible =
      "field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY";
  const std::size_t not_eligible_position = tampered_handoff.find(not_eligible);
  ASSERT_NE(std::string::npos, not_eligible_position);
  tampered_handoff.replace(not_eligible_position, not_eligible.size(),
                           "field_acceptance_status=PASS");
  writeFile(out + "/reports/field_acceptance_handoff.txt", tampered_handoff);
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_handoff.sh").c_str()));
  const std::string tampered_handoff_validation =
      readFile(out + "/reports/field_acceptance_handoff_validation.txt");
  EXPECT_NE(std::string::npos,
            tampered_handoff_validation.find("field_acceptance_status_status=FAIL"));
  writeFile(out + "/reports/field_acceptance_handoff.txt", handoff);
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_handoff.sh").c_str()));

  std::string tampered_handoff_validation_report = handoff_validation;
  const std::string handoff_validation_pass =
      "field_acceptance_handoff_validation_status=PASS";
  const std::size_t handoff_validation_pass_position =
      tampered_handoff_validation_report.find(handoff_validation_pass);
  ASSERT_NE(std::string::npos, handoff_validation_pass_position);
  tampered_handoff_validation_report.replace(
      handoff_validation_pass_position, handoff_validation_pass.size(),
      "field_acceptance_handoff_validation_status=FAIL");
  writeFile(out + "/reports/field_acceptance_handoff_validation.txt",
            tampered_handoff_validation_report);
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_handoff_manifest.sh").c_str()));
  const std::string tampered_handoff_manifest_validation =
      readFile(out + "/reports/field_acceptance_handoff_manifest_validation.txt");
  EXPECT_NE(std::string::npos,
            tampered_handoff_manifest_validation.find(
                "field_acceptance_handoff_validation_status_status=FAIL"));
  writeFile(out + "/reports/field_acceptance_handoff_validation.txt",
            handoff_validation);
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_handoff_manifest.sh").c_str()));

  std::string tampered_collection_plan = collection_plan;
  const std::string expected_final_gate =
      "final_success_gate=record_session.sh generated commands/validate_evidence.sh => field_acceptance_status=PASS";
  const std::size_t final_gate_position =
      tampered_collection_plan.find(expected_final_gate);
  ASSERT_NE(std::string::npos, final_gate_position);
  tampered_collection_plan.replace(final_gate_position, expected_final_gate.size(),
                                   "final_success_gate=missing");
  writeFile(out + "/reports/field_acceptance_collection_plan.txt",
            tampered_collection_plan);
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_collection_plan.sh").c_str()));
  const std::string tampered_collection_validation =
      readFile(out + "/reports/field_acceptance_collection_plan_validation.txt");
  EXPECT_NE(std::string::npos,
            tampered_collection_validation.find(
                "final_success_gate_status=FAIL"));
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));
  const std::string tampered_collection_manifest_validation =
      readFile(out + "/reports/actual_bag_test_suite_manifest_validation.txt");
  EXPECT_NE(
      std::string::npos,
      tampered_collection_manifest_validation.find(
          "field_acceptance_collection_plan_validation_field_acceptance_collection_plan_validation_status_status=FAIL"));
  EXPECT_NE(std::string::npos,
            tampered_collection_manifest_validation.find(
                "field_acceptance_collection_plan_validation_final_success_gate_status_status=FAIL"));
  writeFile(out + "/reports/field_acceptance_collection_plan.txt",
            collection_plan);
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_collection_plan.sh").c_str()));
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));

  const std::string original_manifest =
      readFile(out + "/reports/actual_bag_test_suite_manifest.txt");
  const std::string external_reports = root + "/external_suite/reports";
  makeDir(external_reports);
  writeFile(external_reports + "/actual_bag_test_suite_summary.txt",
            readFile(out + "/reports/actual_bag_test_suite_summary.txt"));
  std::string external_summary_manifest = original_manifest;
  const std::string internal_summary =
      "summary=reports/actual_bag_test_suite_summary.txt";
  const std::size_t internal_summary_position =
      external_summary_manifest.find(internal_summary);
  ASSERT_NE(std::string::npos, internal_summary_position);
  external_summary_manifest.replace(
      internal_summary_position, internal_summary.size(),
      "summary=" + external_reports + "/actual_bag_test_suite_summary.txt");
  writeFile(out + "/reports/actual_bag_test_suite_manifest.txt",
            external_summary_manifest);
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));
  const std::string external_summary_validation =
      readFile(out + "/reports/actual_bag_test_suite_manifest_validation.txt");
  EXPECT_NE(std::string::npos,
            external_summary_validation.find("summary_path_status=FAIL"));
  writeFile(out + "/reports/actual_bag_test_suite_manifest.txt",
            original_manifest);
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));

  const std::string gap =
      readFile(out + "/reports/field_acceptance_gap_report.txt");
  EXPECT_NE(std::string::npos,
            gap.find("field_acceptance_gap_audit_status=FAIL"));
  EXPECT_NE(std::string::npos,
            gap.find("actual_bag_initial_evidence_status=PASS"));
  EXPECT_NE(std::string::npos, gap.find("field_acceptance_ready=NO"));
  EXPECT_NE(std::string::npos,
            gap.find("plc_feedback_evidence_status=MISSING"));
  EXPECT_NE(std::string::npos,
            gap.find("plc_feedback_collection_command=record_session.sh --topics"));
  EXPECT_NE(std::string::npos,
            gap.find("section_export_collection_command=record_session.sh generated commands/capture_section_export.sh"));
  EXPECT_NE(std::string::npos,
            gap.find("pps_ptp_wiring_collection_command=record_session.sh generated commands/capture_time_sync.sh && commands/capture_pps_ptp_wiring.sh"));
  EXPECT_NE(std::string::npos,
            gap.find("runtime_deployment_collection_command=runtime_ops.sh then record_session.sh generated commands/capture_runtime_health.sh && commands/capture_runtime_deployment.sh"));
  EXPECT_NE(std::string::npos,
            gap.find("runtime_stability_24h_collection_command=record_session.sh generated commands/run_runtime_stability.sh"));
  EXPECT_NE(std::string::npos,
            gap.find("field_acceptance_collection_command=record_session.sh generated commands/capture_field_acceptance.sh && commands/validate_evidence.sh"));
  EXPECT_NE(std::string::npos,
            gap.find("pps_ptp_wiring_evidence_status=MISSING"));
  EXPECT_NE(std::string::npos,
            gap.find("power_loss_resume_evidence_status=MISSING"));
  EXPECT_NE(std::string::npos,
            gap.find("runtime_deployment_evidence_status=MISSING"));
  EXPECT_NE(std::string::npos,
            gap.find("runtime_stability_24h_evidence_status=MISSING"));
  EXPECT_NE(std::string::npos,
            gap.find("section_export_evidence_status=MISSING"));
  EXPECT_NE(std::string::npos,
            gap.find("field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY"));

  std::string tampered_collection_command = gap;
  const std::string expected_field_acceptance_command =
      "field_acceptance_collection_command=record_session.sh generated "
      "commands/capture_field_acceptance.sh && commands/validate_evidence.sh";
  const std::size_t field_acceptance_command_position =
      tampered_collection_command.find(expected_field_acceptance_command);
  ASSERT_NE(std::string::npos, field_acceptance_command_position);
  tampered_collection_command.replace(
      field_acceptance_command_position, expected_field_acceptance_command.size(),
      "field_acceptance_collection_command=missing");
  writeFile(out + "/reports/field_acceptance_gap_report.txt",
            tampered_collection_command);
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_gap.sh").c_str()));
  const std::string collection_validation =
      readFile(out + "/reports/field_acceptance_gap_validation.txt");
  EXPECT_NE(std::string::npos,
            collection_validation.find(
                "field_acceptance_collection_command_status=FAIL"));
  writeFile(out + "/reports/field_acceptance_gap_report.txt", gap);
  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_gap.sh").c_str()));

  const std::string summary =
      readFile(out + "/reports/actual_bag_test_suite_summary.txt");
  writeFile(out + "/reports/actual_bag_test_suite_summary.txt",
            summary +
                "verified_suite_status=FAIL\n"
                "suite_manifest_validation_after_execute=PASS\n"
                "field_acceptance_gap_audit_exit=1\n"
                "field_acceptance_gap_validation_after_execute=PASS\n");
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));
  const std::string verified_report = readFile(
      out + "/reports/actual_bag_test_suite_manifest_validation.txt");
  EXPECT_NE(std::string::npos,
            verified_report.find(
                "suite_summary_verified_suite_status_status=FAIL"));

  std::string tampered_gap = gap;
  const std::string expected_ready = "field_acceptance_ready=NO";
  const std::size_t ready_position = tampered_gap.find(expected_ready);
  ASSERT_NE(std::string::npos, ready_position);
  tampered_gap.replace(ready_position, expected_ready.size(),
                       "field_acceptance_ready=YES");
  writeFile(out + "/reports/field_acceptance_gap_report.txt", tampered_gap);
  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_field_acceptance_gap.sh").c_str()));
  const std::string gap_validation =
      readFile(out + "/reports/field_acceptance_gap_validation.txt");
  EXPECT_NE(std::string::npos,
            gap_validation.find("field_acceptance_gap_validation_status=FAIL"));
  EXPECT_NE(std::string::npos,
            gap_validation.find("field_acceptance_ready_status=FAIL"));
}

TEST(ActualBagTestSuiteScript,
     ManifestValidatorAllowsExplicitNoInitialVelocityReference) {
  const std::string root =
      makeTempDir("tunnel_lio_actual_bag_suite_no_velocity_validate");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/suite";
  writeFile(bag, "not a real bag");

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out +
      " --smoke-duration 5 --full-duration 7 --rate 1.0 --skip-bag-inspect"
      " --no-initial-velocity-reference";
  ASSERT_EQ(0, std::system(command.c_str()));

  makeDir(out + "/smoke_5s_rate1_0/reports");
  makeDir(out + "/full_7s_rate1_0/reports");
  writeFile(out + "/reports/actual_bag_test_suite_summary.txt",
            "actual_bag_test_suite_status=PASS\n"
            "smoke_replay_status=PASS\n"
            "full_replay_status=PASS\n"
            "smoke_event_validation_status=PASS\n"
            "full_event_validation_status=PASS\n"
            "ros_residual_status=PASS\n"
            "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n"
            "bag_sensor_set=LIDAR_IMU_ONLY\n"
            "plc_feedback_status=NOT_PRESENT_NA\n"
            "time_reference_status=PRESENT_REQUIRED\n"
            "time_sync_evidence_status=INITIAL_TIME_STATUS_CAPTURE_REQUIRED\n"
            "initial_velocity_reference_required=NO\n"
            "initial_velocity_reference_topic=NONE\n"
            "initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST\n"
            "velocity_reference_played_to_slam=NO\n"
            "continuous_velocity_reference_used=NO\n"
            "field_acceptance_eligible=NO\n");
  writeFile(out + "/reports/actual_bag_test_suite_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(out + "/reports/actual_bag_test_suite_events.txt",
            "event=actual_bag_test_suite;actual_bag_test_suite_status=PASS;"
            "field_acceptance_eligible=NO;velocity_reference_played_to_slam=NO;"
            "continuous_velocity_reference_used=NO\n");
  writeFile(out + "/reports/ros_residual_processes.txt", "");

  const std::string replay_summary =
      "actual_bag_replay_status=PASS\n"
      "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n"
      "bag_sensor_set=LIDAR_IMU_ONLY\n"
      "plc_feedback_status=NOT_PRESENT_NA\n"
      "time_reference_status=PRESENT_REQUIRED\n"
      "time_sync_evidence_status=INITIAL_TIME_STATUS_CAPTURE_REQUIRED\n"
      "initial_velocity_reference_required=NO\n"
      "initial_velocity_reference_topic=NONE\n"
      "initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST\n"
      "velocity_reference_played_to_slam=NO\n"
      "continuous_velocity_reference_used=NO\n";
  writeFile(out + "/smoke_5s_rate1_0/reports/actual_bag_replay_summary.txt",
            replay_summary);
  writeFile(out + "/full_7s_rate1_0/reports/actual_bag_replay_summary.txt",
            replay_summary);
  writeFile(out + "/smoke_5s_rate1_0/reports/"
                    "actual_bag_replay_hil_validation_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(out + "/full_7s_rate1_0/reports/"
                    "actual_bag_replay_hil_validation_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n");

  const std::string no_velocity =
      "initial_velocity_reference_status=NOT_PRESENT_INITIAL_TEST\n"
      "initial_velocity_reference_required=NO\n"
      "initial_velocity_reference_topic=NONE\n"
      "initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST\n"
      "continuous_velocity_reference_used=NO\n"
      "velocity_reference_played_to_slam=NO\n";
  writeFile(out + "/smoke_5s_rate1_0/reports/initial_velocity_reference.txt",
            no_velocity);
  writeFile(out + "/full_7s_rate1_0/reports/initial_velocity_reference.txt",
            no_velocity);

  const std::string inspection =
      "actual_bag_status=PASS\n"
      "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n"
      "bag_sensor_set=LIDAR_IMU_ONLY\n"
      "plc_feedback_status=NOT_PRESENT_NA\n"
      "time_reference_status=PRESENT_REQUIRED\n"
      "time_sync_evidence_status=INITIAL_TIME_STATUS_CAPTURE_REQUIRED\n"
      "initial_velocity_reference_required=NO\n"
      "initial_velocity_reference_topic=NONE\n"
      "initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST\n"
      "topic_count[/plc/left_track_speed]=0\n"
      "topic_count[/plc/right_track_speed]=0\n"
      "topic_count[/plc/cutting_on]=0\n"
      "topic_count[/machine/state]=0\n";
  writeFile(out + "/smoke_5s_rate1_0/reports/actual_bag_inspection.txt",
            inspection);
  writeFile(out + "/full_7s_rate1_0/reports/actual_bag_inspection.txt",
            inspection);

  ASSERT_EQ(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));
  const std::string report = readFile(
      out + "/reports/actual_bag_test_suite_manifest_validation.txt");
  EXPECT_NE(std::string::npos,
            report.find(
                "smoke_initial_velocity_initial_velocity_reference_status_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.find(
                "full_initial_velocity_initial_velocity_reference_status_status=PASS"));
}

TEST(ActualBagTestSuiteScript, ManifestValidatorRejectsNoTimeReferenceMismatch) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_suite_time_mismatch");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  const std::string out = root + "/suite";
  writeFile(bag, "not a real bag");

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + out +
      " --smoke-duration 5 --full-duration 7 --rate 1.0 --skip-bag-inspect"
      " --no-time-reference";
  ASSERT_EQ(0, std::system(command.c_str()));

  makeDir(out + "/smoke_5s_rate1_0/reports");
  makeDir(out + "/full_7s_rate1_0/reports");
  writeFile(out + "/reports/actual_bag_test_suite_summary.txt",
            "actual_bag_test_suite_status=PASS\n"
            "smoke_replay_status=PASS\n"
            "full_replay_status=PASS\n"
            "smoke_event_validation_status=PASS\n"
            "full_event_validation_status=PASS\n"
            "ros_residual_status=PASS\n"
            "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n"
            "bag_sensor_set=LIDAR_IMU_ONLY\n"
            "plc_feedback_status=NOT_PRESENT_NA\n"
            "time_reference_status=NOT_PRESENT_INITIAL_TEST\n"
            "time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST\n"
            "velocity_reference_played_to_slam=NO\n"
            "continuous_velocity_reference_used=NO\n"
            "field_acceptance_eligible=NO\n");
  writeFile(out + "/reports/actual_bag_test_suite_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(out + "/reports/actual_bag_test_suite_events.txt",
            "event=actual_bag_test_suite;actual_bag_test_suite_status=PASS;"
            "field_acceptance_eligible=NO;velocity_reference_played_to_slam=NO;"
            "continuous_velocity_reference_used=NO\n");
  writeFile(out + "/reports/ros_residual_processes.txt", "");

  const std::string good_replay_summary =
      "actual_bag_replay_status=PASS\n"
      "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n"
      "bag_sensor_set=LIDAR_IMU_ONLY\n"
      "plc_feedback_status=NOT_PRESENT_NA\n"
      "time_reference_topic=NONE\n"
      "time_reference_status=NOT_PRESENT_INITIAL_TEST\n"
      "time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST\n"
      "velocity_reference_played_to_slam=NO\n"
      "continuous_velocity_reference_used=NO\n";
  const std::string bad_replay_summary =
      "actual_bag_replay_status=PASS\n"
      "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n"
      "bag_sensor_set=LIDAR_IMU_ONLY\n"
      "plc_feedback_status=NOT_PRESENT_NA\n"
      "time_reference_topic=/time_reference\n"
      "time_reference_status=PRESENT_REQUIRED\n"
      "time_sync_evidence_status=INITIAL_TIME_STATUS_CAPTURE_REQUIRED\n"
      "velocity_reference_played_to_slam=NO\n"
      "continuous_velocity_reference_used=NO\n";
  writeFile(out + "/smoke_5s_rate1_0/reports/actual_bag_replay_summary.txt",
            bad_replay_summary);
  writeFile(out + "/full_7s_rate1_0/reports/actual_bag_replay_summary.txt",
            good_replay_summary);
  writeFile(out + "/smoke_5s_rate1_0/reports/"
                    "actual_bag_replay_hil_validation_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(out + "/full_7s_rate1_0/reports/"
                    "actual_bag_replay_hil_validation_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n");

  const std::string velocity =
      "initial_velocity_reference_status=CAPTURED\n"
      "initial_velocity_reference_policy=START_ONLY_AUDIT\n"
      "continuous_velocity_reference_used=NO\n"
      "velocity_reference_played_to_slam=NO\n";
  writeFile(out + "/smoke_5s_rate1_0/reports/initial_velocity_reference.txt",
            velocity);
  writeFile(out + "/full_7s_rate1_0/reports/initial_velocity_reference.txt",
            velocity);

  const std::string inspection =
      "actual_bag_status=PASS\n"
      "actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY\n"
      "bag_sensor_set=LIDAR_IMU_ONLY\n"
      "plc_feedback_status=NOT_PRESENT_NA\n"
      "time_reference_topic=NONE\n"
      "time_reference_status=NOT_PRESENT_INITIAL_TEST\n"
      "time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST\n"
      "topic_count[/plc/left_track_speed]=0\n"
      "topic_count[/plc/right_track_speed]=0\n"
      "topic_count[/plc/cutting_on]=0\n"
      "topic_count[/machine/state]=0\n";
  writeFile(out + "/smoke_5s_rate1_0/reports/actual_bag_inspection.txt",
            inspection);
  writeFile(out + "/full_7s_rate1_0/reports/actual_bag_inspection.txt",
            inspection);

  EXPECT_NE(0, std::system((out + "/commands/"
                                  "validate_actual_bag_test_suite.sh").c_str()));
}

TEST(ActualBagTestSuiteScript, RequiresFullDurationWhenBagInspectIsSkipped) {
  const std::string root = makeTempDir("tunnel_lio_actual_bag_suite_auto");
  ASSERT_FALSE(root.empty());
  const std::string bag = root + "/Tunnel.bag";
  writeFile(bag, "not a real bag");

  const std::string command =
      scriptPath() + " --bag " + bag + " --out " + root +
      "/suite --smoke-duration 5 --skip-bag-inspect";
  EXPECT_NE(0, std::system(command.c_str()));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
