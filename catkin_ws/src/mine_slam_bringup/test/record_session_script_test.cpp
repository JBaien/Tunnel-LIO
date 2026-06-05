#include <gtest/gtest.h>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

namespace {

bool exists(const std::string& path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

bool isDirectory(const std::string& path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

void makeDir(const std::string& path) {
  ASSERT_EQ(0, mkdir(path.c_str(), 0755));
}

void writeFile(const std::string& path, const std::string& content) {
  std::ofstream output(path.c_str());
  output << content;
}

std::string timeSyncRawPath(const std::string& session_dir) {
  return session_dir + "/logs/time_status_raw.yaml";
}

std::string timeSyncRawLine(const std::string& session_dir) {
  return "raw=" + timeSyncRawPath(session_dir) + "\n";
}

std::string ppsPtpTimeSyncRawLines(const std::string& session_dir) {
  return "time_sync_raw=" + timeSyncRawPath(session_dir) +
         "\ntime_sync_raw_status=PASS\n";
}

void writeTimeSyncRawCapture(const std::string& session_dir) {
  writeFile(timeSyncRawPath(session_dir), "diagnostic-status-raw\n");
}

std::string readFile(const std::string& path) {
  std::ifstream input(path.c_str());
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::string removeLineWithPrefix(const std::string& text,
                                 const std::string& prefix) {
  std::istringstream input(text);
  std::ostringstream output;
  std::string line;
  while (std::getline(input, line)) {
    if (line.find(prefix) == 0) {
      continue;
    }
    output << line << "\n";
  }
  return output.str();
}

void writePassingFieldAcceptanceInputs(const std::string& session_dir,
                                       bool include_runtime_health = true,
                                       bool include_section_export = true) {
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "deployment_status=PASS\n"
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_process_status=PASS\n"
            "start_command=PASS\n");
  if (include_runtime_health) {
    const std::string runtime_health_report =
        "timestamp=2026-06-03T00:00:00+08:00\n"
        "runtime_dir=/tmp/runtime\n"
        "disk_available_gb=120\n"
        "runtime_pid=1234\n"
        "systemd_active=active\n"
        "systemd_active_source=systemctl\n"
        "docker_container_status=running\n"
        "docker_container_status_source=docker_inspect\n";
    writeFile(session_dir + "/logs/runtime_health_latest.txt",
              runtime_health_report);
    writeFile(session_dir + "/logs/runtime_health.txt", runtime_health_report);
  }
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "capture_exit_status=0\n"
            "finished_at=2026-06-04T00:00:00+08:00\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "\n"
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/replay_events.txt",
            "event=session_start;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=0.0\n"
            "event=power_loss;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=4.0\n"
            "event=recovered;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=29.0\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                ppsPtpTimeSyncRawLines(session_dir) +
            "time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=" +
                session_dir + "/logs/time_sync_status.txt\n");
  if (include_section_export) {
    writeFile(session_dir + "/reports/section_export.csv",
              "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
              "test_session,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  }
}

std::string recordSessionLaunchPath() {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);
  const std::string suffix = "/scripts/record_session.sh";
  const std::size_t suffix_pos = script.rfind(suffix);
  if (suffix_pos == std::string::npos) {
    return "";
  }
  return script.substr(0, suffix_pos) + "/launch/record_session.launch";
}

}  // namespace

TEST(RecordSessionScript, DryRunCreatesReproducibleArchiveSkeleton) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_record_session_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/points_raw /tf /tf_static\" "
      "--runtime-dir /tmp/tunnel_lio_runtime/board_alpha --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = std::string(root) + "/test_session";
  EXPECT_TRUE(isDirectory(session_dir + "/bags"));
  EXPECT_TRUE(isDirectory(session_dir + "/pcap"));
  EXPECT_TRUE(isDirectory(session_dir + "/snapshots"));
  EXPECT_TRUE(isDirectory(session_dir + "/logs"));
  EXPECT_TRUE(isDirectory(session_dir + "/commands"));
  EXPECT_TRUE(isDirectory(session_dir + "/reports"));
  EXPECT_TRUE(exists(session_dir + "/metadata.env"));
  EXPECT_TRUE(exists(session_dir + "/evidence_manifest.txt"));
  EXPECT_TRUE(exists(session_dir + "/commands/record_rosbag.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/record_pcap.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/bringup_fusion_timoo_session.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/bringup_fusion_tmlidar_session.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/snapshot_ros_state.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/capture_time_sync.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/capture_pps_ptp_wiring.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/capture_runtime_health.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/capture_runtime_deployment.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/run_runtime_stability.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/capture_runtime_stability.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/capture_section_export.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/capture_power_loss_resume.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/capture_field_acceptance.sh"));
  EXPECT_TRUE(exists(session_dir + "/commands/validate_evidence.sh"));
  EXPECT_TRUE(exists(session_dir + "/snapshots/README.md"));

  const std::string metadata = readFile(session_dir + "/metadata.env");
  EXPECT_NE(std::string::npos, metadata.find("session_name=test_session"));
  EXPECT_NE(std::string::npos, metadata.find("bag_path="));
  EXPECT_NE(std::string::npos, metadata.find("pcap_path="));
  EXPECT_NE(std::string::npos, metadata.find("evidence_manifest_path="));
  EXPECT_NE(std::string::npos,
            metadata.find("runtime_dir=/tmp/tunnel_lio_runtime/board_alpha"));
  EXPECT_NE(std::string::npos, metadata.find("scenario=POWER_LOSS_ORIGIN"));
  EXPECT_NE(std::string::npos, metadata.find("topics=/points_raw /tf /tf_static"));

  const std::string evidence_manifest =
      readFile(session_dir + "/evidence_manifest.txt");
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("session_id=test_session"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("scenario=POWER_LOSS_ORIGIN"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("runtime_dir=/tmp/tunnel_lio_runtime/board_alpha"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("metrics_report=reports/validation_metrics_report.txt"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("event_file=reports/replay_events.txt"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("bag_file=bags/tunnel_lio.bag"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("pcap_file=pcap/tunnel_lio.pcap"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("tf_snapshot=snapshots/tf_monitor.txt"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("params_snapshot=snapshots/rosparams.yaml"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("runtime_log=logs/record_session.log"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("time_sync=logs/time_sync_status.txt"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("pps_ptp_wiring=reports/pps_ptp_wiring_verified.txt"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("runtime_health=logs/runtime_health_latest.txt"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("runtime_deployment=logs/runtime_deployment_check.txt"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("runtime_stability_csv=logs/runtime_stability.csv"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("runtime_stability_summary=logs/runtime_stability_summary.txt"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("runtime_stability_run_log=logs/runtime_stability_run.log"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("section_export=reports/section_export.csv"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("power_loss_resume=reports/power_loss_resume_verified.txt"));
  EXPECT_NE(std::string::npos,
            evidence_manifest.find("field_acceptance=reports/field_acceptance_report.txt"));

  const std::string record_command = readFile(session_dir + "/commands/record_rosbag.sh");
  EXPECT_NE(std::string::npos, record_command.find("rosbag record"));
  EXPECT_NE(std::string::npos, record_command.find("bags/tunnel_lio.bag"));
  EXPECT_NE(std::string::npos, record_command.find("/tf_static"));

  const std::string pcap_command = readFile(session_dir + "/commands/record_pcap.sh");
  EXPECT_NE(std::string::npos, pcap_command.find("tcpdump"));
  EXPECT_NE(std::string::npos, pcap_command.find("pcap/tunnel_lio.pcap"));

  const std::string timoo_bringup_command =
      readFile(session_dir + "/commands/bringup_fusion_timoo_session.sh");
  EXPECT_NE(std::string::npos, timoo_bringup_command.find("roslaunch"));
  EXPECT_NE(std::string::npos,
            timoo_bringup_command.find("bringup_fusion_timoo.launch"));
  EXPECT_NE(std::string::npos,
            timoo_bringup_command.find("section_session_id:=test_session"));

  const std::string tmlidar_bringup_command =
      readFile(session_dir + "/commands/bringup_fusion_tmlidar_session.sh");
  EXPECT_NE(std::string::npos, tmlidar_bringup_command.find("roslaunch"));
  EXPECT_NE(std::string::npos,
            tmlidar_bringup_command.find("bringup_fusion_tmlidar.launch"));
  EXPECT_NE(std::string::npos,
            tmlidar_bringup_command.find("section_session_id:=test_session"));

  const std::string snapshot_command =
      readFile(session_dir + "/commands/snapshot_ros_state.sh");
  EXPECT_NE(std::string::npos, snapshot_command.find("rosparam dump"));
  EXPECT_NE(std::string::npos, snapshot_command.find("tf2_tools"));

  const std::string time_sync_command =
      readFile(session_dir + "/commands/capture_time_sync.sh");
  EXPECT_NE(std::string::npos, time_sync_command.find("TIME_STATUS_TOPIC"));
  EXPECT_NE(std::string::npos, time_sync_command.find("/time/status"));
  EXPECT_NE(std::string::npos, time_sync_command.find("PPS_TOPIC"));
  EXPECT_NE(std::string::npos, time_sync_command.find("/time/pps_event"));
  EXPECT_NE(std::string::npos, time_sync_command.find("time_sync_status.txt"));
  EXPECT_NE(std::string::npos, time_sync_command.find("time_sync_status="));
  EXPECT_NE(std::string::npos, time_sync_command.find("pps_status="));
  EXPECT_NE(std::string::npos, time_sync_command.find("clock_offset_status="));

  const std::string pps_ptp_command =
      readFile(session_dir + "/commands/capture_pps_ptp_wiring.sh");
  EXPECT_NE(std::string::npos,
            pps_ptp_command.find("pps_ptp_wiring_verified.txt"));
  EXPECT_NE(std::string::npos,
            pps_ptp_command.find("PPS_PTP_WIRING_VERIFIED"));
  EXPECT_NE(std::string::npos, pps_ptp_command.find("time_sync_status.txt"));
  EXPECT_NE(std::string::npos, pps_ptp_command.find("timedatectl"));
  EXPECT_NE(std::string::npos, pps_ptp_command.find("chronyc"));

  const std::string health_command =
      readFile(session_dir + "/commands/capture_runtime_health.sh");
  EXPECT_NE(std::string::npos, health_command.find("RUNTIME_DIR"));
  EXPECT_NE(std::string::npos,
            health_command.find("/tmp/tunnel_lio_runtime/board_alpha"));
  EXPECT_NE(std::string::npos, health_command.find("runtime_health.sh"));
  EXPECT_NE(std::string::npos,
            health_command.find("logs/runtime_health_latest.txt"));

  const std::string deployment_command =
      readFile(session_dir + "/commands/capture_runtime_deployment.sh");
  EXPECT_NE(std::string::npos, deployment_command.find("RUNTIME_DIR"));
  EXPECT_NE(std::string::npos,
            deployment_command.find("/tmp/tunnel_lio_runtime/board_alpha"));
  EXPECT_NE(std::string::npos, deployment_command.find("runtime_deployment_check.sh"));
  EXPECT_NE(std::string::npos,
            deployment_command.find("logs/runtime_deployment_check.txt"));

  const std::string stability_command =
      readFile(session_dir + "/commands/run_runtime_stability.sh");
  EXPECT_NE(std::string::npos, stability_command.find("RUNTIME_DIR"));
  EXPECT_NE(std::string::npos,
            stability_command.find("/tmp/tunnel_lio_runtime/board_alpha"));
  EXPECT_NE(std::string::npos,
            stability_command.find("runtime_stability_check.sh"));
  EXPECT_NE(std::string::npos, stability_command.find("--samples"));
  EXPECT_NE(std::string::npos, stability_command.find("--interval"));
  EXPECT_NE(std::string::npos, stability_command.find("capture_runtime_stability.sh"));

  const std::string capture_stability_command =
      readFile(session_dir + "/commands/capture_runtime_stability.sh");
  EXPECT_NE(std::string::npos, capture_stability_command.find("RUNTIME_DIR"));
  EXPECT_NE(std::string::npos,
            capture_stability_command.find("/tmp/tunnel_lio_runtime/board_alpha"));
  EXPECT_NE(std::string::npos,
            capture_stability_command.find("runtime_stability.csv"));
  EXPECT_NE(std::string::npos,
            capture_stability_command.find("runtime_stability_summary.txt"));

  const std::string section_export_command =
      readFile(session_dir + "/commands/capture_section_export.sh");
  EXPECT_NE(std::string::npos, section_export_command.find("SECTION_EXPORT_SOURCE"));
  EXPECT_NE(std::string::npos, section_export_command.find("/section/export"));
  EXPECT_NE(std::string::npos, section_export_command.find("section_export.csv"));
  EXPECT_NE(std::string::npos,
            section_export_command.find("session_id,chainage_m,state_source,quality,completeness,rmse_mm,points"));

  const std::string field_acceptance_command =
      readFile(session_dir + "/commands/capture_field_acceptance.sh");
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("field_acceptance_report.txt"));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("time_sync_report="));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("runtime_deployment_status=PASS"));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("runtime_deployment_report="));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("runtime_stability_status=PASS"));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("runtime_stability_csv_report="));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("runtime_stability_summary_report="));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("runtime_stability_run_log_report="));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("power_loss_resume_report="));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("pps_ptp_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("pps_ptp_wiring_report="));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("event_file_status=PASS"));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("event_file_report="));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("metrics_status=PASS"));
  EXPECT_NE(std::string::npos,
            field_acceptance_command.find("field_acceptance_status=PASS"));

  const std::string power_loss_command =
      readFile(session_dir + "/commands/capture_power_loss_resume.sh");
  EXPECT_NE(std::string::npos,
            power_loss_command.find("power_loss_resume_verified.txt"));
  EXPECT_NE(std::string::npos,
            power_loss_command.find("power_loss_resume_confirmation.txt"));
  EXPECT_NE(std::string::npos,
            power_loss_command.find("FIELD_ACCEPTANCE_MAX_RECOVERY_TIME_S"));
  EXPECT_NE(std::string::npos,
            power_loss_command.find("power_loss_resume_source=metrics_report"));

  const std::string validate_command =
      readFile(session_dir + "/commands/validate_evidence.sh");
  EXPECT_NE(std::string::npos,
            validate_command.find("capture_field_acceptance.sh"));
  EXPECT_NE(std::string::npos,
            validate_command.find("capture_pps_ptp_wiring.sh"));
  EXPECT_NE(std::string::npos,
            validate_command.find("capture_power_loss_resume.sh"));
  EXPECT_NE(std::string::npos,
            validate_command.find("capture_time_sync.sh"));
  EXPECT_NE(std::string::npos,
            validate_command.find("capture_runtime_health.sh"));
  EXPECT_NE(std::string::npos,
            validate_command.find("capture_runtime_deployment.sh"));
  EXPECT_NE(std::string::npos,
            validate_command.find("capture_runtime_stability.sh"));
  EXPECT_NE(std::string::npos,
            validate_command.find("capture_section_export.sh"));
  EXPECT_NE(std::string::npos, validate_command.find("validation_report.launch"));
  EXPECT_NE(std::string::npos, validate_command.find("evidence_manifest_file:="));
  EXPECT_NE(std::string::npos, validate_command.find("reports/evidence_validation_report.txt"));
}

TEST(RecordSessionScript, TimeSyncCaptureRejectsMalformedRosTopics) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_time_sync_bad_topic_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  const std::string capture_command =
      "TIME_STATUS_TOPIC=/time//status \"" + session_dir +
      "/commands/capture_time_sync.sh\"";
  EXPECT_NE(0, std::system(capture_command.c_str()));

  const std::string report =
      readFile(session_dir + "/logs/time_sync_status.txt");
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_status_topic=/time//status"));
  EXPECT_NE(std::string::npos, report.find("capture_status=INVALID_METADATA"));
}

TEST(RecordSessionScript, RejectsManifestSentinelArguments) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_record_session_bad_manifest_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name __DUPLICATE_KEY__ --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio --scenario missing "
       "--topics \"/points_raw /tf\" --runtime-dir /tmp/runtime --dry-run "
       ">/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir __DUPLICATE_KEY__ --dry-run >/dev/null 2>&1").c_str()));
}

TEST(RecordSessionScript, RejectsManifestSeparatorArguments) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_bad_manifest_separator_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name \"test_session;metrics_report=evil.txt\" "
       "--prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
       "--topics \"/points_raw /tf\" --runtime-dir /tmp/runtime --dry-run "
       ">/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + std::string(root) +
       ";session_dir=evil\" --name test_session "
       "--prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
       "--topics \"/points_raw /tf\" --runtime-dir /tmp/runtime --dry-run "
       ">/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario \"POWER_LOSS_ORIGIN;metrics_report=evil.txt\" "
       "--topics \"/points_raw /tf\" --runtime-dir /tmp/runtime --dry-run "
       ">/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix \"bag;pcap_file=evil.pcap\" "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw;touch /tmp/evil\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--pcap-interface \"any;pcap_file=evil.pcap\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--time-status-topic \"/time/status;pps_status=PASS\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
      "--pps-topic \"/time/pps_event;clock_offset_status=PASS\" "
      "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
}

TEST(RecordSessionScript, RejectsCsvSeparatorRuntimeDirArgument) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_bad_runtime_csv_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/runtime,bad --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_FALSE(exists(std::string(root) + "/test_session"));
}

TEST(RecordSessionScript, RejectsManifestWhitespacePollutedArguments) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_bad_manifest_ws_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir '/tmp/runtime ' --dry-run >/dev/null 2>&1").c_str()));
}

TEST(RecordSessionScript, RejectsMalformedScenarioToken) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_bad_scenario_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario 'POWER LOSS' --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
}

TEST(RecordSessionScript, RejectsPathTraversalSessionPathArguments) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_bad_path_segment_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name ../evil_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix ../evil_prefix "
      "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
      "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
}

TEST(RecordSessionScript, RejectsRelativeSessionPaths) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_relative_path_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("cd \"" + std::string(root) + "\" && bash \"" + script +
       "\" --root relative_sessions --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
	       "--runtime-dir relative_runtime --dry-run >/dev/null 2>&1").c_str()));
}

TEST(RecordSessionScript, RejectsAbsolutePathArgumentsWithDotSegments) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_dot_absolute_path_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + std::string(root) +
       "/sessions/../escaped_sessions\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_FALSE(exists(std::string(root) + "/escaped_sessions"));

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/tunnel_lio_runtime/../escaped_runtime "
       "--dry-run >/dev/null 2>&1").c_str()));
  EXPECT_FALSE(exists(std::string(root) + "/test_session"));
}

TEST(RecordSessionScript, RejectsRootOnlyAbsolutePathArguments) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_root_absolute_path_test_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir / --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_FALSE(exists(std::string(root) + "/test_session"));
}

TEST(RecordSessionScript, RejectsShellMetacharSessionPathArguments) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_bad_shell_token_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name '$(id)' "
       "--prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
       "--topics \"/points_raw /tf\" --runtime-dir /tmp/runtime --dry-run "
       ">/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session "
       "--prefix '$(id)' "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
}

TEST(RecordSessionScript, RejectsShellMetacharGeneratedScriptArguments) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_bad_generated_shell_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw $(id)\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw 9bad\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--pcap-interface '$(id)' --runtime-dir /tmp/runtime --dry-run "
       ">/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--pcap-interface 'bad iface' --runtime-dir /tmp/runtime --dry-run "
       ">/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--time-status-topic '/time/status$(id)' "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--time-status-topic '/time/status bad' "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--pps-topic '/time/pps_event$(id)' "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--pps-topic '/time/pps event' "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
}

TEST(RecordSessionScript, RejectsGlobGeneratedScriptArguments) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_bad_generated_glob_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw *\" "
       "--runtime-dir /tmp/runtime --dry-run >/dev/null 2>&1").c_str()));
}

TEST(RecordSessionScript, RejectsMalformedRuntimeStabilityArguments) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_bad_stability_args_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario LONG_STABILITY --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/runtime "
       "--runtime-stability-samples \"2;runtime_dir=evil\" "
       "--dry-run >/dev/null 2>&1").c_str()));
  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario LONG_STABILITY --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/runtime --runtime-stability-interval fast "
       "--dry-run >/dev/null 2>&1").c_str()));
}

TEST(RecordSessionScript, RejectsMalformedStartPcapArgument) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_bad_start_pcap_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);

  EXPECT_NE(0, std::system(
      ("bash \"" + script + "\" --root \"" + root +
       "\" --name test_session --prefix tunnel_lio "
       "--scenario POWER_LOSS_ORIGIN --topics \"/points_raw /tf\" "
       "--runtime-dir /tmp/runtime --start-pcap maybe --dry-run "
       ">/dev/null 2>&1").c_str()));
}

TEST(RecordSessionScript, RuntimeStabilityCommandRunsAndArchivesEvidence) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_record_session_run_stability_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);
  const std::string runtime_dir = root_dir + "/runtime";
  makeDir(runtime_dir);
  makeDir(runtime_dir + "/commands");
  makeDir(runtime_dir + "/logs");

  const std::string fake_stability_script =
      runtime_dir + "/commands/runtime_stability_check.sh";
  writeFile(fake_stability_script,
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            "runtime_dir=$(cd \"$(dirname \"$0\")/..\" && pwd)\n"
            "echo \"$*\" > \"${runtime_dir}/logs/runtime_stability_invocation.txt\"\n"
            "cat > \"${runtime_dir}/logs/runtime_stability.csv\" <<'CSV'\n"
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n"
            "CSV\n"
            "cat > \"${runtime_dir}/logs/runtime_stability_summary.txt\" <<'SUMMARY'\n"
            "overall=PASS\n"
            "samples=1\n"
            "SUMMARY\n");
  ASSERT_EQ(0, chmod(fake_stability_script.c_str(), 0755));

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/points_raw /tf\" --runtime-dir \"" + runtime_dir +
      "\" --runtime-stability-samples 2 --runtime-stability-interval 0 --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  ASSERT_EQ(0, std::system((session_dir + "/commands/run_runtime_stability.sh").c_str()));

  const std::string invocation =
      readFile(runtime_dir + "/logs/runtime_stability_invocation.txt");
  EXPECT_NE(std::string::npos, invocation.find("--samples 2"));
  EXPECT_NE(std::string::npos, invocation.find("--interval 0"));

  const std::string archived_csv =
      readFile(session_dir + "/logs/runtime_stability.csv");
  EXPECT_NE(std::string::npos,
            archived_csv.find("sample,timestamp,disk_guard_status,watchdog_status,health_report"));
  EXPECT_NE(std::string::npos, archived_csv.find(",PASS,PASS,"));

  const std::string archived_summary =
      readFile(session_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, archived_summary.find("overall=PASS"));
  EXPECT_NE(std::string::npos, archived_summary.find("samples=1"));

  const std::string run_log =
      readFile(session_dir + "/logs/runtime_stability_run.log");
  EXPECT_NE(std::string::npos, run_log.find("runtime_dir=" + runtime_dir));
  EXPECT_NE(std::string::npos, run_log.find("samples=2"));
  EXPECT_NE(std::string::npos, run_log.find("interval=0"));
  EXPECT_NE(std::string::npos, run_log.find("exit_status=0"));
  EXPECT_NE(std::string::npos, run_log.find("capture_exit_status=0"));
}

TEST(RecordSessionScript, RuntimeStabilityCommandArchivesCsvHealthReports) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_archive_health_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);
  const std::string runtime_dir = root_dir + "/runtime";
  makeDir(runtime_dir);
  makeDir(runtime_dir + "/commands");
  makeDir(runtime_dir + "/logs");

  const std::string fake_stability_script =
      runtime_dir + "/commands/runtime_stability_check.sh";
  writeFile(fake_stability_script,
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            "runtime_dir=$(cd \"$(dirname \"$0\")/..\" && pwd)\n"
            "cat > \"${runtime_dir}/logs/runtime_health_sample.txt\" <<'HEALTH'\n"
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=${runtime_dir}\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "HEALTH\n"
            "cat > \"${runtime_dir}/logs/runtime_stability.csv\" <<CSV\n"
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,${runtime_dir}/logs/runtime_health_sample.txt\n"
            "CSV\n"
            "cat > \"${runtime_dir}/logs/runtime_stability_summary.txt\" <<'SUMMARY'\n"
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n"
            "SUMMARY\n");
  ASSERT_EQ(0, chmod(fake_stability_script.c_str(), 0755));

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/points_raw /tf\" --runtime-dir \"" + runtime_dir +
      "\" --runtime-stability-samples 1 --runtime-stability-interval 86400 "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  ASSERT_EQ(0,
            std::system(
                (session_dir + "/commands/run_runtime_stability.sh").c_str()));

  const std::string archived_csv =
      readFile(session_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(std::string::npos, archived_csv.find(runtime_dir));
  EXPECT_NE(std::string::npos, archived_csv.find("runtime_health_sample.txt"));
  EXPECT_TRUE(exists(session_dir + "/logs/runtime_health_sample.txt"));
}

TEST(RecordSessionScript, RuntimeStabilityRunLogKeepsCommandOutputSeparate) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_stability_output_log_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);
  const std::string runtime_dir = root_dir + "/runtime";
  makeDir(runtime_dir);
  makeDir(runtime_dir + "/commands");
  makeDir(runtime_dir + "/logs");

  const std::string fake_stability_script =
      runtime_dir + "/commands/runtime_stability_check.sh";
  writeFile(fake_stability_script,
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            "runtime_dir=$(cd \"$(dirname \"$0\")/..\" && pwd)\n"
            "cat > \"${runtime_dir}/logs/runtime_stability.csv\" <<'CSV'\n"
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n"
            "CSV\n"
            "cat > \"${runtime_dir}/logs/runtime_stability_summary.txt\" <<'SUMMARY'\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n"
            "SUMMARY\n"
            "echo raw_runtime_stability_stdout_without_key_value\n"
            "echo \"${runtime_dir}/logs/runtime_stability_summary.txt\"\n");
  ASSERT_EQ(0, chmod(fake_stability_script.c_str(), 0755));

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/points_raw /tf\" --runtime-dir \"" + runtime_dir +
      "\" --runtime-stability-samples 1 --runtime-stability-interval 86400 "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  ASSERT_EQ(0,
            std::system(
                (session_dir + "/commands/run_runtime_stability.sh").c_str()));

  const std::string run_log =
      readFile(session_dir + "/logs/runtime_stability_run.log");
  EXPECT_NE(std::string::npos, run_log.find("started_at="));
  EXPECT_NE(std::string::npos, run_log.find("runtime_dir=" + runtime_dir));
  EXPECT_NE(std::string::npos, run_log.find("samples=1"));
  EXPECT_NE(std::string::npos, run_log.find("interval=86400"));
  EXPECT_NE(std::string::npos, run_log.find("exit_status=0"));
  EXPECT_NE(std::string::npos, run_log.find("capture_exit_status=0"));
  EXPECT_NE(std::string::npos, run_log.find("finished_at="));
  EXPECT_EQ(std::string::npos,
            run_log.find("raw_runtime_stability_stdout_without_key_value"));

  const std::string output_log_path =
      session_dir + "/logs/runtime_stability_command_output.log";
  ASSERT_TRUE(exists(output_log_path));
  const std::string output_log = readFile(output_log_path);
  EXPECT_NE(std::string::npos,
            output_log.find("raw_runtime_stability_stdout_without_key_value"));
  EXPECT_NE(std::string::npos,
            output_log.find(runtime_dir + "/logs/runtime_stability_summary.txt"));
}

TEST(RecordSessionScript, RuntimeStabilityCaptureRequiresMatchingRunLog) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_stale_stability_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);
  const std::string runtime_dir = root_dir + "/runtime";
  makeDir(runtime_dir);
  makeDir(runtime_dir + "/logs");
  writeFile(runtime_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(runtime_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/points_raw /tf\" --runtime-dir \"" + runtime_dir +
      "\" --runtime-stability-samples 1 --runtime-stability-interval 86400 "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  EXPECT_NE(0, std::system(
                   (session_dir + "/commands/capture_runtime_stability.sh")
                       .c_str()));

  const std::string archived_csv =
      readFile(session_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(std::string::npos, archived_csv.find("runtime_health.txt"));

  const std::string archived_summary =
      readFile(session_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, archived_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos,
            archived_summary.find("capture_status=RUN_LOG_MISSING"));
}

TEST(RecordSessionScript,
     RuntimeStabilityCaptureRejectsFailedCaptureExitStatusRunLog) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_failed_capture_status_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);
  const std::string runtime_dir = root_dir + "/runtime";
  makeDir(runtime_dir);
  makeDir(runtime_dir + "/logs");
  writeFile(runtime_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(runtime_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/points_raw /tf\" --runtime-dir \"" + runtime_dir +
      "\" --runtime-stability-samples 1 --runtime-stability-interval 86400 "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=" + runtime_dir +
                "\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "capture_exit_status=1\n"
            "finished_at=2026-06-04T00:00:00+08:00\n");

  EXPECT_NE(0, std::system(
                   (session_dir + "/commands/capture_runtime_stability.sh")
                       .c_str()));

  const std::string archived_csv =
      readFile(session_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(std::string::npos, archived_csv.find("runtime_health.txt"));

  const std::string archived_summary =
      readFile(session_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, archived_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos,
            archived_summary.find("capture_status=RUN_LOG_FAILED"));
}

TEST(RecordSessionScript,
     RuntimeStabilityCaptureRejectsFinishedRunLogMissingCaptureExitStatus) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_missing_capture_status_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);
  const std::string runtime_dir = root_dir + "/runtime";
  makeDir(runtime_dir);
  makeDir(runtime_dir + "/logs");
  writeFile(runtime_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(runtime_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/points_raw /tf\" --runtime-dir \"" + runtime_dir +
      "\" --runtime-stability-samples 1 --runtime-stability-interval 86400 "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=" + runtime_dir +
                "\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "finished_at=2026-06-04T00:00:00+08:00\n");

  EXPECT_NE(0, std::system(
                   (session_dir + "/commands/capture_runtime_stability.sh")
                       .c_str()));

  const std::string archived_csv =
      readFile(session_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(std::string::npos, archived_csv.find("runtime_health.txt"));

  const std::string archived_summary =
      readFile(session_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, archived_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos,
            archived_summary.find("capture_status=RUN_LOG_FAILED"));
}

TEST(RecordSessionScript,
     RuntimeStabilityCaptureRejectsStandaloneIncompleteRunLogMissingCaptureExitStatus) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_incomplete_capture_status_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);
  const std::string runtime_dir = root_dir + "/runtime";
  makeDir(runtime_dir);
  makeDir(runtime_dir + "/logs");
  writeFile(runtime_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(runtime_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/points_raw /tf\" --runtime-dir \"" + runtime_dir +
      "\" --runtime-stability-samples 1 --runtime-stability-interval 86400 "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=" + runtime_dir +
                "\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n");

  EXPECT_NE(0, std::system(
                   (session_dir + "/commands/capture_runtime_stability.sh")
                       .c_str()));

  const std::string archived_csv =
      readFile(session_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(std::string::npos, archived_csv.find("runtime_health.txt"));

  const std::string archived_summary =
      readFile(session_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, archived_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos,
            archived_summary.find("capture_status=RUN_LOG_FAILED"));
}

TEST(RecordSessionScript,
     RuntimeStabilityCaptureRejectsStaleMarkerWithoutCaptureToken) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_stale_marker_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);
  const std::string runtime_dir = root_dir + "/runtime";
  makeDir(runtime_dir);
  makeDir(runtime_dir + "/logs");
  writeFile(runtime_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(runtime_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/points_raw /tf\" --runtime-dir \"" + runtime_dir +
      "\" --runtime-stability-samples 1 --runtime-stability-interval 86400 "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  const std::string run_log = session_dir + "/logs/runtime_stability_run.log";
  const std::string marker =
      session_dir + "/logs/runtime_stability_capture_in_progress";
  writeFile(run_log,
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=" + runtime_dir +
                "\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n");
  writeFile(marker, "run_log=" + run_log + "\n");

  EXPECT_NE(0, std::system(
                   ("RUN_RUNTIME_STABILITY_CAPTURE_MARKER=\"" + marker +
                    "\" \"" + session_dir +
                    "/commands/capture_runtime_stability.sh\"")
                       .c_str()));

  const std::string archived_csv =
      readFile(session_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(std::string::npos, archived_csv.find("runtime_health.txt"));

  const std::string archived_summary =
      readFile(session_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, archived_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos,
            archived_summary.find("capture_status=RUN_LOG_FAILED"));
}

TEST(RecordSessionScript,
     RuntimeStabilityCaptureRejectsMalformedRunLogWithValidMarker) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_malformed_run_log_marker_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);
  const std::string runtime_dir = root_dir + "/runtime";
  makeDir(runtime_dir);
  makeDir(runtime_dir + "/logs");
  writeFile(runtime_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(runtime_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/points_raw /tf\" --runtime-dir \"" + runtime_dir +
      "\" --runtime-stability-samples 1 --runtime-stability-interval 86400 "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  const std::string run_log = session_dir + "/logs/runtime_stability_run.log";
  const std::string marker =
      session_dir + "/logs/runtime_stability_capture_in_progress";
  writeFile(run_log,
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=" + runtime_dir +
                "\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "capture_token=123456789-42\n"
            "polluted run log line\n");
  writeFile(marker, "run_log=" + run_log + "\n"
                    "capture_token=123456789-42\n");

  EXPECT_NE(0, std::system(
                   ("RUN_RUNTIME_STABILITY_CAPTURE_MARKER=\"" + marker +
                    "\" \"" + session_dir +
                    "/commands/capture_runtime_stability.sh\"")
                       .c_str()));

  const std::string archived_csv =
      readFile(session_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(std::string::npos, archived_csv.find("runtime_health.txt"));

  const std::string archived_summary =
      readFile(session_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, archived_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos,
            archived_summary.find("capture_status=RUN_LOG_FAILED"));
}

TEST(RecordSessionScript,
     RuntimeStabilityCaptureRejectsMalformedMarkerWithValidToken) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_record_session_malformed_capture_marker_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);
  const std::string runtime_dir = root_dir + "/runtime";
  makeDir(runtime_dir);
  makeDir(runtime_dir + "/logs");
  writeFile(runtime_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(runtime_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/points_raw /tf\" --runtime-dir \"" + runtime_dir +
      "\" --runtime-stability-samples 1 --runtime-stability-interval 86400 "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  const std::string run_log = session_dir + "/logs/runtime_stability_run.log";
  const std::string marker =
      session_dir + "/logs/runtime_stability_capture_in_progress";
  writeFile(run_log,
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=" + runtime_dir +
                "\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "capture_token=123456789-42\n");
  writeFile(marker, "run_log=" + run_log + "\n"
                    "capture_token=123456789-42\n"
                    "polluted marker line\n");

  EXPECT_NE(0, std::system(
                   ("RUN_RUNTIME_STABILITY_CAPTURE_MARKER=\"" + marker +
                    "\" \"" + session_dir +
                    "/commands/capture_runtime_stability.sh\"")
                       .c_str()));

  const std::string archived_csv =
      readFile(session_dir + "/logs/runtime_stability.csv");
  EXPECT_EQ(std::string::npos, archived_csv.find("runtime_health.txt"));

  const std::string archived_summary =
      readFile(session_dir + "/logs/runtime_stability_summary.txt");
  EXPECT_NE(std::string::npos, archived_summary.find("overall=FAIL"));
  EXPECT_NE(std::string::npos,
            archived_summary.find("capture_status=RUN_LOG_FAILED"));
}

TEST(RecordSessionScript, SectionExportCaptureRejectsMalformedSourceEvidence) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_record_session_section_export_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario STATIC_IDLE "
      "--topics \"/points_raw /tf\" --runtime-dir /tmp/tunnel_lio_runtime/test "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string valid_source = root_dir + "/valid_section_export.csv";
  writeFile(valid_source,
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "test_session,10.000,IDLE_STATIC,A,0.930,18.000,240\n"
            "test_session,11.000,CMD_MOVE_NO_DISP,B,0.800,30.000,180\n"
            "test_session,12.000,RELOCALIZING,C,0.700,45.000,120\n");
  const std::string session_dir = root_dir + "/test_session";
  ASSERT_EQ(0,
            std::system(("SECTION_EXPORT_SOURCE=\"" + valid_source + "\" \"" +
                         session_dir + "/commands/capture_section_export.sh\"")
                            .c_str()));
  EXPECT_EQ(readFile(valid_source),
            readFile(session_dir + "/reports/section_export.csv"));

  const std::string malformed_source =
      root_dir + "/malformed_section_export.csv";
  writeFile(malformed_source,
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "test_session,10.000,IDLE_STATIC,A,0.930,18.000,240\n"
            "test_session,not_numeric,IDLE_STATIC,A,complete,rmse,points\n");

  EXPECT_NE(0,
            std::system(("SECTION_EXPORT_SOURCE=\"" + malformed_source + "\" \"" +
                         session_dir + "/commands/capture_section_export.sh\"")
                            .c_str()));

  const std::string malformed_quality_source =
      root_dir + "/malformed_quality_section_export.csv";
  writeFile(malformed_quality_source,
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "test_session,10.000,IDLE_STATIC,Z,0.930,18.000,240\n");

  EXPECT_NE(0,
            std::system(("SECTION_EXPORT_SOURCE=\"" +
                         malformed_quality_source + "\" \"" + session_dir +
                         "/commands/capture_section_export.sh\"")
                            .c_str()));

  const std::string malformed_state_source =
      root_dir + "/malformed_state_section_export.csv";
  writeFile(malformed_state_source,
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "test_session,10.000,UNKNOWN_STATE,A,0.930,18.000,240\n");

  EXPECT_NE(0,
            std::system(("SECTION_EXPORT_SOURCE=\"" +
                         malformed_state_source + "\" \"" + session_dir +
                         "/commands/capture_section_export.sh\"")
                            .c_str()));

  const std::string mismatched_session_source =
      root_dir + "/mismatched_session_section_export.csv";
  writeFile(mismatched_session_source,
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "other_session,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  EXPECT_NE(0,
            std::system(("SECTION_EXPORT_SOURCE=\"" +
                         mismatched_session_source + "\" \"" + session_dir +
                         "/commands/capture_section_export.sh\"")
                            .c_str()));
}

TEST(RecordSessionScript, SectionExportCaptureRejectsWhitespacePollutedSourceFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_record_session_section_export_ws_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario STATIC_IDLE "
      "--topics \"/points_raw /tf\" --runtime-dir /tmp/tunnel_lio_runtime/test "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  const std::string polluted_source =
      root_dir + "/polluted_section_export.csv";
  writeFile(polluted_source,
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "test_session ,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  EXPECT_NE(0,
            std::system(("SECTION_EXPORT_SOURCE=\"" + polluted_source + "\" \"" +
                         session_dir + "/commands/capture_section_export.sh\"")
                            .c_str()));
}

TEST(RecordSessionScript, SectionExportCaptureRejectsMalformedServiceOutput) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_record_session_section_service_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario STATIC_IDLE "
      "--topics \"/points_raw /tf\" --runtime-dir /tmp/tunnel_lio_runtime/test "
      "--dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string fake_bin = root_dir + "/fake_bin";
  makeDir(fake_bin);
  const std::string fake_rosservice = fake_bin + "/rosservice";
  writeFile(fake_rosservice,
            "#!/usr/bin/env bash\n"
            "set -euo pipefail\n"
            "cat > \"${FAKE_SECTION_EXPORT_TARGET}\" <<'CSV'\n"
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "other_session,10.000,IDLE_STATIC,A,0.930,18.000,240\n"
            "CSV\n"
            "exit 0\n");
  ASSERT_EQ(0, chmod(fake_rosservice.c_str(), 0755));

  const std::string session_dir = root_dir + "/test_session";
  const std::string capture_command =
      "PATH=\"" + fake_bin + ":$PATH\" FAKE_SECTION_EXPORT_TARGET=\"" +
      session_dir + "/reports/section_export.csv\" \"" + session_dir +
      "/commands/capture_section_export.sh\"";

  EXPECT_NE(0, std::system(capture_command.c_str()));
}

TEST(RecordSessionScript, TimeSyncCaptureRequiresOkLevelInMatchingBlocks) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_time_sync_capture_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  const std::string source_yaml = root_dir + "/time_status.yaml";
  writeFile(source_yaml,
            "status:\n"
            "  -\n"
            "    level: 1\n"
            "    name: \"lio_time_manager pps: /time/pps_event\"\n"
            "    values:\n"
            "      -\n"
            "        key: \"interval_jitter_ms\"\n"
            "        value: \"9.9\"\n"
            "  -\n"
            "    level: 0\n"
            "    name: \"unrelated ok diagnostic\"\n"
            "  -\n"
            "    level: 1\n"
            "    name: \"lio_time_manager clock: /time/pps_event\"\n"
            "    values:\n"
            "      -\n"
            "        key: \"mean_offset_ms\"\n"
            "        value: \"8.8\"\n");

  const std::string capture_command =
      "TIME_SYNC_SOURCE=\"" + source_yaml + "\" \"" + session_dir +
      "/commands/capture_time_sync.sh\"";
  ASSERT_EQ(0, std::system(capture_command.c_str()));

  const std::string report =
      readFile(session_dir + "/logs/time_sync_status.txt");
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("capture_status=CAPTURED"));
  EXPECT_NE(std::string::npos, report.find("pps_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("clock_offset_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_jitter_ms=9.9"));
  EXPECT_NE(std::string::npos, report.find("mean_offset_ms=8.8"));
}

TEST(RecordSessionScript, TimeSyncCaptureRejectsMalformedNumbers) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_time_sync_bad_numbers_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  const std::string source_yaml = root_dir + "/time_status.yaml";
  writeFile(source_yaml,
            "status:\n"
            "  -\n"
            "    level: 0\n"
            "    name: \"lio_time_manager pps: /time/pps_event\"\n"
            "    values:\n"
            "      -\n"
            "        key: \"interval_jitter_ms\"\n"
            "        value: \"not_numeric\"\n"
            "  -\n"
            "    level: 0\n"
            "    name: \"lio_time_manager clock: /time/pps_event\"\n"
            "    values:\n"
            "      -\n"
            "        key: \"mean_offset_ms\"\n"
            "        value: \"also_bad\"\n");

  const std::string capture_command =
      "TIME_SYNC_SOURCE=\"" + source_yaml + "\" \"" + session_dir +
      "/commands/capture_time_sync.sh\" >/dev/null 2>&1";
  EXPECT_NE(0, std::system(capture_command.c_str()));

  const std::string report =
      readFile(session_dir + "/logs/time_sync_status.txt");
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("capture_status=CAPTURED"));
  EXPECT_NE(std::string::npos, report.find("pps_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("clock_offset_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_jitter_ms=not_numeric"));
  EXPECT_NE(std::string::npos, report.find("mean_offset_ms=also_bad"));
}

TEST(RecordSessionScript, TimeSyncCaptureAcceptsScientificNotationNumbers) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_time_sync_scientific_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  const std::string source_yaml = root_dir + "/time_status.yaml";
  writeFile(source_yaml,
            "status:\n"
            "  -\n"
            "    level: 0\n"
            "    name: \"lio_time_manager pps: /time/pps_event\"\n"
            "    values:\n"
            "      -\n"
            "        key: \"interval_jitter_ms\"\n"
            "        value: \"1e-3\"\n"
            "  -\n"
            "    level: 0\n"
            "    name: \"lio_time_manager clock: /time/pps_event\"\n"
            "    values:\n"
            "      -\n"
            "        key: \"mean_offset_ms\"\n"
            "        value: \"-2.5e-1\"\n");

  const std::string capture_command =
      "TIME_SYNC_SOURCE=\"" + source_yaml + "\" \"" + session_dir +
      "/commands/capture_time_sync.sh\"";
  EXPECT_EQ(0, std::system(capture_command.c_str()));

  const std::string report =
      readFile(session_dir + "/logs/time_sync_status.txt");
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("capture_status=CAPTURED"));
  EXPECT_NE(std::string::npos, report.find("pps_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("clock_offset_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("pps_jitter_ms=1e-3"));
  EXPECT_NE(std::string::npos, report.find("mean_offset_ms=-2.5e-1"));
}

TEST(RecordSessionScript, TimeSyncCaptureRejectsSeparatorTopicOverrides) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_time_sync_bad_topic_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  const std::string source_yaml = root_dir + "/time_status.yaml";
  writeFile(source_yaml,
            "status:\n"
            "  -\n"
            "    level: 0\n"
            "    name: \"lio_time_manager pps: /time/pps_event\"\n"
            "    values:\n"
            "      -\n"
            "        key: \"interval_jitter_ms\"\n"
            "        value: \"0.4\"\n"
            "  -\n"
            "    level: 0\n"
            "    name: \"lio_time_manager clock: /time/pps_event\"\n"
            "    values:\n"
            "      -\n"
            "        key: \"mean_offset_ms\"\n"
            "        value: \"0.8\"\n");

  const std::string capture_command =
      "TIME_STATUS_TOPIC=\"/time/status;pps_status=PASS\" "
      "TIME_SYNC_SOURCE=\"" +
      source_yaml + "\" \"" + session_dir +
      "/commands/capture_time_sync.sh\" >/dev/null 2>&1";
  EXPECT_NE(0, std::system(capture_command.c_str()));

  const std::string report =
      readFile(session_dir + "/logs/time_sync_status.txt");
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("time_status_topic=/time/status;pps_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("pps_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("clock_offset_status=FAIL"));
}

TEST(RecordSessionScript, PpsPtpWiringCaptureRequiresTimeSyncAndManualConfirmation) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));

  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  const std::string capture_command =
      "\"" + session_dir + "/commands/capture_pps_ptp_wiring.sh\"";
  ASSERT_EQ(0, std::system(capture_command.c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("pps_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("clock_offset_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation_source=manual_file"));
  EXPECT_NE(std::string::npos, report.find("pps_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("ptp_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_verified_by=qa_operator"));
  EXPECT_NE(std::string::npos,
            report.find("wiring_verified_at=2026-06-03T00:00:00+08:00"));
  EXPECT_NE(std::string::npos, report.find("pps_jitter_ms=0.4"));
  EXPECT_NE(std::string::npos, report.find("mean_offset_ms=0.8"));
  EXPECT_NE(std::string::npos,
            report.find("time_sync_timestamp=2026-06-03T00:00:00+08:00"));
  EXPECT_NE(std::string::npos,
            report.find("time_sync_raw=" + timeSyncRawPath(session_dir)));
  EXPECT_NE(std::string::npos, report.find("time_sync_raw_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("ptp_status="));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsTimeSyncMissingTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_missing_time_ts_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_timestamp=missing"));
}

TEST(RecordSessionScript, PpsPtpWiringAcceptsScientificNotationTimeSyncNumbers) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_scientific_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=1e-3\n"
            "mean_offset_ms=-2.5e-1\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_EQ(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("pps_jitter_ms=1e-3"));
  EXPECT_NE(std::string::npos, report.find("mean_offset_ms=-2.5e-1"));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsMalformedManualConfirmationOverall) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_bad_overall_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));

  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS_BAD\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=FAIL"));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsMalformedTimeSyncNumbers) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_bad_time_numbers_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=not_numeric\n"
            "mean_offset_ms=also_bad\n");

  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_jitter_ms=not_numeric"));
  EXPECT_NE(std::string::npos, report.find("mean_offset_ms=also_bad"));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsMalformedTimeSyncTopics) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_bad_topic_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time//status\n"
            "pps_topic=/time/1pps\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));

  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_status_topic=/time//status"));
  EXPECT_NE(std::string::npos, report.find("pps_topic=/time/1pps"));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsTimeSyncMissingRawCapture) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_missing_raw_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_raw_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_raw=missing"));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsTimeSyncUnrelatedDuplicateKey) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_time_duplicate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "operator=qa\n"
            "operator=qa\n");

  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_jitter_ms=0.4"));
  EXPECT_NE(std::string::npos, report.find("mean_offset_ms=0.8"));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsDuplicateManualConfirmationOverall) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_duplicate_overall_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));

  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_ptp_wiring_verified=FAIL\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("wiring_confirmation_overall=__DUPLICATE_KEY__"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=FAIL"));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsManualConfirmationWithoutAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_missing_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation_source=manual_file"));
  EXPECT_NE(std::string::npos, report.find("pps_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("ptp_wiring_verified=missing"));
  EXPECT_NE(std::string::npos, report.find("wiring_verified_by=missing"));
  EXPECT_NE(std::string::npos, report.find("wiring_verified_at=missing"));
}

TEST(RecordSessionScript,
     PpsPtpWiringRejectsManualConfirmationUnrelatedDuplicateKey) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_pps_ptp_wiring_duplicate_manual_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));

  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "operator=qa\n"
            "operator=qa\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=FAIL"));
}

TEST(RecordSessionScript,
     PpsPtpWiringRejectsManualConfirmationMalformedKeyValueLine) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_pps_ptp_wiring_malformed_manual_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));

  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "not_a_key_value_pair\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("wiring_confirmation_keys_status=FAIL"));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsManualConfirmationWhitespaceAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_ws_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator \n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_verified_by=qa_operator "));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsManualConfirmationNonIsoAuditTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_noniso_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=not-a-time\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_verified_at=not-a-time"));
}

TEST(RecordSessionScript,
     PpsPtpWiringRejectsManualConfirmationImpossibleCalendarAuditTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_calendar_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-02-31T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("wiring_verified_at=2026-02-31T00:00:00+08:00"));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsManualConfirmationSeparatorAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_separator_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator;pps_ptp_wiring_verified=PASS\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("wiring_verified_by=qa_operator;pps_ptp_wiring_verified"));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsManualConfirmationCarriageReturnAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_cr_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/reports/pps_ptp_wiring_confirmation.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\r\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_pps_ptp_wiring.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=FAIL"));
}

TEST(RecordSessionScript, PpsPtpWiringRejectsEnvOverrideConfirmation) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_pps_ptp_wiring_env_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));

  const std::string capture_command =
      "PPS_PTP_WIRING_VERIFIED=PASS \"" + session_dir +
      "/commands/capture_pps_ptp_wiring.sh\"";
  EXPECT_NE(0, std::system(capture_command.c_str()));

  const std::string report =
      readFile(session_dir + "/reports/pps_ptp_wiring_verified.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation_source=manual_env"));
}

TEST(RecordSessionScript, FieldAcceptanceDerivesPowerLossResumeFromMetrics) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" "
      "--runtime-dir /tmp/tunnel_lio_runtime/test_session --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "deployment_status=PASS\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/test_session\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_process_status=PASS\n"
            "start_command=PASS\n");
  const std::string runtime_health_report =
      "timestamp=2026-06-03T00:00:00+08:00\n"
      "runtime_dir=/tmp/tunnel_lio_runtime/test_session\n"
      "disk_available_gb=120\n"
      "runtime_pid=1234\n"
      "systemd_active=active\n"
      "systemd_active_source=systemctl\n"
      "docker_container_status=running\n"
      "docker_container_status_source=docker_inspect\n";
  writeFile(session_dir + "/logs/runtime_health_latest.txt",
            runtime_health_report);
  writeFile(session_dir + "/logs/runtime_health.txt", runtime_health_report);
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/test_session\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "capture_exit_status=0\n"
            "finished_at=2026-06-04T00:00:00+08:00\n");
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "\n"
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/replay_events.txt",
            "event=session_start;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=0.0\n"
            "event=power_loss;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=4.0\n"
            "event=recovered;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=29.0\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                ppsPtpTimeSyncRawLines(session_dir) +
            "time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=" +
                session_dir + "/logs/time_sync_status.txt\n");
  writeFile(session_dir + "/reports/section_export.csv",
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "test_session,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  ASSERT_EQ(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("session_id=test_session"));
  EXPECT_NE(std::string::npos, report.find("scenario=POWER_LOSS_ORIGIN"));
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.find("event_file_report=" + session_dir +
                        "/reports/replay_events.txt"));
  EXPECT_NE(std::string::npos, report.find("metrics_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("\ndeployment_status=PASS\n"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_deployment_report=" + session_dir +
                        "/logs/runtime_deployment_check.txt"));
  EXPECT_NE(std::string::npos,
            report.find("time_sync_report=" + session_dir +
                        "/logs/time_sync_status.txt"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_report=" + session_dir +
                        "/reports/pps_ptp_wiring_verified.txt"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_report=" + session_dir +
                        "/reports/power_loss_resume_verified.txt"));
  EXPECT_NE(std::string::npos,
            report.find("section_export_report=" + session_dir +
                        "/reports/section_export.csv"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_health_report=" + session_dir +
                        "/logs/runtime_health_latest.txt"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_csv_report=" + session_dir +
                        "/logs/runtime_stability.csv"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_summary_report=" + session_dir +
                        "/logs/runtime_stability_summary.txt"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_report=" + session_dir +
                        "/logs/runtime_stability_run.log"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
  EXPECT_NE(std::string::npos, report.find("pps_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("ptp_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_verified_by=qa_operator"));
  EXPECT_NE(std::string::npos,
            report.find("wiring_verified_at=2026-06-03T00:00:00+08:00"));
}

TEST(RecordSessionScript, FieldAcceptanceAcceptsReplayEventComments) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_event_comments_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/replay_events.txt",
            "# Normalized replay/HIL validation events.\n"
            "\n"
            "  # comments may be indented\n"
            "event=session_start;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=0.0\n"
            "event=static_sample;session_id=test_session;"
            "scenario=POWER_LOSS_ORIGIN;t=1.0;static_drift_m=0.01\n");

  ASSERT_EQ(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.find("event_file_report=" + session_dir +
                        "/reports/replay_events.txt"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsReplayEventFileWithoutMetricEvidence) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_empty_event_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/replay_events.txt",
            "event=session_start;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=0.0\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("event_file_report=" + session_dir +
                        "/reports/replay_events.txt"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsReplayEventFileWithMalformedMetric) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_bad_event_metric_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/replay_events.txt",
            "event=session_start;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=0.0\n"
            "event=static_sample;session_id=test_session;"
            "scenario=POWER_LOSS_ORIGIN;t=1.0;static_drift_m=bad\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("event_file_report=" + session_dir +
                        "/reports/replay_events.txt"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsReplayEventFileWithNegativeCounters) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_negative_event_metric_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/replay_events.txt",
            "event=session_start;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=0.0\n"
            "event=static_sample;session_id=test_session;"
            "scenario=POWER_LOSS_ORIGIN;t=1.0;static_drift_m=-0.01\n"
            "event=length_sample;session_id=test_session;"
            "scenario=POWER_LOSS_ORIGIN;t=2.0;length_error_percent=-0.1\n"
            "event=loop_verified;session_id=test_session;"
            "scenario=POWER_LOSS_ORIGIN;t=3.0;wrong_loop=-1\n"
            "event=queue_sample;session_id=test_session;"
            "scenario=POWER_LOSS_ORIGIN;t=4.0;queue_backlog=-5\n"
            "event=pps_sample;session_id=test_session;"
            "scenario=POWER_LOSS_ORIGIN;t=5.0;pps_jitter_ms=-0.5\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("event_file_report=" + session_dir +
                        "/reports/replay_events.txt"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsReplayEventFileWithNegativeTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_negative_event_time_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/replay_events.txt",
            "event=session_start;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=0.0\n"
            "event=static_sample;session_id=test_session;"
            "scenario=POWER_LOSS_ORIGIN;t=-1.0;static_drift_m=0.01\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("event_file_report=" + session_dir +
                        "/reports/replay_events.txt"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsReplayEventFileWithInvalidUnrelatedTokens) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_bad_unrelated_event_token_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/replay_events.txt",
            "event=session_start;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=0.0\n"
            "event=power_loss;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=4.0\n"
            "event=recovered;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=29.0\n"
            "event=static_sample;session_id=../bad;scenario=OTHER_SCENARIO;t=1.0;static_drift_m=0.01\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("event_file_report=" + session_dir +
                        "/reports/replay_events.txt"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsReplayEventFileOverValidationThreshold) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_event_threshold_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/replay_events.txt",
            "event=session_start;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=0.0\n"
            "event=static_sample;session_id=test_session;"
            "scenario=POWER_LOSS_ORIGIN;t=1.0;static_drift_m=0.06\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("event_file_report=" + session_dir +
                        "/reports/replay_events.txt"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMissingTimeSyncTopicEvidence) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_missing_time_topics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_status_topic=missing"));
  EXPECT_NE(std::string::npos, report.find("pps_topic=missing"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMalformedTimeSyncTopics) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_bad_time_topics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time//status\n"
            "pps_topic=/time/1pps\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_status_topic=/time//status"));
  EXPECT_NE(std::string::npos, report.find("pps_topic=/time/1pps"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsTimeSyncUnrelatedDuplicateKey) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_time_duplicate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "operator=qa\n"
            "operator=qa\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_jitter_ms=0.4"));
  EXPECT_NE(std::string::npos, report.find("mean_offset_ms=0.8"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsTimeSyncMalformedKeyValueLine) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_time_malformed_key_value_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  const std::string time_sync_report =
      session_dir + "/logs/time_sync_status.txt";
  writeFile(time_sync_report, readFile(time_sync_report) +
                                  "not_a_key_value_pair\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_keys_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsTimeSyncRawCaptureOutsideSession) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_time_raw_escape_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);
  char external_template[] =
      "/tmp/tunnel_lio_field_acceptance_external_raw_XXXXXX";
  char* external = mkdtemp(external_template);
  ASSERT_NE(nullptr, external);
  const std::string external_raw =
      std::string(external) + "/time_status_raw.yaml";

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(external_raw, "diagnostic-status-raw\n");
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "raw=" +
                external_raw + "\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_raw_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_raw=" + external_raw));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringTimeSyncTopicMismatch) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_pps_topic_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "pps_ptp_wiring_verified=PASS\n"
            "time_status_topic=/other/status\n"
            "pps_topic=/other/pps_event\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_time_status_topic=/other/status"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_pps_topic=/other/pps_event"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringTimeSyncNumberMismatch) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_pps_number_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "pps_ptp_wiring_verified=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_jitter_ms=9.9\n"
            "mean_offset_ms=8.8\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_pps_jitter_ms=9.9"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_mean_offset_ms=8.8"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringTimeSyncStatusMismatch) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_pps_status_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=FAIL\n"
            "capture_status=UNAVAILABLE\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=FAIL\n"
            "clock_offset_status=FAIL\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_capture_status=UNAVAILABLE"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_pps_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_clock_offset_status=FAIL"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsPpsPtpWiringMissingTimeSyncTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_pps_missing_time_ts_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" "
      "--runtime-dir /tmp/tunnel_lio_runtime/test_session --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  const std::string pps_report =
      session_dir + "/reports/pps_ptp_wiring_verified.txt";
  writeFile(pps_report,
            removeLineWithPrefix(readFile(pps_report), "time_sync_timestamp="));

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_time_sync_timestamp=missing"));
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringMissingTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_pps_missing_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" "
      "--runtime-dir /tmp/tunnel_lio_runtime/test_session --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  const std::string pps_report =
      session_dir + "/reports/pps_ptp_wiring_verified.txt";
  writeFile(pps_report,
            removeLineWithPrefix(readFile(pps_report), "timestamp="));

  EXPECT_NE(0, std::system(
                   (session_dir + "/commands/capture_field_acceptance.sh")
                       .c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_timestamp=missing"));
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringTimeSyncReportMismatch) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_pps_report_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=/tmp/other_time_sync_status.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("pps_ptp_wiring_time_sync_report=/tmp/other_time_sync_status.txt"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringMissingConfirmationOverall) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_pps_missing_confirmation_overall_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=" +
                session_dir + "/logs/time_sync_status.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation_overall=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringConfirmationOverallFail) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_pps_confirmation_overall_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "wiring_confirmation_overall=FAIL\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=" +
                session_dir + "/logs/time_sync_status.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation_overall=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringMissingConfirmationStatus) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_pps_missing_confirmation_status_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=" +
                session_dir + "/logs/time_sync_status.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringConfirmationStatusFail) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_pps_confirmation_status_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "wiring_confirmation=FAIL\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=" +
                session_dir + "/logs/time_sync_status.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringMissingConfirmationKeysStatus) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_pps_missing_confirmation_keys_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=" +
                session_dir + "/logs/time_sync_status.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation_keys_status=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringConfirmationKeysStatusFail) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_pps_confirmation_keys_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=FAIL\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=" +
                session_dir + "/logs/time_sync_status.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation_keys_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringUnrelatedDuplicateKey) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_pps_duplicate_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=" +
                session_dir + "/logs/time_sync_status.txt\n"
            "operator=qa\n"
            "operator=qa\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringEmptyKeyLine) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_pps_empty_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  const std::string pps_ptp_wiring_report =
      session_dir + "/reports/pps_ptp_wiring_verified.txt";
  writeFile(pps_ptp_wiring_report,
            readFile(pps_ptp_wiring_report) + "=orphan_value\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_keys_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMissingRuntimeHealthEvidence) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_missing_health_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" "
      "--runtime-dir /tmp/tunnel_lio_runtime/test_session --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir, false, true);

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsDuplicateRuntimeHealthKeys) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_duplicate_health_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir, true, true);
  writeFile(session_dir + "/logs/runtime_health_latest.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "runtime_pid=missing\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeHealthUnrelatedDuplicateKey) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_health_duplicate_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir, true, true);
  writeFile(session_dir + "/logs/runtime_health_latest.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "docker_container_status=running\n"
            "operator=qa\n"
            "operator=qa\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsSentinelRuntimeHealthTextFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_sentinel_health_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir, true, true);
  writeFile(session_dir + "/logs/runtime_health_latest.txt",
            "runtime_dir=missing\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=FAIL"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsInactiveRuntimeHealthProcessState) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_inactive_health_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir, true, true);
  writeFile(session_dir + "/logs/runtime_health_latest.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=inactive\n"
            "docker_container_status=exited\n");

  EXPECT_NE(0, std::system(
                   (session_dir + "/commands/capture_field_acceptance.sh")
                       .c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_systemd_active=inactive"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_health_docker_container_status=exited"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsWhitespaceDuplicateRuntimeStabilitySummaryKeys) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_spaced_stability_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            " samples=2\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_samples=__DUPLICATE_KEY__"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilitySummaryUnrelatedDuplicateKey) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_stability_summary_duplicate_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n"
            "operator=qa\n"
            "operator=qa\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityRunLogUnrelatedDuplicateKey) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_stability_run_log_duplicate_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "capture_exit_status=0\n"
            "finished_at=2026-06-04T00:00:00+08:00\n"
            "operator=qa\n"
            "operator=qa\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMissingSectionExportEvidence) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_missing_section_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir, true, false);

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("section_export_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsWhitespacePollutedSectionExportFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_section_ws_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/section_export.csv",
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "test_session, 10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("section_export_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMissingRuntimeStabilityCsvEvidence) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_missing_stability_csv_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability.csv", "");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_samples=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMissingRuntimeStabilityRunLog) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_missing_stability_run_log_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  ASSERT_EQ(0, unlink((session_dir + "/logs/runtime_stability_run.log").c_str()));

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_status=FAIL"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsEnvLoweredStabilityDurationBelow24h) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_low_duration_override_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=2\n"
            "interval_s=3600\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n"
            "2,2026-06-02T01:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "samples=2\n"
            "interval=3600\n"
            "exit_status=0\n"
            "capture_exit_status=0\n"
            "finished_at=2026-06-03T02:00:00+08:00\n");

  const std::string capture_command =
      "FIELD_ACCEPTANCE_MIN_STABILITY_HOURS=1 \"" + session_dir +
      "/commands/capture_field_acceptance.sh\"";
  EXPECT_NE(0, std::system(capture_command.c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_min_duration_h=1"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_min_duration_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_duration_h=2.00"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeStabilitySampleCountMismatch) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_sample_count_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_process_status=PASS\n"
            "start_command=PASS\n");
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n"
            "2,2026-06-03T00:01:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(session_dir + "/logs/runtime_health.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_samples=2"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_sample_count_match=FAIL"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityCsvTimestampRegression) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_csv_time_regression_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "timestamp=2026-06-03T23:59:59+08:00\n"
            "overall=PASS\n"
            "samples=2\n"
            "interval_s=43200\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T12:00:00+08:00,PASS,PASS,runtime_health_1.txt\n"
            "2,2026-06-03T06:00:00+08:00,PASS,PASS,runtime_health_2.txt\n");
  writeFile(session_dir + "/logs/runtime_health_1.txt",
            "timestamp=2026-06-03T12:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_health_2.txt",
            "timestamp=2026-06-03T06:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "samples=2\n"
            "interval=43200\n"
            "exit_status=0\n"
            "capture_exit_status=0\n"
            "finished_at=2026-06-04T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_csv_first_timestamp=2026-06-03T12:00:00+08:00"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_csv_last_timestamp=2026-06-03T06:00:00+08:00"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityCsvMissingHealthReportFile) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_csv_missing_health_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,missing_runtime_health.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_status=FAIL"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityCsvInvalidHealthReportContent) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_csv_bad_health_content_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_health.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/other_runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeStabilityCsvFailure) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_csv_failure_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_process_status=PASS\n"
            "start_command=PASS\n");
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,FAIL,runtime_health.txt\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_sample_count_match=PASS"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeStabilityCsvWhitespacePollution) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_csv_ws_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS ,PASS,runtime_health.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeStabilityCsvSentinelHealthReport) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_csv_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,__DUPLICATE_KEY__\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeStabilityCsvSeparatorHealthReport) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_csv_separator_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt;field_acceptance_status=PASS\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_status=FAIL"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityCsvImpossibleCalendarTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_csv_calendar_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-02-31T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_status=FAIL"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityCsvNonSequentialSampleIndex) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_csv_sample_index_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "2,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_status=FAIL"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityCsvTimestampBeforeRunLogWindow) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_csv_window_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-02T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_csv_status=FAIL"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsFinalReportTimestampBeforeEvidenceTimes) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_report_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  makeDir(session_dir + "/fake_bin");
  writeFile(session_dir + "/fake_bin/date",
            "#!/usr/bin/env bash\n"
            "if [[ \"$1\" == \"--iso-8601=seconds\" ]]; then\n"
            "  echo \"2026-06-03T23:59:59+08:00\"\n"
            "  exit 0\n"
            "fi\n"
            "exec /bin/date \"$@\"\n");
  ASSERT_EQ(0, chmod((session_dir + "/fake_bin/date").c_str(), 0755));

  const std::string capture_command =
      "PATH=\"" + session_dir + "/fake_bin:$PATH\" \"" + session_dir +
      "/commands/capture_field_acceptance.sh\"";
  EXPECT_NE(0, std::system(capture_command.c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("timestamp=2026-06-03T23:59:59+08:00"));
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("field_acceptance_timestamp_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringMissingAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_pps_missing_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_health_latest.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "\n"
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation_source=manual_file"));
  EXPECT_NE(std::string::npos, report.find("pps_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("ptp_wiring_verified=missing"));
  EXPECT_NE(std::string::npos, report.find("wiring_verified_by=missing"));
  EXPECT_NE(std::string::npos, report.find("wiring_verified_at=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringSentinelAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_pps_sentinel_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=__DUPLICATE_KEY__\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_verified_by=__DUPLICATE_KEY__"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringNonIsoAuditTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_pps_noniso_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=not-a-time\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_verified_at=not-a-time"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsPpsPtpWiringImpossibleCalendarAuditTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_pps_calendar_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-02-31T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("wiring_verified_at=2026-02-31T00:00:00+08:00"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPpsPtpWiringCarriageReturnAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_pps_cr_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\r\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMalformedPpsPtpWiringOverall) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_bad_pps_overall_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_health_latest.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS_BAD\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMalformedTimeSyncStatus) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_bad_time_status_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS_BAD\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMalformedDeploymentOverall) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_bad_deploy_overall_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario RUNTIME_DEPLOYMENT "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS_BAD\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_deployment_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeDeploymentUnrelatedDuplicateKey) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_deploy_duplicate_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_process_status=PASS\n"
            "start_command=PASS\n"
            "operator=qa\n"
            "operator=qa\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_deployment_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeDeploymentMissingTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_deploy_missing_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_process_status=PASS\n"
            "start_command=PASS\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_deployment_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_deployment_timestamp=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsTimeSyncMissingTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_time_sync_missing_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_timestamp=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsDeploymentWithoutRuntimeSkeleton) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_deploy_skeleton_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_deployment_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMalformedTimeSyncNumbers) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_bad_time_sync_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario TIME_SYNC "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=not_numeric\n"
            "mean_offset_ms=also_bad\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeStabilityFailureCounters) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_stability_failures_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n"
            "disk_failures=1\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_disk_failures=1"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_watchdog_failures=0"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_health_failures=0"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeStabilityWatchdogSkipped) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_stability_watchdog_skipped_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=1\n"
            "health_failures=0\n");

  EXPECT_NE(0,
            std::system((session_dir +
                         "/commands/capture_field_acceptance.sh")
                            .c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_watchdog_skipped=1"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityZeroCountersWithLeadingZeros) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_leading_zero_stability_failures_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=00\n"
            "watchdog_failures=000\n"
            "health_failures=00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_deployment_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("section_export_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("metrics_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_disk_failures=00"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_watchdog_failures=000"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_health_failures=00"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityRunLogExitStatusWithLeadingZeros) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_leading_zero_run_status_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=00\n"
            "capture_exit_status=0\n"
            "finished_at=2026-06-04T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_deployment_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("section_export_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("metrics_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_exit_status=00"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityRunLogCaptureExitStatusNonzero) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_capture_exit_status_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "capture_exit_status=1\n"
            "finished_at=2026-06-04T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_deployment_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("section_export_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("metrics_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_exit_status=0"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_capture_exit_status=1"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeStabilityRunLogImpossibleCalendarDate) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_calendar_run_log_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-02-31T00:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "capture_exit_status=0\n"
            "finished_at=2026-06-04T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_started_at=2026-02-31T00:00:00+08:00"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityRunLogFinishedBeforeStartedAt) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_reversed_run_log_time_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:02+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "capture_exit_status=0\n"
            "finished_at=2026-06-03T00:00:01+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_started_at=2026-06-03T00:00:02+08:00"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_finished_at=2026-06-03T00:00:01+08:00"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilityRunLogElapsedTooShort) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_short_run_log_elapsed_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "capture_exit_status=0\n"
            "finished_at=2026-06-03T00:00:01+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_run_log_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_elapsed_s=1"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_required_elapsed_s=86340"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_duration_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_started_at=2026-06-03T00:00:00+08:00"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_run_log_finished_at=2026-06-03T00:00:01+08:00"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsRuntimeStabilitySummaryMissingTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_summary_missing_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_summary_timestamp=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMissingRuntimeStabilityFailureCounters) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_missing_stability_failures_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_disk_failures=missing"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_watchdog_failures=missing"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_stability_health_failures=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMissingRuntimeStabilitySamplesAndInterval) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_missing_stability_timing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_samples=missing"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_interval_s=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMalformedRuntimeStabilityOverall) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_bad_stability_overall_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS_BAD\n"
            "samples=1440\n"
            "interval_s=60\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMalformedRuntimeStabilitySamples) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_bad_samples_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440abc\n"
            "interval_s=60\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_samples=1440abc"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMalformedRuntimeStabilityInterval) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_bad_interval_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60abc\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_interval_s=60abc"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsTooSlowVerifiedPowerLossResume) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_slow_resume_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "recovery_time_s=70\n"
            "max_recovery_time_s=45\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=70"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPowerLossResumeMissingTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_resume_timestamp_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "power_loss_resume_confirmation_overall=missing\n"
            "power_loss_resume_confirmation_keys_status=missing\n"
            "metrics_report=" +
                session_dir +
            "/reports/validation_metrics_report.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");

  EXPECT_NE(0, std::system(
                   (session_dir + "/commands/capture_field_acceptance.sh")
                       .c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_timestamp=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMalformedVerifiedPowerLossResumeTime) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_bad_resume_time_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "recovery_time_s=25abc\n"
            "max_recovery_time_s=45\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25abc"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsFailingMetricsPowerLossResume) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_bad_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=FAIL;total_records=1;failed_records=1\n"
            "---\n"
            "session=power_loss;scenario=POWER_LOSS_ORIGIN;status=FAIL;failed_checks=1\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMetricsZeroCountsWithLeadingZeros) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_leading_zero_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=00\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=00\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_deployment_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("section_export_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMetricsSummaryValueWhitespacePollution) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_metrics_whitespace_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS ;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMetricsWithDuplicateKeys) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_duplicate_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=1;failed_records=0\n"
            "---\n"
            "session=power_loss;scenario=POWER_LOSS_ORIGIN;status=FAIL;failed_checks=1\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMetricsDuplicateKeysInUnrelatedRecord) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_unrelated_duplicate_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=2;failed_records=0\n"
            "---\n"
            "session=other_session;scenario=POWER_LOSS_ORIGIN;status=FAIL;"
            "failed_checks=1;operator=qa;operator=qa\n"
            "recovery_time_s=70\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;"
            "failed_checks=0\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("event_file_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_deployment_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("runtime_stability_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=PASS"));
  EXPECT_NE(std::string::npos, report.find("section_export_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMetricsDuplicateKeysInMatchingDetailLine) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_detail_duplicate_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;"
            "failed_checks=0\n"
            "operator=qa;operator=qa\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMetricsWithoutPositiveTotalRecords) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_zero_records_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=0;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMetricsWithoutMatchingScenarioRecord) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_summary_only_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0,
            std::system(
                (session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMetricsRecoveryTimeFromOtherRecord) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_wrong_recovery_record_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=2;failed_records=0\n"
            "---\n"
            "session=other_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=70\n");

  EXPECT_NE(0,
            std::system(
                (session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=70"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMetricsPowerLossResumeRecoveryTimeMismatch) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_resume_time_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=30\n");
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=" + session_dir + "/reports/validation_metrics_report.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");

  EXPECT_NE(0,
            std::system(
                (session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsMetricsPowerLossResumeWithoutMetricsReportPath) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_resume_missing_metrics_path_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");

  EXPECT_NE(0,
            std::system(
                (session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_source=metrics_report"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsEnvOverrideDeploymentEvidence) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_env_deploy_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=env_override\n"
            "docker_container_status=running\n"
            "docker_container_status_source=env_override\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=manual_file\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_deployment_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("systemd_active_source=env_override"));
  EXPECT_NE(std::string::npos,
            report.find("docker_container_status_source=env_override"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsEnvOverrideRuntimeHealthEvidence) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_env_health_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir, true, true);
  writeFile(session_dir + "/logs/runtime_health_latest.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=env_override\n"
            "docker_container_status=running\n"
            "docker_container_status_source=env_override\n");

  EXPECT_NE(0, std::system(
                   (session_dir + "/commands/capture_field_acceptance.sh")
                       .c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_health_systemd_active_source=env_override"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_health_docker_container_status_source=env_override"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsRuntimeHealthMissingTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_health_missing_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir, true, true);
  writeFile(session_dir + "/logs/runtime_health_latest.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");

  EXPECT_NE(0, std::system(
                   (session_dir + "/commands/capture_field_acceptance.sh")
                       .c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_timestamp=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsFutureRuntimeHealthTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_health_future_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir, true, true);
  writeFile(session_dir + "/logs/runtime_health_latest.txt",
            "timestamp=2099-01-01T00:00:00+08:00\n"
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");

  EXPECT_NE(0, std::system(
                   (session_dir + "/commands/capture_field_acceptance.sh")
                       .c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("field_acceptance_timestamp_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.find("runtime_health_timestamp=2099-01-01T00:00:00+08:00"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsEnvOverridePhysicalEvidence) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_env_physical_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario LONG_STABILITY "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=manual_env\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_env\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_env"));
  EXPECT_NE(std::string::npos, report.find("pps_ptp_wiring_verified=FAIL"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation_source=manual_env"));
}

TEST(RecordSessionScript, FieldAcceptancePassesManualPowerLossResumeWithAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_resume_manual_pass_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" "
      "--runtime-dir /tmp/tunnel_lio_runtime/test_session --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeTimeSyncRawCapture(session_dir);
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                timeSyncRawLine(session_dir));
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "deployment_status=PASS\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/test_session\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_process_status=PASS\n"
            "start_command=PASS\n");
  const std::string runtime_health_report =
      "timestamp=2026-06-03T00:00:00+08:00\n"
      "runtime_dir=/tmp/tunnel_lio_runtime/test_session\n"
      "disk_available_gb=120\n"
      "runtime_pid=1234\n"
      "systemd_active=active\n"
      "systemd_active_source=systemctl\n"
      "docker_container_status=running\n"
      "docker_container_status_source=docker_inspect\n";
  writeFile(session_dir + "/logs/runtime_health_latest.txt",
            runtime_health_report);
  writeFile(session_dir + "/logs/runtime_health.txt", runtime_health_report);
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(session_dir + "/logs/runtime_stability_run.log",
            "started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/test_session\n"
            "samples=1\n"
            "interval=86400\n"
            "exit_status=0\n"
            "capture_exit_status=0\n"
            "finished_at=2026-06-04T00:00:00+08:00\n");
  writeFile(session_dir + "/logs/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n" +
                ppsPtpTimeSyncRawLines(session_dir) +
            "time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=" +
                session_dir + "/logs/time_sync_status.txt\n");
  writeFile(session_dir + "/reports/section_export.csv",
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "test_session,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/replay_events.txt",
            "event=session_start;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=0.0\n"
            "event=power_loss;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=4.0\n"
            "event=recovered;session_id=test_session;scenario=POWER_LOSS_ORIGIN;t=29.0\n");

  EXPECT_EQ(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("session_id=test_session"));
  EXPECT_NE(std::string::npos, report.find("scenario=POWER_LOSS_ORIGIN"));
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("metrics_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_confirmation_overall=PASS"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_confirmation_keys_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation_overall=PASS"));
  EXPECT_NE(std::string::npos, report.find("wiring_confirmation_keys_status=PASS"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsManualPowerLossResumeMissingConfirmationKeysStatus) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_resume_missing_confirmation_keys_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_confirmation_keys_status=missing"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsManualPowerLossResumeConfirmationKeysStatusFail) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_resume_confirmation_keys_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=FAIL\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_confirmation_keys_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsPowerLossResumeUnrelatedDuplicateKey) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_power_loss_duplicate_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n"
            "metrics_report=" +
                session_dir + "/reports/validation_metrics_report.txt\n"
            "operator=qa\n"
            "operator=qa\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsManualPowerLossResumeWhenMetricsFail) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_manual_resume_metrics_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --runtime-dir /tmp/runtime --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=FAIL;total_records=1;failed_records=1\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=FAIL;failed_checks=1\n"
            "recovery_time_s=25\n");
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0,
            std::system(
                (session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsManualPowerLossResumeWithoutAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_resume_missing_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos, report.find("resume_verified_by=missing"));
  EXPECT_NE(std::string::npos, report.find("resume_verified_at=missing"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsManualPowerLossResumeSentinelAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_resume_sentinel_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=__DUPLICATE_KEY__\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("resume_verified_by=__DUPLICATE_KEY__"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsManualPowerLossResumeNonIsoAuditTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_resume_noniso_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=not-a-time\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("resume_verified_at=not-a-time"));
}

TEST(RecordSessionScript,
     FieldAcceptanceRejectsManualPowerLossResumeImpossibleCalendarAuditTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_field_acceptance_resume_calendar_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writePassingFieldAcceptanceInputs(session_dir);
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-02-31T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("resume_verified_at=2026-02-31T00:00:00+08:00"));
}

TEST(RecordSessionScript, FieldAcceptanceRejectsMalformedManualPowerLossResumeOverall) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_field_acceptance_bad_resume_overall_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/logs/time_sync_status.txt",
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.4\n"
            "mean_offset_ms=0.8\n");
  writeFile(session_dir + "/logs/runtime_deployment_check.txt",
            "deployment_status=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(session_dir + "/logs/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1440\n"
            "interval_s=60\n");
  writeFile(session_dir + "/reports/power_loss_resume_verified.txt",
            "power_loss_resume_status=PASS_BAD\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(session_dir + "/reports/pps_ptp_wiring_verified.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_field_acceptance.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/field_acceptance_report.txt");
  EXPECT_NE(std::string::npos, report.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureDerivesPassFromMetrics) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "\n"
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");

  ASSERT_EQ(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
  EXPECT_NE(std::string::npos, report.find("max_recovery_time_s=45"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureAcceptsScientificNotationMetrics) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_scientific_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=2.5e1\n");

  EXPECT_EQ(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=2.5e1"));
  EXPECT_NE(std::string::npos, report.find("max_recovery_time_s=45"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsMalformedMetricsRecoveryTime) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_bad_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25abc\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25abc"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsFailingMetricsReport) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_failing_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=FAIL;total_records=1;failed_records=1\n"
            "---\n"
            "session=power_loss;scenario=POWER_LOSS_ORIGIN;status=FAIL;failed_checks=1\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=missing"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsMetricsZeroCountsWithLeadingZeros) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_leading_zero_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=00\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=00\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=missing"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsMetricsSummaryValueWhitespacePollution) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_metrics_whitespace_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS ;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsMetricsDuplicateKeysInUnrelatedRecord) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_power_loss_resume_unrelated_duplicate_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=2;failed_records=0\n"
            "---\n"
            "session=other_session;scenario=POWER_LOSS_ORIGIN;status=FAIL;"
            "failed_checks=1;operator=qa;operator=qa\n"
            "recovery_time_s=70\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;"
            "failed_checks=0\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsMetricsDuplicateKeysInMatchingDetailLine) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_power_loss_resume_detail_duplicate_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;"
            "failed_checks=0\n"
            "operator=qa;operator=qa\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=missing"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsMetricsWithoutPositiveTotalRecords) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_zero_records_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=0;failed_records=0\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsMetricsWithoutMatchingScenarioRecord) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_power_loss_resume_summary_only_metrics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0,
            std::system(
                (session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=missing"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsMetricsRecoveryTimeFromOtherRecord) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_power_loss_resume_wrong_recovery_record_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/validation_metrics_report.txt",
            "overall=PASS;total_records=2;failed_records=0\n"
            "---\n"
            "session=other_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n"
            "---\n"
            "session=test_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=70\n");

  EXPECT_NE(0,
            std::system(
                (session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_source=metrics_report"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=70"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsManualConfirmationWithoutRecoveryTime) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_manual_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/power_loss_resume_confirmation.txt",
            "power_loss_resume_status=PASS\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=missing"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsManualConfirmationWithoutAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_manual_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/power_loss_resume_confirmation.txt",
            "power_loss_resume_status=PASS\n"
            "recovery_time_s=25\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
  EXPECT_NE(std::string::npos, report.find("resume_verified_by=missing"));
  EXPECT_NE(std::string::npos, report.find("resume_verified_at=missing"));
}

TEST(RecordSessionScript,
     PowerLossResumeCaptureRejectsManualConfirmationNonIsoAuditTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_manual_noniso_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/power_loss_resume_confirmation.txt",
            "power_loss_resume_status=PASS\n"
            "recovery_time_s=25\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=not-a-time\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos, report.find("resume_verified_at=not-a-time"));
}

TEST(RecordSessionScript,
     PowerLossResumeCaptureRejectsManualConfirmationImpossibleCalendarAuditTimestamp) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_manual_calendar_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/power_loss_resume_confirmation.txt",
            "power_loss_resume_status=PASS\n"
            "recovery_time_s=25\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-02-31T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos,
            report.find("resume_verified_at=2026-02-31T00:00:00+08:00"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsManualConfirmationSeparatorAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_manual_separator_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/power_loss_resume_confirmation.txt",
            "power_loss_resume_status=PASS\n"
            "recovery_time_s=25\n"
            "resume_verified_by=qa_operator;power_loss_resume_status=PASS\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos,
            report.find("resume_verified_by=qa_operator;power_loss_resume_status"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsManualConfirmationCarriageReturnAuditFields) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_manual_cr_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/power_loss_resume_confirmation.txt",
            "power_loss_resume_status=PASS\n"
            "recovery_time_s=25\n"
            "resume_verified_by=qa_operator\r\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsMalformedManualConfirmationOverall) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_manual_bad_overall_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/power_loss_resume_confirmation.txt",
            "power_loss_resume_status=PASS_BAD\n"
            "recovery_time_s=25\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsDuplicateManualConfirmationOverall) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_manual_duplicate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/power_loss_resume_confirmation.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_status=FAIL\n"
            "recovery_time_s=25\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_confirmation_overall=__DUPLICATE_KEY__"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
}

TEST(RecordSessionScript,
     PowerLossResumeCaptureRejectsManualConfirmationUnrelatedDuplicateKey) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_power_loss_resume_manual_duplicate_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/power_loss_resume_confirmation.txt",
            "power_loss_resume_status=PASS\n"
            "recovery_time_s=25\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n"
            "operator=qa\n"
            "operator=qa\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_confirmation_overall=PASS"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
}

TEST(RecordSessionScript,
     PowerLossResumeCaptureRejectsManualConfirmationEmptyKeyLine) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] =
      "/tmp/tunnel_lio_power_loss_resume_manual_empty_key_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/power_loss_resume_confirmation.txt",
            "power_loss_resume_status=PASS\n"
            "recovery_time_s=25\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n"
            "=orphan_value\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos,
            report.find("power_loss_resume_confirmation_keys_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=25"));
}

TEST(RecordSessionScript, PowerLossResumeCaptureRejectsTooSlowManualConfirmation) {
  const char* script_env = std::getenv("RECORD_SESSION_SCRIPT");
  const std::string script =
      script_env == nullptr ? std::string(RECORD_SESSION_SCRIPT_PATH)
                            : std::string(script_env);

  char root_template[] = "/tmp/tunnel_lio_power_loss_resume_manual_slow_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string root_dir(root);

  const std::string command =
      "bash \"" + script + "\" --root \"" + root_dir +
      "\" --name test_session --prefix tunnel_lio --scenario POWER_LOSS_ORIGIN "
      "--topics \"/time/status /tf\" --dry-run";
  ASSERT_EQ(0, std::system(command.c_str()));

  const std::string session_dir = root_dir + "/test_session";
  writeFile(session_dir + "/reports/power_loss_resume_confirmation.txt",
            "power_loss_resume_status=PASS\n"
            "recovery_time_s=70\n");

  EXPECT_NE(0, std::system((session_dir + "/commands/capture_power_loss_resume.sh").c_str()));

  const std::string report =
      readFile(session_dir + "/reports/power_loss_resume_verified.txt");
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.find("power_loss_resume_source=manual_file"));
  EXPECT_NE(std::string::npos, report.find("recovery_time_s=70"));
  EXPECT_NE(std::string::npos, report.find("max_recovery_time_s=45"));
}

TEST(RecordSessionScript, LaunchPassesTimeSyncTopicsToRecorder) {
  const std::string launch_file = recordSessionLaunchPath();
  ASSERT_FALSE(launch_file.empty());

  const std::string launch = readFile(launch_file);
  EXPECT_NE(std::string::npos,
            launch.find("<arg name=\"time_status_topic\" default=\"/time/status\""));
  EXPECT_NE(std::string::npos,
            launch.find("<arg name=\"pps_topic\" default=\"/time/pps_event\""));
  EXPECT_NE(std::string::npos,
            launch.find("--time-status-topic $(arg time_status_topic)"));
  EXPECT_NE(std::string::npos,
            launch.find("--pps-topic $(arg pps_topic)"));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
