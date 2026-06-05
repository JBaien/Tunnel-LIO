#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include "lio_eval_tools/evidence_manifest.h"

namespace lio_eval_tools {
namespace {

#ifndef LIO_EVAL_TOOLS_CONFIG_DIR
#define LIO_EVAL_TOOLS_CONFIG_DIR ""
#endif

void writeFile(const std::string& path, const std::string& content) {
  std::ofstream output(path.c_str());
  output << content;
}

std::string readFile(const std::string& path) {
  std::ifstream input(path.c_str());
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::string firstDataLine(const std::string& text) {
  std::stringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    const std::size_t first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    return line;
  }
  return "";
}

void writeRuntimeStabilityRunLog(
    const std::string& base_dir,
    const std::string& runtime_dir = "/tmp/tunnel_lio_runtime/s1",
    const std::string& samples = "1",
    const std::string& interval = "86400",
    const std::string& exit_status = "0",
    const std::string& started_at = "2026-06-03T00:00:00+08:00",
    const std::string& finished_at = "2026-06-04T00:00:00+08:00",
    const std::string& capture_exit_status = "0") {
  writeFile(base_dir + "/runtime_stability_run.log",
            "started_at=" +
                started_at +
                "\n"
            "runtime_dir=" +
                runtime_dir +
                "\n"
                "samples=" +
                samples +
                "\n"
                "interval=" +
                interval +
                "\n"
                "exit_status=" +
                exit_status +
                "\n"
                "capture_exit_status=" +
                capture_exit_status +
                "\n"
                "finished_at=" +
                finished_at +
                "\n");
}

std::string replaySessionStartEvent(const std::string& session_id,
                                    const std::string& scenario) {
  return "event=session_start;scenario=" + scenario +
         ";session_id=" + session_id + ";t=0.0\n";
}

std::string removeLineWithPrefix(const std::string& text,
                                 const std::string& prefix) {
  std::string output;
  std::size_t line_begin = 0;
  while (line_begin < text.size()) {
    const std::size_t line_end = text.find('\n', line_begin);
    const std::size_t line_size =
        line_end == std::string::npos ? text.size() - line_begin
                                      : line_end - line_begin + 1;
    const std::string line = text.substr(line_begin, line_size);
    if (line.find(prefix) != 0) {
      output += line;
    }
    if (line_end == std::string::npos) {
      break;
    }
    line_begin = line_end + 1;
  }
  return output;
}

std::string replaceLineWithPrefix(const std::string& text,
                                  const std::string& prefix,
                                  const std::string& replacement_line) {
  std::string output;
  std::size_t line_begin = 0;
  while (line_begin < text.size()) {
    const std::size_t line_end = text.find('\n', line_begin);
    const std::size_t line_size =
        line_end == std::string::npos ? text.size() - line_begin
                                      : line_end - line_begin + 1;
    const std::string line = text.substr(line_begin, line_size);
    if (line.find(prefix) == 0) {
      output += replacement_line;
      if (replacement_line.empty() || replacement_line.back() != '\n') {
        output += "\n";
      }
    } else {
      output += line;
    }
    if (line_end == std::string::npos) {
      break;
    }
    line_begin = line_end + 1;
  }
  return output;
}

std::string replaceOrAppendLineWithPrefix(const std::string& text,
                                          const std::string& prefix,
                                          const std::string& replacement_line) {
  const std::string replaced =
      replaceLineWithPrefix(text, prefix, replacement_line);
  if (replaced != text) {
    return replaced;
  }
  std::string output = text;
  if (!output.empty() && output[output.size() - 1] != '\n') {
    output += "\n";
  }
  output += replacement_line;
  if (replacement_line.empty() || replacement_line[replacement_line.size() - 1] != '\n') {
    output += "\n";
  }
  return output;
}

std::string fieldAcceptanceReport(
    const std::string& runtime_stability_csv_status = "PASS",
    const std::string& runtime_stability_csv_samples = "1",
    const std::string& runtime_stability_sample_count_match = "PASS",
    const std::string& runtime_stability_samples = "1",
    const std::string& runtime_stability_interval_s = "86400",
    const std::string& runtime_stability_overall = "PASS",
    const std::string& power_loss_resume_overall = "PASS",
    const std::string& pps_ptp_wiring_overall = "PASS",
    const std::string& metrics_report = "metrics.txt",
    const std::string& power_loss_resume_source = "metrics_report",
    const std::string& power_loss_resume_confirmation_overall = "missing",
    const std::string& session_id = "s1",
    const std::string& scenario = "POWER_LOSS_ORIGIN",
    const std::string& deployment_runtime_dir = "/tmp/tunnel_lio_runtime/s1",
    const std::string& runtime_health_runtime_dir = "/tmp/tunnel_lio_runtime/s1",
    const std::string& runtime_health_disk_available_gb = "120",
    const std::string& runtime_health_pid = "1234",
    const std::string& runtime_health_systemd_active = "active",
    const std::string& runtime_health_docker_container_status = "running",
    const std::string& systemd_unit_file = "PASS",
    const std::string& systemd_env_file = "PASS",
    const std::string& docker_compose_file = "PASS",
    const std::string& docker_env_file = "PASS",
    const std::string& start_command = "PASS",
    const std::string& runtime_process_status = "PASS",
    const std::string& timestamp = "2026-06-04T00:00:00+08:00",
    const std::string& pps_ptp_wiring_time_sync_report = "time_sync.txt",
    const std::string& section_export_report = "section_export.csv",
    const std::string& runtime_health_report = "runtime_health.txt",
    const std::string& runtime_deployment_report = "runtime_deployment.txt",
    const std::string& runtime_stability_csv_report = "runtime_stability.csv",
    const std::string& runtime_stability_summary_report =
        "runtime_stability_summary.txt",
    const std::string& runtime_stability_run_log_report =
        "runtime_stability_run.log",
    const std::string& power_loss_resume_report = "power_loss_resume.txt",
    const std::string& time_sync_report = "time_sync.txt",
    const std::string& pps_ptp_wiring_report = "pps_ptp_wiring.txt",
    const std::string& event_file_status = "PASS",
    const std::string& event_file_report = "events.txt",
    const std::string& runtime_stability_csv_first_timestamp =
        "2026-06-03T00:00:00+08:00",
    const std::string& runtime_stability_csv_last_timestamp =
        "2026-06-03T00:00:00+08:00") {
  return "timestamp=" + timestamp +
         "\n"
         "field_acceptance_timestamp_status=PASS\n"
         "session_id=" +
         session_id +
         "\n"
         "scenario=" +
         scenario +
         "\n"
         "field_acceptance_status=PASS\n"
         "event_file_status=" +
         event_file_status +
         "\n"
         "event_file_report=" +
         event_file_report +
         "\n"
         "time_sync_status=PASS\n"
         "time_sync_keys_status=PASS\n"
         "time_sync_report=" +
         time_sync_report +
         "\n"
         "time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
         "time_sync_raw=time_status_raw.yaml\n"
         "time_sync_raw_status=PASS\n"
         "time_status_topic=/time/status\n"
         "pps_topic=/time/pps_event\n"
         "time_capture_status=CAPTURED\n"
         "time_pps_status=PASS\n"
         "time_clock_offset_status=PASS\n"
         "runtime_health_status=PASS\n"
         "runtime_health_keys_status=PASS\n"
         "runtime_health_report=" +
         runtime_health_report +
         "\n"
         "runtime_health_timestamp=2026-06-03T00:00:00+08:00\n"
         "runtime_health_runtime_dir=" +
         runtime_health_runtime_dir +
         "\n"
         "runtime_health_disk_available_gb=" +
         runtime_health_disk_available_gb +
         "\n"
         "runtime_health_pid=" +
         runtime_health_pid +
         "\n"
         "runtime_health_systemd_active=" +
         runtime_health_systemd_active +
         "\n"
         "runtime_health_systemd_active_source=" +
         std::string("systemctl\n") +
         "runtime_health_docker_container_status=" +
         runtime_health_docker_container_status +
         "\n"
         "runtime_health_docker_container_status_source=" +
         std::string("docker_inspect\n") +
         "section_export_status=PASS\n"
         "section_export_report=" +
         section_export_report +
         "\n"
         "runtime_deployment_status=PASS\n"
         "runtime_deployment_keys_status=PASS\n"
         "runtime_deployment_report=" +
         runtime_deployment_report +
         "\n"
         "runtime_deployment_timestamp=2026-06-03T00:00:00+08:00\n"
         "deployment_overall=PASS\n"
         "deployment_status=PASS\n"
         "systemd_unit_file=" +
         systemd_unit_file +
         "\n"
         "systemd_env_file=" +
         systemd_env_file +
         "\n"
         "docker_compose_file=" +
         docker_compose_file +
         "\n"
         "docker_env_file=" +
         docker_env_file +
         "\n"
         "start_command=" +
         start_command +
         "\n"
         "runtime_process_status=" +
         runtime_process_status +
         "\n"
         "deployment_runtime_dir=" +
         deployment_runtime_dir +
         "\n"
         "runtime_stability_status=PASS\n"
         "runtime_stability_min_duration_h=24\n"
         "runtime_stability_min_duration_status=PASS\n"
         "runtime_stability_csv_report=" +
         runtime_stability_csv_report +
         "\n"
         "runtime_stability_csv_first_timestamp=" +
         runtime_stability_csv_first_timestamp +
         "\n"
         "runtime_stability_csv_last_timestamp=" +
         runtime_stability_csv_last_timestamp +
         "\n"
         "runtime_stability_summary_report=" +
         runtime_stability_summary_report +
         "\n"
         "runtime_stability_summary_timestamp=2026-06-03T00:00:01+08:00\n"
         "runtime_stability_run_log_report=" +
         runtime_stability_run_log_report +
         "\n"
         "runtime_stability_overall=" +
         runtime_stability_overall +
         "\n"
         "runtime_stability_summary_keys_status=PASS\n"
         "runtime_stability_csv_status=" +
         runtime_stability_csv_status +
         "\n"
         "runtime_stability_csv_samples=" +
         runtime_stability_csv_samples +
         "\n"
         "runtime_stability_sample_count_match=" +
         runtime_stability_sample_count_match +
         "\n"
         "runtime_stability_run_log_status=PASS\n"
         "runtime_stability_run_log_keys_status=PASS\n"
         "runtime_stability_run_log_started_at=2026-06-03T00:00:00+08:00\n"
         "runtime_stability_run_log_finished_at=2026-06-04T00:00:00+08:00\n"
         "runtime_stability_run_log_runtime_dir=" +
         deployment_runtime_dir +
         "\n"
         "runtime_stability_run_log_samples=" +
         runtime_stability_samples +
         "\n"
         "runtime_stability_run_log_interval=" +
         runtime_stability_interval_s +
         "\n"
         "runtime_stability_run_log_elapsed_s=86400\n"
         "runtime_stability_run_log_required_elapsed_s=86340\n"
         "runtime_stability_run_log_duration_status=PASS\n"
         "runtime_stability_run_log_exit_status=0\n"
         "runtime_stability_run_log_capture_exit_status=0\n"
         "power_loss_resume_status=PASS\n"
         "power_loss_resume_keys_status=PASS\n"
         "power_loss_resume_report=" +
         power_loss_resume_report +
         "\n"
         "power_loss_resume_timestamp=2026-06-03T00:00:00+08:00\n"
         "power_loss_resume_overall=" +
         power_loss_resume_overall +
         "\n"
         "power_loss_resume_source=" +
         power_loss_resume_source +
         "\n"
         "power_loss_resume_confirmation_overall=" +
         power_loss_resume_confirmation_overall +
         "\n"
         "power_loss_resume_confirmation_keys_status=" +
         (power_loss_resume_source == "manual_file" ? "PASS" : "missing") +
         "\n"
         "metrics_report=" +
         metrics_report +
         "\n"
         "metrics_status=PASS\n"
         "recovery_time_s=25\n"
         "max_recovery_time_s=45\n"
         "resume_verified_by=qa_operator\n"
         "resume_verified_at=2026-06-03T00:00:00+08:00\n"
         "pps_jitter_ms=0.5\n"
         "mean_offset_ms=1.2\n"
         "pps_ptp_wiring_verified=PASS\n"
         "pps_ptp_wiring_keys_status=PASS\n"
         "pps_ptp_wiring_report=" +
         pps_ptp_wiring_report +
         "\n"
         "pps_ptp_wiring_timestamp=2026-06-03T00:00:00+08:00\n"
         "pps_ptp_wiring_overall=" +
         pps_ptp_wiring_overall +
         "\n"
         "pps_ptp_wiring_time_sync_status=PASS\n"
         "pps_ptp_wiring_capture_status=CAPTURED\n"
         "pps_ptp_wiring_time_status_topic=/time/status\n"
         "pps_ptp_wiring_pps_topic=/time/pps_event\n"
         "pps_ptp_wiring_pps_status=PASS\n"
         "pps_ptp_wiring_clock_offset_status=PASS\n"
         "pps_ptp_wiring_pps_jitter_ms=0.5\n"
         "pps_ptp_wiring_mean_offset_ms=1.2\n"
         "pps_ptp_wiring_time_sync_report=" +
         pps_ptp_wiring_time_sync_report +
         "\n"
         "pps_ptp_wiring_time_sync_raw=time_status_raw.yaml\n"
         "pps_ptp_wiring_time_sync_raw_status=PASS\n"
         "pps_ptp_wiring_time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
         "wiring_confirmation=PASS\n"
         "wiring_confirmation_overall=PASS\n"
         "wiring_confirmation_keys_status=PASS\n"
         "wiring_confirmation_source=manual_file\n"
         "pps_wiring_verified=PASS\n"
         "ptp_wiring_verified=PASS\n"
         "wiring_verified_by=qa_operator\n"
         "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
         "systemd_active=active\n"
         "systemd_active_source=systemctl\n"
         "docker_container_status=running\n"
         "docker_container_status_source=docker_inspect\n"
         "runtime_stability_duration_h=24\n"
         "runtime_stability_samples=" +
         runtime_stability_samples +
         "\n"
         "runtime_stability_interval_s=" +
         runtime_stability_interval_s +
         "\n"
         "runtime_stability_disk_failures=0\n"
         "runtime_stability_watchdog_failures=0\n"
         "runtime_stability_watchdog_skipped=0\n"
         "runtime_stability_health_failures=0\n";
}

void writePassingEvidenceFiles(const std::string& base_dir,
                               const std::string& section_export_content) {
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(base_dir + "/events.txt",
            replaySessionStartEvent("s1", "POWER_LOSS_ORIGIN") +
                "event=power_loss;scenario=POWER_LOSS_ORIGIN;session_id=s1;t=4.0\n"
                "event=recovered;scenario=POWER_LOSS_ORIGIN;session_id=s1;t=29.0\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "time_sync_raw=time_status_raw.yaml\n"
            "time_sync_raw_status=PASS\n"
            "time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=time_sync.txt\n"
            "ptp_status=available\n");
  writeFile(base_dir + "/runtime_health.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/section_export.csv", section_export_content);
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeRuntimeStabilityRunLog(base_dir);
  writeFile(base_dir + "/power_loss_resume.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=missing\n"
            "power_loss_resume_confirmation_keys_status=missing\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt", fieldAcceptanceReport());
}

void rewriteRuntimeDirEvidence(const std::string& base_dir,
                               const std::string& runtime_dir) {
  writeRuntimeStabilityRunLog(base_dir, runtime_dir);
  writeFile(base_dir + "/runtime_health.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=" + runtime_dir +
            "\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=" + runtime_dir +
            "\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport(
                "PASS",
                "1",
                "PASS",
                "1",
                "86400",
                "PASS",
                "PASS",
                "PASS",
                "metrics.txt",
                "metrics_report",
                "missing",
                "s1",
                "POWER_LOSS_ORIGIN",
                runtime_dir,
                runtime_dir));
}

std::string passingEvidenceManifestLine() {
  return "session_id=s1;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "section_export=section_export.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s1";
}

EvidenceManifest parsePassingEvidenceManifest() {
  return parseEvidenceManifestRecord(passingEvidenceManifestLine());
}

TEST(EvidenceManifest, PassesWhenRequiredEvidenceFilesAndMetricsPass) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_ok_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(base_dir + "/events.txt",
            replaySessionStartEvent("s1", "POWER_LOSS_ORIGIN") +
                "event=power_loss;scenario=POWER_LOSS_ORIGIN;session_id=s1;t=4.0\n"
                "event=recovered;scenario=POWER_LOSS_ORIGIN;session_id=s1;t=29.0\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "time_sync_raw=time_status_raw.yaml\n"
            "time_sync_raw_status=PASS\n"
            "time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=time_sync.txt\n"
            "ptp_status=available\n");
  writeFile(base_dir + "/runtime_health.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/section_export.csv",
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeRuntimeStabilityRunLog(base_dir);
  writeFile(base_dir + "/power_loss_resume.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "timestamp=2026-06-04T00:00:00+08:00\n"
            "field_acceptance_timestamp_status=PASS\n"
            "session_id=s1\n"
            "scenario=POWER_LOSS_ORIGIN\n"
            "field_acceptance_status=PASS\n"
            "event_file_status=PASS\n"
            "event_file_report=events.txt\n"
            "time_sync_status=PASS\n"
            "time_sync_keys_status=PASS\n"
            "time_sync_report=time_sync.txt\n"
            "time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_raw=time_status_raw.yaml\n"
            "time_sync_raw_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "time_capture_status=CAPTURED\n"
            "time_pps_status=PASS\n"
            "time_clock_offset_status=PASS\n"
            "runtime_health_status=PASS\n"
            "runtime_health_keys_status=PASS\n"
            "runtime_health_report=runtime_health.txt\n"
            "runtime_health_timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_health_runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "runtime_health_disk_available_gb=120\n"
            "runtime_health_pid=1234\n"
            "runtime_health_systemd_active=active\n"
            "runtime_health_systemd_active_source=systemctl\n"
            "runtime_health_docker_container_status=running\n"
            "runtime_health_docker_container_status_source=docker_inspect\n"
            "section_export_status=PASS\n"
            "section_export_report=section_export.csv\n"
            "runtime_deployment_status=PASS\n"
            "runtime_deployment_keys_status=PASS\n"
            "runtime_deployment_report=runtime_deployment.txt\n"
            "runtime_deployment_timestamp=2026-06-03T00:00:00+08:00\n"
            "deployment_overall=PASS\n"
            "deployment_status=PASS\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "runtime_stability_status=PASS\n"
            "runtime_stability_min_duration_h=24\n"
            "runtime_stability_min_duration_status=PASS\n"
            "runtime_stability_csv_report=runtime_stability.csv\n"
            "runtime_stability_csv_first_timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_stability_csv_last_timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_stability_summary_report=runtime_stability_summary.txt\n"
            "runtime_stability_summary_timestamp=2026-06-03T00:00:01+08:00\n"
            "runtime_stability_run_log_report=runtime_stability_run.log\n"
            "runtime_stability_overall=PASS\n"
            "runtime_stability_summary_keys_status=PASS\n"
            "runtime_stability_csv_status=PASS\n"
            "runtime_stability_csv_samples=1\n"
            "runtime_stability_sample_count_match=PASS\n"
            "runtime_stability_run_log_status=PASS\n"
            "runtime_stability_run_log_keys_status=PASS\n"
            "runtime_stability_run_log_started_at=2026-06-03T00:00:00+08:00\n"
            "runtime_stability_run_log_finished_at=2026-06-04T00:00:00+08:00\n"
            "runtime_stability_run_log_runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "runtime_stability_run_log_samples=1\n"
            "runtime_stability_run_log_interval=86400\n"
            "runtime_stability_run_log_elapsed_s=86400\n"
            "runtime_stability_run_log_required_elapsed_s=86340\n"
            "runtime_stability_run_log_duration_status=PASS\n"
            "runtime_stability_run_log_exit_status=0\n"
            "runtime_stability_run_log_capture_exit_status=0\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_keys_status=PASS\n"
            "power_loss_resume_report=power_loss_resume.txt\n"
            "power_loss_resume_timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_overall=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=metrics.txt\n"
            "power_loss_resume_confirmation_overall=missing\n"
            "power_loss_resume_confirmation_keys_status=missing\n"
            "metrics_status=PASS\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "pps_ptp_wiring_verified=PASS\n"
            "pps_ptp_wiring_keys_status=PASS\n"
            "pps_ptp_wiring_overall=PASS\n"
            "pps_ptp_wiring_time_sync_status=PASS\n"
            "pps_ptp_wiring_capture_status=CAPTURED\n"
            "pps_ptp_wiring_time_status_topic=/time/status\n"
            "pps_ptp_wiring_pps_topic=/time/pps_event\n"
            "pps_ptp_wiring_pps_status=PASS\n"
            "pps_ptp_wiring_clock_offset_status=PASS\n"
            "pps_ptp_wiring_pps_jitter_ms=0.5\n"
            "pps_ptp_wiring_mean_offset_ms=1.2\n"
            "pps_ptp_wiring_report=pps_ptp_wiring.txt\n"
            "pps_ptp_wiring_timestamp=2026-06-03T00:00:00+08:00\n"
            "pps_ptp_wiring_time_sync_report=time_sync.txt\n"
            "pps_ptp_wiring_time_sync_raw=time_status_raw.yaml\n"
            "pps_ptp_wiring_time_sync_raw_status=PASS\n"
            "pps_ptp_wiring_time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s1;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "section_export=section_export.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s1");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_TRUE(report.passed);
  EXPECT_EQ(17u, report.checked_files);
  EXPECT_TRUE(report.missing_files.empty());
  EXPECT_TRUE(report.section_export_checked);
  EXPECT_TRUE(report.section_export_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_EQ("s1", report.session_id);
  EXPECT_EQ("/tmp/tunnel_lio_runtime/s1", report.runtime_dir);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_deployment_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_run_log_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_dir=/tmp/tunnel_lio_runtime/s1"));
}

TEST(EvidenceManifest, SectionExportAcceptsEightMachineStateSources) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_eight_states_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,CMD_MOVE_NO_DISP,B,0.800,30.000,180\n"
      "s1,11.000,RELOCALIZING,C,0.700,45.000,120\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.section_export_checked);
  EXPECT_TRUE(report.section_export_passed);
  EXPECT_TRUE(report.passed);
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=PASS"));
}

TEST(EvidenceManifest, SampleEvidenceManifestPassesCurrentGate) {
  const std::string config_dir = LIO_EVAL_TOOLS_CONFIG_DIR;
  ASSERT_FALSE(config_dir.empty());

  const std::string manifest_text =
      readFile(config_dir + "/sample_evidence_manifest.txt");
  ASSERT_FALSE(manifest_text.empty());

  const EvidenceBundleReport report = evaluateEvidenceBundle(
      parseEvidenceManifestRecord(firstDataLine(manifest_text)), config_dir);

  EXPECT_TRUE(report.passed) << report.text;
  EXPECT_NE(std::string::npos, report.text.find("event_file_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=PASS"));
}

TEST(EvidenceManifest, FailsWhenManifestHasMalformedSemicolonToken) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_malformed_token_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);
  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  const EvidenceManifest manifest =
      parseEvidenceManifestRecord(passingEvidenceManifestLine() +
                                  ";not_a_key_value_pair");
  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_NE(std::string::npos,
            report.text.find("manifest_malformed_tokens=true"));
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncHasMalformedKeyValueLine) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_line_token_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);
  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_sync.txt",
            readFile(base_dir + "/time_sync.txt") +
                "not_a_key_value_pair\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceHasMalformedKeyValueLine) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_line_token_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);
  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport() + "not_a_key_value_pair\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsGeneratedKeysStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_keys_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  const char* required_keys[] = {
      "time_sync_keys_status",
      "runtime_deployment_keys_status",
      "runtime_health_keys_status",
      "runtime_stability_summary_keys_status",
      "runtime_stability_run_log_keys_status",
      "power_loss_resume_keys_status",
      "pps_ptp_wiring_keys_status"};
  for (const char* key : required_keys) {
    SCOPED_TRACE(key);
    writeFile(base_dir + "/field_acceptance.txt",
              removeLineWithPrefix(fieldAcceptanceReport(),
                                   std::string(key) + "="));

    const EvidenceBundleReport report =
        evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

    EXPECT_FALSE(report.passed);
    EXPECT_TRUE(report.pps_ptp_wiring_passed);
    EXPECT_TRUE(report.power_loss_resume_passed);
    EXPECT_FALSE(report.field_acceptance_passed);
    EXPECT_NE(std::string::npos,
              report.text.find("field_acceptance_status=FAIL"));
  }
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceGeneratedKeysStatusFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_keys_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  const char* required_keys[] = {
      "time_sync_keys_status",
      "runtime_deployment_keys_status",
      "runtime_health_keys_status",
      "runtime_stability_summary_keys_status",
      "runtime_stability_run_log_keys_status",
      "power_loss_resume_keys_status",
      "pps_ptp_wiring_keys_status"};
  for (const char* key : required_keys) {
    SCOPED_TRACE(key);
    writeFile(base_dir + "/field_acceptance.txt",
              replaceLineWithPrefix(fieldAcceptanceReport(),
                                    std::string(key) + "=",
                                    std::string(key) + "=FAIL"));

    const EvidenceBundleReport report =
        evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

    EXPECT_FALSE(report.passed);
    EXPECT_TRUE(report.pps_ptp_wiring_passed);
    EXPECT_TRUE(report.power_loss_resume_passed);
    EXPECT_FALSE(report.field_acceptance_passed);
    EXPECT_NE(std::string::npos,
              report.text.find("field_acceptance_status=FAIL"));
  }
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringOmitsConfirmationKeysStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_keys_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            removeLineWithPrefix(readFile(base_dir + "/pps_ptp_wiring.txt"),
                                 "wiring_confirmation_keys_status="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringConfirmationKeysStatusFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_keys_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            replaceLineWithPrefix(readFile(base_dir + "/pps_ptp_wiring.txt"),
                                  "wiring_confirmation_keys_status=",
                                  "wiring_confirmation_keys_status=FAIL"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringOmitsConfirmationOverall) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_overall_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            removeLineWithPrefix(readFile(base_dir + "/pps_ptp_wiring.txt"),
                                 "wiring_confirmation_overall="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringConfirmationOverallFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_overall_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            replaceLineWithPrefix(readFile(base_dir + "/pps_ptp_wiring.txt"),
                                  "wiring_confirmation_overall=",
                                  "wiring_confirmation_overall=FAIL"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenManualPowerLossResumeOmitsConfirmationKeysStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_resume_keys_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400",
                                  "PASS", "PASS", "PASS", "metrics.txt",
                                  "manual_file", "PASS"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenManualPowerLossResumeConfirmationKeysStatusFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_resume_keys_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=FAIL\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400",
                                  "PASS", "PASS", "PASS", "metrics.txt",
                                  "manual_file", "PASS"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestSessionIdUsesSentinelValue) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_session_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=__DUPLICATE_KEY__;scenario=POWER_LOSS_ORIGIN;"
      "metrics_report=metrics.txt;event_file=events.txt;bag_file=session.bag;"
      "pcap_file=raw.pcap;tf_snapshot=tf.txt;params_snapshot=params.yaml;"
      "runtime_log=ros.log;time_sync=time_sync.txt;"
      "runtime_health=runtime_health.txt;pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "section_export=section_export.csv;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s1");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestSessionIdIsMalformedToken) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_session_token_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "bad session,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=bad session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "metrics_report", "missing",
                                  "bad session", "POWER_LOSS_ORIGIN"));
  EvidenceManifest manifest = parsePassingEvidenceManifest();
  manifest.session_id = "bad session";

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_TRUE(report.metrics_passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_TRUE(report.section_export_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestScenarioUsesSentinelValue) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_scenario_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s1;scenario=missing;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s1");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestScenarioIsMalformedToken) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_scenario_token_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=s1;scenario=POWER LOSS;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "metrics_report", "missing",
                                  "s1", "POWER LOSS"));
  EvidenceManifest manifest = parsePassingEvidenceManifest();
  manifest.scenario = "POWER LOSS";

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_TRUE(report.metrics_passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_TRUE(report.section_export_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestRuntimeDirUsesSentinelValue) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_runtime_dir_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s1;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;runtime_dir=missing");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestRuntimeDirIsRelativePath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_runtime_dir_relative_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  EvidenceManifest manifest = parsePassingEvidenceManifest();
  manifest.runtime_dir = "relative_runtime";

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_TRUE(report.metrics_passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_TRUE(report.section_export_passed);
  EXPECT_TRUE(report.field_acceptance_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeDirIsFilesystemRootAcrossBundle) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_runtime_dir_root_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  rewriteRuntimeDirEvidence(base_dir, "/");
  EvidenceManifest manifest = parsePassingEvidenceManifest();
  manifest.runtime_dir = "/";

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_health_passed);
  EXPECT_FALSE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeDirContainsDotPathSegmentsAcrossBundle) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_runtime_dir_dot_segment_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);
  const std::string runtime_dir =
      "/tmp/tunnel_lio_runtime/../tunnel_lio_runtime/s1";

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  rewriteRuntimeDirEvidence(base_dir, runtime_dir);
  EvidenceManifest manifest = parsePassingEvidenceManifest();
  manifest.runtime_dir = runtime_dir;

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_health_passed);
  EXPECT_FALSE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeDirContainsCsvSeparatorAcrossBundle) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_runtime_dir_csv_separator_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);
  const std::string runtime_dir = "/tmp/tunnel_lio_runtime/s1,bad";

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  rewriteRuntimeDirEvidence(base_dir, runtime_dir);
  EvidenceManifest manifest = parsePassingEvidenceManifest();
  manifest.runtime_dir = runtime_dir;

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_health_passed);
  EXPECT_FALSE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestScenarioContainsMetadataSeparator) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_scenario_separator_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s1;scenario=POWER_LOSS_ORIGIN\ninjected=PASS;"
      "metrics_report=metrics.txt;event_file=events.txt;"
      "bag_file=session.bag;pcap_file=raw.pcap;tf_snapshot=tf.txt;"
      "params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s1");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestHasUnrelatedDuplicateKey) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_unrelated_duplicate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s1;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "section_export=section_export.csv;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s1;"
      "operator=qa;operator=qa");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.metrics_passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestFilePathUsesSentinelValue) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_file_path_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/__DUPLICATE_KEY__",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=__DUPLICATE_KEY__\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "__DUPLICATE_KEY__"));
  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s1;scenario=POWER_LOSS_ORIGIN;"
      "metrics_report=__DUPLICATE_KEY__;event_file=events.txt;"
      "bag_file=session.bag;pcap_file=raw.pcap;tf_snapshot=tf.txt;"
      "params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "section_export=section_export.csv;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s1");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.missing_files.empty());
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestFilePathContainsMetadataSeparator) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_file_path_separator_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/ros.log\ninjected=PASS", "log-placeholder\n");
  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s1;scenario=POWER_LOSS_ORIGIN;"
      "metrics_report=metrics.txt;event_file=events.txt;"
      "bag_file=session.bag;pcap_file=raw.pcap;tf_snapshot=tf.txt;"
      "params_snapshot=params.yaml;runtime_log=ros.log\ninjected=PASS;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "section_export=section_export.csv;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s1");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.missing_files.empty());
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestEvidenceFileUsesAbsolutePath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_absolute_file_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);
  char external_template[] =
      "/tmp/tunnel_lio_evidence_manifest_absolute_external_XXXXXX";
  char* external = mkdtemp(external_template);
  ASSERT_NE(nullptr, external);
  const std::string external_bag = std::string(external) + "/session.bag";

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(external_bag, "external-bag-placeholder\n");
  EvidenceManifest manifest = parsePassingEvidenceManifest();
  for (EvidenceFileCheck& file : manifest.files) {
    if (file.key == "bag_file") {
      file.path = external_bag;
    }
  }

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_TRUE(report.metrics_passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_TRUE(report.section_export_passed);
  EXPECT_TRUE(report.field_acceptance_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, RejectsParsingMetricsReportFromInvalidManifestPath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_invalid_metrics_path_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);
  char external_template[] =
      "/tmp/tunnel_lio_evidence_manifest_external_metrics_XXXXXX";
  char* external = mkdtemp(external_template);
  ASSERT_NE(nullptr, external);
  const std::string external_metrics =
      std::string(external) + "/metrics.txt";

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(external_metrics,
            "overall=PASS;total_records=1;failed_records=0\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;"
            "failed_checks=0;recovery_time_s=25\n");
  EvidenceManifest manifest = parsePassingEvidenceManifest();
  for (EvidenceFileCheck& file : manifest.files) {
    if (file.key == "metrics_report") {
      file.path = external_metrics;
    }
  }

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestEvidenceFileEscapesBundle) {
  char parent_template[] =
      "/tmp/tunnel_lio_evidence_manifest_escape_parent_XXXXXX";
  char* parent = mkdtemp(parent_template);
  ASSERT_NE(nullptr, parent);
  const std::string base_dir = std::string(parent) + "/bundle";
  const std::string outside_dir = std::string(parent) + "/outside";
  ASSERT_EQ(0, mkdir(base_dir.c_str(), 0700));
  ASSERT_EQ(0, mkdir(outside_dir.c_str(), 0700));

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(outside_dir + "/session.bag", "outside-bag-placeholder\n");
  EvidenceManifest manifest = parsePassingEvidenceManifest();
  for (EvidenceFileCheck& file : manifest.files) {
    if (file.key == "bag_file") {
      file.path = "../outside/session.bag";
    }
  }

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_TRUE(report.metrics_passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_TRUE(report.section_export_passed);
  EXPECT_TRUE(report.field_acceptance_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, PassesWhenFieldAcceptanceUsesManualPowerLossResume) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_manual_resume_ok_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "manual_file", "PASS"));

  const EvidenceManifest manifest = parsePassingEvidenceManifest();
  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_TRUE(report.passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_TRUE(report.field_acceptance_passed);
}

TEST(EvidenceManifest,
     FailsWhenFieldAcceptanceOmitsManualPowerLossResumeConfirmationKeysStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_resume_confirmation_keys_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");
  std::string field_report =
      fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                            "PASS", "PASS", "metrics.txt",
                            "manual_file", "PASS");
  field_report =
      removeLineWithPrefix(field_report,
                           "power_loss_resume_confirmation_keys_status=");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenFieldAcceptanceManualPowerLossResumeConfirmationKeysStatusFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_resume_confirmation_keys_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceOrAppendLineWithPrefix(
                fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400",
                                      "PASS", "PASS", "PASS", "metrics.txt",
                                      "manual_file", "PASS"),
                "power_loss_resume_confirmation_keys_status=",
                "power_loss_resume_confirmation_keys_status=FAIL"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceManualResumeAuditDiffers) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_resume_audit_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");
  std::string field_report =
      fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                            "PASS", "PASS", "metrics.txt",
                            "manual_file", "PASS");
  field_report = removeLineWithPrefix(field_report, "resume_verified_by=");
  field_report = removeLineWithPrefix(field_report, "resume_verified_at=");
  field_report += "resume_verified_by=other_operator\n"
                  "resume_verified_at=2026-06-03T00:00:01+08:00\n";
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenManualFieldAcceptanceReferencesDifferentMetricsReport) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_manual_field_metrics_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "other_metrics.txt",
                                  "manual_file", "PASS"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringUsesAuditSentinelValues) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_audit_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "time_sync_raw=time_status_raw.yaml\n"
            "time_sync_raw_status=PASS\n"
            "time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=missing\n"
            "wiring_verified_at=__DUPLICATE_KEY__\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringAuditFieldContainsCarriageReturn) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_audit_cr_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\r\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringAuditFieldHasWhitespacePollution) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_audit_ws_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator \n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringAuditTimestampIsNotIsoSeconds) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_audit_time_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=not-a-time\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManualPowerLossResumeUsesAuditSentinelValues) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_resume_audit_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=missing\n"
            "resume_verified_at=__DUPLICATE_KEY__\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenManualPowerLossResumeAuditTimestampIsNotIsoSeconds) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_resume_audit_time_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=not-a-time\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "manual_file", "PASS"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceUsesAuditSentinelValues) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_audit_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=PASS\n"
            "power_loss_resume_confirmation_keys_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");

  std::string report_text =
      fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                            "PASS", "PASS", "metrics.txt", "manual_file",
                            "PASS");
  const std::size_t resume_by_pos =
      report_text.find("resume_verified_by=qa_operator");
  ASSERT_NE(std::string::npos, resume_by_pos);
  report_text.replace(resume_by_pos,
                      std::string("resume_verified_by=qa_operator").size(),
                      "resume_verified_by=missing");
  const std::size_t wiring_at_pos =
      report_text.find("wiring_verified_at=2026-06-03T00:00:00+08:00");
  ASSERT_NE(std::string::npos, wiring_at_pos);
  report_text.replace(
      wiring_at_pos,
      std::string("wiring_verified_at=2026-06-03T00:00:00+08:00").size(),
      "wiring_verified_at=__DUPLICATE_KEY__");
  writeFile(base_dir + "/field_acceptance.txt", report_text);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenSectionExportHasMalformedNumericFields) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_bad_section_numbers_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,not_a_number,IDLE_STATIC,A,complete,rmse,points\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.section_export_checked);
  EXPECT_FALSE(report.section_export_passed);
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenSectionExportHasMalformedRowAfterValidRow) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_bad_section_later_row_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n"
      "s1,11.000,IDLE_STATIC,A,invalid,18.000,240\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.section_export_checked);
  EXPECT_FALSE(report.section_export_passed);
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenSectionExportHasUnknownQuality) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_bad_section_quality_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,Z,0.930,18.000,240\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.section_export_checked);
  EXPECT_FALSE(report.section_export_passed);
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenSectionExportHasUnknownStateSource) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_bad_section_state_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,UNKNOWN_STATE,A,0.930,18.000,240\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.section_export_checked);
  EXPECT_FALSE(report.section_export_passed);
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenSectionExportSessionIdDiffersFromManifest) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_bad_section_session_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "other_session,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.section_export_checked);
  EXPECT_FALSE(report.section_export_passed);
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenSectionExportFieldsHaveWhitespacePollution) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_section_whitespace_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240 \n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.section_export_checked);
  EXPECT_FALSE(report.section_export_passed);
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceDurationHasTrailingJunk) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_bad_field_duration_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=PASS\n"
            "time_sync_status=PASS\n"
            "time_capture_status=CAPTURED\n"
            "time_pps_status=PASS\n"
            "time_clock_offset_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "deployment_overall=PASS\n"
            "runtime_stability_status=PASS\n"
            "runtime_stability_overall=PASS\n"
            "runtime_stability_csv_status=PASS\n"
            "runtime_stability_csv_samples=1\n"
            "runtime_stability_sample_count_match=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_overall=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "pps_ptp_wiring_verified=PASS\n"
            "pps_ptp_wiring_overall=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24abc\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceHasMalformedRuntimeStabilitySamples) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_bad_field_samples_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=s3;scenario=LONG_STABILITY;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s1\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "ptp_status=available\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeRuntimeStabilityRunLog(base_dir);
  writeFile(base_dir + "/power_loss_resume.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=PASS\n"
            "time_sync_status=PASS\n"
            "time_capture_status=CAPTURED\n"
            "time_pps_status=PASS\n"
            "time_clock_offset_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "deployment_overall=PASS\n"
            "runtime_stability_status=PASS\n"
            "runtime_stability_overall=PASS\n"
            "runtime_stability_csv_status=PASS\n"
            "runtime_stability_csv_samples=1\n"
            "runtime_stability_sample_count_match=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_overall=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "pps_ptp_wiring_verified=PASS\n"
            "pps_ptp_wiring_overall=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1440abc\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s1;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s1");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRuntimeStabilityCsvStatusFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_csv_status_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("FAIL", "1", "PASS"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsRuntimeStabilityRunLogStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_run_log_status_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "runtime_stability_run_log_status="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_run_log_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_run_log_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRunLogStartedAtDiffersFromRunLog) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_run_log_started_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "runtime_stability_run_log_started_at=",
                "runtime_stability_run_log_started_at=2026-06-03T00:00:02+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_run_log_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_run_log_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsTimeSyncReportPath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_time_sync_path_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(), "time_sync_report="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsPpsPtpWiringReportPath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_wiring_path_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "pps_ptp_wiring_report="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_NE(std::string::npos, report.text.find("pps_ptp_wiring_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsMetricsStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_metrics_status_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(), "metrics_status="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.metrics_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenReplayEventFileDoesNotBindScenario) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_event_scenario_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/events.txt",
            "event=session_start;session_id=s1;t=0.0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenReplayEventFileHasNoMetricEvidence) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_event_metric_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/events.txt",
            replaySessionStartEvent("s1", "POWER_LOSS_ORIGIN"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenReplayEventFileHasNegativeTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_event_negative_time_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/events.txt",
            replaySessionStartEvent("s1", "POWER_LOSS_ORIGIN") +
                "event=static_sample;session_id=s1;"
                "scenario=POWER_LOSS_ORIGIN;t=-1.0;static_drift_m=0.01\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.event_file_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenReplayEventFileHasRecordWithoutSessionId) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_event_session_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/events.txt",
            replaySessionStartEvent("s1", "POWER_LOSS_ORIGIN") +
                "event=static_sample;session_id=s1;"
                "scenario=POWER_LOSS_ORIGIN;t=1.0;static_drift_m=0.01\n"
                "event=queue_sample;scenario=POWER_LOSS_ORIGIN;"
                "t=2.0;queue_backlog=1\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.event_file_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenReplayEventFileHasRecordWithoutScenario) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_event_scenario_record_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/events.txt",
            replaySessionStartEvent("s1", "POWER_LOSS_ORIGIN") +
                "event=static_sample;session_id=s1;"
                "scenario=POWER_LOSS_ORIGIN;t=1.0;static_drift_m=0.01\n"
                "event=queue_sample;session_id=s1;t=2.0;queue_backlog=1\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.event_file_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenReplayEventFileHasDuplicateKeyOutsideMatchingSession) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_event_duplicate_other_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/events.txt",
            replaySessionStartEvent("s1", "POWER_LOSS_ORIGIN") +
                "event=static_sample;session_id=s1;"
                "scenario=POWER_LOSS_ORIGIN;t=1.0;static_drift_m=0.01\n"
                "event=queue_sample;session_id=other;session_id=other2;"
                "scenario=POWER_LOSS_ORIGIN;t=2.0;queue_backlog=1\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.event_file_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenReplayEventFileHasMalformedToken) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_event_malformed_token_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/events.txt",
            replaySessionStartEvent("s1", "POWER_LOSS_ORIGIN") +
                "event=static_sample;session_id=s1;"
                "scenario=POWER_LOSS_ORIGIN;t=1.0;static_drift_m=0.01;"
                "not_a_key_value_pair\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.event_file_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenReplayEventFileHasEmptyToken) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_event_empty_token_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/events.txt",
            replaySessionStartEvent("s1", "POWER_LOSS_ORIGIN") +
                "event=static_sample;session_id=s1;"
                "scenario=POWER_LOSS_ORIGIN;t=1.0;static_drift_m=0.01;\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.event_file_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenReplayMetricEvidenceBelongsToOtherSession) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_event_metric_other_session_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/events.txt",
            replaySessionStartEvent("s1", "POWER_LOSS_ORIGIN") +
                "event=static_sample;session_id=other_session;"
                "scenario=POWER_LOSS_ORIGIN;t=1.0;static_drift_m=0.01\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.event_file_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("event_file_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsEventFileStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_event_status_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "event_file_status="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.event_file_passed);
  EXPECT_NE(std::string::npos, report.text.find("event_file_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsSectionExportReportPath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_section_export_path_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "section_export_report="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.section_export_passed);
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsRuntimeHealthReportPath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_runtime_health_path_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "runtime_health_report="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsRuntimeDeploymentReportPath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_runtime_deployment_path_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "runtime_deployment_report="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_deployment_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsRuntimeStabilityCsvReportPath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_stability_csv_path_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "runtime_stability_csv_report="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsRuntimeStabilitySummaryReportPath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_stability_summary_path_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "runtime_stability_summary_report="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsRuntimeStabilityRunLogReportPath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_stability_run_log_path_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "runtime_stability_run_log_report="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_run_log_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_run_log_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsPowerLossResumeReportPath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_power_loss_path_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "power_loss_resume_report="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRuntimeStabilitySampleCountMismatch) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_sample_match_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "FAIL"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRuntimeStabilityCsvSamplesMalformed) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_csv_samples_bad_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1abc", "PASS"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRuntimeStabilityCsvSamplesDifferFromCsv) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_csv_samples_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "2", "PASS"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRuntimeStabilitySamplesDifferFromSummary) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_summary_samples_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "2"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRuntimeStabilityDurationExceedsSamplesAndInterval) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_duration_overstated_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "60"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsRuntimeStabilityMinDurationStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_min_duration_status_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "runtime_stability_min_duration_status="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceMinDurationIsBelow24h) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_min_duration_low_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report =
      replaceOrAppendLineWithPrefix(fieldAcceptanceReport(),
                                    "runtime_stability_min_duration_h=",
                                    "runtime_stability_min_duration_h=1");
  field_report = replaceOrAppendLineWithPrefix(
      field_report,
      "runtime_stability_min_duration_status=",
      "runtime_stability_min_duration_status=PASS");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRuntimeStabilityIntervalDiffersFromSummary) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_stability_interval_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=60\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeRuntimeStabilityRunLog(base_dir, "/tmp/tunnel_lio_runtime/s1", "1", "60");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRuntimeStabilityDurationDiffersFromSummary) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_stability_duration_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-02T00:00:00+08:00,PASS,PASS,runtime_health.txt\n"
            "2,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=2\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeRuntimeStabilityRunLog(base_dir,
                              "/tmp/tunnel_lio_runtime/s1",
                              "2",
                              "86400",
                              "0",
                              "2026-06-02T00:00:00+08:00",
                              "2026-06-04T00:00:00+08:00");
  std::string field_report =
      fieldAcceptanceReport("PASS", "2", "PASS", "2", "86400");
  field_report = replaceLineWithPrefix(
      field_report,
      "runtime_stability_run_log_started_at=",
      "runtime_stability_run_log_started_at=2026-06-02T00:00:00+08:00");
  field_report = replaceLineWithPrefix(
      field_report,
      "runtime_stability_run_log_finished_at=",
      "runtime_stability_run_log_finished_at=2026-06-04T00:00:00+08:00");
  field_report = replaceLineWithPrefix(
      field_report,
      "runtime_stability_run_log_elapsed_s=",
      "runtime_stability_run_log_elapsed_s=172800");
  field_report = replaceLineWithPrefix(
      field_report,
      "runtime_stability_run_log_required_elapsed_s=",
      "runtime_stability_run_log_required_elapsed_s=172740");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_TRUE(report.runtime_stability_run_log_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRuntimeStabilityOverallFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_stability_overall_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400",
                                  "FAIL", "PASS", "PASS"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePowerLossResumeOverallFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_resume_overall_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400",
                                  "PASS", "FAIL", "PASS"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePpsPtpWiringOverallFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_pps_overall_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400",
                                  "PASS", "PASS", "FAIL"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncMetricsAreNotNumeric) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_bad_time_numbers_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=s3;scenario=LONG_STABILITY;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s12\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=missing\n"
            "mean_offset_ms=not_a_number\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=missing\n"
            "mean_offset_ms=not_a_number\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=time_sync.txt\n");
  writeFile(base_dir + "/runtime_health.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s13\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=PASS\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s12;scenario=TIME_SYNC;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s12");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncMetricsAreNonFinite) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_time_inf_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=inf\n"
            "raw=time_status_raw.yaml\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncOmitsOverallStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_missing_overall_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncOmitsTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_missing_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsTimeSyncTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_time_timestamp_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "time_sync_timestamp="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceTimeSyncTimestampDiffers) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_time_timestamp_differs_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "time_sync_timestamp=",
                "time_sync_timestamp=2026-06-03T00:00:01+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceTimestampPrecedesTimeSync) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_time_timestamp_order_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_sync.txt",
            "timestamp=2026-06-05T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "time_sync_timestamp=",
                "time_sync_timestamp=2026-06-05T00:00:00+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringOmitsTimeSyncTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_time_timestamp_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            removeLineWithPrefix(readFile(base_dir + "/pps_ptp_wiring.txt"),
                                 "time_sync_timestamp="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringTimeSyncTimestampDiffers) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_time_timestamp_differs_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            replaceLineWithPrefix(
                readFile(base_dir + "/pps_ptp_wiring.txt"),
                "time_sync_timestamp=",
                "time_sync_timestamp=2026-06-03T00:00:01+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringOmitsTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_timestamp_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            removeLineWithPrefix(readFile(base_dir + "/pps_ptp_wiring.txt"),
                                 "timestamp="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsPpsPtpWiringTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_pps_timestamp_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "pps_ptp_wiring_timestamp="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePpsPtpWiringTimestampDiffers) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_pps_timestamp_differs_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "pps_ptp_wiring_timestamp=",
                "pps_ptp_wiring_timestamp=2026-06-03T00:00:01+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceTimestampPrecedesPpsPtpWiring) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_pps_timestamp_order_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            replaceLineWithPrefix(
                readFile(base_dir + "/pps_ptp_wiring.txt"),
                "timestamp=",
                "timestamp=2026-06-05T00:00:00+08:00"));
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "pps_ptp_wiring_timestamp=",
                "pps_ptp_wiring_timestamp=2026-06-05T00:00:00+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPowerLossResumeOmitsTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_resume_timestamp_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            removeLineWithPrefix(readFile(base_dir + "/power_loss_resume.txt"),
                                 "timestamp="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenFieldAcceptanceOmitsPowerLossResumeTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_resume_timestamp_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "power_loss_resume_timestamp="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenFieldAcceptancePowerLossResumeTimestampDiffers) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_resume_timestamp_differs_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "power_loss_resume_timestamp=",
                "power_loss_resume_timestamp=2026-06-03T00:00:01+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenFieldAcceptanceTimestampPrecedesPowerLossResume) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_resume_timestamp_order_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            replaceLineWithPrefix(
                readFile(base_dir + "/power_loss_resume.txt"),
                "timestamp=",
                "timestamp=2026-06-05T00:00:00+08:00"));
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "power_loss_resume_timestamp=",
                "power_loss_resume_timestamp=2026-06-05T00:00:00+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncRawCaptureIsMissing) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_missing_raw_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=missing_time_status_raw.yaml\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, PassesWhenTimeSyncRawCaptureUsesAbsoluteBundlePath) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_abs_raw_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  const std::string raw_path = base_dir + "/time_status_raw.yaml";
  writeFile(raw_path, "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            std::string("timestamp=2026-06-03T00:00:00+08:00\n"
                        "time_sync_status=PASS\n"
                        "time_status_topic=/time/status\n"
                        "pps_topic=/time/pps_event\n"
                        "capture_status=CAPTURED\n"
                        "pps_status=PASS\n"
                        "clock_offset_status=PASS\n"
                        "pps_jitter_ms=0.5\n"
                        "mean_offset_ms=1.2\n"
                        "raw=") +
                raw_path + "\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            replaceLineWithPrefix(readFile(base_dir + "/pps_ptp_wiring.txt"),
                                  "time_sync_raw=",
                                  "time_sync_raw=" + raw_path + "\n"));
  std::string field_report =
      replaceLineWithPrefix(fieldAcceptanceReport(), "time_sync_raw=",
                            "time_sync_raw=" + raw_path + "\n");
  field_report =
      replaceLineWithPrefix(field_report, "pps_ptp_wiring_time_sync_raw=",
                            "pps_ptp_wiring_time_sync_raw=" + raw_path + "\n");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_TRUE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=PASS"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncRawCaptureAbsolutePathEscapesBundle) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_escape_raw_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);
  char external_template[] =
      "/tmp/tunnel_lio_evidence_manifest_external_raw_XXXXXX";
  char* external = mkdtemp(external_template);
  ASSERT_NE(nullptr, external);
  const std::string external_raw_path =
      std::string(external) + "/time_status_raw.yaml";

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(external_raw_path, "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            std::string("time_sync_status=PASS\n"
                        "time_status_topic=/time/status\n"
                        "pps_topic=/time/pps_event\n"
                        "capture_status=CAPTURED\n"
                        "pps_status=PASS\n"
                        "clock_offset_status=PASS\n"
                        "pps_jitter_ms=0.5\n"
                        "mean_offset_ms=1.2\n"
                        "raw=") +
                external_raw_path + "\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncNumericMetricContainsCarriageReturn) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_number_cr_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\r\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncNumericMetricHasLeadingWhitespace) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_number_space_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms= 0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsTimeSyncNumbers) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_time_sync_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=PASS\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceHasMalformedStrictGateFields) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_strict_gate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=PASS\n"
            "time_sync_status=PASS\n"
            "time_capture_status=CAPTURED\n"
            "time_pps_status=PASS_BAD\n"
            "time_clock_offset_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "deployment_overall=PASS_BAD\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsDeploymentStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_deployment_status_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(), "deployment_status="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsTimeSyncTopics) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_time_topics_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(
                removeLineWithPrefix(fieldAcceptanceReport(),
                                     "time_status_topic="),
                "pps_topic="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceHasUnrelatedDuplicateKey) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_unrelated_duplicate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report = fieldAcceptanceReport();
  field_report += "operator=qa\n"
                  "operator=qa\n";
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncTopicsUseSentinelValues) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_topic_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=missing\n"
            "pps_topic=__DUPLICATE_KEY__\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncHasUnrelatedDuplicateKey) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_unrelated_duplicate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "operator=qa\n"
            "operator=qa\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("pps_ptp_wiring_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenTimeSyncTopicsAreMalformedRosNames) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_topic_malformed_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=$bad_topic\n"
            "pps_topic=/time//pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  std::string field_report =
      removeLineWithPrefix(fieldAcceptanceReport(), "time_status_topic=");
  field_report = removeLineWithPrefix(field_report, "pps_topic=");
  field_report += "time_status_topic=$bad_topic\n"
                  "pps_topic=/time//pps_event\n";
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceTopicsDifferFromTimeSync) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_time_topic_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report =
      removeLineWithPrefix(fieldAcceptanceReport(), "time_status_topic=");
  field_report = removeLineWithPrefix(field_report, "pps_topic=");
  field_report += "time_status_topic=/other/status\n"
                  "pps_topic=/other/pps_event\n";
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringTopicsDifferFromTimeSync) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_wiring_time_topics_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/other/status\n"
            "pps_topic=/other/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "ptp_status=available\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringMetricsDifferFromTimeSync) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_wiring_time_metrics_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.6\n"
            "mean_offset_ms=1.3\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "ptp_status=available\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringReferencesDifferentTimeSyncReport) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_wiring_time_report_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=other_time_sync.txt\n"
            "ptp_status=available\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringOmitsTimeSyncRawFields) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_wiring_time_raw_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=time_sync.txt\n"
            "ptp_status=available\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePpsPtpWiringTimeSyncReportDiffers) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_wiring_time_report_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report =
      removeLineWithPrefix(fieldAcceptanceReport(),
                           "pps_ptp_wiring_time_sync_report=");
  field_report += "pps_ptp_wiring_time_sync_report=/tmp/other_time_sync_status.txt\n";
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsTimeSyncRawFields) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_time_raw_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report =
      removeLineWithPrefix(fieldAcceptanceReport(), "time_sync_raw=");
  field_report =
      removeLineWithPrefix(field_report, "time_sync_raw_status=");
  field_report =
      removeLineWithPrefix(field_report, "pps_ptp_wiring_time_sync_raw=");
  field_report = removeLineWithPrefix(
      field_report, "pps_ptp_wiring_time_sync_raw_status=");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePpsPtpWiringTimeSyncRawDiffers) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_wiring_time_raw_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report =
      removeLineWithPrefix(fieldAcceptanceReport(),
                           "pps_ptp_wiring_time_sync_raw=");
  field_report += "pps_ptp_wiring_time_sync_raw=other_time_status_raw.yaml\n";
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePpsPtpWiringStatusFieldsDiffer) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_wiring_status_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report =
      removeLineWithPrefix(fieldAcceptanceReport(),
                           "pps_ptp_wiring_time_sync_status=");
  field_report =
      removeLineWithPrefix(field_report, "pps_ptp_wiring_capture_status=");
  field_report =
      removeLineWithPrefix(field_report, "pps_ptp_wiring_pps_status=");
  field_report =
      removeLineWithPrefix(field_report, "pps_ptp_wiring_clock_offset_status=");
  field_report +=
      "pps_ptp_wiring_time_sync_status=FAIL\n"
      "pps_ptp_wiring_capture_status=UNAVAILABLE\n"
      "pps_ptp_wiring_pps_status=FAIL\n"
      "pps_ptp_wiring_clock_offset_status=FAIL\n";
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePpsPtpWiringTopicsDiffer) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_wiring_topic_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report =
      removeLineWithPrefix(fieldAcceptanceReport(),
                           "pps_ptp_wiring_time_status_topic=");
  field_report =
      removeLineWithPrefix(field_report, "pps_ptp_wiring_pps_topic=");
  field_report +=
      "pps_ptp_wiring_time_status_topic=/other/time_status\n"
      "pps_ptp_wiring_pps_topic=/other/pps_event\n";
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePpsPtpWiringMetricsDiffer) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_wiring_metrics_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report =
      removeLineWithPrefix(fieldAcceptanceReport(),
                           "pps_ptp_wiring_pps_jitter_ms=");
  field_report =
      removeLineWithPrefix(field_report, "pps_ptp_wiring_mean_offset_ms=");
  field_report +=
      "pps_ptp_wiring_pps_jitter_ms=0.6\n"
      "pps_ptp_wiring_mean_offset_ms=1.3\n";
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceMetricsDifferFromTimeSync) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_time_metrics_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report =
      removeLineWithPrefix(fieldAcceptanceReport(), "pps_jitter_ms=");
  field_report = removeLineWithPrefix(field_report, "mean_offset_ms=");
  field_report += "pps_jitter_ms=0.6\n"
                  "mean_offset_ms=1.3\n";
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsPpsPtpWiringWhenTimeSyncEvidenceFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_pps_time_sync_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=FAIL\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.time_sync_passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeHealthUsesSentinelStatusFields) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_runtime_health_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=missing\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=__DUPLICATE_KEY__\n"
            "docker_container_status=missing\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_health_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeHealthDirDiffersFromManifest) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_health_wrong_runtime_dir_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/other_runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_health_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeHealthUsesEnvOverrideSources) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_health_env_override_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=env_override\n"
            "docker_container_status=running\n"
            "docker_container_status_source=env_override\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_health_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeHealthOmitsTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_health_missing_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_health_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsRuntimeHealthTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_health_timestamp_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "runtime_health_timestamp="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRuntimeHealthTimestampDiffers) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_health_timestamp_differs_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "runtime_health_timestamp=",
                "runtime_health_timestamp=2026-06-03T00:00:01+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceTimestampPrecedesRuntimeHealth) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_health_timestamp_order_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_health.txt",
            "timestamp=2026-06-05T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "runtime_health_timestamp=",
                "runtime_health_timestamp=2026-06-05T00:00:00+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityWatchdogWasSkipped) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_watchdog_skipped_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=1\n"
            "health_failures=0\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceOrAppendLineWithPrefix(
                fieldAcceptanceReport(),
                "runtime_stability_watchdog_skipped=",
                "runtime_stability_watchdog_skipped=1\n"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, ReportsMissingFilesAndFailedMetrics) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_bad_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=FAIL;total_records=1;failed_records=1\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s2\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=UNAVAILABLE\n"
            "pps_status=FAIL\n"
            "clock_offset_status=FAIL\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=FAIL\n"
            "time_sync_status=FAIL\n"
            "capture_status=UNAVAILABLE\n"
            "pps_status=FAIL\n"
            "clock_offset_status=FAIL\n"
            "wiring_confirmation=FAIL\n");
  writeFile(base_dir + "/runtime_health.txt", "runtime_pid=missing\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_active=inactive\n"
            "docker_container_status=exited\n"
            "runtime_process_status=FAIL\n"
            "deployment_status=FAIL\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,FAIL,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=FAIL\n"
            "samples=1\n"
            "watchdog_failures=1\n");
  writeRuntimeStabilityRunLog(base_dir, "/tmp/tunnel_lio_runtime/s2");
  writeFile(base_dir + "/section_export.csv",
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "s2,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=FAIL\n"
            "power_loss_resume_source=missing\n"
            "recovery_time_s=missing\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=FAIL\n"
            "time_sync_status=FAIL\n"
            "runtime_deployment_status=FAIL\n"
            "runtime_stability_status=FAIL\n"
            "power_loss_resume_status=FAIL\n"
            "pps_ptp_wiring_verified=FAIL\n"
            "systemd_active=inactive\n"
            "docker_container_status=exited\n"
            "runtime_stability_duration_h=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s2;scenario=POWER_LOSS_MICRO_MOVE;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "section_export=section_export.csv;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s2");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  ASSERT_EQ(1u, report.missing_files.size());
  EXPECT_EQ("pcap_file", report.missing_files[0].key);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("time_sync_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_deployment_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("missing_file=pcap_file"));
}

TEST(EvidenceManifest, FailsWhenRequiredEvidenceFileIsEmpty) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_empty_file_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/raw.pcap", "");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  ASSERT_FALSE(report.missing_files.empty());
  EXPECT_EQ("pcap_file", report.missing_files[0].key);
  EXPECT_NE(std::string::npos, report.text.find("missing_file=pcap_file"));
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeHealthFieldsAreEmptyOrMalformed) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_bad_health_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=\n"
            "disk_available_gb=not_numeric\n"
            "systemd_active=\n"
            "docker_container_status=running\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_health_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeHealthPidIsMissing) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_missing_pid_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=missing\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_health_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenRuntimeHealthFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_health_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=missing\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_health_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenSectionExportFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_section_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "other_session,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.section_export_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManifestOmitsSectionExportEvidence) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_missing_section_binding_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s1;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s1");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.section_export_checked);
  EXPECT_FALSE(report.missing_files.empty());
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("evidence_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("missing_file=section_export"));
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenReportOmitsHealthAndSectionStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_missing_status_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=PASS\n"
            "time_sync_status=PASS\n"
            "time_capture_status=CAPTURED\n"
            "time_pps_status=PASS\n"
            "time_clock_offset_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "deployment_overall=PASS\n"
            "runtime_stability_status=PASS\n"
            "runtime_stability_overall=PASS\n"
            "runtime_stability_csv_status=PASS\n"
            "runtime_stability_csv_samples=1\n"
            "runtime_stability_sample_count_match=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_overall=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "pps_ptp_wiring_verified=PASS\n"
            "pps_ptp_wiring_overall=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("section_export_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenReportSessionDiffersFromManifest) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_wrong_session_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "metrics_report", "missing",
                                  "other_session", "POWER_LOSS_ORIGIN"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenRuntimeDirsDifferFromManifest) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_wrong_runtime_dir_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "metrics_report", "missing",
                                  "s1", "POWER_LOSS_ORIGIN",
                                  "/tmp/other_runtime",
                                  "/tmp/tunnel_lio_runtime/s1"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenRuntimeHealthDetailsAreMalformed) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_bad_health_details_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "metrics_report", "missing",
                                  "s1", "POWER_LOSS_ORIGIN",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "not-a-number", "0",
                                  "__DUPLICATE_KEY__", "missing"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenRuntimeHealthDetailsDiffer) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_health_details_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "metrics_report", "missing",
                                  "s1", "POWER_LOSS_ORIGIN",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "121", "5678",
                                  "inactive", "paused"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenRuntimeHealthReportsInactiveProcessState) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_inactive_health_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=inactive\n"
            "docker_container_status=exited\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "metrics_report", "missing",
                                  "s1", "POWER_LOSS_ORIGIN",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "120", "1234", "inactive", "exited"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_health_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsFieldAcceptanceWhenRuntimeDeploymentDetailsAreMalformed) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_bad_deploy_details_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "metrics_report", "missing",
                                  "s1", "POWER_LOSS_ORIGIN",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "120", "1234", "active", "running",
                                  "FAIL", "__DUPLICATE_KEY__", "missing",
                                  "PASS;injected=1", "NO"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsFieldAcceptanceWhenRuntimeProcessStatusIsMalformed) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_bad_runtime_process_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "metrics_report", "missing",
                                  "s1", "POWER_LOSS_ORIGIN",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "120", "1234", "active", "running",
                                  "PASS", "PASS", "PASS", "PASS", "PASS",
                                  "__DUPLICATE_KEY__"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenTimestampIsMalformed) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_bad_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "metrics_report", "missing",
                                  "s1", "POWER_LOSS_ORIGIN",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "120", "1234", "active", "running",
                                  "PASS", "PASS", "PASS", "PASS", "PASS",
                                  "PASS", "__DUPLICATE_KEY__"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenTimestampIsNotIsoSeconds) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_bad_timestamp_format_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "metrics.txt",
                                  "metrics_report", "missing",
                                  "s1", "POWER_LOSS_ORIGIN",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "/tmp/tunnel_lio_runtime/s1",
                                  "120", "1234", "active", "running",
                                  "PASS", "PASS", "PASS", "PASS", "PASS",
                                  "PASS", "not-a-time"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.runtime_health_passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenTimestampCalendarDateIsImpossible) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_bad_calendar_date_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "timestamp=",
                "timestamp=2026-02-31T00:00:00+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.time_sync_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenTimestampPrecedesEvidenceTimes) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_timestamp_order_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "timestamp=",
                "timestamp=2026-06-03T23:59:59+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.runtime_stability_run_log_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenTimestampStatusIsNotPass) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_timestamp_status_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(fieldAcceptanceReport(),
                                  "field_acceptance_timestamp_status=",
                                  "field_acceptance_timestamp_status=FAIL"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.runtime_stability_run_log_passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenRunLogDurationStatusIsNotPass) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_run_log_duration_status_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "runtime_stability_run_log_duration_status=",
                "runtime_stability_run_log_duration_status=FAIL"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_TRUE(report.runtime_stability_run_log_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsMetricsReportWithFailedRecordsDespiteOverallPass) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_metrics_bad_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=2;failed_records=1\n"
            "---\n"
            "session=bad;scenario=STATIC_IDLE;status=FAIL;failed_checks=1\n"
            "static_drift_m=0.2\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
}

TEST(EvidenceManifest, FailsMetricsReportWithSummaryValueWhitespacePollution) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_metrics_summary_ws_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0 \n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsMetricsReportWithMalformedTokenInSummary) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_metrics_summary_token_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0;"
            "not_a_key_value_pair\n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;"
            "failed_checks=0\n"
            "recovery_time_s=25\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsMetricsReportWithMalformedTokenInDetailLine) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_metrics_detail_token_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;"
            "failed_checks=0\n"
            "recovery_time_s=25;not_a_key_value_pair\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsMetricsReportWithoutManifestSessionRecord) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_metrics_wrong_session_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=other;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsMetricsReportWithSignedTotalRecords) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_metrics_signed_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=+1;failed_records=0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
}

TEST(EvidenceManifest, FailsMetricsReportWithOverflowTotalRecords) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_metrics_overflow_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=999999999999999999999999999999999999;"
            "failed_records=0\n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsMetricsReportWithDuplicateKeys) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_metrics_duplicate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=1;failed_records=0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsMetricsReportWithUnrelatedDuplicateKeyInSummary) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_metrics_summary_duplicate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0;"
            "operator=qa;operator=qa\n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsMetricsReportWithUnrelatedDuplicateKeyInRecord) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_metrics_record_duplicate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;"
            "failed_checks=0;operator=qa;operator=qa\n"
            "recovery_time_s=25\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsMetricsReportWithUnrelatedDuplicateKeyInDetailLine) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_metrics_detail_duplicate_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25;operator=qa;operator=qa\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsFieldAcceptanceWhenMetricsReportFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_metrics_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=FAIL;total_records=1;failed_records=1\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsPowerLossResumeWhenMetricsRecoveryTimeHasWhitespacePollution) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_metrics_recovery_ws_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25 \n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.metrics_passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsPowerLossResumeWhenMetricsReportFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_resume_metrics_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=FAIL;total_records=1;failed_records=1\n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.metrics_passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("metrics_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityCsvHasNoSamples) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_empty_stability_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n"
            "---\n"
            "session=s3;scenario=LONG_STABILITY;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s3\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/runtime_health.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s14\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "timestamp=2026-06-03T00:00:01+08:00\n"
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeRuntimeStabilityRunLog(base_dir, "/tmp/tunnel_lio_runtime/s3");
  writeFile(base_dir + "/section_export.csv",
            "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
            "s3,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=PASS\n"
            "time_sync_status=PASS\n"
            "time_capture_status=CAPTURED\n"
            "time_pps_status=PASS\n"
            "time_clock_offset_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "deployment_overall=PASS\n"
            "runtime_stability_status=PASS\n"
            "runtime_stability_overall=PASS\n"
            "runtime_stability_csv_status=PASS\n"
            "runtime_stability_csv_samples=1\n"
            "runtime_stability_sample_count_match=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_overall=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "pps_ptp_wiring_verified=PASS\n"
            "pps_ptp_wiring_overall=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s3;scenario=LONG_STABILITY;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "section_export=section_export.csv;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s3");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.missing_files.empty());
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=PASS"));
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=PASS"));
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityCsvHasFailingSampleStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_bad_stability_sample_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,FAIL,PASS,runtime_health.txt\n"
            "2,2026-06-03T00:01:00+08:00,PASS,SKIP,runtime_health.txt\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_csv_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityRunLogIsMissing) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_missing_stability_run_log_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  ASSERT_EQ(0, unlink((base_dir + "/runtime_stability_run.log").c_str()));

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s1;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "section_export=section_export.csv;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s1");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("missing_file=runtime_stability_run_log"));
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityCsvFieldsHaveWhitespacePollution) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_stability_csv_ws_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt \n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_csv_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityCsvUsesSentinelHealthReport) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_stability_health_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,__DUPLICATE_KEY__\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_csv_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityCsvTimestampIsImpossibleCalendarDate) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_stability_csv_calendar_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-02-31T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityCsvSampleIndexIsNotSequential) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_stability_csv_sample_index_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "2,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityCsvTimestampPrecedesRunLogWindow) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_stability_csv_window_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-02T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityCsvLaterSampleFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_bad_later_stability_sample_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n"
            "2,2026-06-03T00:01:00+08:00,PASS,FAIL,runtime_health.txt\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_csv_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilitySummarySampleCountDiffersFromCsv) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_stability_sample_count_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");

  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n"
            "2,2026-06-03T00:01:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilitySummarySamplesNotNumeric) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_bad_stability_samples_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s13\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/runtime_health.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s13\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=not_a_number\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=PASS\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s13;scenario=LONG_STABILITY;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s13");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilitySummarySamplesFractional) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_fractional_stability_samples_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s14\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/runtime_health.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s14\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1.5\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=PASS\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s14;scenario=LONG_STABILITY;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s14");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilitySummaryHasFailureCounters) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_stability_failure_counters_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s15\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/runtime_health.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s15\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "disk_failures=0\n"
            "watchdog_failures=1\n"
            "health_failures=0\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=PASS\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s15;scenario=LONG_STABILITY;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;runtime_health=runtime_health.txt;"
      "pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s15");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilitySummaryOmitsFailureCounters) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_missing_stability_counters_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilitySummaryZeroCountersHaveLeadingZeros) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_stability_counter_leading_zero_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=00\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilitySummaryOmitsInterval) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_missing_stability_interval_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_NE(std::string::npos, report.text.find("runtime_stability_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenMetricsPowerLossResumeOmitsMetricsReport) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_resume_no_metrics_report_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");

  const EvidenceManifest manifest = parsePassingEvidenceManifest();
  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
}

TEST(EvidenceManifest, FailsWhenMetricsPowerLossResumeReferencesDifferentMetricsReport) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_resume_wrong_metrics_report_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=other_metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");

  const EvidenceManifest manifest = parsePassingEvidenceManifest();
  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
}

TEST(EvidenceManifest, FailsWhenMetricsPowerLossResumeUsesRecoveryTimeFromOtherRecord) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_resume_wrong_recovery_record_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=2;failed_records=0\n"
            "---\n"
            "session=other_session;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=25\n"
            "---\n"
            "session=s1;scenario=POWER_LOSS_ORIGIN;status=PASS;failed_checks=0\n"
            "recovery_time_s=70\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
            "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");

  const EvidenceManifest manifest = parsePassingEvidenceManifest();
  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
  EXPECT_NE(std::string::npos,
            report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceReferencesDifferentMetricsReport) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_wrong_metrics_report_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "1", "PASS", "1", "86400", "PASS",
                                  "PASS", "PASS", "other_metrics.txt"));

  const EvidenceManifest manifest = parsePassingEvidenceManifest();
  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.field_acceptance_passed);
}

TEST(EvidenceManifest, FailsWhenPowerLossResumeTakesTooLong) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_slow_resume_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s5\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=70\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=FAIL\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=FAIL\n"
            "pps_ptp_wiring_verified=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s5;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_health=runtime_health.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s5");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeDeploymentProcessIsNotActive) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_inactive_runtime_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s4\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "systemd_active=inactive\n"
            "docker_container_status=exited\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=inactive\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=exited\n"
            "start_command=PASS\n"
            "runtime_process_status=FAIL\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=FAIL\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=FAIL\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=PASS\n"
            "pps_ptp_wiring_verified=PASS\n"
            "systemd_active=inactive\n"
            "docker_container_status=exited\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s4;scenario=RUNTIME_DEPLOYMENT;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_health=runtime_health.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s4");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_deployment_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_deployment_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeDeploymentUsesSentinelRuntimeDir) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_deployment_runtime_dir_sentinel_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=__DUPLICATE_KEY__\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_deployment_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_deployment_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeDeploymentDirDiffersFromManifest) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_deployment_wrong_runtime_dir_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/other_runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_deployment_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeDeploymentOmitsTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_deployment_missing_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_deployment_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsRuntimeDeploymentTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_deployment_timestamp_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "runtime_deployment_timestamp="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_deployment_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceRuntimeDeploymentTimestampDiffers) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_deployment_timestamp_differs_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "runtime_deployment_timestamp=",
                "runtime_deployment_timestamp=2026-06-03T00:00:01+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_deployment_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceTimestampPrecedesRuntimeDeployment) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_deployment_timestamp_order_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "timestamp=2026-06-05T00:00:00+08:00\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(
                fieldAcceptanceReport(),
                "runtime_deployment_timestamp=",
                "runtime_deployment_timestamp=2026-06-05T00:00:00+08:00"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_deployment_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_deployment_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeDeploymentUsesEnvOverrideEvidence) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_env_deploy_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s5\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=env_override\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=env_override\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=FAIL\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=FAIL\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=PASS\n"
            "pps_ptp_wiring_verified=PASS\n"
            "systemd_active=active\n"
            "docker_container_status=running\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s5;scenario=RUNTIME_DEPLOYMENT;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_health=runtime_health.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s5");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_deployment_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_deployment_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringUsesEnvOverrideEvidence) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_env_pps_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s6\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_env\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=FAIL\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "pps_ptp_wiring_verified=FAIL\n"
            "wiring_confirmation_source=manual_env\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s6;scenario=TIME_SYNC;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_health=runtime_health.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s6");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPpsPtpWiringOmitsManualAuditFields) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_pps_missing_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s10\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "time_sync_raw=time_status_raw.yaml\n"
            "time_sync_raw_status=PASS\n"
            "time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=time_sync.txt\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=FAIL\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_ptp_wiring_verified=FAIL\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s10;scenario=TIME_SYNC;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_health=runtime_health.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s10");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.pps_ptp_wiring_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("pps_ptp_wiring_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsPpsPtpAuditFields) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_field_pps_missing_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s11\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "time_sync_raw=time_status_raw.yaml\n"
            "time_sync_raw_status=PASS\n"
            "time_sync_timestamp=2026-06-03T00:00:00+08:00\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "time_sync_report=time_sync.txt\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=PASS\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=metrics_report\n"
         "metrics_report=metrics.txt\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s11;scenario=TIME_SYNC;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_health=runtime_health.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s11");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("pps_ptp_wiring_status=PASS"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePpsPtpAuditDiffersFromWiring) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_pps_audit_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report =
      removeLineWithPrefix(fieldAcceptanceReport(), "wiring_verified_by=");
  field_report = removeLineWithPrefix(field_report, "wiring_verified_at=");
  field_report += "wiring_verified_by=other_operator\n"
                  "wiring_verified_at=2026-06-03T00:00:01+08:00\n";
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsPpsPtpWiringConfirmationOverall) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_pps_confirmation_overall_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "wiring_confirmation_overall="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePpsPtpWiringConfirmationOverallFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_pps_confirmation_overall_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report = fieldAcceptanceReport();
  if (field_report.find("wiring_confirmation_overall=") == std::string::npos) {
    field_report += "wiring_confirmation_overall=FAIL\n";
  } else {
    field_report = replaceLineWithPrefix(field_report,
                                         "wiring_confirmation_overall=",
                                         "wiring_confirmation_overall=FAIL");
  }
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsPpsPtpWiringConfirmationStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_pps_confirmation_status_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "wiring_confirmation="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePpsPtpWiringConfirmationStatusFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_pps_confirmation_status_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceOrAppendLineWithPrefix(fieldAcceptanceReport(),
                                          "wiring_confirmation=",
                                          "wiring_confirmation=FAIL"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceOmitsPpsPtpWiringConfirmationKeysStatus) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_pps_confirmation_keys_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            removeLineWithPrefix(fieldAcceptanceReport(),
                                 "wiring_confirmation_keys_status="));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptancePpsPtpWiringConfirmationKeysStatusFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_pps_confirmation_keys_fail_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/field_acceptance.txt",
            replaceLineWithPrefix(fieldAcceptanceReport(),
                                  "wiring_confirmation_keys_status=",
                                  "wiring_confirmation_keys_status=FAIL"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.pps_ptp_wiring_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenFieldAcceptanceZeroFailureCountersHaveLeadingZeros) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_counter_leading_zero_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report = replaceLineWithPrefix(
      fieldAcceptanceReport(),
      "runtime_stability_disk_failures=",
      "runtime_stability_disk_failures=00");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityRunLogExitStatusHasLeadingZeros) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_run_log_exit_leading_zero_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeRuntimeStabilityRunLog(base_dir,
                              "/tmp/tunnel_lio_runtime/s1",
                              "1",
                              "86400",
                              "00");
  std::string field_report = replaceLineWithPrefix(
      fieldAcceptanceReport(),
      "runtime_stability_run_log_exit_status=",
      "runtime_stability_run_log_exit_status=00");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_run_log_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_run_log_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityRunLogCaptureExitStatusNonzero) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_run_log_capture_exit_nonzero_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeRuntimeStabilityRunLog(base_dir,
                              "/tmp/tunnel_lio_runtime/s1",
                              "1",
                              "86400",
                              "0",
                              "2026-06-03T00:00:00+08:00",
                              "2026-06-04T00:00:00+08:00",
                              "1");
  std::string field_report = replaceLineWithPrefix(
      fieldAcceptanceReport(),
      "runtime_stability_run_log_capture_exit_status=",
      "runtime_stability_run_log_capture_exit_status=1");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_run_log_passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_run_log_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenFieldAcceptanceRuntimeStabilityRunLogCaptureExitStatusDiffers) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_capture_exit_mismatch_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  std::string field_report = replaceLineWithPrefix(
      fieldAcceptanceReport(),
      "runtime_stability_run_log_capture_exit_status=",
      "runtime_stability_run_log_capture_exit_status=1");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_run_log_passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityRunLogFinishedBeforeStartedAt) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_run_log_time_reversed_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeRuntimeStabilityRunLog(base_dir,
                              "/tmp/tunnel_lio_runtime/s1",
                              "1",
                              "86400",
                              "0",
                              "2026-06-03T00:00:02+08:00",
                              "2026-06-03T00:00:01+08:00");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_run_log_passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_run_log_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityRunLogElapsedIsTooShort) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_run_log_elapsed_short_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeRuntimeStabilityRunLog(base_dir,
                              "/tmp/tunnel_lio_runtime/s1",
                              "1",
                              "86400",
                              "0",
                              "2026-06-03T00:00:00+08:00",
                              "2026-06-03T00:00:01+08:00");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_run_log_passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_run_log_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityCsvTimestampRegresses) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_csv_time_regression_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T12:00:00+08:00,PASS,PASS,runtime_health_1.txt\n"
            "2,2026-06-03T06:00:00+08:00,PASS,PASS,runtime_health_2.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "timestamp=2026-06-03T23:59:59+08:00\n"
            "overall=PASS\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "samples=2\n"
            "interval_s=43200\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  writeRuntimeStabilityRunLog(base_dir,
                              "/tmp/tunnel_lio_runtime/s1",
                              "2",
                              "43200",
                              "0",
                              "2026-06-03T00:00:00+08:00",
                              "2026-06-04T00:00:00+08:00");
  writeFile(base_dir + "/field_acceptance.txt",
            fieldAcceptanceReport("PASS", "2", "PASS", "2", "43200"));

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityCsvReferencesMissingHealthReport) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_csv_missing_health_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,missing_runtime_health.txt\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilityCsvHealthReportContentFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_csv_bad_health_content_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_health.txt",
            "timestamp=2026-06-03T00:00:00+08:00\n"
            "runtime_dir=/tmp/other_runtime\n"
            "disk_available_gb=120\n"
            "runtime_pid=1234\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_csv_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenFieldAcceptanceOmitsRuntimeStabilityCsvFirstTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_csv_first_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  const std::string field_report = removeLineWithPrefix(
      fieldAcceptanceReport(), "runtime_stability_csv_first_timestamp=");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_csv_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenRuntimeStabilitySummaryOmitsTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_summary_missing_timestamp_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.runtime_stability_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("runtime_stability_status=FAIL"));
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenFieldAcceptanceOmitsRuntimeStabilitySummaryTimestamp) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_field_summary_timestamp_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  const std::string field_report = removeLineWithPrefix(
      fieldAcceptanceReport(), "runtime_stability_summary_timestamp=");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest,
     FailsWhenFieldAcceptanceTimestampPrecedesRuntimeStabilitySummary) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_summary_timestamp_future_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.930,18.000,240\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "timestamp=2026-06-05T00:00:00+08:00\n"
            "overall=PASS\n"
            "runtime_dir=/tmp/tunnel_lio_runtime/s1\n"
            "samples=1\n"
            "interval_s=86400\n"
            "disk_failures=0\n"
            "watchdog_failures=0\n"
            "watchdog_skipped=0\n"
            "health_failures=0\n");
  std::string field_report = replaceOrAppendLineWithPrefix(
      fieldAcceptanceReport(),
      "runtime_stability_summary_timestamp=",
      "runtime_stability_summary_timestamp=2026-06-05T00:00:00+08:00");
  writeFile(base_dir + "/field_acceptance.txt", field_report);

  const EvidenceBundleReport report =
      evaluateEvidenceBundle(parsePassingEvidenceManifest(), base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_TRUE(report.runtime_stability_passed);
  EXPECT_FALSE(report.field_acceptance_passed);
  EXPECT_NE(std::string::npos, report.text.find("field_acceptance_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenPowerLossResumeUsesEnvOverrideEvidence) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_env_resume_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s7\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=manual_env\n"
            "recovery_time_s=missing\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=FAIL\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=FAIL\n"
            "power_loss_resume_source=manual_env\n"
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s7;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_health=runtime_health.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s7");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManualPowerLossResumeOmitsRecoveryTime) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_manual_resume_missing_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s8\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=missing\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=FAIL\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=FAIL\n"
            "power_loss_resume_source=manual_file\n"
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s8;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_health=runtime_health.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s8");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManualPowerLossResumeOmitsAuditFields) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_manual_resume_audit_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s10\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=FAIL\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=FAIL\n"
            "power_loss_resume_source=manual_file\n"
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s10;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_health=runtime_health.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s10");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
}

TEST(EvidenceManifest, FailsWhenManualPowerLossResumeConfirmationOverallFails) {
  char root_template[] =
      "/tmp/tunnel_lio_evidence_manifest_manual_resume_bad_overall_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writePassingEvidenceFiles(
      base_dir,
      "session_id,chainage_m,state_source,quality,completeness,rmse_mm,points\n"
      "s1,10.000,IDLE_STATIC,A,0.920,18.000,240\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_confirmation_overall=FAIL\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=25\n"
            "max_recovery_time_s=45\n"
            "resume_verified_by=qa_operator\n"
            "resume_verified_at=2026-06-03T00:00:00+08:00\n");

  const EvidenceManifest manifest = parsePassingEvidenceManifest();
  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
}

TEST(EvidenceManifest, FailsWhenManualPowerLossResumeTakesTooLong) {
  char root_template[] = "/tmp/tunnel_lio_evidence_manifest_manual_resume_slow_XXXXXX";
  char* root = mkdtemp(root_template);
  ASSERT_NE(nullptr, root);
  const std::string base_dir(root);

  writeFile(base_dir + "/metrics.txt",
            "overall=PASS;total_records=1;failed_records=0\n");
  writeFile(base_dir + "/events.txt", "event=session_start;session_id=s9\n");
  writeFile(base_dir + "/session.bag", "bag-placeholder\n");
  writeFile(base_dir + "/raw.pcap", "pcap-placeholder\n");
  writeFile(base_dir + "/tf.txt", "tf-placeholder\n");
  writeFile(base_dir + "/params.yaml", "params-placeholder\n");
  writeFile(base_dir + "/ros.log", "log-placeholder\n");
  writeFile(base_dir + "/time_status_raw.yaml", "diagnostic-status-raw\n");
  writeFile(base_dir + "/time_sync.txt",
            "time_sync_status=PASS\n"
            "time_status_topic=/time/status\n"
            "pps_topic=/time/pps_event\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "raw=time_status_raw.yaml\n");
  writeFile(base_dir + "/pps_ptp_wiring.txt",
            "pps_ptp_wiring_verified=PASS\n"
            "time_sync_status=PASS\n"
            "capture_status=CAPTURED\n"
            "pps_status=PASS\n"
            "clock_offset_status=PASS\n"
            "pps_jitter_ms=0.5\n"
            "mean_offset_ms=1.2\n"
            "wiring_confirmation=PASS\n"
            "wiring_confirmation_overall=PASS\n"
            "wiring_confirmation_keys_status=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n");
  writeFile(base_dir + "/runtime_health.txt",
            "runtime_dir=/tmp/runtime\n"
            "disk_available_gb=120\n"
            "systemd_active=active\n"
            "docker_container_status=running\n");
  writeFile(base_dir + "/runtime_deployment.txt",
            "runtime_dir=/tmp/runtime\n"
            "systemd_unit_file=PASS\n"
            "systemd_env_file=PASS\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_compose_file=PASS\n"
            "docker_env_file=PASS\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "start_command=PASS\n"
            "runtime_process_status=PASS\n"
            "deployment_status=PASS\n");
  writeFile(base_dir + "/runtime_stability.csv",
            "sample,timestamp,disk_guard_status,watchdog_status,health_report\n"
            "1,2026-06-03T00:00:00+08:00,PASS,PASS,runtime_health.txt\n");
  writeFile(base_dir + "/runtime_stability_summary.txt",
            "overall=PASS\n"
            "samples=1\n");
  writeFile(base_dir + "/power_loss_resume.txt",
            "power_loss_resume_status=PASS\n"
            "power_loss_resume_source=manual_file\n"
            "recovery_time_s=70\n"
            "max_recovery_time_s=45\n");
  writeFile(base_dir + "/field_acceptance.txt",
            "field_acceptance_status=FAIL\n"
            "time_sync_status=PASS\n"
            "runtime_deployment_status=PASS\n"
            "runtime_stability_status=PASS\n"
            "power_loss_resume_status=FAIL\n"
            "power_loss_resume_source=manual_file\n"
            "pps_ptp_wiring_verified=PASS\n"
            "wiring_confirmation_source=manual_file\n"
            "pps_wiring_verified=PASS\n"
            "ptp_wiring_verified=PASS\n"
            "wiring_verified_by=qa_operator\n"
            "wiring_verified_at=2026-06-03T00:00:00+08:00\n"
            "systemd_active=active\n"
            "systemd_active_source=systemctl\n"
            "docker_container_status=running\n"
            "docker_container_status_source=docker_inspect\n"
            "runtime_stability_duration_h=24\n"
            "runtime_stability_samples=1\n"
            "runtime_stability_interval_s=86400\n"
            "runtime_stability_disk_failures=0\n"
            "runtime_stability_watchdog_failures=0\n"
            "runtime_stability_watchdog_skipped=0\n"
            "runtime_stability_health_failures=0\n");

  const EvidenceManifest manifest = parseEvidenceManifestRecord(
      "session_id=s9;scenario=POWER_LOSS_ORIGIN;metrics_report=metrics.txt;"
      "event_file=events.txt;bag_file=session.bag;pcap_file=raw.pcap;"
      "tf_snapshot=tf.txt;params_snapshot=params.yaml;runtime_log=ros.log;"
      "time_sync=time_sync.txt;pps_ptp_wiring=pps_ptp_wiring.txt;"
      "runtime_health=runtime_health.txt;"
      "runtime_deployment=runtime_deployment.txt;"
      "runtime_stability_csv=runtime_stability.csv;"
      "runtime_stability_summary=runtime_stability_summary.txt;"
      "runtime_stability_run_log=runtime_stability_run.log;"
      "power_loss_resume=power_loss_resume.txt;"
      "field_acceptance=field_acceptance.txt;"
      "runtime_dir=/tmp/tunnel_lio_runtime/s9");

  const EvidenceBundleReport report = evaluateEvidenceBundle(manifest, base_dir);

  EXPECT_FALSE(report.passed);
  EXPECT_FALSE(report.power_loss_resume_passed);
  EXPECT_NE(std::string::npos,
            report.text.find("power_loss_resume_status=FAIL"));
}

}  // namespace
}  // namespace lio_eval_tools

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
