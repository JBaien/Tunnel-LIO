#include "lio_eval_tools/evidence_manifest.h"
#include "lio_eval_tools/replay_metric_accumulator.h"
#include "lio_eval_tools/validation_metrics.h"

#include <fstream>
#include <sstream>
#include <string>

#include <ros/ros.h>

namespace {

std::string readTextFile(const std::string& path) {
  std::ifstream input(path.c_str());
  if (!input) {
    return "";
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

bool writeTextFile(const std::string& path, const std::string& text) {
  std::ofstream output(path.c_str());
  if (!output) {
    return false;
  }
  output << text;
  return true;
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

double paramDouble(const ros::NodeHandle& nh,
                   const std::string& name,
                   const double fallback) {
  double value = fallback;
  nh.param(name, value, fallback);
  return value;
}

int paramInt(const ros::NodeHandle& nh,
             const std::string& name,
             const int fallback) {
  int value = fallback;
  nh.param(name, value, fallback);
  return value;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "validation_report_node");
  ros::NodeHandle private_nh("~");

  std::string metrics_file;
  std::string event_file;
  std::string report_file;
  std::string scenario_thresholds_file;
  std::string evidence_manifest_file;
  std::string evidence_base_dir;
  private_nh.param<std::string>("metrics_file", metrics_file, "");
  private_nh.param<std::string>("event_file", event_file, "");
  private_nh.param<std::string>("report_file", report_file, "");
  private_nh.param<std::string>("scenario_thresholds_file",
                                scenario_thresholds_file, "");
  private_nh.param<std::string>("evidence_manifest_file",
                                evidence_manifest_file, "");
  private_nh.param<std::string>("evidence_base_dir", evidence_base_dir, "");

  lio_eval_tools::ValidationThresholds thresholds;
  thresholds.max_static_drift_m =
      paramDouble(private_nh, "max_static_drift_m", thresholds.max_static_drift_m);
  thresholds.max_length_error_percent = paramDouble(
      private_nh, "max_length_error_percent", thresholds.max_length_error_percent);
  thresholds.max_recovery_time_s =
      paramDouble(private_nh, "max_recovery_time_s", thresholds.max_recovery_time_s);
  thresholds.max_wrong_loop_count =
      paramInt(private_nh, "max_wrong_loop_count", thresholds.max_wrong_loop_count);
  thresholds.max_queue_backlog =
      paramInt(private_nh, "max_queue_backlog", thresholds.max_queue_backlog);
  thresholds.max_pps_jitter_ms =
      paramDouble(private_nh, "max_pps_jitter_ms", thresholds.max_pps_jitter_ms);

  std::vector<lio_eval_tools::ValidationMetrics> records;
  if (!event_file.empty()) {
    const std::string text = readTextFile(event_file);
    if (text.empty()) {
      ROS_ERROR_STREAM("event_file is empty or unreadable: " << event_file);
      return 2;
    }
    const std::vector<lio_eval_tools::ReplayEvent> events =
        lio_eval_tools::parseReplayEventRecords(text);
    if (events.empty()) {
      ROS_ERROR_STREAM("event_file contains no replay events: " << event_file);
      return 2;
    }
    records.push_back(lio_eval_tools::aggregateReplayMetrics(events));
  } else if (!metrics_file.empty()) {
    const std::string text = readTextFile(metrics_file);
    if (text.empty()) {
      ROS_ERROR_STREAM("metrics_file is empty or unreadable: " << metrics_file);
      return 2;
    }
    records = lio_eval_tools::parseMetricRecords(text);
  } else {
    ROS_ERROR_STREAM("metrics_file or event_file must be provided");
    return 2;
  }

  std::vector<lio_eval_tools::ScenarioValidationThresholds> scenario_thresholds;
  if (!scenario_thresholds_file.empty()) {
    scenario_thresholds = lio_eval_tools::parseScenarioThresholdRecords(
        readTextFile(scenario_thresholds_file));
  }
  const lio_eval_tools::ValidationBatchReport report =
      lio_eval_tools::evaluateValidationBatch(records, thresholds,
                                              scenario_thresholds);

  bool overall_passed = report.passed;
  std::ostringstream combined_report;
  combined_report << report.text;

  if (!evidence_manifest_file.empty()) {
    const std::string manifest_text = readTextFile(evidence_manifest_file);
    const std::string manifest_line = firstDataLine(manifest_text);
    if (manifest_line.empty()) {
      ROS_ERROR_STREAM("evidence_manifest_file contains no manifest records: "
                       << evidence_manifest_file);
      return 2;
    }
    const lio_eval_tools::EvidenceBundleReport evidence_report =
        lio_eval_tools::evaluateEvidenceBundle(
            lio_eval_tools::parseEvidenceManifestRecord(manifest_line),
            evidence_base_dir);
    overall_passed = overall_passed && evidence_report.passed;
    combined_report << "---\n" << evidence_report.text;
  }

  const std::string report_text = combined_report.str();
  if (!report_file.empty() && !writeTextFile(report_file, report_text)) {
    ROS_ERROR_STREAM("failed to write validation report: " << report_file);
    return 3;
  }

  if (overall_passed) {
    ROS_INFO_STREAM(report_text);
    return 0;
  }
  ROS_WARN_STREAM(report_text);
  return 1;
}
