#pragma once

#include <string>
#include <vector>

namespace lio_eval_tools {

struct EvidenceFileCheck {
  std::string key;
  std::string path;
};

struct EvidenceManifest {
  std::string session_id;
  std::string scenario;
  std::string runtime_dir;
  bool has_duplicate_keys = false;
  bool has_malformed_tokens = false;
  std::vector<EvidenceFileCheck> files;
};

struct EvidenceBundleReport {
  bool passed = false;
  std::string session_id;
  std::string scenario;
  std::string runtime_dir;
  std::size_t checked_files = 0;
  std::vector<EvidenceFileCheck> missing_files;
  bool metrics_passed = false;
  bool event_file_passed = false;
  bool time_sync_passed = false;
  bool pps_ptp_wiring_passed = false;
  bool runtime_health_passed = false;
  bool runtime_deployment_passed = false;
  bool runtime_stability_csv_passed = false;
  bool runtime_stability_run_log_passed = false;
  bool runtime_stability_passed = false;
  bool power_loss_resume_passed = false;
  bool field_acceptance_passed = false;
  bool section_export_checked = false;
  bool section_export_passed = false;
  std::string text;
};

EvidenceManifest parseEvidenceManifestRecord(const std::string& line);

EvidenceBundleReport evaluateEvidenceBundle(const EvidenceManifest& manifest,
                                             const std::string& base_dir);

}  // namespace lio_eval_tools
