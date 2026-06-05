#pragma once

#include <array>
#include <set>
#include <string>
#include <vector>

namespace mine_slam_calibration {

struct TransformSpec {
  std::string parent;
  std::string child;
  std::array<double, 3> translation{{0.0, 0.0, 0.0}};
  std::array<double, 3> rotation_rpy{{0.0, 0.0, 0.0}};
};

struct ExtrinsicsData {
  std::set<std::string> frames;
  std::vector<std::string> required_frames;
  std::string reference_frame;
  std::vector<TransformSpec> transforms;
};

struct AuditReport {
  bool ok = false;
  std::string path;
  std::vector<std::string> issues;
  std::string text;
};

ExtrinsicsData loadExtrinsics(const std::string& path);
std::vector<std::string> auditExtrinsics(const ExtrinsicsData& data);
AuditReport auditExtrinsicsFile(const std::string& path);

}  // namespace mine_slam_calibration
