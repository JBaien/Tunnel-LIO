#include "mine_slam_calibration/extrinsics.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace mine_slam_calibration {
namespace {

constexpr double kMaxTranslationNormM = 20.0;
constexpr double kMaxUnambiguousRpyRad = 2.0 * M_PI;

bool validFrameName(const std::string& value) {
  return !value.empty() && value[0] != '/';
}

std::array<double, 3> readVector3(const YAML::Node& node) {
  std::array<double, 3> values{{0.0, 0.0, 0.0}};
  if (!node || !node.IsSequence() || node.size() != 3) {
    return values;
  }
  for (std::size_t index = 0; index < 3; ++index) {
    values[index] = node[index].as<double>();
  }
  return values;
}

void auditVector(const TransformSpec& transform,
                 const int index,
                 const std::string& key,
                 const std::array<double, 3>& values,
                 std::vector<std::string>* issues) {
  for (std::size_t offset = 0; offset < values.size(); ++offset) {
    if (!std::isfinite(values[offset])) {
      std::ostringstream stream;
      stream << "transform[" << index << "]." << key << "[" << offset
             << "] must be finite numeric";
      issues->push_back(stream.str());
    }
  }
}

void auditTransformMagnitude(const TransformSpec& transform,
                             const int index,
                             std::vector<std::string>* issues) {
  const double translation_norm = std::sqrt(
      transform.translation[0] * transform.translation[0] +
      transform.translation[1] * transform.translation[1] +
      transform.translation[2] * transform.translation[2]);
  if (translation_norm > kMaxTranslationNormM) {
    std::ostringstream stream;
    stream << "transform[" << index << "].translation norm " << translation_norm
           << " exceeds max " << kMaxTranslationNormM << " m";
    issues->push_back(stream.str());
  }

  for (std::size_t offset = 0; offset < transform.rotation_rpy.size(); ++offset) {
    if (std::fabs(transform.rotation_rpy[offset]) > kMaxUnambiguousRpyRad) {
      std::ostringstream stream;
      stream << "transform[" << index << "].rotation_rpy[" << offset
             << "] exceeds +/-2pi and is ambiguous";
      issues->push_back(stream.str());
    }
  }
}

std::vector<std::string> findCycles(const std::map<std::string, std::string>& child_to_parent) {
  std::set<std::string> issues;
  for (const auto& item : child_to_parent) {
    std::set<std::string> seen;
    std::string current = item.first;
    while (child_to_parent.find(current) != child_to_parent.end()) {
      if (seen.find(current) != seen.end()) {
        issues.insert("TF tree contains a cycle at '" + current + "'");
        break;
      }
      seen.insert(current);
      current = child_to_parent.at(current);
    }
  }
  return std::vector<std::string>(issues.begin(), issues.end());
}

std::string sortedRootsMessage(const std::set<std::string>& roots) {
  std::ostringstream stream;
  stream << "TF tree must have exactly one root, found [";
  bool first = true;
  for (const std::string& root : roots) {
    if (!first) {
      stream << ", ";
    }
    stream << "'" << root << "'";
    first = false;
  }
  stream << "]";
  return stream.str();
}

}  // namespace

ExtrinsicsData loadExtrinsics(const std::string& path) {
  ExtrinsicsData data;
  const YAML::Node root = YAML::LoadFile(path);

  const YAML::Node frames = root["frames"];
  if (frames && frames.IsMap()) {
    for (const auto& item : frames) {
      data.frames.insert(item.first.as<std::string>());
    }
  }

  const YAML::Node calibration = root["calibration"];
  if (calibration && calibration["reference_frame"]) {
    data.reference_frame = calibration["reference_frame"].as<std::string>();
  }

  const YAML::Node required = root["required_frames"];
  if (required && required.IsSequence()) {
    for (const auto& item : required) {
      data.required_frames.push_back(item.as<std::string>());
    }
  }

  const YAML::Node transforms = root["transforms"];
  if (transforms && transforms.IsSequence()) {
    for (const auto& item : transforms) {
      TransformSpec transform;
      if (item["parent"]) {
        transform.parent = item["parent"].as<std::string>();
      }
      if (item["child"]) {
        transform.child = item["child"].as<std::string>();
      }
      transform.translation = readVector3(item["translation"]);
      transform.rotation_rpy = readVector3(item["rotation_rpy"]);
      data.transforms.push_back(transform);
    }
  }

  return data;
}

std::vector<std::string> auditExtrinsics(const ExtrinsicsData& data) {
  std::vector<std::string> issues;
  if (data.frames.empty()) {
    issues.push_back("frames must be a non-empty mapping");
  }
  if (data.transforms.empty()) {
    issues.push_back("transforms must be a non-empty list");
  }

  for (const std::string& frame : data.required_frames) {
    if (data.frames.find(frame) == data.frames.end()) {
      issues.push_back("required frame '" + frame + "' is not declared");
    }
  }

  std::map<std::string, std::string> child_to_parent;
  for (std::size_t index = 0; index < data.transforms.size(); ++index) {
    const TransformSpec& transform = data.transforms[index];
    if (!validFrameName(transform.parent)) {
      issues.push_back("transform[" + std::to_string(index) + "].parent is missing or invalid");
    }
    if (!validFrameName(transform.child)) {
      issues.push_back("transform[" + std::to_string(index) + "].child is missing or invalid");
    }
    if (!transform.parent.empty() && transform.parent == transform.child) {
      issues.push_back("transform[" + std::to_string(index) + "] has identical parent and child");
    }
    if (validFrameName(transform.parent) && data.frames.find(transform.parent) == data.frames.end()) {
      issues.push_back("transform[" + std::to_string(index) + "].parent '" + transform.parent +
                       "' is not declared");
    }
    if (validFrameName(transform.child) && data.frames.find(transform.child) == data.frames.end()) {
      issues.push_back("transform[" + std::to_string(index) + "].child '" + transform.child +
                       "' is not declared");
    }
    if (validFrameName(transform.child)) {
      const auto existing = child_to_parent.find(transform.child);
      if (existing != child_to_parent.end()) {
        issues.push_back("frame '" + transform.child + "' has multiple parents: '" +
                         existing->second + "' and '" + transform.parent + "'");
      }
      child_to_parent[transform.child] = transform.parent;
    }
    auditVector(transform, static_cast<int>(index), "translation", transform.translation, &issues);
    auditVector(transform, static_cast<int>(index), "rotation_rpy", transform.rotation_rpy, &issues);
    auditTransformMagnitude(transform, static_cast<int>(index), &issues);
  }

  std::set<std::string> roots = data.frames;
  for (const auto& item : child_to_parent) {
    roots.erase(item.first);
  }
  if (!data.reference_frame.empty() && data.frames.find(data.reference_frame) == data.frames.end()) {
    issues.push_back("reference_frame '" + data.reference_frame + "' is not declared");
  }
  if (!data.reference_frame.empty() && roots.find(data.reference_frame) == roots.end()) {
    issues.push_back("reference_frame '" + data.reference_frame + "' must be a TF root");
  }
  if (roots.size() != 1) {
    issues.push_back(sortedRootsMessage(roots));
  }

  const std::vector<std::string> cycle_issues = findCycles(child_to_parent);
  issues.insert(issues.end(), cycle_issues.begin(), cycle_issues.end());
  return issues;
}

AuditReport auditExtrinsicsFile(const std::string& path) {
  AuditReport report;
  report.path = path;
  try {
    const ExtrinsicsData data = loadExtrinsics(path);
    report.issues = auditExtrinsics(data);
  } catch (const std::exception& error) {
    report.issues.push_back(std::string("failed to load extrinsics file: ") + error.what());
  }
  report.ok = report.issues.empty();

  std::ostringstream text;
  text << "path=" << report.path << "\n";
  text << "ok=" << (report.ok ? "true" : "false") << "\n";
  text << "issue_count=" << report.issues.size() << "\n";
  for (std::size_t i = 0; i < report.issues.size(); ++i) {
    text << "issue[" << i << "]=" << report.issues[i] << "\n";
  }
  report.text = text.str();
  return report;
}

}  // namespace mine_slam_calibration
