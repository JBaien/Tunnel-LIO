#include "tca_manager/tca_detection.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace tca_manager {
namespace {

double distance(const Point3& a, const Point3& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool validPoint(const Point3& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool validPoint(const PointXYZI& point) {
  return std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.z) &&
         std::isfinite(point.intensity) &&
         point.intensity >= 0.0;
}

bool validPointCloud(const std::vector<PointXYZI>& points) {
  for (const PointXYZI& point : points) {
    if (!validPoint(point)) {
      return false;
    }
  }
  return true;
}

bool validRingEdges(const std::vector<double>& ring_edges) {
  if (ring_edges.size() < 2) {
    return false;
  }
  if (!std::isfinite(ring_edges.front()) || ring_edges.front() < 0.0) {
    return false;
  }
  for (std::size_t index = 1; index < ring_edges.size(); ++index) {
    if (!std::isfinite(ring_edges[index]) || ring_edges[index] <= ring_edges[index - 1]) {
      return false;
    }
  }
  return true;
}

bool validContextSignature(const std::vector<int>& context_signature) {
  if (context_signature.empty()) {
    return false;
  }
  bool has_observation = false;
  for (const int count : context_signature) {
    if (count < 0) {
      return false;
    }
    if (count > 0) {
      has_observation = true;
    }
  }
  return has_observation;
}

bool validDetectionConfig(const TcaDetectionConfig& config) {
  return std::isfinite(config.intensity_threshold) &&
         config.intensity_threshold >= 0.0 &&
         config.min_reflective_points > 0 &&
         std::isfinite(config.cluster_radius_m) &&
         config.cluster_radius_m > 0.0 &&
         validRingEdges(config.context_ring_edges);
}

bool validLedgerConfig(const double min_score, const double min_top_score_ratio) {
  return std::isfinite(min_score) &&
         min_score >= 0.0 &&
         min_score <= 1.0 &&
         std::isfinite(min_top_score_ratio) &&
         min_top_score_ratio >= 1.0;
}

bool validSafeToken(const std::string& value) {
  if (value.empty() || value == "." || value == "..") {
    return false;
  }
  for (const char c : value) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-' || c == '.')) {
      return false;
    }
  }
  return true;
}

bool validAnchor(const TcaAnchor& anchor) {
  return validSafeToken(anchor.anchor_id) &&
         std::isfinite(anchor.chainage_m) &&
         anchor.chainage_m >= 0.0 &&
         validPoint(anchor.center) &&
         validContextSignature(anchor.context_signature) &&
         std::isfinite(anchor.height_m) &&
         anchor.height_m >= 0.0;
}

Point3 centroid(const std::vector<PointXYZI>& points) {
  Point3 result;
  if (points.empty()) {
    return result;
  }

  for (const PointXYZI& point : points) {
    result.x += point.x;
    result.y += point.y;
    result.z += point.z;
  }
  const double count = static_cast<double>(points.size());
  result.x /= count;
  result.y /= count;
  result.z /= count;
  return result;
}

std::vector<std::vector<PointXYZI> > clusterPoints(const std::vector<PointXYZI>& points,
                                                   const double radius) {
  std::vector<std::vector<PointXYZI> > clusters;
  for (const PointXYZI& point : points) {
    bool assigned = false;
    for (std::vector<PointXYZI>& cluster : clusters) {
      const Point3 center = centroid(cluster);
      if (distance(Point3{point.x, point.y, point.z}, center) <= radius) {
        cluster.push_back(point);
        assigned = true;
        break;
      }
    }
    if (!assigned) {
      clusters.push_back(std::vector<PointXYZI>(1, point));
    }
  }
  return clusters;
}

boost::property_tree::ptree pointTree(const Point3& point) {
  boost::property_tree::ptree array;
  boost::property_tree::ptree item;
  item.put("", point.x);
  array.push_back(std::make_pair("", item));
  item.put("", point.y);
  array.push_back(std::make_pair("", item));
  item.put("", point.z);
  array.push_back(std::make_pair("", item));
  return array;
}

boost::property_tree::ptree intArrayTree(const std::vector<int>& values) {
  boost::property_tree::ptree array;
  for (const int value : values) {
    boost::property_tree::ptree item;
    item.put("", value);
    array.push_back(std::make_pair("", item));
  }
  return array;
}

}  // namespace

std::vector<TcaDetection> detectReflectiveTargets(const std::vector<PointXYZI>& points,
                                                  const TcaDetectionConfig& config) {
  if (!validDetectionConfig(config) || !validPointCloud(points)) {
    return std::vector<TcaDetection>{};
  }

  std::vector<PointXYZI> reflective;
  for (const PointXYZI& point : points) {
    if (point.intensity >= config.intensity_threshold) {
      reflective.push_back(point);
    }
  }

  std::vector<TcaDetection> detections;
  const std::vector<std::vector<PointXYZI> > clusters =
      clusterPoints(reflective, config.cluster_radius_m);
  for (const std::vector<PointXYZI>& cluster : clusters) {
    if (static_cast<int>(cluster.size()) < config.min_reflective_points) {
      continue;
    }

    TcaDetection detection;
    detection.center = centroid(cluster);
    detection.reflective_points = static_cast<int>(cluster.size());
    detection.context_signature =
        buildContextSignature(points, detection.center, config.context_ring_edges);
    if (detection.context_signature.ring_counts.empty()) {
      continue;
    }
    detections.push_back(detection);
  }
  return detections;
}

ContextSignature buildContextSignature(const std::vector<PointXYZI>& points,
                                       const Point3& center,
                                       const std::vector<double>& ring_edges) {
  ContextSignature signature;
  if (!validPoint(center) || !validRingEdges(ring_edges) || !validPointCloud(points)) {
    return signature;
  }

  signature.ring_counts.assign(ring_edges.size() - 1, 0);
  for (const PointXYZI& point : points) {
    const double d = distance(Point3{point.x, point.y, point.z}, center);
    for (std::size_t index = 0; index + 1 < ring_edges.size(); ++index) {
      if (ring_edges[index] < d && d <= ring_edges[index + 1]) {
        ++signature.ring_counts[index];
        break;
      }
    }
  }
  if (!validContextSignature(signature.ring_counts)) {
    signature.ring_counts.clear();
  }
  return signature;
}

double contextSimilarity(const std::vector<int>& left, const std::vector<int>& right) {
  if (!validContextSignature(left) || !validContextSignature(right) ||
      left.size() != right.size()) {
    return 0.0;
  }

  double total = 0.0;
  for (std::size_t index = 0; index < left.size(); ++index) {
    const int denom = std::max(std::max(left[index], right[index]), 1);
    total += 1.0 - std::abs(left[index] - right[index]) / static_cast<double>(denom);
  }
  return std::max(0.0, total / static_cast<double>(left.size()));
}

void TcaDetectionCache::update(const std::vector<TcaDetection>& detections) {
  if (detections.empty()) {
    clear();
    return;
  }

  last_detection_count_ = static_cast<int>(detections.size());
  last_detection_ = detections.front();
  has_detection_ = true;
}

void TcaDetectionCache::clear() {
  has_detection_ = false;
  last_detection_count_ = 0;
  last_detection_ = TcaDetection{};
}

bool TcaDetectionCache::buildAnchor(const std::string& anchor_id, TcaAnchor* anchor) const {
  if (!has_detection_ ||
      anchor == nullptr ||
      !validSafeToken(anchor_id) ||
      !validPoint(last_detection_.center) ||
      !validContextSignature(last_detection_.context_signature.ring_counts) ||
      !std::isfinite(last_detection_.center.x) ||
      last_detection_.center.x < 0.0) {
    return false;
  }

  TcaAnchor result;
  result.anchor_id = anchor_id;
  result.chainage_m = last_detection_.center.x;
  result.center = last_detection_.center;
  result.context_signature = last_detection_.context_signature.ring_counts;
  *anchor = result;
  return true;
}

TcaLedger::TcaLedger(const double min_score, const double min_top_score_ratio)
    : min_score_(min_score), min_top_score_ratio_(min_top_score_ratio) {}

void TcaLedger::addAnchor(const TcaAnchor& anchor) {
  if (validAnchor(anchor)) {
    anchors_.push_back(anchor);
  }
}

OptionalTcaMatch TcaLedger::match(const Point3& center,
                                  const std::vector<int>& context_signature) const {
  if (!validLedgerConfig(min_score_, min_top_score_ratio_) ||
      !validPoint(center) ||
      !validContextSignature(context_signature)) {
    return OptionalTcaMatch{};
  }

  std::vector<std::pair<double, TcaAnchor> > scored;
  for (const TcaAnchor& anchor : anchors_) {
    const double candidate_score = score(anchor, center, context_signature);
    if (candidate_score >= min_score_) {
      scored.push_back(std::make_pair(candidate_score, anchor));
    }
  }

  if (scored.empty()) {
    return OptionalTcaMatch{};
  }

  std::sort(scored.begin(), scored.end(),
            [](const std::pair<double, TcaAnchor>& lhs,
               const std::pair<double, TcaAnchor>& rhs) { return lhs.first > rhs.first; });

  if (scored.size() >= 2 && scored[1].first > 0.0 &&
      scored[0].first / scored[1].first < min_top_score_ratio_) {
    return OptionalTcaMatch{};
  }

  OptionalTcaMatch result;
  result.has_value = true;
  result.value.anchor_id = scored[0].second.anchor_id;
  result.value.score = scored[0].first;
  result.value.chainage_m = scored[0].second.chainage_m;
  return result;
}

void TcaLedger::load(const std::string& path) {
  anchors_.clear();
  if (!boost::filesystem::is_regular_file(path)) {
    return;
  }

  boost::property_tree::ptree root;
  try {
    boost::property_tree::read_json(path, root);
  } catch (const std::exception&) {
    return;
  }

  const boost::property_tree::ptree empty;
  for (const auto& item : root.get_child("anchors", empty)) {
    try {
      const boost::property_tree::ptree& data = item.second;
      TcaAnchor anchor;
      anchor.anchor_id = data.get<std::string>("anchor_id", "");
      const auto chainage_m = data.get_optional<double>("chainage_m");
      const auto height_m = data.get_optional<double>("height_m");
      if (!chainage_m || !height_m) {
        continue;
      }
      anchor.chainage_m = *chainage_m;
      anchor.side = data.get<std::string>("side", "unknown");
      anchor.height_m = *height_m;
      anchor.installed_at = data.get<std::string>("installed_at", "");

      std::vector<double> center_values;
      for (const auto& value : data.get_child("center", empty)) {
        center_values.push_back(value.second.get_value<double>());
      }
      const bool has_center = center_values.size() >= 3;
      if (has_center) {
        anchor.center = Point3{center_values[0], center_values[1], center_values[2]};
      }

      for (const auto& value : data.get_child("context_signature", empty)) {
        anchor.context_signature.push_back(value.second.get_value<int>());
      }
      if (has_center) {
        addAnchor(anchor);
      }
    } catch (const std::exception&) {
      continue;
    }
  }
}

void TcaLedger::save(const std::string& path) const {
  const boost::filesystem::path output_path(path);
  if (output_path.has_parent_path()) {
    boost::filesystem::create_directories(output_path.parent_path());
  }

  boost::property_tree::ptree anchors_tree;
  for (const TcaAnchor& anchor : anchors_) {
    boost::property_tree::ptree item;
    item.put("anchor_id", anchor.anchor_id);
    item.put("chainage_m", anchor.chainage_m);
    item.add_child("center", pointTree(anchor.center));
    item.add_child("context_signature", intArrayTree(anchor.context_signature));
    item.put("side", anchor.side);
    item.put("height_m", anchor.height_m);
    item.put("installed_at", anchor.installed_at);
    anchors_tree.push_back(std::make_pair("", item));
  }

  boost::property_tree::ptree root;
  root.add_child("anchors", anchors_tree);
  const std::string tmp_path = path + ".tmp";
  boost::property_tree::write_json(tmp_path, root);
  boost::filesystem::rename(tmp_path, path);
}

double TcaLedger::score(const TcaAnchor& anchor,
                        const Point3& center,
                        const std::vector<int>& context_signature) const {
  if (!validAnchor(anchor) || !validPoint(center) || !validContextSignature(context_signature)) {
    return -std::numeric_limits<double>::infinity();
  }
  const double distance_score = 1.0 / (1.0 + distance(anchor.center, center));
  const double context_score = contextSimilarity(anchor.context_signature, context_signature);
  return 0.55 * context_score + 0.45 * distance_score;
}

}  // namespace tca_manager
