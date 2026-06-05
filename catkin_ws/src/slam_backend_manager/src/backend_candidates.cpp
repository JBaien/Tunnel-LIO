#include "slam_backend_manager/backend_candidates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace slam_backend_manager {
namespace {

const double kPi = 3.14159265358979323846;

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

double distance3(const Point3& left, const Point3& right) {
  const double dx = left.x - right.x;
  const double dy = left.y - right.y;
  const double dz = left.z - right.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double spanSimilarity(const Point3& left, const Point3& right) {
  const double values_left[3] = {left.x, left.y, left.z};
  const double values_right[3] = {right.x, right.y, right.z};
  double best = 1.0;
  for (int index = 0; index < 3; ++index) {
    const double denom = std::max(std::max(std::abs(values_left[index]), std::abs(values_right[index])), 1e-6);
    const double score = 1.0 - std::min(1.0, std::abs(values_left[index] - values_right[index]) / denom);
    best = std::min(best, score);
  }
  return std::max(0.0, best);
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

bool validGeometryDescriptorConfig(const DescriptorConfig& config) {
  return validRingEdges(config.ring_edges) &&
         config.sector_count > 0 &&
         config.max_geometry_bin_count >= 0;
}

bool validIntensityDescriptorConfig(const DescriptorConfig& config) {
  return validRingEdges(config.ring_edges) &&
         config.sector_count > 0 &&
         std::isfinite(config.intensity_quantization) &&
         config.intensity_quantization > 0.0 &&
         config.min_intensity_bin_points > 0;
}

bool validCandidateConfig(const BackendConfig& config) {
  return config.sector_count > 0 &&
         std::isfinite(config.min_loop_score) &&
         config.min_loop_score >= 0.0 &&
         config.min_loop_score <= 1.0 &&
         std::isfinite(config.min_top_score_ratio) &&
         config.min_top_score_ratio >= 1.0 &&
         std::isfinite(config.min_rotation_uniqueness_ratio) &&
         config.min_rotation_uniqueness_ratio >= 1.0 &&
         std::isfinite(config.min_loop_chainage_separation_m) &&
         config.min_loop_chainage_separation_m >= 0.0 &&
         std::isfinite(config.intensity_descriptor_weight) &&
         config.intensity_descriptor_weight >= 0.0 &&
         config.intensity_descriptor_weight <= 1.0;
}

bool finitePoint3(const Point3& point) {
  return std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.z);
}

bool validDescriptorGeometryPoints(const std::vector<Point3>& points) {
  for (const Point3& point : points) {
    if (!finitePoint3(point)) {
      return false;
    }
  }
  return true;
}

bool validDescriptorIntensityPoints(const std::vector<Point3>& points) {
  for (const Point3& point : points) {
    if (!finitePoint3(point) ||
        !std::isfinite(point.intensity) ||
        point.intensity < 0.0) {
      return false;
    }
  }
  return true;
}

bool validDescriptorBins(const std::vector<int>& descriptor) {
  for (const int bin : descriptor) {
    if (bin < 0) {
      return false;
    }
  }
  return true;
}

bool validKeyframeId(const std::string& value) {
  if (value.empty() || value == "." || value == "..") {
    return false;
  }
  for (const char character : value) {
    const bool uppercase = character >= 'A' && character <= 'Z';
    const bool lowercase = character >= 'a' && character <= 'z';
    const bool digit = character >= '0' && character <= '9';
    if (!uppercase && !lowercase && !digit && character != '_' &&
        character != '-' && character != '.') {
      return false;
    }
  }
  return true;
}

bool finiteGeometrySummary(const GeometrySummary& summary) {
  return summary.valid &&
         finitePoint3(summary.centroid) &&
         finitePoint3(summary.span);
}

bool validGeometryVerificationConfig(const BackendConfig& config) {
  return std::isfinite(config.min_geometric_score) &&
         config.min_geometric_score >= 0.0 &&
         config.min_geometric_score <= 1.0 &&
         std::isfinite(config.max_centroid_distance_m) &&
         config.max_centroid_distance_m >= 0.0;
}

bool validIcpVerificationConfig(const BackendConfig& config) {
  return std::isfinite(config.max_icp_rmse_m) &&
         config.max_icp_rmse_m >= 0.0 &&
         std::isfinite(config.min_icp_inlier_ratio) &&
         config.min_icp_inlier_ratio >= 0.0 &&
         config.min_icp_inlier_ratio <= 1.0 &&
         std::isfinite(config.icp_inlier_threshold_m) &&
         config.icp_inlier_threshold_m >= 0.0 &&
         config.icp_iterations > 0 &&
         config.max_icp_points >= 0;
}

bool validIcpPoints(const std::vector<Point3>& points) {
  for (const Point3& point : points) {
    if (!finitePoint3(point)) {
      return false;
    }
  }
  return true;
}

std::vector<int> shiftSectors(const std::vector<int>& descriptor, const int sector_count, const int shift) {
  const int ring_count = static_cast<int>(descriptor.size()) / sector_count;
  std::vector<int> shifted;
  shifted.reserve(descriptor.size());
  for (int ring_index = 0; ring_index < ring_count; ++ring_index) {
    const int offset = ring_index * sector_count;
    for (int sector = 0; sector < sector_count; ++sector) {
      shifted.push_back(descriptor[offset + (sector - shift + sector_count) % sector_count]);
    }
  }
  return shifted;
}

double descriptorSimilarityNoShift(const std::vector<int>& left, const std::vector<int>& right) {
  double total = 0.0;
  for (std::size_t index = 0; index < left.size(); ++index) {
    const int denom = std::max(std::max(left[index], right[index]), 1);
    total += 1.0 - std::abs(left[index] - right[index]) / static_cast<double>(denom);
  }
  return std::max(0.0, total / static_cast<double>(left.size()));
}

double topScoreRatio(const double best, const double second_best) {
  if (best <= 0.0) {
    return 0.0;
  }
  if (second_best <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return best / second_best;
}

std::vector<Point2> toXyPoints(const std::vector<Point3>& points, const int max_points) {
  std::vector<Point2> result;
  if (points.empty()) {
    return result;
  }
  int step = 1;
  if (max_points > 0 && static_cast<int>(points.size()) > max_points) {
    step = std::max(1, static_cast<int>(points.size()) / max_points);
  }
  for (std::size_t index = 0; index < points.size(); index += step) {
    result.push_back(Point2{points[index].x, points[index].y});
    if (max_points > 0 && static_cast<int>(result.size()) >= max_points) {
      break;
    }
  }
  return result;
}

Point2 centroidXy(const std::vector<Point2>& points) {
  Point2 result;
  for (const Point2& point : points) {
    result.x += point.x;
    result.y += point.y;
  }
  const double count = static_cast<double>(points.size());
  result.x /= count;
  result.y /= count;
  return result;
}

double distanceSq(const Point2& left, const Point2& right) {
  const double dx = left.x - right.x;
  const double dy = left.y - right.y;
  return dx * dx + dy * dy;
}

double nearestDistance(const Point2& point, const std::vector<Point2>& target) {
  double best = std::numeric_limits<double>::infinity();
  for (const Point2& candidate : target) {
    best = std::min(best, distanceSq(point, candidate));
  }
  return std::sqrt(best);
}

double rmse(const std::vector<Point2>& source, const std::vector<Point2>& target) {
  double sum = 0.0;
  for (const Point2& point : source) {
    const double d = nearestDistance(point, target);
    sum += d * d;
  }
  return std::sqrt(sum / static_cast<double>(source.size()));
}

std::pair<std::pair<double, double>, Point2> estimate2dTransform(const std::vector<Point2>& source,
                                                                 const std::vector<Point2>& target) {
  const Point2 source_centroid = centroidXy(source);
  const Point2 target_centroid = centroidXy(target);
  double sxx = 0.0;
  double syy = 0.0;
  double sxy = 0.0;
  double syx = 0.0;
  for (std::size_t index = 0; index < source.size(); ++index) {
    const double sx = source[index].x - source_centroid.x;
    const double sy = source[index].y - source_centroid.y;
    const double tx = target[index].x - target_centroid.x;
    const double ty = target[index].y - target_centroid.y;
    sxx += sx * tx;
    syy += sy * ty;
    sxy += sx * ty;
    syx += sy * tx;
  }
  const double angle = std::atan2(sxy - syx, sxx + syy);
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  Point2 translation;
  translation.x = target_centroid.x - (c * source_centroid.x - s * source_centroid.y);
  translation.y = target_centroid.y - (s * source_centroid.x + c * source_centroid.y);
  return std::make_pair(std::make_pair(c, s), translation);
}

Point2 transformXy(const Point2& point, const std::pair<double, double>& rotation, const Point2& translation) {
  return Point2{rotation.first * point.x - rotation.second * point.y + translation.x,
                rotation.second * point.x + rotation.first * point.y + translation.y};
}

std::pair<std::vector<Point2>, int> runIcpFromSeed(const std::vector<Point2>& source,
                                                   const std::vector<Point2>& target,
                                                   const double yaw_seed,
                                                   const int max_iterations) {
  const Point2 source_centroid = centroidXy(source);
  const Point2 target_centroid = centroidXy(target);
  const double c = std::cos(yaw_seed);
  const double s = std::sin(yaw_seed);

  std::vector<Point2> transformed;
  transformed.reserve(source.size());
  for (const Point2& point : source) {
    transformed.push_back(Point2{
        c * (point.x - source_centroid.x) - s * (point.y - source_centroid.y) + target_centroid.x,
        s * (point.x - source_centroid.x) + c * (point.y - source_centroid.y) + target_centroid.y});
  }

  double last_rmse = std::numeric_limits<double>::infinity();
  int used_iterations = 0;
  for (int iteration = 0; iteration < std::max(1, max_iterations); ++iteration) {
    std::vector<Point2> paired_source;
    std::vector<Point2> paired_target;
    for (const Point2& point : transformed) {
      double best = std::numeric_limits<double>::infinity();
      Point2 nearest;
      for (const Point2& candidate : target) {
        const double d = distanceSq(point, candidate);
        if (d < best) {
          best = d;
          nearest = candidate;
        }
      }
      paired_source.push_back(point);
      paired_target.push_back(nearest);
    }
    const std::pair<std::pair<double, double>, Point2> transform =
        estimate2dTransform(paired_source, paired_target);
    for (Point2& point : transformed) {
      point = transformXy(point, transform.first, transform.second);
    }
    const double current_rmse = rmse(transformed, target);
    used_iterations = iteration + 1;
    if (std::abs(last_rmse - current_rmse) < 1e-6) {
      break;
    }
    last_rmse = current_rmse;
  }
  return std::make_pair(transformed, used_iterations);
}

}  // namespace

StableMapPolicy::StableMapPolicy(const std::string& min_quality, const bool require_static_or_forward)
    : min_quality_(min_quality), require_static_or_forward_(require_static_or_forward) {}

bool StableMapPolicy::canPromote(const std::string& machine_state,
                                 const std::string& section_quality,
                                 const bool has_unresolved_loop) const {
  if (has_unresolved_loop) {
    return false;
  }
  if (qualityRank(min_quality_) > qualityRank("C")) {
    return false;
  }
  if (require_static_or_forward_ && machine_state != "IDLE_STATIC" && machine_state != "FWD_MOVE") {
    return false;
  }
  return qualityRank(section_quality) <= qualityRank(min_quality_);
}

HistogramDescriptor makeHistogramDescriptor(const std::vector<Point3>& points,
                                            const std::vector<double>& ring_edges) {
  HistogramDescriptor descriptor;
  if (!validRingEdges(ring_edges) || !validDescriptorGeometryPoints(points)) {
    return descriptor;
  }
  descriptor.bins.assign(ring_edges.size() - 1, 0);
  for (const Point3& point : points) {
    const double radius = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    for (std::size_t index = 0; index + 1 < ring_edges.size(); ++index) {
      if (ring_edges[index] < radius && radius <= ring_edges[index + 1]) {
        ++descriptor.bins[index];
        break;
      }
    }
  }
  return descriptor;
}

HistogramDescriptor makeScanContextDescriptor(const std::vector<Point3>& points,
                                              const std::vector<double>& ring_edges,
                                              const int sector_count) {
  DescriptorConfig config;
  config.ring_edges = ring_edges;
  config.sector_count = sector_count;
  return makeScanContextDescriptor(points, config);
}

HistogramDescriptor makeScanContextDescriptor(const std::vector<Point3>& points,
                                              const DescriptorConfig& config) {
  HistogramDescriptor descriptor;
  if (!validGeometryDescriptorConfig(config) || !validDescriptorGeometryPoints(points)) {
    return descriptor;
  }

  const int ring_count = static_cast<int>(config.ring_edges.size()) - 1;
  descriptor.bins.assign(ring_count * config.sector_count, 0);
  for (const Point3& point : points) {
    const double radius = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    int ring_index = -1;
    for (int index = 0; index < ring_count; ++index) {
      if (config.ring_edges[index] < radius && radius <= config.ring_edges[index + 1]) {
        ring_index = index;
        break;
      }
    }
    if (ring_index < 0) {
      continue;
    }
    double angle = std::atan2(point.y, point.x);
    if (angle < 0.0) {
      angle += 2.0 * kPi;
    }
    const int sector_index =
        std::min(config.sector_count - 1,
                 static_cast<int>(angle / (2.0 * kPi) * config.sector_count));
    int& bin = descriptor.bins[ring_index * config.sector_count + sector_index];
    if (config.max_geometry_bin_count > 0) {
      bin = std::min(config.max_geometry_bin_count, bin + 1);
    } else {
      ++bin;
    }
  }
  return descriptor;
}

HistogramDescriptor makeIntensityScanContextDescriptor(
    const std::vector<Point3>& points,
    const std::vector<double>& ring_edges,
    const int sector_count) {
  DescriptorConfig config;
  config.ring_edges = ring_edges;
  config.sector_count = sector_count;
  return makeIntensityScanContextDescriptor(points, config);
}

HistogramDescriptor makeIntensityScanContextDescriptor(
    const std::vector<Point3>& points,
    const DescriptorConfig& config) {
  HistogramDescriptor descriptor;
  if (!validIntensityDescriptorConfig(config) || !validDescriptorIntensityPoints(points)) {
    return descriptor;
  }

  const int ring_count = static_cast<int>(config.ring_edges.size()) - 1;
  std::vector<double> sums(ring_count * config.sector_count, 0.0);
  std::vector<int> counts(ring_count * config.sector_count, 0);
  descriptor.bins.assign(ring_count * config.sector_count, 0);
  for (const Point3& point : points) {
    const double radius = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    int ring_index = -1;
    for (int index = 0; index < ring_count; ++index) {
      if (config.ring_edges[index] < radius && radius <= config.ring_edges[index + 1]) {
        ring_index = index;
        break;
      }
    }
    if (ring_index < 0) {
      continue;
    }
    double angle = std::atan2(point.y, point.x);
    if (angle < 0.0) {
      angle += 2.0 * kPi;
    }
    const int sector_index =
        std::min(config.sector_count - 1,
                 static_cast<int>(angle / (2.0 * kPi) * config.sector_count));
    const int bin = ring_index * config.sector_count + sector_index;
    sums[bin] += point.intensity;
    ++counts[bin];
  }

  for (std::size_t index = 0; index < descriptor.bins.size(); ++index) {
    if (counts[index] >= config.min_intensity_bin_points) {
      const double mean = sums[index] / static_cast<double>(counts[index]);
      descriptor.bins[index] =
          static_cast<int>(std::round(mean / config.intensity_quantization));
    }
  }
  return descriptor;
}

std::vector<int> makeScanContextRingKey(const std::vector<int>& descriptor,
                                        const int sector_count) {
  if (descriptor.empty() || sector_count <= 0 || descriptor.size() % sector_count != 0 ||
      !validDescriptorBins(descriptor)) {
    return std::vector<int>{};
  }

  const int ring_count = static_cast<int>(descriptor.size()) / sector_count;
  std::vector<int> ring_key(ring_count, 0);
  for (int ring_index = 0; ring_index < ring_count; ++ring_index) {
    const int offset = ring_index * sector_count;
    for (int sector = 0; sector < sector_count; ++sector) {
      ring_key[ring_index] += descriptor[offset + sector];
    }
  }
  return ring_key;
}

double ringKeySimilarity(const std::vector<int>& left,
                         const std::vector<int>& right) {
  if (left.empty() || right.empty() || left.size() != right.size() ||
      !validDescriptorBins(left) || !validDescriptorBins(right)) {
    return 0.0;
  }
  return descriptorSimilarityNoShift(left, right);
}

GeometrySummary makeGeometrySummary(const std::vector<Point3>& points) {
  GeometrySummary summary;
  if (points.empty()) {
    return summary;
  }
  for (const Point3& point : points) {
    if (!finitePoint3(point)) {
      return summary;
    }
  }

  summary.valid = true;
  summary.point_count = static_cast<int>(points.size());
  Point3 min_point = points.front();
  Point3 max_point = points.front();
  for (const Point3& point : points) {
    summary.centroid.x += point.x;
    summary.centroid.y += point.y;
    summary.centroid.z += point.z;
    min_point.x = std::min(min_point.x, point.x);
    min_point.y = std::min(min_point.y, point.y);
    min_point.z = std::min(min_point.z, point.z);
    max_point.x = std::max(max_point.x, point.x);
    max_point.y = std::max(max_point.y, point.y);
    max_point.z = std::max(max_point.z, point.z);
  }
  const double count = static_cast<double>(points.size());
  summary.centroid.x /= count;
  summary.centroid.y /= count;
  summary.centroid.z /= count;
  summary.span = Point3{max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z};
  return summary;
}

OptionalLoopCandidate chooseLoopCandidate(const Keyframe& current,
                                          const std::vector<Keyframe>& candidates,
                                          const BackendConfig& config) {
  if (!validCandidateConfig(config)) {
    return OptionalLoopCandidate{};
  }
  if (!validKeyframeId(current.keyframe_id) ||
      !std::isfinite(current.chainage_m) ||
      current.descriptor.empty() ||
      !validDescriptorBins(current.descriptor) ||
      !validDescriptorBins(current.intensity_descriptor)) {
    return OptionalLoopCandidate{};
  }

  std::vector<std::pair<double, Keyframe> > scored;
  for (const Keyframe& candidate : candidates) {
    if (candidate.keyframe_id == current.keyframe_id) {
      continue;
    }
    if (!validKeyframeId(candidate.keyframe_id) ||
        !std::isfinite(candidate.chainage_m) ||
        candidate.descriptor.empty() ||
        !validDescriptorBins(candidate.descriptor) ||
        !validDescriptorBins(candidate.intensity_descriptor)) {
      continue;
    }
    if (std::abs(candidate.chainage_m - current.chainage_m) <
        config.min_loop_chainage_separation_m) {
      continue;
    }
    const DescriptorMatch geometry_match =
        matchScanContextDescriptor(current.descriptor, candidate.descriptor, config.sector_count);
    if (!geometry_match.valid) {
      continue;
    }
    if (config.min_rotation_uniqueness_ratio > 1.0 &&
        geometry_match.top_score_ratio < config.min_rotation_uniqueness_ratio) {
      continue;
    }
    const double geometry_score = geometry_match.score;
    double score = geometry_score;
    if (config.intensity_descriptor_weight > 0.0 &&
        !current.intensity_descriptor.empty() &&
        !candidate.intensity_descriptor.empty()) {
      const double intensity_score = descriptorSimilarity(
          current.intensity_descriptor, candidate.intensity_descriptor,
          config.sector_count);
      const double weight = std::max(0.0, std::min(1.0, config.intensity_descriptor_weight));
      score = geometry_score * (1.0 - weight) + intensity_score * weight;
    }
    if (score >= config.min_loop_score) {
      scored.push_back(std::make_pair(score, candidate));
    }
  }

  if (scored.empty()) {
    return OptionalLoopCandidate{};
  }
  std::sort(scored.begin(), scored.end(),
            [](const std::pair<double, Keyframe>& lhs,
               const std::pair<double, Keyframe>& rhs) { return lhs.first > rhs.first; });
  if (scored.size() >= 2 && scored[1].first > 0.0 &&
      scored[0].first / scored[1].first < config.min_top_score_ratio) {
    return OptionalLoopCandidate{};
  }
  OptionalLoopCandidate result;
  result.has_value = true;
  result.value.keyframe_id = scored[0].second.keyframe_id;
  result.value.score = scored[0].first;
  result.value.chainage_m = scored[0].second.chainage_m;
  return result;
}

LoopVerification verifyLoopGeometry(const Keyframe& current,
                                    const Keyframe& candidate,
                                    const BackendConfig& config) {
  if (!validGeometryVerificationConfig(config)) {
    return LoopVerification{false, 0.0, std::numeric_limits<double>::infinity(), "invalid_geometry_config"};
  }
  if (!finiteGeometrySummary(current.geometry) || !finiteGeometrySummary(candidate.geometry)) {
    return LoopVerification{false, 0.0, std::numeric_limits<double>::infinity(), "missing_geometry"};
  }

  const double centroid_distance = distance3(current.geometry.centroid, candidate.geometry.centroid);
  if (centroid_distance > config.max_centroid_distance_m) {
    return LoopVerification{false, 0.0, centroid_distance, "centroid_distance_high"};
  }

  const double score = spanSimilarity(current.geometry.span, candidate.geometry.span);
  if (score < config.min_geometric_score) {
    return LoopVerification{false, score, centroid_distance, "geometry_score_low"};
  }
  return LoopVerification{true, score, centroid_distance, "accepted"};
}

IcpVerification verifyLoopIcp(const std::vector<Point3>& current_points,
                              const std::vector<Point3>& candidate_points,
                              const BackendConfig& config) {
  if (!validIcpVerificationConfig(config)) {
    return IcpVerification{false, std::numeric_limits<double>::infinity(), 0.0, 0, "invalid_icp_config"};
  }
  if (!validIcpPoints(current_points) || !validIcpPoints(candidate_points)) {
    return IcpVerification{false, std::numeric_limits<double>::infinity(), 0.0, 0, "invalid_icp_points"};
  }

  const std::vector<Point2> current = toXyPoints(current_points, config.max_icp_points);
  const std::vector<Point2> candidate = toXyPoints(candidate_points, config.max_icp_points);
  if (current.size() < 3 || candidate.size() < 3) {
    return IcpVerification{false, std::numeric_limits<double>::infinity(), 0.0, 0, "insufficient_points"};
  }

  std::vector<Point2> best_transformed;
  int best_iterations = 0;
  double best_rmse = std::numeric_limits<double>::infinity();
  const double seeds[4] = {0.0, kPi / 2.0, kPi, -kPi / 2.0};
  for (const double seed : seeds) {
    const std::pair<std::vector<Point2>, int> run =
        runIcpFromSeed(current, candidate, seed, config.icp_iterations);
    const double run_rmse = rmse(run.first, candidate);
    if (run_rmse < best_rmse) {
      best_rmse = run_rmse;
      best_transformed = run.first;
      best_iterations = run.second;
    }
  }

  std::vector<double> distances;
  for (const Point2& point : best_transformed) {
    distances.push_back(nearestDistance(point, candidate));
  }
  double sum = 0.0;
  int inliers = 0;
  for (const double d : distances) {
    sum += d * d;
    if (d <= config.icp_inlier_threshold_m) {
      ++inliers;
    }
  }
  const double final_rmse = std::sqrt(sum / static_cast<double>(distances.size()));
  const double inlier_ratio = inliers / static_cast<double>(distances.size());
  if (final_rmse > config.max_icp_rmse_m) {
    return IcpVerification{false, final_rmse, inlier_ratio, best_iterations, "icp_rmse_high"};
  }
  if (inlier_ratio < config.min_icp_inlier_ratio) {
    return IcpVerification{false, final_rmse, inlier_ratio, best_iterations, "icp_inlier_ratio_low"};
  }
  return IcpVerification{true, final_rmse, inlier_ratio, best_iterations, "accepted"};
}

double descriptorSimilarity(const std::vector<int>& left,
                            const std::vector<int>& right,
                            const int sector_count) {
  return matchScanContextDescriptor(left, right, sector_count).score;
}

DescriptorMatch matchScanContextDescriptor(const std::vector<int>& left,
                                           const std::vector<int>& right,
                                           const int sector_count) {
  DescriptorMatch result;
  if (left.empty() || right.empty() || left.size() != right.size() ||
      !validDescriptorBins(left) || !validDescriptorBins(right)) {
    result.reason = "invalid_descriptor";
    return result;
  }
  if (sector_count > 1 && left.size() % sector_count == 0) {
    double best = -1.0;
    double second_best = -1.0;
    int best_shift = 0;
    for (int shift = 0; shift < sector_count; ++shift) {
      const double score =
          descriptorSimilarityNoShift(left, shiftSectors(right, sector_count, shift));
      if (score > best + 1e-12) {
        second_best = best;
        best = score;
        best_shift = shift;
      } else if (score > second_best) {
        second_best = score;
      }
    }
    result.valid = true;
    result.score = std::max(0.0, best);
    result.sector_shift = best_shift;
    result.second_best_score = std::max(0.0, second_best);
    result.top_score_ratio = topScoreRatio(result.score, result.second_best_score);
    result.reason = result.top_score_ratio <= 1.0 + 1e-9 ? "ambiguous_rotation" : "accepted";
    return result;
  }
  result.valid = true;
  result.score = descriptorSimilarityNoShift(left, right);
  result.sector_shift = 0;
  result.second_best_score = 0.0;
  result.top_score_ratio = topScoreRatio(result.score, result.second_best_score);
  result.reason = "accepted";
  return result;
}

int qualityRank(const std::string& value) {
  if (value == "A") {
    return 0;
  }
  if (value == "B") {
    return 1;
  }
  if (value == "C") {
    return 2;
  }
  return 99;
}

}  // namespace slam_backend_manager
