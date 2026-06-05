#include "lio_preprocess/filtering.h"

#include <cmath>

namespace lio_preprocess {

namespace {

bool validRangeConfig(const PointFilterConfig& config) {
  return std::isfinite(config.min_range) && std::isfinite(config.max_range) && config.min_range >= 0.0 &&
         config.max_range >= config.min_range;
}

bool validCropBox(const CropBox& box) {
  return std::isfinite(box.x_min) && std::isfinite(box.x_max) && std::isfinite(box.y_min) &&
         std::isfinite(box.y_max) && std::isfinite(box.z_min) && std::isfinite(box.z_max) &&
         box.x_min <= box.x_max && box.y_min <= box.y_max && box.z_min <= box.z_max;
}

}  // namespace

bool CropBox::contains(double x, double y, double z) const {
  return x_min <= x && x <= x_max && y_min <= y && y <= y_max && z_min <= z && z <= z_max;
}

KeepDecision shouldKeepXYZ(double x, double y, double z, const PointFilterConfig& config) {
  KeepDecision decision;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    decision.reason = "nan";
    return decision;
  }
  if (!validRangeConfig(config)) {
    decision.reason = "range";
    return decision;
  }
  if (config.enable_body_crop && !validCropBox(config.body_crop)) {
    decision.reason = "body";
    return decision;
  }
  const double distance = std::sqrt(x * x + y * y + z * z);
  if (distance < config.min_range || distance > config.max_range) {
    decision.reason = "range";
    return decision;
  }
  if (config.enable_body_crop && config.body_crop.contains(x, y, z)) {
    decision.reason = "body";
    return decision;
  }
  decision.keep = true;
  return decision;
}

std::vector<PointTuple> filterPointTuples(const std::vector<PointTuple>& points, const PointFilterConfig& config, FilterStats* stats) {
  FilterStats local;
  std::vector<PointTuple> kept;
  for (std::vector<PointTuple>::const_iterator it = points.begin(); it != points.end(); ++it) {
    ++local.input_points;
    if (it->size() < 3) {
      ++local.dropped_nan;
      continue;
    }
    const KeepDecision decision = shouldKeepXYZ((*it)[0], (*it)[1], (*it)[2], config);
    if (decision.keep) {
      kept.push_back(*it);
      ++local.output_points;
    } else if (decision.reason == "nan") {
      ++local.dropped_nan;
    } else if (decision.reason == "range") {
      ++local.dropped_range;
    } else if (decision.reason == "body") {
      ++local.dropped_body;
    }
  }
  if (stats != nullptr) {
    *stats = local;
  }
  return kept;
}

}  // namespace lio_preprocess
