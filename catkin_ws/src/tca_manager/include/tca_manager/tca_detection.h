#pragma once

#include <string>
#include <vector>

namespace tca_manager {

struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct PointXYZI {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double intensity = 0.0;
};

struct ContextSignature {
  std::vector<int> ring_counts;
};

struct TcaDetectionConfig {
  double intensity_threshold = 200.0;
  int min_reflective_points = 5;
  double cluster_radius_m = 0.35;
  std::vector<double> context_ring_edges = {0.0, 3.0, 5.0, 8.0};
};

struct TcaDetection {
  Point3 center;
  int reflective_points = 0;
  ContextSignature context_signature;
};

struct TcaAnchor {
  std::string anchor_id;
  double chainage_m = 0.0;
  Point3 center;
  std::vector<int> context_signature;
  std::string side = "unknown";
  double height_m = 0.0;
  std::string installed_at;
};

struct TcaMatch {
  std::string anchor_id;
  double score = 0.0;
  double chainage_m = 0.0;
};

struct OptionalTcaMatch {
  bool has_value = false;
  TcaMatch value;
};

std::vector<TcaDetection> detectReflectiveTargets(const std::vector<PointXYZI>& points,
                                                  const TcaDetectionConfig& config);
ContextSignature buildContextSignature(const std::vector<PointXYZI>& points,
                                       const Point3& center,
                                       const std::vector<double>& ring_edges);
double contextSimilarity(const std::vector<int>& left, const std::vector<int>& right);

class TcaDetectionCache {
 public:
  void update(const std::vector<TcaDetection>& detections);
  void clear();

  bool hasDetection() const { return has_detection_; }
  int lastDetectionCount() const { return last_detection_count_; }
  const TcaDetection& detection() const { return last_detection_; }
  bool buildAnchor(const std::string& anchor_id, TcaAnchor* anchor) const;

 private:
  bool has_detection_ = false;
  int last_detection_count_ = 0;
  TcaDetection last_detection_;
};

class TcaLedger {
 public:
  TcaLedger(double min_score = 0.5, double min_top_score_ratio = 1.5);

  void addAnchor(const TcaAnchor& anchor);
  OptionalTcaMatch match(const Point3& center, const std::vector<int>& context_signature) const;
  void load(const std::string& path);
  void save(const std::string& path) const;
  const std::vector<TcaAnchor>& anchors() const { return anchors_; }

 private:
  double score(const TcaAnchor& anchor,
               const Point3& center,
               const std::vector<int>& context_signature) const;

  double min_score_ = 0.5;
  double min_top_score_ratio_ = 1.5;
  std::vector<TcaAnchor> anchors_;
};

}  // namespace tca_manager
