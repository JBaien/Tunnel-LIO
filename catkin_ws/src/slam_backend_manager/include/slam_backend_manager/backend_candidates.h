#pragma once

#include <string>
#include <vector>

namespace slam_backend_manager {

struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double intensity = 0.0;
};

struct HistogramDescriptor {
  std::vector<int> bins;
};

struct DescriptorConfig {
  std::vector<double> ring_edges = {0.0, 5.0, 10.0, 20.0, 40.0};
  int sector_count = 12;
  int max_geometry_bin_count = 0;
  double intensity_quantization = 1.0;
  int min_intensity_bin_points = 1;
};

struct GeometrySummary {
  bool valid = false;
  int point_count = 0;
  Point3 centroid;
  Point3 span;
};

struct Keyframe {
  std::string keyframe_id;
  double chainage_m = 0.0;
  std::vector<int> descriptor;
  GeometrySummary geometry;
  std::vector<Point3> sample_points;
  std::vector<int> intensity_descriptor;
};

struct BackendConfig {
  std::vector<double> ring_edges = {0.0, 5.0, 10.0, 20.0, 40.0};
  int sector_count = 12;
  int max_descriptor_bin_count = 0;
  double intensity_quantization = 1.0;
  int min_intensity_bin_points = 1;
  double min_loop_score = 0.5;
  double min_top_score_ratio = 1.5;
  double min_loop_chainage_separation_m = 10.0;
  double min_geometric_score = 0.75;
  double max_centroid_distance_m = 0.5;
  double max_icp_rmse_m = 0.15;
  double min_icp_inlier_ratio = 0.8;
  double icp_inlier_threshold_m = 0.25;
  int icp_iterations = 20;
  int max_icp_points = 300;
  double intensity_descriptor_weight = 0.0;
};

struct LoopCandidate {
  std::string keyframe_id;
  double score = 0.0;
  double chainage_m = 0.0;
};

struct OptionalLoopCandidate {
  bool has_value = false;
  LoopCandidate value;
};

struct LoopVerification {
  bool accepted = false;
  double score = 0.0;
  double centroid_distance_m = 0.0;
  std::string reason;
};

struct IcpVerification {
  bool accepted = false;
  double rmse_m = 0.0;
  double inlier_ratio = 0.0;
  int iterations = 0;
  std::string reason;
};

class StableMapPolicy {
 public:
  StableMapPolicy(const std::string& min_quality = "B", bool require_static_or_forward = true);
  bool canPromote(const std::string& machine_state,
                  const std::string& section_quality,
                  bool has_unresolved_loop) const;

 private:
  std::string min_quality_;
  bool require_static_or_forward_ = true;
};

HistogramDescriptor makeHistogramDescriptor(const std::vector<Point3>& points,
                                            const std::vector<double>& ring_edges);
HistogramDescriptor makeScanContextDescriptor(const std::vector<Point3>& points,
                                              const std::vector<double>& ring_edges,
                                              int sector_count);
HistogramDescriptor makeScanContextDescriptor(const std::vector<Point3>& points,
                                              const DescriptorConfig& config);
HistogramDescriptor makeIntensityScanContextDescriptor(const std::vector<Point3>& points,
                                                       const std::vector<double>& ring_edges,
                                                       int sector_count);
HistogramDescriptor makeIntensityScanContextDescriptor(const std::vector<Point3>& points,
                                                       const DescriptorConfig& config);
GeometrySummary makeGeometrySummary(const std::vector<Point3>& points);
OptionalLoopCandidate chooseLoopCandidate(const Keyframe& current,
                                          const std::vector<Keyframe>& candidates,
                                          const BackendConfig& config);
LoopVerification verifyLoopGeometry(const Keyframe& current,
                                    const Keyframe& candidate,
                                    const BackendConfig& config);
IcpVerification verifyLoopIcp(const std::vector<Point3>& current_points,
                              const std::vector<Point3>& candidate_points,
                              const BackendConfig& config);
double descriptorSimilarity(const std::vector<int>& left,
                            const std::vector<int>& right,
                            int sector_count = 0);
int qualityRank(const std::string& value);

}  // namespace slam_backend_manager
