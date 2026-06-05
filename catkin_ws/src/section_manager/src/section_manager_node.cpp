#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>

#include "section_manager/ExportSections.h"
#include "section_manager/control_field_parser.h"
#include "section_manager/section_extraction.h"

namespace section_manager {

class SectionManagerNode {
 public:
  SectionManagerNode() : private_nh_("~") {
    private_nh_.param("slice_thickness_m", config_.slice_thickness_m, 0.2);
    private_nh_.param("angle_bins", config_.angle_bins, 36);
    private_nh_.param("min_points", config_.min_points, 30);
    private_nh_.param("section_model", config_.section_model, std::string("rectangle"));
    private_nh_.param("rectangle_width_m", config_.rectangle_width_m, 4.0);
    private_nh_.param("rectangle_height_m", config_.rectangle_height_m, 2.0);
    private_nh_.param("arch_wall_height_m", config_.arch_wall_height_m, 1.0);
    private_nh_.param("arch_roof_radius_m", config_.arch_roof_radius_m, 2.0);
    private_nh_.param("section_spacing_m", section_spacing_m_, 1.0);
    private_nh_.param("max_export_history", max_export_history_, 1000);
    private_nh_.param("session_id", session_id_, std::string("unassigned"));

    std::string submap_topic;
    std::string mapping_control_topic;
    std::string observed_section_topic;
    std::string structural_section_topic;
    std::string diagnostics_topic;
    std::string export_service;
    private_nh_.param("submap_topic", submap_topic, std::string("/map/local_submap"));
    private_nh_.param("mapping_control_topic", mapping_control_topic, std::string("/mapping/control"));
    private_nh_.param("observed_section_topic", observed_section_topic, std::string("/section/observed"));
    private_nh_.param("structural_section_topic", structural_section_topic, std::string("/section/structural"));
    private_nh_.param("diagnostics_topic", diagnostics_topic, std::string("/diagnostics/section_manager"));
    private_nh_.param("export_service", export_service, std::string("/section/export"));

    observed_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(observed_section_topic, 5);
    structural_pub_ = nh_.advertise<std_msgs::String>(structural_section_topic, 5);
    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic, 5);
    export_srv_ = nh_.advertiseService(export_service, &SectionManagerNode::exportCallback, this);
    submap_sub_ = nh_.subscribe(submap_topic, 2, &SectionManagerNode::submapCallback, this);
    control_sub_ = nh_.subscribe(mapping_control_topic, 10, &SectionManagerNode::controlCallback, this);
  }

 private:
  void submapCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    last_submap_ = *message;
    has_submap_ = true;
  }

  void controlCallback(const std_msgs::StringConstPtr& message) {
    const double chainage = parseStrictDoubleFieldOr(
        message->data, "chainage_m", last_chainage_m_);
    const bool section_sample = parseStrictBoolFieldOr(
        message->data, "section_sample", false);
    const std::string state_source = parseTextFieldOr(
        message->data, "machine_state", "unknown");
    if (!section_sample || !has_submap_) {
      return;
    }

    SectionObservation observation = extractSectionPoints(readCloudPoints(last_submap_), chainage, config_);
    observation.session_id = session_id_;
    observation.state_source = state_source;
    last_observation_ = observation;
    has_observation_ = true;
    if (observation.pointCount() < config_.min_points) {
      publishDiagnostics("insufficient section points", diagnostic_msgs::DiagnosticStatus::WARN);
      return;
    }

    sensor_msgs::PointCloud2 output = createSectionCloud(last_submap_, observation);
    observed_pub_.publish(output);
    std_msgs::String structural;
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(3);
    stream << "session_id=" << observation.session_id
           << ";chainage_m=" << observation.chainage_m
           << ";state_source=" << observation.state_source
           << ";quality=" << observation.quality
           << ";completeness=" << observation.completeness << ";rmse_mm=" << observation.rmse_mm
           << ";points=" << observation.pointCount();
    structural.data = stream.str();
    structural_pub_.publish(structural);
    last_history_update_ =
        upsertSectionObservation(&section_history_, observation, section_spacing_m_);
    if (section_history_.size() > static_cast<std::size_t>(max_export_history_)) {
      section_history_.erase(section_history_.begin());
    }
    last_chainage_m_ = chainage;
    publishDiagnostics("ok", diagnostic_msgs::DiagnosticStatus::OK);
  }

  bool exportCallback(ExportSections::Request& request, ExportSections::Response& response) {
    if (request.output_path.empty()) {
      response.success = false;
      response.message = "output_path is empty";
      response.exported_count = 0;
      return true;
    }
    if (request.min_chainage_m > request.max_chainage_m) {
      response.success = false;
      response.message = "min_chainage_m is greater than max_chainage_m";
      response.exported_count = 0;
      return true;
    }

    const SectionExportResult result =
        exportSectionsCsv(section_history_, request.min_chainage_m, request.max_chainage_m, request.min_quality);
    std::ofstream output(request.output_path.c_str());
    if (!output) {
      response.success = false;
      response.message = "failed to open output_path";
      response.exported_count = 0;
      return true;
    }
    output << result.csv;
    response.success = true;
    response.message = "exported sections";
    response.exported_count = static_cast<uint32_t>(result.exported_count);
    return true;
  }

  std::vector<PointXYZ> readCloudPoints(const sensor_msgs::PointCloud2& cloud) const {
    std::vector<PointXYZ> points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      points.push_back(PointXYZ{static_cast<double>(*iter_x), static_cast<double>(*iter_y), static_cast<double>(*iter_z)});
    }
    return points;
  }

  sensor_msgs::PointCloud2 createSectionCloud(const sensor_msgs::PointCloud2& source, const SectionObservation& observation) const {
    sensor_msgs::PointCloud2 output;
    output.header = source.header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(observation.points_yz.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");
    for (std::vector<PointYZ>::const_iterator point = observation.points_yz.begin(); point != observation.points_yz.end();
         ++point, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = static_cast<float>(observation.chainage_m);
      *iter_y = static_cast<float>(point->y);
      *iter_z = static_cast<float>(point->z);
    }
    return output;
  }

  void publishDiagnostics(const std::string& message, unsigned char level) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "section_manager";
    status.hardware_id = "section_extraction";
    status.level = level;
    status.message = message;
    if (has_observation_) {
      status.values.push_back(keyValue("chainage_m", last_observation_.chainage_m));
      status.values.push_back(keyValue("session_id", last_observation_.session_id));
      status.values.push_back(keyValue("state_source", last_observation_.state_source));
      status.values.push_back(keyValue("point_count", last_observation_.pointCount()));
      status.values.push_back(keyValue("completeness", last_observation_.completeness));
      status.values.push_back(keyValue("rmse_mm", last_observation_.rmse_mm));
      status.values.push_back(keyValue("quality", last_observation_.quality));
      status.values.push_back(keyValue("history_inserted",
                                       last_history_update_.inserted ? "true" : "false"));
      status.values.push_back(keyValue("history_replaced",
                                       last_history_update_.replaced ? "true" : "false"));
      status.values.push_back(keyValue("history_index",
                                       static_cast<int>(last_history_update_.index)));
      status.values.push_back(keyValue("history_size",
                                       static_cast<int>(section_history_.size())));
    }
    array.status.push_back(status);
    diag_pub_.publish(array);
  }

  diagnostic_msgs::KeyValue keyValue(const std::string& key, const std::string& value) const {
    diagnostic_msgs::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
  }

  diagnostic_msgs::KeyValue keyValue(const std::string& key, double value) const {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(3);
    stream << value;
    return keyValue(key, stream.str());
  }

  diagnostic_msgs::KeyValue keyValue(const std::string& key, int value) const {
    return keyValue(key, std::to_string(value));
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher observed_pub_;
  ros::Publisher structural_pub_;
  ros::Publisher diag_pub_;
  ros::ServiceServer export_srv_;
  ros::Subscriber submap_sub_;
  ros::Subscriber control_sub_;
  SectionConfig config_;
  double section_spacing_m_ = 1.0;
  int max_export_history_ = 1000;
  std::string session_id_ = "unassigned";
  sensor_msgs::PointCloud2 last_submap_;
  bool has_submap_ = false;
  double last_chainage_m_ = 0.0;
  SectionObservation last_observation_;
  bool has_observation_ = false;
  std::vector<SectionObservation> section_history_;
  SectionHistoryUpdate last_history_update_;
};

}  // namespace section_manager

int main(int argc, char** argv) {
  ros::init(argc, argv, "section_manager");
  section_manager::SectionManagerNode node;
  ros::spin();
  return 0;
}
