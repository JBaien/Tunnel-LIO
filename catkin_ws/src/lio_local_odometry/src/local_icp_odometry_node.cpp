#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <XmlRpcValue.h>

#include "lio_local_odometry/local_odometry_math.h"
#include "lio_local_odometry/local_registration.h"

namespace {

using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

std::string doubleToString(double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << value;
  return stream.str();
}

diagnostic_msgs::KeyValue kv(const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

geometry_msgs::Pose eigenToPose(const Eigen::Affine3d& pose) {
  geometry_msgs::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.rotation());
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
  return msg;
}

std::vector<Eigen::Vector3d> cloudToEigenPoints(const CloudT& cloud) {
  std::vector<Eigen::Vector3d> points;
  points.reserve(cloud.size());
  for (const auto& point : cloud.points) {
    points.emplace_back(point.x, point.y, point.z);
  }
  return points;
}

std::vector<double> doubleVectorParam(ros::NodeHandle& nh,
                                      const std::string& name,
                                      const std::vector<double>& defaults) {
  XmlRpc::XmlRpcValue raw;
  if (!nh.getParam(name, raw) || raw.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return defaults;
  }

  std::vector<double> values;
  values.reserve(raw.size());
  for (int i = 0; i < raw.size(); ++i) {
    if (raw[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      values.push_back(static_cast<double>(raw[i]));
    } else if (raw[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      values.push_back(static_cast<int>(raw[i]));
    }
  }
  return values.empty() ? defaults : values;
}

}  // namespace

namespace lio_local_odometry {

class LocalIcpOdometryNode {
 public:
  LocalIcpOdometryNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
      : nh_(nh), private_nh_(private_nh) {
    loadParameters();
    cloud_sub_ = nh_.subscribe(input_cloud_topic_,
                               std::max(1, cloud_queue_size_),
                               &LocalIcpOdometryNode::cloudCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    submap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(submap_topic_, 2);
    diagnostics_pub_ =
        nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 5);
    diagnostics_timer_ = nh_.createTimer(
        ros::Duration(diagnostics_period_sec_),
        &LocalIcpOdometryNode::diagnosticsTimerCallback, this);

    pose_.setIdentity();
    last_keyframe_.reset(new CloudT);
    local_submap_.reset(new CloudT);
  }

 private:
  void loadParameters() {
    private_nh_.param("input_cloud_topic", input_cloud_topic_,
                      std::string("/lio/points_deskewed"));
    private_nh_.param("odom_topic", odom_topic_, std::string("/lio/odom_local"));
    private_nh_.param("submap_topic", submap_topic_,
                      std::string("/map/local_submap"));
    private_nh_.param("diagnostics_topic", diagnostics_topic_,
                      std::string("/diagnostics/lio_local_odometry"));
    private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
    private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
    private_nh_.param("min_points", min_points_, 100);
    private_nh_.param("max_fitness_score", max_fitness_score_, 0.5);
    private_nh_.param("min_geometry_eigen_ratio",
                      min_geometry_eigen_ratio_, 0.01);
    private_nh_.param("min_geometry_score", min_geometry_score_, 0.2);
    private_nh_.param("voxel_leaf_size", voxel_leaf_size_, 0.10);
    private_nh_.param("registration_method", registration_method_,
                      std::string("icp"));
    registration_voxel_leaf_sizes_ =
        doubleVectorParam(private_nh_, "registration_voxel_leaf_sizes",
                          std::vector<double>{0.30, 0.15, 0.0});
    gicp_voxel_leaf_sizes_ =
        doubleVectorParam(private_nh_, "gicp_voxel_leaf_sizes",
                          std::vector<double>{0.30, 0.15, 0.0});
    ndt_resolutions_ =
        doubleVectorParam(private_nh_, "ndt_resolutions",
                          std::vector<double>{1.0, 0.5, 0.25});
    private_nh_.param("ndt_step_size", ndt_step_size_, 0.1);
    private_nh_.param("ndt_refinement_iterations",
                      ndt_refinement_iterations_, 30);
    private_nh_.param("max_correspondence_distance",
                      max_correspondence_distance_, 1.0);
    private_nh_.param("transformation_epsilon", transformation_epsilon_, 1e-4);
    private_nh_.param("euclidean_fitness_epsilon", euclidean_fitness_epsilon_,
                      1e-3);
    private_nh_.param("max_iterations", max_iterations_, 50);
    private_nh_.param("enable_constant_velocity_initial_guess",
                      enable_constant_velocity_initial_guess_, false);
    private_nh_.param("max_initial_guess_translation",
                      max_initial_guess_translation_, 5.0);
    private_nh_.param("max_initial_guess_scale",
                      max_initial_guess_scale_, 4.0);
    private_nh_.param("publish_submap", publish_submap_, true);
    private_nh_.param("submap_voxel_leaf_size", submap_voxel_leaf_size_, 0.20);
    private_nh_.param("max_submap_points", max_submap_points_, 200000);
    private_nh_.param("min_stable_submap_points", min_stable_submap_points_,
                      1000);
    private_nh_.param("min_submap_observability",
                      min_submap_observability_, 0.5);
    private_nh_.param("max_consecutive_registration_rejections",
                      max_consecutive_registration_rejections_, 2);
    private_nh_.param("diagnostics_period_sec", diagnostics_period_sec_, 1.0);
    private_nh_.param("cloud_queue_size", cloud_queue_size_, 5);
    private_nh_.param("reseed_keyframe_after_consecutive_rejections",
                      reseed_keyframe_after_consecutive_rejections_, 0);
    private_nh_.param("publish_odometry_on_keyframe_reseed",
                      publish_odometry_on_keyframe_reseed_, true);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    ++cloud_count_;
    last_stamp_ = msg->header.stamp;

    CloudT::Ptr input(new CloudT);
    pcl::fromROSMsg(*msg, *input);

    CloudT::Ptr filtered = filterCloud(input);
    last_input_points_ = input->size();
    last_filtered_points_ = filtered->size();
    last_geometry_observability_ = evaluateGeometryObservability(
        cloudToEigenPoints(*filtered), static_cast<std::size_t>(min_points_),
        min_geometry_eigen_ratio_);

    if (filtered->size() < static_cast<std::size_t>(min_points_)) {
      ++dropped_clouds_;
      last_status_message_ = "insufficient filtered points";
      last_registration_usable_ = false;
      return;
    }

    if (!initialized_) {
      *last_keyframe_ = *filtered;
      last_keyframe_stamp_ = msg->header.stamp;
      initialized_ = true;
      last_registration_usable_ = true;
      last_status_message_ = "initialized";
      last_observability_score_ = last_geometry_observability_.score;
      appendToSubmap(filtered);
      updateSubmapQuality();
      publishOdometry(msg->header.stamp);
      publishSubmap(msg->header.stamp);
      return;
    }

    const Eigen::Matrix4f initial_guess =
        makeRegistrationInitialGuess(msg->header.stamp);
    last_initial_guess_translation_ =
        initial_guess.block<3, 1>(0, 3).norm();

    MultiScaleIcpResult registration_result;
    if (registration_method_ == "ndt") {
      MultiScaleNdtConfig registration_config;
      registration_config.resolutions = ndt_resolutions_;
      registration_config.step_size = ndt_step_size_;
      registration_config.max_correspondence_distance =
          max_correspondence_distance_;
      registration_config.transformation_epsilon = transformation_epsilon_;
      registration_config.max_iterations = max_iterations_;
      registration_config.refinement_iterations = ndt_refinement_iterations_;
      registration_config.min_points = static_cast<std::size_t>(min_points_);
      const MultiScaleNdtRegistration registration(registration_config);
      registration_result =
          registration.align(filtered, last_keyframe_, initial_guess);
    } else if (registration_method_ == "gicp") {
      MultiScaleGicpConfig registration_config;
      registration_config.voxel_leaf_sizes = gicp_voxel_leaf_sizes_;
      registration_config.max_correspondence_distance =
          max_correspondence_distance_;
      registration_config.transformation_epsilon = transformation_epsilon_;
      registration_config.euclidean_fitness_epsilon =
          euclidean_fitness_epsilon_;
      registration_config.max_iterations = max_iterations_;
      registration_config.min_points = static_cast<std::size_t>(min_points_);
      const MultiScaleGicpRegistration registration(registration_config);
      registration_result =
          registration.align(filtered, last_keyframe_, initial_guess);
    } else {
      MultiScaleIcpConfig registration_config;
      registration_config.voxel_leaf_sizes = registration_voxel_leaf_sizes_;
      registration_config.max_correspondence_distance =
          max_correspondence_distance_;
      registration_config.transformation_epsilon = transformation_epsilon_;
      registration_config.euclidean_fitness_epsilon =
          euclidean_fitness_epsilon_;
      registration_config.max_iterations = max_iterations_;
      registration_config.min_points = static_cast<std::size_t>(min_points_);
      const MultiScaleIcpRegistration registration(registration_config);
      registration_result =
          registration.align(filtered, last_keyframe_, initial_guess);
    }

    last_fitness_score_ = registration_result.fitness_score;
    last_registration_stages_ = registration_result.stages_used;
    last_registration_reason_ = registration_result.reason;
    const Eigen::Matrix4f current_to_previous =
        registration_result.source_to_target;
    RegistrationGate gate;
    gate.min_points = static_cast<std::size_t>(min_points_);
    gate.max_fitness_score = max_fitness_score_;
    gate.min_geometry_score = min_geometry_score_;
    last_registration_usable_ =
        isRegistrationUsable(registration_result.converged, last_fitness_score_,
                             filtered->size(),
                             last_geometry_observability_.score, gate);
    const double fitness_observability =
        observabilityScore(last_fitness_score_, max_fitness_score_);
    last_observability_score_ =
        std::min(fitness_observability, last_geometry_observability_.score);

    if (!last_registration_usable_) {
      ++rejected_registrations_;
      ++consecutive_registration_rejections_;
      last_status_message_ = rejectedRegistrationStatus(
          registration_result.converged, last_fitness_score_, gate);
      const bool reseeded = shouldReseedAfterRejectedRegistration(
          consecutive_registration_rejections_,
          last_geometry_observability_.score,
          min_geometry_score_,
          static_cast<std::size_t>(
              std::max(0, reseed_keyframe_after_consecutive_rejections_)));
      if (reseeded) {
        reseedKeyframe(filtered, msg->header.stamp);
        last_status_message_ = "registration_rejected_reseeded";
        updateSubmapQuality();
        if (publish_odometry_on_keyframe_reseed_) {
          publishOdometry(msg->header.stamp);
        }
        return;
      }
      updateSubmapQuality();
      return;
    }

    consecutive_registration_rejections_ = 0;
    const double motion_dt = (msg->header.stamp - last_keyframe_stamp_).toSec();
    if (std::isfinite(motion_dt) && motion_dt > 0.0) {
      last_registration_delta_ = current_to_previous;
      last_motion_dt_ = motion_dt;
      has_motion_prior_ = true;
    }
    pose_ = accumulatePose(pose_, current_to_previous);
    *last_keyframe_ = *filtered;
    last_keyframe_stamp_ = msg->header.stamp;
    last_status_message_ = "ok";
    appendToSubmap(filtered);
    updateSubmapQuality();
    publishOdometry(msg->header.stamp);
    publishSubmap(msg->header.stamp);
  }

  CloudT::Ptr filterCloud(const CloudT::Ptr& input) const {
    return voxelDownsampleFiniteCloud(input, voxel_leaf_size_);
  }

  Eigen::Matrix4f makeRegistrationInitialGuess(const ros::Time& stamp) const {
    if (!enable_constant_velocity_initial_guess_ || !has_motion_prior_ ||
        last_motion_dt_ <= 0.0 || last_keyframe_stamp_.isZero()) {
      return Eigen::Matrix4f::Identity();
    }
    const double dt = (stamp - last_keyframe_stamp_).toSec();
    if (!std::isfinite(dt) || dt <= 0.0) {
      return Eigen::Matrix4f::Identity();
    }
    const double clamped_scale = std::max(
        0.1, std::min(max_initial_guess_scale_, dt / last_motion_dt_));
    return scaledTranslationInitialGuess(last_registration_delta_,
                                         clamped_scale,
                                         max_initial_guess_translation_);
  }

  std::string rejectedRegistrationStatus(
      bool converged,
      double fitness_score,
      const RegistrationGate& gate) const {
    if (last_geometry_observability_.score < min_geometry_score_) {
      return last_geometry_observability_.reason;
    }
    if (!converged) {
      return last_registration_reason_;
    }
    if (!std::isfinite(fitness_score)) {
      return "invalid_fitness";
    }
    if (fitness_score > gate.max_fitness_score) {
      return "fitness_gate";
    }
    return last_registration_reason_;
  }

  void reseedKeyframe(const CloudT::Ptr& filtered, const ros::Time& stamp) {
    *last_keyframe_ = *filtered;
    last_keyframe_stamp_ = stamp;
    last_registration_delta_ = Eigen::Matrix4f::Identity();
    last_motion_dt_ = 0.0;
    last_initial_guess_translation_ = 0.0;
    has_motion_prior_ = false;
    consecutive_registration_rejections_ = 0;
    ++keyframe_reseeds_;
  }

  void appendToSubmap(const CloudT::Ptr& cloud) {
    if (!publish_submap_) {
      return;
    }

    CloudT transformed;
    pcl::transformPointCloud(*cloud, transformed, pose_.matrix().cast<float>());
    *local_submap_ += transformed;

    if (submap_voxel_leaf_size_ > 0.0) {
      local_submap_ =
          voxelDownsampleFiniteCloud(local_submap_, submap_voxel_leaf_size_);
    }

    if (local_submap_->size() > static_cast<std::size_t>(max_submap_points_)) {
      const auto erase_count =
          local_submap_->size() - static_cast<std::size_t>(max_submap_points_);
      local_submap_->points.erase(local_submap_->points.begin(),
                                  local_submap_->points.begin() + erase_count);
      local_submap_->width = local_submap_->points.size();
      local_submap_->height = 1;
      local_submap_->is_dense = false;
    }
  }

  void updateSubmapQuality() {
    SubmapQualityGate gate;
    gate.min_stable_points =
        static_cast<std::size_t>(std::max(0, min_stable_submap_points_));
    gate.min_observability_score = min_submap_observability_;
    gate.max_consecutive_rejections = static_cast<std::size_t>(
        std::max(0, max_consecutive_registration_rejections_));
    gate.max_points = static_cast<std::size_t>(std::max(0, max_submap_points_));
    last_submap_quality_ =
        evaluateSubmapQuality(last_registration_usable_,
                              last_observability_score_,
                              local_submap_->size(),
                              consecutive_registration_rejections_, gate);
  }

  void publishOdometry(const ros::Time& stamp) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose = eigenToPose(pose_);
    odom.pose.covariance[0] =
        std::max(1e-6, 1.0 - last_observability_score_);
    odom.pose.covariance[7] =
        std::max(1e-6, 1.0 - last_observability_score_);
    odom.pose.covariance[14] =
        std::max(1e-6, 1.0 - last_observability_score_);
    odom_pub_.publish(odom);
    ++published_odometry_;
  }

  void publishSubmap(const ros::Time& stamp) {
    if (!publish_submap_ || local_submap_->empty()) {
      return;
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*local_submap_, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_id_;
    submap_pub_.publish(msg);
  }

  void diagnosticsTimerCallback(const ros::TimerEvent&) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();

    diagnostic_msgs::DiagnosticStatus status;
    status.name = "lio_local_odometry";
    status.hardware_id = input_cloud_topic_;
    if (!initialized_) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = "waiting for first usable cloud";
    } else if (!last_registration_usable_) {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      status.message = last_status_message_;
    } else {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = last_status_message_;
    }

    status.values = {
        kv("cloud_count", std::to_string(cloud_count_)),
        kv("published_odometry", std::to_string(published_odometry_)),
        kv("dropped_clouds", std::to_string(dropped_clouds_)),
        kv("rejected_registrations", std::to_string(rejected_registrations_)),
        kv("keyframe_reseeds", std::to_string(keyframe_reseeds_)),
        kv("cloud_queue_size", std::to_string(std::max(1, cloud_queue_size_))),
        kv("reseed_keyframe_after_consecutive_rejections",
           std::to_string(reseed_keyframe_after_consecutive_rejections_)),
        kv("last_input_points", std::to_string(last_input_points_)),
        kv("last_filtered_points", std::to_string(last_filtered_points_)),
        kv("registration_method", registration_method_),
        kv("last_fitness_score", doubleToString(last_fitness_score_)),
        kv("registration_stages", std::to_string(last_registration_stages_)),
        kv("registration_reason", last_registration_reason_),
        kv("constant_velocity_initial_guess",
           enable_constant_velocity_initial_guess_ ? "true" : "false"),
        kv("motion_prior_ready", has_motion_prior_ ? "true" : "false"),
        kv("last_motion_dt", doubleToString(last_motion_dt_)),
        kv("last_initial_guess_translation",
           doubleToString(last_initial_guess_translation_)),
        kv("observability_score", doubleToString(last_observability_score_)),
        kv("geometry_score", doubleToString(last_geometry_observability_.score)),
        kv("geometry_min_eigen_ratio",
           doubleToString(last_geometry_observability_.min_eigen_ratio)),
        kv("geometry_reason", last_geometry_observability_.reason),
        kv("submap_points", std::to_string(local_submap_->size())),
        kv("consecutive_registration_rejections",
           std::to_string(consecutive_registration_rejections_)),
        kv("submap_quality_score",
           doubleToString(last_submap_quality_.quality_score)),
        kv("submap_capacity_ratio",
           doubleToString(last_submap_quality_.capacity_ratio)),
        kv("submap_can_promote",
           last_submap_quality_.can_promote ? "true" : "false"),
        kv("submap_quality_reason", last_submap_quality_.reason),
    };
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher submap_pub_;
  ros::Publisher diagnostics_pub_;
  ros::Timer diagnostics_timer_;

  std::string input_cloud_topic_;
  std::string odom_topic_;
  std::string submap_topic_;
  std::string diagnostics_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string registration_method_ = "icp";

  int min_points_ = 100;
  int max_iterations_ = 50;
  int max_submap_points_ = 200000;
  int min_stable_submap_points_ = 1000;
  int max_consecutive_registration_rejections_ = 2;
  int ndt_refinement_iterations_ = 30;
  int cloud_queue_size_ = 5;
  int reseed_keyframe_after_consecutive_rejections_ = 0;
  std::vector<double> registration_voxel_leaf_sizes_;
  std::vector<double> gicp_voxel_leaf_sizes_;
  std::vector<double> ndt_resolutions_;
  double max_fitness_score_ = 0.5;
  double min_geometry_eigen_ratio_ = 0.01;
  double min_geometry_score_ = 0.2;
  double voxel_leaf_size_ = 0.10;
  double max_correspondence_distance_ = 1.0;
  double transformation_epsilon_ = 1e-4;
  double euclidean_fitness_epsilon_ = 1e-3;
  double ndt_step_size_ = 0.1;
  double submap_voxel_leaf_size_ = 0.20;
  double min_submap_observability_ = 0.5;
  double diagnostics_period_sec_ = 1.0;
  double max_initial_guess_translation_ = 5.0;
  double max_initial_guess_scale_ = 4.0;
  bool publish_submap_ = true;
  bool enable_constant_velocity_initial_guess_ = false;
  bool publish_odometry_on_keyframe_reseed_ = true;

  bool initialized_ = false;
  bool last_registration_usable_ = false;
  std::string last_status_message_ = "starting";
  ros::Time last_stamp_;
  ros::Time last_keyframe_stamp_;
  Eigen::Affine3d pose_;
  Eigen::Matrix4f last_registration_delta_ = Eigen::Matrix4f::Identity();
  CloudT::Ptr last_keyframe_;
  CloudT::Ptr local_submap_;

  std::size_t cloud_count_ = 0;
  std::size_t published_odometry_ = 0;
  std::size_t dropped_clouds_ = 0;
  std::size_t rejected_registrations_ = 0;
  std::size_t consecutive_registration_rejections_ = 0;
  std::size_t keyframe_reseeds_ = 0;
  std::size_t last_input_points_ = 0;
  std::size_t last_filtered_points_ = 0;
  std::size_t last_registration_stages_ = 0;
  double last_fitness_score_ = std::numeric_limits<double>::infinity();
  double last_observability_score_ = 0.0;
  double last_motion_dt_ = 0.0;
  double last_initial_guess_translation_ = 0.0;
  std::string last_registration_reason_ = "not_started";
  GeometryObservability last_geometry_observability_;
  SubmapQuality last_submap_quality_;
  bool has_motion_prior_ = false;
};

}  // namespace lio_local_odometry

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_icp_odometry");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  lio_local_odometry::LocalIcpOdometryNode node(nh, private_nh);
  ros::spin();
  return 0;
}
