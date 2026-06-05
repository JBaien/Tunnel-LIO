#include "lidar_fusion/multi_lidar_fusion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_eigen/tf2_eigen.h>

namespace lidar_fusion {

namespace {

double safeNonNegativeMetric(const double value) {
    return std::isfinite(value) && value >= 0.0 ? value : 0.0;
}

std::string safeDiagnosticText(const std::string& value,
                               const std::string& fallback,
                               const bool allow_empty) {
    if (value.empty()) {
        return allow_empty ? value : fallback;
    }
    for (const char ch : value) {
        if (ch == ';' || ch == '\n' || ch == '\r') {
            return fallback;
        }
    }
    return value;
}

std::string safeOverlapStatus(const std::string& status) {
    if (status == "ok" || status == "warn" || status == "fail" ||
        status == "insufficient") {
        return status;
    }
    return "invalid";
}

bool hasPointField(const sensor_msgs::PointCloud2& cloud,
                   const std::string& name) {
    return std::find_if(cloud.fields.begin(), cloud.fields.end(),
                        [&name](const sensor_msgs::PointField& field) {
                            return field.name == name;
                        }) != cloud.fields.end();
}

std::vector<size_t> overlapSampleIndices(const size_t cloud_size,
                                         const size_t max_points) {
    std::vector<size_t> indices;
    if (cloud_size == 0) {
        return indices;
    }
    if (max_points == 0 || cloud_size <= max_points) {
        indices.reserve(cloud_size);
        for (size_t i = 0; i < cloud_size; ++i) {
            indices.push_back(i);
        }
        return indices;
    }

    indices.reserve(max_points);
    const double stride =
        static_cast<double>(cloud_size) / static_cast<double>(max_points);
    for (size_t i = 0; i < max_points; ++i) {
        const size_t index = std::min(
            cloud_size - 1,
            static_cast<size_t>(std::floor(static_cast<double>(i) * stride)));
        if (indices.empty() || indices.back() != index) {
            indices.push_back(index);
        }
    }
    return indices;
}

}  // namespace

PointXYZIRTS makeFusedPoint(const PointXYZIRT& source,
                            const Eigen::Vector3f& transformed_xyz,
                            const float time_offset,
                            const std::uint16_t sensor_id) {
    PointXYZIRTS point;
    point.x = transformed_xyz.x();
    point.y = transformed_xyz.y();
    point.z = transformed_xyz.z();
    point.intensity = source.intensity;
    point.ring = source.ring;
    point.sensor_id = sensor_id;
    point.time = source.time + time_offset;
    return point;
}

bool hasFusionInputFields(const sensor_msgs::PointCloud2& cloud,
                          const bool allow_missing_ring_time) {
    const std::array<std::string, 4> xyz_intensity = {
        {"x", "y", "z", "intensity"}};
    for (const std::string& field : xyz_intensity) {
        if (!hasPointField(cloud, field)) {
            return false;
        }
    }
    if (!allow_missing_ring_time) {
        return hasPointField(cloud, "ring") && hasPointField(cloud, "time");
    }
    return true;
}

bool convertToFusionInputCloud(const sensor_msgs::PointCloud2& cloud,
                               const bool allow_missing_ring_time,
                               pcl::PointCloud<PointXYZIRT>& output,
                               bool* used_legacy_defaults) {
    if (used_legacy_defaults) {
        *used_legacy_defaults = false;
    }
    if (!hasFusionInputFields(cloud, allow_missing_ring_time)) {
        return false;
    }

    const bool has_ring = hasPointField(cloud, "ring");
    const bool has_time = hasPointField(cloud, "time");
    if (has_ring && has_time) {
        try {
            pcl::fromROSMsg(cloud, output);
        } catch (const std::exception&) {
            return false;
        }
        return true;
    }

    if (!allow_missing_ring_time) {
        return false;
    }

    try {
        output.clear();
        output.width = cloud.width;
        output.height = cloud.height;
        output.is_dense = cloud.is_dense;
        pcl_conversions::toPCL(cloud.header, output.header);
        output.points.reserve(static_cast<size_t>(cloud.width) * cloud.height);

        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<float> intensity(cloud,
                                                               "intensity");
        for (size_t i = 0; i < static_cast<size_t>(cloud.width) * cloud.height;
             ++i, ++x, ++y, ++z, ++intensity) {
            PointXYZIRT point;
            point.x = *x;
            point.y = *y;
            point.z = *z;
            point.intensity = *intensity;
            point.ring = 0;
            point.time = 0.0f;
            output.points.push_back(point);
        }
    } catch (const std::exception&) {
        return false;
    }

    if (used_legacy_defaults) {
        *used_legacy_defaults = true;
    }
    return true;
}

FusionOverlapResidual computeOverlapResidual(
    const pcl::PointCloud<PointXYZIRTS>& cloud,
    const double max_pair_distance) {
    return computeOverlapResidual(cloud, max_pair_distance, 0);
}

FusionOverlapResidual computeOverlapResidual(
    const pcl::PointCloud<PointXYZIRTS>& cloud,
    const double max_pair_distance,
    const size_t max_points) {
    FusionOverlapResidual residual;
    if (cloud.empty() || !std::isfinite(max_pair_distance) ||
        max_pair_distance <= 0.0) {
        return residual;
    }

    const std::vector<size_t> sample_indices =
        overlapSampleIndices(cloud.points.size(), max_points);
    if (sample_indices.empty()) {
        return residual;
    }

    const double inv_leaf = 1.0 / max_pair_distance;
    auto quantize = [inv_leaf](const float value) -> int64_t {
        return static_cast<int64_t>(std::floor(static_cast<double>(value) *
                                              inv_leaf));
    };
    auto mix = [](const int64_t x, const int64_t y,
                  const int64_t z) -> uint64_t {
        uint64_t h = 1469598103934665603ull;
        auto add = [&h](const int64_t value) {
            const uint64_t v = static_cast<uint64_t>(value);
            h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
            h *= 1099511628211ull;
        };
        add(x);
        add(y);
        add(z);
        return h;
    };

    std::unordered_map<uint64_t, std::vector<size_t>> occupied;
    occupied.reserve(sample_indices.size());
    for (const size_t index : sample_indices) {
        const auto& point = cloud.points[index];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }
        occupied[mix(quantize(point.x), quantize(point.y),
                     quantize(point.z))]
            .push_back(index);
    }

    const double max_distance_sq = max_pair_distance * max_pair_distance;
    double sum_distance_sq = 0.0;
    for (const size_t index : sample_indices) {
        const auto& reference = cloud.points[index];
        if (reference.sensor_id != 0 || !std::isfinite(reference.x) ||
            !std::isfinite(reference.y) || !std::isfinite(reference.z)) {
            continue;
        }

        const int64_t qx = quantize(reference.x);
        const int64_t qy = quantize(reference.y);
        const int64_t qz = quantize(reference.z);
        std::unordered_map<std::uint16_t, double> best_distance_sq_by_sensor;

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    const auto bucket =
                        occupied.find(mix(qx + dx, qy + dy, qz + dz));
                    if (bucket == occupied.end()) {
                        continue;
                    }
                    for (const size_t candidate_index : bucket->second) {
                        const auto& candidate = cloud.points[candidate_index];
                        if (candidate.sensor_id == reference.sensor_id) {
                            continue;
                        }
                        const double x =
                            static_cast<double>(reference.x - candidate.x);
                        const double y =
                            static_cast<double>(reference.y - candidate.y);
                        const double z =
                            static_cast<double>(reference.z - candidate.z);
                        const double distance_sq = x * x + y * y + z * z;
                        if (distance_sq > max_distance_sq) {
                            continue;
                        }
                        const auto existing =
                            best_distance_sq_by_sensor.find(candidate.sensor_id);
                        if (existing == best_distance_sq_by_sensor.end()) {
                            best_distance_sq_by_sensor[candidate.sensor_id] =
                                distance_sq;
                        } else if (distance_sq < existing->second) {
                            existing->second = distance_sq;
                        }
                    }
                }
            }
        }

        for (const auto& item : best_distance_sq_by_sensor) {
            const double best_distance_sq = item.second;
            ++residual.pairs;
            sum_distance_sq += best_distance_sq;
            residual.max_distance =
                std::max(residual.max_distance, std::sqrt(best_distance_sq));
        }
    }

    if (residual.pairs > 0) {
        residual.rmse =
            std::sqrt(sum_distance_sq / static_cast<double>(residual.pairs));
    }
    return residual;
}

std::string formatFusionDiagnostics(const FusionDiagnosticSnapshot& snapshot) {
    std::ostringstream stream;
    stream << "callbacks=" << snapshot.sync_callbacks
           << ";published=" << snapshot.published
           << ";last_output_points=" << snapshot.last_output_points
           << ";last_sync_span="
           << safeNonNegativeMetric(snapshot.last_sync_span)
           << ";overlap_pairs=" << snapshot.overlap_pairs
           << ";overlap_rmse="
           << safeNonNegativeMetric(snapshot.overlap_rmse)
           << ";overlap_max="
           << safeNonNegativeMetric(snapshot.overlap_max)
           << ";overlap_status=" << safeOverlapStatus(snapshot.overlap_status);
    const size_t input_count =
        std::max(snapshot.last_input_points.size(),
                 snapshot.last_input_frames.size());
    for (size_t i = 0; i < input_count; ++i) {
        stream << ";input" << i << "_points="
               << (i < snapshot.last_input_points.size()
                       ? snapshot.last_input_points[i]
                       : 0);
        stream << ";input" << i << "_frame="
               << safeDiagnosticText(i < snapshot.last_input_frames.size()
                                          ? snapshot.last_input_frames[i]
                                          : "",
                                      "invalid", true);
    }
    stream << ";dropped_empty=" << snapshot.dropped_empty
           << ";dropped_min_points=" << snapshot.dropped_min_points
           << ";dropped_field=" << snapshot.dropped_field
           << ";dropped_tf=" << snapshot.dropped_tf
           << ";dropped_conversion=" << snapshot.dropped_conversion
           << ";dropped_exception=" << snapshot.dropped_exception
           << ";legacy_xyzi_clouds=" << snapshot.legacy_xyzi_clouds
           << ";legacy_xyzi_points=" << snapshot.legacy_xyzi_points;
    return stream.str();
}

MultiLidarFusion::MultiLidarFusion(ros::NodeHandle& nh,
                                   ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), tf_listener_(tf_buffer_) {
    loadParameters();
    validateConfig();

    fused_cloud_pub_ =
        nh_.advertise<CloudMsg>(config_.output_topic, 10, false);
    diagnostics_pub_ =
        nh_.advertise<std_msgs::String>(config_.diagnostics_topic, 5, false);
    initializeSubscribers();

    diagnostics_.last_input_points.assign(config_.lidar_num, 0);
    diagnostics_.last_input_frames.assign(config_.lidar_num, "");

    if (config_.enable_diagnostics) {
        diagnostics_timer_ =
            private_nh_.createTimer(ros::Duration(config_.diagnostics_period),
                                    &MultiLidarFusion::diagnosticsTimerCallback,
                                    this);
    }

    ROS_INFO_STREAM("MultiLidarFusion initialized: lidar_num="
                    << config_.lidar_num
                    << ", output_topic=" << config_.output_topic
                    << ", output_frame_id=" << config_.output_frame_id
                    << ", sync_slop=" << config_.sync_slop
                    << ", tf_timeout=" << config_.tf_timeout
                    << ", allow_missing_ring_time="
                    << (config_.allow_missing_ring_time ? "true" : "false"));
    for (size_t i = 0; i < config_.lidar_topics.size(); ++i) {
        ROS_INFO_STREAM("  lidar[" << i << "] topic="
                                   << config_.lidar_topics[i]);
    }
}

template <typename T>
void MultiLidarFusion::readParam(const std::string& key, T& value) const {
    if (private_nh_.getParam(key, value)) {
        return;
    }
    if (nh_.getParam("multi_lidar_fusion/" + key, value)) {
        return;
    }
    ros::param::get("/multi_lidar_fusion/" + key, value);
}

bool MultiLidarFusion::readStringVectorParam(
    const std::string& key, std::vector<std::string>& value) const {
    XmlRpc::XmlRpcValue raw;
    if (!private_nh_.getParam(key, raw) &&
        !nh_.getParam("multi_lidar_fusion/" + key, raw) &&
        !ros::param::get("/multi_lidar_fusion/" + key, raw)) {
        return false;
    }

    if (raw.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR_STREAM("Parameter " << key << " must be a string array");
        return false;
    }

    value.clear();
    for (int i = 0; i < raw.size(); ++i) {
        if (raw[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR_STREAM("Parameter " << key << "[" << i
                                          << "] must be a string");
            return false;
        }
        value.push_back(static_cast<std::string>(raw[i]));
    }
    return true;
}

void MultiLidarFusion::loadParameters() {
    readParam("lidar_num", config_.lidar_num);
    readStringVectorParam("lidar_topics", config_.lidar_topics);
    readParam("output_topic", config_.output_topic);
    readParam("diagnostics_topic", config_.diagnostics_topic);
    readParam("output_frame_id", config_.output_frame_id);
    readParam("sync_queue_size", config_.sync_queue_size);
    readParam("sync_slop", config_.sync_slop);
    readParam("tf_timeout", config_.tf_timeout);
    readParam("drop_empty_cloud", config_.drop_empty_cloud);
    readParam("min_points_per_lidar", config_.min_points_per_lidar);
    readParam("normalize_point_time", config_.normalize_point_time);
    readParam("allow_missing_ring_time", config_.allow_missing_ring_time);
    readParam("enable_voxel_filter", config_.enable_voxel_filter);
    readParam("voxel_leaf_size", config_.voxel_leaf_size);
    readParam("enable_diagnostics", config_.enable_diagnostics);
    readParam("diagnostics_period", config_.diagnostics_period);
    readParam("overlap_pair_distance", config_.overlap_pair_distance);
    readParam("overlap_max_points", config_.overlap_max_points);
    readParam("overlap_warn_rmse", config_.overlap_warn_rmse);
    readParam("overlap_fail_rmse", config_.overlap_fail_rmse);
}

void MultiLidarFusion::validateConfig() {
    if (config_.lidar_num != 2 && config_.lidar_num != 3) {
        throw std::runtime_error("lidar_num must be 2 or 3");
    }
    if (config_.lidar_topics.size() !=
        static_cast<size_t>(config_.lidar_num)) {
        std::ostringstream oss;
        oss << "lidar_topics size (" << config_.lidar_topics.size()
            << ") must equal lidar_num (" << config_.lidar_num << ")";
        throw std::runtime_error(oss.str());
    }
    for (const auto& topic : config_.lidar_topics) {
        if (topic.empty()) {
            throw std::runtime_error("lidar_topics must not contain empty topic");
        }
    }
    if (config_.output_topic.empty()) {
        throw std::runtime_error("output_topic must not be empty");
    }
    if (config_.diagnostics_topic.empty()) {
        throw std::runtime_error("diagnostics_topic must not be empty");
    }
    if (config_.output_frame_id.empty()) {
        throw std::runtime_error("output_frame_id must not be empty");
    }
    if (config_.sync_queue_size < 1) {
        ROS_WARN("sync_queue_size < 1, reset to 20");
        config_.sync_queue_size = 20;
    }
    if (!std::isfinite(config_.sync_slop) || config_.sync_slop <= 0.0) {
        ROS_WARN("sync_slop is invalid, reset to 0.05");
        config_.sync_slop = 0.05;
    }
    if (!std::isfinite(config_.tf_timeout) || config_.tf_timeout <= 0.0) {
        ROS_WARN("tf_timeout is invalid, reset to 0.05");
        config_.tf_timeout = 0.05;
    }
    if (config_.min_points_per_lidar < 0) {
        config_.min_points_per_lidar = 0;
    }
    if (!std::isfinite(config_.voxel_leaf_size) ||
        config_.voxel_leaf_size <= 0.0) {
        ROS_WARN("voxel_leaf_size is invalid, reset to 0.05");
        config_.voxel_leaf_size = 0.05;
    }
    if (!std::isfinite(config_.diagnostics_period) ||
        config_.diagnostics_period <= 0.0) {
        config_.diagnostics_period = 1.0;
    }
    if (!std::isfinite(config_.overlap_pair_distance) ||
        config_.overlap_pair_distance <= 0.0) {
        config_.overlap_pair_distance = 0.10;
    }
    if (config_.overlap_max_points < 0) {
        config_.overlap_max_points = 0;
    }
    if (!std::isfinite(config_.overlap_warn_rmse) ||
        config_.overlap_warn_rmse < 0.0) {
        config_.overlap_warn_rmse = 0.08;
    }
    if (!std::isfinite(config_.overlap_fail_rmse) ||
        config_.overlap_fail_rmse < config_.overlap_warn_rmse) {
        config_.overlap_fail_rmse = config_.overlap_warn_rmse;
    }
}

void MultiLidarFusion::initializeSubscribers() {
    subscribers_.reserve(config_.lidar_num);
    for (const auto& topic : config_.lidar_topics) {
        subscribers_.push_back(
            std::make_shared<message_filters::Subscriber<CloudMsg>>(
                nh_, topic, config_.sync_queue_size,
                ros::TransportHints().tcpNoDelay(true)));
    }

    if (config_.lidar_num == 2) {
        sync2_ = std::make_shared<Synchronizer2>(
            SyncPolicy2(config_.sync_queue_size), *subscribers_[0],
            *subscribers_[1]);
        sync2_->setMaxIntervalDuration(ros::Duration(config_.sync_slop));
        sync2_->registerCallback(
            boost::bind(&MultiLidarFusion::callback2, this, _1, _2));
    } else {
        sync3_ = std::make_shared<Synchronizer3>(
            SyncPolicy3(config_.sync_queue_size), *subscribers_[0],
            *subscribers_[1], *subscribers_[2]);
        sync3_->setMaxIntervalDuration(ros::Duration(config_.sync_slop));
        sync3_->registerCallback(
            boost::bind(&MultiLidarFusion::callback3, this, _1, _2, _3));
    }
}

void MultiLidarFusion::callback2(const CloudConstPtr& cloud1,
                                 const CloudConstPtr& cloud2) {
    processClouds({cloud1, cloud2});
}

void MultiLidarFusion::callback3(const CloudConstPtr& cloud1,
                                 const CloudConstPtr& cloud2,
                                 const CloudConstPtr& cloud3) {
    processClouds({cloud1, cloud2, cloud3});
}

void MultiLidarFusion::processClouds(
    const std::vector<CloudConstPtr>& clouds) {
    ++diagnostics_.sync_callbacks;
    diagnostics_.last_callback_wall_time = ros::Time::now();

    try {
        std::vector<size_t> point_counts(clouds.size(), 0);
        for (size_t i = 0; i < clouds.size(); ++i) {
            if (!validateInputCloud(clouds[i], i, point_counts[i])) {
                return;
            }
        }

        const ros::Time output_stamp = chooseOutputStamp(clouds);
        const double sync_span = computeSyncSpan(clouds);
        updateInputDiagnostics(clouds, point_counts, sync_span);

        std::vector<FusedPointCloud::Ptr> transformed_clouds;
        transformed_clouds.reserve(clouds.size());
        for (size_t i = 0; i < clouds.size(); ++i) {
            const auto& cloud = clouds[i];
            FusedPointCloud::Ptr transformed(new FusedPointCloud);
            if (!transformCloudToOutputFrame(cloud, output_stamp,
                                             static_cast<uint16_t>(i),
                                             *transformed)) {
                ++diagnostics_.dropped_tf;
                return;
            }
            transformed_clouds.push_back(transformed);
        }

        CloudMsg output_msg;
        if (!fuseTransformedClouds(transformed_clouds, output_stamp,
                                   output_msg)) {
            return;
        }

        fused_cloud_pub_.publish(output_msg);
        diagnostics_.last_publish_stamp = output_msg.header.stamp;
        diagnostics_.last_output_points =
            static_cast<size_t>(output_msg.width) * output_msg.height;
        ++diagnostics_.published;
    } catch (const std::exception& e) {
        ++diagnostics_.dropped_exception;
        ROS_ERROR_STREAM("Multi lidar fusion callback failed: " << e.what());
    }
}

bool MultiLidarFusion::validateInputCloud(const CloudConstPtr& cloud,
                                          size_t index,
                                          size_t& point_count) {
    if (!cloud) {
        ++diagnostics_.dropped_empty;
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index
                                                    << "] cloud pointer is null");
        return false;
    }
    point_count = static_cast<size_t>(cloud->width) * cloud->height;
    if (cloud->header.stamp.isZero()) {
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index
                                                    << "] stamp is zero");
        ++diagnostics_.dropped_empty;
        return false;
    }
    if (cloud->header.frame_id.empty()) {
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index
                                                    << "] frame_id is empty");
        ++diagnostics_.dropped_tf;
        return false;
    }
    if (point_count == 0 && config_.drop_empty_cloud) {
        ++diagnostics_.dropped_empty;
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index
                                                    << "] cloud is empty");
        return false;
    }
    if (point_count < static_cast<size_t>(config_.min_points_per_lidar)) {
        ++diagnostics_.dropped_min_points;
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index << "] has only "
                                                    << point_count
                                                    << " points");
        return false;
    }
    if (!hasRequiredFields(*cloud)) {
        ++diagnostics_.dropped_field;
        ROS_WARN_STREAM("Drop fusion group: lidar[" << index
                                                    << "] does not contain "
                                                       "x/y/z/intensity/ring/time");
        return false;
    }
    return true;
}

bool MultiLidarFusion::hasRequiredFields(const CloudMsg& cloud) const {
    return hasFusionInputFields(cloud, config_.allow_missing_ring_time);
}

bool MultiLidarFusion::transformCloudToOutputFrame(
    const CloudConstPtr& cloud, const ros::Time& output_stamp,
    const uint16_t sensor_id, FusedPointCloud& transformed_cloud) {
    geometry_msgs::TransformStamped transform;
    try {
        if (!tf_buffer_.canTransform(config_.output_frame_id,
                                     cloud->header.frame_id,
                                     cloud->header.stamp,
                                     ros::Duration(config_.tf_timeout))) {
            ROS_WARN_STREAM("TF unavailable: " << cloud->header.frame_id
                                               << " -> "
                                               << config_.output_frame_id
                                               << " at "
                                               << cloud->header.stamp.toSec());
            return false;
        }
        transform = tf_buffer_.lookupTransform(
            config_.output_frame_id, cloud->header.frame_id,
            cloud->header.stamp, ros::Duration(config_.tf_timeout));
    } catch (const tf2::TransformException& e) {
        ROS_WARN_STREAM("TF lookup failed: " << e.what());
        return false;
    }

    InputPointCloud input_cloud;
    bool used_legacy_defaults = false;
    if (!convertToFusionInputCloud(*cloud, config_.allow_missing_ring_time,
                                   input_cloud, &used_legacy_defaults)) {
        ++diagnostics_.dropped_conversion;
        ROS_WARN_STREAM("PointCloud2 conversion failed for "
                        << cloud->header.frame_id);
        return false;
    }
    if (used_legacy_defaults) {
        ++diagnostics_.legacy_xyzi_clouds;
        diagnostics_.legacy_xyzi_points += input_cloud.points.size();
    }

    const Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform);
    transformed_cloud.clear();
    transformed_cloud.points.resize(input_cloud.points.size());
    transformed_cloud.width = input_cloud.width;
    transformed_cloud.height = input_cloud.height;
    transformed_cloud.is_dense = input_cloud.is_dense;
    transformed_cloud.header = input_cloud.header;

    const float time_offset =
        config_.normalize_point_time
            ? static_cast<float>((cloud->header.stamp - output_stamp).toSec())
            : 0.0f;

    for (size_t i = 0; i < input_cloud.points.size(); ++i) {
        const auto& src = input_cloud.points[i];
        const Eigen::Vector3f p(src.x, src.y, src.z);
        const Eigen::Vector3f q = transform_eigen.cast<float>() * p;
        transformed_cloud.points[i] =
            makeFusedPoint(src, q, time_offset, sensor_id);
    }

    return true;
}

bool MultiLidarFusion::fuseTransformedClouds(
    const std::vector<FusedPointCloud::Ptr>& clouds, const ros::Time& output_stamp,
    CloudMsg& output_msg) {
    FusedPointCloud::Ptr fused(new FusedPointCloud);
    size_t total_points = 0;
    for (const auto& cloud : clouds) {
        total_points += cloud->points.size();
    }
    fused->points.reserve(total_points);
    fused->height = 1;
    fused->is_dense = false;

    for (const auto& cloud : clouds) {
        fused->points.insert(fused->points.end(), cloud->points.begin(),
                             cloud->points.end());
    }
    fused->width = static_cast<uint32_t>(fused->points.size());

    if (config_.enable_voxel_filter) {
        applyVoxelFilter(fused);
    }

    updateOverlapDiagnostics(*fused);

    try {
        pcl::toROSMsg(*fused, output_msg);
    } catch (const std::exception& e) {
        ++diagnostics_.dropped_conversion;
        ROS_WARN_STREAM("toROSMsg failed: " << e.what());
        return false;
    }

    output_msg.header.stamp = output_stamp;
    output_msg.header.frame_id = config_.output_frame_id;
    return true;
}

void MultiLidarFusion::updateOverlapDiagnostics(
    const FusedPointCloud& fused_cloud) {
    const FusionOverlapResidual residual =
        computeOverlapResidual(
            fused_cloud, config_.overlap_pair_distance,
            static_cast<size_t>(config_.overlap_max_points));
    diagnostics_.overlap_pairs = residual.pairs;
    diagnostics_.overlap_rmse = residual.rmse;
    diagnostics_.overlap_max = residual.max_distance;
    if (residual.pairs == 0) {
        diagnostics_.overlap_status = "insufficient";
    } else if (residual.rmse >= config_.overlap_fail_rmse) {
        diagnostics_.overlap_status = "fail";
    } else if (residual.rmse >= config_.overlap_warn_rmse) {
        diagnostics_.overlap_status = "warn";
    } else {
        diagnostics_.overlap_status = "ok";
    }
}

void MultiLidarFusion::applyVoxelFilter(FusedPointCloud::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        return;
    }

    const double inv_leaf = 1.0 / config_.voxel_leaf_size;
    auto quantize = [inv_leaf](float v) -> int64_t {
        return static_cast<int64_t>(std::floor(static_cast<double>(v) *
                                              inv_leaf));
    };
    auto mix = [](int64_t x, int64_t y, int64_t z) -> uint64_t {
        uint64_t h = 1469598103934665603ull;
        auto add = [&h](int64_t value) {
            uint64_t v = static_cast<uint64_t>(value);
            h ^= v + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
            h *= 1099511628211ull;
        };
        add(x);
        add(y);
        add(z);
        return h;
    };

    std::unordered_set<uint64_t> occupied;
    occupied.reserve(cloud->points.size());
    FusedPointCloud::Ptr filtered(new FusedPointCloud);
    filtered->points.reserve(cloud->points.size());
    filtered->height = 1;
    filtered->is_dense = cloud->is_dense;

    for (const auto& point : cloud->points) {
        const uint64_t key =
            mix(quantize(point.x), quantize(point.y), quantize(point.z));
        if (occupied.insert(key).second) {
            filtered->points.push_back(point);
        }
    }

    filtered->width = static_cast<uint32_t>(filtered->points.size());
    cloud = filtered;
    ++diagnostics_.voxel_filtered;
}

ros::Time MultiLidarFusion::chooseOutputStamp(
    const std::vector<CloudConstPtr>& clouds) const {
    ros::Time output_stamp = clouds.front()->header.stamp;
    for (const auto& cloud : clouds) {
        if (cloud->header.stamp < output_stamp) {
            output_stamp = cloud->header.stamp;
        }
    }
    return output_stamp;
}

double MultiLidarFusion::computeSyncSpan(
    const std::vector<CloudConstPtr>& clouds) const {
    ros::Time min_stamp = clouds.front()->header.stamp;
    ros::Time max_stamp = clouds.front()->header.stamp;
    for (const auto& cloud : clouds) {
        min_stamp = std::min(min_stamp, cloud->header.stamp);
        max_stamp = std::max(max_stamp, cloud->header.stamp);
    }
    return (max_stamp - min_stamp).toSec();
}

void MultiLidarFusion::updateInputDiagnostics(
    const std::vector<CloudConstPtr>& clouds,
    const std::vector<size_t>& point_counts, double sync_span) {
    diagnostics_.last_sync_span = sync_span;
    diagnostics_.last_input_points = point_counts;
    diagnostics_.last_input_frames.resize(clouds.size());
    for (size_t i = 0; i < clouds.size(); ++i) {
        diagnostics_.last_input_frames[i] = clouds[i]->header.frame_id;
    }

    if (sync_span > config_.sync_slop * 0.8) {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "Multi lidar sync span is near slop limit: "
                     << sync_span << " s, sync_slop=" << config_.sync_slop);
    }
}

void MultiLidarFusion::diagnosticsTimerCallback(const ros::TimerEvent&) {
    std::ostringstream counts;
    for (size_t i = 0; i < diagnostics_.last_input_points.size(); ++i) {
        if (i > 0) {
            counts << ", ";
        }
        counts << "lidar" << (i + 1) << "="
               << diagnostics_.last_input_points[i] << "pts/"
               << diagnostics_.last_input_frames[i];
    }

    FusionDiagnosticSnapshot snapshot;
    snapshot.sync_callbacks = diagnostics_.sync_callbacks;
    snapshot.published = diagnostics_.published;
    snapshot.last_output_points = diagnostics_.last_output_points;
    snapshot.last_sync_span = diagnostics_.last_sync_span;
    snapshot.overlap_pairs = diagnostics_.overlap_pairs;
    snapshot.overlap_rmse = diagnostics_.overlap_rmse;
    snapshot.overlap_max = diagnostics_.overlap_max;
    snapshot.overlap_status = diagnostics_.overlap_status;
    snapshot.last_input_points = diagnostics_.last_input_points;
    snapshot.last_input_frames = diagnostics_.last_input_frames;
    snapshot.dropped_empty = diagnostics_.dropped_empty;
    snapshot.dropped_min_points = diagnostics_.dropped_min_points;
    snapshot.dropped_field = diagnostics_.dropped_field;
    snapshot.dropped_tf = diagnostics_.dropped_tf;
    snapshot.dropped_conversion = diagnostics_.dropped_conversion;
    snapshot.dropped_exception = diagnostics_.dropped_exception;
    snapshot.legacy_xyzi_clouds = diagnostics_.legacy_xyzi_clouds;
    snapshot.legacy_xyzi_points = diagnostics_.legacy_xyzi_points;

    std_msgs::String message;
    message.data = formatFusionDiagnostics(snapshot);
    diagnostics_pub_.publish(message);

    ROS_INFO_STREAM("multi_lidar_fusion diagnostics: " << message.data
                    << ", inputs=[" << counts.str() << "]");
}

}  // namespace lidar_fusion
