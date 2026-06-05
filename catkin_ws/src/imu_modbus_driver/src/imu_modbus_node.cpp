#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <modbus/modbus.h>
#include <thread>
#include <chrono>
#include <vector>
#include <cstring>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

#include "imu_modbus_driver/imu_modbus_config.h"

class IMUModbusNode {
private:
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    ros::Publisher diagnostics_pub_;

    modbus_t* ctx_;
    imu_modbus_driver::ImuModbusConfig config_;
    bool connected_;
    std::vector<uint16_t> register_data_;

    imu_modbus_driver::ImuRuntimeStats runtime_stats_;

public:
    IMUModbusNode()
        : nh_("~"),
          connected_(false) {
        loadParameters();

        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(config_.output_topic, 10);
        diagnostics_pub_ =
            nh_.advertise<diagnostic_msgs::DiagnosticArray>(config_.diagnostics_topic, 10);

        register_data_.resize(config_.num_registers);
        ctx_ = nullptr;

        runtime_stats_.reset(ros::Time::now().toSec());
    }

    ~IMUModbusNode() {
        if (ctx_) {
            modbus_close(ctx_);
            modbus_free(ctx_);
        }
    }

    void loadParameters() {
        nh_.param<std::string>("ip_address", config_.ip_address, config_.ip_address);
        nh_.param<int>("port", config_.port, config_.port);
        nh_.param<int>("start_register", config_.start_register, config_.start_register);
        nh_.param<int>("num_registers", config_.num_registers, config_.num_registers);
        nh_.param<int>("max_reconnect_attempts",
                       config_.max_reconnect_attempts,
                       config_.max_reconnect_attempts);
        nh_.param<int>("reconnect_delay_ms",
                       config_.reconnect_delay_ms,
                       config_.reconnect_delay_ms);
        nh_.param<std::string>("output_topic", config_.output_topic, config_.output_topic);
        nh_.param<std::string>("frame_id", config_.frame_id, config_.frame_id);
        nh_.param<double>("poll_rate_hz", config_.poll_rate_hz, config_.poll_rate_hz);
        nh_.param<double>("stats_period_sec",
                          config_.stats_period_sec,
                          config_.stats_period_sec);
        nh_.param<double>("warn_min_publish_rate_ratio",
                          config_.warn_min_publish_rate_ratio,
                          config_.warn_min_publish_rate_ratio);
        nh_.param<double>("warn_read_latency_ms",
                          config_.warn_read_latency_ms,
                          config_.warn_read_latency_ms);
        nh_.param<double>("warn_acc_saturation_mps2",
                          config_.warn_acc_saturation_mps2,
                          config_.warn_acc_saturation_mps2);
        nh_.param<double>("warn_gyro_saturation_radps",
                          config_.warn_gyro_saturation_radps,
                          config_.warn_gyro_saturation_radps);
        nh_.param<double>("orientation_covariance_x",
                          config_.orientation_covariance_x,
                          config_.orientation_covariance_x);
        nh_.param<double>("orientation_covariance_y",
                          config_.orientation_covariance_y,
                          config_.orientation_covariance_y);
        nh_.param<double>("orientation_covariance_z",
                          config_.orientation_covariance_z,
                          config_.orientation_covariance_z);
        nh_.param<double>("angular_velocity_covariance_x",
                          config_.angular_velocity_covariance_x,
                          config_.angular_velocity_covariance_x);
        nh_.param<double>("angular_velocity_covariance_y",
                          config_.angular_velocity_covariance_y,
                          config_.angular_velocity_covariance_y);
        nh_.param<double>("angular_velocity_covariance_z",
                          config_.angular_velocity_covariance_z,
                          config_.angular_velocity_covariance_z);
        nh_.param<double>("linear_acceleration_covariance_x",
                          config_.linear_acceleration_covariance_x,
                          config_.linear_acceleration_covariance_x);
        nh_.param<double>("linear_acceleration_covariance_y",
                          config_.linear_acceleration_covariance_y,
                          config_.linear_acceleration_covariance_y);
        nh_.param<double>("linear_acceleration_covariance_z",
                          config_.linear_acceleration_covariance_z,
                          config_.linear_acceleration_covariance_z);
        nh_.param<bool>("enable_temperature_diagnostics",
                        config_.enable_temperature_diagnostics,
                        config_.enable_temperature_diagnostics);
        nh_.param<int>("temperature_register",
                       config_.temperature_register,
                       config_.temperature_register);
        nh_.param<std::string>("temperature_register_type",
                               config_.temperature_register_type,
                               config_.temperature_register_type);
        nh_.param<double>("temperature_scale",
                          config_.temperature_scale,
                          config_.temperature_scale);
        nh_.param<double>("temperature_offset_c",
                          config_.temperature_offset_c,
                          config_.temperature_offset_c);
        nh_.param<double>("warn_temperature_abs_c",
                          config_.warn_temperature_abs_c,
                          config_.warn_temperature_abs_c);
        nh_.param<double>("warn_temperature_delta_c",
                          config_.warn_temperature_delta_c,
                          config_.warn_temperature_delta_c);
        nh_.param<std::string>("diagnostics_topic",
                               config_.diagnostics_topic,
                               config_.diagnostics_topic);
        nh_.param<std::string>("timestamp_source",
                               config_.timestamp_source,
                               config_.timestamp_source);
        nh_.param<std::string>("hardware_time_status",
                               config_.hardware_time_status,
                               config_.hardware_time_status);
        nh_.param<std::string>("pps_status", config_.pps_status, config_.pps_status);

        if (config_.poll_rate_hz <= 0.0) {
            ROS_WARN("poll_rate_hz <= 0, reset to 400 Hz");
            config_.poll_rate_hz = 400.0;
        }
        if (config_.stats_period_sec <= 0.0) {
            ROS_WARN("stats_period_sec <= 0, reset to 5 s");
            config_.stats_period_sec = 5.0;
        }
        if (config_.warn_min_publish_rate_ratio <= 0.0 ||
            config_.warn_min_publish_rate_ratio > 1.0) {
            ROS_WARN("warn_min_publish_rate_ratio out of range, reset to 0.8");
            config_.warn_min_publish_rate_ratio = 0.8;
        }
        if (config_.warn_read_latency_ms <= 0.0) {
            ROS_WARN("warn_read_latency_ms <= 0, reset to 50 ms");
            config_.warn_read_latency_ms = 50.0;
        }
        if (config_.warn_acc_saturation_mps2 < 0.0) {
            ROS_WARN("warn_acc_saturation_mps2 < 0, reset to 150 m/s^2");
            config_.warn_acc_saturation_mps2 = 150.0;
        }
        if (config_.warn_gyro_saturation_radps < 0.0) {
            ROS_WARN("warn_gyro_saturation_radps < 0, reset to 1800 deg/s");
            config_.warn_gyro_saturation_radps = 1800.0 * M_PI / 180.0;
        }
        sanitizeCovariance("orientation_covariance_x", &config_.orientation_covariance_x, 0.001);
        sanitizeCovariance("orientation_covariance_y", &config_.orientation_covariance_y, 0.001);
        sanitizeCovariance("orientation_covariance_z", &config_.orientation_covariance_z, 0.001);
        sanitizeCovariance("angular_velocity_covariance_x",
                           &config_.angular_velocity_covariance_x,
                           imu_modbus_driver::ImuModbusConfig().angular_velocity_covariance_x);
        sanitizeCovariance("angular_velocity_covariance_y",
                           &config_.angular_velocity_covariance_y,
                           imu_modbus_driver::ImuModbusConfig().angular_velocity_covariance_y);
        sanitizeCovariance("angular_velocity_covariance_z",
                           &config_.angular_velocity_covariance_z,
                           imu_modbus_driver::ImuModbusConfig().angular_velocity_covariance_z);
        sanitizeCovariance("linear_acceleration_covariance_x",
                           &config_.linear_acceleration_covariance_x,
                           imu_modbus_driver::ImuModbusConfig().linear_acceleration_covariance_x);
        sanitizeCovariance("linear_acceleration_covariance_y",
                           &config_.linear_acceleration_covariance_y,
                           imu_modbus_driver::ImuModbusConfig().linear_acceleration_covariance_y);
        sanitizeCovariance("linear_acceleration_covariance_z",
                           &config_.linear_acceleration_covariance_z,
                           imu_modbus_driver::ImuModbusConfig().linear_acceleration_covariance_z);
        if (config_.temperature_scale == 0.0 || !std::isfinite(config_.temperature_scale)) {
            ROS_WARN("temperature_scale invalid, reset to 1.0");
            config_.temperature_scale = 1.0;
        }
        if (!std::isfinite(config_.temperature_offset_c)) {
            ROS_WARN("temperature_offset_c invalid, reset to 0.0");
            config_.temperature_offset_c = 0.0;
        }
        if (config_.warn_temperature_abs_c < 0.0 ||
            !std::isfinite(config_.warn_temperature_abs_c)) {
            ROS_WARN("warn_temperature_abs_c invalid, reset to 85 C");
            config_.warn_temperature_abs_c = 85.0;
        }
        if (config_.warn_temperature_delta_c < 0.0 ||
            !std::isfinite(config_.warn_temperature_delta_c)) {
            ROS_WARN("warn_temperature_delta_c invalid, reset to 15 C");
            config_.warn_temperature_delta_c = 15.0;
        }
    }

    void sanitizeCovariance(const std::string& name, double* value, const double fallback) {
        if (!std::isfinite(*value) || *value < 0.0) {
            ROS_WARN("%s invalid, reset to default", name.c_str());
            *value = fallback;
        }
    }

    bool connect() {
        if (ctx_) {
            modbus_close(ctx_);
            modbus_free(ctx_);
        }
        ctx_ = modbus_new_tcp(config_.ip_address.c_str(), config_.port);
        if (!ctx_) {
            ROS_ERROR("Unable to allocate libmodbus context");
            return false;
        }
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        modbus_set_response_timeout(ctx_, timeout.tv_sec, timeout.tv_usec);
        modbus_set_byte_timeout(ctx_, timeout.tv_sec, timeout.tv_usec);
        if (modbus_connect(ctx_) == -1) {
            ROS_ERROR("Connection failed: %s", modbus_strerror(errno));
            modbus_free(ctx_);
            ctx_ = nullptr;
            return false;
        }
        connected_ = true;
        ROS_INFO("Connected to IMU at %s:%d", config_.ip_address.c_str(), config_.port);
        return true;
    }

    bool reconnect() {
        ROS_WARN("Reconnecting...");
        connected_ = false;
        for (int attempt = 1; attempt <= config_.max_reconnect_attempts; ++attempt) {
            runtime_stats_.observeReconnectAttempt();
            if (connect()) {
                runtime_stats_.observeReconnectSuccess();
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(config_.reconnect_delay_ms));
        }
        return false;
    }

    float registersToIEEEFloat(uint16_t low, uint16_t high) {
        return imu_modbus_driver::registersToIEEEFloat(low, high);
    }

    int32_t registersToInt32(uint16_t low, uint16_t high) {
        return imu_modbus_driver::registersToInt32(low, high);
    }

    bool readRegisters() {
        if (!connected_ || !ctx_) return false;
        const ros::WallTime start = ros::WallTime::now();
        int ret = modbus_read_registers(
            ctx_, config_.start_register, config_.num_registers, register_data_.data());
        const ros::WallTime end = ros::WallTime::now();
        const bool success = ret != -1;
        runtime_stats_.observeRead(start.toSec(), end.toSec(), success);
        if (ret == -1) {
            connected_ = false;
            return false;
        }
        return true;
    }

    // 数据有效性检查
    bool isDataValid(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
        return imu_modbus_driver::isDataValid(acc, gyro);
    }

    bool isDataNearSaturation(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
        return imu_modbus_driver::isDataNearSaturation(
            acc, gyro, config_.warn_acc_saturation_mps2, config_.warn_gyro_saturation_radps);
    }

    bool readTemperature(double* temperature_c) const {
        if (!config_.enable_temperature_diagnostics || config_.temperature_register < 0) {
            return false;
        }
        const int base = config_.temperature_register - config_.start_register;
        const bool decoded = imu_modbus_driver::decodeTemperatureRegisters(
            register_data_,
            base,
            config_.temperature_register_type,
            config_.temperature_scale,
            config_.temperature_offset_c,
            temperature_c);
        if (!decoded && base >= 0 && base < static_cast<int>(register_data_.size())) {
            ROS_WARN_THROTTLE(5.0, "Unsupported temperature_register_type: %s",
                              config_.temperature_register_type.c_str());
        }
        return decoded;
    }

    // 四元数归一化函数
    inline void normalizeQuaternion(tf2::Quaternion& q) {
        double norm = std::sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
        if (norm > 1e-6) {
            q.setW(q.w() / norm);
            q.setX(q.x() / norm);
            q.setY(q.y() / norm);
            q.setZ(q.z() / norm);
        } else {
            q.setW(1.0);
            q.setX(0.0);
            q.setY(0.0);
            q.setZ(0.0);
            ROS_WARN("Quaternion norm too small, reset to identity");
        }
    }

    void processAndPublishData() {
        if (register_data_.size() < static_cast<size_t>(config_.num_registers)) return;

        // 读取加速度计数据（INT32格式）
        Eigen::Vector3d acc(
            registersToInt32(register_data_[72 - config_.start_register], register_data_[73 - config_.start_register]) / 1e6,
            registersToInt32(register_data_[74 - config_.start_register], register_data_[75 - config_.start_register]) / 1e6,
            registersToInt32(register_data_[76 - config_.start_register], register_data_[77 - config_.start_register]) / 1e6
        );

        // 读取陀螺仪数据（INT32格式）
        Eigen::Vector3d gyro(
            (registersToInt32(register_data_[78 - config_.start_register], register_data_[79 - config_.start_register]) * M_PI / 180.0) / 1e6,
            (registersToInt32(register_data_[80 - config_.start_register], register_data_[81 - config_.start_register]) * M_PI / 180.0) / 1e6,
            (registersToInt32(register_data_[82 - config_.start_register], register_data_[83 - config_.start_register]) * M_PI / 180.0) / 1e6
        );

        // 读取原始姿态角（float格式）
        // 航向角：寄存器40032和40033
        Eigen::Vector3d rpy_rad;
        rpy_rad(0) = registersToIEEEFloat(register_data_[31 - config_.start_register], register_data_[32 - config_.start_register]) * M_PI / 180.0;  // 航向 (yaw)
        rpy_rad(1) = registersToIEEEFloat(register_data_[33 - config_.start_register], register_data_[34 - config_.start_register]) * M_PI / 180.0; // 横摇 (roll)
        rpy_rad(2) = registersToIEEEFloat(register_data_[35 - config_.start_register], register_data_[36 - config_.start_register]) * M_PI / 180.0; // 纵摇 (pitch)

        // 数据有效性检查
        if (!isDataValid(acc, gyro)) {
            runtime_stats_.observeInvalidFrame();
            ROS_WARN_THROTTLE(1.0, "Invalid IMU data detected, skipping this sample");
            return;
        }
        if (isDataNearSaturation(acc, gyro)) {
            runtime_stats_.observeSaturation();
            ROS_WARN_THROTTLE(1.0, "Near-saturation IMU data detected");
        }
        double temperature_c = 0.0;
        if (readTemperature(&temperature_c)) {
            runtime_stats_.observeTemperature(temperature_c,
                                              config_.warn_temperature_abs_c,
                                              config_.warn_temperature_delta_c);
        }

        // 使用当前时间戳（直接使用，不需要dt计算）
        ros::Time now = ros::Time::now();

        // 将原始RPY角转换为四元数（用于LIO-SAM）
        tf2::Quaternion q;
        q.setRPY(rpy_rad(1), rpy_rad(2), rpy_rad(0)); // roll, pitch, yaw

        // 归一化四元数
        normalizeQuaternion(q);

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = config_.frame_id;

        imu_msg.linear_acceleration.x = acc(0);
        imu_msg.linear_acceleration.y = acc(1);
        imu_msg.linear_acceleration.z = acc(2);

        imu_msg.angular_velocity.x = gyro(0);
        imu_msg.angular_velocity.y = gyro(1);
        imu_msg.angular_velocity.z = gyro(2);

        // 使用原始姿态角转换的四元数
        imu_msg.orientation.w = q.w();
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();

        const Eigen::Matrix<double, 9, 1> orientation_covariance =
            imu_modbus_driver::makeDiagonalCovariance(config_.orientation_covariance_x,
                                                      config_.orientation_covariance_y,
                                                      config_.orientation_covariance_z);
        const Eigen::Matrix<double, 9, 1> angular_velocity_covariance =
            imu_modbus_driver::makeDiagonalCovariance(config_.angular_velocity_covariance_x,
                                                      config_.angular_velocity_covariance_y,
                                                      config_.angular_velocity_covariance_z);
        const Eigen::Matrix<double, 9, 1> linear_acceleration_covariance =
            imu_modbus_driver::makeDiagonalCovariance(config_.linear_acceleration_covariance_x,
                                                      config_.linear_acceleration_covariance_y,
                                                      config_.linear_acceleration_covariance_z);
        for (int i = 0; i < 9; ++i) {
            imu_msg.orientation_covariance[i] = orientation_covariance(i);
            imu_msg.angular_velocity_covariance[i] = angular_velocity_covariance(i);
            imu_msg.linear_acceleration_covariance[i] = linear_acceleration_covariance(i);
        }

        // 发布完整的LIO-SAM专用IMU数据
        imu_pub_.publish(imu_msg);

        runtime_stats_.observePublish(now.toSec());
    }

    void addDiagnosticValue(diagnostic_msgs::DiagnosticStatus& status,
                            const std::string& key,
                            const std::string& value) const {
        diagnostic_msgs::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(item);
    }

    std::string formatDouble(double value, int precision) const {
        std::ostringstream stream;
        stream.setf(std::ios::fixed);
        stream.precision(precision);
        stream << value;
        return stream.str();
    }

    void publishDiagnostics(const ros::Time& stamp,
                            const imu_modbus_driver::ImuRuntimeStatsSnapshot& snapshot) {
        diagnostic_msgs::DiagnosticArray array;
        array.header.stamp = stamp;

        diagnostic_msgs::DiagnosticStatus status;
        status.name = "imu_modbus_driver";
        status.hardware_id = config_.ip_address + ":" + std::to_string(config_.port);
        const imu_modbus_driver::ImuDiagnosticLevel level =
            snapshot.diagnosticLevel(connected_,
                                     config_.poll_rate_hz,
                                     config_.warn_min_publish_rate_ratio,
                                     config_.warn_read_latency_ms);
        status.level = static_cast<int>(level);
        if (level == imu_modbus_driver::ImuDiagnosticLevel::ERROR) {
            status.message = "disconnected";
        } else if (level == imu_modbus_driver::ImuDiagnosticLevel::WARN) {
            status.message = "degraded";
        } else {
            status.message = "connected";
        }

        addDiagnosticValue(status, "frame_id", config_.frame_id);
        addDiagnosticValue(status, "output_topic", config_.output_topic);
        addDiagnosticValue(status, "timestamp_source", config_.timestamp_source);
        addDiagnosticValue(status, "hardware_time_status", config_.hardware_time_status);
        addDiagnosticValue(status, "pps_status", config_.pps_status);
        addDiagnosticValue(status, "configured_poll_rate_hz",
                           formatDouble(config_.poll_rate_hz, 3));
        addDiagnosticValue(status, "warn_min_publish_rate_ratio",
                           formatDouble(config_.warn_min_publish_rate_ratio, 3));
        addDiagnosticValue(status, "warn_read_latency_ms",
                           formatDouble(config_.warn_read_latency_ms, 3));
        addDiagnosticValue(status, "warn_acc_saturation_mps2",
                           formatDouble(config_.warn_acc_saturation_mps2, 3));
        addDiagnosticValue(status, "warn_gyro_saturation_radps",
                           formatDouble(config_.warn_gyro_saturation_radps, 3));
        addDiagnosticValue(status, "orientation_covariance_x",
                           formatDouble(config_.orientation_covariance_x, 12));
        addDiagnosticValue(status, "orientation_covariance_y",
                           formatDouble(config_.orientation_covariance_y, 12));
        addDiagnosticValue(status, "orientation_covariance_z",
                           formatDouble(config_.orientation_covariance_z, 12));
        addDiagnosticValue(status, "angular_velocity_covariance_x",
                           formatDouble(config_.angular_velocity_covariance_x, 12));
        addDiagnosticValue(status, "angular_velocity_covariance_y",
                           formatDouble(config_.angular_velocity_covariance_y, 12));
        addDiagnosticValue(status, "angular_velocity_covariance_z",
                           formatDouble(config_.angular_velocity_covariance_z, 12));
        addDiagnosticValue(status, "linear_acceleration_covariance_x",
                           formatDouble(config_.linear_acceleration_covariance_x, 12));
        addDiagnosticValue(status, "linear_acceleration_covariance_y",
                           formatDouble(config_.linear_acceleration_covariance_y, 12));
        addDiagnosticValue(status, "linear_acceleration_covariance_z",
                           formatDouble(config_.linear_acceleration_covariance_z, 12));
        addDiagnosticValue(status, "enable_temperature_diagnostics",
                           config_.enable_temperature_diagnostics ? "true" : "false");
        addDiagnosticValue(status, "temperature_register",
                           std::to_string(config_.temperature_register));
        addDiagnosticValue(status, "temperature_register_type",
                           config_.temperature_register_type);
        addDiagnosticValue(status, "temperature_scale",
                           formatDouble(config_.temperature_scale, 6));
        addDiagnosticValue(status, "temperature_offset_c",
                           formatDouble(config_.temperature_offset_c, 3));
        addDiagnosticValue(status, "warn_temperature_abs_c",
                           formatDouble(config_.warn_temperature_abs_c, 3));
        addDiagnosticValue(status, "warn_temperature_delta_c",
                           formatDouble(config_.warn_temperature_delta_c, 3));
        addDiagnosticValue(status, "stats_window_sec",
                           formatDouble(snapshot.window_duration_sec, 3));
        addDiagnosticValue(status, "publish_count", std::to_string(snapshot.publish_count));
        addDiagnosticValue(status, "publish_rate_hz",
                           formatDouble(snapshot.publish_rate_hz, 3));
        addDiagnosticValue(status, "read_count", std::to_string(snapshot.read_count));
        addDiagnosticValue(status, "read_error_count",
                           std::to_string(snapshot.read_error_count));
        addDiagnosticValue(status, "invalid_frame_count",
                           std::to_string(snapshot.invalid_frame_count));
        addDiagnosticValue(status, "saturation_count",
                           std::to_string(snapshot.saturation_count));
        addDiagnosticValue(status, "reconnect_attempt_count",
                           std::to_string(snapshot.reconnect_attempt_count));
        addDiagnosticValue(status, "reconnect_success_count",
                           std::to_string(snapshot.reconnect_success_count));
        addDiagnosticValue(status, "temperature_sample_count",
                           std::to_string(snapshot.temperature_sample_count));
        addDiagnosticValue(status, "temperature_warning_count",
                           std::to_string(snapshot.temperature_warning_count));
        addDiagnosticValue(status, "latest_temperature_c",
                           formatDouble(snapshot.latest_temperature_c, 3));
        addDiagnosticValue(status, "min_temperature_c",
                           formatDouble(snapshot.min_temperature_c, 3));
        addDiagnosticValue(status, "max_temperature_c",
                           formatDouble(snapshot.max_temperature_c, 3));
        addDiagnosticValue(status, "last_read_latency_ms",
                           formatDouble(snapshot.last_read_latency_ms, 3));
        addDiagnosticValue(status, "mean_read_latency_ms",
                           formatDouble(snapshot.mean_read_latency_ms, 3));
        addDiagnosticValue(status, "max_read_latency_ms",
                           formatDouble(snapshot.max_read_latency_ms, 3));

        array.status.push_back(status);
        diagnostics_pub_.publish(array);
    }

    void publishDiagnosticsIfDue(const ros::Time& stamp) {
        const double now_sec = stamp.toSec();
        if (!runtime_stats_.due(now_sec, config_.stats_period_sec)) {
            return;
        }
        const imu_modbus_driver::ImuRuntimeStatsSnapshot snapshot =
            runtime_stats_.snapshot(now_sec);
        ROS_INFO_THROTTLE(config_.stats_period_sec,
                          "IMU publish rate: %.1f Hz, read latency mean/max: %.1f/%.1f ms, errors: %d",
                          snapshot.publish_rate_hz,
                          snapshot.mean_read_latency_ms,
                          snapshot.max_read_latency_ms,
                          snapshot.read_error_count);
        publishDiagnostics(stamp, snapshot);
        runtime_stats_.reset(now_sec);
    }

    void run() {
        ros::Rate rate(config_.poll_rate_hz);
        if (!connect()) return;
        while (ros::ok()) {
            if (!connected_ && !reconnect()) break;
            if (readRegisters()) {
                processAndPublishData();
            } else {
                connected_ = false;
            }
            publishDiagnosticsIfDue(ros::Time::now());
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_modbus_node");
    IMUModbusNode node;
    node.run();
    return 0;
}
