# Tunnel-LIO

履带式掘进机三激光雷达 + IMU 长期建图系统工程仓库。

## ROS 工作区结构

本仓库采用统一 catkin 工作区：

```text
Tunnel-LIO/
  catkin_ws/
    src/
      imu_modbus_driver/    # IMU Modbus TCP 驱动
      lidar_fusion/         # 2/3 路激光雷达点云融合节点
      lio_local_odometry/   # 多尺度 PCL ICP/NDT/GICP 局部里程计、几何退化门控、子图输出和质量治理
      lio_preprocess/       # C++ 点云过滤、近机体遮蔽和 IMU 角速度去畸变
      lio_session_manager/  # 会话 manifest、WAL、断电恢复入口和稳定锚点 ICP 精配准
      lio_state_estimator/  # C++ IMU 滑窗健康评估和状态预测接口
      lio_time_manager/     # C++ 传感器频率、延迟、PPS/设备时间偏移诊断
      lio_eval_tools/       # C++ HIL/回放验收指标判定和报告生成
      machine_state_manager/# 掘进机工况状态机与写图门控
      mapping_control/      # 截面采样、长度限幅和控制断面策略
      section_manager/      # 观测截面、结构截面质量评估与输出
      slam_backend_manager/ # C++ 参数化几何/强度回环候选、精验证、可调 Ceres 6DoF 因子图、稳定图台账和链距后端治理首版
      tca_manager/          # C++ TCA 标靶检测、上下文台账和非唯一拒绝
      mine_slam_calibration/# C++ 外参 TF 发布、YAML 管理和离线审计
      timoo*/               # Timoo 雷达驱动包族
      lidar_*/              # tmlidar 雷达驱动包族
  docs/                     # 方案、硬件说明、开发规划
```

旧的 `imu_ws/`、`timoo/`、`tmlidar_ws/` 工作区壳已合并到 `catkin_ws/src`。当前保留原 ROS 包名，避免影响已有 `roslaunch`、`$(find package)`、include 和 topic 配置。

## 构建

完整构建前需要 ROS Noetic 和 IMU 驱动的系统依赖：

```bash
sudo apt-get install -y libmodbus-dev
```

```bash
cd /home/bai/Desktop/Tunnel-LIO/catkin_ws
catkin_make
source devel/setup.bash
```

如果当前机器暂未安装 `libmodbus-dev`，可先验证雷达驱动与融合节点：

```bash
cd /home/bai/Desktop/Tunnel-LIO/catkin_ws
catkin_make -DCATKIN_BLACKLIST_PACKAGES=imu_modbus_driver
```

## 常用启动

统一传感器 bringup：

```bash
roslaunch mine_slam_bringup bringup_sensors.launch lidar_driver_family:=timoo
roslaunch mine_slam_bringup bringup_sensors.launch lidar_driver_family:=tmlidar
```

IMU Modbus 驱动：

```bash
roslaunch imu_modbus_driver imu_modbus.launch
```

三雷达融合：

```bash
roslaunch lidar_fusion multi_lidar_fusion.launch
```

点云预处理（C++/roscpp）：

```bash
roslaunch lio_preprocess preprocess.launch
```

局部 ICP/NDT/GICP 里程计：

```bash
roslaunch lio_local_odometry local_icp_odometry.launch
```

IMU 状态预测（C++/roscpp）：

```bash
roslaunch lio_state_estimator state_estimator.launch
```

工况状态机（C++/roscpp）：

```bash
roslaunch machine_state_manager machine_state.launch
```

会话与断电恢复入口（C++/roscpp）：

```bash
roslaunch lio_session_manager session_manager.launch
```

截面与长度控制策略（C++/roscpp）：

```bash
roslaunch mapping_control mapping_control.launch
```

截面提取（C++/roscpp）：

```bash
roslaunch section_manager section_manager.launch
```

TCA 锚点（C++/roscpp）：

```bash
roslaunch tca_manager tca_manager.launch
```

保守后端治理（C++/roscpp）：

```bash
roslaunch slam_backend_manager slam_backend.launch
```

HIL/回放验收报告（C++/roscpp）：

```bash
roslaunch lio_eval_tools validation_report.launch \
  metrics_file:=/path/to/replay_metrics.txt \
  scenario_thresholds_file:=/path/to/scenario_validation_thresholds.txt \
  report_file:=/tmp/tunnel_lio_validation_report.txt
```

时间状态诊断（C++/roscpp，含 PPS/设备时间偏移估计）：

```bash
roslaunch lio_time_manager time_status.launch
```

外参与静态 TF（C++/roscpp 发布，Python 离线审计）：

```bash
roslaunch mine_slam_calibration publish_extrinsics.launch
rosrun mine_slam_calibration audit_extrinsics.py \
  /home/bai/Desktop/Tunnel-LIO/catkin_ws/src/mine_slam_calibration/config/extrinsics.yaml
```

传感器 + 三雷达融合：

```bash
roslaunch mine_slam_bringup bringup_fusion_timoo.launch
roslaunch mine_slam_bringup bringup_fusion_tmlidar.launch
```

按驱动族选择融合配置：

```bash
roslaunch lidar_fusion multi_lidar_fusion.launch \
  config:=/home/bai/Desktop/Tunnel-LIO/catkin_ws/src/lidar_fusion/config/multi_lidar_fusion_timoo.yaml

roslaunch lidar_fusion multi_lidar_fusion.launch \
  config:=/home/bai/Desktop/Tunnel-LIO/catkin_ws/src/lidar_fusion/config/multi_lidar_fusion_tmlidar.yaml
```

录制当前 session：

```bash
roslaunch mine_slam_bringup record_session.launch \
  session_root:=/tmp/tunnel_lio_sessions \
  session_name:=manual_smoke
```

## 关键文档

- [完整开发规划](docs/履带式掘进机三激光雷达IMU长期建图系统开发规划.md)
- [最新方案 v5.1](docs/方案v5.1.md)
- [GUJ120 硬件说明书](docs/【说明书】GUJ120%20矿用本安型激光雷达物位传感器.docx)
- [SIRIUS-F002SP 使用说明书](docs/光纤捷联惯导系统SIRIUS-F002SP使用说明书v1.0_20211115.doc)
