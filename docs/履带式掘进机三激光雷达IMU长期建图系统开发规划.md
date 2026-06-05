# 履带式掘进机三激光雷达 + IMU 长期建图系统完整开发规划

版本：v5.12
日期：2026-06-05
适用对象：井下履带式掘进机三 GUJ120 激光雷达 + SIRIUS-F002SP/Modbus IMU 长期建图、截面生产与断电续建系统

## 1. 编制依据与总体结论

本文档根据 `docs` 目录下已有方案文件，重点吸收 `docs/方案v5.1.md` 的最新技术路线，同时结合 `docs/方案v4.1.md`、`docs/方案v3.1.md`、`docs/井下掘进机LiDAR建图方案提取.md`、GUJ120 激光雷达说明书、SIRIUS-F002SP 光纤捷联惯导说明书，以及当前仓库 `catkin_ws/src` 中的 IMU、三雷达融合、timoo 雷达驱动包族和 tmlidar 雷达驱动包族现状生成。

当前已确认工程条件如下：

| 项目 | 已确认内容 | 工程含义 |
|---|---|---|
| IMU 驱动 | `catkin_ws/src/imu_modbus_driver` 已支持 Modbus TCP，默认 502 端口，发布 `sensor_msgs/Imu` 和 `/diagnostics/imu_modbus` | 可作为 P0 基础驱动使用；已具备 frame/topic/rate 参数化、协方差参数化、协方差 diagnostics 发布、发布频率、读取 RTT、读取失败计数、无效样本计数、饱和样本计数、默认关闭的可配置温度/温漂诊断、可单测复用的温度寄存器解码、重连尝试/成功计数、`timestamp_source`、`hardware_time_status` 和 `pps_status` 诊断；当前默认显式标记为 `host_now/host_time_only/unconfigured`，防止误把 ROS 接收时刻当硬件采样时刻；IMU 原生 Y-forward/Z-up 到 ROS x-forward/y-left/z-up 的坐标适配已通过 `imu_native -> imu_link` 显式 TF 落地；后续需增强真实硬件时间和 PPS |
| 雷达驱动 | `catkin_ws/src/timoo*` 与 `catkin_ws/src/lidar_*` 两套 3D 雷达 ROS 驱动包族 | 可根据现场雷达型号二选一或并行保留；要求输出 `XYZIRT/XYZIR + time/ring` |
| 三雷达融合 | `lidar_fusion` 已有 `multi_lidar_fusion_node`，支持 2/3 路点云、TF2 外参、`ring/time/sensor_id` 保留、点内时间归一化、同步诊断和中心雷达到侧雷达重叠区 RMSE/max/status 诊断 | 可作为三雷达统一输入层和外参/同步健康观测的首版工程基础 |
| GUJ120 | 16 线机械旋转 LiDAR，5/10/20 Hz，UDP/IP，MSOP/DIFOP 默认 2368/8603，1248 bytes 包，支持 PTP、单/双回波、电机锁相相位 | 驱动层必须保留包级时间、点级相对时间、ring、强度和传感器 ID |
| SIRIUS-F002SP | 具备 PPS、EVENTMARK、快速恢复、刚性安装要求；当前项目另有 Modbus 驱动实现 | IMU 不是简单姿态源，而是去畸变、短时预测、健康监测和重启恢复的关键输入 |
| 主要工况 | 长时间静止截割强振动，短时前进/后退，长直弱几何巷道，断电后原地或近原地重启 | 系统主目标是“静止不写假位移、截面质量优先、长度受控、续建保守” |
| ROS 实现语言 | ROS 运行节点统一使用 C++/roscpp | 在线 ROS 运行节点必须使用 C++/roscpp；Python 仅允许作为离线工具、数据生成、审计脚本或测试辅助，不得作为在线 ROS 运行节点接入 launch |

总体技术结论：

1. 主系统采用“增强型松耦合”的 LiDAR-IMU 路线：IMU 负责点级去畸变、短时运动先验、bias/gravity 管理和健康监测；三雷达融合后的几何注册主导位姿更新。
2. 在线链路只承担驱动、时间管理、点云预处理、三雷达融合、局部注册、PLC/工况门控、局部子图、截面生产和断电快速恢复。
3. 低频或离线链路承担多会话治理、保守回环、TCA 锚点、稳定图晋升、质量审计和成果导出。
4. 截面质量优先于全局轨迹精度，但长度不能无约束漂移；长度控制依赖退化方向保护、PLC 状态机、控制断面、TCA 锚点和低频图优化共同实现。
5. 断电续建必须分级验证：最后稳定位姿优先，局部子图精配准验证，失败再走 TCA/全局候选，仍失败则新建暂存会话，禁止错误恢复后污染稳定基线图。

### 1.1 当前工程落地状态

截至 2026-06-05，仓库已统一为 `catkin_ws` ROS 工作区，并完成 P0 工程骨架落地。当前已可构建、可测试、可由 bringup 串联启动的模块如下：

| 模块 | ROS 包 | 当前输出/入口 | 状态 |
|---|---|---|---|
| 统一启动与证据归档 | `mine_slam_bringup` | `bringup_sensors.launch`、`bringup_fusion_timoo.launch`、`bringup_fusion_tmlidar.launch`、`record_session.launch`、`diag_dump_event.launch`、`/diag/dump_event` | 已接入传感器、外参、融合、预处理、局部里程计、状态估计、状态机、会话、映射控制、截面管理、TCA 和后端治理；两套 fusion bringup 已提供 `section_session_id` 参数并透传到 `section_manager.launch` 的 `session_id`，使截面结构化消息、诊断和 CSV 能归属到明确会话；`record_session.sh`/`record_session.launch` 已生成 rosbag、PCAP 命令、会话化 Timoo/TMLidar fusion 启动命令、参数/TF 快照命令、time sync、PPS/PTP wiring、power-loss resume、runtime health/deployment/stability、section export、field acceptance 捕获命令、session 侧 `run_runtime_stability.sh` 长稳采样触发命令、metadata、`reports/`、`evidence_manifest.txt`、`validate_evidence.sh` 和 session 归档目录，其中 `commands/bringup_fusion_timoo_session.sh` 与 `commands/bringup_fusion_tmlidar_session.sh` 会自动以当前 `session_name` 设置 `section_session_id`，`commands/capture_section_export.sh` 会从 `/section/export` 或 `SECTION_EXPORT_SOURCE` 生成 `reports/section_export.csv`，且两条路径都会立即校验固定表头、`session_id` 与当前 session 一致、工况枚举、质量等级和数值字段，manifest 默认声明 `section_export=reports/section_export.csv`；并支持 `scenario`、`runtime_dir`、`time_status_topic`、`pps_topic`、长稳样本数和长稳间隔参数写入/透传，其中非空 `runtime_dir` 还必须拒绝逗号以避免污染 runtime stability CSV 路径证据，且 session 生成入口要求长稳样本数为正整数、长稳间隔为非负整数，防止非法默认值进入 metadata 和 `run_runtime_stability.sh`；`capture_time_sync.sh` 从 `/time/status` 捕获 `lio_time_manager pps` 与 `lio_time_manager clock` 诊断块，校验匹配 block 内 `level=0`，归档 PPS jitter 与 host/device mean offset，并显式写出 `time_sync_status=PASS/FAIL` 与 `raw` 原始 YAML 路径；`capture_pps_ptp_wiring.sh` 基于 time sync PASS 和 `reports/pps_ptp_wiring_confirmation.txt` 人工接线确认，要求确认文件总字段 `pps_ptp_wiring_verified` 严格等于 `PASS`，且 `pps_wiring_verified=PASS`、`ptp_wiring_verified=PASS`、`wiring_verified_by` 和 `wiring_verified_at` 均为有效文本值，生成 `reports/pps_ptp_wiring_verified.txt`，并附带 `timedatectl`/`chronyc` 状态快照；`PPS_PTP_WIRING_VERIFIED` 仅作为本地诊断覆盖输入，会被标记为 `manual_env` 且不能生成最终 PASS；`capture_power_loss_resume.sh` 基于 `validation_metrics_report.txt` 或包含实测 `recovery_time_s` 的 `reports/power_loss_resume_confirmation.txt` 人工确认生成 `reports/power_loss_resume_verified.txt`，metrics 来源必须证明报告首条非空汇总行 `overall=PASS`、`total_records` 为严格正整数、`failed_records=0` 且无重复 key，并且必须存在 `session` 与当前 session 一致、`scenario` 与当前 scenario 一致、`status=PASS`、`failed_checks=0` 的匹配记录，同时在 verified 文件中下沉与当前 session 的 `reports/validation_metrics_report.txt` 绝对路径一致的 `metrics_report` 字段，且 `capture_power_loss_resume.sh` 与 `capture_field_acceptance.sh` 必须在脚本级 gate 同步执行该规则，manual_file 来源必须要求确认文件总字段 `power_loss_resume_status` 严格等于 `PASS`，输出报告中的 `power_loss_resume_confirmation_overall` 也必须严格等于 `PASS`，并包含有效文本值 `resume_verified_by` 和 `resume_verified_at`，恢复时间必须是严格非负数，最大恢复时间必须是严格正数并满足 `recovery_time_s <= 45 s`，`POWER_LOSS_RESUME_VERIFIED` 环境变量同样只记录为 `manual_env` 且不能通过最终验收；`capture_field_acceptance.sh` 将 time sync、实机 systemd/Docker active/running、24 h 长稳、断电续建确认和 PPS/PTP 接线确认汇总为最终现场验收 gate，默认保守 FAIL，并会重新校验 time sync 的 `capture_status=CAPTURED`、`pps_status=PASS`、`clock_offset_status=PASS`、`pps_jitter_ms` 非负数和 `mean_offset_ms` 合法数值，把这些字段下沉到最终报告，同时重新校验 runtime deployment 总字段 `deployment_status` 和 PPS/PTP verified 文件总字段 `pps_ptp_wiring_verified` 均严格等于 `PASS`；长稳 summary 的 `overall` 字段必须严格等于 `PASS`，长稳样本数和间隔必须为严格正整数，`disk_failures/watchdog_failures/watchdog_skipped/health_failures` 必须为 0，session 侧 `runtime_stability.csv` 必须存在且统计 `runtime_stability_csv_samples`，要求 `runtime_stability_csv_status=PASS` 且 `runtime_stability_sample_count_match=PASS`，且对 metrics/manual_file 来源的 `power_loss_resume_verified.txt` 重新校验 metrics 首条非空汇总行 `overall=PASS`、`total_records` 为严格正整数、`failed_records=0`、无重复 key、当前 session/scenario 匹配 PASS 记录、metrics_report 字段与当前 session metrics 报告路径一致、manual_file 审计字段、严格数值格式和 `recovery_time_s <= max_recovery_time_s`，防止矛盾、失败、无审计或污染证据进入最终 PASS；`run_runtime_stability.sh` 执行后将板端 CSV/summary 复制到 session 日志，并生成纯 `key=value` 的 `runtime_stability_run.log` 记录 runtime_dir、samples、interval 和退出状态，普通 stdout/stderr 另存为 `runtime_stability_command_output.log`；`validate_evidence.sh` 在生成报告前刷新 time sync、PPS/PTP wiring、runtime health/deployment/stability、section export、power-loss resume 和 field acceptance 证据，降低过期证据误验收风险；`/diag/dump_event` 已提供 C++/roscpp 事件切片服务，按事件时间窗生成 manifest、rosbag filter 命令和 PCAP 提取命令 |
| 板端运行运维 | `mine_slam_bringup` | `runtime_ops.sh` | 已提供 dry-run 可验证的板端运行计划脚本，生成 `runtime.env`、`start_runtime.sh`、`disk_guard.sh`、`watchdog_check.sh`、`runtime_health.sh`、`runtime_deployment_check.sh`、`runtime_stability_check.sh`、`systemd/tunnel-lio.service`、`systemd/tunnel-lio.env`、`install_systemd.sh`、`docker/Dockerfile`、`docker/docker-compose.yaml`、`docker/tunnel-lio.env`、`docker_build.sh`、`docker_up.sh`、`docker_down.sh` 和 `docker_logs.sh`，覆盖 ROS_LOG_DIR 固定、磁盘水位、日志保留、关键 topic 看门狗检查、板端健康快照、systemd/Docker 部署文件与命令骨架检查、实机 systemd active 与 Docker running 运行态检查、长稳周期采样汇总和每次 runtime health 报告复验、systemd 服务化部署骨架、Docker compose 构建/启动/停止/日志封装骨架和可选 `taskset` CPU 绑定；当 `--launch-file bringup_fusion_timoo.launch` 或 `bringup_fusion_tmlidar.launch` 时，默认以 `runtime_name` 作为 `section_session_id` 写入 `runtime.env` 并传入 `start_runtime.sh` 的 roslaunch 命令，也可通过 `--section-session-id` 显式覆盖；`runtime_name`、`runtime_root`、`workspace`、`launch_package`、`launch_file`、`watchdog_topic`、可选 `cpu_set` 和非空 `section_session_id` 在生成目录、写入 `runtime.env` 和组装 roslaunch/watchdog/taskset 参数前必须拒绝 `missing`、`__DUPLICATE_KEY__`、分号、换行和回车，其中 `runtime_root/runtime_name` 还必须拒绝会污染 runtime stability CSV 的逗号，且 `min_free_gb`、`log_retention_days`、`watchdog_timeout_s` 必须为正整数，防止污染长期 runtime 的启动脚本、截面 session 归属、磁盘守护、看门狗和证据环境；`runtime_deployment_check.sh` 输出 `systemd_active_source` 和 `docker_container_status_source`，只有 `systemctl` 来源的 systemd 为 `active` 且 `docker_inspect` 来源的 Docker 容器为 `running` 时才允许 `runtime_process_status=PASS` 和部署证据 PASS；`runtime_stability_check.sh` 直接运行时必须要求 `--samples` 为严格正整数、`--interval` 为非负整数，且每次采样必须复验 health 报告绑定当前 runtime、磁盘/PID 合法并证明 `systemd_active=active`、`docker_container_status=running`，不满足时累加 `health_failures` 并使 summary `overall=FAIL`；`TUNNEL_LIO_SKIP_WATCHDOG=1` 仅允许离线 dry-run 诊断采样，采样行写出 `watchdog_status=SKIP` 时必须计入 `watchdog_skipped` 并使 summary `overall=FAIL`，不能作为最终长稳 PASS 证据；`TUNNEL_LIO_SYSTEMD_ACTIVE`/`TUNNEL_LIO_DOCKER_STATUS` 环境变量覆盖会被标记为 `env_override`，不能通过最终验收；后续仍需实机 systemd/Docker 启停和 24 h 长稳 |
| IMU Modbus 驱动 | `imu_modbus_driver` | `/sensors/imu/raw`、`/diagnostics/imu_modbus` | 已参数化 IP/端口/topic/frame/rate/诊断阈值、饱和告警阈值、温度寄存器、温度阈值、三组 IMU 协方差对角项和时间来源状态，含寄存器解析、数据有效性、运行频率、读取延迟、读取失败计数、无效样本计数、饱和样本计数、温度寄存器 `float32/int32/int16/uint16` 解码、温度样本/告警计数、重连尝试/成功计数、协方差对角矩阵、协方差 diagnostics 字段、`timestamp_source`、`hardware_time_status`、`pps_status` 诊断单测；温度诊断默认关闭，现场确认温度寄存器后启用；`imu_modbus.launch` 已纳入 launch-check |
| 三雷达融合 | `lidar_fusion` | `/points_raw`、`/diagnostics/lidar_fusion` | 已保留现有 2/3 雷达 TF 融合入口，Timoo/TMLidar 配置分离；三雷达主融合输出已使用带 `sensor_id` 的点型，保留 `ring/time/sensor_id` 来源字段；已发布回放友好的分号键值诊断 topic，包含 sync span、每路点数/frame、drop 计数和 `overlap_pairs/overlap_rmse/overlap_max/overlap_status` 重叠区一致性诊断；融合诊断格式化和重叠残差计算已拒绝非有限距离、`nan/inf` 数值和分号/换行文本污染，节点配置会将非有限 sync/TF/voxel/overlap 阈值重置为保守默认；`multi_lidar_fusion.launch` 已纳入 launch-check |
| 外参与 TF 审计 | `mine_slam_calibration` | `config/extrinsics.yaml`、`publish_extrinsics.launch`、`audit_tf.launch`、`/calib/audit_tf`、`audit_extrinsics.py` | 静态 TF 发布和在线审计入口均已迁移为 C++/roscpp 节点；已集中管理 `base_link/lidar_center/lidar_left/lidar_right/imu_native/imu_link`；`imu_native -> imu_link` 已配置为绕 Z 轴 +90 deg，对应原生 Y 前、Z 上到 ROS x 前、Y 左、Z 上的显式轴适配；C++ 审计已覆盖必需 frame、单根 TF 树、多父节点、循环、非有限数值、异常平移量级和 RPY 歧义角，并通过 `/calib/audit_tf` 返回结构化报告；Python 审计脚本保留为离线工具 |
| 时间状态监控 | `lio_time_manager` | `/time/status` | 已迁移为 C++/roscpp 节点；已统计点云/IMU 频率、延迟、stale、时间回退、PPS/设备时间偏移均值与抖动；已接入 `/time/pps_event`，输出 PPS 事件频率、最近间隔、间隔 jitter、stale 和时间回退诊断；已聚合 `/diagnostics/imu_modbus` 和 `/diagnostics/plc_modbus` 中的发布频率、读取 RTT、读取错误和无效帧状态，并下沉 IMU `timestamp_source/hardware_time_status/pps_status`，使现场证据能区分硬件时间、PPS 与 host 时间兜底；IMU 无效样本、饱和样本、温度样本/告警、三组协方差、重连尝试和重连成功会分别通过 `invalid_frame_count`、`saturation_count`、`temperature_sample_count`、`temperature_warning_count`、`latest/min/max_temperature_c`、`orientation/angular_velocity/linear_acceleration_covariance_*`、`reconnect_attempt_count` 和 `reconnect_success_count` 进入 `/time/status`；诊断数值字段必须拒绝前导空白、尾部污染、`nan/inf` 和 int 越界，防止异常 IMU/PLC diagnostics 污染 time sync 现场证据；传感器、设备时间和 PPS 事件观测入口必须拒绝非有限 `sensor_stamp/device_time/host_time/event_stamp/receipt_time`，防止 NaN/Inf 时间戳污染频率、latency、clock offset、PPS interval/jitter 和 time sync 证据；传感器、PPS 和 diagnostics stale 判定必须把非有限 `now`、非法 stale 阈值和非有限 diagnostic receipt time 保守处理为 stale/未接收，防止 time sync 健康状态被 NaN 时间误报 fresh |
| 点云预处理接口 | `lio_preprocess` | `/lio/points_deskewed`、`/diagnostics/lio_preprocess` | 已迁移为 C++/roscpp 节点；已实现距离过滤、近机体遮蔽、IMU 可用性门控、PointCloud2 输入字段/布局 fail-closed 校验、点内时间字段识别、IMU 角速度连续时间去畸变、去畸变配置/IMU 样本 fail-closed 校验和诊断首版 |
| 局部 ICP/NDT/GICP 里程计 | `lio_local_odometry` | `/lio/odom_local`、`/map/local_submap`、`/diagnostics/lio_local_odometry` | 已实现多尺度 PCL ICP、NDT 粗配准 + ICP 精配准可选路径、GICP/Surfel 协方差感知注册可选路径、位姿累计、fitness + 几何退化验收门控、协方差特征值可观测性评分、滚动局部子图、子图质量治理、可配置输入队列、连续拒绝后几何可观测 keyframe reseed 恢复和注册配置/质量门控 fail-closed 校验首版；`/diagnostics/lio_local_odometry` 已下沉 `keyframe_reseeds/cloud_queue_size/reseed_keyframe_after_consecutive_rejections`，用于区分真实连续跟踪和保守恢复 |
| IMU 状态预测接口 | `lio_state_estimator` | `/lio/state_predict`、`/diagnostics/lio_state_estimator` | 已迁移为 C++/roscpp 节点；已实现 IMU 滑窗频率、均值、振动、健康评分、短时预积分、静止窗口判定、gyro bias 估计、gravity 方向估计、非有限 IMU 样本/预积分观测 fail-closed 校验和 `invalid_sample_count/rejected_updates` 诊断首版，并将静止窗口 gyro bias 与 gravity direction 接入预积分角速度/重力补偿修正；bias/gravity 阈值仍需现场标定 |
| 工况状态机与 PLC 接入 | `machine_state_manager` | `/plc/left_track_speed`、`/plc/right_track_speed`、`/plc/cutting_on`、`/session/relocalizing`、`/machine/state`、`/diagnostics/machine_state`、`/diagnostics/plc_modbus` | 已迁移为 C++/roscpp 节点；已覆盖 `IDLE_STATIC/CUTTING_STATIC/FWD_MOVE/REV_MOVE/TURNING/CMD_MOVE_NO_DISP/CONFLICT/RELOCALIZING` 八态；PLC 有运动命令但 LiDAR 未确认位移时输出 `CMD_MOVE_NO_DISP` 并冻结写图，`/session/relocalizing` 为 true 时优先输出 `RELOCALIZING` 并在诊断下沉 `relocalizing`；已新增 PLC Modbus TCP 首版节点、寄存器解析核心、状态有效位、链路诊断、PLC parser 配置 fail-closed 校验和非有限工况信号/阈值保守 `CONFLICT` 分类首版，现场需复核寄存器表和比例系数 |
| 会话与断电恢复入口 | `lio_session_manager` | `/session/status`、`/session/snapshot`、`/session/recover`、`/session/current_recovery_cloud`、`/session/stable_anchor_cloud` | 已迁移为 C++/roscpp 节点；已实现完整 manifest 快照 WAL、manifest 原子写入、manifest 丢失/损坏时的 WAL 回放恢复、`session_id` 路径 token gate、manifest `state` 枚举 gate、manifest 时间单调 gate、manifest 写侧/读侧/恢复决策 fail-closed gate、snapshot_event token gate、WAL record 写入与回放单行/JSON/event/duplicate-key/payload semantic/consistency gate、稳定图台账恢复锚点读取、稳定锚点 `keyframe_id` token gate、多尺度 ICP 精配准、恢复配准质量门控、local/TCA/global 三级保守恢复决策和恢复分数/锚点/alignment gate fail-closed 校验 |
| 截面与长度控制策略 | `mapping_control` | `/mapping/control`、`/diagnostics/mapping_control` | 已迁移为 C++/roscpp 节点；已实现冻结/拒绝/弱方向限幅、控制断面间距、A/B/C 截面质量分级策略和 mapping policy 输入/配置 fail-closed 校验首版；`CMD_MOVE_NO_DISP` 明确拒绝扩图并下沉 `commanded_motion_without_displacement`，`CONFLICT/RELOCALIZING` 保守拒绝写稳定图 |
| 截面管理 | `section_manager` | `/section/observed`、`/section/structural`、`/section/export`、`/diagnostics/section_manager` | 已迁移为 C++/roscpp 节点；已实现按链距切片、观测截面输出、矩形结构截面 RMSE、完整度估计、质量等级、按链距范围/最低质量门槛 CSV 导出，以及按 `section_spacing_m` 对历史截面做间距去重和质量优先替换；`section_manager.launch` 已接收 `session_id` 参数，fusion bringup 通过 `section_session_id` 透传；截面结构化消息、诊断和 CSV 已包含 `session_id` 与 `state_source`，其中 `state_source` 来自 `/mapping/control` 下沉的 `machine_state`；来自 `/mapping/control` 的 `chainage_m` 已使用严格分号键值解析，字段名必须精确匹配，重复 key、前导空白、尾部污染、`nan/inf` 或溢出均回退到上一链距，不得由 `std::stod` 抛异常或部分解析污染截面链距；`section_sample` 和 `machine_state` 同样使用严格字段解析，重复、哨兵、前导空白、换行污染或后缀字段名匹配均回退默认/上一状态，防止截面采样开关和工况来源被污染；截面提取入口已对 slice/rectangle/chainage/point 坐标执行 fail-closed 校验，历史替换和 CSV 导出必须拒绝非有限指标、负 RMSE、非法质量等级或污染文本，防止 NaN/非法截面成果进入结构化输出和证据包 |
| TCA 锚点管理 | `tca_manager` | `/tca/detection`、`/tca/match`、`/tca/register`、`/diagnostics/tca_manager` | 已迁移为 C++/roscpp 节点；已实现高强度标靶检测、空间聚类、上下文签名、点云/上下文/台账输入 fail-closed 校验、JSON 台账、候选匹配和非唯一拒绝首版 |
| 保守后端治理 | `slam_backend_manager` | `/backend/loop_candidate`、`/backend/loop_verified`、`/backend/stable_promotion`、`/diagnostics/slam_backend` | 已迁移为 C++/roscpp 节点；已实现参数化 Scan Context 风格环-扇区几何描述子、参数化 ISC 风格强度上下文描述子、后端 submap PointCloud2 读取 fail-closed 校验、描述子/候选配置 fail-closed 校验、描述子输入点非有限坐标和强度污染 fail-closed 校验、候选 keyframe 链距、descriptor bin 与 `keyframe_id` token 污染 fail-closed 校验、旋转对齐相似度、ring key、best yaw shift/second-best 诊断、可配置旋转唯一性门控、保守候选选择、链距间隔约束、Top1/Top2 非唯一拒绝、几何包络精验证、轻量 ICP 精配准验证及 ICP 输入点/阈值 fail-closed 校验、unresolved 回环清除、稳定图 JSON 台账、一维链距位姿图优化、可调 Ceres 非线性 6DoF 位姿图优化、协方差加权、Huber 风格鲁棒核、图优化输入 fail-closed 校验和位姿图 `keyframe_id` token 污染 fail-closed 校验首版；来自 `/mapping/control` 的 `chainage_m` 已使用严格分号键值解析，字段名必须精确匹配，重复 key、前导空白、尾部污染、`nan/inf` 或溢出均回退到上一链距，防止后端 keyframe 链距被畸形控制字段污染；`quality` 同样使用严格字段解析，重复、哨兵、前导空白、换行污染或后缀字段名匹配均回退上一质量，防止稳定图晋升质量被污染；稳定图晋升策略自身要求 `min_stable_quality` 为 A/B/C，否则不允许任何 keyframe 晋升；稳定图台账读取和晋升写入均要求 `keyframe_id` 为安全 token，拒绝 `.`、`..`、分号、空白、换行、回车、斜杠或其它污染字符，且要求 `chainage_m/promoted_at` 存在且有限、`section_quality` 为 A/B/C，损坏或不完整条目必须跳过且不得覆盖已有稳定锚点 |
| HIL/回放验收工具 | `lio_eval_tools` | `validation_report.launch`、`validation_thresholds.yaml`、`scenario_validation_thresholds.txt`、`sample_validation_metrics.txt`、`sample_replay_events.txt`、`sample_evidence_manifest.txt` | 已实现 C++/roscpp 离线报告节点、分号键值指标解析、规范化 replay/HIL 事件流聚合、静止漂移/长度误差/恢复时间/错回环/队列堆积/PPS 抖动阈值判定、按场景阈值覆盖、场景阈值缺省字段运行时默认合并、畸形场景阈值 fail closed、metrics/event 重复 key fail closed、场景阈值重复 key fail closed、批量报告生成、证据包 manifest 完整性校验、time sync 证据内容校验、PPS/PTP wiring 独立证据校验、power-loss resume 独立证据校验、runtime health 快照内容校验、runtime deployment 部署检查内容校验、runtime stability CSV/summary 校验、section export CSV 校验、field acceptance 最终验收校验、ISO 秒级证据时间戳真实日历日期校验、launch-check 和 launch 自动收尾契约首版；metrics 文件和 replay event 中的数值指标必须完整解析且为有限值，replay event 中参与恢复时间计算的 `t` 时间戳也必须完整解析且为有限值，前导空白、尾随污染、`nan/inf`、畸形整数字段、整型溢出或畸形恢复时间戳都必须触发对应指标 FAIL，不得被截断解析或按默认值误放行；metrics 文件和 replay event 的任一分号键值记录不得包含重复 key，数值或文本 key 重复都必须使对应记录/聚合指标 FAIL，不得以前写或后写覆盖制造通过状态；`scenario_validation_thresholds.txt` 任一记录不得包含重复 key，尤其重复 `scenario` 不得使场景覆盖静默失效；检测到重复 key 的阈值记录必须毒化整批场景阈值，使对应 batch fail closed；所有通过 `validEvidenceTextValue()` 进入验收语义的 manifest 顶层字段、证据文件路径和关键文本字段均必须拒绝空值、首尾空白、`missing`、`__DUPLICATE_KEY__`、分号、换行和回车；`section_export` 已提升为 evidence manifest 必需证据键，缺失、路径无效或 CSV 未通过时，`section_export_status`、`field_acceptance_status` 和最终 `evidence_status` 均必须保持 FAIL；time sync 必须证明 `/time/status` 已捕获、PPS 诊断 PASS、clock offset 诊断 PASS，并包含可解析数值 `pps_jitter_ms` 与 `mean_offset_ms`，其中 PPS jitter 必须非负；PPS/PTP wiring 必须证明独立 time sync 证据本身 PASS、wiring 报告内 time sync PASS、PPS/clock offset PASS、可解析数值 jitter/offset、人工接线确认 PASS、`wiring_confirmation_source=manual_file`、`pps_wiring_verified=PASS`、`ptp_wiring_verified=PASS`、`wiring_verified_by` 和 `wiring_verified_at`，报告输出 `pps_ptp_wiring_status=PASS/FAIL`；section export 必须包含固定 CSV 表头 `session_id,chainage_m,state_source,quality,completeness,rmse_mm,points` 和至少一条数据行，且所有非空数据行必须满足 `session_id` 与 evidence manifest 会话一致、7 字段非空、`state_source` 为 `IDLE_STATIC/CUTTING_STATIC/FWD_MOVE/REV_MOVE/TURNING/CMD_MOVE_NO_DISP/CONFLICT/RELOCALIZING` 之一、`quality` 为 A/B/C、`chainage_m/completeness/rmse_mm` 可解析、`completeness` 位于 0-1、`rmse_mm` 非负、`points` 为正整数，报告输出 `section_export_status=PASS/FAIL`；power-loss resume 必须证明总字段 `power_loss_resume_status` 严格等于 `PASS`，且来源只能是 `metrics_report` 或 `manual_file`，`metrics_report` 来源还必须包含非空 `metrics_report` 字段，且该字段解析后必须与 evidence manifest 的 `metrics_report` 路径一致，并且 metrics 报告本身必须通过 `overall=PASS`、严格正整数 `total_records`、`failed_records=0`、当前 `session/scenario/status=PASS/failed_checks=0` 匹配记录和无重复 key 校验，verified/field acceptance 报告中的 `recovery_time_s` 还必须与匹配记录块内的 `recovery_time_s` 数值一致，两类来源都必须包含可解析的 `recovery_time_s/max_recovery_time_s` 并满足 `recovery_time_s <= max_recovery_time_s`，报告输出 `power_loss_resume_status=PASS/FAIL`；runtime health 必须包含非空且非哨兵值的 `runtime_dir`、可解析且非负的 `disk_available_gb`、严格正整数 `runtime_pid`，且必须证明 `systemd_active=active`、`docker_container_status=running`；缺字段、空值、非数值、非 active/running 或 `runtime_pid=missing/0` 均输出 `runtime_health_status=FAIL`；runtime deployment 必须同时证明非空且非哨兵值的 `runtime_dir`、systemd/Docker 文件骨架、启动命令、`systemd_active=active`、`systemd_active_source=systemctl`、`docker_container_status=running`、`docker_container_status_source=docker_inspect` 和 `runtime_process_status=PASS`，禁止 `env_override` 作为实机证据；runtime stability CSV 必须包含固定采样表头和至少一条采样记录，且所有非空采样记录必须满足 `disk_guard_status=PASS`、`watchdog_status=PASS`、`health_report` 非空且不是 `missing` 或 `__DUPLICATE_KEY__`，且不含分号、换行或回车，summary 的 `overall` 字段必须严格等于 `PASS`、`samples` 必须是正整数并等于 CSV 非空采样记录数、`interval_s` 必须显式存在且为严格正整数，且 `disk_failures/watchdog_failures/watchdog_skipped/health_failures` 必须显式存在并且字面量严格等于 `0`，`runtime_stability_run_log` 的 `exit_status` 也必须字面量严格等于 `0`；field acceptance 必须同时证明 metrics、time sync、runtime health、runtime deployment、runtime stability CSV/summary、断电续建、PPS/PTP 接线确认和 section export 全部 PASS，且最终报告自身必须包含符合 ISO-8601 seconds 且日历日期真实的 `timestamp`、有效 `time_status_topic/pps_topic`、可解析的 `pps_jitter_ms/mean_offset_ms`、`pps_wiring_verified=PASS`、`ptp_wiring_verified=PASS`、`wiring_verified_by` 和 `wiring_verified_at`，其中 PPS jitter 必须非负，并下沉严格正整数 `runtime_stability_samples/runtime_stability_interval_s` 且与 summary 中的 `samples/interval_s` 一致、必须下沉 `runtime_stability_csv_status=PASS`、`runtime_stability_csv_samples` 与 `runtime_stability_sample_count_match=PASS`，以及字面量为 `0` 的 `runtime_stability_disk_failures/runtime_stability_watchdog_failures/runtime_stability_watchdog_skipped/runtime_stability_health_failures`，部署运行态来源必须为 `systemctl`/`docker_inspect`、PPS/PTP 来源必须为 `manual_file`、断电续建来源不得为 `manual_env` 且必须重验恢复时间门槛；`validation_report_node` 可将指标 PASS/FAIL 与 metrics/event/bag/pcap/TF/参数/日志/时间同步、PPS/PTP 接线、断电续建、截面成果、板端健康、部署检查、长稳采样和最终现场验收证据完整性合并输出 |

补充说明：`lio_eval_tools` 的 evidence manifest 校验中，所有必需证据文件不仅要存在，还必须是非空 regular file；空 bag、PCAP、TF 快照、参数快照、运行日志、metrics/event 报告、time sync、PPS/PTP wiring、runtime health/deployment/stability、power-loss resume、section export 或 field acceptance 文件均按缺失证据处理。

v4.40 运行态来源补充：runtime health 与 runtime deployment 使用同一实机来源口径，`systemd_active_source` 必须为 `systemctl`，`docker_container_status_source` 必须为 `docker_inspect`。`runtime_health.sh`、长稳 health 复验、`capture_field_acceptance.sh` 和 `lio_eval_tools` evidence manifest 均必须拒绝缺失来源、`env_override` 或 `unavailable` 来源。

v4.41 runtime health 时间戳补充：runtime health 快照必须带符合 ISO-8601 seconds 且日历日期真实的 `timestamp`，并在最终 `field_acceptance_report.txt` 中下沉为 `runtime_health_timestamp`。`capture_field_acceptance.sh` 必须要求该快照时间戳合法、最终验收报告 `timestamp` 不早于 runtime health 快照时间；`lio_eval_tools` evidence manifest 必须同时复核独立 runtime health 时间戳、最终报告下沉字段与独立 health 完全一致，以及最终报告时间覆盖该 health 快照。缺失、畸形、不可能日期、未来 health 快照或最终报告与独立 health 时间不一致时，`runtime_health_status`、`field_acceptance_status` 和 `evidence_status` 均保持 FAIL。

v4.42 runtime deployment 时间戳补充：runtime deployment 部署检查快照必须带符合 ISO-8601 seconds 且日历日期真实的 `timestamp`，并在最终 `field_acceptance_report.txt` 中下沉为 `runtime_deployment_timestamp`。`capture_field_acceptance.sh` 必须要求该快照时间戳合法、最终验收报告 `timestamp` 不早于 deployment 快照时间；`lio_eval_tools` evidence manifest 必须同时复核独立 runtime deployment 时间戳、最终报告下沉字段与独立 deployment 完全一致，以及最终报告时间覆盖该 deployment 快照。缺失、畸形、不可能日期、未来 deployment 快照或最终报告与独立 deployment 时间不一致时，`runtime_deployment_status`、`field_acceptance_status` 和 `evidence_status` 均保持 FAIL。

v4.43 time sync 时间戳补充：time sync 捕获快照必须带符合 ISO-8601 seconds 且日历日期真实的 `timestamp`，并在最终 `field_acceptance_report.txt` 中下沉为 `time_sync_timestamp`。`capture_field_acceptance.sh` 必须要求该快照时间戳合法、最终验收报告 `timestamp` 不早于 time sync 快照时间；`lio_eval_tools` evidence manifest 必须同时复核独立 time sync 时间戳、最终报告下沉字段与独立 time sync 完全一致，以及最终报告时间覆盖该 time sync 快照。缺失、畸形、不可能日期、未来 time sync 快照或最终报告与独立 time sync 时间不一致时，`time_sync_status`、`field_acceptance_status` 和 `evidence_status` 均保持 FAIL。

v4.44 PPS/PTP wiring 时间戳绑定补充：PPS/PTP wiring 独立 verified 报告必须把当前独立 time sync 快照时间下沉为 `time_sync_timestamp`，最终 `field_acceptance_report.txt` 必须再下沉为 `pps_ptp_wiring_time_sync_timestamp`。`capture_pps_ptp_wiring.sh`、`capture_field_acceptance.sh` 和 `lio_eval_tools` evidence manifest 必须同时要求这两个下沉字段与独立 time sync 的 `timestamp` 完全一致。缺失、畸形、不一致或 stale PPS/PTP wiring 证据时，`pps_ptp_wiring_verified/pps_ptp_wiring_status`、`field_acceptance_status` 和 `evidence_status` 均保持 FAIL。

v4.45 PPS/PTP wiring 报告生成时间补充：PPS/PTP wiring 独立 verified 报告自身必须包含符合 ISO-8601 seconds 且日历日期真实的 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉为 `pps_ptp_wiring_timestamp`。`capture_field_acceptance.sh` 和 `lio_eval_tools` evidence manifest 必须同时要求最终下沉字段与独立 wiring 报告 `timestamp` 完全一致，且最终报告自身 `timestamp` 不早于该 wiring 报告生成时间。缺失、畸形、不一致或最终报告时间早于 PPS/PTP wiring 报告生成时间时，`pps_ptp_wiring_verified/pps_ptp_wiring_status`、`field_acceptance_status` 和 `evidence_status` 均保持 FAIL。

v4.46 power-loss resume 报告生成时间补充：power-loss resume 独立 verified 报告自身必须包含符合 ISO-8601 seconds 且日历日期真实的 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉为 `power_loss_resume_timestamp`。`capture_field_acceptance.sh` 和 `lio_eval_tools` evidence manifest 必须同时要求独立报告 timestamp 合法、最终下沉字段与独立 power-loss resume 报告 `timestamp` 完全一致，且最终报告自身 `timestamp` 不早于该 power-loss resume 报告生成时间。缺失、畸形、不一致或最终报告时间早于 power-loss resume 报告生成时间时，`power_loss_resume_status`、`field_acceptance_status` 和 `evidence_status` 均保持 FAIL。

v4.47 runtime stability summary 生成时间补充：24 h 长稳 summary 自身必须包含符合 ISO-8601 seconds 且日历日期真实的 `timestamp`，且该时间必须落在独立 `runtime_stability_run.log` 的 `started_at/finished_at` 闭区间内；最终 `field_acceptance_report.txt` 必须下沉为 `runtime_stability_summary_timestamp`，并要求最终报告自身 `timestamp` 不早于该 summary 生成时间。`runtime_ops.sh`、`record_session.sh` 生成脚本和 `lio_eval_tools` evidence manifest 必须一致拒绝 summary timestamp 缺失、畸形、越出 run log 时间窗、最终下沉缺失/不一致或最终报告时间早于 summary 生成时间的长稳证据。

v4.48 runtime stability CSV 采样时间补充：24 h 长稳 CSV 每条采样记录的 `timestamp` 必须符合 ISO-8601 seconds、日历日期真实，并按 `sample_index` 非递减。最终 `field_acceptance_report.txt` 必须下沉独立 CSV 推导出的 `runtime_stability_csv_first_timestamp` 与 `runtime_stability_csv_last_timestamp`，要求首末时间合法、`first <= last`、均落在 `runtime_stability_run.log` 的 `started_at/finished_at` 闭区间内，且最终报告自身 `timestamp` 不早于最后一条 CSV 采样时间。`record_session.sh` 生成脚本和 `lio_eval_tools` evidence manifest 必须一致拒绝 CSV 采样时间缺失、畸形、倒序、越出 run log 时间窗、最终下沉缺失/不一致或最终报告时间早于最后采样时间的长稳证据。

v4.49 runtime stability CSV health report 实物证据补充：24 h 长稳 CSV 每条采样记录的 `health_report` 不能只满足安全文本格式，还必须能解析到当前 session/evidence bundle 内的非空 regular file。`run_runtime_stability.sh` 归档板端长稳 CSV 时，必须把 CSV 中引用的 runtime health 快照复制到 session `logs/` 并将 `health_report` 改写为安全 basename；`capture_field_acceptance.sh` 和 `lio_eval_tools` evidence manifest 必须一致拒绝缺失、空文件、越出 session/bundle 或不安全路径引用的 health report 采样证据。

v4.50 runtime stability CSV health report 内容复验补充：每条 CSV `health_report` 引用的 runtime health 快照必须内容本身通过 runtime health PASS 语义，并绑定当前 manifest/session 的 `runtime_dir`。`capture_field_acceptance.sh` 和 `lio_eval_tools` evidence manifest 必须逐条复验 health 快照的合法时间戳、运行目录、非负磁盘余量、严格正整数 PID、`systemd_active=active/systemd_active_source=systemctl` 和 `docker_container_status=running/docker_container_status_source=docker_inspect`，防止仅引用存在但内容失败、来源覆盖或 runtime_dir 不匹配的 health 文件伪造 24 h 长稳 PASS。

v4.51 实际海底隧道 bag 回放补充：`mine_slam_bringup/scripts/actual_bag_replay.sh`、`bringup_replay_tunnel_bag.launch` 和 `Tunnel.bag` 专用配置已形成真实数据回放入口。该入口默认只把 `/velodyne_points`、`/left/lslidar_point_cloud`、`/right/velodyne_points`、`/imu/data` 和 `/time_reference` 播放给 SLAM，显式排除 `/novatel_data/inspvax`，仅离线读取首个 NovAtel 速度样本作为初始非零速度审计；左雷达 legacy XYZI 点云只能在专用兼容配置中补 `ring=0/time=0`，并通过 `legacy_xyzi_clouds/legacy_xyzi_points` 诊断计数暴露，不得静默当作完整 XYZIRT 驱动输出。

v4.52 实际 bag replay 进程隔离补充：实际数据回放命令必须启动自有临时 `ROS_MASTER_URI` 和受控 `roscore`，所有 roslaunch、rosbag play 与 rostopic capture 均继承该 master；退出时必须先清理 pipeline process group，再清理 roscore process group，并用 `pgrep` 证明无 `roslaunch/rosbag/rostopic/roscore/rosmaster` 残留，避免连续回放时污染默认 11311 master 或占用端口。

v4.53 实际 bag replay 覆盖 gate 补充：实际数据回放诊断必须覆盖回放窗口，`capture_timeout_s` 按 `ceil(duration / rate) + 30 s` 计算，避免慢速回放时提前结束 capture；summary 必须输出 `minimum_fusion_published` 和 `fusion_duration_coverage_status`。当前 `Tunnel.bag` 为海底隧道实际包，首个 NovAtel 速度样本为 4.417557 m/s，仅用于 `START_ONLY_AUDIT` 初始非零速度审计，`/novatel_data/inspvax` 不进入 rosbag play。当前证据显示 60 s、0.3x 慢速真实数据回放通过覆盖 gate；195 s、1.0x 全包回放能跑到 pipeline 无错误且无速度参考污染，但融合发布量未达到 5 Hz 覆盖门槛，属于当前机器/配置下实时吞吐待优化，不得据此宣称最终 `field_acceptance_status=PASS`。

v4.54 实际 bag replay 局部里程计诊断补充：实际数据回放必须同步捕获 `/diagnostics/lio_local_odometry`，summary 必须下沉 `lio_local_odometry_diag_captured`、`local_odometry_clouds`、`local_odometry_published`、`local_odometry_published_status` 和 `local_odometry_rejected_registrations`，并把 local odometry 诊断捕获和正向 odom 发布纳入 PASS gate，使 1.0x 全包吞吐问题能区分三雷达融合覆盖、预处理、局部 ICP/NDT/GICP 注册和 odom 发布层。

v4.55 实际 bag replay 诊断解析补充：实际数据回放 summary 解析器必须同时支持 `std_msgs/String` 风格的 `data: "key=value;..."` 诊断和 ROS `DiagnosticArray` YAML 的跨行 `key:`/`value:` 诊断；同一行 `key=value` 的 key 左边界必须允许引号等非 key 字符，防止 `callbacks` 在 `data: "callbacks=..."` 中被误解析为 0。当前 60 s、0.3x 真实 bag 证据已验证 `fusion_callbacks=576`、`fusion_published=576`、`local_odometry_clouds=312`、`local_odometry_published=257`、`local_odometry_rejected_registrations=55`、`fusion_duration_coverage_status=PASS`。

v4.56 实际 bag replay 质量 gate 补充：实际数据回放 summary 必须输出 `minimum_local_odometry_published` 与 `local_odometry_duration_coverage_status`，并将 local odometry 覆盖率纳入 overall PASS gate；仅 `local_odometry_published > 0` 不再足以通过。三雷达融合 overlap 诊断增加 `overlap_max_points` 抽样上限，避免每帧全量 10 万点重叠诊断压低回放吞吐；`lio_local_odometry` 增加大坐标稳健 voxel 降采样、注册器外部初值 overload、仅基于上一段 SLAM 成功注册的常速平移初值和 `/diagnostics/lio_local_odometry` 初值字段，不使用 `/novatel_data/inspvax` 连续速度。当前完整 `Tunnel.bag` 195 s、0.3x 质量模式证据已通过：`fusion_published=1906 >= 975`、`local_odometry_published=787 >= 195`、`local_odometry_duration_coverage_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；1.0x 实时回放仍存在 local odometry 末端连续拒绝/吞吐不足，保留为性能优化项，不能作为最终现场验收或实时验收 PASS。

v4.57 实际 bag replay 初测边界补充：当前海底隧道 `Tunnel.bag` 明确按 `INITIAL_LIDAR_IMU_ONLY` 初测处理，bag 内无 PLC 反馈状态，车辆假设为 `CONTINUOUS_MOTION`，振动等级按 `NORMAL` 记录；`actual_bag_replay.sh` 生成的 plan、inspection 和 summary 必须写出 `bag_sensor_set=LIDAR_IMU_ONLY`、`plc_feedback_status=NOT_PRESENT_NA`、`plc_feedback_gate_status=NA_INITIAL_TEST` 和 `field_acceptance_requires_plc_feedback=YES`。回放启动命令必须关闭无 PLC 情况下会误判的状态/截面门控，即传入 `start_machine_state:=false`、`start_mapping_control:=false` 和 `start_section_manager:=false`；这条包只验证三雷达/IMU 前端、去畸变、局部里程计、状态预测和实际数据证据入口，不验证 PLC 状态机、截割/空转门控、section export、断电续建、PPS/PTP 接线或最终 `field_acceptance_status=PASS`。

v4.58 工况八态闭环补充：`machine_state_manager` 已把规划中的 `CMD_MOVE_NO_DISP` 和 `RELOCALIZING` 接入在线分类、冻结策略和诊断输出；`mapping_control` 对 `CMD_MOVE_NO_DISP` 明确拒绝扩图，对 `CONFLICT/RELOCALIZING` 保守拒绝写稳定图；`record_session.sh` 与 `lio_eval_tools` 的 section export 校验同步接受八态 `state_source`，防止正式 PLC/HIL 截面证据因状态枚举落后被误拒绝或绕过。该补强是软件侧门控闭环，不替代真实 PLC 反馈、PPS/PTP 接线和现场/HIL 最终验收。

v4.59 实际 bag replay 到 HIL 证据桥接补充：`actual_bag_replay.sh` 必须在实际执行 summary 外额外生成 `reports/actual_bag_replay_metrics_report.txt` 和 `reports/actual_bag_replay_events.txt`，并生成 `commands/validate_actual_bag_events.sh` 直接调用 `lio_eval_tools validation_report.launch` 消费该 event_file。两类文件必须写明 `validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY` 和 `field_acceptance_eligible=NO`；dry-run 只能输出 metrics `overall=FAIL`、`failed_records=1` 和 event `queue_backlog=-1`，防止未执行计划被当成 PASS。实际执行时只有 `actual_bag_replay_status=PASS` 才允许 metrics summary PASS；这仍只证明 LiDAR+IMU 前端/局部里程计初测，不替代 PLC、section export、PPS/PTP、断电续建、24 h 长稳或最终 `field_acceptance_status=PASS`。

v4.60 实际 bag 局部里程计恢复补充：海底隧道 LiDAR+IMU-only 初测 bag 的初始速度不为 0，且无 PLC 状态反馈。`lio_local_odometry` 已增加可配置输入队列、连续注册拒绝后的几何可观测 keyframe reseed 和 `keyframe_reseeds` 诊断，`actual_bag_replay.sh` 必须把 `local_odometry_keyframe_reseeds` 下沉到 summary、metrics 和 event。当前 60 s、0.3x 真实 bag 证据 `reports/actual_bag_replay_tunnel_v460_reseed_60s_rate03` 已通过：`fusion_published=576 >= 300`、`local_odometry_published=284 >= 60`、`local_odometry_rejected_registrations=14`、`local_odometry_keyframe_reseeds=7`，且 `validate_actual_bag_events.sh` 输出 `overall=PASS`；该证据仍声明 `validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY` 和 `field_acceptance_eligible=NO`。

v4.61 实际 bag 实时整包回放补充：在 v4.60 恢复策略基础上，当前海底隧道 `Tunnel.bag` 已完成 60 s/1.0x 和整包 195 s/1.0x 实时初测 replay。60 s/1.0x 证据 `reports/actual_bag_replay_tunnel_v460_reseed_60s_rate10` 通过：`fusion_published=398 >= 300`、`local_odometry_published=106 >= 60`、`local_odometry_rejected_registrations=25`、`local_odometry_keyframe_reseeds=9`；整包 195 s/1.0x 证据 `reports/actual_bag_replay_tunnel_v460_reseed_full195s_rate10` 通过：`fusion_published=1178 >= 975`、`local_odometry_published=210 >= 195`、`local_odometry_rejected_registrations=23`、`local_odometry_keyframe_reseeds=10`，其 generated event validation 输出 `overall=PASS`。该证据说明当前软件链路已可直接用同类 LiDAR+IMU-only bag 做初步实际数据 replay；仍不能证明 PLC 状态机、section export、PPS/PTP 接线、断电续建、systemd/Docker 实机启停或 24 h 长稳，最终现场入口仍必须是 `field_acceptance_status=PASS`。

v4.62 实际 bag 初测套件补充：新增 `actual_bag_test_suite.sh` 作为用户收集 LiDAR+IMU-only bag 跑实际数据前的统一入口，生成 smoke/full 两段 `actual_bag_replay.sh --execute` 命令、两段 `validate_actual_bag_events.sh` 事件校验、suite summary、metrics/events 和 ROS 残留进程检查；`--skip-bag-inspect` 仅用于 dry-run 契约测试，必须显式给出 `--full-duration`，真实 bag 默认从 rosbag 自动读取整包时长。当前 `Tunnel.bag` suite 证据 `reports/actual_bag_test_suite_tunnel_v462_rate10` 已通过：60 s/1.0x smoke `fusion_published=445 >= 300`、`local_odometry_published=116 >= 60`、`local_odometry_keyframe_reseeds=12`，195 s/1.0x full `fusion_published=1341 >= 975`、`local_odometry_published=256 >= 195`、`local_odometry_keyframe_reseeds=13`；suite summary 输出 `actual_bag_test_suite_status=PASS`、`smoke_event_validation_status=PASS`、`full_event_validation_status=PASS`、`ros_residual_status=PASS`、`field_acceptance_eligible=NO`。inspection 证明 PLC topic 计数均为 0，初始速度参考首样本约 `4.417557 m/s`，且 `velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；该 suite 证明同类海底隧道 LiDAR+IMU-only bag 已可用于初步测试流程，但不替代 PLC/PPS/PTP/断电续建/实机部署/24 h 长稳。

v4.63 实际 bag suite manifest 校验补充：`actual_bag_test_suite.sh` 现在额外生成 `reports/actual_bag_test_suite_manifest.txt` 和 `commands/validate_actual_bag_test_suite.sh`，复验 suite summary、suite metrics/event、smoke/full replay summary、两段 event validation、两段初始速度审计、两段 bag inspection 和 ROS 残留报告，要求 `actual_bag_test_suite_status=PASS`、smoke/full replay/event validation 均 PASS、PLC topic 计数均为 0、速度参考未进入 SLAM 且 `field_acceptance_eligible=NO`。当前新证据 `reports/actual_bag_test_suite_tunnel_v463_manifest_rate10` 已通过：suite summary `actual_bag_test_suite_status=PASS`，manifest validation `actual_bag_test_suite_manifest_validation_status=PASS`，full replay `fusion_published=1312 >= 975`、`local_odometry_published=254 >= 195`、`local_odometry_rejected_registrations=39`、`local_odometry_keyframe_reseeds=14`，初始速度首样本 `4.417557 m/s` 仅作 `START_ONLY_AUDIT`，最终 `pgrep` 证明无 ROS 残留进程。该 manifest 校验是实际 bag 初测证据自检，不是最终现场 evidence manifest。

v4.64 实际 bag 到最终现场验收 gap audit 补充：`actual_bag_test_suite.sh` 现在生成 `commands/audit_field_acceptance_gap.sh` 和 `reports/field_acceptance_gap_report.txt`，在 suite manifest validation PASS 后仍固定输出 `field_acceptance_ready=NO` 和 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`，把 LiDAR+IMU-only 初测 PASS 与最终现场 PASS 分离。当前新证据 `reports/actual_bag_test_suite_tunnel_v464_gap_rate10` 已通过：suite summary `actual_bag_test_suite_status=PASS`，manifest validation `actual_bag_test_suite_manifest_validation_status=PASS`，full replay `fusion_published=1325 >= 975`、`local_odometry_published=261 >= 195`、`local_odometry_rejected_registrations=32`、`local_odometry_keyframe_reseeds=14`，初始速度首样本 `4.417557 m/s` 仅作 `START_ONLY_AUDIT`，`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`。同目录 gap report 输出 `actual_bag_initial_evidence_status=PASS`，但 `plc_feedback_evidence_status/section_export_evidence_status/pps_ptp_wiring_evidence_status/power_loss_resume_evidence_status/runtime_deployment_evidence_status/runtime_stability_24h_evidence_status/field_acceptance_report_status` 均为 `MISSING`；该 audit 命令预期非零退出，用作“还差多少”的预现场 gate，而不是最终验收 PASS。

v4.65 用户实际 bag topic 映射补充：`actual_bag_replay.sh` 与 `actual_bag_test_suite.sh` 现在支持 `--center-topic/--left-topic/--right-topic/--imu-topic/--time-reference-topic/--initial-velocity-topic`。默认仍使用当前 `Tunnel.bag` topic；用户 bag topic 不同时，脚本在 inspection、plan 和 dry-run 报告中记录源 topic 与 canonical replay topic，并由 `rosbag play` 将源 topic remap 到 `/velodyne_points`、`/left/lslidar_point_cloud`、`/right/velodyne_points`、`/imu/data` 和 `/time_reference`，不改变 SLAM launch 与融合节点。`--initial-velocity-topic` 只进入起步速度审计和排除字段，仍不得出现在 rosbag play topic 列表中。所有 topic override 必须是绝对 ROS topic，非法 topic 在生成证据目录前 fail closed；suite 会把 topic override 同步透传到 smoke/full replay，避免用户 bag 到正式回放阶段才因 topic 名不匹配失败。当前短时真实 bag smoke 证据 `reports/actual_bag_replay_tunnel_v465_topic_default_smoke_8s` 已通过，summary 记录源/canonical/play/excluded velocity topic 字段，8 s/1.0x `fusion_published=78 >= 40`、`local_odometry_published=28 >= 8`，事件校验 `overall=PASS`，初始速度 `4.417557 m/s` 仍仅作 `START_ONLY_AUDIT`。

v4.66 用户实际 bag 无 time reference 初测补充：`actual_bag_replay.sh` 与 `actual_bag_test_suite.sh` 现在支持显式 `--no-time-reference`，用于用户 bag 只有三雷达/IMU、没有 `/time_reference` 的 LiDAR+IMU-only 初测。该模式不会把 `/time_reference` 放入 `rosbag play --topics`，inspection、plan、summary 和 suite manifest 会写出 `time_reference_topic=NONE`、`time_reference_status=NOT_PRESENT_INITIAL_TEST` 和 `time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST`；同时仍保留 `/time/status` capture，验证 replay 链路内基于 LiDAR/IMU 的时间诊断可发布。`--no-time-reference` 与 `--time-reference-topic` 互斥，避免把缺失外部时间源误标成已提供。当前真实短时证据 `reports/actual_bag_replay_tunnel_v466_no_time_reference_smoke_8s` 已通过：8 s/1.0x `actual_bag_replay_status=PASS`、`fusion_published=78 >= 40`、`local_odometry_published=29 >= 8`、`time_status_captured=PASS`、事件校验 `overall=PASS`，初始速度 `4.417557 m/s` 仍仅作 `START_ONLY_AUDIT`。该 PASS 仍不是 PPS/PTP/time sync 最终验收证据。

v4.67 actual bag suite manifest 时间模式复验补充：`validate_actual_bag_test_suite.sh` 现在把 suite manifest 中的 `time_reference_status/time_sync_evidence_status` 当作整包一致性契约，复验 suite summary、smoke/full replay summary、smoke/full inspection；当 manifest 声明 `NOT_PRESENT_INITIAL_TEST` 时，还要求两段 replay summary 和 inspection 均写 `time_reference_topic=NONE`。新增契约测试证明 no-time-reference 子证据若被篡改为 `PRESENT_REQUIRED`，manifest validation 必须 FAIL。当前真实短 suite 证据 `reports/actual_bag_test_suite_tunnel_v467_no_time_reference_manifest_8s` 已通过：suite summary `actual_bag_test_suite_status=PASS`，manifest validation `actual_bag_test_suite_manifest_validation_status=PASS`，smoke/full 均为 8 s/1.0x replay PASS，二者均记录 `time_reference_status=NOT_PRESENT_INITIAL_TEST`、`time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST` 和 `time_reference_topic=NONE`；gap audit 继续输出 `field_acceptance_ready=NO` 与最终现场缺口清单。

v4.68 用户实际 bag intake/profile 补充：新增 `actual_bag_profile.sh` 作为用户收集 bag 进入初测前的第一道入口。该脚本只读取 rosbag topic/type/字段/frame 和时长，自动选择中心/左/右雷达、IMU、可选 time reference 和初始速度参考 topic，生成 `reports/actual_bag_profile.txt` 与 `commands/run_recommended_actual_bag_test_suite.sh`，不会启动 SLAM，也不会生成最终 field acceptance 证据。若 bag 中无外部 time reference，推荐命令自动使用 `--no-time-reference`；若存在 time reference，则显式推荐 `--time-reference-topic`；若存在 NovAtel/velocity 参考，仅记录为初始非零速度审计，仍写出 `velocity_reference_played_to_slam=NO` 和 `continuous_velocity_reference_used=NO`。当前真实 `Tunnel.bag` profile 证据 `reports/actual_bag_profile_tunnel_v468` 已通过：`actual_bag_profile_status=PASS`、`bag_duration_s=194.997160`、`pointcloud_topic_count=3`、`imu_topic_count=1`、`time_reference_topic=/time_reference`、`center_lidar_topic=/velodyne_points`、`left_lidar_topic=/left/lslidar_point_cloud`、`right_lidar_topic=/right/velodyne_points`、`imu_topic=/imu/data`、`initial_velocity_reference_topic=/novatel_data/inspvax`，且生成的推荐 suite 命令包含 topic override、`--time-reference-topic /time_reference`、`--initial-velocity-topic /novatel_data/inspvax`、`--rate 1.0 --execute`。该 profile 只能证明“用户 bag 可被识别并生成初测命令”，`field_acceptance_eligible` 必须保持 `NO`。

v4.69 profile 生成命令端到端 suite 复验补充：已直接执行 `reports/actual_bag_profile_tunnel_v468/commands/run_recommended_actual_bag_test_suite.sh`，证明 profile 生成的推荐命令不仅能落盘，也能驱动真实 smoke/full replay、事件校验、suite summary、manifest validation 和 field acceptance gap audit。新证据 `reports/actual_bag_profile_tunnel_v468/recommended_suite` 已通过：suite summary `actual_bag_test_suite_status=PASS`，60 s/1.0x smoke `fusion_published=444 >= 300`、`local_odometry_published=114 >= 60`、`local_odometry_keyframe_reseeds=12`，195 s/1.0x full `fusion_published=1311 >= 975`、`local_odometry_published=257 >= 195`、`local_odometry_rejected_registrations=37`、`local_odometry_keyframe_reseeds=15`，两段 event validation 均输出 `overall=PASS;total_records=1;failed_records=0`；生成的 `commands/validate_actual_bag_test_suite.sh` 复验 `actual_bag_test_suite_manifest_validation_status=PASS`，ROS 残留检查为 PASS。`commands/audit_field_acceptance_gap.sh` 预期非零退出，报告仍固定输出 `field_acceptance_gap_audit_status=FAIL`、`actual_bag_initial_evidence_status=PASS`、`field_acceptance_ready=NO` 和最终现场缺口清单，防止把无 PLC 的 LiDAR+IMU-only 初测误当成最终 `field_acceptance_status=PASS`。

v4.70 profile 初测边界字段补充：`actual_bag_profile.sh` 现在在第一份用户 bag profile 报告中直接下沉 LiDAR+IMU-only 初测边界，不再等到 replay/suite 才暴露无 PLC 语义。profile 必须写出 `plc_feedback_topic_count`、`plc_feedback_status`、`plc_feedback_gate_status=NA_INITIAL_TEST`、`machine_motion_assumption=CONTINUOUS_MOTION`、`vibration_profile=NORMAL` 和 `field_acceptance_requires_plc_feedback=YES`，使用户在运行 suite 前即可看到该包只能做前端初测，不能验证 PLC 状态机、截面生产或最终现场验收。当前真实 `Tunnel.bag` 新 profile 证据 `reports/actual_bag_profile_tunnel_v470_motion_semantics` 已通过：`actual_bag_profile_status=PASS`、`plc_feedback_topic_count=0`、`plc_feedback_status=NOT_PRESENT_NA`、`machine_motion_assumption=CONTINUOUS_MOTION`、`vibration_profile=NORMAL`、`field_acceptance_requires_plc_feedback=YES`，并继续推荐 time reference、三雷达、IMU 和初始速度审计 topic override，且 `velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`。

v4.71 profile 起步速度审计补充：`actual_bag_profile.sh` 现在在识别外部速度参考 topic 后直接读取首个样本，支持 NovAtel `north_velocity/east_velocity/up_velocity`、`TwistStamped`/`Odometry` 线速度和简单标量速度，输出 `initial_velocity_reference_status=CAPTURED/UNPARSEABLE/MISSING`、`initial_velocity_reference_policy=START_ONLY_AUDIT`、首样本时间、三轴速度和合速度；该速度仍只用于初始非零运动审计，不得进入连续 SLAM 输入。当前真实 `Tunnel.bag` 新 profile 证据 `reports/actual_bag_profile_tunnel_v471_initial_velocity_sample` 已通过：`initial_velocity_reference_status=CAPTURED`、`initial_velocity_reference_topic=/novatel_data/inspvax`、`initial_velocity_first_sample_stamp_s=1621322534.000000`、`initial_velocity_north_mps=4.416000`、`initial_velocity_east_mps=0.117000`、`initial_velocity_up_mps=-0.008000`、`initial_velocity_speed_mps=4.417557`，并继续写出 `velocity_reference_played_to_slam=NO` 与 `continuous_velocity_reference_used=NO`。该数值与 v4.69 smoke/full replay 的 `initial_velocity_reference.txt` 一致，证明 profile 和 replay 对起步速度审计口径一致。

v4.72 profile 自检 gate 补充：`actual_bag_profile.sh` 现在随 profile 一起生成 `commands/validate_actual_bag_profile.sh`，在运行 suite 前即可复验 profile 报告本身。validator 读取 `reports/actual_bag_profile.txt`，拒绝缺失/重复/畸形 key，要求 `actual_bag_profile_status=PASS`、LiDAR+IMU-only 初测边界、`field_acceptance_eligible=NO`、`plc_feedback_topic_count=0`、速度参考 `START_ONLY_AUDIT`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`、time reference 模式一致、推荐 suite 命令存在且包含 `actual_bag_test_suite.sh --execute`；若速度首样本状态为 `CAPTURED`，还必须证明首样本时间和速度分量可解析。当前真实 `Tunnel.bag` 新证据 `reports/actual_bag_profile_tunnel_v472_profile_validation` 已通过：`actual_bag_profile_validation_status=PASS`、`duplicate_key_count=0`、`malformed_line_count=0`、`recommended_suite_command_status=PASS`、`initial_velocity_reference_policy_status=PASS`、`velocity_reference_played_to_slam_status=PASS`、`continuous_velocity_reference_used_status=PASS`，并继续下沉 `field_acceptance_eligible=NO`。新增篡改测试证明若把 `continuous_velocity_reference_used` 改成 `YES`，profile validation 必须 FAIL。

v4.73 profile 推荐入口自带 gate 补充：`actual_bag_profile.sh` 生成的 `commands/run_recommended_actual_bag_test_suite.sh` 现在必须先执行同目录 `validate_actual_bag_profile.sh`，只有 profile 自检通过后才进入 `actual_bag_test_suite.sh --execute`。当前真实 `Tunnel.bag` 新证据 `reports/actual_bag_profile_tunnel_v473_recommended_gate` 已通过：profile validation 输出 `actual_bag_profile_validation_status=PASS`、`recommended_suite_command_status=PASS`、`field_acceptance_eligible=NO`、`velocity_reference_played_to_slam=NO` 和 `continuous_velocity_reference_used=NO`；生成的推荐入口脚本顺序为 `validate_actual_bag_profile.sh` 在前、`actual_bag_test_suite.sh` 在后。该补强防止用户跳过 profile 自检直接把不完整或被篡改的 profile 当作实际 bag 初测入口。

v4.74 profile validator 反查推荐入口 gate 补充：`commands/validate_actual_bag_profile.sh` 现在不仅复验 profile 报告和推荐 suite 命令字段，还会读取 `commands/run_recommended_actual_bag_test_suite.sh`，要求入口脚本存在 `set -euo pipefail`、包含 `validate_actual_bag_profile.sh`、包含 `actual_bag_test_suite.sh`，且 validator 调用必须排在 suite 调用之前。当前真实 `Tunnel.bag` 新证据 `reports/actual_bag_profile_tunnel_v474_validator_entry_gate` 已通过：`actual_bag_profile_validation_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_command_status=PASS`、`field_acceptance_eligible=NO`、`velocity_reference_played_to_slam=NO` 和 `continuous_velocity_reference_used=NO`。新增篡改测试证明若删除推荐入口脚本中的 validator 调用，profile validation 必须 FAIL，并下沉 `recommended_suite_entry_gate_status=FAIL`。

v4.75 actual bag field acceptance gap validator 补充：`actual_bag_test_suite.sh` 现在生成 `commands/validate_field_acceptance_gap.sh`，独立复验 `reports/field_acceptance_gap_report.txt` 必须保持 `field_acceptance_gap_audit_status=FAIL`、`actual_bag_initial_evidence_status=PASS`、`field_acceptance_ready=NO`、`field_acceptance_eligible=NO`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`，并要求 PLC feedback、section export、PPS/PTP wiring、power-loss resume、runtime deployment、24 h runtime stability 和最终 field acceptance report 全部仍为缺失证据。当前真实 `Tunnel.bag` 新证据 `reports/actual_bag_test_suite_tunnel_v475_gap_validation_rate10` 已通过：suite summary `actual_bag_test_suite_status=PASS`，suite manifest validation `actual_bag_test_suite_manifest_validation_status=PASS`，gap audit 预期退出 1 后输出 `actual_bag_initial_evidence_status=PASS` 与 `field_acceptance_ready=NO`，新增 `field_acceptance_gap_validation_status=PASS`。新增篡改测试证明若把 gap report 改成 `field_acceptance_ready=YES`，gap validation 必须 FAIL。

v4.76 actual bag verified execute 补充：`actual_bag_test_suite.sh --execute` 现在不再只运行 `run_suite.sh`，而是通过 `commands/run_verified_suite.sh` 顺序执行 smoke/full replay suite、`validate_actual_bag_test_suite.sh`、预期退出 1 的 `audit_field_acceptance_gap.sh` 和 `validate_field_acceptance_gap.sh`，并在 suite summary 中追加 `verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_gap_audit_exit=1` 与 `field_acceptance_gap_validation_after_execute=PASS`。当前真实 `Tunnel.bag` 新证据 `reports/actual_bag_test_suite_tunnel_v476_verified_execute_rate10` 已通过：`actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、manifest validation PASS、gap validation PASS，且最终状态仍保持 `field_acceptance_ready=NO` 和 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.77 actual bag verified 字段 manifest 严格复验补充：`validate_actual_bag_test_suite.sh` 现在对 suite summary 中已经出现的 `verified_suite_status`、`suite_manifest_validation_after_execute`、`field_acceptance_gap_audit_exit` 和 `field_acceptance_gap_validation_after_execute` 执行严格期望值校验；未执行 wrapper 的 dry-run/synthetic 证据可保持 `NOT_PRESENT_NA`，但一旦字段出现即不得为 FAIL、错误退出码或其它值。新增篡改测试证明把 `verified_suite_status=FAIL` 追加到 synthetic executed suite 后，manifest validation 必须 FAIL 并输出 `suite_summary_verified_suite_status_status=FAIL`。

v4.78 actual bag verified wrapper 终态 manifest 闭环补充：`commands/run_verified_suite.sh` 现在在追加 verified suite 四个字段后会再次执行 `validate_actual_bag_test_suite.sh`，使最终 `actual_bag_test_suite_manifest_validation.txt` 自动包含 verified 字段 PASS，而不是依赖事后手动重跑。当前真实 `Tunnel.bag` 新证据 `reports/actual_bag_test_suite_tunnel_v478_verified_final_validation_rate10` 已通过：suite summary `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_gap_audit_exit=1`、`field_acceptance_gap_validation_after_execute=PASS`；最终 manifest validation 自动输出 `suite_summary_verified_suite_status_status=PASS`、`suite_summary_suite_manifest_validation_after_execute_status=PASS`、`suite_summary_field_acceptance_gap_audit_exit_status=PASS` 和 `suite_summary_field_acceptance_gap_validation_after_execute_status=PASS`；gap validation 仍为 PASS，gap report 继续声明 `field_acceptance_ready=NO` 和 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.79 actual bag profile 推荐入口 verified execute 要求补充：`actual_bag_profile.sh` 现在在 profile 报告中显式写出 `recommended_suite_verified_execute_required=YES`，`commands/validate_actual_bag_profile.sh` 必须校验该字段为 YES，防止 profile 层只声明一个普通 suite 入口而没有绑定 verified execute 终态闭环。新增篡改测试证明把该字段改为 NO 后 profile validation 必须 FAIL，并输出 `recommended_suite_verified_execute_required_status=FAIL`。当前真实 `Tunnel.bag` 新证据 `reports/actual_bag_profile_tunnel_v479_verified_execute_requirement` 已通过：profile validation 输出 `actual_bag_profile_validation_status=PASS`、`recommended_suite_verified_execute_required_status=PASS`、`recommended_suite_entry_gate_status=PASS`；同目录 `commands/run_recommended_actual_bag_test_suite.sh` 端到端执行通过，生成的 `recommended_suite` summary 为 `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、`field_acceptance_gap_audit_exit=1`，最终 manifest validation 自动复验 verified 四字段 PASS，gap validation PASS，且最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.80 actual bag profile 推荐入口脚本 `--execute` 反查补充：`commands/validate_actual_bag_profile.sh` 现在不仅校验 profile 报告中的 `recommended_suite_command` 包含 `--execute`，还会直接反查 `commands/run_recommended_actual_bag_test_suite.sh` 本体必须包含 `--execute`，并继续要求 strict mode、profile validator 调用早于 suite 调用。新增篡改测试证明只从入口脚本中删除 `--execute`、但不改 profile 报告时，profile validation 也必须 FAIL 并输出 `recommended_suite_entry_gate_status=FAIL`。当前真实 `Tunnel.bag` 新证据 `reports/actual_bag_profile_tunnel_v480_entry_execute_gate` 已通过：profile validation 输出 `actual_bag_profile_validation_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_verified_execute_required_status=PASS`；真实目录篡改入口脚本删除 `--execute` 后 validator 输出 `actual_bag_profile_validation_status=FAIL` 和 `recommended_suite_entry_gate_status=FAIL`，恢复入口脚本后 validation 再次 PASS。

v4.81 actual bag profile 推荐入口命令一致性补充：`commands/validate_actual_bag_profile.sh` 现在要求 `commands/run_recommended_actual_bag_test_suite.sh` 本体必须包含 profile 报告中的 `recommended_suite_command` 原文，防止入口脚本被改成不同 replay rate、不同输出目录、不同 topic 或其它命令后仍因包含 `actual_bag_test_suite.sh --execute` 而误通过。新增篡改测试证明只把入口脚本中的 `--rate 1.0` 改成 `--rate 0.5`、但不改 profile 报告时，profile validation 必须 FAIL 并输出 `recommended_suite_entry_gate_status=FAIL`。当前真实 `Tunnel.bag` 新证据 `reports/actual_bag_profile_tunnel_v481_entry_command_match` 已通过：profile validation 输出 `actual_bag_profile_validation_status=PASS`、`recommended_suite_command_status=PASS`、`recommended_suite_entry_gate_status=PASS`；真实目录篡改入口脚本 rate 后 validator 输出 `actual_bag_profile_validation_status=FAIL` 和 `recommended_suite_entry_gate_status=FAIL`，恢复入口脚本后 validation 再次 PASS。

v4.82 actual bag profile 起步速度审计参数保护补充：`commands/validate_actual_bag_profile.sh` 现在在 `initial_velocity_reference_topic` 存在且不是 `missing` 时，要求 `recommended_suite_command` 必须包含同一 topic 的 `--initial-velocity-topic` 参数，防止 profile 已捕获非零初始速度参考但推荐入口丢失起步速度审计。新增篡改测试证明同时从 profile 报告的 `recommended_suite_command` 和入口脚本中删除 `--initial-velocity-topic`、但保留 `initial_velocity_reference_topic` 时，profile validation 必须 FAIL 并输出 `recommended_suite_command_status=FAIL`。当前真实 `Tunnel.bag` 新证据 `reports/actual_bag_profile_tunnel_v482_initial_velocity_audit_arg` 已通过：profile validation 输出 `actual_bag_profile_validation_status=PASS`、`recommended_suite_command_status=PASS`、`recommended_suite_entry_gate_status=PASS`，profile 捕获 `initial_velocity_reference_topic=/novatel_data/inspvax` 和 `initial_velocity_speed_mps=4.417557`；真实目录双文件篡改删除 `--initial-velocity-topic /novatel_data/inspvax` 后 validator 输出 `actual_bag_profile_validation_status=FAIL` 和 `recommended_suite_command_status=FAIL`，恢复后 validation 再次 PASS。

v4.83 actual bag suite 起步速度捕获 manifest gate 补充：`validate_actual_bag_test_suite.sh` 现在要求 smoke/full 两段 `reports/initial_velocity_reference.txt` 均输出 `initial_velocity_reference_status=CAPTURED`，不再只检查 `START_ONLY_AUDIT` 策略和速度参考未进入 SLAM。新增篡改测试证明把 smoke 初始速度状态改为 `MISSING` 后 suite manifest validation 必须 FAIL，并输出 `smoke_initial_velocity_initial_velocity_reference_status_status=FAIL`。当前真实 `Tunnel.bag` 新证据 `reports/actual_bag_test_suite_tunnel_v483_initial_velocity_capture_manifest_rate10` 已通过：suite summary `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`；manifest validation 输出 `actual_bag_test_suite_manifest_validation_status=PASS`、`smoke_initial_velocity_initial_velocity_reference_status_status=PASS` 和 `full_initial_velocity_initial_velocity_reference_status_status=PASS`；smoke/full 初始速度文件均捕获 `/novatel_data/inspvax` 首样本速度 `4.417557 m/s`，且继续声明 `velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`。真实目录篡改 smoke 状态为 `MISSING` 后 validation FAIL，恢复后 validation 再次 PASS。

v4.84 actual bag 无速度参考初测入口补充：`actual_bag_profile.sh`、`actual_bag_replay.sh` 和 `actual_bag_test_suite.sh` 现在显式支持 `--no-initial-velocity-reference`。当用户 LiDAR+IMU-only 初测 bag 没有任何速度参考 topic 时，profile 推荐命令必须使用该参数，replay/suite 的 inspection、plan、summary、initial velocity report 和 manifest 必须写出 `initial_velocity_reference_required=NO`、`initial_velocity_reference_status=NOT_PRESENT_INITIAL_TEST`、`initial_velocity_reference_topic=NONE` 和 `initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST`，且不得再检查或播放默认 `/novatel_data/inspvax`。suite manifest validator 现在按 `initial_velocity_reference_required` 分支：`YES` 时 smoke/full 仍必须 `CAPTURED`，`NO` 时 smoke/full 必须证明速度参考明确不存在且仍有 `velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`。新增执行态回归测试覆盖 no-velocity 模式下生成的 `run_suite.sh` 必须在 `set -u` 下写出这些字段。当前真实 `Tunnel.bag` 仍属于有速度参考路径，新证据 `reports/actual_bag_test_suite_tunnel_v484_optional_initial_velocity_guard_rate10_fixed` 已通过：suite summary `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、manifest validation `actual_bag_test_suite_manifest_validation_status=PASS`；smoke/full 均捕获 `/novatel_data/inspvax` 首样本速度 `4.417557 m/s`，但 rosbag play topics 只有三路雷达、IMU 和 `/time_reference`，速度参考继续不进入 SLAM。gap audit 仍输出 `field_acceptance_ready=NO` 和 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.85 actual bag field acceptance gap 采集命令补充：`actual_bag_test_suite.sh` 生成的 `field_acceptance_gap_report.txt` 现在不仅固定列出缺失证据，还会为 PLC feedback、section export、PPS/PTP wiring、power-loss resume、runtime deployment、24 h runtime stability 和最终 field acceptance 分别写出建议采集命令；`validate_field_acceptance_gap.sh` 必须逐项复验这些命令字段，缺失、篡改、重复 key 或畸形行都会 FAIL。当前真实 `Tunnel.bag` 短套件证据 `reports/actual_bag_test_suite_tunnel_v485_gap_collection_commands_8s` 已通过：8 s smoke + 8 s full、1.0x、`actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、manifest validation PASS、gap validation PASS；gap report 仍保持 `field_acceptance_ready=NO` 和 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`，但已明确下一步证据采集入口。

v4.86 actual bag 初测就绪证据入口补充：`actual_bag_test_suite.sh` 现在生成 `commands/audit_actual_bag_initial_test_readiness.sh`、`commands/validate_actual_bag_initial_test_readiness.sh` 和 `reports/actual_bag_initial_test_readiness.txt`，把 suite summary、manifest validation 和 field acceptance gap validation 收口成单一初测就绪报告。该报告只有在 smoke/full replay、事件校验、ROS 残留检查、manifest validation、gap validation、初始速度隔离和 LiDAR+IMU-only 边界全部通过时，才输出 `actual_bag_initial_test_readiness_status=PASS` 与 `actual_bag_user_bag_test_ready=YES`；同时必须继续输出 `field_acceptance_eligible=NO` 和 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。当前真实 `Tunnel.bag` 短套件证据 `reports/actual_bag_test_suite_tunnel_v486_initial_readiness_8s` 已通过：8 s smoke + 8 s full、1.0x、`actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、manifest validation PASS、gap validation PASS、readiness validation PASS，并明确当前 bag 可作为用户同类 LiDAR+IMU-only bag 的初测入口证据。

v4.87 actual bag profile 推荐入口补充：`actual_bag_profile.sh` 现在在 `reports/actual_bag_profile.txt` 中写出 `recommended_suite_initial_readiness_required=YES`，`commands/validate_actual_bag_profile.sh` 严格复验该字段，且 `commands/run_recommended_actual_bag_test_suite.sh` 在运行推荐的 `actual_bag_test_suite.sh --execute` 后必须调用推荐 suite 输出目录下的 `commands/validate_actual_bag_initial_test_readiness.sh`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v487_initial_readiness_entry` 已通过 profile validation，且其推荐入口已端到端跑完 60 s smoke + 195 s full、1.0x、verified suite、manifest validation、gap validation 和 readiness validation；最终 readiness 为 `actual_bag_initial_test_readiness_status=PASS`、`actual_bag_user_bag_test_ready=YES`，同时仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.88 field acceptance handoff 补充：`actual_bag_test_suite.sh` 现在生成 `commands/generate_field_acceptance_handoff.sh`、`commands/validate_field_acceptance_handoff.sh` 和 `reports/field_acceptance_handoff.txt`，在 readiness validation 与 gap validation 均 PASS 后，把 PLC feedback、section export、PPS/PTP wiring、power-loss resume、runtime deployment、24 h runtime stability 和最终 field acceptance 的采集命令收口为单一现场证据交接报告。该报告允许输出 `field_acceptance_handoff_status=PASS` 与 `field_acceptance_handoff_ready=YES`，但必须同时保持 `field_acceptance_ready=NO`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` 和完整 `required_next_evidence`，防止 handoff PASS 被误读为最终验收 PASS。当前真实短套件证据 `reports/actual_bag_test_suite_tunnel_v488_field_acceptance_handoff_8s` 已通过 8 s smoke + 8 s full、verified suite、manifest validation、gap validation、readiness validation 和 handoff validation。

v4.89 actual bag profile 推荐入口 handoff 闭环补充：`actual_bag_profile.sh` 现在在 `reports/actual_bag_profile.txt` 中写出 `recommended_suite_field_acceptance_handoff_required=YES`，`commands/validate_actual_bag_profile.sh` 必须校验该字段，并反查 `commands/run_recommended_actual_bag_test_suite.sh` 在推荐的 `actual_bag_test_suite.sh --execute`、`validate_actual_bag_initial_test_readiness.sh` 之后继续调用推荐 suite 输出目录下的 `commands/validate_field_acceptance_handoff.sh`。新增篡改测试覆盖缺失/篡改 handoff 必跑字段和推荐入口删除 handoff validator 均必须使 profile validation FAIL。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v489_handoff_entry` 已通过：profile validation 输出 `recommended_suite_field_acceptance_handoff_required_status=PASS` 与 `recommended_suite_field_acceptance_handoff_entry_status=PASS`；推荐入口端到端跑完 60 s smoke + 195 s full、1.0x verified suite、manifest validation、gap validation、readiness validation 和 handoff validation，最终 `field_acceptance_handoff_status=PASS`、`field_acceptance_handoff_ready=YES`，但仍保持 `field_acceptance_ready=NO` 与 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.90 field acceptance handoff bundle manifest 补充：`actual_bag_test_suite.sh` 现在生成 `reports/field_acceptance_handoff_manifest.txt` 和 `commands/validate_field_acceptance_handoff_manifest.sh`，把 suite summary、suite manifest validation、field acceptance gap、gap validation、actual bag readiness、readiness validation、handoff 报告和 handoff validation 收口为单一 handoff bundle manifest。validator 必须复验上述文件存在、关键 validator 命令可执行、suite/gap/readiness/handoff 各级状态均 PASS，同时继续要求 `field_acceptance_eligible=NO`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` 和完整 `required_next_evidence`；若 handoff validation 报告被改成 FAIL 或缺失，handoff manifest validation 必须 FAIL。当前真实短套件证据 `reports/actual_bag_test_suite_tunnel_v490_handoff_manifest_8s` 已通过 8 s smoke + 8 s full、verified suite、manifest validation、gap validation、readiness validation、handoff validation 和 handoff manifest validation；summary 输出 `field_acceptance_handoff_manifest_after_execute=PASS`，最终仍不是现场验收 PASS。

v4.91 actual bag profile 推荐入口 handoff bundle manifest 闭环补充：`actual_bag_profile.sh` 现在在 `reports/actual_bag_profile.txt` 中写出 `recommended_suite_field_acceptance_handoff_manifest_required=YES`，`commands/validate_actual_bag_profile.sh` 必须校验该字段，并反查 `commands/run_recommended_actual_bag_test_suite.sh` 在推荐的 `actual_bag_test_suite.sh --execute`、`validate_actual_bag_initial_test_readiness.sh`、`validate_field_acceptance_handoff.sh` 之后继续调用推荐 suite 输出目录下的 `commands/validate_field_acceptance_handoff_manifest.sh`。新增篡改测试覆盖缺失/篡改 handoff manifest 必跑字段和推荐入口删除 handoff manifest validator 均必须使 profile validation FAIL。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v491_handoff_manifest_entry` 已通过：profile validation 输出 `recommended_suite_field_acceptance_handoff_manifest_required_status=PASS` 与 `recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`；推荐入口端到端跑完 60 s smoke + 195 s full、1.0x verified suite、manifest validation、gap validation、readiness validation、handoff validation 和 handoff manifest validation，summary 输出 `field_acceptance_handoff_manifest_after_execute=PASS`，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.92 actual bag 现场采集计划入口补充：`actual_bag_test_suite.sh` 现在生成 `reports/field_acceptance_collection_plan.txt`、`commands/generate_field_acceptance_collection_plan.sh` 和 `commands/validate_field_acceptance_collection_plan.sh`，在 handoff 与 handoff bundle manifest 均 PASS 后，把 PLC feedback、section export、PPS/PTP wiring、power-loss resume、runtime deployment、24 h runtime stability 和 final field acceptance 的 7 步采集顺序、采集命令、最终 `validate_evidence.sh => field_acceptance_status=PASS` 成功门槛写成独立可审计计划。validator 必须拒绝缺失、重复 key、畸形行、采集命令篡改、最终成功门槛篡改、引用 handoff/handoff manifest 状态不一致或把 LiDAR+IMU-only 初测误升级成最终 PASS；`run_verified_suite.sh` 会在 handoff manifest 后生成并复验该计划，summary 输出 `field_acceptance_collection_plan_after_execute=PASS`。`actual_bag_profile.sh` 同步写出 `recommended_suite_field_acceptance_collection_plan_required=YES`，profile validator 反查推荐入口在 handoff manifest validator 后继续调用 `validate_field_acceptance_collection_plan.sh`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v492_collection_plan_entry` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、readiness、handoff、handoff manifest 和 collection plan validation；最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.93 actual bag suite manifest 对 collection plan 的二次复验补充：`actual_bag_test_suite_manifest.txt` 现在显式声明 `field_acceptance_collection_plan_validation=reports/field_acceptance_collection_plan_validation.txt`，`validate_actual_bag_test_suite.sh` 在发现 `field_acceptance_collection_plan_after_execute=PASS` 或 collection plan validation 报告已经生成时，必须复验 collection plan 文件存在、collection plan validation 文件存在、计划本体 `field_acceptance_collection_plan_status=PASS`、`collection_plan_ready=YES`、`final_success_gate=record_session.sh generated commands/validate_evidence.sh => field_acceptance_status=PASS`，以及 validation 报告 `field_acceptance_collection_plan_validation_status=PASS` 和 `final_success_gate_status=PASS`。新增篡改测试覆盖 collection plan 的最终成功门槛被改成 `missing` 后，单独 `validate_field_acceptance_collection_plan.sh` 和上层 `validate_actual_bag_test_suite.sh` 均必须 FAIL，防止只重跑 suite manifest validator 时漏掉现场采集计划被污染。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v493_collection_plan_manifest` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite 和 collection plan manifest 复验；`actual_bag_test_suite_manifest_validation.txt` 下沉 `field_acceptance_collection_plan_validation_field_acceptance_collection_plan_validation_status_status=PASS` 与 `field_acceptance_collection_plan_validation_final_success_gate_status_status=PASS`，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.94 actual bag profile 推荐入口后置 manifest 复验补充：`actual_bag_profile.sh` 现在在 `reports/actual_bag_profile.txt` 中写出 `recommended_suite_collection_plan_manifest_revalidation_required=YES`，`commands/run_recommended_actual_bag_test_suite.sh` 必须在推荐 suite 的 `validate_field_acceptance_collection_plan.sh` 之后再次调用 `validate_actual_bag_test_suite.sh`，使从 profile 启动的真实 bag 初测入口在 collection plan validator 通过后仍重新执行 suite manifest 对 collection plan 的上层复验。`commands/validate_actual_bag_profile.sh` 必须同时校验该 required 字段和后置调用顺序；新增篡改测试覆盖删除该后置 suite manifest validator 时 profile validation 必须 FAIL。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v494_collection_plan_manifest_entry` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；profile validation 下沉 `recommended_suite_collection_plan_manifest_revalidation_required_status=PASS` 与 `recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`，推荐入口脚本第 10 行调用 collection plan validator、第 11 行调用 suite manifest validator，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.95 actual bag profile 推荐入口 `suite_out` 绑定补充：`commands/validate_actual_bag_profile.sh` 现在必须反查 `commands/run_recommended_actual_bag_test_suite.sh` 中的 `suite_out=` 变量与 `reports/actual_bag_profile.txt` 的 `recommended_suite_out` 完全一致，防止入口脚本保留正确 `actual_bag_test_suite.sh --out ...` 命令却把后续 readiness/handoff/collection plan/suite manifest validators 指向其它目录。新增篡改测试覆盖把入口脚本 `suite_out` 改到 `other_suite` 时，profile validation 必须输出 `recommended_suite_entry_gate_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v495_suite_out_binding` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；profile 报告 `recommended_suite_out` 与入口脚本第 4 行 `suite_out` 均指向同一个 `recommended_suite` 目录，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.96 actual bag profile 推荐输出目录锚定补充：`commands/validate_actual_bag_profile.sh` 现在不仅要求入口脚本 `suite_out` 与 profile 报告 `recommended_suite_out` 一致，还要求 `recommended_suite_out` 精确等于当前 profile 输出根目录下的 `recommended_suite`。这防止 profile 报告和入口脚本被同步篡改到外部 `other_suite` 后仍通过自洽检查。新增篡改测试覆盖同时改写 `reports/actual_bag_profile.txt` 和 `commands/run_recommended_actual_bag_test_suite.sh` 中的推荐输出目录时，profile validation 必须输出 `recommended_suite_out_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v496_suite_out_anchor` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；profile validation 下沉 `recommended_suite_out_status=PASS`，推荐入口脚本第 4 行 `suite_out` 与 profile 根目录下的 `recommended_suite` 锚点一致，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.97 actual bag suite manifest 路径锚定补充：`commands/validate_actual_bag_test_suite.sh` 现在要求 `actual_bag_test_suite_manifest.txt` 中 summary、metrics、event、smoke/full replay summary、smoke/full HIL validation、smoke/full initial velocity、smoke/full inspection、ROS residual、gap/readiness/handoff/collection plan 等报告路径必须精确等于 suite 生成时的固定相对路径。绝对路径、`../`、兄弟 suite 目录或其它 session 目录即使内容自洽也必须由对应 `*_path_status=FAIL` 拒绝，防止 suite manifest 拼接外部 PASS 报告。新增篡改测试覆盖把 `summary=reports/actual_bag_test_suite_summary.txt` 改为 suite 根目录外部绝对路径时，manifest validation 必须输出 `summary_path_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v497_suite_manifest_path_anchor` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；suite manifest validation 下沉 `summary_path_status=PASS`、`metrics_report_path_status=PASS`、`event_file_path_status=PASS`、`smoke_summary_path_status=PASS`、`full_summary_path_status=PASS`、`smoke_initial_velocity_path_status=PASS`、`full_initial_velocity_path_status=PASS`、`field_acceptance_collection_plan_path_status=PASS` 与 `field_acceptance_collection_plan_validation_path_status=PASS`，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.98 actual bag handoff bundle manifest 路径锚定补充：`commands/validate_field_acceptance_handoff_manifest.sh` 现在要求 `field_acceptance_handoff_manifest.txt` 中 suite summary、suite manifest、suite manifest validation、field acceptance gap report/validation、actual bag readiness report/validation、field acceptance handoff report/validation 等报告路径必须精确等于当前 suite 内固定相对路径。绝对路径、`../`、兄弟 suite 目录或其它 session 目录即使内容自洽也必须由对应 `*_path_status=FAIL` 拒绝，防止 handoff bundle manifest 拼接外部 PASS 报告。新增篡改测试覆盖把 `field_acceptance_handoff_validation=reports/field_acceptance_handoff_validation.txt` 改为 suite 根目录外部绝对路径时，handoff manifest validation 必须输出 `field_acceptance_handoff_validation_path_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v498_handoff_manifest_path_anchor` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、handoff manifest validation、collection plan validation 和后置 suite manifest revalidation；handoff manifest validation 下沉 `suite_summary_path_status=PASS`、`suite_manifest_path_status=PASS`、`suite_manifest_validation_path_status=PASS`、`field_acceptance_gap_report_path_status=PASS`、`field_acceptance_gap_validation_path_status=PASS`、`actual_bag_initial_test_readiness_path_status=PASS`、`actual_bag_initial_test_readiness_validation_path_status=PASS`、`field_acceptance_handoff_path_status=PASS` 与 `field_acceptance_handoff_validation_path_status=PASS`，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v4.99 actual bag collection plan source 路径锚定补充：`commands/validate_field_acceptance_collection_plan.sh` 现在不仅检查 `source_field_acceptance_handoff`、`source_field_acceptance_handoff_validation`、`source_field_acceptance_handoff_manifest` 和 `source_field_acceptance_handoff_manifest_validation` 指向的文件存在，还要求这四个 source 字段精确等于当前 suite `reports/` 下对应报告的绝对路径。外部目录、兄弟 suite 或其它 session 中同名 PASS 报告即使存在，也必须由对应 `source_field_acceptance_*_path_status=FAIL` 拒绝，防止 collection plan provenance 拼接外部 handoff 证据。新增篡改测试覆盖把 `source_field_acceptance_handoff_manifest_validation` 改为 suite 根目录外部绝对路径时，collection plan validation 必须输出 `source_field_acceptance_handoff_manifest_validation_path_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v499_collection_plan_source_path_anchor` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、handoff manifest validation、collection plan validation 和后置 suite manifest revalidation；collection plan validation 下沉 `source_field_acceptance_handoff_path_status=PASS`、`source_field_acceptance_handoff_validation_path_status=PASS`、`source_field_acceptance_handoff_manifest_path_status=PASS` 与 `source_field_acceptance_handoff_manifest_validation_path_status=PASS`，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v5.00 actual bag suite manifest 对 collection plan source provenance 的上层复验补充：`commands/validate_actual_bag_test_suite.sh` 现在在 collection plan 已执行或 validation 报告存在时，直接复验 `field_acceptance_collection_plan.txt` 中四个 `source_field_acceptance_*` 字段必须等于当前 suite `reports/` 下对应报告路径，并要求 `field_acceptance_collection_plan_validation.txt` 下沉的四个 `source_field_acceptance_*_path_status` 均为 PASS。新增篡改测试覆盖 collection plan source 被改到外部目录、但 collection plan validation 报告仍是旧 PASS 时，上层 suite manifest validation 必须输出 `field_acceptance_collection_plan_source_field_acceptance_handoff_manifest_validation_path_status=FAIL` 并拒绝 PASS。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v500_suite_manifest_collection_source_revalidation` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；suite manifest validation 下沉 `field_acceptance_collection_plan_source_field_acceptance_handoff_path_status=PASS`、`field_acceptance_collection_plan_source_field_acceptance_handoff_validation_path_status=PASS`、`field_acceptance_collection_plan_source_field_acceptance_handoff_manifest_path_status=PASS` 与 `field_acceptance_collection_plan_source_field_acceptance_handoff_manifest_validation_path_status=PASS`，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v5.01 actual bag profile 推荐入口可执行行 gate 补充：`commands/validate_actual_bag_profile.sh` 现在检查推荐入口脚本时只承认非空、非注释的可执行行；readiness、handoff、handoff manifest、collection plan 和 post-collection suite manifest validator 必须按固定顺序作为真实命令行出现，注释、说明文本或 echo 片段中的同名脚本不再能满足 entry gate。新增篡改测试覆盖删除 post-collection `"$suite_out/commands/validate_actual_bag_test_suite.sh"` 真实调用、只保留同名注释时，profile validation 必须输出 `recommended_suite_collection_plan_manifest_revalidation_entry_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v501_profile_entry_executable_line_gate` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite 和后置 suite manifest revalidation；入口脚本第 5-11 行为真实可执行 validator 顺序，profile validation 下沉 `recommended_suite_entry_gate_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS` 与 `recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v5.02 actual bag profile 推荐 suite 命令精确可执行行 gate 补充：`commands/validate_actual_bag_profile.sh` 现在要求推荐入口脚本中的实际 suite 命令行必须与 `reports/actual_bag_profile.txt` 的 `recommended_suite_command` 完全一致；`echo <recommended_suite_command>`、wrapper、注释、说明文本或只包含子串的命令均不能满足 `recommended_suite_entry_gate`。新增篡改测试覆盖把真实 suite 命令替换为 `echo` 同名命令时，profile validation 必须输出 `recommended_suite_entry_gate_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v502_profile_entry_exact_suite_command` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；入口脚本第 6 行为精确匹配 `recommended_suite_command` 的真实可执行命令，profile validation 下沉 `recommended_suite_command_status=PASS`、`recommended_suite_entry_gate_status=PASS` 与 `recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`，仍证明 `velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`，最终保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v5.03 actual bag profile 推荐入口 `suite_out` 单一赋值 gate 补充：`commands/validate_actual_bag_profile.sh` 现在要求推荐入口脚本中的非注释可执行 `suite_out=` 赋值行只能出现一次，必须精确等于 `reports/actual_bag_profile.txt` 的 `recommended_suite_out`，且必须出现在推荐 suite 命令之前；先写正确 `suite_out`、再重写到外部目录的脚本不能满足 `recommended_suite_entry_gate`。新增篡改测试覆盖在正确 `suite_out` 后插入 `suite_out=<other_suite>` 时，profile validation 必须输出 `recommended_suite_entry_gate_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v503_profile_entry_suite_out_single_assignment` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；入口脚本第 4 行唯一绑定当前 profile 根目录下的 `recommended_suite`，第 6 行为精确推荐 suite 命令，后续 validators 均引用同一 `suite_out`，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v5.04 actual bag profile 后置 gate 精确 suite 命令锚定补充：`commands/validate_actual_bag_profile.sh` 现在要求 readiness、handoff、handoff manifest、collection plan 和 post-collection suite manifest revalidation 的顺序判断全部锚定到 `reports/actual_bag_profile.txt` 中的精确 `recommended_suite_command`，不得把额外插入的 `echo actual_bag_test_suite.sh`、wrapper 或其它包含同名脚本的文本行当作 suite 已执行证据。新增篡改测试覆盖把 readiness validator 移到真实 suite 命令之前、但放在伪 `echo actual_bag_test_suite.sh` 之后时，profile validation 必须输出 `recommended_suite_initial_readiness_entry_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v504_profile_post_gates_exact_suite_anchor` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；profile validation 下沉 `recommended_suite_initial_readiness_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS` 与 `recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`，最终仍保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v5.05 actual bag profile 推荐入口提前终止 gate 补充：`commands/validate_actual_bag_profile.sh` 现在要求推荐入口脚本从 profile validator 到 post-collection suite manifest revalidation 的闭环执行窗口内不得包含 `exit`、`return` 或 `exec` 提前终止命令，防止入口脚本先通过 profile validation 再在真实 suite 或后置 validators 前退出。新增篡改测试覆盖在精确 `recommended_suite_command` 前插入 `exit 0` 时，profile validation 必须输出 `recommended_suite_entry_gate_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v505_profile_entry_no_early_termination` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；profile validation 下沉 `recommended_suite_entry_gate_status=PASS`、`recommended_suite_initial_readiness_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS` 与 `recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`，仍证明起步速度参考 `/novatel_data/inspvax` 只作初始审计、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`，最终保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v5.06 actual bag profile 推荐入口包裹式提前终止 gate 补充：`commands/validate_actual_bag_profile.sh` 现在不仅拒绝以 `exit`、`return` 或 `exec` 开头的提前终止行，还会拒绝闭环窗口内 shell 边界上的 `exit`、`return` 或 `exec` token，防止 `if true; then exit 0; fi` 这类控制结构在真实 suite 或后置 validators 前提前退出。新增篡改测试覆盖在精确 `recommended_suite_command` 前插入 `if true; then exit 0; fi` 时，profile validation 必须输出 `recommended_suite_entry_gate_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v506_profile_entry_wrapped_early_termination_gate` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；profile validation 下沉 `recommended_suite_entry_gate_status=PASS`、`recommended_suite_initial_readiness_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS` 与 `recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`，起步速度参考 `/novatel_data/inspvax` 仍只作初始审计，`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`，最终保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v5.07 actual bag profile 推荐入口 strict mode 持续性 gate 补充：`commands/validate_actual_bag_profile.sh` 现在不仅要求推荐入口脚本包含 `set -euo pipefail`，还要求 profile validator 到 post-collection suite manifest revalidation 的闭环窗口内不得出现 `set +e`、`set +u` 或 `set +o pipefail` 这类 strict mode relaxation，防止 validator 失败后入口脚本仍继续执行并制造证据链外观。新增篡改测试覆盖在 profile validator 后插入 `set +e` 时，profile validation 必须输出 `recommended_suite_entry_gate_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v507_profile_entry_strict_mode_relaxation_gate` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；profile validation 下沉 `recommended_suite_entry_gate_status=PASS`、`recommended_suite_initial_readiness_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS` 与 `recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`，起步速度参考 `/novatel_data/inspvax` 仍只作初始审计，`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`，最终保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v5.08 actual bag profile 推荐入口包裹式 strict mode relaxation gate 补充：`commands/validate_actual_bag_profile.sh` 现在会在闭环窗口内按 shell 分隔符抽取 token，从任意 `set` token 后识别 `+e`、`+u` 或 `+o pipefail`，防止 `if true; then set +e; fi` 这类控制结构关闭 strict mode 后继续执行。新增篡改测试覆盖在 profile validator 后插入 `if true; then set +e; fi` 时，profile validation 必须输出 `recommended_suite_entry_gate_status=FAIL`。当前真实 `Tunnel.bag` 证据 `reports/actual_bag_profile_tunnel_v508_profile_entry_wrapped_strict_mode_relaxation_gate` 已通过 profile validation、60 s smoke + 195 s full 推荐 suite、collection plan validation 和后置 suite manifest revalidation；profile validation 下沉 `recommended_suite_entry_gate_status=PASS`、`recommended_suite_initial_readiness_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS` 与 `recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`，起步速度参考 `/novatel_data/inspvax` 仍只作初始审计，`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`，最终保持 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`。

v5.09 actual bag profile PLC-present profile-only gate 补充：`commands/validate_actual_bag_profile.sh` 不再把 `plc_feedback_topic_count` 固定限制为 0。若用户后续收集的 bag 已包含 `/plc/left_track_speed`、`/plc/right_track_speed`、`/plc/cutting_on` 或 `/machine/state`，profile 允许输出 `plc_feedback_status=PRESENT_PROFILE_ONLY` 与 `plc_feedback_gate_status=PRESENT_PROFILE_ONLY_FIELD_VALIDATION_REQUIRED`，但仍必须保持 `field_acceptance_eligible=NO`，防止“存在 PLC topic”被误当成 PLC 状态机、section export 或最终 field acceptance PASS。新增合成 bag 回归测试覆盖 4 个 PLC topic 同时存在时 profile validation 必须 PASS，且 `velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO` 不变。当前真实 `Tunnel.bag` v509 profile 证据 `reports/actual_bag_profile_tunnel_v509_plc_profile_only_gate` 已通过 profile validation，确认该包仍为 `plc_feedback_topic_count=0`、`plc_feedback_status=NOT_PRESENT_NA`、`plc_feedback_gate_status=NA_INITIAL_TEST`、`field_acceptance_eligible=NO`，起步速度参考仍未连续用于 SLAM。

当前验证口径：

| 验证项 | 结果 |
|---|---|
| 全工作区构建 | `catkin_make -DCATKIN_BLACKLIST_PACKAGES=""` 通过 |
| 全工作区测试 | `catkin_make run_tests -DCATKIN_BLACKLIST_PACKAGES=""` 通过 |
| 本轮 bringup 测试 | `catkin_make run_tests_mine_slam_bringup -DCATKIN_BLACKLIST_PACKAGES=""` 通过；`catkin_test_results build/test_results/mine_slam_bringup`：540 tests，0 errors，0 failures，0 skipped |
| 本轮八态门控回归 | `catkin_make run_tests_machine_state_manager run_tests_mapping_control run_tests_mine_slam_bringup_gtest_record_session_script_test run_tests_lio_eval_tools_gtest_evidence_manifest_test -DCATKIN_BLACKLIST_PACKAGES=""` 通过；`machine_state_manager`：17 tests，`mapping_control`：9 tests，`mine_slam_bringup`：472 tests，`lio_eval_tools`：523 tests，均 0 errors/failures/skipped |
| 测试汇总 | `catkin_test_results build/test_results`：1551 tests，0 errors，0 failures，0 skipped |
| 实际 bag 完整质量模式回放 | `reports/actual_bag_replay_tunnel_full_v456_rate03_quality_gate_motion_prior/reports/actual_bag_replay_summary.txt`：`actual_bag_replay_status=PASS`，195 s，0.3x，`ros_master_cleanup_status=PASS`，`fusion_callbacks=1906`，`fusion_published=1906`，`minimum_fusion_published=975`，`fusion_duration_coverage_status=PASS`，`lio_local_odometry_diag_captured=PASS`，`local_odometry_clouds=885`，`local_odometry_published=787`，`minimum_local_odometry_published=195`，`local_odometry_duration_coverage_status=PASS`，`local_odometry_rejected_registrations=98`，`points_raw_width=93551`，`deskewed_cloud_width=83870`，`pipeline_error_status=PASS`，`velocity_reference_played_to_slam=NO`，`continuous_velocity_reference_used=NO`；`initial_velocity_reference.txt` 记录首个 NovAtel 速度样本 `4.417557 m/s` 且策略为 `START_ONLY_AUDIT`，回放后 `pgrep` 确认无 ROS 残留进程 |
| 实际 bag LiDAR+IMU-only 初测回放 | `reports/actual_bag_replay_tunnel_v460_reseed_60s_rate03/reports/actual_bag_replay_summary.txt`：`actual_bag_replay_status=PASS`，60 s，0.3x，`actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY`，`plc_feedback_status=NOT_PRESENT_NA`，`plc_feedback_gate_status=NA_INITIAL_TEST`，`machine_motion_assumption=CONTINUOUS_MOTION`，`field_acceptance_requires_plc_feedback=YES`，`ros_master_cleanup_status=PASS`，`fusion_published=576 >= 300`，`local_odometry_published=284 >= 60`，`local_odometry_rejected_registrations=14`，`local_odometry_keyframe_reseeds=7`，`local_odometry_duration_coverage_status=PASS`，`pipeline_error_status=PASS`，`velocity_reference_played_to_slam=NO`，`continuous_velocity_reference_used=NO`；`actual_bag_replay_metrics_report.txt` 和 `actual_bag_replay_events.txt` 声明 `validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY` 与 `field_acceptance_eligible=NO`，`commands/validate_actual_bag_events.sh` 生成 `actual_bag_replay_hil_validation_report.txt`：`overall=PASS`；inspection 证明 `/plc/left_track_speed`、`/plc/right_track_speed`、`/plc/cutting_on` 和 `/machine/state` 计数均为 0，启动命令显式关闭 `start_machine_state/start_mapping_control/start_section_manager` |
| 实际 bag 实时回放状态 | `reports/actual_bag_replay_tunnel_v460_reseed_60s_rate10/reports/actual_bag_replay_summary.txt`：60 s，1.0x，`actual_bag_replay_status=PASS`，`fusion_published=398 >= 300`，`local_odometry_published=106 >= 60`，`local_odometry_rejected_registrations=25`，`local_odometry_keyframe_reseeds=9`；`reports/actual_bag_replay_tunnel_v460_reseed_full195s_rate10/reports/actual_bag_replay_summary.txt`：195 s，1.0x，`actual_bag_replay_status=PASS`，`fusion_published=1178 >= 975`，`local_odometry_published=210 >= 195`，`local_odometry_rejected_registrations=23`，`local_odometry_keyframe_reseeds=10`，generated event validation `overall=PASS`；旧 `reports/actual_bag_replay_tunnel_60s_v456_1x_overlap_sampled` 仅作为历史性能诊断，不再代表当前状态 |
| 实际 bag 一键初测套件 | `reports/actual_bag_test_suite_tunnel_v462_rate10/reports/actual_bag_test_suite_summary.txt`：`actual_bag_test_suite_status=PASS`，60 s smoke + 195 s full，1.0x，`smoke_replay_status=PASS`，`full_replay_status=PASS`，`smoke_event_validation_status=PASS`，`full_event_validation_status=PASS`，`ros_residual_status=PASS`，`actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY`，`plc_feedback_status=NOT_PRESENT_NA`，`velocity_reference_played_to_slam=NO`，`continuous_velocity_reference_used=NO`，`field_acceptance_eligible=NO`；full replay `fusion_published=1341 >= 975`，`local_odometry_published=256 >= 195`，`local_odometry_rejected_registrations=36`，`local_odometry_keyframe_reseeds=13`；inspection 记录 PLC topic 计数均为 0，初始速度参考首样本 `4.417557 m/s` 仅作起步审计 |
| 实际 bag suite manifest 校验 | `reports/actual_bag_test_suite_tunnel_v463_manifest_rate10/reports/actual_bag_test_suite_manifest_validation.txt`：`actual_bag_test_suite_manifest_validation_status=PASS`，逐项复验 suite summary、metrics/event、smoke/full replay summary、event validation、初始速度审计、bag inspection 和 ROS 残留报告；同目录 summary 为 `actual_bag_test_suite_status=PASS`，full replay `fusion_published=1312 >= 975`、`local_odometry_published=254 >= 195`、`local_odometry_rejected_registrations=39`、`local_odometry_keyframe_reseeds=14`，`field_acceptance_eligible=NO` |
| 实际 bag field acceptance gap audit | `reports/actual_bag_test_suite_tunnel_v464_gap_rate10/reports/field_acceptance_gap_report.txt`：`field_acceptance_gap_audit_status=FAIL`，`actual_bag_initial_evidence_status=PASS`，`field_acceptance_ready=NO`，`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`；缺口清单为 PLC feedback、section export、PPS/PTP wiring、power-loss resume、runtime deployment、24 h runtime stability 和最终 field acceptance report。该 FAIL 是预期行为，表示当前 bag 已能做初步 LiDAR+IMU 实测开发，但不能宣称最终现场验收通过 |
| 实际 bag topic 映射 smoke | `reports/actual_bag_replay_tunnel_v465_topic_default_smoke_8s/reports/actual_bag_replay_summary.txt`：`actual_bag_replay_status=PASS`，8 s，1.0x，`center_lidar_topic/canonical_center_lidar_topic/play_topics/excluded_velocity_reference_topic/initial_velocity_reference_topic` 已落盘，`fusion_published=78 >= 40`，`local_odometry_published=28 >= 8`，`velocity_reference_played_to_slam=NO`，`continuous_velocity_reference_used=NO`；`reports/initial_velocity_reference.txt` 记录首样本 `4.417557 m/s` 且策略为 `START_ONLY_AUDIT`，`actual_bag_replay_hil_validation_report.txt` 输出 `overall=PASS` |
| 实际 bag 无 time reference smoke | `reports/actual_bag_replay_tunnel_v466_no_time_reference_smoke_8s/reports/actual_bag_replay_summary.txt`：`actual_bag_replay_status=PASS`，8 s，1.0x，`time_reference_topic=NONE`，`time_reference_status=NOT_PRESENT_INITIAL_TEST`，`time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST`，`play_topics=/velodyne_points,/left/lslidar_point_cloud,/right/velodyne_points,/imu/data`，`time_status_captured=PASS`，`fusion_published=78 >= 40`，`local_odometry_published=29 >= 8`，`velocity_reference_played_to_slam=NO`，`continuous_velocity_reference_used=NO`；event validation `overall=PASS` |
| 实际 bag 无 time reference suite manifest | `reports/actual_bag_test_suite_tunnel_v467_no_time_reference_manifest_8s/reports/actual_bag_test_suite_manifest_validation.txt`：`actual_bag_test_suite_manifest_validation_status=PASS`，`time_reference_status=NOT_PRESENT_INITIAL_TEST`，`time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST`，smoke/full replay summary 与 inspection 均通过 time reference mode 一致性复验；suite summary 为 `actual_bag_test_suite_status=PASS`，smoke/full replay/event validation 均 PASS，gap audit 预期输出 `field_acceptance_gap_audit_status=FAIL`、`actual_bag_initial_evidence_status=PASS`、`field_acceptance_ready=NO` |
| 实际 bag intake/profile | `reports/actual_bag_profile_tunnel_v468/reports/actual_bag_profile.txt`：`actual_bag_profile_status=PASS`，`bag_duration_s=194.997160`，自动识别 `/velodyne_points`、`/left/lslidar_point_cloud`、`/right/velodyne_points`、`/imu/data`、`/time_reference` 和 `/novatel_data/inspvax`；生成 `commands/run_recommended_actual_bag_test_suite.sh`，推荐 topic override、`--time-reference-topic /time_reference`、`--initial-velocity-topic /novatel_data/inspvax`、`--rate 1.0 --execute`，同时声明 `field_acceptance_eligible=NO`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO` |
| profile 生成命令端到端 suite | `reports/actual_bag_profile_tunnel_v468/recommended_suite/reports/actual_bag_test_suite_summary.txt`：`actual_bag_test_suite_status=PASS`，smoke/full replay/event validation 均 PASS，`ros_residual_status=PASS`，`time_reference_status=PRESENT_REQUIRED`，`field_acceptance_eligible=NO`，`velocity_reference_played_to_slam=NO`，`continuous_velocity_reference_used=NO`；full replay `fusion_published=1311 >= 975`、`local_odometry_published=257 >= 195`、`local_odometry_rejected_registrations=37`、`local_odometry_keyframe_reseeds=15`；`commands/validate_actual_bag_test_suite.sh` 输出 `actual_bag_test_suite_manifest_validation_status=PASS`；gap audit 预期退出 1，报告 `actual_bag_initial_evidence_status=PASS`、`field_acceptance_ready=NO` |
| profile 初测边界字段 | `reports/actual_bag_profile_tunnel_v470_motion_semantics/reports/actual_bag_profile.txt`：`actual_bag_profile_status=PASS`，`plc_feedback_topic_count=0`，`plc_feedback_status=NOT_PRESENT_NA`，`plc_feedback_gate_status=NA_INITIAL_TEST`，`machine_motion_assumption=CONTINUOUS_MOTION`，`vibration_profile=NORMAL`，`field_acceptance_requires_plc_feedback=YES`，并继续声明 `field_acceptance_eligible=NO`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO` |
| profile 起步速度审计 | `reports/actual_bag_profile_tunnel_v471_initial_velocity_sample/reports/actual_bag_profile.txt`：`initial_velocity_reference_status=CAPTURED`，`initial_velocity_reference_policy=START_ONLY_AUDIT`，`initial_velocity_reference_topic=/novatel_data/inspvax`，首样本 `north/east/up/speed=4.416000/0.117000/-0.008000/4.417557 m/s`；仍声明 `velocity_reference_played_to_slam=NO` 和 `continuous_velocity_reference_used=NO` |
| profile 自检 gate | `reports/actual_bag_profile_tunnel_v472_profile_validation/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`，`duplicate_key_count=0`，`malformed_line_count=0`，`actual_bag_profile_status_status=PASS`，`recommended_suite_command_status=PASS`，`initial_velocity_reference_policy_status=PASS`，`velocity_reference_played_to_slam_status=PASS`，`continuous_velocity_reference_used_status=PASS`，并下沉 `field_acceptance_eligible=NO`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO` |
| profile-gated 推荐 suite 入口 | `reports/actual_bag_profile_tunnel_v473_recommended_gate/commands/run_recommended_actual_bag_test_suite.sh`：入口脚本先执行 `"$script_dir/validate_actual_bag_profile.sh"`，再执行推荐的 `actual_bag_test_suite.sh --execute`；同目录 `reports/actual_bag_profile_validation.txt` 输出 `actual_bag_profile_validation_status=PASS`、`recommended_suite_command_status=PASS`、`field_acceptance_eligible=NO`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO` |
| profile validator 反查推荐入口 gate | `reports/actual_bag_profile_tunnel_v474_validator_entry_gate/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`，新增 `recommended_suite_entry_gate_status=PASS`，并继续输出 `recommended_suite_command_status=PASS`、`field_acceptance_eligible=NO`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录推荐入口脚本仍按 validator 在前、suite 在后的顺序生成 |
| 实际 bag field acceptance gap validation | `reports/actual_bag_test_suite_tunnel_v475_gap_validation_rate10/reports/field_acceptance_gap_validation.txt`：`field_acceptance_gap_validation_status=PASS`，`field_acceptance_ready_status=PASS`，`field_acceptance_status_status=PASS`，`required_next_evidence_status=PASS`，并复验 PLC feedback、section export、PPS/PTP wiring、power-loss resume、runtime deployment、24 h runtime stability 和 final field acceptance report 均保持缺失；同目录 suite summary `actual_bag_test_suite_status=PASS`，manifest validation `actual_bag_test_suite_manifest_validation_status=PASS` |
| 实际 bag verified execute | `reports/actual_bag_test_suite_tunnel_v476_verified_execute_rate10/reports/actual_bag_test_suite_summary.txt`：`actual_bag_test_suite_status=PASS`，新增 `verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_gap_audit_exit=1`、`field_acceptance_gap_validation_after_execute=PASS`；同目录 `actual_bag_test_suite_manifest_validation.txt` 与 `field_acceptance_gap_validation.txt` 均 PASS，缺口报告继续声明 `field_acceptance_ready=NO` |
| 实际 bag verified 字段 manifest 闭环 | `reports/actual_bag_test_suite_tunnel_v478_verified_final_validation_rate10/reports/actual_bag_test_suite_manifest_validation.txt`：`actual_bag_test_suite_manifest_validation_status=PASS`，并自动复验 `suite_summary_verified_suite_status_status=PASS`、`suite_summary_suite_manifest_validation_after_execute_status=PASS`、`suite_summary_field_acceptance_gap_audit_exit_status=PASS`、`suite_summary_field_acceptance_gap_validation_after_execute_status=PASS`；同目录 summary `actual_bag_test_suite_status=PASS`、verified 四字段 PASS/exit 1，gap validation PASS，缺口报告继续声明 `field_acceptance_ready=NO` |
| profile 推荐入口 verified execute 要求 | `reports/actual_bag_profile_tunnel_v479_verified_execute_requirement/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_verified_execute_required_status=PASS`、`recommended_suite_entry_gate_status=PASS`；同目录 `recommended_suite/reports/actual_bag_test_suite_manifest_validation.txt`：`actual_bag_test_suite_manifest_validation_status=PASS`，verified 四字段 manifest 状态均 PASS；`recommended_suite/reports/field_acceptance_gap_validation.txt` 为 PASS，缺口报告仍声明 `field_acceptance_ready=NO` |
| profile 推荐入口脚本 `--execute` 反查 | `reports/actual_bag_profile_tunnel_v480_entry_execute_gate/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_verified_execute_required_status=PASS`；真实篡改同目录 `commands/run_recommended_actual_bag_test_suite.sh` 删除 `--execute` 后 validation 输出 `actual_bag_profile_validation_status=FAIL` 和 `recommended_suite_entry_gate_status=FAIL`，恢复入口脚本后 validation 再次 PASS |
| profile 推荐入口命令一致性 | `reports/actual_bag_profile_tunnel_v481_entry_command_match/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_command_status=PASS`、`recommended_suite_entry_gate_status=PASS`；真实篡改同目录 `commands/run_recommended_actual_bag_test_suite.sh` 将 `--rate 1.0` 改成 `--rate 0.5` 后 validation 输出 `actual_bag_profile_validation_status=FAIL` 和 `recommended_suite_entry_gate_status=FAIL`，恢复入口脚本后 validation 再次 PASS |
| profile 起步速度审计参数保护 | `reports/actual_bag_profile_tunnel_v482_initial_velocity_audit_arg/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_command_status=PASS`、`recommended_suite_entry_gate_status=PASS`；profile 捕获 `/novatel_data/inspvax` 首样本速度 `4.417557 m/s`；真实双文件篡改删除 `--initial-velocity-topic /novatel_data/inspvax` 后 validation 输出 `actual_bag_profile_validation_status=FAIL` 和 `recommended_suite_command_status=FAIL`，恢复后 validation 再次 PASS |
| suite 起步速度捕获 manifest gate | `reports/actual_bag_test_suite_tunnel_v483_initial_velocity_capture_manifest_rate10/reports/actual_bag_test_suite_manifest_validation.txt`：`actual_bag_test_suite_manifest_validation_status=PASS`、`smoke_initial_velocity_initial_velocity_reference_status_status=PASS`、`full_initial_velocity_initial_velocity_reference_status_status=PASS`；smoke/full `initial_velocity_reference.txt` 均捕获 `/novatel_data/inspvax` 首样本速度 `4.417557 m/s`；真实篡改 smoke 状态为 `MISSING` 后 validation 输出 `actual_bag_test_suite_manifest_validation_status=FAIL` 和 `smoke_initial_velocity_initial_velocity_reference_status_status=FAIL`，恢复后 validation 再次 PASS |
| actual bag 可选起步速度参考 gate | `reports/actual_bag_test_suite_tunnel_v484_optional_initial_velocity_guard_rate10_fixed/reports/actual_bag_test_suite_manifest_validation.txt`：`actual_bag_test_suite_manifest_validation_status=PASS`、`suite_summary_initial_velocity_reference_required_status=PASS`、`suite_summary_initial_velocity_reference_topic_status=PASS`、`smoke_initial_velocity_initial_velocity_reference_status_status=PASS`、`full_initial_velocity_initial_velocity_reference_status_status=PASS`；smoke/full 均捕获 `/novatel_data/inspvax` 首样本速度 `4.417557 m/s`，`play_topics=/velodyne_points,/left/lslidar_point_cloud,/right/velodyne_points,/imu/data,/time_reference`，并继续证明 `velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`。无速度参考路径由 `--no-initial-velocity-reference` 契约测试覆盖，要求 required=NO、topic=NONE、status=NOT_PRESENT_INITIAL_TEST、policy=NOT_AVAILABLE_INITIAL_TEST |
| actual bag field acceptance gap 采集命令 | `reports/actual_bag_test_suite_tunnel_v485_gap_collection_commands_8s/reports/field_acceptance_gap_validation.txt`：`field_acceptance_gap_validation_status=PASS`、`plc_feedback_collection_command_status=PASS`、`section_export_collection_command_status=PASS`、`pps_ptp_wiring_collection_command_status=PASS`、`power_loss_resume_collection_command_status=PASS`、`runtime_deployment_collection_command_status=PASS`、`runtime_stability_24h_collection_command_status=PASS`、`field_acceptance_collection_command_status=PASS`；同目录 summary 为 `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`，gap report 明确输出 `required_next_evidence` 和 7 类采集命令 |
| actual bag 初测就绪证据入口 | `reports/actual_bag_test_suite_tunnel_v486_initial_readiness_8s/reports/actual_bag_initial_test_readiness_validation.txt`：`actual_bag_initial_test_readiness_validation_status=PASS`、`actual_bag_user_bag_test_ready_status=PASS`、`actual_bag_initial_test_readiness_status_status=PASS`、`suite_summary_actual_bag_initial_test_readiness_after_execute_status=PASS`；同目录 readiness 报告输出 `actual_bag_initial_test_readiness_status=PASS`、`actual_bag_user_bag_test_ready=YES`、`field_acceptance_eligible=NO`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`，证明当前 LiDAR+IMU-only 实际 bag 可作为用户同类 bag 初测入口，但仍不是最终现场验收 |
| profile 推荐入口 readiness 闭环 | `reports/actual_bag_profile_tunnel_v487_initial_readiness_entry/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_initial_readiness_required_status=PASS`、`recommended_suite_initial_readiness_entry_status=PASS`；同目录 `commands/run_recommended_actual_bag_test_suite.sh` 会先校验 profile，再跑推荐 `actual_bag_test_suite.sh --execute`，最后调用 `recommended_suite/commands/validate_actual_bag_initial_test_readiness.sh`。推荐入口已端到端跑完真实 `Tunnel.bag` 60 s smoke + 195 s full，`recommended_suite/reports/actual_bag_initial_test_readiness.txt` 输出 `actual_bag_initial_test_readiness_status=PASS`、`actual_bag_user_bag_test_ready=YES`，但 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| field acceptance handoff 交接证据 | `reports/actual_bag_test_suite_tunnel_v488_field_acceptance_handoff_8s/reports/field_acceptance_handoff_validation.txt`：`field_acceptance_handoff_validation_status=PASS`、`field_acceptance_handoff_status_status=PASS`、`field_acceptance_handoff_ready_status=PASS`、`field_acceptance_collection_command_status=PASS`、`final_gate_command_status=PASS`；同目录 handoff 报告输出 `field_acceptance_handoff_status=PASS`、`field_acceptance_handoff_ready=YES`、`field_acceptance_ready=NO`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` 和完整 `required_next_evidence`，证明当前只完成“转现场证据采集”的交接闭环，不是最终验收 |
| profile 推荐入口 handoff 闭环 | `reports/actual_bag_profile_tunnel_v489_handoff_entry/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_field_acceptance_handoff_required_status=PASS`、`recommended_suite_field_acceptance_handoff_entry_status=PASS`；同目录 `commands/run_recommended_actual_bag_test_suite.sh` 会先校验 profile，再跑推荐 `actual_bag_test_suite.sh --execute`，随后调用 `recommended_suite/commands/validate_actual_bag_initial_test_readiness.sh` 和 `recommended_suite/commands/validate_field_acceptance_handoff.sh`。推荐入口已端到端跑完真实 `Tunnel.bag` 60 s smoke + 195 s full，`recommended_suite/reports/field_acceptance_handoff_validation.txt` 输出 `field_acceptance_handoff_validation_status=PASS`、`field_acceptance_handoff_ready=YES`，但最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| field acceptance handoff bundle manifest | `reports/actual_bag_test_suite_tunnel_v490_handoff_manifest_8s/reports/field_acceptance_handoff_manifest_validation.txt`：`field_acceptance_handoff_manifest_validation_status=PASS`、`suite_summary_actual_bag_test_suite_status_status=PASS`、`field_acceptance_handoff_validation_field_acceptance_handoff_validation_status_status=PASS`、`field_acceptance_handoff_field_acceptance_status_status=PASS`；同目录 `field_acceptance_handoff_manifest.txt` 固定列出 suite summary、suite manifest validation、gap/readiness/handoff 报告和各 validator 命令，summary 输出 `field_acceptance_handoff_manifest_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐入口 handoff bundle manifest 闭环 | `reports/actual_bag_profile_tunnel_v491_handoff_manifest_entry/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_required_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`；同目录推荐入口已端到端跑完真实 `Tunnel.bag` 60 s smoke + 195 s full，`recommended_suite/reports/field_acceptance_handoff_manifest_validation.txt` 输出 `field_acceptance_handoff_manifest_validation_status=PASS`，suite summary 输出 `field_acceptance_handoff_manifest_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐入口现场采集计划闭环 | `reports/actual_bag_profile_tunnel_v492_collection_plan_entry/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_field_acceptance_collection_plan_required_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS`；同目录推荐入口已端到端跑完真实 `Tunnel.bag` 60 s smoke + 195 s full，`recommended_suite/reports/field_acceptance_collection_plan_validation.txt` 输出 `field_acceptance_collection_plan_validation_status=PASS`、`collection_plan_ready_status=PASS`、`final_success_gate_status=PASS`，suite summary 输出 `field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| suite manifest 对 collection plan 复验 | `reports/actual_bag_profile_tunnel_v493_collection_plan_manifest/recommended_suite/reports/actual_bag_test_suite_manifest_validation.txt`：`actual_bag_test_suite_manifest_validation_status=PASS`、`field_acceptance_collection_plan_exists_status=PASS`、`field_acceptance_collection_plan_validation_exists_status=PASS`、`field_acceptance_collection_plan_field_acceptance_collection_plan_status_status=PASS`、`field_acceptance_collection_plan_final_success_gate_status=PASS`、`field_acceptance_collection_plan_validation_field_acceptance_collection_plan_validation_status_status=PASS`、`field_acceptance_collection_plan_validation_final_success_gate_status_status=PASS`；同目录 summary 输出 `field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐入口后置 manifest 复验 | `reports/actual_bag_profile_tunnel_v494_collection_plan_manifest_entry/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_required_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`、`recommended_suite_field_acceptance_collection_plan_required_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录 `commands/run_recommended_actual_bag_test_suite.sh` 第 10 行调用 `validate_field_acceptance_collection_plan.sh`、第 11 行调用 `validate_actual_bag_test_suite.sh`，推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，suite manifest validation 中 collection plan 复验字段均 PASS，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐入口 `suite_out` 绑定 | `reports/actual_bag_profile_tunnel_v495_suite_out_binding/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_required_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录 profile 报告 `recommended_suite_out=/home/bai/Desktop/Tunnel-LIO/reports/actual_bag_profile_tunnel_v495_suite_out_binding/recommended_suite`，入口脚本第 4 行 `suite_out` 与该目录一致，推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，suite manifest validation 中 collection plan 复验字段均 PASS，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐输出目录锚定 | `reports/actual_bag_profile_tunnel_v496_suite_out_anchor/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_out_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录 profile 报告 `recommended_suite_out=/home/bai/Desktop/Tunnel-LIO/reports/actual_bag_profile_tunnel_v496_suite_out_anchor/recommended_suite`，入口脚本第 4 行 `suite_out` 与当前 profile 根目录下的 `recommended_suite` 锚点一致，推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，suite manifest validation 中 collection plan 复验字段均 PASS，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| suite manifest 报告路径锚定 | `reports/actual_bag_profile_tunnel_v497_suite_manifest_path_anchor/recommended_suite/reports/actual_bag_test_suite_manifest_validation.txt`：`actual_bag_test_suite_manifest_validation_status=PASS`、`summary_path_status=PASS`、`metrics_report_path_status=PASS`、`event_file_path_status=PASS`、`smoke_summary_path_status=PASS`、`full_summary_path_status=PASS`、`smoke_event_validation_path_status=PASS`、`full_event_validation_path_status=PASS`、`smoke_initial_velocity_path_status=PASS`、`full_initial_velocity_path_status=PASS`、`smoke_inspection_path_status=PASS`、`full_inspection_path_status=PASS`、`field_acceptance_collection_plan_path_status=PASS`、`field_acceptance_collection_plan_validation_path_status=PASS`；同目录 profile validation PASS，推荐入口脚本第 4 行 `suite_out` 仍锚定当前 profile 根目录下的 `recommended_suite`，suite summary 输出 `actual_bag_test_suite_status=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| handoff bundle manifest 报告路径锚定 | `reports/actual_bag_profile_tunnel_v498_handoff_manifest_path_anchor/recommended_suite/reports/field_acceptance_handoff_manifest_validation.txt`：`field_acceptance_handoff_manifest_validation_status=PASS`、`suite_summary_path_status=PASS`、`suite_manifest_path_status=PASS`、`suite_manifest_validation_path_status=PASS`、`field_acceptance_gap_report_path_status=PASS`、`field_acceptance_gap_validation_path_status=PASS`、`actual_bag_initial_test_readiness_path_status=PASS`、`actual_bag_initial_test_readiness_validation_path_status=PASS`、`field_acceptance_handoff_path_status=PASS`、`field_acceptance_handoff_validation_path_status=PASS`；同目录 profile validation PASS，推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`field_acceptance_handoff_manifest_after_execute=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| collection plan source 路径锚定 | `reports/actual_bag_profile_tunnel_v499_collection_plan_source_path_anchor/recommended_suite/reports/field_acceptance_collection_plan_validation.txt`：`field_acceptance_collection_plan_validation_status=PASS`、`source_field_acceptance_handoff_path_status=PASS`、`source_field_acceptance_handoff_validation_path_status=PASS`、`source_field_acceptance_handoff_manifest_path_status=PASS`、`source_field_acceptance_handoff_manifest_validation_path_status=PASS`、`final_success_gate_status=PASS`；同目录 profile validation PASS，推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`field_acceptance_handoff_manifest_after_execute=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| suite manifest 对 collection plan source 复验 | `reports/actual_bag_profile_tunnel_v500_suite_manifest_collection_source_revalidation/recommended_suite/reports/actual_bag_test_suite_manifest_validation.txt`：`actual_bag_test_suite_manifest_validation_status=PASS`、`field_acceptance_collection_plan_source_field_acceptance_handoff_path_status=PASS`、`field_acceptance_collection_plan_source_field_acceptance_handoff_validation_path_status=PASS`、`field_acceptance_collection_plan_source_field_acceptance_handoff_manifest_path_status=PASS`、`field_acceptance_collection_plan_source_field_acceptance_handoff_manifest_validation_path_status=PASS`、`field_acceptance_collection_plan_validation_source_field_acceptance_handoff_path_status_status=PASS`、`field_acceptance_collection_plan_validation_source_field_acceptance_handoff_validation_path_status_status=PASS`、`field_acceptance_collection_plan_validation_source_field_acceptance_handoff_manifest_path_status_status=PASS`、`field_acceptance_collection_plan_validation_source_field_acceptance_handoff_manifest_validation_path_status_status=PASS`；同目录 profile validation PASS，推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐入口可执行行 gate | `reports/actual_bag_profile_tunnel_v501_profile_entry_executable_line_gate/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`、`recommended_suite_out_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录推荐入口脚本第 5-11 行按真实命令顺序执行 profile validator、actual bag suite、readiness、handoff、handoff manifest、collection plan 和 post-collection suite manifest validator，推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐 suite 命令精确行 gate | `reports/actual_bag_profile_tunnel_v502_profile_entry_exact_suite_command/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_command_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`、`recommended_suite_out_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录推荐入口脚本第 6 行精确等于 profile 报告的 `recommended_suite_command`，推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，suite manifest validation 对 collection plan source provenance 上层复验字段均 PASS，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐入口 `suite_out` 单一赋值 gate | `reports/actual_bag_profile_tunnel_v503_profile_entry_suite_out_single_assignment/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_out_status=PASS`、`recommended_suite_command_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录推荐入口脚本第 4 行唯一声明 `suite_out=/home/bai/Desktop/Tunnel-LIO/reports/actual_bag_profile_tunnel_v503_profile_entry_suite_out_single_assignment/recommended_suite`，第 6 行精确执行 profile 推荐 suite 命令，第 7-11 行 readiness、handoff、handoff manifest、collection plan 和 post-collection suite manifest validators 均复用该目录；推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 后置 gate 精确 suite 命令锚定 | `reports/actual_bag_profile_tunnel_v504_profile_post_gates_exact_suite_anchor/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_initial_readiness_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`、`recommended_suite_command_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录推荐入口脚本第 6 行精确执行 profile 推荐 suite 命令，第 7-11 行所有后置 validators 均必须相对该精确命令排序，推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐入口提前终止 gate | `reports/actual_bag_profile_tunnel_v505_profile_entry_no_early_termination/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_initial_readiness_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`、`recommended_suite_command_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录推荐入口脚本第 5 行先执行 profile validator，第 6 行精确执行 profile 推荐 suite 命令，第 7-11 行执行后置 validators，闭环窗口内无 `exit`、`return` 或 `exec` 提前终止命令；推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐入口包裹式提前终止 gate | `reports/actual_bag_profile_tunnel_v506_profile_entry_wrapped_early_termination_gate/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_initial_readiness_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`、`recommended_suite_command_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录推荐入口脚本第 5 行先执行 profile validator，第 6 行精确执行 profile 推荐 suite 命令，第 7-11 行执行后置 validators，闭环窗口内无 shell 边界上的 `exit`、`return` 或 `exec` 提前终止 token；推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐入口 strict mode 持续性 gate | `reports/actual_bag_profile_tunnel_v507_profile_entry_strict_mode_relaxation_gate/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_initial_readiness_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`、`recommended_suite_command_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录推荐入口脚本第 2 行启用 `set -euo pipefail`，第 5 行先执行 profile validator，第 6 行精确执行 profile 推荐 suite 命令，第 7-11 行执行后置 validators，闭环窗口内无 `set +e`、`set +u` 或 `set +o pipefail` strict mode relaxation；推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| profile 推荐入口包裹式 strict mode relaxation gate | `reports/actual_bag_profile_tunnel_v508_profile_entry_wrapped_strict_mode_relaxation_gate/reports/actual_bag_profile_validation.txt`：`actual_bag_profile_validation_status=PASS`、`recommended_suite_entry_gate_status=PASS`、`recommended_suite_initial_readiness_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_entry_status=PASS`、`recommended_suite_field_acceptance_handoff_manifest_entry_status=PASS`、`recommended_suite_field_acceptance_collection_plan_entry_status=PASS`、`recommended_suite_collection_plan_manifest_revalidation_entry_status=PASS`、`recommended_suite_command_status=PASS`、`velocity_reference_played_to_slam=NO`、`continuous_velocity_reference_used=NO`；同目录推荐入口脚本第 2 行启用 `set -euo pipefail`，第 5 行先执行 profile validator，第 6 行精确执行 profile 推荐 suite 命令，第 7-11 行执行后置 validators，闭环窗口内无 shell 控制结构包裹的 `set +e`、`set +u` 或 `set +o pipefail` strict mode relaxation；推荐 suite summary 输出 `actual_bag_test_suite_status=PASS`、`verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_collection_plan_after_execute=PASS`，最终仍为 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` |
| 本轮软件补强 | v5.12 收紧 `lidar_pointcloud` 与 `timoo_pointcloud` 的 `gen_calibration.py` 标定 XML 转 YAML 工具：enabled laser 必须具备 `laser_id/rot/vert/dist/dist_x/dist_y/vert_offset/focal_distance/focal_slope` 等 `calibration.cc` 读取必需字段，缺 `points_`、缺 `id_`、laser id 越界、enabled laser 记录数不一致或缺必需字段时非零退出且不落 YAML；同时修正 Python3 下无 `xrange` 路径和 disabled/non-contiguous laser id 时的字段归并方式，并新增两个包的脚本 gtest。该项是离线标定输入污染 fail-closed 补强，不替代现场外参标定或实物安装复核 |
| 本轮算法补强 | v5.11 将 `slam_backend_manager` 的全局描述子匹配从仅返回旋转最大相似度，扩展为可审计 `DescriptorMatch`：输出 best yaw shift、second-best score、Top1/Second 比值和 `accepted/ambiguous_rotation` 原因；新增 Scan Context ring key 辅助接口，候选配置新增默认关闭的 `min_rotation_uniqueness_ratio`，显式开启后可拒绝长直/对称片段中 yaw shift 不唯一的单候选回环。该项是软件侧 Scan Context/ISC 候选误匹配抑制首版，不替代真实 revisit bag、现场标靶样本或学习型全局描述子替换验证 |
| 本轮实际数据补强 | v5.09 将 actual bag profile 的 PLC 边界从“必须无 PLC topic”改为“无 PLC topic 或 PLC-present profile-only 均可验证”：带 PLC topic 的后续 bag 不会因 profile 阶段被误拒绝，但必须下沉 `plc_feedback_status=PRESENT_PROFILE_ONLY`、`plc_feedback_gate_status=PRESENT_PROFILE_ONLY_FIELD_VALIDATION_REQUIRED` 且 `field_acceptance_eligible=NO`。真实 `Tunnel.bag` v509 profile validation PASS，确认当前包仍是 LiDAR+IMU-only、无 PLC 反馈、速度参考只用于初始审计 |
| 本轮实际数据补强 | v5.08 将 actual bag profile 推荐入口 strict mode relaxation gate 从“行首 `set` 命令”收紧到“闭环窗口内任意 shell 分隔符后的 `set` token”：`if true; then set +e; fi` 这类包裹式关闭 strict mode 不能绕过 profile validator、真实 suite 和后置 validators。真实 `Tunnel.bag` v508 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，起步速度参考只用于初始审计，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v5.07 将 actual bag profile 推荐入口从“存在 strict mode 声明”收紧为“闭环窗口持续保持 strict mode”：profile validator 到 post-collection suite manifest revalidation 之间出现 `set +e`、`set +u` 或 `set +o pipefail` 都会使 `recommended_suite_entry_gate_status=FAIL`。真实 `Tunnel.bag` v507 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，起步速度参考只用于初始审计，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v5.06 将 actual bag profile 推荐入口提前终止 gate 从“首 token 为 `exit`、`return` 或 `exec`”收紧到“闭环窗口内任意 shell 边界上的提前终止 token”：`if true; then exit 0; fi` 这类包裹式退出不能绕过真实 suite 和后置 validators。真实 `Tunnel.bag` v506 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，起步速度参考只用于初始审计，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v5.05 将 actual bag profile 推荐入口从“命令顺序正确”继续收紧为“闭环窗口不可提前终止”：profile validator 到 post-collection suite manifest revalidation 之间出现 `exit`、`return` 或 `exec` 都会使 `recommended_suite_entry_gate_status=FAIL`。真实 `Tunnel.bag` v505 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，起步速度参考只用于初始审计，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v5.04 将 actual bag profile 推荐入口的后置 readiness/handoff/collection-plan gate 从“包含 `actual_bag_test_suite.sh` 的首行”收紧为精确 `recommended_suite_command` 锚点：伪 `echo`、wrapper 或说明文本不能制造 suite 已执行顺序外观。真实 `Tunnel.bag` v504 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v5.03 将 actual bag profile 推荐入口的 `suite_out` 绑定从“出现正确赋值”收紧为“非注释可执行行唯一赋值、精确等于 profile 推荐目录、且早于 suite 命令”：入口脚本不得先写正确目录再重写到外部目录，使后续 readiness/handoff/collection plan validators 验错 suite。真实 `Tunnel.bag` v503 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v5.02 将 actual bag profile 推荐 suite 命令 gate 从子串/包含关系收紧为非注释可执行行的精确等值检查：`echo`、wrapper、注释或只包含 `recommended_suite_command` 的文本不能替代真实 `actual_bag_test_suite.sh --execute` 命令。真实 `Tunnel.bag` v502 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v5.01 将 actual bag profile 推荐入口 gate 从纯文本查找收紧为非注释可执行行顺序检查：注释或说明文本中的 validator 名不能替代真实执行。真实 `Tunnel.bag` v501 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v5.00 将 collection plan source provenance 纳入 suite manifest 上层复验：即使 `field_acceptance_collection_plan_validation.txt` 仍保留旧 PASS，`validate_actual_bag_test_suite.sh` 也必须直接复验 plan 本体四个 `source_field_acceptance_*` 路径，并要求 validation 报告下沉的四个 path status 均 PASS。真实 `Tunnel.bag` v500 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v4.99 将 field acceptance collection plan 中的四个 source provenance 路径固定到当前 suite `reports/` 下的 handoff、handoff validation、handoff manifest 和 handoff manifest validation 报告：即使外部目录存在同名 PASS 报告，collection plan validator 也必须通过 `source_field_acceptance_*_path_status=FAIL` 拒绝。真实 `Tunnel.bag` v499 profile 推荐入口 60 s + 195 s verified suite、handoff manifest validation、collection plan validation 和后置 suite manifest revalidation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v4.98 将 field acceptance handoff bundle manifest 中的报告路径固定为 suite 内部相对路径：即使外部目录存在同名 PASS 报告，handoff manifest validator 也必须通过 `*_path_status=FAIL` 拒绝绝对路径、`../` 或兄弟 suite 拼接。真实 `Tunnel.bag` v498 profile 推荐入口 60 s + 195 s verified suite、handoff manifest validation、collection plan validation 和后置 suite manifest revalidation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v4.97 将 actual bag suite manifest 中的报告路径固定为 suite 内部相对路径：即使外部目录存在同名 PASS 报告，manifest validator 也必须通过 `*_path_status=FAIL` 拒绝绝对路径、`../` 或兄弟 suite 拼接。真实 `Tunnel.bag` v497 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v4.96 将 profile 报告中的 `recommended_suite_out` 锚定到当前 profile 根目录下的 `recommended_suite`：即使 profile 报告与入口脚本被同步篡改到外部目录，也必须由 `recommended_suite_out_status=FAIL` 拒绝。真实 `Tunnel.bag` v496 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v4.95 将 profile 推荐入口的 `suite_out` 变量与 profile 报告的 `recommended_suite_out` 绑定：profile validator 必须拒绝入口脚本把后续 validators 指向其它目录，防止真实 suite 命令与后续 validator 目录脱钩。真实 `Tunnel.bag` v495 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v4.94 将 profile 推荐入口与 collection plan 后置 suite manifest revalidation 绑定：profile 报告必须声明 `recommended_suite_collection_plan_manifest_revalidation_required=YES`，profile validator 必须反查推荐入口在 collection plan validator 后再次调用 `validate_actual_bag_test_suite.sh`；缺字段、字段篡改或入口脚本删除该后置 manifest validator 均 FAIL。真实 `Tunnel.bag` v494 profile 推荐入口 60 s + 195 s verified suite、collection plan validation 和后置 suite manifest revalidation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v4.93 将 collection plan 纳入 suite manifest 二次复验：manifest 必须声明 collection plan validation 报告，`validate_actual_bag_test_suite.sh` 必须在已执行或报告已生成时复验 collection plan 本体、collection plan validation 和最终成功门槛；篡改 `final_success_gate` 会同时导致 collection plan validator 与 suite manifest validator FAIL。真实 `Tunnel.bag` v493 profile 推荐入口 60 s + 195 s verified suite 与 collection plan manifest 复验 PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v4.92 将 actual bag handoff bundle 进一步收口为现场采集计划入口：suite 必须生成 `field_acceptance_collection_plan.txt` 和 validator，按 1-7 步固定 PLC feedback、section export、PPS/PTP wiring、power-loss resume、runtime deployment、24 h runtime stability、final field acceptance，并写明最终成功门槛为 `record_session.sh generated commands/validate_evidence.sh => field_acceptance_status=PASS`；profile 推荐入口必须在 handoff manifest validator 后继续调用 collection plan validator。真实 `Tunnel.bag` v492 profile validation PASS，推荐入口 60 s + 195 s verified suite、handoff manifest 和 collection plan validation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v4.91 将 profile 推荐入口与 handoff bundle manifest gate 绑定：profile 报告必须声明 `recommended_suite_field_acceptance_handoff_manifest_required=YES`，profile validator 必须反查推荐入口在 suite verified execute、readiness validator 和 handoff validator 后调用 handoff manifest validator；缺字段、字段篡改或入口脚本删除 handoff manifest validator 均 FAIL。真实 `Tunnel.bag` v491 profile validation PASS，推荐入口 60 s + 195 s verified suite、readiness、handoff 和 handoff manifest validation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v4.90 将 handoff 交接证据升级为 bundle manifest：actual bag suite 必须生成 `field_acceptance_handoff_manifest.txt`，validator 必须复验交接依赖文件和命令均存在且状态一致；篡改 handoff validation 为 FAIL 会导致 handoff manifest validation FAIL。真实 `Tunnel.bag` v490 8 s + 8 s verified suite、readiness、handoff 和 handoff manifest validation 均 PASS |
| 本轮实际数据补强 | v4.89 将 profile 推荐入口与 suite handoff gate 绑定：profile 报告必须声明 `recommended_suite_field_acceptance_handoff_required=YES`，profile validator 必须反查推荐入口在 suite verified execute 和 readiness validator 后调用 handoff validator；缺字段、字段篡改或入口脚本删除 handoff validator 均 FAIL。真实 `Tunnel.bag` v489 profile validation PASS，推荐入口 60 s + 195 s verified suite、readiness validation 和 handoff validation PASS，最终仍不是现场验收 PASS |
| 本轮实际数据补强 | v4.88 将 actual bag suite 的 readiness/gap 结果收口为独立 `field_acceptance_handoff` gate：handoff PASS 只允许表示可以按报告中的 7 类命令采集真实 PLC、section、PPS/PTP、断电续建、部署、24 h 长稳和最终验收证据；validator 会拒绝 handoff 报告被篡改成最终 `field_acceptance_status=PASS`。真实 `Tunnel.bag` v488 8 s + 8 s verified suite、readiness validation 和 handoff validation 均 PASS |
| 本轮实际数据补强 | v4.87 将 profile 入口与 suite readiness gate 绑定：profile 报告必须声明 `recommended_suite_initial_readiness_required=YES`，profile validator 必须反查推荐入口包含 suite 后置 readiness validator，缺字段、字段篡改或入口脚本删除 readiness validator 均 FAIL。真实 `Tunnel.bag` v487 profile validation PASS，推荐入口 60 s + 195 s verified suite 和 readiness validation PASS |
| 本轮实际数据补强 | v4.86 将 actual bag suite 的多个 PASS 证据收口为单一 `actual_bag_initial_test_readiness` gate：ready 只代表“可用用户同类 LiDAR+IMU-only bag 做初测”，必须同时保留 `field_acceptance_eligible=NO` 和最终缺口清单；新增篡改 FAIL 契约测试覆盖 readiness 报告不能被改成其它 ready 状态。真实 `Tunnel.bag` v486 8 s + 8 s verified suite PASS，readiness validation PASS |
| 本轮实际数据补强 | v4.85 将 final gap report 从“只列缺口”升级为“缺口 + 对应采集命令”：actual bag suite 会生成 PLC feedback、section export、PPS/PTP wiring、power-loss resume、runtime deployment、24 h runtime stability 和 final field acceptance 的建议命令，validator 严格复验这些字段并有篡改 FAIL 契约测试。真实 `Tunnel.bag` v485 8 s + 8 s verified suite PASS，最终 gap 仍保持 `field_acceptance_ready=NO` |
| 本轮实际数据补强 | v4.84 将 actual bag profile/replay/suite 扩展为“速度参考可选但语义必须显式”的初测入口：有速度参考时仍要求 smoke/full `CAPTURED`，无速度参考时必须使用 `--no-initial-velocity-reference` 并写出 required NO/topic NONE/status NOT_PRESENT/policy NOT_AVAILABLE；新增执行态回归测试覆盖生成的 `run_suite.sh` 不得因漏注入初始速度字段在真实执行阶段失败。真实 `Tunnel.bag` v484 fixed 1.0x verified suite PASS，最终 gap 仍保持 `field_acceptance_ready=NO` |
| 本轮实际数据补强 | v4.83 将 actual bag suite manifest validator 扩展到 smoke/full 初始速度捕获状态，要求 `initial_velocity_reference_status=CAPTURED`，防止只有策略字段 PASS 但没有真实首样本证据。真实 `Tunnel.bag` v483 1.0x verified suite PASS，篡改 smoke 捕获状态检查 FAIL |
| 本轮实际数据补强 | v4.82 将捕获到的 `initial_velocity_reference_topic` 与推荐 suite 命令中的 `--initial-velocity-topic` 绑定校验，防止非零初速度审计被推荐入口丢掉。真实 `Tunnel.bag` v482 profile validation PASS，删除速度审计参数篡改检查 FAIL |
| 本轮实际数据补强 | v4.81 将推荐入口脚本本体与 profile 报告中的 `recommended_suite_command` 做原文一致性校验，防止入口脚本改 topic、rate、输出目录或其它命令后仍通过。真实 `Tunnel.bag` v481 profile validation PASS，入口脚本 rate 篡改检查 FAIL |
| 本轮实际数据补强 | v4.80 将推荐入口脚本本体的 `--execute` 纳入 profile validator 反查，防止只改入口脚本绕过 verified execute。真实 `Tunnel.bag` v480 profile validation PASS，入口脚本删除 `--execute` 篡改检查 FAIL |
| 本轮实际数据补强 | v4.79 将 profile 生成的推荐 suite 入口显式绑定 verified execute 要求，profile validator 拒绝将 `recommended_suite_verified_execute_required` 篡改为 NO。真实 `Tunnel.bag` v479 profile validation PASS，推荐入口端到端执行 PASS，并生成 verified suite manifest 闭环证据 |
| 本轮实际数据补强 | v4.78 让 `run_verified_suite.sh` 在追加 verified suite 字段后再次运行 manifest validation，确保最终报告天然包含 verified 字段 PASS。真实 `Tunnel.bag` v478 1.0x smoke/full suite 退出 0，最终 manifest validation 已自动闭环 |
| 本轮实际数据补强 | v4.77 将 optional verified execute 字段纳入 suite manifest validator 严格校验，字段缺失时允许 `NOT_PRESENT_NA`，字段出现后必须等于 PASS/exit 1；新增篡改 FAIL 契约测试覆盖 `verified_suite_status=FAIL` |
| 本轮实际数据补强 | v4.76 将 `actual_bag_test_suite.sh --execute` 收口为 verified execute wrapper，自动运行 suite、manifest validation、预期非零 gap audit 和 gap validation。真实 `Tunnel.bag` v476 1.0x smoke/full suite 退出 0，并在 summary 中追加 verified suite PASS 字段 |
| 本轮实际数据补强 | v4.75 给 actual bag suite 增加独立 `validate_field_acceptance_gap.sh`，使 `field_acceptance_gap_report.txt` 被篡改成最终 ready 或缺口字段缺失时不能通过。真实 `Tunnel.bag` v475 1.0x smoke/full suite PASS，gap audit 预期非零，gap validation PASS |
| 本轮实际数据补强 | v4.74 将推荐入口脚本的 gate 顺序纳入 profile validator 自检，防止 `run_recommended_actual_bag_test_suite.sh` 被篡改为跳过 validator 后仍得到 profile validation PASS。新增篡改 FAIL 契约测试，真实 `Tunnel.bag` v474 validator entry gate 已生成 PASS 证据 |
| 本轮实际数据补强 | v4.73 将 profile validator 固定接入推荐 suite 入口，用户运行 `run_recommended_actual_bag_test_suite.sh` 时会先校验 profile，校验失败则不会进入 smoke/full replay suite。新增顺序契约测试覆盖 validator 在 suite 前执行，真实 `Tunnel.bag` v473 profile-gated 推荐入口已生成 PASS 证据 |
| 本轮实际数据补强 | v4.72 将 actual bag profile 自检前置为独立命令，用户运行 suite 前即可确认 profile 证据本身完整、一致且没有把速度参考连续接入 SLAM。新增 profile validator PASS/篡改 FAIL 契约测试，真实 `Tunnel.bag` profile validation 已生成 PASS 证据 |
| 本轮实际数据补强 | v4.66 给 actual bag replay/suite 增加显式 `--no-time-reference` 初测模式；新增契约测试覆盖 replay 不播放 `/time_reference`、证据记录 `NOT_PRESENT_INITIAL_TEST`、与 `--time-reference-topic` 互斥、suite smoke/full 参数透传；真实 `Tunnel.bag` no-time-reference smoke PASS，证明用户只有 LiDAR/IMU 的初测 bag 不会因缺失 `/time_reference` 被入口脚本误拒绝 |
| 本轮实际数据补强 | v4.60 修复 LiDAR+IMU-only 初测 bag 暴露的局部里程计早期拒绝级联，增加 keyframe reseed 恢复诊断和 evidence 下沉；`catkin_make run_tests_lio_local_odometry run_tests_mine_slam_bringup -DCATKIN_BLACKLIST_PACKAGES=""` 通过，`lio_local_odometry`：41 tests，`mine_slam_bringup`：472 tests，均 0 errors/failures/skipped；60 s/0.3x 真实 bag replay 与 generated event validation 均 PASS，但 `field_acceptance_eligible=NO` |
| 本轮实际数据补强 | v4.65 给 actual bag replay/suite 增加用户 bag topic override 和 canonical replay remap；新增契约测试覆盖自定义三雷达/IMU/time reference/速度审计 topic、速度参考不播放、非法 topic fail closed、suite smoke/full 参数透传；该补强用于用户采集 bag 的 topic 名与 `Tunnel.bag` 不一致时仍能进入同一实际数据初测证据链 |
| 本轮实际数据补强 | v4.64 给 actual bag suite 增加 field acceptance gap audit；新增/更新契约测试覆盖 dry-run 生成、synthetic executed suite PASS 后 gap audit 预期非零和缺口字段；真实 `Tunnel.bag` v464 suite manifest validation PASS，gap audit 明确最终 `field_acceptance_ready=NO` |
| 本轮实际数据补强 | v4.63 给 actual bag suite 增加独立 manifest 和校验命令；新增 synthetic executed suite 校验契约测试，覆盖字面量 key 读取、PLC topic 计数、速度参考隔离和 suite/replay/event 多层 PASS 一致性；真实 `Tunnel.bag` v463 manifest validation PASS |
| 本轮实际数据补强 | v4.62 新增实际 bag 一键初测 suite：`actual_bag_test_suite.sh` 生成 smoke/full replay、两段 event validation 和 suite summary；契约测试新增 2 条并纳入 bringup 回归，`bash -n` 通过，`catkin_make run_tests_mine_slam_bringup -DCATKIN_BLACKLIST_PACKAGES=""` 与 `catkin_test_results build/test_results/mine_slam_bringup` 通过 476 tests；真实 `Tunnel.bag` suite 在 LiDAR+IMU-only、无 PLC、车辆持续运动、普通振动口径下 PASS，但 `field_acceptance_eligible=NO` |
| 本轮实际数据补强 | v4.61 将实际数据门槛提升到 1.0x：60 s/1.0x 和整包 195 s/1.0x `Tunnel.bag` replay 均 PASS，generated event validation 均 PASS；这证明当前同类 LiDAR+IMU-only bag 可进入初步测试流程，但仍不替代 PLC/PPS/PTP/断电续建/实机部署/24 h 长稳证据 |
| 本轮实际数据补强 | v4.59 将实际 bag replay 输出桥接到标准化 HIL event/metrics 证据：生成 `actual_bag_replay_metrics_report.txt`、`actual_bag_replay_events.txt` 和 `validate_actual_bag_events.sh`；scope 固定为 `ACTUAL_LIDAR_IMU_FRONTEND_ONLY`，并显式 `field_acceptance_eligible=NO`，dry-run 不得 PASS |
| 本轮软件门控补强 | v4.58 将工况状态机、mapping control、section export 脚本和 evidence manifest 校验统一到 `IDLE_STATIC/CUTTING_STATIC/FWD_MOVE/REV_MOVE/TURNING/CMD_MOVE_NO_DISP/CONFLICT/RELOCALIZING` 八态；`/session/relocalizing` 可触发重定位冻结，PLC 指令运动但 LiDAR 未位移会进入 `CMD_MOVE_NO_DISP` 并拒绝扩图 |
| 本轮实际数据补强 | v4.57 将当前海底隧道 bag 固定为 LiDAR+IMU-only 初测口径，证据文件下沉 PLC 缺失为 `NOT_PRESENT_NA`，车辆持续运动为 `CONTINUOUS_MOTION`，并在 replay 启动命令中关闭无 PLC 条件下会误判的状态机、mapping control 和 section manager；最终现场验收仍必须另取 PLC/PPS/PTP/HIL/24 h 证据 |
| 本轮实际数据补强 | v4.56 实际 bag replay 新增 local odometry 1 Hz 覆盖 gate、`--local-odometry-config` 调参入口、融合 overlap 抽样上限、局部注册外部初值 overload 和仅基于自身成功注册的常速初值；完整 195 s、0.3x 真实 bag 质量模式证据已通过 |
| 本轮实际数据补强 | v4.55 实际 bag replay 诊断解析器兼容 `data: "key=value"` 和 `DiagnosticArray` YAML 跨行 `key/value` 两类格式，避免 fusion `callbacks` 或 local odometry `published_odometry` 被误解析为 0 |
| 本轮实际数据补强 | v4.54 实际 bag replay 新增 `/diagnostics/lio_local_odometry` 捕获和 summary 下沉字段，并将 local odometry 诊断捕获与正向 odom 发布纳入 PASS gate |
| 本轮实际数据补强 | v4.53 实际 bag replay 诊断 capture 覆盖整个回放窗口，慢速回放 timeout 按 `duration/rate` 自动放大，summary 新增覆盖门槛并将 0.3x PASS 与 1.0x 全包吞吐 FAIL 分开记录；新增回归测试覆盖慢速回放 timeout 契约 |
| 本轮实际数据补强 | v4.52 实际 bag replay 改为自有临时 ROS master，生成的 `run_replay.sh` 使用空闲本地端口、导出 `ROS_MASTER_URI`、启动受控 `roscore`，并按 process group 清理 pipeline 与 roscore；新增回归测试覆盖 pipeline process group 和 owned ROS master 清理契约 |
| 本轮实际数据补强 | v4.51 形成 `Tunnel.bag` 海底隧道回放入口、专用三雷达/IMU/状态估计配置、replay-only identity 外参、session_root 归档透传和实际 bag 证据 gate；初始速度参考仅写入 `initial_velocity_reference.txt` 做 `START_ONLY_AUDIT`，不进入 rosbag play topic 列表，不参与连续定位 |
| 本轮证据链补强续 | v4.50 绑定 runtime stability CSV `health_report` 内容 PASS 证据：最终 `field_acceptance_report.txt` 和 `lio_eval_tools` manifest 均逐条复验 CSV 引用的 runtime health 快照内容，要求合法 timestamp、runtime_dir 与当前 manifest/session 一致、磁盘/PID 合法、systemd active 来源为 `systemctl`、Docker running 来源为 `docker_inspect`；缺字段、内容 FAIL、来源覆盖或 runtime_dir 不匹配必须使 `runtime_stability_csv_status/runtime_stability_status/field_acceptance_status/evidence_status` 保持 FAIL |
| 本轮证据链补强续 | v4.49 绑定 runtime stability CSV `health_report` 实物文件证据：session 归档会把 runtime_dir 中的 health 快照复制到当前 session `logs/` 并把 CSV 引用改写为安全 basename，最终 `field_acceptance_report.txt` 和 `lio_eval_tools` manifest 均要求每条 CSV `health_report` 引用能解析到当前 session/evidence bundle 内非空 regular file；缺失、空文件、越界路径或不安全引用必须使 `runtime_stability_csv_status/runtime_stability_status/field_acceptance_status/evidence_status` 保持 FAIL |
| 本轮证据链补强续 | v4.48 绑定 runtime stability CSV 首末采样时间证据：独立 `runtime_stability.csv` 的每条采样 `timestamp` 必须合法且按采样顺序非递减，最终 `field_acceptance_report.txt` 必须下沉完全一致的 `runtime_stability_csv_first_timestamp/runtime_stability_csv_last_timestamp`，且首末采样时间必须落在 `runtime_stability_run.log` 的时间窗内，最终报告 `timestamp` 必须覆盖最后一条 CSV 采样时间；脚本生成侧和 `lio_eval_tools` manifest 侧一致拒绝缺失、不一致、畸形、倒序、越窗或时间倒置的 24 h 长稳 CSV 证据 |
| 本轮证据链补强续 | v4.47 绑定 runtime stability summary 报告自身生成时间证据：独立 `runtime_stability_summary.txt` 必须包含合法 `timestamp`，该时间必须落在 `runtime_stability_run.log` 的 `started_at/finished_at` 闭区间内，最终 `field_acceptance_report.txt` 必须下沉完全一致的 `runtime_stability_summary_timestamp`，且最终报告 `timestamp` 必须覆盖该 summary 生成时间；脚本生成侧和 `lio_eval_tools` manifest 侧一致拒绝缺失、不一致、畸形、越窗或时间倒置的 24 h 长稳 summary 证据 |
| 本轮证据链补强续 | v4.46 绑定 power-loss resume verified 报告自身生成时间证据：独立 `power_loss_resume_verified.txt` 必须包含合法 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉完全一致的 `power_loss_resume_timestamp`，且最终报告 `timestamp` 必须覆盖该 power-loss resume 报告生成时间；脚本生成侧和 `lio_eval_tools` manifest 侧一致拒绝缺失、不一致、畸形或时间倒置的断电续建证据 |
| 本轮证据链补强续 | v4.45 绑定 PPS/PTP wiring verified 报告自身生成时间证据：独立 `pps_ptp_wiring_verified.txt` 必须包含合法 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉完全一致的 `pps_ptp_wiring_timestamp`，且最终报告 `timestamp` 必须覆盖该 wiring 报告生成时间；脚本生成侧和 `lio_eval_tools` manifest 侧一致拒绝缺失、不一致、畸形或时间倒置的 PPS/PTP wiring 证据 |
| 本轮证据链补强续 | v4.44 绑定 PPS/PTP wiring 与 time sync 快照时间戳证据：独立 `pps_ptp_wiring_verified.txt` 必须下沉与独立 `time_sync` 一致的 `time_sync_timestamp`，最终 `field_acceptance_report.txt` 必须下沉一致的 `pps_ptp_wiring_time_sync_timestamp`；脚本生成侧和 `lio_eval_tools` manifest 侧一致拒绝缺失、不一致、畸形或 stale PPS/PTP wiring 证据 |
| 本轮证据链补强 | v2.62 收紧 `lio_eval_tools` evidence manifest 正整数字段溢出处理：`total_records` 等字段发生 `strtol` 溢出时必须 fail closed；v2.63 收紧 `section_manager` 与 `slam_backend_manager` 控制文本 `chainage_m` 解析：字段名精确匹配，畸形/重复/污染/非有限值回退到上一链距；v2.64 收紧 `slam_backend_manager` 稳定图台账读取/晋升：损坏、不完整、非有限或非法质量条目不能进入稳定锚点集合；v2.65 收紧 `section_manager` 与 `slam_backend_manager` 控制文本 bool/text 字段解析：`section_sample`、`machine_state` 和 `quality` 必须精确匹配唯一 key，重复、哨兵、前导空白或换行污染回退默认/上一状态，不得由 regex 后缀匹配污染截面采样、工况来源或稳定图晋升质量；v2.66 收紧 `record_session.sh` 脚本侧 metrics report 解析：`capture_power_loss_resume.sh` 与 `capture_field_acceptance.sh` 对分号记录和匹配记录块只裁剪 key，value 保留原文进入 PASS/数值 gate，防止 `overall=PASS ` 等尾随空白污染被误放行；v2.67 收紧 section export CSV 字段原样校验：`capture_section_export.sh`、`capture_field_acceptance.sh` 与 `lio_eval_tools` evidence manifest 必须拒绝 `session_id/chainage_m/state_source/quality/completeness/rmse_mm/points` 首尾空白污染；v2.68 收紧 runtime stability CSV 字段原样校验：`capture_field_acceptance.sh` 与 `lio_eval_tools` evidence manifest 必须拒绝 `disk_guard_status/watchdog_status/health_report` 等单元格首尾空白污染；v2.69 收紧 `lio_eval_tools` evidence manifest metrics report 行解析：空行可用 trim 判断，但 summary/record/recovery 行必须保留原始行进入 key/value 解析，防止行尾 value 污染被抹掉；v2.70 收紧生成脚本与 C++ evidence manifest 的有效文本值校验：`is_valid_text_value()` 和 `validEvidenceTextValue()` 必须拒绝首尾空白污染，防止人工审计字段、topic、路径或运行态文本值被 trim 语义误放行；v2.71 收紧 `validation_metrics` 与 replay event 批量读取：空行/注释可用 trim 判断，但 metrics、场景阈值和 replay 事件记录必须保留原始行进入 key/value 解析，防止最后一个 value 的行尾空白污染被抹掉；v2.72 收紧 `record_session.sh` 与 `runtime_ops.sh` 入口参数校验：manifest/runtime 文本值和绝对路径必须拒绝首尾空白，防止污染参数进入 session 归档或板端 runtime 骨架；v2.73 收紧 `/diag/dump_event` 事件切片 metadata 读取：`metadata.env` 只裁剪 key，`bag_path/pcap_path/session_name/topics` 的 value 必须原样进入有效性校验，拒绝首尾空白、哨兵、重复 key、metadata 分隔符和会污染生成 shell 命令的字符，防止事件切片命令把污染 metadata trim 后误生成；v2.74 收紧 `/diag/dump_event` service request 入口：`session_dir/output_root/reason` 必须通过有效文本值校验，`event_id` 必须是安全 token，事件时间和窗口必须为有限值，防止污染 request 写入 manifest、目录或生成的 shell 命令；v2.75 收紧 `/diag/dump_event` 路径来源：service request 的 `session_dir/output_root` 和 metadata 的 `bag_path/pcap_path` 必须是绝对路径，防止事件切片证据依赖调用时工作目录；v2.76 收紧 `/diag/dump_event` metadata 会话名：`session_name` 必须与 session 生成入口一致为安全 token，防止非 token 会话名写入事件 manifest；v2.77 收紧 `/diag/dump_event` token 路径段：`event_id` 和 metadata `session_name` 不能是 `.` 或 `..`，防止安全字符 token 仍被解释为当前/父目录；v2.78 收紧 `/diag/dump_event` 事件时间：`event_time_s` 必须为有限且非负，防止负事件时间生成 `start_time_s=0` 但 `end_time_s` 为负的事件切片命令；v2.79 收紧 `/diag/dump_event` 绝对路径单调语义：service request 的 `session_dir/output_root` 和 metadata 的 `bag_path/pcap_path` 不得包含 `.` 或 `..` 路径段，防止路径归一化逃逸事件切片目录或 artifact 来源；v2.80 收紧 session/runtime 生成入口绝对路径：`record_session.sh` 的 `session_root/runtime_dir` 与 `runtime_ops.sh` 的 `runtime_root/workspace` 不得包含 `.` 或 `..` 路径段，防止生成归档、runtime 骨架或 Docker volume 来源依赖路径归一化；v2.81 收紧根目录绝对路径：`/diag/dump_event` 的绝对路径输入、`record_session.sh` 的 `session_root/runtime_dir` 和 `runtime_ops.sh` 的 `runtime_root/workspace` 不得单独为 `/`，防止 artifact 来源、session 归档或 runtime 骨架落到文件系统根目录；v2.82 收紧 `lio_eval_tools` runtime 目录证据校验：evidence manifest、runtime health、runtime deployment 和 field acceptance 中的 `runtime_dir` 不得单独为 `/` 且不得包含 `.` 或 `..` 路径段，防止伪造证据包用根目录或归一化路径通过最终验收；v2.83 收紧 `lio_eval_tools` time sync topic 证据校验：time sync 与 field acceptance 中的 `time_status_topic/pps_topic` 必须符合 ROS topic 命名，防止伪造证据包用无效 topic 文本通过时间同步和最终验收 |
| 本轮证据链补强续 | v2.84 绑定 time sync topic 证据一致性：`lio_eval_tools` 必须要求 field acceptance 下沉的 `time_status_topic/pps_topic` 与独立 time sync 证据完全一致，防止伪造最终报告用合法但不同的 topic 文本绕过时间同步证据链 |
| 本轮证据链补强续 | v2.85 绑定 time sync 数值证据一致性：`lio_eval_tools` 必须要求 PPS/PTP wiring 和 field acceptance 下沉的 `pps_jitter_ms/mean_offset_ms` 与独立 time sync 证据完全一致，防止伪造 downstream 报告用合法但不同的时间同步数值绕过证据链 |
| 本轮证据链补强续 | v2.86 绑定 runtime health 下沉明细一致性：`lio_eval_tools` 必须要求 field acceptance 下沉的 `runtime_health_disk_available_gb/runtime_health_pid/runtime_health_systemd_active/runtime_health_docker_container_status` 与独立 runtime health 证据完全一致，防止伪造最终报告用另一组合法健康快照绕过证据链 |
| 本轮证据链补强续 | v2.87 绑定 PPS/PTP wiring 人工审计字段一致性：`lio_eval_tools` 必须要求 field acceptance 下沉的 `wiring_verified_by/wiring_verified_at` 与独立 PPS/PTP wiring 证据完全一致，防止伪造最终报告用另一组合法接线审计字段绕过证据链 |
| 本轮证据链补强续 | v2.88 绑定 power-loss resume 下沉字段一致性：`lio_eval_tools` 必须要求 field acceptance 下沉的 `power_loss_resume_source/recovery_time_s/max_recovery_time_s/power_loss_resume_confirmation_overall` 与独立 power-loss resume 证据完全一致，且 `manual_file` 来源下 `resume_verified_by/resume_verified_at` 也必须一致，防止伪造最终报告用另一组合法断电续建审计字段绕过证据链 |
| 本轮证据链补强续 | v2.89 绑定 field acceptance metrics 报告路径一致性：`lio_eval_tools` 必须要求 field acceptance 下沉的 `metrics_report` 始终指向 evidence manifest 中的独立 metrics 报告，即使断电续建来源为 `manual_file` 也不能用另一组合法 metrics 路径绕过最终验收证据链 |
| 本轮证据链补强续 | v2.90 收紧生成侧 time sync topic 校验：`record_session.sh` 生成的 `capture_time_sync.sh` 和 `capture_field_acceptance.sh` 必须对 `time_status_topic/pps_topic` 执行与入口和 C++ evidence manifest 一致的 ROS topic 名校验，防止环境覆盖或污染 time sync 证据用格式错误 topic 先生成 PASS 外观报告 |
| 本轮证据链补强续 | v2.91 收紧 PPS/PTP wiring 生成侧 time sync topic 复验：`capture_pps_ptp_wiring.sh` 必须重新读取、输出并校验 `logs/time_sync_status.txt` 的 `time_status_topic/pps_topic`，防止污染 time sync 报告用格式错误 topic 和合法 PASS 状态字段生成 `pps_ptp_wiring_verified=PASS` 外观证据 |
| 本轮证据链补强续 | v2.92 绑定 PPS/PTP wiring topic 证据一致性：`lio_eval_tools` 必须要求 `pps_ptp_wiring_verified.txt` 下沉的 `time_status_topic/pps_topic` 格式合法且与独立 time sync 证据完全一致，防止伪造 wiring 报告用另一组合法 topic 或格式错误 topic 绕过证据链 |
| 本轮证据链补强续 | v2.93 收紧 field acceptance 生成侧 PPS/PTP wiring topic 复验：`capture_field_acceptance.sh` 必须重新校验 `pps_ptp_wiring_verified.txt` 下沉的 `time_status_topic/pps_topic` 格式合法且与独立 time sync 证据一致，防止污染 wiring 报告先生成最终 PASS 外观证据 |
| 本轮证据链补强续 | v2.94 收紧 field acceptance 生成侧 PPS/PTP wiring 数值复验：`capture_field_acceptance.sh` 必须重新校验 `pps_ptp_wiring_verified.txt` 下沉的 `pps_jitter_ms/mean_offset_ms` 格式合法且与独立 time sync 证据一致，防止污染 wiring 报告先生成最终 PASS 外观证据 |
| 本轮证据链补强续 | v2.95 收紧 field acceptance 生成侧 PPS/PTP wiring 状态复验：`capture_field_acceptance.sh` 必须重新校验 `pps_ptp_wiring_verified.txt` 下沉的 `time_sync_status/capture_status/pps_status/clock_offset_status` 与独立 time sync PASS 语义一致，防止污染 wiring 报告先生成最终 PASS 外观证据 |
| 本轮证据链补强续 | v2.96 绑定 PPS/PTP wiring 的 time sync report 路径证据：`lio_eval_tools` 与 `capture_field_acceptance.sh` 必须要求 `pps_ptp_wiring_verified.txt` 下沉的 `time_sync_report` 解析后等于 evidence manifest/session 声明的独立 time sync 证据路径，且最终 `field_acceptance_report.txt` 必须下沉并复验 `pps_ptp_wiring_time_sync_report`，防止伪造 wiring 报告引用另一份 time sync 证据 |
| 本轮证据链补强续 | v2.97 绑定最终 field acceptance 的 PPS/PTP wiring 状态下沉证据：`lio_eval_tools` 必须要求 `field_acceptance_report.txt` 下沉的 `pps_ptp_wiring_time_sync_status/pps_ptp_wiring_capture_status/pps_ptp_wiring_pps_status/pps_ptp_wiring_clock_offset_status` 具备与独立 wiring 证据一致的 PASS/CAPTURED 语义，防止伪造最终报告隐藏 wiring 报告内 time sync 状态失败 |
| 本轮证据链补强续 | v2.98 绑定最终 field acceptance 的 PPS/PTP wiring topic 与数值下沉证据：`lio_eval_tools` 必须要求 `field_acceptance_report.txt` 下沉的 `pps_ptp_wiring_time_status_topic/pps_ptp_wiring_pps_topic/pps_ptp_wiring_pps_jitter_ms/pps_ptp_wiring_mean_offset_ms` 格式合法且与独立 time sync 证据完全一致，防止伪造最终报告隐藏 wiring 报告内 topic 或时间同步数值污染 |
| 本轮证据链补强续 | v2.99 绑定 field acceptance 生成侧 metrics 来源断电续建恢复时间：`capture_field_acceptance.sh` 必须在 `power_loss_resume_source=metrics_report` 时重新抽取当前 session/scenario 匹配记录的 `recovery_time_s`，并要求其与 `power_loss_resume_verified.txt` 下沉的 `recovery_time_s` 数值等价，防止污染 verified 文件用同样在门槛内但来自另一份记录或手写的恢复时间绕过最终验收 |
| 本轮证据链补强续 | v3.00 收紧 runtime health 运行态闭环：`record_session.sh` 生成的 `capture_field_acceptance.sh` 与 `lio_eval_tools` evidence manifest 必须要求 runtime health 快照中的 `systemd_active=active` 且 `docker_container_status=running`，并继续与最终 field acceptance 下沉字段一致，防止 health 快照显示 inactive/exited 但 deployment 报告单独 active/running 时绕过最终验收 |
| 本轮证据链补强续 | v3.01 收紧板端长稳采样 health 报告闭环：`runtime_ops.sh` 生成的 `runtime_stability_check.sh` 不能只相信 `runtime_health.sh` 退出码；每次采样必须解析 health 报告，要求 `runtime_dir` 绑定当前 runtime、`disk_available_gb` 为非负数、`runtime_pid` 为正整数、`systemd_active=active` 且 `docker_container_status=running`，否则 `health_failures` 必须累加并使 summary `overall=FAIL` |
| 本轮证据链补强续 | v3.02 收紧板端长稳采样参数闭环：`runtime_ops.sh` 生成的 `runtime_stability_check.sh` 必须在创建 CSV/summary 前拒绝 `--samples 0`、非数字 samples 和负 interval；`--samples` 必须为严格正整数，`--interval` 必须为非负整数，防止直接调用脚本时不采样或畸形采样仍生成 `overall=PASS` 外观证据 |
| 本轮证据链补强续 | v3.03 收紧板端 runtime 路径 CSV 安全性：`runtime_ops.sh` 生成 runtime 前必须拒绝 `runtime_root` 中的逗号，且 `runtime_name` 继续由安全 token 约束拒绝逗号，防止 `runtime_health.sh` 输出的 health_report 路径污染 `runtime_stability.csv` 固定 5 列格式 |
| 本轮证据链补强续 | v3.04 收紧 session 归档 runtime 路径 CSV 安全性：`record_session.sh` 生成 session 前必须拒绝非空 `runtime_dir` 中的逗号，防止 session 侧长稳运行与归档命令携带会污染 `runtime_stability.csv` health_report 路径的 runtime 目录进入证据链 |
| 本轮证据链补强续 | v3.05 收紧最终证据包 runtime 路径 CSV 安全性：`lio_eval_tools` 对 manifest 顶层 `runtime_dir`、runtime health/deployment 报告和 field acceptance 下沉 runtime 目录使用专用 runtime 路径校验，拒绝逗号，防止伪造证据包绕过生成侧 CSV 安全门控 |
| 本轮证据链补强续 | v3.06 收紧板端长稳 health_report 路径 CSV 安全性：`runtime_stability_check.sh` 每次采样必须先拒绝 `runtime_health.sh` 返回路径中的逗号，改写为 `missing`、累加 `health_failures` 并使 summary `overall=FAIL`，防止被替换的 health 脚本污染长稳 CSV 固定列数 |
| 本轮证据链补强续 | v3.07 收紧板端长稳 health_report 路径行完整性：`runtime_stability_check.sh` 每次采样必须继续拒绝 `runtime_health.sh` 返回路径中的换行和回车，改写为 `missing`、累加 `health_failures` 并使 summary `overall=FAIL`，防止被替换的 health 脚本把单条长稳采样拆成多行 |
| 本轮证据链补强续 | v3.08 收紧板端长稳 health_report 路径证据文本安全性：`runtime_stability_check.sh` 每次采样必须继续拒绝 `runtime_health.sh` 返回路径中的分号，改写为 `missing`、累加 `health_failures` 并使 summary `overall=FAIL`，防止被替换的 health 脚本把证据路径污染为分号键值片段 |
| 本轮证据链补强续 | v3.09 收紧 session 侧长稳归档新鲜度：`capture_runtime_stability.sh` 复制板端 CSV/summary 前必须要求当前 session 的 `runtime_stability_run.log` 存在，且 `runtime_dir/samples/interval/exit_status=0` 与本次配置一致；若 run log 已存在 `capture_exit_status`，则该字段也必须为字面量 `0`；若缺失该字段，则只能由 `run_runtime_stability.sh` 通过带一次性 token 的内部 marker 触发首次归档且 run log 尚无 `finished_at`；缺失、不匹配、standalone 残缺 run log、已完成但缺捕获退出码、既有捕获失败、stale marker 授权失败、run log 或 marker 结构污染时写入 FAIL 占位 summary 并非零退出，防止 `validate_evidence.sh` 误归档板端旧长稳结果 |
| 本轮证据链补强续 | v3.10 收紧最终证据包长稳触发绑定：`record_session.sh` 的 evidence manifest 必须声明 `runtime_stability_run_log=logs/runtime_stability_run.log`；`lio_eval_tools` 必须把该文件作为必需证据，并要求其中 `started_at/finished_at` 为 ISO-8601 seconds、`runtime_dir` 与 manifest 有效运行目录一致、`samples/interval` 与长稳 summary 一致且 `exit_status=0`、`capture_exit_status=0`，缺失或不匹配时 `runtime_stability_status/evidence_status` 保持 FAIL |
| 本轮证据链补强续 | v3.11 收紧 field acceptance 长稳触发下沉证据：`capture_field_acceptance.sh` 必须读取当前 session 的 `logs/runtime_stability_run.log`，要求 `started_at/finished_at` 为 ISO-8601 seconds、`runtime_dir` 与当前 session 运行目录一致、`samples/interval` 与 summary 一致且 `exit_status=0`、`capture_exit_status=0`，并在最终报告下沉 `runtime_stability_run_log_status` 和明细字段；`lio_eval_tools` 必须继续要求 field acceptance 报告内这些下沉字段与独立 run log/summary/manifest 一致，缺失或不匹配时 `field_acceptance_status/evidence_status` 保持 FAIL |
| 本轮证据链补强续 | v3.12 收紧 field acceptance 与独立 run log 的精确绑定：`lio_eval_tools` 必须在独立 `runtime_stability_run.log` 通过后保留 `started_at/finished_at/runtime_dir/samples/interval/exit_status/capture_exit_status` 原始字段，并要求 `field_acceptance_report.txt` 下沉的对应 `runtime_stability_run_log_*` 字段逐项一致；仅格式合法但时间戳或其它字段来自另一份 run log 时，`field_acceptance_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.13 绑定 field acceptance 的 section export 来源路径证据：`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须下沉 `section_export_report`，`lio_eval_tools` 必须要求该字段解析后与 evidence manifest 的独立 `section_export` 路径一致；最终报告只写 `section_export_status=PASS` 但不指明通过的 CSV 文件时，`field_acceptance_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.14 绑定 field acceptance 的 runtime health 来源路径证据：`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须下沉 `runtime_health_report`，`lio_eval_tools` 必须要求该字段解析后与 evidence manifest 的独立 `runtime_health` 快照路径一致；最终报告只写 `runtime_health_status=PASS` 和健康明细但不指明来源快照时，`field_acceptance_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.15 绑定 field acceptance 的 runtime deployment 来源路径证据：`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须下沉 `runtime_deployment_report`，`lio_eval_tools` 必须要求该字段解析后与 evidence manifest 的独立 `runtime_deployment` 检查文件路径一致；最终报告只写 `runtime_deployment_status=PASS` 和部署明细但不指明来源检查文件时，`field_acceptance_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.16 绑定 field acceptance 的 runtime stability 来源路径证据：`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须下沉 `runtime_stability_csv_report`、`runtime_stability_summary_report` 和 `runtime_stability_run_log_report`，`lio_eval_tools` 必须要求三者分别解析到 evidence manifest 的独立长稳 CSV、summary 和 run log 路径；最终报告只写长稳 PASS 状态和明细但不指明来源文件时，`field_acceptance_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.17 绑定 field acceptance 的 power-loss resume 来源路径证据：`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须下沉 `power_loss_resume_report`，`lio_eval_tools` 必须要求该字段解析后与 evidence manifest 的独立 `power_loss_resume` 证据文件路径一致；最终报告只写断电续建 PASS 状态、来源和恢复时间但不指明来源文件时，`field_acceptance_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.18 绑定 field acceptance 的 time sync 来源路径证据：`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须下沉 `time_sync_report`，`lio_eval_tools` 必须要求该字段解析后与 evidence manifest 的独立 `time_sync` 证据文件路径一致；最终报告只写 time sync PASS 状态、topic 和数值但不指明来源文件时，`field_acceptance_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.19 绑定 field acceptance 的 PPS/PTP wiring 来源路径证据：`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须下沉 `pps_ptp_wiring_report`，`lio_eval_tools` 必须要求该字段解析后与 evidence manifest 的独立 `pps_ptp_wiring` 证据文件路径一致；最终报告只写 wiring PASS 状态、time sync 下沉字段和人工审计字段但不指明来源文件时，`field_acceptance_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.20 绑定 field acceptance 的 metrics 状态下沉证据：`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须复验当前 session 的 `reports/validation_metrics_report.txt` 并下沉 `metrics_status=PASS/FAIL`，`lio_eval_tools` 必须要求最终验收报告自身包含 `metrics_status=PASS`；即使断电续建来源为 `manual_file` 且人工审计字段全部合法，只要 metrics 报告失败、缺失、重复 key、当前 session/scenario 记录不匹配或未在最终报告下沉 PASS 状态，`field_acceptance_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.21 绑定 evidence manifest 的 replay/HIL event_file 内容证据：`lio_eval_tools` 不得只检查 manifest 声明的 `event_file` 存在，必须解析规范化事件流、聚合 replay metrics，并要求聚合结果绑定当前 `session_id/scenario` 且默认指标评估 PASS；缺失当前场景、缺失当前会话、畸形时间戳或事件指标失败时，`event_file_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.22 同步 sample evidence bundle 到当前 gate：`sample_evidence_manifest.txt` 必须声明 `runtime_stability_run_log`，样例 replay event 必须绑定 manifest 的 `session_id/scenario`，PPS/PTP wiring 与 field acceptance 样例报告必须下沉当前交叉校验字段；仓库样例 launch 必须可输出 `event_file_status=PASS`、`field_acceptance_status=PASS` 和 `evidence_status=PASS` |
| 本轮证据链补强续 | v3.23 收紧 replay/HIL event_file 空壳事件证据：`lio_eval_tools` 的 evidence manifest 校验不得接受只有 `session_start`、没有任何指标字段或完整断电恢复事件对的事件流；至少必须包含静止漂移、长度误差、错回环、队列堆积、PPS 抖动等指标字段之一，或包含 `power_loss` 到 `recovered/recovery_complete` 的恢复事件对 |
| 本轮证据链补强续 | v3.24 统一 final field acceptance 的 replay/HIL event 入口：`capture_field_acceptance.sh` 必须校验并下沉 `event_file_status` 和 `event_file_report`；`lio_eval_tools` 必须要求独立 event_file 解析 PASS，且 field acceptance 报告内的 `event_file_report` 指向 evidence manifest 声明的同一事件文件。event_file 缺失、场景/会话不绑定、时间戳畸形、只有空壳事件或最终报告未下沉 event 状态时，`field_acceptance_status/evidence_status` 均必须保持 FAIL |
| 本轮证据链补强续 | v3.25 收紧 field acceptance 长稳时长下限：`FIELD_ACCEPTANCE_MIN_STABILITY_HOURS` 只能提高验收门槛，不得低于 24 h；低于 24 或格式非法时，`capture_field_acceptance.sh` 必须输出 `runtime_stability_min_duration_status=FAIL`，并使 `runtime_stability_status/field_acceptance_status` 保持 FAIL，防止用环境变量把 24 h 板端长稳降级成短跑样例 |
| 本轮证据链补强续 | v3.26 收紧最终证据包长稳门槛下沉字段校验：`lio_eval_tools` 必须要求 `field_acceptance_report.txt` 包含 `runtime_stability_min_duration_status=PASS` 和数值型 `runtime_stability_min_duration_h >= 24`，且 `runtime_stability_duration_h >= runtime_stability_min_duration_h`；缺字段、低于 24、状态 FAIL 或最终报告自称 PASS 但未下沉该门槛时，`field_acceptance_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.27 收紧 replay/HIL event_file 当前会话证据绑定：`lio_eval_tools` 必须只用当前 evidence manifest 的 `session_id/scenario` 事件判断是否存在真实指标证据并聚合 replay metrics；其它 session 或其它 scenario 的静止漂移、长度误差、错回环、队列堆积、PPS 抖动或断电恢复事件不得替当前会话补证据，否则 `event_file_status/field_acceptance_status/evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.28 收紧 field acceptance 生成侧 replay/HIL event_file 指标数值校验：`record_session.sh` 生成的 `capture_field_acceptance.sh` 在当前 `session_id/scenario` replay 事件中遇到 `static_drift_m`、`length_error_percent`、`chainage_m`、`reference_chainage_m` 或 `pps_jitter_ms` 时必须严格数值可解析，`wrong_loop` 和 `queue_backlog` 必须为严格整数，`chainage_m/reference_chainage_m` 必须成对出现；畸形值、`nan/inf`、尾随污染或缺少配对字段时，`event_file_status/field_acceptance_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.29 收紧 replay/HIL event_file 非负指标语义：`lio_eval_tools` replay 聚合器和 `record_session.sh` 生成的 `capture_field_acceptance.sh` 必须拒绝当前 `session_id/scenario` 事件中的负值 `static_drift_m`、`length_error_percent`、`wrong_loop`、`queue_backlog` 和 `pps_jitter_ms`；这些物理非负指标不得用负数抵消或压低最终静止漂移、长度误差、错回环、队列堆积或 PPS 抖动验收结果 |
| 本轮证据链补强续 | v3.30 收紧 replay/HIL event_file 时间戳非负语义：`lio_eval_tools` replay 聚合器和 evidence manifest 校验、`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须拒绝任一规范化事件记录中的 `t<0`；负事件时间不得参与 metrics 聚合、`event_file_status` 或 `field_acceptance_status` 的 PASS 判定 |
| 本轮证据链补强续 | v3.31 收紧 evidence manifest 侧 replay/HIL event_file 逐记录结构校验：`lio_eval_tools` 不得忽略非当前 `session_id/scenario` 的畸形事件记录；任一规范化事件记录缺失或污染 `event/session_id/scenario/t`、包含重复 key 或非法 session/scenario token 时，`event_file_status`、`field_acceptance_status` 与最终 `evidence_status` 必须保持 FAIL |
| 本轮证据链补强续 | v3.32 收紧 direct replay/HIL event_file 聚合入口：`validation_report_node` 显式使用 `event_file` 参数时，`lio_eval_tools` replay 聚合器也必须逐记录拒绝缺失或污染 `event/session_id/scenario/t`、重复 key、非法 session/scenario token 的规范化事件，不能只在 evidence manifest 入口执行结构校验 |
| 本轮算法补强 | v3.33 收紧 `slam_backend_manager` 参数化 Scan Context/ISC 描述子配置：几何/强度上下文的 `ring_edges` 必须有限、非负且严格递增，`sector_count` 必须为正；后端候选配置中的最小分数、Top1/Top2 比值、链距间隔和强度描述子权重必须为有限且在合理范围内，非法配置必须 fail closed，不得生成空洞描述子外观或误恢复全局候选 |
| 本轮算法补强 | v3.34 收紧 `tca_manager` TCA 上下文与锚点台账输入：高反标靶检测配置、上下文 ring 边界、ledger 匹配阈值、anchor 基础字段和 context signature 必须合法；非法配置、空 context、空 anchor id、非有限链距/中心或非法匹配阈值必须 fail closed，不得仅靠距离分数生成 TCA 匹配 |
| 本轮算法补强 | v3.35 收紧 `lio_local_odometry` 局部注册与子图质量门控：ICP/NDT/GICP 多尺度配置、阈值、迭代次数和点数门槛必须合法；非有限 fitness/observability、非法 gate 或非有限几何点必须 fail closed，不得把污染参数或 NaN 观测度晋升为可用注册/可晋升子图 |
| 本轮算法补强 | v3.36 收紧 `lio_session_manager` 断电恢复保守 gate：local/TCA/global 恢复分数和阈值必须为有限 `[0,1]` 且 local relocalize 不得高于 local resume；稳定图恢复锚点查询参数、ledger 条目、alignment 质量和 gate 阈值必须合法，任一 NaN/非法质量/空 keyframe/非法阈值都不得触发恢复锚点接受 |
| 本轮算法补强 | v3.37 收紧 `slam_backend_manager` 图优化输入：一维链距图和 6DoF 位姿图节点、约束、端点引用、权重、协方差/鲁棒核参数和 Ceres 配置必须合法；任一 NaN/Inf、空或重复 keyframe、未知端点、越界权重或非法 config 都必须 fail closed，不得让污染图进入优化或输出可被稳定图/恢复链误用的位姿 |
| 本轮算法补强 | v3.38 收紧 `lio_preprocess` IMU 去畸变输入：deskew 配置、cloud stamp、point time 和 IMU 角速度样本必须合法；非有限 IMU、非法 reference/max_abs_point_time/point_time_scale 或污染时间字段必须 fail closed，不得把 NaN 旋转写入 `/lio/points_deskewed` |
| 本轮算法补强 | v3.39 收紧 `lio_state_estimator` IMU 状态预测输入：滑窗统计、静止 bias/gravity 估计和短时预积分必须拒绝非有限 stamp/acc/gyro、非有限 bias 和非有限 gravity direction；污染样本只能下沉 `invalid_sample_count` 或 `preintegration_rejected_updates`，不得把 NaN position/velocity/angular_velocity/gravity_direction 写入 `/lio/state_predict` 或诊断 |
| 本轮算法补强 | v3.40 收紧 `machine_state_manager` PLC/工况输入：PLC 寄存器索引、状态 bit 和 track_speed_scale 必须合法；非有限或非正比例系数、负寄存器索引或非法 bit 必须 fail closed，不得发布 NaN 轨速；机器状态分类遇到非有限轨速/雷达速度/IMU 振动或非法阈值必须保守输出 `CONFLICT`，确保冻结策略不被污染输入绕过 |
| 本轮算法补强 | v3.41 收紧 `mapping_control` 映射策略输入：策略配置、请求增量、观测度、链距和工况状态必须合法；非有限 delta/chainage/observability、非法配置或未知工况必须 REJECT，A/B/C 截面评分不得接受负 RMSE 或非有限完整度，防止 NaN 控制量、错误稳定图写入或伪高质量截面进入后续链路 |
| 本轮算法补强 | v3.42 收紧 `section_manager` 截面提取、历史替换和 CSV 导出输入：slice/rectangle/chainage/points/quality/numeric fields 必须合法；非有限点、非法截面配置、负 RMSE、非法质量或污染 CSV 文本不得生成高质量截面、替换历史截面或导出成果 |
| 本轮算法补强 | v3.43 收紧 `lidar_fusion` 三雷达融合诊断与重叠区一致性输入：overlap pair distance、sync/TF/voxel/overlap 阈值必须有限且合法；诊断 payload 中的 sync span、RMSE/max 和 frame/status 文本不得输出 `nan/inf`、分号或换行污染，防止融合健康证据和后续 time/status 聚合被污染 |
| 本轮算法补强 | v3.44 收紧 `lio_time_manager` 时间同步观测入口：传感器时间、设备时间、host receipt 时间和 PPS event 时间必须为有限值；非有限样本不得进入频率、latency、clock offset、PPS interval/jitter 统计，防止 `/time/status` 和最终 time sync 证据输出 NaN/Inf |
| 本轮算法补强 | v3.45 收紧 `lio_time_manager` stale 判定时间输入：`status(now)` 遇到非有限 now 或非法 stale 阈值必须保守 stale，diagnostics 非有限 receipt time 不得标记为 received，防止 IMU/PLC/PPS time sync 健康证据被 NaN 状态时间误报 fresh |
| 本轮证据链补强续 | v3.46 收紧 `lio_eval_tools` 长稳零值证据文本：runtime stability summary 的 `disk_failures/watchdog_failures/health_failures`、run log 的 `exit_status/capture_exit_status` 和最终 field acceptance 下沉的三类长稳失败计数字段必须字面量等于 `0`；`00`、`000` 等前导零污染不得被数值归一化为通过 |
| 本轮证据链补强续 | v3.47 收紧 `lio_eval_tools` metrics 报告重复 key 证据语义：evidence manifest 校验 metrics report 时，summary 行、当前 session/scenario 匹配记录行和恢复时间明细行中的任一重复 key 都必须使 `metrics_status`、`power_loss_resume_status` 和 `field_acceptance_status` 保持 FAIL；重复的非关键文本字段不得因关键字段仍为 PASS 而被忽略 |
| 本轮证据链补强续 | v3.48 收紧 `lio_eval_tools` 行式证据重复 key 语义：time sync、PPS/PTP wiring、runtime health、runtime deployment、runtime stability summary、runtime stability run log、power-loss resume 和最终 field acceptance 的任一 `key=value` 记录不得包含重复 key；重复的非关键审计字段也必须使对应子 gate 和最终验收保持 FAIL |
| 本轮证据链补强续 | v3.49 收紧 `lio_eval_tools` evidence manifest 顶层重复 key 语义：manifest 分号键值记录中的任一重复 key 都必须下沉 `manifest_duplicate_keys=true` 并使最终 `evidence_status` 保持 FAIL；重复的非必需审计字段不得被解析器忽略后制造完整验收外观 |
| 本轮证据链补强续 | v3.50 收紧 `lio_eval_tools` evidence manifest 内容证据路径解析语义：manifest 中的 metrics/event/time sync/PPS-PTP/runtime/power-loss/section/field acceptance 内容证据只有在路径为合法 bundle 相对路径且文件存在时才允许参与子 gate 解析；非法绝对路径、逃逸路径或污染路径不得因外部文件内容自称 PASS 而输出对应子状态 PASS |
| 本轮证据链补强续 | v3.51 收紧 `record_session.sh` 生成的 field acceptance 长稳零值文本语义：`runtime_stability_summary.txt` 的 `disk_failures/watchdog_failures/health_failures` 和 `runtime_stability_run.log` 的 `exit_status/capture_exit_status` 必须字面量等于 `0`；`00/000` 等前导零污染不得让 `capture_field_acceptance.sh` 输出 `field_acceptance_status=PASS` |
| 本轮证据链补强续 | v3.52 收紧 `record_session.sh` 生成的 metrics 来源断电续建零值文本语义：`validation_metrics_report.txt` 的 summary `failed_records` 和当前 session/scenario 记录 `failed_checks` 必须字面量等于 `0`；`00/000` 等前导零污染不得让 `capture_power_loss_resume.sh` 或 `capture_field_acceptance.sh` 输出 PASS |
| 本轮证据链补强续 | v3.53 收紧 `record_session.sh` 生成的 metrics 来源断电续建重复 key 语义：`validation_metrics_report.txt` 中任一非空分号键值记录包含重复 key，即使是无关 session/scenario 或非关键 `operator` 字段，也不得让 `capture_power_loss_resume.sh` 或 `capture_field_acceptance.sh` 输出 PASS |
| 本轮证据链补强续 | v3.54 收紧 `record_session.sh` 生成的 metrics 恢复时间提取语义：当前 session/scenario 匹配记录块内若出现重复 key 或解析失败的分号明细行，`metrics_report_recovery_time()` 不得继续从后续 detail 行借出 `recovery_time_s`，必须让断电续建和最终 field acceptance 报告保留 `recovery_time_s=missing` |
| 本轮证据链补强续 | v3.55 收紧 `record_session.sh` 生成的 time sync 行式重复 key 语义：`logs/time_sync_status.txt` 中任一 key 重复，即使是 `operator` 等非关键审计字段，也不得让 `capture_pps_ptp_wiring.sh` 或 `capture_field_acceptance.sh` 输出 `time_sync_status=PASS`、`pps_ptp_wiring_verified=PASS` 或 `field_acceptance_status=PASS` |
| 本轮证据链补强续 | v3.56 收紧 `record_session.sh` 生成的行式证据聚合输入重复 key 语义：PPS/PTP 与 power-loss 人工确认源文件，以及 field acceptance 聚合读取的 runtime health、runtime deployment、runtime stability summary/run log、PPS/PTP verified 和 power-loss verified 文件中任一 key 重复，即使是 `operator` 等非关键审计字段，也必须阻断对应子 gate 和最终 `field_acceptance_status=PASS` |
| 本轮证据链补强续 | v3.57 收紧 `lio_eval_tools` evidence manifest 对 v3.56 下沉 keys 状态的最终复核：`field_acceptance_report.txt` 必须包含 `time_sync_keys_status/runtime_deployment_keys_status/runtime_health_keys_status/runtime_stability_summary_keys_status/runtime_stability_run_log_keys_status/power_loss_resume_keys_status/pps_ptp_wiring_keys_status=PASS`；PPS/PTP 独立报告必须包含 `wiring_confirmation_keys_status=PASS`；manual_file 断电续建独立报告必须包含 `power_loss_resume_confirmation_keys_status=PASS`，否则最终 `field_acceptance_status/evidence_status` 保持 FAIL |
| 本轮证据链补强续 | v3.58 收紧 PPS/PTP 独立证据的人工确认总字段复核：`lio_eval_tools` evidence manifest 必须要求 `pps_ptp_wiring_verified.txt` 同时包含 `wiring_confirmation=PASS`、`wiring_confirmation_overall=PASS` 和 `wiring_confirmation_keys_status=PASS`，缺失或 FAIL 都不得只靠下游字段自洽通过 PPS/PTP wiring、field acceptance 或最终 evidence gate |
| 本轮证据链补强续 | v3.59 收紧最终 field acceptance 对 PPS/PTP 人工确认总字段的复核：`capture_field_acceptance.sh` 必须从独立 `pps_ptp_wiring_verified.txt` 下沉 `wiring_confirmation_overall=PASS`，并把该字段纳入脚本侧最终 PASS gate；manifest 侧也必须复核最终报告自身包含该字段且为 PASS |
| 本轮证据链补强续 | v3.60 收紧最终 field acceptance 对 PPS/PTP 人工确认源 keys 状态的复核：`capture_field_acceptance.sh` 必须从独立 `pps_ptp_wiring_verified.txt` 下沉 `wiring_confirmation_keys_status=PASS`，并把该字段纳入脚本侧最终 PASS gate；manifest 侧也必须复核最终报告自身包含该字段且为 PASS |
| 本轮证据链补强续 | v3.61 收紧最终 field acceptance 对 PPS/PTP 人工确认状态字段的复核：`capture_field_acceptance.sh` 必须从独立 `pps_ptp_wiring_verified.txt` 下沉 `wiring_confirmation=PASS`，并把该字段纳入脚本侧最终 PASS gate；manifest 侧也必须复核最终报告自身包含该字段且为 PASS |
| 本轮证据链补强续 | v3.62 收紧最终 field acceptance 对 manual_file 断电续建人工确认 keys 状态的复核：`capture_field_acceptance.sh` 必须从独立 `power_loss_resume_verified.txt` 下沉 `power_loss_resume_confirmation_keys_status=PASS`，并在 `power_loss_resume_source=manual_file` 时纳入脚本侧最终 PASS gate；manifest 侧也必须复核最终报告自身包含该字段且为 PASS |
| 本轮算法补强续 | v3.63 收紧 `slam_backend_manager` 回环几何精验证 fail-closed 语义：几何摘要必须拒绝非有限点坐标，几何验证阈值必须为有限合法值；NaN/Inf 几何或非法阈值不得绕过 centroid/span 比较并生成 accepted 回环 |
| 本轮算法补强续 | v3.64 收紧 `lio_session_manager` 稳定锚点恢复精配准 fail-closed 语义：current recovery cloud 和 stable anchor cloud 必须全部为有限点云；任一 NaN/Inf 坐标不得被 ICP 过滤后继续生成 converged 恢复配准证据 |
| 本轮证据链补强续 | v3.65 收紧 `lio_session_manager` manifest WAL 回放恢复会话绑定：manifest 文件和 WAL snapshot 的 `session_id` 必须匹配正在恢复的 session，错 session 或非有限关键数值不得覆盖上一条有效 snapshot |
| 本轮算法补强续 | v3.66 收紧 `slam_backend_manager` 位姿图协方差 stddev 语义：一维链距和 6DoF 约束的 optional stddev 只允许 `-1` sentinel 或严格正有限值；`0` 和其它负值不得回退默认并参与优化 |
| 本轮算法补强续 | v3.67 收紧 `tca_manager` JSON 台账加载 fail-closed 语义：malformed ledger 或畸形 anchor 记录不得抛异常中断恢复链路；`load()` 必须清空并跳过非法输入，保持 TCA 匹配不可用而非崩溃 |
| 本轮算法补强续 | v3.68 收紧 `tca_manager` TCA 点云输入污染 gate：高反检测和上下文签名必须在聚类/计数前拒绝非有限 `x/y/z/intensity`，不得从 NaN/Inf 点云生成标靶候选、上下文签名或恢复锚点输入 |
| 本轮算法补强续 | v3.69 收紧 `tca_manager` TCA 强度物理语义：点云 `intensity` 与 `intensity_threshold` 必须为非负有限值；负强度或负阈值不得参与高反筛选、上下文签名或 TCA 恢复锚点候选生成 |
| 本轮算法补强续 | v3.70 收紧 `tca_manager` TCA 上下文有效性：context signature 必须至少包含一个正 ring 观测；空点云、无 ring 命中或全零台账上下文不得作为 TCA 匹配依据 |
| 本轮算法补强续 | v3.78 收紧 `lio_preprocess` 过滤点 tuple 结构 gate：距离过滤、近机体遮蔽和 NaN 统计前必须确认每个点至少包含 `x/y/z` 三个槽位，短 tuple 只能计为 dropped_nan 并丢弃，不得越界读取或进入 `/lio/points_deskewed` |
| 本轮算法补强续 | v3.79 收紧 `lio_preprocess` 过滤配置 fail-closed gate：`min_range/max_range` 必须为有限非负且 `max_range >= min_range`，启用 body crop 时 crop box 六个边界必须有限且各轴 `min <= max`，非法配置不得因比较结果为 false 而保留点云 |
| 本轮算法补强续 | v3.80 收紧 `lio_preprocess` PointCloud2 入口 fail-closed gate：`x/y/z` 必需字段必须唯一存在，所有字段必须可读且 offset 不越过 `point_step`，`row_step/data` 必须覆盖声明点云；重复字段、不可读字段或截断数据不得发布 `/lio/points_deskewed` |
| 本轮算法补强续 | v3.81 收紧 `lio_preprocess` PointCloud2 坐标写回契约：`x/y/z` 必需字段不仅要可读，还必须为 `FLOAT32/FLOAT64`，整型坐标字段不得通过入口校验后发布无法写回去畸变结果的 `/lio/points_deskewed` |
| 本轮算法补强续 | v3.82 收紧 `lio_preprocess` PointCloud2 辅助字段数值 gate：保留下来的点中任一辅助字段读出 NaN/Inf 时必须整帧拒绝，非有限 `intensity/time/ring` 等字段不得进入 `/lio/points_deskewed`、点时间匹配或后续强度上下文链路 |
| 本轮算法补强续 | v3.83 收紧 `lio_preprocess` PointCloud2 全字段名唯一 gate：除 `x/y/z` 重复继续返回 `duplicate_required_field` 外，任一辅助字段重复名必须以 `duplicate_field` 整帧拒绝，避免重复 `time/intensity/ring` 造成去畸变或强度上下文歧义 |
| 本轮算法补强续 | v3.84 收紧 `slam_backend_manager` Scan Context/ISC 描述子输入污染 gate：几何描述子遇到非有限 `x/y/z` 必须返回空描述子，强度上下文遇到非有限或负 `intensity` 必须返回空描述子，不得用被污染的 bins 参与全局候选恢复 |
| 本轮算法补强续 | v3.85 收紧 `slam_backend_manager` 全局候选输入污染 gate：候选选择必须拒绝非有限 keyframe 链距和负 descriptor bin，`descriptorSimilarity()` 遇到负 bin 必须返回 0，避免污染候选在链距间隔和 Top1/Top2 gate 中获得高分 |
| 本轮算法补强续 | v3.86 收紧 `slam_backend_manager` 轻量 ICP 精配准验证 gate：ICP 配置阈值必须有限且范围合法，输入点坐标必须全部有限，污染参数或点云不得被接受为全局候选恢复的精验证通过证据 |
| 本轮算法补强续 | v3.87 收紧 `slam_backend_manager` 稳定图晋升策略配置 gate：`min_stable_quality` 必须为 A/B/C，非法最低质量阈值不得退化为“全质量等级放行” |
| 本轮算法补强续 | v3.88 收紧 `slam_backend_manager` 稳定图 `keyframe_id` token gate：稳定图台账读取和晋升写入必须拒绝包含分号、路径段或换行污染的 `keyframe_id`，避免污染稳定锚点、诊断文本和断电恢复引用 |
| 本轮算法补强续 | v3.89 收紧 `slam_backend_manager` 位姿图 `keyframe_id` token gate：一维链距图和 6DoF 位姿图节点/约束端点必须拒绝分号、路径段、空白或换行污染的 `keyframe_id`，避免污染优化图、回环治理和稳定图晋升依据 |
| 本轮算法补强续 | v3.90 收紧 `slam_backend_manager` 全局候选 `keyframe_id` token gate：当前 keyframe ID 污染时不得选择候选，候选 keyframe ID 污染时必须跳过，避免污染 `/backend/loop_candidate` 和后续精验证/晋升链路 |
| 本轮算法补强续 | v3.91 收紧 `slam_backend_manager` 后端 submap PointCloud2 reader：节点读取 `/map/local_submap` 时必须拒绝截断 data、字段 offset 越界、重复字段名、非有限点和负/非有限强度，失败时诊断 WARN 且不得生成 keyframe |
| 本轮算法补强续 | v3.92 收紧 `lio_session_manager` 稳定锚点恢复 `keyframe_id` token gate：读取后端稳定图台账和直接决策稳定锚点时必须拒绝分号、路径段、空白或换行污染的锚点 ID，避免污染断电恢复引用和 WAL 记录 |
| 本轮算法补强续 | v3.93 收紧 `lio_session_manager` `session_id` 路径 token gate：创建、读取、WAL 追加、恢复和最新 session 扫描必须拒绝分号、路径段、空白或换行污染的 session ID，避免 session 目录、manifest 和 WAL 归属逃逸或污染 |
| 本轮算法补强续 | v3.94 收紧 `lio_session_manager` manifest `state` 枚举 gate：manifest 文件和 WAL snapshot 只能恢复 `ACTIVE/TEMP` 状态，污染状态必须跳过，避免污染 `/session/status`、恢复决策和 WAL 证据文本 |
| 本轮算法补强续 | v3.95 收紧 `lio_session_manager` manifest 写侧 gate：`writeManifest()` 和 `commitManifestSnapshot()` 写入前必须复验 session ID、state、时间、链距、稳定位姿和 WAL 序号合法，污染 manifest 不得覆盖 manifest.json 或追加 session.wal |
| 本轮算法补强续 | v3.96 收紧 `lio_session_manager` WAL record 单行 gate：`appendWal()` 必须在打开文件前拒绝换行/回车污染，防止单次追加拆成多条 WAL 记录并注入伪 `manifest_snapshot` |
| 本轮算法补强续 | v3.97 收紧 `lio_session_manager` WAL record JSON/event gate：`appendWal()` 只能追加可解析 JSON object，且 `event` 必须为 `manifest_snapshot` 或 `recover`，坏 JSON、缺 event、污染 event 或未知 event 不得进入 `session.wal` |
| 本轮算法补强续 | v3.98 收紧 `lio_session_manager` WAL record duplicate-key gate：`appendWal()` 必须拒绝同级 JSON 重复 key，包括重复 `event` 和嵌套 manifest 字段，避免不同解析器对同一 WAL 证据读出不同语义 |
| 本轮算法补强续 | v3.99 收紧 `lio_session_manager` manifest 直接读取 gate：`loadManifest()` 必须拒绝污染 state、错 session、非法数值、负 WAL 序号和 duplicate-key manifest，直接 API 读取与恢复路径使用同一合法性边界 |
| 本轮算法补强续 | v4.00 收紧 `lio_session_manager` WAL 回放 duplicate-key gate：`recoverManifest()` 回放磁盘 `session.wal` 时必须跳过同级 JSON 重复 key 的 snapshot，避免断电后污染 WAL 行覆盖更早合法 manifest |
| 本轮算法补强续 | v4.01 收紧 `lio_session_manager` snapshot_event token gate：`commitManifestSnapshot()` 写 WAL 前必须拒绝分号、换行、空白或路径段污染的 snapshot 事件名，避免污染事件文本进入恢复证据链 |
| 本轮算法补强续 | v4.02 收紧 `lio_session_manager` WAL payload semantic gate：`appendWal()` 写入前必须校验 `manifest_snapshot` 的 `snapshot_event/stamp/manifest` 与 `recover` 的 `stamp/base_action/action/stable_anchor_decision/stable_anchor` 语义，拒绝污染字段直接进入 WAL |
| 本轮算法补强续 | v4.03 收紧 `lio_session_manager` WAL evidence consistency gate：`manifest_snapshot.stamp` 必须与 `manifest.updated_at` 一致，`recover.action` 必须与 `base_action` 一致，只有稳定锚点 accepted PASS 时才允许提升为 `RECOVER_WITH_STABLE_ANCHOR` |
| 本轮算法补强续 | v4.04 收紧 `lio_session_manager` manifest time monotonic gate：`created_at` 必须非负，`updated_at` 必须不早于 `created_at`，写侧、读侧、WAL 入口和回放恢复共用该边界 |
| 本轮算法补强续 | v4.05 收紧 `lio_session_manager` tiered recovery input gate：`decideTieredRecovery()` 必须拒绝污染 manifest、时间倒置 manifest 和 `now < updated_at` 的未来 snapshot，保守进入 `CREATE_TEMP_SESSION` |
| 本轮证据链补强续 | v4.06 收紧 `lio_eval_tools` ISO 秒级证据时间戳日历 gate：field acceptance、PPS/PTP wiring、power-loss resume 和 runtime stability run log 等审计时间戳必须符合真实月份天数和闰年规则，不可能日期必须 fail closed |
| 本轮证据链补强续 | v4.07 收紧 `record_session.sh` 生成侧 ISO 秒级审计时间戳日历 gate：`capture_pps_ptp_wiring.sh`、`capture_power_loss_resume.sh` 和 `capture_field_acceptance.sh` 必须拒绝不可能日历日期，防止源头生成 PASS 外观证据 |
| 本轮证据链补强续 | v4.08 收紧 runtime stability run log 起止时间顺序 gate：`lio_eval_tools` 与 `capture_field_acceptance.sh` 必须要求 `finished_at >= started_at`，防止倒置时间生成长稳 PASS 外观证据 |
| 本轮证据链补强续 | v4.09 收紧 runtime stability run log 实际 elapsed gate：`lio_eval_tools` 与 `capture_field_acceptance.sh` 必须要求 `finished_at - started_at` 覆盖 `samples * interval_s` 的配置时长，并允许最多 `min(interval_s, 60 s)` 调度容差，防止短运行伪造 24 h 长稳 PASS 外观证据 |
| 本轮证据链补强续 | v4.10 收紧 runtime stability CSV 采样语义 gate：`runtime_stability.csv` 的 `sample` 必须从 1 连续递增，`timestamp` 必须为 ISO 秒级真实日历时间，脚本生成侧和 `lio_eval_tools` manifest 侧一致拒绝不可能日期、跳号或污染采样行 |
| 本轮证据链补强续 | v4.11 绑定 runtime stability CSV 与 run log 时间窗口：`runtime_stability.csv` 的每条采样 `timestamp` 必须落在独立 `runtime_stability_run.log` 的 `started_at/finished_at` 闭区间内，脚本生成侧和 `lio_eval_tools` manifest 侧一致拒绝跨时间窗口拼接的长稳证据 |
| 本轮证据链补强续 | v4.12 收紧 field acceptance 最终报告时间顺序 gate：最终 `field_acceptance_report.txt` 的 `timestamp` 必须不早于长稳 run log 完成时间、PPS/PTP 人工确认时间，且 manual_file 断电续建来源时还必须不早于 resume 人工确认时间；脚本生成侧下沉 `field_acceptance_timestamp_status` |
| 本轮证据链补强续 | v4.13 绑定 field acceptance 时间顺序状态下沉字段：`lio_eval_tools` manifest 侧必须要求最终报告显式包含 `field_acceptance_timestamp_status=PASS`，并继续独立复算最终报告时间不早于关键证据时间，防止手写最终报告省略或伪造该状态字段绕过证据链 |
| 本轮证据链补强续 | v4.14 绑定 field acceptance 长稳 run log 时长下沉字段：最终报告必须显式下沉 `runtime_stability_run_log_elapsed_s`、`runtime_stability_run_log_required_elapsed_s` 和 `runtime_stability_run_log_duration_status=PASS`，manifest 侧按 run log 起止时间、samples 和 interval 重算并逐项一致复核 |
| 本轮证据链补强续 | v4.15 绑定 field acceptance 长稳 summary 时长下沉字段：最终报告的 `runtime_stability_duration_h` 必须与 `runtime_stability_samples * runtime_stability_interval_s / 3600` 按脚本两位小数输出语义一致，防止手写最终报告把更长或不同配置的长稳证据改写成 24 h 外观 |
| 本轮证据链补强续 | v4.16 绑定独立 time sync 总状态证据：`logs/time_sync_status.txt` 必须显式包含 `time_sync_status=PASS`，`capture_pps_ptp_wiring.sh`、`capture_field_acceptance.sh` 和 `lio_eval_tools` manifest 侧都必须把该字段作为 PASS 必要条件，防止下游从子字段重新推导时间同步 PASS 外观 |
| 本轮证据链补强续 | v4.17 绑定 time sync 原始捕获证据：`logs/time_sync_status.txt` 必须包含指向当前 evidence bundle 内非空 raw YAML 的 `raw` 字段，manifest 侧必须拒绝缺失、空文件、非法路径或逃逸 bundle 的原始 `/time/status` 捕获物 |
| 样例验收 launch | `roslaunch lio_eval_tools validation_report.launch ...` 输出 `evidence_status=PASS`、`field_acceptance_status=PASS` |
| C++/roscpp 运行语言契约 | `mine_slam_bringup` 已接入 `ros_language_contract_test`，自动扫描 CMake/package/launch 中的 `rospy`、`catkin_install_python`、`catkin_add_nosetests` 和 `.py` launch 节点入口 |
| ROS 包解析 | `rospack find` 已确认核心新增包均可解析 |

当前真实未闭环项收敛为三类：完整 Scan Context++/ISC 现场验证或学习型全局描述子替换、硬件 PPS/PTP 接线实物确认、真实现场/HIL 验收证据闭环。实际 `Tunnel.bag` 已完成 LiDAR+IMU-only 初测软件回放、一键初测 suite PASS、suite manifest validation PASS、field acceptance gap audit、初测 readiness gate、field acceptance handoff gate、handoff bundle manifest validation、field acceptance collection plan validation、suite manifest 对 collection plan 的二次复验，以及 profile 推荐入口到 collection plan validator、后置 suite manifest revalidation、`suite_out` 目录绑定、推荐输出目录 profile-root 锚定、suite manifest 报告路径固定相对路径锚定、handoff bundle manifest 报告路径固定相对路径锚定、collection plan source provenance 路径锚定、suite manifest 对 collection plan source provenance 的上层复验、profile 推荐入口真实可执行行 gate、profile 推荐 suite 命令精确可执行行 gate、profile 推荐入口 `suite_out` 单一赋值 gate、profile 后置 gate 精确 suite 命令锚定、profile 推荐入口提前终止 gate、profile 推荐入口包裹式提前终止 gate、profile 推荐入口 strict mode 持续性 gate、profile 推荐入口包裹式 strict mode relaxation gate 和 PLC-present profile-only gate 闭环，可直接用于用户收集同类 bag 前的算法/回放开发闭环；但该 bag 无 PLC 反馈状态，只能证明前端实际数据入口、初测就绪、现场证据采集交接和采集计划完整性，不证明 PLC 状态机、section export 或最终现场验收。v5.10 已新增不依赖现场硬件的软件升级切片：`lidar_pointcloud` 抽出 GUJ120 包尾设备时间、azimuth wrap 插值、点级 offset 和 per-scan timing diagnostics；`lio_preprocess` 增加默认关闭的 IMU 线加速度平移去畸变补偿及诊断；`lio_local_odometry` 增加多尺度 voxel pyramid 预计算与重复 leaf 去重，降低 ICP/GICP 重复降采样开销；`section_manager` 增加默认关闭的 `arch` 拱形截面 RMSE 模型和配置入口。v5.11 已新增后端全局描述子 yaw-shift 唯一性软件门控：公开 best shift/second-best/Top1-Second ratio 诊断、ring key 辅助接口和默认关闭的 `min_rotation_uniqueness_ratio`，用于在现场参数冻结前先抑制对称长直片段的误候选。v5.12 已新增 `lidar_pointcloud/timoo_pointcloud` 标定 XML 必需字段 fail-closed 校验和脚本 gtest，防止缺字段标定 YAML 进入驱动。PLC feedback field validation、section export、PPS/PTP wiring、power-loss resume、runtime deployment、24 h runtime stability 和最终 field acceptance report 仍是剩余缺口。现场/HIL 闭环仍需补齐现场标靶样本、长直巷道 bag、PLC 反馈状态样本、PPS/PTP 硬件接线证据、断电续建回放、systemd/Docker 实机启停、24 h 板端长稳和现场外参/安装复核，并最终以 `field_acceptance_status=PASS` 作为统一证据入口。

下列内容不再列为“未完成算法级内容”，而是已完成可测试首版、仍需实测升级的工程模块：IMU/PLC Modbus 运行频率/RTT/失败诊断及时间管理聚合、统一 session 归档骨架、session 证据 manifest 与校验命令、time sync 证据捕获与校验、PPS/PTP wiring 证据生成脚本、field acceptance 最终验收 gate、实际 bag replay 入口与初始速度参考隔离、GUJ120 包尾设备时间解析、azimuth wrap 插值、点级 offset 模型和 converter timing diagnostics、IMU 连续时间去畸变、默认关闭的 IMU 线加速度平移补偿去畸变、多尺度 voxel ICP 局部注册、多尺度 voxel pyramid 预计算与重复 leaf 去重、NDT 粗配准 + ICP 精配准可选路径、GICP/Surfel 协方差感知注册可选路径、局部子图质量治理、局部注册几何退化评分与门控、TCA 锚点检测、全局候选恢复、ring key、best yaw shift/second-best 诊断、可配置旋转唯一性门控、几何包络精验证、轻量 ICP 精配准验证、参数化 Scan Context 风格几何描述子、参数化 ISC 风格强度上下文描述子、一维链距位姿图优化、可调 Ceres 非线性 6DoF 位姿图优化、协方差加权与 Huber 风格鲁棒核、unresolved 回环清除、稳定图台账、稳定图晋升、稳定图恢复锚点读取、稳定锚点多尺度 ICP 恢复精配准、manifest 快照 WAL 回放恢复、PPS/设备时间偏移估计、恢复配准质量门控、local/TCA/global 三级恢复策略和支持场景阈值、矩形/拱形截面 RMSE 模型、标定 XML 必需字段 fail-closed 校验、最终指标文件、规范化 replay/HIL event_file、验收证据包 manifest 校验、外参异常量级与 RPY 歧义角审计、融合点云 `sensor_id` 来源字段、融合诊断 topic、中心雷达到侧雷达重叠区 RMSE/max/status 诊断、板端 runtime 运维脚本。

## 2. 系统目标与验收口径

### 2.1 总体目标

开发一套适用于井下履带式掘进机的长期建图系统，实现三台 GUJ120 激光雷达与一台 IMU 的稳定融合，在掘进机长期静止截割、短时慢速移动、弱几何长直巷道和断电重启工况下，持续输出可维护的局部地图、稳定基线图、活动会话图和截面数据库。

### 2.2 核心目标

| 目标 | 说明 |
|---|---|
| 静止不漂 | 空闲静止和静止截割时，系统不得把振动、时间误差或外参微弹性解释成持续位移 |
| 截面优先 | 以断面轮廓完整度、边界清晰度、重复观测一致性为主要成果指标 |
| 长度受控 | 在长直弱几何段允许有限漂移，但必须有退化检测、冻结/限幅和控制锚点机制 |
| 断电续建 | 断电后原地或小位移重启时，系统应能快速恢复或保守新建会话 |
| 工程可复现 | 任何现场失败必须能通过 PCAP/rosbag/参数快照复现、定位和回归 |
| 可部署可维护 | 支持 ROS1 工程链路、板端在线最小集、日志滚动、异常降级和参数版本化 |

### 2.3 分阶段验收指标

| 阶段 | 重点 | 建议验收指标 |
|---|---|---|
| Alpha | 驱动、时间、状态机、中心雷达 + IMU 基线 | 静止待机不扩图；三雷达/IMU/PLC 数据可稳定录制回放；100 m 控制区长度误差 <= 1.0% |
| Beta | 三雷达连续时间融合、截面数据库、断电恢复 | 静止截割状态下位姿冻结；控制断面重复观测一致；断电 45 s 内恢复或安全降级；长度误差 <= 0.5% |
| Release | TCA、保守回环、板端长稳 | TCA 热点恢复成功率 > 98%；错回环验收集为 0；关键控制区长度误差 0.3%-0.5%；24 h 在线链路不堆积、不崩溃 |

建议具体门槛：

| 验收项 | 建议门槛 | 备注 |
|---|---:|---|
| 空闲静止漂移 | < 1 cm/h | `IDLE_STATIC` 下轨迹应基本固定 |
| 静止截割漂移 | < 2 cm/h | 允许质量降级，不允许持续扩图 |
| A 级结构截面 RMSE | <= 25 mm | 控制断面人工或高精参考比对 |
| A 级结构截面完整度 | >= 90% | 顶板、底板、侧壁按配置统计 |
| B 级结构截面 RMSE | <= 40 mm | 运动段或静止截割段可放宽 |
| 在线前端平均延迟 | < 80 ms | 不含低频后端 |
| 在线前端 P95 延迟 | < 100 ms | 10 Hz 雷达输入不应堆积 |
| 内存峰值 | < 6 GB | 预留系统、缓存和日志空间 |
| 数据保全 | 事件回溯完整 | 重启、失配、错环、时间跳变均可导出证据 |

## 3. 总体架构

系统按“两条链路、四类地图、一套状态机”组织。

两条链路：

1. 在线运行链路：传感器接入、时间管理、连续时间去畸变、三雷达融合、局部注册、IMU 增强松耦合、PLC/工况状态机、局部子图、截面生产、断电快速恢复。
2. 低频/离线治理链路：稳定图晋升、TCA 锚点管理、保守回环、会话合并、长期一致性优化、质量审计、成果导出。

四类地图：

| 地图 | 用途 | 写入策略 |
|---|---|---|
| 局部子图 | 在线配准、截面生产、近场恢复 | 由状态机控制，滚动维护 |
| 活动会话图 | 当前班次或当前重启后的暂存结果 | 先保守写入，验证后可晋升 |
| 稳定基线图 | 长期可复用参考地图 | 只接受已验证的稳定区、回环和锚点约束 |
| 截面数据库 | 观测截面、结构截面、质量等级、链距索引 | 与轨迹/地图解耦，按链距和会话版本维护 |

架构数据流：

```text
GUJ120 left/center/right
        -> timoo 或 tmlidar 驱动包族
        -> /lidar{1,2,3}/..._points
        -> lidar_fusion multi_lidar_fusion_node
        -> /points_raw 或 /lio/points_fused_raw
        -> 点级时间恢复 / 连续时间去畸变 / 质量赋权
        -> 几何优先局部注册
        -> 退化检测与弱方向冻结
        -> 局部子图 / 活动会话图 / 截面数据库

IMU Modbus/PPS
        -> imu_modbus_driver
        -> /imu_raw
        -> IMU bias/gravity 滑窗、短时预测、去畸变轨迹

PLC Modbus TCP 502
        -> /plc/left_track_speed, /plc/right_track_speed, /plc/cutting_on
        -> 工况状态机
        -> 写图策略、恢复策略、告警策略

活动会话图 + 稳定基线图 + TCA
        -> 断电重定位、保守回环、会话晋升
```

推荐 ROS 坐标系：

```text
map
  odom
    base_link
      lidar_center
      lidar_left
      lidar_right
      imu_link
```

工程约束：

1. `lidar_center` 或 `base_link` 必须在 P0 阶段冻结为三雷达融合参考坐标系。
2. `lidar_fusion` 不写死外参，外参由标定 YAML 和 `/tf_static` 维护。
3. IMU 原生坐标系必须显式适配到 ROS 使用坐标系，禁止在多个模块中重复隐式变换。
4. ROS 时间只作为消息分发和日志时间，不作为最终传感器采样时间；融合主时间应优先使用设备时间、PPS/硬件时间戳和离线时偏标定结果。
5. 在线 ROS 运行节点统一使用 C++/roscpp；launch 中不得接入 `rospy` 节点，Python 仅作为离线标定、审计、数据准备或测试辅助。

## 4. 统一接口规划

### 4.1 传感器输入接口

| 接口 | 生产者 | 传输 | 当前/建议 topic | 消息 | 关键字段 |
|---|---|---|---|---|---|
| 左雷达点云 | `timoo` 或 `tmlidar` 驱动包族 | UDP -> ROS | `/lidar1/timoo_points` 或 `/lidar1/lidar_points` | `sensor_msgs/PointCloud2` | `x,y,z,intensity,ring,time` |
| 中雷达点云 | `timoo` 或 `tmlidar` 驱动包族 | UDP -> ROS | `/lidar2/timoo_points` 或 `/lidar2/lidar_points` | `sensor_msgs/PointCloud2` | `x,y,z,intensity,ring,time` |
| 右雷达点云 | `timoo` 或 `tmlidar` 驱动包族 | UDP -> ROS | `/lidar3/timoo_points` 或 `/lidar3/lidar_points` | `sensor_msgs/PointCloud2` | `x,y,z,intensity,ring,time` |
| 雷达原始包 | 后续增强 | UDP 2368/8603 | `/sensors/guj120/*/packets` | 自定义 packet msg 或 PCAP | `sensor_id, device_ts, host_rx_ts, msop, difop` |
| IMU | `imu_modbus_driver` | Modbus TCP 502 | `/imu_modbus_node/imu_raw` 或重映射 `/sensors/imu/raw` | `sensor_msgs/Imu` | `stamp, acc, gyro, orientation, covariance` |
| PPS/事件 | `lio_time_manager` 已接入首版；后续增强硬件抓边 | GPIO/串口/采集卡 | `/time/pps_event` | `std_msgs/Header` | `stamp` 表示设备事件时间，接收时刻用于估计 host/device 偏移 |
| PLC | `machine_state_manager/plc_modbus_node` | Modbus TCP 502 | `/plc/left_track_speed`、`/plc/right_track_speed`、`/plc/cutting_on`、`/diagnostics/plc_modbus` | `std_msgs/Float64`、`std_msgs/Bool`、`diagnostic_msgs/DiagnosticArray` | `left_track,right_track,cutter,valid,latency` |

### 4.2 在线处理输出接口

| 接口 | 生产者 | Topic | 消息 | 用途 |
|---|---|---|---|---|
| 三雷达融合原始点云 | `lidar_fusion` | `/points_raw` | `PointCloud2` | FAST-LIO/LIO-SAM/自研前端输入 |
| 去畸变融合点云 | 预处理模块 | `/lio/points_deskewed` | `PointCloud2` | 几何注册、截面生产 |
| IMU 预测状态 | IMU 增强模块 | `/lio/state_predict` | 自定义或 `nav_msgs/Odometry` | 注册初值、健康监测 |
| 局部里程计 | 局部注册模块 | `/lio/odom_local` | `nav_msgs/Odometry` | 位姿、速度、协方差 |
| 退化状态 | 退化检测模块 | `/lio/degeneracy` | 自定义 | 弱方向、冻结/限幅策略 |
| 工况状态 | 状态机模块 | `/machine/state` | 自定义 | 写图策略、恢复策略 |
| 局部子图 | 地图模块 | `/map/local_submap` | `PointCloud2`/服务 | 在线配准、恢复 |
| 截面输出 | 截面模块 | `/section/observed`, `/section/structural` | 自定义/文件 | 截面产品 |
| 会话状态 | 会话模块 | `/session/status` | 自定义 | 稳定图/活动图状态 |

### 4.3 服务接口

| 服务 | 功能 |
|---|---|
| `/session/snapshot` | 主动生成会话快照、最后稳定位姿和 manifest |
| `/session/recover` | 断电后按三级流程重定位并恢复或新建会话 |
| `/section/export` | 按链距范围和最低质量等级导出当前节点截面历史 CSV，CSV 包含 `session_id,chainage_m,state_source,quality,completeness,rmse_mm,points`；会话版本过滤后续接入截面数据库持久化 |
| `/tca/register` | 注册或更新 TCA 锚点 |
| `/calib/audit_tf` | C++/roscpp 在线审计外参 YAML，返回必需 frame、TF 树、多父节点、循环、非有限数值、异常平移量级和 RPY 歧义角报告；重叠区残差已由三雷达融合诊断输出，现场标靶流程继续闭环阈值和验收样本 |
| `/diag/dump_event` | 按 session 归档 metadata 和事件时间窗生成事件证据目录、manifest、rosbag filter 命令和 PCAP 提取命令；真实过滤执行和地图切片文件后续由现场/回放流程触发 |

## 5. 模块化开发规划

角色定义：

| 角色 | 边界 |
|---|---|
| SYS | 驱动、时间、ROS、PLC、存储、部署、监控 |
| FE | 点云预处理、IMU 融合、局部注册、退化保护 |
| BE | 地图、截面、会话、断电恢复、回环、TCA 数据库 |
| QA | 数据采集、评测、仿真、回归、验收 |
| ME | 机械安装、共刚体结构、减振、标靶施工与台账 |

### 5.1 模块总表

| 模块 | 目标 | 功能描述 | 输入 | 输出 | 依赖 | 优先级 | 主责 |
|---|---|---|---|---|---|---|---|
| M01 传感器驱动与原始数据 | 打通 LiDAR/IMU/PLC 数据链路 | `timoo`/`tmlidar` 雷达驱动包族，`imu_modbus_driver` Modbus 驱动，PLC Modbus 客户端，PCAP/rosbag 双录制 | UDP、Modbus、PPS | 原始 topic、PCAP、bag | 无 | P0 | SYS |
| M02 时间管理 | 统一设备时间和融合时间 | 雷达包时间、点级 time、IMU/PPS、主机接收时间、时偏估计、时间健康监测 | M01 | `/time/status`、时偏参数 | M01 | P0 | SYS+FE |
| M03 外参与坐标管理 | 消除 TF 和坐标歧义 | 三雷达到中心/机体外参、IMU 坐标适配、TF 审计、版本化标定 YAML | 点云、IMU、标定数据 | `/tf_static`、`calib/*.yaml` | M01/M02 | P0 | FE+SYS+ME |
| M04 点云预处理与去畸变 | 输出可注册点云 | 点级时间恢复、连续时间 IMU 插值、粉尘/飞散物过滤、近机体遮蔽、质量赋权 | 三雷达点云、IMU、外参 | `/lio/points_deskewed` | M02/M03 | P0 | FE |
| M05 三雷达融合 | 统一三视角点云 | 复用并增强 `lidar_fusion`，按 TF 投到 `base_link/lidar_center`，保留 `ring/time/sensor_id`，诊断重叠残差 | 三路点云 | `/points_raw`、融合诊断 | M03 | P0 | SYS+FE |
| M06 IMU 增强松耦合状态估计 | 提供去畸变轨迹和注册初值 | 预积分、bias/gravity 滑窗、健康评分、短时预测；位姿主更新仍由几何注册决定 | IMU、局部里程计 | `/lio/state_predict` | M02/M04 | P0 | FE |
| M07 几何优先局部注册 | 输出主里程计 | 多尺度 voxel ICP 首版，NDT 粗配准 + ICP 精配准可选路径首版，GICP/Surfel 协方差感知注册可选路径首版，后续增强 BIEVR 风格高分辨率接口和重叠区一致性检查 | 去畸变点云、状态先验 | `/lio/odom_local`、残差 | M04/M06 | P0 | FE |
| M08 退化检测与弱方向保护 | 控制长度漂移 | Hessian/可观测性分析，纵向弱方向冻结/限幅，强度/TCA 条件增强 | 注册残差、局部图 | `/lio/degeneracy`、约束策略 | M07 | P0 | FE |
| M09 PLC 工况状态机 | 控制写图权限 | PLC Modbus TCP 读取左/右履带、截割电机和有效位，结合 LiDAR 位移、IMU 振动联合判断八态，输出写图/冻结/告警策略 | PLC、IMU、里程计 | `/plc/*`、`/machine/state`、`/diagnostics/plc_modbus` | M01/M06/M07 | P0 | SYS+FE |
| M10 局部子图管理 | 支撑配准和截面 | 滚动子图、质量标签、稳定/活动区标记、近场动态区剔除 | 点云、位姿、状态 | `/map/local_submap` | M07/M09 | P0 | BE |
| M11 截面数据库 | 生成核心成果 | 链距切片、观测截面、结构截面、质量 A/B/C、矩形/拱形配置、导出接口、同间距窗口内质量优先替换、截面 `session_id/state_source` 元数据；`section_manager.launch` 必须接收 `session_id`，fusion bringup 必须通过 `section_session_id` 透传，避免多会话成果混淆 | 局部子图、位姿、状态 | `/section/*`、`/section/export`、截面 CSV 文件 | M10 | P0 | BE+QA |
| M12 断电恢复与会话管理 | 原地重启续建且不污染图 | WAL/manifest、完整 manifest 快照 WAL 回放、最后稳定位姿、当前点云与稳定锚点多尺度 ICP 精配准、失败新建暂存会话、晋升策略 | 快照、当前点云、稳定图 | `/session/status`、`/session/recover` | M10/M11 | P0 | BE+SYS |
| M13 TCA 标靶锚点 | 提升热点和低特征段可靠性 | 反光圆柱检测、上下文 3-8 m 局部子图、站位台账、非唯一拒绝 | 点云、截面、台账 | `/tca/detection`、锚点约束 | M10/M12 | P1 | FE+BE+ME |
| M14 保守回环与稳定图治理 | 长期一致性维护 | 参数化 Scan Context++/ISC 风格候选、精验证、稳定区闭环、活动区暂不闭环 | 关键帧、会话图 | 稳定图优化结果 | M12/M13 | P1 | BE |
| M15 数据记录与评测 | 建立研发闭环 | 双录制、失败切片、事件证据目录、指标面板、参数回归、session 证据包 manifest 与校验命令、time sync/PPS-PTP wiring/power-loss resume/runtime health/section export/field acceptance 归档、日报/周报模板 | 全部 topic/文件 | 报表、事件切片命令、证据完整性报告、回归结果、时间同步证据、PPS/PTP 接线证据、断电续建证据、截面 CSV 成果证据、板端健康快照、最终现场验收报告 | M01/M17 | P0 | QA+SYS |
| M16 仿真与 HIL | 前置验证状态机和恢复 | Gazebo 功能仿真、PCAP/bag 回放、振动注入、断电重启测试 | 仿真模型、日志 | 仿真报告 | M15 | P1 | QA |
| M17 板端部署与运维 | 长时稳定运行 | Docker compose 构建/启动/停止/日志脚本、systemd 进程编排、日志滚动、可选 `taskset` CPU 绑定、看门狗、磁盘策略、板端健康快照、长稳周期采样汇总、fusion runtime 截面 session 传参 | 全系统 | 部署包、systemd 单元、Docker compose、健康快照日志、长稳 CSV/summary、运维手册 | 全部 P0 | P0 | SYS |

### 5.2 P0 模块开发步骤与验收

#### M01 传感器驱动与原始数据

开发步骤：

1. 固定雷达驱动族：现场若使用 timoo，默认三路 topic 为 `/lidar{1,2,3}/timoo_points`；若使用 tmlidar，默认三路 topic 为 `/lidar{1,2,3}/lidar_points`。
2. 确认所有点云字段包含 `x,y,z,intensity,ring,time`。缺少 `time/ring` 时，驱动层必须补齐，禁止由后端猜测。
3. 保留 `lidar_fusion/config/multi_lidar_fusion_timoo.yaml` 与 `multi_lidar_fusion_tmlidar.yaml` 两套配置。
4. IMU 使用 `catkin_ws/src/imu_modbus_driver` 作为基础，确认 502 端口、寄存器映射、发布频率、协方差和重连策略。
5. 在已落地 PLC Modbus 客户端基础上，按现场寄存器表复核左履带、右履带、截割电机、数据有效位和轮询延迟诊断。
6. 使用 `record_session.launch`/`record_session.sh` 建立统一 session 目录，保存 rosbag、PCAP 命令、metadata、ROS 参数、TF、topic/node 列表、日志、time sync、PPS/PTP wiring、power-loss resume、runtime health、runtime deployment、runtime stability、section export、field acceptance、`reports/` 和 `evidence_manifest.txt`；所有现场联调必须以同一 session_id、`scenario` 和 `runtime_dir` 归档，并通过 `time_status_topic`、`pps_topic` 明确时间同步诊断接口；归档目录必须生成 `commands/bringup_fusion_timoo_session.sh` 和 `commands/bringup_fusion_tmlidar_session.sh`，两者启动 fusion bringup 时必须将当前 `session_name` 作为 `section_session_id` 传入，保证正式采集的截面成果与证据包 session 一致；`commands/capture_section_export.sh` 必须通过 `/section/export` 或 `SECTION_EXPORT_SOURCE` 生成 `reports/section_export.csv`，服务输出和源文件输入都必须包含固定表头和至少一条数据行，且所有非空数据行都必须通过 session_id 一致性、字段数量与数值格式校验；长稳场景通过 `commands/run_runtime_stability.sh` 触发板端 `runtime_stability_check.sh` 后再归档结果，并保留纯 `key=value` 的 `logs/runtime_stability_run.log` 记录实际参数和退出状态，普通命令输出另存为 `logs/runtime_stability_command_output.log`；PPS/PTP 接线验证由 `commands/capture_pps_ptp_wiring.sh` 在 time sync PASS 且人工确认文件 `reports/pps_ptp_wiring_confirmation.txt` 存在后生成 `reports/pps_ptp_wiring_verified.txt`，最终验收要求确认文件总字段和 verified 文件总字段 `pps_ptp_wiring_verified` 均严格等于 `PASS`，且 `wiring_confirmation_source=manual_file`、`pps_wiring_verified=PASS`、`ptp_wiring_verified=PASS`、`wiring_verified_by` 和 `wiring_verified_at`；断电续建验证由 `commands/capture_power_loss_resume.sh` 基于 `validation_metrics_report.txt` 的 `recovery_time_s <= 45 s` 或包含实测 `recovery_time_s` 的 `reports/power_loss_resume_confirmation.txt` 生成为 `reports/power_loss_resume_verified.txt`，最终验收只接受 `power_loss_resume_source=metrics_report/manual_file`，其中 manual_file 确认总字段 `power_loss_resume_status` 必须严格等于 `PASS`，且必须满足恢复时间门槛；最终现场验收必须重新校验 time sync 报告中的 `pps_jitter_ms` 和 `mean_offset_ms` 数值合法性，并把两者写入 `field_acceptance_report.txt`；`PPS_PTP_WIRING_VERIFIED` 和 `POWER_LOSS_RESUME_VERIFIED` 环境变量只用于本地诊断，会被记录为 `manual_env` 且不能通过最终 gate；最后通过 `commands/validate_evidence.sh` 输出指标报告和证据完整性报告。

验收标准：

| 项目 | 标准 |
|---|---|
| 三雷达 topic | 三路均稳定 10 Hz 或配置频率，无空点云持续输出 |
| IMU topic | 实际输出频率稳定，长时无断连；断连后可重连 |
| PLC topic | 有效状态可读，轮询延迟有统计 |
| 数据回放 | 任一采集包可一键回放到融合节点 |
| 原始证据 | PCAP、bag、参数、TF、日志、time sync、PPS/PTP wiring、power-loss resume、runtime health、runtime deployment、runtime stability、section export、field acceptance、metrics/event 报告能按同一 session_id 和 scenario 归档；dry-run 可生成完整归档骨架、证据 manifest、time sync/PPS-PTP wiring/power-loss resume/runtime health/deployment/stability/section export/field acceptance 捕获命令、长稳采样触发命令和校验命令并回归测试；time sync 报告必须包含 `capture_status=CAPTURED`、`pps_status=PASS`、`clock_offset_status=PASS`、可解析数值 `pps_jitter_ms` 和 `mean_offset_ms`，其中 PPS jitter 必须非负；PPS/PTP 接线报告必须由 `capture_pps_ptp_wiring.sh` 基于 time sync PASS 和人工接线确认文件生成 `pps_ptp_wiring_verified=PASS`，确认文件总字段 `pps_ptp_wiring_verified` 必须严格等于 `PASS`，并包含可解析数值 jitter/offset、`wiring_confirmation_source=manual_file`、PPS 接线 PASS、PTP 接线 PASS、有效文本确认人和确认时间；截面导出证据必须生成 `reports/section_export.csv`，manifest 字段为 `section_export=reports/section_export.csv`，CSV 必须包含固定表头 `session_id,chainage_m,state_source,quality,completeness,rmse_mm,points` 和至少一条数据行，且所有非空数据行必须满足 `session_id` 与 evidence manifest 会话一致、字段非空、`state_source` 为正式工况枚举、`quality` 为 A/B/C、`chainage_m/completeness/rmse_mm` 可解析、`completeness` 位于 0-1、`rmse_mm` 非负、`points` 为正整数，校验报告必须输出 `section_export_status=PASS`；断电续建报告必须由 `capture_power_loss_resume.sh` 基于恢复指标或人工确认文件生成 `power_loss_resume_status=PASS`，人工确认文件总字段 `power_loss_resume_status` 必须严格等于 `PASS`，metrics 来源必须证明 `validation_metrics_report.txt` 首条非空汇总行 `overall=PASS`、`total_records` 为严格正整数、`failed_records=0` 且无重复 key，并且存在当前 session/scenario 匹配的 `status=PASS`、`failed_checks=0` 记录，同时要求 verified 文件中的 `metrics_report` 字段与当前 session metrics 报告路径一致，manual_file 来源必须要求确认文件总字段 `power_loss_resume_status` 严格等于 `PASS`，输出报告中的 `power_loss_resume_confirmation_overall` 也必须严格等于 `PASS`，并包含有效文本值 `resume_verified_by` 和 `resume_verified_at`，metrics 来源必须同时下沉非空 `metrics_report` 字段，且该字段解析后必须与 evidence manifest 的 `metrics_report` 路径一致，metrics/manual_file 来源都必须包含严格非负数 `recovery_time_s` 和严格正数 `max_recovery_time_s` 且满足恢复时间门槛，最终 gate 只接受 `metrics_report` 或 `manual_file` 来源；执行 `run_runtime_stability.sh` 后必须生成 session 侧纯 `key=value` 的 `runtime_stability_run.log`、`runtime_stability.csv` 和 `runtime_stability_summary.txt`，并将普通 stdout/stderr 隔离到 `runtime_stability_command_output.log`，manifest 必须声明 `runtime_stability_run_log`，且 run log 必须包含合法 `started_at/finished_at`、与 manifest 运行目录一致的 `runtime_dir`、与 summary 一致的 `samples/interval` 和 `exit_status=0`；CSV 必须包含固定表头和至少一条采样记录，且所有非空采样记录必须满足 `disk_guard_status=PASS`、`watchdog_status=PASS`、`health_report` 非空且不是 `missing` 或 `__DUPLICATE_KEY__`，且不含分号、换行或回车，summary 的 `overall` 字段必须严格等于 `PASS`，`samples` 必须为正整数并等于 CSV 非空采样记录数，`interval_s` 必须显式存在且为严格正整数，缺字段、非数字、0、小数或与 CSV 行数不一致均不得通过，且 `disk_failures/watchdog_failures/watchdog_skipped/health_failures` 必须显式存在并严格为 0；最终现场验收必须生成 `field_acceptance_report.txt`，其中 `field_acceptance_status=PASS` 依赖 time sync、实机 deployment、严格数值 `runtime_stability_duration_h >= 24` 的 24 h stability、断电续建和 PPS/PTP 接线验证全部 PASS，且报告自身必须下沉记录 `time_status_topic`、`pps_topic`、`time_capture_status`、`time_pps_status`、`time_clock_offset_status`、`pps_jitter_ms`、`mean_offset_ms`、`deployment_overall`、PPS 接线 PASS、PTP 接线 PASS、PPS/PTP 确认人/确认时间、断电续建确认总字段、确认人/确认时间、严格正整数 `runtime_stability_samples/runtime_stability_interval_s` 且 `runtime_stability_interval_s` 必须与 summary 一致、必须下沉 `runtime_stability_csv_status=PASS`、`runtime_stability_csv_samples` 与 `runtime_stability_sample_count_match=PASS`，以及为 0 的长稳 failure counters；runtime deployment 必须带有 `deployment_status=PASS`、`systemd_active_source=systemctl` 和 `docker_container_status_source=docker_inspect`，环境变量覆盖来源不得通过最终验收；PPS/PTP 和断电续建环境变量覆盖来源会被记录为 `manual_env`，不得通过最终验收；field acceptance 必须重新校验 time sync 总字段、topic 字段、子状态和值、runtime deployment 总字段 `deployment_status` 严格等于 `PASS`、PPS/PTP verified 文件总字段 `pps_ptp_wiring_verified` 严格等于 `PASS`、断电续建 verified 文件总字段 `power_loss_resume_status` 严格等于 `PASS`、metrics 首条非空汇总行 `overall=PASS`、`total_records` 为严格正整数、`failed_records=0`、无重复 key、当前 session/scenario 匹配 PASS 记录、metrics_report 路径绑定、manual_file 审计字段、恢复时间严格数值格式和 `recovery_time_s <= max_recovery_time_s`，metrics 失败、超时、缺字段、缺审计、重复 key、time sync topic/数值污染或格式污染即保持 FAIL；`validate_evidence.sh` 必须在报告前刷新 time sync/PPS-PTP wiring/power-loss resume/runtime health/deployment/stability/section export/field acceptance 证据 |

v1.71 补充约束：最终现场验收 gate 以 `field_acceptance_status=PASS` 为统一入口，且必须同时绑定 metrics、time sync、runtime health、runtime deployment、runtime stability CSV/summary、power-loss resume、PPS/PTP wiring 和 section export 八类独立证据；缺失 runtime health、section export 缺失/错会话、metrics 重复 key、任一上游证据 FAIL 或现场报告自称 PASS 但上游不一致时，`field_acceptance_status` 和 `evidence_status` 均必须保持 FAIL。

v1.72 补充约束：所有现场脚本中从 `key=value` 报告读取单值字段的 `line_value()` 必须拒绝重复 key；重复 key 应输出不可通过 PASS/数值校验的哨兵值，并使 PPS/PTP wiring、power-loss resume 或 field acceptance 对应 gate 保持 FAIL，禁止同一证据文件内用前写或后写覆盖制造通过状态。

v1.73 补充约束：`field_acceptance_report.txt` 自身必须下沉 `runtime_health_status=PASS` 和 `section_export_status=PASS`；缺字段或非 PASS 时，即使 evidence manifest 指向的 runtime health 快照与 section export CSV 独立校验通过，`field_acceptance_status` 和 `evidence_status` 仍必须保持 FAIL，防止最终验收报告与上游证据链脱节。

v1.74 补充约束：PPS/PTP 接线确认与断电续建人工确认的独立捕获入口必须直接覆盖重复总字段场景；`pps_ptp_wiring_confirmation.txt` 中重复 `pps_ptp_wiring_verified` 或 `power_loss_resume_confirmation.txt` 中重复 `power_loss_resume_status` 时，脚本必须下沉 `__DUPLICATE_KEY__` 哨兵并保持 `pps_ptp_wiring_verified=FAIL` 或 `power_loss_resume_status=FAIL`。

v1.75 补充约束：C++ 证据包校验器不得只按“非空”接受人工审计字段；`wiring_verified_by`、`wiring_verified_at`、`resume_verified_by`、`resume_verified_at` 必须拒绝 `missing` 和 `__DUPLICATE_KEY__`。该约束同时适用于 PPS/PTP wiring 独立证据、manual_file 断电续建独立证据和最终 `field_acceptance_report.txt`。

v1.76 补充约束：C++ 证据包校验器对关键文本字段必须拒绝 `missing` 和 `__DUPLICATE_KEY__`，不得只按非空判定。该规则已覆盖 time sync 的 `time_status_topic/pps_topic`，以及 runtime health 的 `runtime_dir/systemd_active/docker_container_status`，防止过期、缺失或重复 key 证据被误当作有效现场证据。

v1.77 补充约束：C++ 证据包校验器继续扩大同一哨兵值拒绝规则；runtime deployment 的 `runtime_dir` 和 runtime stability CSV 的 `health_report` 必须拒绝空值、`missing` 与 `__DUPLICATE_KEY__`。部署目录或长稳健康快照字段被污染时，即使 systemd/Docker 与长稳状态字段自称 PASS，对应独立 gate 和最终 `evidence_status` 仍必须保持 FAIL。

v1.78 补充约束：evidence manifest 顶层字段必须具备有效语义；`session_id`、`scenario` 和 manifest `runtime_dir` 不得为空、`missing` 或 `__DUPLICATE_KEY__`。顶层会话、场景或运行目录字段被污染时，即使 metrics、time sync、runtime health/deployment/stability、power-loss resume 和 field acceptance 文件级证据全部 PASS，最终 `evidence_status` 仍必须保持 FAIL。

v1.79 补充约束：evidence manifest 中所有证据文件路径字段必须拒绝空值、`missing` 和 `__DUPLICATE_KEY__`。即使磁盘上存在名为 `missing` 或 `__DUPLICATE_KEY__` 的文件，校验器也必须按缺失证据处理，禁止重复 key 或占位路径绕过文件级证据完整性检查。

v1.80 补充约束：`record_session.sh` 作为 evidence manifest 生成入口，必须在创建 session 目录前拒绝会污染 manifest 顶层语义的参数；`session_id`、`scenario` 和非空 `runtime_dir` 不得为 `missing`、`__DUPLICATE_KEY__`，且不得包含分号、换行或回车。生成侧提前失败，校验侧继续保守 FAIL，两者共同防止污染归档进入验收链。

v1.81 补充约束：`runtime_ops.sh` 作为板端长期运行骨架生成入口，必须在创建 runtime 目录、写入 `runtime.env` 和组装 `section_session_id:=...` roslaunch 参数前拒绝会污染 runtime metadata 或截面 session 归属的参数；`runtime_name` 和非空 `section_session_id` 不得为 `missing`、`__DUPLICATE_KEY__`，且不得包含分号、换行或回车。长期 runtime 的 fusion 启动可继续默认以 `runtime_name` 作为 `section_session_id`，但该默认值同样必须通过生成侧校验。

v1.82 补充约束：C++ 证据包校验器的统一文本字段校验必须拒绝 metadata 分隔符；所有通过 `validEvidenceTextValue()` 判定的值，除不得为空、`missing` 或 `__DUPLICATE_KEY__` 外，还不得包含分号、换行或回车。该规则覆盖 evidence manifest 顶层 `session_id/scenario/runtime_dir`、证据文件路径字段、time sync topic、runtime health/deployment 文本状态、长稳 CSV `health_report` 和人工审计字段，禁止直接伪造污染 manifest 绕过生成侧校验。

v1.83 补充约束：截面成果证据不得再作为可选补充项处理；`section_export` 必须作为 evidence manifest 的必需文件路径字段参与证据完整性检查。缺少该字段、字段为空、路径无效、CSV 文件不存在、CSV 表头/数据/会话/状态/质量/数值任一校验失败时，最终 `field_acceptance_status` 和 `evidence_status` 必须保持 FAIL，即使 `field_acceptance_report.txt` 自称 `section_export_status=PASS`。

v1.84 补充约束：evidence manifest 声明的所有必需证据文件必须是非空 regular file；仅存在但大小为 0 的 bag、PCAP、TF 快照、参数快照、运行日志、metrics/event 报告、time sync、PPS/PTP wiring、runtime health/deployment/stability、power-loss resume、section export 或 field acceptance 文件，均必须按缺失证据处理并输出对应 `missing_file=...`，最终 `evidence_status` 必须保持 FAIL。

v1.85 补充约束：最终现场验收报告必须直接下沉 runtime deployment 原始总字段 `deployment_status=PASS`，不能只输出 `deployment_overall=PASS` 或 `runtime_deployment_status=PASS`。`capture_field_acceptance.sh` 生成的 `field_acceptance_report.txt` 必须同时包含 `runtime_deployment_status=PASS`、`deployment_overall=PASS` 和 `deployment_status=PASS`；C++ 证据包校验器必须在最终 gate 中重验 `deployment_status=PASS`，缺字段、重复 key、非 PASS 或只由其它字段间接表达时，`field_acceptance_status` 和 `evidence_status` 均必须保持 FAIL。

v1.86 补充约束：`capture_field_acceptance.sh` 必须在最终现场验收生成侧重新复核 session 侧 `runtime_stability.csv` 的每条非空采样记录；固定表头之外，`disk_guard_status` 和 `watchdog_status` 必须严格为 `PASS`，`health_report` 不得为空、`missing` 或 `__DUPLICATE_KEY__`。任一行不满足时，即使 summary 自称 `overall=PASS`，也必须输出 `runtime_stability_csv_status=FAIL`、`runtime_stability_status=FAIL` 和 `field_acceptance_status=FAIL`。

v1.87 补充约束：`capture_field_acceptance.sh` 生成最终现场验收报告时，必须对 `runtime_health_latest.txt` 中的关键文本字段执行与 C++ 证据包一致的非空/非哨兵/无 metadata 分隔符校验；`runtime_dir`、`systemd_active` 或 `docker_container_status` 为空、`missing`、`__DUPLICATE_KEY__` 或包含分号时，即使磁盘余量和 PID 数值合法，也必须保持 `runtime_health_status=FAIL` 和 `field_acceptance_status=FAIL`，禁止生成侧自称运行健康 PASS。

v1.88 补充约束：`capture_field_acceptance.sh` 生成最终现场验收报告时，必须对 `runtime_deployment_check.txt` 执行与 runtime deployment 独立证据一致的部署骨架复核；`runtime_dir` 必须是有效文本值，`systemd_unit_file/systemd_env_file/docker_compose_file/docker_env_file/start_command/runtime_process_status/deployment_status` 必须全部 PASS，且 systemd active 必须来自 `systemctl`、Docker running 必须来自 `docker_inspect`。缺少任一骨架字段、启动命令、运行态总字段或来源字段时，`runtime_deployment_status` 和 `field_acceptance_status` 均必须保持 FAIL。

v1.89 补充约束：`capture_field_acceptance.sh` 生成最终现场验收报告时，必须对 PPS/PTP 接线确认和 manual_file 断电续建确认中的人工审计字段执行与 C++ 证据包一致的有效文本校验；`wiring_verified_by/wiring_verified_at/resume_verified_by/resume_verified_at` 不得为空、`missing`、`__DUPLICATE_KEY__`，也不得包含分号、换行或回车。任一人工审计字段被污染时，即使接线、恢复时间和确认总字段均自称 PASS，`pps_ptp_wiring_verified` 或 `power_loss_resume_status` 以及最终 `field_acceptance_status` 均必须保持 FAIL。

v1.90 补充约束：`capture_field_acceptance.sh` 生成最终现场验收报告时，runtime stability CSV 不得作为可跳过证据处理；`logs/runtime_stability.csv` 缺失或为空时必须输出 `runtime_stability_csv_status=FAIL`、`runtime_stability_csv_samples=missing`、`runtime_stability_sample_count_match=FAIL`，并使 `runtime_stability_status` 和 `field_acceptance_status` 保持 FAIL。只有 CSV 表头固定、至少一条非空采样记录、所有采样行通过，且 CSV 非空采样数与 summary `samples` 一致时，runtime stability 才能 PASS。

v1.91 补充约束：`capture_field_acceptance.sh` 生成最终现场验收报告时，runtime stability CSV 的 `health_report` 字段必须执行与 C++ 证据包一致的有效文本校验；`health_report` 不得为空、`missing`、`__DUPLICATE_KEY__`，也不得包含分号、换行或回车。任一采样行的 `health_report` 被 metadata 分隔符污染时，必须输出 `runtime_stability_csv_status=FAIL`，并使 `runtime_stability_status` 和 `field_acceptance_status` 保持 FAIL。

v1.92 补充约束：`capture_pps_ptp_wiring.sh` 和 `capture_power_loss_resume.sh` 作为独立人工确认入口，也必须对审计字段执行与 C++ 证据包一致的有效文本校验；`wiring_verified_by/wiring_verified_at/resume_verified_by/resume_verified_at` 不得为空、`missing`、`__DUPLICATE_KEY__`，也不得包含分号、换行或回车。任一审计字段被 metadata 分隔符污染时，对应独立报告必须保持 `pps_ptp_wiring_verified=FAIL` 或 `power_loss_resume_status=FAIL`，且脚本返回非零。

v1.93 补充约束：`capture_pps_ptp_wiring.sh` 作为 PPS/PTP 接线独立证据入口，必须对 time sync 数值字段执行与 C++ 证据包一致的数值校验；`pps_jitter_ms` 必须是非负数，`mean_offset_ms` 必须是合法数值。任一字段缺失、重复 key、非数字或 PPS jitter 为负时，`time_sync_status` 与 `pps_ptp_wiring_verified` 必须保持 FAIL，脚本返回非零。

v1.94 补充约束：`capture_time_sync.sh` 生成 time sync 证据时，不能只信任诊断块 `level=0`；若 PPS 诊断块为 OK，则 `interval_jitter_ms` 必须解析为非负数，否则 `pps_status` 必须回落 FAIL；若 clock offset 诊断块为 OK，则 `mean_offset_ms` 必须解析为合法数值，否则 `clock_offset_status` 必须回落 FAIL。任一 OK 诊断块携带非法数值时，脚本必须保留原始字段、写出 FAIL 状态并返回非零。

v1.95 补充约束：现场生成脚本中的 shell 数值校验必须与 C++ 证据包的 strict double 语义保持一致，不得因科学计数法、显式正号或小数点尾缀造成生成侧假阴性；`pps_jitter_ms/mean_offset_ms/recovery_time_s/max_recovery_time_s/runtime_stability_duration_h` 等证据数值字段必须先通过 strict double 格式，再按字段语义执行非负、正数或门槛比较。`capture_time_sync.sh` 从 YAML 诊断块提取 `level` 时，每个匹配 block 只能输出一个值，禁止 awk `END` 二次输出导致 `0\n0` 形式破坏状态判定。

v1.96 补充约束：现场生成脚本中的人工审计字段有效文本校验必须与 C++ 证据包 `validEvidenceTextValue()` 完全对齐；`wiring_verified_by/wiring_verified_at/resume_verified_by/resume_verified_at` 不得包含分号、换行或回车。`capture_pps_ptp_wiring.sh`、`capture_power_loss_resume.sh` 和 `capture_field_acceptance.sh` 读取到 CRLF 或嵌入控制换行的人工确认/verified 字段时，必须保持对应独立报告或最终 `field_acceptance_status` 为 FAIL，并返回非零。

v1.97 补充约束：`capture_time_sync.sh` 生成 time sync 证据时，运行时 `TIME_STATUS_TOPIC` 与 `PPS_TOPIC` 覆盖值必须执行与 C++ 证据包 `validEvidenceTextValue()` 一致的有效文本校验；topic 元数据不得为空、`missing`、`__DUPLICATE_KEY__`，也不得包含分号、换行或回车。任一 topic override 被污染时，脚本必须拒绝捕获输入、输出 `capture_status=INVALID_METADATA`、保持 `pps_status=FAIL` 和 `clock_offset_status=FAIL`，并返回非零。

v1.98 补充约束：板端 `runtime_ops.sh` 生成 runtime 目录、`runtime.env`、systemd/Docker 文件和长稳证据命令前，`--root` 必须执行与 `runtime_name`、`section_session_id` 一致的有效文本校验；root 不得为空、`missing`、`__DUPLICATE_KEY__`，也不得包含分号、换行或回车。任何会进入 `runtime_dir`、runtime health/deployment/stability 路径或 evidence manifest 文本字段的 runtime 根路径被污染时，脚本必须提前拒绝生成。

v1.99 补充约束：`record_session.sh` 的 `--prefix` 会进入 evidence manifest 的 `bag_file` 和 `pcap_file` 路径字段，必须执行与 manifest 顶层文本值和必需证据文件路径一致的有效文本校验；prefix 不得为空、`missing`、`__DUPLICATE_KEY__`，也不得包含分号、换行或回车。任一 artifact prefix 被污染时，脚本必须在创建 session 证据路径前拒绝执行。

v2.00 补充约束：`record_session.sh` 生成 `record_rosbag.sh`、`record_pcap.sh` 和 `metadata.env` 前，`--topics` 与 `--pcap-interface` 必须执行有效文本校验；两者不得为空、`missing`、`__DUPLICATE_KEY__`，也不得包含分号、换行或回车。topic 列表或 PCAP 网卡参数被 metadata 分隔符污染时，脚本必须提前拒绝生成可执行采集命令，防止 rosbag/tcpdump 命令和 session metadata 被污染。

v2.01 补充约束：`record_session.sh` 生成 `metadata.env` 和 `capture_time_sync.sh` 前，`--time-status-topic` 与 `--pps-topic` 必须执行与 C++ 证据包 `validEvidenceTextValue()` 一致的有效文本校验；两者不得为空、`missing`、`__DUPLICATE_KEY__`，也不得包含分号、换行或回车。time sync 或 PPS topic 被 metadata 分隔符污染时，脚本必须在创建 session 证据目录前拒绝执行，防止污染值进入时间同步归档和最终验收链路。

v2.02 补充约束：`runtime_ops.sh` 生成 `runtime.env`、`start_runtime.sh`、`watchdog_check.sh`、systemd/Docker 文件和长稳证据命令前，`--workspace`、`--launch-package`、`--launch-file`、`--watchdog-topic` 与可选 `--cpu-set` 必须执行与 runtime metadata 一致的有效文本校验；除允许 `cpu_set` 为空表示不绑定 CPU 外，上述字段不得为空、`missing`、`__DUPLICATE_KEY__`，也不得包含分号、换行或回车。任何会进入 source 路径、roslaunch 命令、watchdog topic 或 taskset 参数的 runtime 生成值被污染时，脚本必须提前拒绝生成。

v2.03 补充约束：`runtime_ops.sh` 生成 `runtime.env`、`disk_guard.sh`、`watchdog_check.sh`、systemd/Docker 文件和长稳证据命令前，`--min-free-gb`、`--log-retention-days` 与 `--watchdog-timeout` 必须是正整数。任一数值字段为空、包含分号/换行/回车、为非整数或为 0 时，脚本必须在创建 runtime 证据目录前拒绝执行，防止未加引号的 shell 数值位置被污染或生成不可执行的磁盘守护/看门狗命令。

v2.04 补充约束：`record_session.sh` 生成 `metadata.env` 和 `run_runtime_stability.sh` 前，`--runtime-stability-samples` 必须是正整数，`--runtime-stability-interval` 必须是非负整数。任一字段包含分号、换行、回车或非整数文本时，脚本必须在创建 session 证据目录前拒绝执行；`runtime-stability-interval=0` 仅允许作为测试/快速采样场景进入生成脚本，最终现场验收仍以长稳 summary 中严格正整数 interval 为准。

v2.05 补充约束：`record_session.sh` 生成 session 目录、`metadata.env` 和所有 session 侧命令前，`--root` 必须执行与 manifest 文本字段一致的有效文本校验；session root 不得为空、`missing`、`__DUPLICATE_KEY__`，也不得包含分号、换行或回车。任何会进入 `session_dir`、bag/pcap/snapshot/log/report/command 路径或 metadata 的 session 根路径被污染时，脚本必须提前拒绝生成，防止归档路径和证据环境被分隔符污染。

v2.06 补充约束：`record_session.sh` 生成 session 目录和 artifact 文件路径前，`--name`/`session_id` 与 `--prefix`/`artifact_prefix` 必须同时满足 manifest 文本有效性和单一路径段有效性。`session_id` 与 `artifact_prefix` 不得为 `.` 或 `..`，不得包含 `/` 或 `\`；任一字段试图通过 `../` 或反斜杠穿越归档目录时，脚本必须在创建 session 证据目录、bag/pcap 路径和 metadata 前拒绝执行。

v2.07 补充约束：`runtime_ops.sh` 生成 runtime 目录、`runtime.env`、systemd/Docker 部署骨架、健康/部署/长稳命令前，`--name`/`runtime_name` 必须同时满足 metadata 文本有效性和单一路径段有效性。`runtime_name` 不得为 `.` 或 `..`，不得包含 `/` 或 `\`；任一字段试图通过 `../` 或反斜杠穿越 runtime root 时，脚本必须提前拒绝执行，防止 logs/state/commands/systemd/docker 证据路径逃出预期运行根目录。

v2.08 补充约束：`runtime_ops.sh` 生成 `start_runtime.sh`、systemd `ExecStart` 和 Docker 启动封装前，所有会进入未引用 roslaunch 命令 token 的字段必须执行安全 token 校验。`runtime_name`、非空 `section_session_id`、`launch_package` 和 `launch_file` 仅允许 `[A-Za-z0-9_.-]` 字符；包含空格、`$`、反引号、括号、引号、管道、重定向或其他 shell 元字符时必须拒绝执行。fusion runtime 默认用 `runtime_name` 作为 `section_session_id`，因此 `runtime_name` 也必须满足该 roslaunch token 约束。

v2.09 补充约束：`record_session.sh` 生成 session 目录、`metadata.env`、evidence manifest、fusion 启动命令和 bag/pcap artifact 路径前，`--name`/`session_id` 与 `--prefix`/`artifact_prefix` 必须同时满足 manifest 文本有效性、单一路径段有效性和生成脚本 token 有效性。两者仅允许 `[A-Za-z0-9_.-]` 字符；包含空格、`$`、反引号、括号、引号、管道、重定向或其他 shell 元字符时必须拒绝执行，防止生成脚本在后续执行时发生命令替换或参数拆分。

v2.10 补充约束：`record_session.sh` 生成 `record_rosbag.sh`、`record_pcap.sh`、time sync/PPS 捕获脚本和最终验收命令前，所有会作为生成脚本文本字面量进入命令或变量赋值的字段必须拒绝 shell 元字符。`topics`、`pcap_interface`、`time_status_topic`、`pps_topic` 和可选 `runtime_dir` 不得包含 `$`、反引号、单双引号、括号、管道、后台符或重定向符；任一字段污染时必须提前拒绝生成，防止后续执行生成脚本时发生命令替换、参数拆分或重定向污染。

v2.13 补充约束：`record_session.sh` 生成 `record_rosbag.sh`、`record_pcap.sh`、time sync/PPS 捕获脚本和最终验收命令前，所有会作为生成脚本文本字面量进入命令或变量赋值的字段还必须拒绝 shell glob 字符。`topics`、`pcap_interface`、`time_status_topic`、`pps_topic` 和可选 `runtime_dir` 不得包含 `*`、`?`、`[` 或 `]`；任一字段污染时必须提前拒绝生成，防止未引用的 topic 列表或其它生成脚本文本在后续执行时按当前目录内容展开成路径参数。

v2.11 补充约束：`runtime_ops.sh` 生成 `start_runtime.sh`、`disk_guard.sh`、`watchdog_check.sh`、runtime health/deployment/stability 采样脚本、systemd 环境/服务文件和 Docker compose/env 文件前，所有会作为生成脚本文本字面量进入命令、变量赋值或部署文件路径的字段必须拒绝 shell 元字符。`runtime_root`、`workspace`、`watchdog_topic` 和可选 `cpu_set` 不得包含 `$`、反引号、单双引号、括号、管道、后台符或重定向符；任一字段污染时必须提前拒绝生成，防止板端脚本后续执行时发生命令替换、参数拆分或重定向污染。

v2.12 补充约束：`runtime_ops.sh` 的可选 `--cpu-set` 不能只按普通文本字段处理；非空时必须满足 `taskset -c` CPU 列表格式，即一个或多个十进制 CPU 编号或编号范围，范围/编号之间用逗号分隔，例如 `2`、`2-5`、`0,2-5`。`cpu_set` 为空表示不绑定 CPU；包含字母、空格、缺失编号、非法分隔符或不完整范围时必须提前拒绝生成，防止长期 runtime dry-run 计划生成成功但板端 `taskset` 启动失败。

v2.14 补充约束：`runtime_ops.sh` 的可选 `--cpu-set` 在通过格式校验后，还必须逐段校验 CPU 范围端点顺序；形如 `5-2` 的反向范围必须提前拒绝，`2-2`、`2-5` 和逗号组合范围仍允许。该约束确保 dry-run 生成的 `start_runtime.sh` 与板端 `taskset -c` 的实际接受语义一致，避免长稳部署阶段才暴露 CPU 亲和参数错误。

v2.15 补充约束：`runtime_ops.sh` 生成 `watchdog_check.sh` 前，`--watchdog-topic` 必须满足 ROS topic 名称格式，允许绝对或相对 topic，例如 `/time/status`、`/tf_static` 或 `diagnostics`，但不得包含空格、空段、非法起始字符或其它非 topic 名称字符。任一非法 topic 必须在 dry-run 生成 runtime 目录前拒绝，防止板端长稳阶段才发现 `rostopic echo -n1` 看门狗参数不可执行。

v2.16 补充约束：`record_session.sh` 生成 time sync、PPS/PTP wiring 和 field acceptance 捕获脚本前，`--time-status-topic` 与 `--pps-topic` 必须满足 ROS 单 topic 名称格式，允许绝对或相对 topic，但不得包含空格、空段、非法起始字符或其它非 topic 名称字符。任一非法单 topic 必须在生成 session 证据目录和捕获脚本前拒绝，防止后续 `rostopic echo` 捕获失败或把错误 topic 名称写入证据链。

v2.17 补充约束：`record_session.sh` 生成 `record_rosbag.sh` 和 session metadata 前，`--topics` 必须按空白拆分为一个或多个合法 ROS topic 名称；每个 topic token 都必须满足单 topic 名称格式。空白列表、非法起始字符、空段或其它非 topic 名称 token 必须提前拒绝，防止后续 `rosbag record` 因非法 topic 参数失败，或把不可执行的 topic 列表写入证据 manifest 和 session 命令。

v2.18 补充约束：`record_session.sh` 生成 `record_pcap.sh` 和 session metadata 前，`--pcap-interface` 必须满足 tcpdump 可用的接口名字符集，允许 `any`、`eth0`、`enp3s0`、`br-...`、`vlan.10` 等由字母、数字、下划线、点、冒号和短横线组成的接口名；包含空格或其它非法字符时必须提前拒绝，防止 session dry-run 生成成功但后续 `tcpdump -i` 无法启动。

v2.19 补充约束：`record_session.sh` 解析 `--start-pcap` 时，省略显式值仍表示启用 PCAP 捕获；若提供显式值，则只能是 `true`、`1`、`false` 或 `0`。其它文本值必须提前拒绝，防止现场以为已请求 PCAP 捕获，但 session dry-run 因非法布尔值被静默解释为关闭捕获。

v2.20 补充约束：`runtime_ops.sh` 生成 `docker-compose.yaml` 前，`--root`/`runtime_root` 与 `--workspace` 作为 Docker compose volume source 路径时不得包含冒号。冒号在 POSIX 路径中虽可作为普通字符，但会与 compose 短语法 `source:target[:mode]` 冲突；包含冒号时必须提前拒绝，防止 runtime dry-run 生成成功但 Docker 启动阶段才因卷挂载解析错误失败。

v2.21 补充约束：`runtime_ops.sh` 生成 runtime 目录、systemd service、Docker compose 文件和 `start_runtime.sh` 前，`--root`/`runtime_root` 与 `--workspace` 必须是绝对路径。相对路径必须提前拒绝，防止 systemd `ExecStart`、`WorkingDirectory`、Docker compose volume source 和 `source <workspace>/devel/setup.bash` 随调用目录变化而生成不可复现或不可启动的板端 runtime 计划。

v2.22 补充约束：`record_session.sh` 生成 session 目录、metadata、evidence manifest 和所有 session 侧命令前，`--root`/`session_root` 与非空 `--runtime-dir` 必须是绝对路径。相对路径必须提前拒绝，防止 `record_rosbag.sh`、`record_pcap.sh`、runtime health/deployment/stability 捕获命令和最终验收命令随执行目录变化而生成不可复现的证据链。

v2.23 补充约束：`record_session.sh` 生成 metadata、evidence manifest 和验收报告入口前，`--scenario` 必须是大写场景 token，仅允许大写字母、数字和下划线，例如 `STATIC_IDLE`、`POWER_LOSS_ORIGIN`、`LONG_STABILITY`、`TIME_SYNC` 或 `RUNTIME_DEPLOYMENT`。包含空格、小写字母或其它非法字符时必须提前拒绝，防止 session dry-run 生成成功但后续 scenario threshold 匹配不到预期场景。

v2.24 补充约束：`lio_eval_tools` 的 evidence manifest 最终 PASS gate 必须与 `record_session.sh` 生成端使用一致的 scenario token 语义。即便 manifest 不是由 `record_session.sh` 生成，`scenario` 也只能由大写字母、数字和下划线组成；包含空格、小写字母或其它非法字符时，metrics、time sync、PPS/PTP wiring、runtime health/deployment/stability、section export、power-loss resume 和 field acceptance 子证据即使全部 PASS，最终 `evidence_status` 也必须保持 FAIL。

v2.25 补充约束：`lio_eval_tools` 的 evidence manifest 最终 PASS gate 必须与 session 生成端的 `runtime_dir` 路径语义保持一致。manifest 顶层 `runtime_dir` 必须是非空、非哨兵值、无分号/换行/回车且以 `/` 开头的绝对路径；如果是相对路径，则即便全部证据文件存在且 metrics、time sync、PPS/PTP wiring、runtime health/deployment/stability、section export、power-loss resume 和 field acceptance 子证据全部 PASS，最终 `evidence_status` 也必须保持 FAIL，防止现场验收报告依赖调用目录解释运行目录。

v2.26 补充约束：`lio_eval_tools` 的 evidence manifest 中 `metrics_report`、`event_file`、`bag_file`、`pcap_file`、`tf_snapshot`、`params_snapshot`、`runtime_log`、`time_sync`、`pps_ptp_wiring`、`runtime_health`、`runtime_deployment`、`runtime_stability_csv`、`runtime_stability_summary`、`runtime_stability_run_log`、`power_loss_resume`、`section_export` 和 `field_acceptance` 等必需证据文件路径必须是证据包内的相对安全路径。路径不得为空、不得以 `/` 开头、不得包含反斜杠，且按 `/` 拆分后的任一路径段不得为空、`.` 或 `..`；如果 manifest 试图引用绝对路径或通过 `../` 逃逸到证据包外，即便外部文件存在且所有子证据语义均 PASS，最终 `evidence_status` 也必须保持 FAIL。

v2.27 补充约束：`lio_eval_tools` 的 evidence manifest 顶层 `session_id` 必须与 `record_session.sh` 生成端的安全 session token 语义保持一致。`session_id` 必须是非空、非哨兵值、无分号/换行/回车，不能是 `.` 或 `..`，且只能由 ASCII 字母、数字、下划线、点和短横线组成；包含空格、斜杠、反斜杠或其它非法字符时，即便 section export 中的 `session_id` 与 manifest 匹配且全部子证据均 PASS，最终 `evidence_status` 也必须保持 FAIL。

v2.28 补充约束：`lio_eval_tools` 的 `metrics_report` 证据不能只依赖首条汇总行 `overall=PASS`。报告首条非空汇总行必须满足 `overall=PASS`、`total_records` 为严格正整数且 `failed_records=0`，并且后续会话记录中必须至少存在一条 `session=<manifest session_id>`、`scenario=<manifest scenario>` 且 `status=PASS` 的记录；如果 metrics 报告仅包含汇总 PASS，或只包含其它 session/scenario 的 PASS 记录，最终 `metrics_status`、`field_acceptance_status` 和 `evidence_status` 都必须保持 FAIL。

v2.29 补充约束：`field_acceptance_report.txt` 不能只用 `field_acceptance_status=PASS` 和上游 PASS 字段表达最终验收；报告自身必须包含 `session_id=<manifest session_id>` 和 `scenario=<manifest scenario>`。`record_session.sh` 生成的 `commands/capture_field_acceptance.sh` 必须把当前 session 名称和场景写入最终现场验收报告；如果报告缺少这两个字段，或字段值与 evidence manifest 顶层 `session_id/scenario` 不一致，则即使 metrics、time sync、PPS/PTP wiring、runtime health/deployment/stability、section export 和 power-loss resume 子证据全部 PASS，最终 `field_acceptance_status` 与 `evidence_status` 仍必须保持 FAIL。

v2.30 补充约束：`runtime_health` 与 `runtime_deployment` 证据不能只证明某个运行目录健康或 active/running；当 evidence manifest 顶层 `runtime_dir` 是合法绝对路径时，两类报告中的 `runtime_dir` 必须与 manifest 顶层 `runtime_dir` 完全一致。`record_session.sh` 生成的 `commands/capture_field_acceptance.sh` 也必须按当前 session 的 `runtime_dir` 重新校验 runtime health/deployment 输入报告。若健康快照或部署检查来自其它运行目录，即使 systemd/Docker 状态均为 PASS，`runtime_health_status` 或 `runtime_deployment_status` 以及最终 `field_acceptance_status/evidence_status` 都必须保持 FAIL。

v2.31 补充约束：`field_acceptance_report.txt` 自身必须下沉并绑定 runtime health/deployment 的运行目录。最终报告中的 `runtime_health_runtime_dir` 和 `deployment_runtime_dir` 必须是有效文本值，且在 manifest 顶层 `runtime_dir` 是合法绝对路径时必须与其完全一致；缺字段、哨兵值、格式污染或指向其它运行目录时，即使独立 `runtime_health`/`runtime_deployment` 文件均 PASS，最终 `field_acceptance_status` 与 `evidence_status` 仍必须保持 FAIL。

v2.32 补充约束：`field_acceptance_report.txt` 自身下沉的 runtime health 明细必须与运行健康 PASS 语义一致。最终报告中的 `runtime_health_disk_available_gb` 必须是可解析非负数，`runtime_health_pid` 必须是严格正整数，`runtime_health_systemd_active` 和 `runtime_health_docker_container_status` 必须是有效文本值；缺字段、非数字、0 PID、哨兵值或格式污染时，即使独立 `runtime_health` 文件 PASS，最终 `field_acceptance_status` 与 `evidence_status` 仍必须保持 FAIL。

v2.33 补充约束：`field_acceptance_report.txt` 自身下沉的 runtime deployment 骨架明细必须与部署 PASS 语义一致。最终报告中的 `systemd_unit_file`、`systemd_env_file`、`docker_compose_file`、`docker_env_file` 和 `start_command` 必须严格等于 `PASS`；缺字段、FAIL、哨兵值或格式污染时，即使独立 `runtime_deployment` 文件 PASS，最终 `field_acceptance_status` 与 `evidence_status` 仍必须保持 FAIL。

v2.34 补充约束：`field_acceptance_report.txt` 自身下沉的 runtime process 总状态必须与部署 PASS 语义一致。最终报告中的 `runtime_process_status` 必须严格等于 `PASS`；缺字段、FAIL、哨兵值或格式污染时，即使独立 `runtime_deployment` 文件 PASS，最终 `field_acceptance_status` 与 `evidence_status` 仍必须保持 FAIL。

v2.35 补充约束：`field_acceptance_report.txt` 自身必须包含有效生成时间字段。最终报告中的 `timestamp` 必须是有效文本值，不得为空、`missing`、`__DUPLICATE_KEY__`，也不得包含分号、换行或回车；缺字段或时间戳污染时，即使所有独立证据和其它最终报告字段均 PASS，最终 `field_acceptance_status` 与 `evidence_status` 仍必须保持 FAIL。

v2.36 补充约束：`field_acceptance_report.txt` 自身的 `timestamp` 必须严格符合 ISO-8601 seconds 格式：`YYYY-MM-DDTHH:MM:SSZ` 或 `YYYY-MM-DDTHH:MM:SS±HH:MM`，并满足基础日期、时间和时区数值范围。任意非空但非时间格式的文本、缺字段、哨兵值或格式污染均必须使最终 `field_acceptance_status` 与 `evidence_status` 保持 FAIL。

v2.37 补充约束：PPS/PTP wiring 独立证据中的 `wiring_verified_at`、manual_file 断电续建独立证据中的 `resume_verified_at`，以及最终 `field_acceptance_report.txt` 透传的 PPS/PTP `wiring_verified_at` 必须严格符合 ISO-8601 seconds 格式：`YYYY-MM-DDTHH:MM:SSZ` 或 `YYYY-MM-DDTHH:MM:SS±HH:MM`。任意非空但非时间格式的人工审计时间不得通过 `pps_ptp_wiring_status`、`power_loss_resume_status`、`field_acceptance_status` 或最终 `evidence_status`。

v2.38 补充约束：`record_session.sh` 生成的 `commands/capture_pps_ptp_wiring.sh`、`commands/capture_power_loss_resume.sh` 和 `commands/capture_field_acceptance.sh` 必须在脚本生成侧执行与 C++ 证据包校验器一致的人工审计时间格式 gate。PPS/PTP 人工确认文件或 verified 文件中的 `wiring_verified_at`、manual_file 断电续建确认文件或 verified 文件中的 `resume_verified_at` 若不是 ISO-8601 seconds 格式，即使其它人工确认字段均为 PASS，也必须使对应脚本输出 FAIL 并返回非零。

v2.39 补充约束：runtime stability summary 不能省略长稳 failure counters。`disk_failures`、`watchdog_failures` 和 `health_failures` 必须在 `runtime_stability_summary.txt` 中显式存在且严格为 0；缺任一字段时，`record_session.sh` 生成的 `commands/capture_field_acceptance.sh` 必须输出 `runtime_stability_status=FAIL` 并保留对应 `runtime_stability_*_failures=missing`，`lio_eval_tools` 的 evidence manifest 校验也必须使 `runtime_stability_status`、`field_acceptance_status` 和最终 `evidence_status` 保持 FAIL。

v2.40 补充约束：runtime stability summary 不能省略长稳采样间隔。`interval_s` 必须在 `runtime_stability_summary.txt` 中显式存在且为严格正整数；缺字段、0、小数、非数字或哨兵值必须使 `runtime_stability_status`、`field_acceptance_status` 和最终 `evidence_status` 保持 FAIL。最终 `field_acceptance_report.txt` 下沉的 `runtime_stability_interval_s` 必须与 summary 中的 `interval_s` 完全一致，缺失 `samples/interval_s` 时 `record_session.sh` 生成的 `commands/capture_field_acceptance.sh` 必须保留对应 `runtime_stability_samples=missing` 或 `runtime_stability_interval_s=missing`，不得默认补成 0。

v2.41 补充约束：metrics_report 来源的 `power_loss_resume_verified.txt` 必须显式包含有效 `metrics_report` 字段，并与当前 session 的 `reports/validation_metrics_report.txt` 绝对路径一致。缺字段、哨兵值或指向其它 metrics 文件时，`record_session.sh` 生成的 `commands/capture_field_acceptance.sh` 必须使 `power_loss_resume_status` 与 `field_acceptance_status` 保持 FAIL。

v2.42 补充约束：`record_session.sh` 生成的 `commands/capture_power_loss_resume.sh` 与 `commands/capture_field_acceptance.sh` 对 metrics 来源断电续建执行脚本级 gate 时，不得只依据首条 summary PASS。metrics 报告还必须包含 `session=<当前 session>`、`scenario=<当前 scenario>`、`status=PASS`、`failed_checks=0` 的匹配记录；summary-only、错 session 或错 scenario 的 metrics 报告必须使 `power_loss_resume_status` 与 `field_acceptance_status` 保持 FAIL。

v2.43 补充约束：metrics_report 来源的断电续建恢复时间必须绑定到当前 session/scenario 的匹配指标记录；`capture_power_loss_resume.sh` 与 `capture_field_acceptance.sh` 不得从其它 session 或 scenario 的 `recovery_time_s` 借值。若匹配记录缺失、恢复时间缺失/畸形/超时，即使其它记录包含可通过的恢复时间，`power_loss_resume_status` 与 `field_acceptance_status` 仍必须保持 FAIL。

v2.44 补充约束：`lio_eval_tools` 的 evidence manifest 校验必须执行与生成脚本一致的 metrics 恢复时间绑定。metrics_report 来源的 `power_loss_resume_verified.txt` 和最终 `field_acceptance_report.txt` 不能只靠自报 `recovery_time_s` 通过；校验器必须回读 manifest 指向的 metrics 报告，确认自报恢复时间与当前 session/scenario 匹配记录块中的 `recovery_time_s` 数值一致。其它记录中存在可通过恢复时间、但当前匹配记录缺失/畸形/超时/不一致时，`power_loss_resume_status`、`field_acceptance_status` 和 `evidence_status` 均必须保持 FAIL。

v2.45 补充约束：`lio_eval_tools` 的 `power_loss_resume_status` 独立校验必须把 metrics 报告自身 PASS 作为 metrics_report 来源的前置条件。即使 verified 文件或最终 field acceptance 报告自报 `power_loss_resume_status=PASS`、`recovery_time_s` 合法且当前记录块存在匹配恢复时间，只要 metrics summary 不是 `overall=PASS`、`total_records` 不是严格正整数、`failed_records` 不是严格 0、存在重复 key 或缺少当前 session/scenario 的 `status=PASS/failed_checks=0` 匹配记录，`power_loss_resume_status`、`field_acceptance_status` 和 `evidence_status` 都必须保持 FAIL。

v2.46 补充约束：`lio_eval_tools` 的 `pps_ptp_wiring_status` 独立校验必须绑定 evidence manifest 中独立 `time_sync` 证据的实际 PASS 结果。`pps_ptp_wiring_verified.txt` 内部自报 `time_sync_status=PASS`、PPS/clock offset PASS 和合法 jitter/offset 不足以单独通过；只要独立 `time_sync.txt` 捕获失败、PPS 诊断 FAIL、clock offset 诊断 FAIL 或数值字段污染，`pps_ptp_wiring_status`、`field_acceptance_status` 和 `evidence_status` 都必须保持 FAIL。

v2.47 补充约束：`record_session.sh` 生成的 `commands/capture_field_acceptance.sh` 必须执行与 evidence manifest 一致的 PPS/PTP wiring 与当前 time sync 绑定。即使 `reports/pps_ptp_wiring_verified.txt` 自报 `pps_ptp_wiring_verified=PASS` 且人工接线审计字段完整，只要当前 `logs/time_sync_status.txt` 未通过 `capture_status=CAPTURED`、`pps_status=PASS`、`clock_offset_status=PASS`、合法 `pps_jitter_ms/mean_offset_ms` 校验，最终 `field_acceptance_report.txt` 中的 `pps_ptp_wiring_verified` 和 `field_acceptance_status` 都必须保持 FAIL。

v2.48 补充约束：`record_session.sh` 生成的 `commands/capture_field_acceptance.sh` 的 time sync 子 gate 必须校验 `logs/time_sync_status.txt` 中的 `time_status_topic` 和 `pps_topic`。两者必须是有效文本值，不得缺失、为 `missing`、`__DUPLICATE_KEY__` 或包含分号/换行/回车；任一字段无效时，即使 `capture_status=CAPTURED`、`pps_status=PASS`、`clock_offset_status=PASS` 且数值字段合法，最终 `field_acceptance_report.txt` 中的 `time_sync_status`、`pps_ptp_wiring_verified` 和 `field_acceptance_status` 都必须保持 FAIL，并下沉 `time_status_topic/pps_topic` 的实际值或 `missing`。

v2.49 补充约束：`lio_eval_tools` 的 evidence manifest 校验必须对最终 `field_acceptance_report.txt` 执行与脚本侧一致的 time sync topic 下沉校验。最终报告自身必须包含有效 `time_status_topic` 和 `pps_topic`，不得只依赖独立 `time_sync.txt` 已通过；缺失、为 `missing`、`__DUPLICATE_KEY__` 或包含分号/换行/回车时，`field_acceptance_status` 和 `evidence_status` 必须保持 FAIL。

v2.50 补充约束：`lio_eval_tools` 的 metrics 文件解析和 replay event 聚合必须拒绝数值字段污染。`static_drift_m/length_error_percent/recovery_time_s/pps_jitter_ms` 等浮点字段必须完整解析且为有限值，`wrong_loop_count/queue_backlog_max/wrong_loop/queue_backlog` 等整数字段必须完整解析；尾随字符、`nan`、`inf`、畸形整数或不完整 `chainage_m/reference_chainage_m` 对必须使对应指标 FAIL，缺字段场景仍沿用既有 safe default。

v2.51 补充约束：`lio_eval_tools` 的 replay event 聚合中，参与恢复时间计算的 `power_loss`、`recovered` 和 `recovery_complete` 事件 `t` 字段必须完整解析且为有限值。若字段存在但畸形、带尾随污染或为 `nan/inf`，聚合结果必须使 `recovery_time_s` FAIL，不得按 `0.0` fallback 生成瞬时恢复或负恢复时间的误验收；缺少 `t` 的事件仍沿用既有 safe default。

v2.52 补充约束：`scenario_validation_thresholds.txt` 的缺省字段必须与当前运行时默认阈值合并，而不是回退到 `ValidationThresholds{}` 的内置默认值。若 `validation_thresholds.yaml` 或 ROS 参数把某项默认阈值调严，场景覆盖记录只写其它字段时，未写字段仍必须采用该运行时默认阈值；否则对应场景记录可能被误放行。

v2.53 补充约束：`scenario_validation_thresholds.txt` 中显式出现的阈值覆盖字段必须完整解析且为有限值，整数字段必须完整解析。字段存在但畸形、带尾随污染或为 `nan/inf` 时，不得回退到运行时默认值或内置默认值，必须使匹配场景的对应指标 fail closed。

v2.54 补充约束：`metrics_file` 和规范化 replay/HIL `event_file` 的每条分号键值记录不得包含重复 key。重复 key 解析后必须下沉不可通过的 `__DUPLICATE_KEY__` 语义；无论重复的是 `static_drift_m/pps_jitter_ms` 等数值字段，还是 `scenario/session_id/event` 等文本字段，都必须使直接指标记录或事件聚合结果 fail closed，不得以前写或后写覆盖制造 PASS。

v2.55 补充约束：`scenario_validation_thresholds.txt` 的每条分号键值阈值记录同样不得包含重复 key。重复阈值字段、重复 `scenario` 或其它重复字段都必须使场景阈值配置 fail closed；其中重复 `scenario` 不能被解析成不可匹配哨兵后静默跳过，否则可能让本应收紧的长直、断电恢复或错回环场景覆盖失效。

v2.56 补充约束：`record_session.sh` 生成的 `commands/capture_pps_ptp_wiring.sh`、`commands/capture_power_loss_resume.sh` 和 `commands/capture_field_acceptance.sh` 中，读取 `key=value` 证据行的 `line_value()` 必须按第一个 `=` 切分并对 key 执行首尾空白裁剪后再计数。裁剪后重复的 key 必须输出 `__DUPLICATE_KEY__` 并使对应 gate fail closed；value 不得被脚本裁剪后再判定，必须保留原始内容，使分号、换行、回车和哨兵值污染仍由后续校验显式拒绝。

v2.57 补充约束：`lio_eval_tools` 的 evidence manifest 校验器解析行式 `key=value` 证据和分号键值记录时，必须只对 key 执行首尾空白裁剪以完成字段匹配和重复 key 检测，value 必须保持原始内容进入后续 gate。人工审计字段、topic/path/session/scenario、状态枚举、ISO-8601 时间戳和其它有效文本字段中的回车、换行、分号、哨兵值或额外污染不得在解析阶段被 trim 掉后误判为 PASS；脚本侧和 C++ 侧必须保持这一 fail-closed 语义一致。

v2.58 补充约束：`lio_eval_tools` 的 evidence manifest 校验器中，所有通过 strict double 解析的证据字段，包括 `pps_jitter_ms`、`mean_offset_ms`、`disk_available_gb`、`recovery_time_s`、`max_recovery_time_s`、`runtime_stability_duration_h`、截面 `chainage_m/completeness/rmse_mm` 等，必须由数值解析完整消费整个 value。尾部回车、换行、空白、单位、注释或其它污染都必须使对应 evidence gate fail closed，不得通过 `trim(end)` 或类似逻辑被当作合法数值。

v2.59 补充约束：`lio_eval_tools` 的 evidence manifest 严格 double 解析不得接受前导空白。由于 C/C++ `strtod` 会默认跳过前导空白，校验器必须在调用前显式拒绝首字符空白，并继续要求完整消费 value；否则脚本侧正则已拒绝的 `pps_jitter_ms= 0.5`、`recovery_time_s= 12.0` 或截面数值前导空白可能在 C++ evidence gate 中被误判为 PASS。

v2.60 补充约束：`lio_eval_tools` 的 validation metrics 与 replay event 分号键值解析必须和 evidence manifest 保持同一 fail-closed 语义：只对 key 执行首尾空白裁剪，value 必须保持原始内容进入严格数值 gate。`static_drift_m`、`length_error_percent`、`recovery_time_s`、`pps_jitter_ms`、replay 事件时间戳 `t`、`wrong_loop_count`、`queue_backlog_max`、`wrong_loop` 和 `queue_backlog` 等 double/int 字段均不得接受前导空白、尾部回车、尾部 tab/空白、单位或注释；否则 replay/HIL 指标可能在报告层被误判为 PASS，再污染断电续建和最终现场验收 gate。

v2.61 补充约束：`lio_time_manager` 聚合 `/diagnostics/imu_modbus` 与 `/diagnostics/plc_modbus` 的数值字段时，double 字段必须拒绝空值、前导空白、尾部污染、`nan/inf` 和其它非有限值，int 字段必须拒绝空值、前导空白、尾部污染和超过 `int` 范围的数值。污染字段必须保持默认值，不得下沉到 `/time/status` 后被 `capture_time_sync.sh` 或 evidence manifest 当作真实传感器/PLC 健康证据。

v2.62 补充约束：`lio_eval_tools` 的 evidence manifest 中所有严格正整数字段，包括 metrics 汇总 `total_records`、runtime health `runtime_pid`、runtime stability `samples/interval_s/runtime_stability_csv_samples` 和 section export `points` 等，必须在调用 `strtol` 前清空 `errno` 并拒绝 `ERANGE`。超出 `long` 范围的超长数字不得被 `strtol` 饱和为 `LONG_MAX` 后当作合法正整数，否则断电续建和最终现场验收可能被伪造的超大 `total_records` 误放行。

v2.63 补充约束：`section_manager` 与 `slam_backend_manager` 从 `/mapping/control` 文本读取 `chainage_m` 时，必须使用分号键值解析和严格 double gate。字段名必须精确匹配，`last_chainage_m` 等后缀字段不得被误匹配为 `chainage_m`；重复 `chainage_m`、前导空白、尾部污染、`nan/inf`、非有限值或溢出必须回退到上一链距，不得依赖 `std::stod` 的部分解析或异常路径污染截面链距、后端 keyframe 链距和稳定图晋升证据。

v2.64 补充约束：`slam_backend_manager` 的稳定图 JSON 台账必须 fail closed。读取台账时，只有 `keyframe_id` 非空、`chainage_m` 存在且有限、`section_quality` 为 A/B/C、`promoted_at` 存在且有限、`loop_verified` 可解析的条目才能进入稳定锚点集合；缺字段、畸形字段、非有限数值或非法质量等级必须跳过。晋升写入时同样必须拒绝非法条目，且非法同名 keyframe 不得删除或覆盖已有合法稳定锚点，防止损坏台账污染断电恢复锚点和稳定图治理证据。

v2.65 补充约束：`section_manager` 与 `slam_backend_manager` 从 `/mapping/control` 文本读取 `section_sample`、`machine_state` 和 `quality` 时，必须使用分号键值解析和严格 bool/text gate。字段名必须精确匹配，`last_section_sample`、`last_machine_state`、`bad_quality` 等后缀字段不得被误匹配为目标字段；重复 key、空值、`missing`、`__DUPLICATE_KEY__`、前导空白、分号、换行或回车污染必须回退到默认值或上一状态，不得依赖 regex 子串匹配污染截面采样、工况来源、后端 keyframe 质量和稳定图晋升证据。

v2.66 补充约束：`record_session.sh` 生成的 `commands/capture_power_loss_resume.sh` 与 `commands/capture_field_acceptance.sh` 在解析 `validation_metrics_report.txt` 的分号键值 summary/record 和匹配记录块内 `key=value` 行时，必须只对 key 执行首尾空白裁剪，value 必须保持原始内容进入状态比较、正整数 gate、恢复时间数值 gate 和当前 session/scenario 匹配逻辑。`overall=PASS `、`total_records= 1`、`session= test_session`、`status=PASS ` 或 `recovery_time_s=25 ` 等污染值必须使 power-loss resume 与 field acceptance 保持 FAIL，不得被 AWK `gsub`/trim 后误判为 PASS。

v2.67 补充约束：section export CSV 的 7 个数据字段必须保持原始字段值进入校验，不得先 trim 后比较或数值解析。`capture_section_export.sh`、`capture_field_acceptance.sh` 与 `lio_eval_tools` evidence manifest 必须拒绝空字段和任何首尾空白污染；`test_session `、` 10.000`、`A `、`240 ` 等污染值必须使 `section_export_status`、`field_acceptance_status` 和 `evidence_status` 保持 FAIL。

v2.68 补充约束：runtime stability CSV 的 `sample,timestamp,disk_guard_status,watchdog_status,health_report` 数据字段必须保持原始字段值进入校验，不得先 trim 后比较状态或文本值。`capture_field_acceptance.sh` 与 `lio_eval_tools` evidence manifest 必须拒绝空字段和任何首尾空白污染；`PASS `、` PASS`、`runtime_health.txt ` 或 ` missing` 等污染值必须使 `runtime_stability_csv_status`、`runtime_stability_status`、`field_acceptance_status` 和 `evidence_status` 保持 FAIL。

v2.69 补充约束：`lio_eval_tools` evidence manifest 校验 `validation_metrics_report.txt` 时，不得先 trim 整行再解析 summary、record 或匹配记录块内的 `key=value`。空行可用 trim 判断跳过，但进入 `parseKeyValuePairs()` 的行必须保留原始内容；`failed_records=0 `、`failed_checks=0 ` 或 `recovery_time_s=25 ` 等行尾 value 污染必须使 `metrics_status`、`power_loss_resume_status`、`field_acceptance_status` 或 `evidence_status` 保持 FAIL。

v2.70 补充约束：所有通过脚本侧 `is_valid_text_value()` 或 C++ `validEvidenceTextValue()` 进入验收语义的文本证据值，必须保持原始 value 参与首尾空白污染检查。`wiring_verified_by=qa_operator `、`time_status_topic= /time/status`、`runtime_dir=/tmp/runtime ` 或 `health_report= runtime_health.txt` 等污染值，必须使对应 time sync、PPS/PTP wiring、power-loss resume、runtime health/deployment、runtime stability、field acceptance 或 evidence manifest gate 保持 FAIL。

v2.71 补充约束：`lio_eval_tools` 的 `parseMetricRecords()`、`parseScenarioThresholdRecords()` 和 `parseReplayEventRecords()` 只能用 trim 判断空行或注释行，进入 `parseKeyValuePairs()` 的记录行必须保持原始内容。`pps_jitter_ms=0.4 `、`max_length_error_percent=0.3 ` 或 `event=recovered;t=34.5 ` 等位于行尾的 value 污染必须继续触发指标、场景阈值或恢复时间 fail closed，不得因整行 trim 后被误判为合法。

v2.72 补充约束：`record_session.sh` 和 `runtime_ops.sh` 的入口级 manifest/runtime 文本值与绝对路径参数必须拒绝首尾空白污染。`--runtime-dir '/tmp/runtime '`、`--workspace '/opt/tunnel_lio/catkin_ws '`、带首尾空白的 session root 或 runtime root 等参数必须在生成 session 目录、metadata、runtime.env、systemd/Docker 骨架和后续证据命令前 fail fast，不得依赖后续 evidence manifest 再补救。

v2.73 补充约束：`/diag/dump_event` 读取 session 侧 `metadata.env` 时，只允许裁剪 key；value 必须保持原文并在生成 `event_manifest.env`、`filter_rosbag.sh` 或 `extract_pcap_window.sh` 前通过有效性校验。`bag_path=/.../tunnel_lio.bag `、`pcap_path=missing`、重复 `bag_path` 或包含分号、双引号、反引号、美元符号、反斜杠等污染字符的 metadata 必须 fail fast，不得生成事件切片目录或命令。

v2.74 补充约束：`/diag/dump_event` 的 service request 在读取 metadata 和创建事件目录前必须 fail fast。`session_dir`、`output_root` 和 `reason` 必须拒绝空值、首尾空白、`missing`、`__DUPLICATE_KEY__`、分号、换行、回车、双引号、美元符号、反引号和反斜杠；`event_id` 还必须只包含 `[A-Za-z0-9_.-]`，不得用静默替换把非法 ID 改写成目录名；`event_time_s/window_before_s/window_after_s` 必须是有限值且窗口非负。污染 request 不得写入 `source_event_id`、`reason`、`session_dir`，不得生成含污染路径的 `filter_rosbag.sh` 或 `extract_pcap_window.sh`。

v2.75 补充约束：`/diag/dump_event` 的路径输入必须在生成事件目录和 shell 脚本前证明为绝对路径。service request 的 `session_dir/output_root` 以及 `metadata.env` 中的 `bag_path/pcap_path` 不得是 `session_a`、`events`、`bags/tunnel_lio.bag` 或其它相对路径；相对路径必须 fail fast，不能写入 `event_manifest.env`，也不能生成依赖当前工作目录的 `filter_rosbag.sh` 或 `extract_pcap_window.sh`。

v2.76 补充约束：`/diag/dump_event` 读取 `metadata.env` 时，`session_name` 必须沿用 session 归档入口的安全 token 规则，只允许 `[A-Za-z0-9_.-]`，并继续拒绝空值、首尾空白、`missing` 和 `__DUPLICATE_KEY__`。`session_name=session a`、`session_name=session/name` 或其它非 token 会话名必须 fail fast，不得写入事件侧 `event_manifest.env`。

v2.77 补充约束：`/diag/dump_event` 的安全 token 还必须满足单一路径段语义，不能只是字符集合法。service request 的 `event_id` 和 `metadata.env` 的 `session_name` 必须拒绝单独的 `.` 或 `..`；`event_id=..` 不得被拼接为 `output_root/..` 创建父目录事件切片，`session_name=..` 也不得写入事件 manifest。

v2.78 补充约束：`/diag/dump_event` 的 `event_time_s` 必须是有限非负数。`event_time_s=-1` 或其它负事件时间必须在读取 metadata 和创建事件目录前 fail fast，不得生成 `end_time_s` 为负的 `event_manifest.env`、`filter_rosbag.sh` 或 `extract_pcap_window.sh`。

v2.79 补充约束：`/diag/dump_event` 的绝对路径输入必须同时拒绝 `.` 和 `..` 路径段。service request 的 `session_dir/output_root` 以及 `metadata.env` 中的 `bag_path/pcap_path` 即便以 `/` 开头，也不得包含 `/./`、`/../`、以 `/.` 或 `/..` 结尾等需要路径归一化解释的段；`output_root=/.../events/../escaped_events` 或 `bag_path=/.../bags/../bags/tunnel_lio.bag` 必须 fail fast，不得创建事件目录、写入 manifest 或生成 shell 命令。

v2.80 补充约束：`record_session.sh` 与 `runtime_ops.sh` 的绝对路径参数必须拒绝 `.` 和 `..` 路径段。`record_session.sh --root /.../sessions/../escaped_sessions`、`--runtime-dir /.../runtime/../escaped_runtime`、`runtime_ops.sh --root /.../runtime/../escaped_runtime` 或 `--workspace /opt/tunnel_lio/../catkin_ws` 必须在创建 session/runtime 目录、写入 metadata/runtime.env、生成 systemd/Docker 骨架或命令脚本前 fail fast。

v2.81 补充约束：所有进入 `/diag/dump_event`、`record_session.sh` 和 `runtime_ops.sh` 的绝对路径参数不得单独为文件系统根目录 `/`。`metadata.env` 的 `bag_path/pcap_path`、service request 的 `session_dir/output_root`、`record_session.sh` 的 `--root/--runtime-dir`、`runtime_ops.sh` 的 `--root/--workspace` 为 `/` 时，必须在读取/生成 metadata、创建事件/session/runtime 目录、写入 manifest/runtime.env 或生成 systemd/Docker/命令脚本前 fail fast。

v2.82 补充约束：`lio_eval_tools` 的 evidence manifest 校验器必须对运行目录证据执行与生成侧一致的绝对路径语义。manifest 顶层 `runtime_dir`、`runtime_health.txt` 的 `runtime_dir`、`runtime_deployment.txt` 的 `runtime_dir`，以及 `field_acceptance_report.txt` 下沉的 `runtime_health_runtime_dir/deployment_runtime_dir` 不得单独为 `/`，也不得包含 `.` 或 `..` 路径段；若伪造证据包让这些字段彼此一致但指向根目录或依赖路径归一化解释，`runtime_health_status`、`runtime_deployment_status`、`field_acceptance_status` 和最终 `evidence_status` 必须保持 FAIL。

v2.83 补充约束：`lio_eval_tools` 的 evidence manifest 校验器必须对 time sync topic 证据执行与 `record_session.sh` 生成侧一致的 ROS topic 名校验。`logs/time_sync_status.txt` 和 `field_acceptance_report.txt` 中的 `time_status_topic/pps_topic` 不得只是非空有效文本，还必须满足 `(/|~)?[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*`；`$bad_topic`、`/time//pps_event`、尾随 `/` 或数字开头段等污染 topic 必须使 `time_sync_status`、`pps_ptp_wiring_status`、`field_acceptance_status` 和最终 `evidence_status` 保持 FAIL。

v2.84 补充约束：`lio_eval_tools` 的 evidence manifest 校验器必须把 `field_acceptance_report.txt` 中下沉的 `time_status_topic/pps_topic` 与 `logs/time_sync_status.txt` 的同名字段完全绑定。独立 time sync 证据与最终验收报告即使各自 topic 格式合法，只要 topic 文本不一致，`field_acceptance_status` 和最终 `evidence_status` 必须保持 FAIL；`pps_ptp_wiring_status` 仍只依赖独立 time sync PASS 和人工接线证据。

v2.85 补充约束：`lio_eval_tools` 的 evidence manifest 校验器必须把 PPS/PTP wiring 和最终 field acceptance 中下沉的 `pps_jitter_ms/mean_offset_ms` 与独立 `logs/time_sync_status.txt` 的同名字段完全绑定。下游报告即使数值格式合法、状态字段均为 PASS，只要 jitter 或 mean offset 文本与独立 time sync 证据不一致，`pps_ptp_wiring_status`、`field_acceptance_status` 和最终 `evidence_status` 必须保持 FAIL。

v2.86 补充约束：`lio_eval_tools` 的 evidence manifest 校验器必须把最终 field acceptance 中下沉的 runtime health 明细与独立 `runtime_health.txt` 完全绑定。`runtime_health_disk_available_gb/runtime_health_pid/runtime_health_systemd_active/runtime_health_docker_container_status` 即使格式合法，只要与独立 runtime health 证据不一致，`field_acceptance_status` 和最终 `evidence_status` 必须保持 FAIL。

v2.87 补充约束：`lio_eval_tools` 的 evidence manifest 校验器必须把最终 field acceptance 中下沉的 PPS/PTP wiring 人工审计字段与独立 `pps_ptp_wiring_verified.txt` 完全绑定。`wiring_verified_by/wiring_verified_at` 即使格式合法，只要与独立 PPS/PTP wiring 证据不一致，`field_acceptance_status` 和最终 `evidence_status` 必须保持 FAIL。

v2.88 补充约束：`lio_eval_tools` 的 evidence manifest 校验器必须把最终 field acceptance 中下沉的断电续建字段与独立 `power_loss_resume_verified.txt` 完全绑定。`power_loss_resume_source/recovery_time_s/max_recovery_time_s/power_loss_resume_confirmation_overall` 即使格式合法，只要与独立断电续建证据不一致，`field_acceptance_status` 和最终 `evidence_status` 必须保持 FAIL；当来源为 `manual_file` 时，`resume_verified_by/resume_verified_at` 也必须与独立断电续建证据一致。

v2.89 补充约束：`lio_eval_tools` 的 evidence manifest 校验器必须把最终 field acceptance 中下沉的 `metrics_report` 与 evidence manifest 声明的独立 metrics 报告完全绑定。无论断电续建来源是 `metrics_report` 还是 `manual_file`，只要最终验收报告中的 `metrics_report` 解析路径与独立 metrics 报告不一致，`field_acceptance_status` 和最终 `evidence_status` 必须保持 FAIL。

v2.90 补充约束：`record_session.sh` 生成的 `capture_time_sync.sh` 与 `capture_field_acceptance.sh` 必须对 `time_status_topic/pps_topic` 使用与 session 入口和 `lio_eval_tools` evidence manifest 一致的 ROS topic 名正则校验。`/time//status`、`/time/1pps`、尾随 `/` 或其他格式错误 topic 即使是非空有效文本，也必须使 time sync 捕获或最终 field acceptance 复验保持 FAIL，不得先生成脚本侧 PASS 外观证据。

v2.91 补充约束：`record_session.sh` 生成的 `capture_pps_ptp_wiring.sh` 必须对 `logs/time_sync_status.txt` 中的 `time_status_topic/pps_topic` 执行同一 ROS topic 名校验，并将这两个字段下沉到 `reports/pps_ptp_wiring_verified.txt`。若 time sync 报告被污染为 `/time//status`、`/time/1pps` 或其他格式错误 topic，即便 `capture_status/pps_status/clock_offset_status` 和数值字段都呈现 PASS，PPS/PTP wiring 捕获也必须保持 FAIL。

v2.92 补充约束：`lio_eval_tools` 的 evidence manifest 校验器必须把 `pps_ptp_wiring_verified.txt` 中下沉的 `time_status_topic/pps_topic` 与独立 `time_sync_status.txt` 的同名字段完全绑定。wiring 报告即使 `pps_ptp_wiring_verified=PASS`、人工接线审计字段和 jitter/offset 数值均合法，只要 topic 缺失、格式错误或与独立 time sync 证据不一致，`pps_ptp_wiring_status`、`field_acceptance_status` 和最终 `evidence_status` 必须保持 FAIL。

v2.93 补充约束：`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须对 `pps_ptp_wiring_verified.txt` 中的 `time_status_topic/pps_topic` 执行与 time sync 捕获、PPS/PTP wiring 捕获和 `lio_eval_tools` evidence manifest 一致的校验，并把 `pps_ptp_wiring_time_status_topic/pps_ptp_wiring_pps_topic` 下沉到 `field_acceptance_report.txt`。若 wiring 报告 topic 缺失、格式错误或与 `logs/time_sync_status.txt` 不一致，即便 `pps_ptp_wiring_verified=PASS` 和人工审计字段均合法，最终 `field_acceptance_status` 也必须保持 FAIL。

v2.94 补充约束：`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须对 `pps_ptp_wiring_verified.txt` 中的 `pps_jitter_ms/mean_offset_ms` 执行与 time sync 捕获、PPS/PTP wiring 捕获和 `lio_eval_tools` evidence manifest 一致的数值校验，并把 `pps_ptp_wiring_pps_jitter_ms/pps_ptp_wiring_mean_offset_ms` 下沉到 `field_acceptance_report.txt`。若 wiring 报告的 PPS jitter 或 mean offset 缺失、格式错误、PPS jitter 为负或与 `logs/time_sync_status.txt` 不一致，即便 wiring 报告 topic、接线总字段和人工审计字段均合法，最终 `field_acceptance_status` 也必须保持 FAIL。

v2.95 补充约束：`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须对 `pps_ptp_wiring_verified.txt` 中的 `time_sync_status/capture_status/pps_status/clock_offset_status` 执行与 PPS/PTP wiring 捕获和 `lio_eval_tools` evidence manifest 一致的状态校验，并把 `pps_ptp_wiring_time_sync_status/pps_ptp_wiring_capture_status/pps_ptp_wiring_pps_status/pps_ptp_wiring_clock_offset_status` 下沉到 `field_acceptance_report.txt`。若 wiring 报告自称 `pps_ptp_wiring_verified=PASS` 但 time sync 状态不是 PASS、capture_status 不是 CAPTURED、pps_status 不是 PASS 或 clock_offset_status 不是 PASS，即便 topic、数值和人工审计字段均合法，最终 `field_acceptance_status` 也必须保持 FAIL。

v2.96 补充约束：PPS/PTP wiring 报告中的 `time_sync_report` 必须作为来源路径证据参与闭环校验。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须读取 `pps_ptp_wiring_verified.txt` 中的 `time_sync_report`，要求其与当前 session 的 `logs/time_sync_status.txt` 完全一致，并把 `pps_ptp_wiring_time_sync_report` 下沉到 `field_acceptance_report.txt`；`lio_eval_tools` evidence manifest 必须把 wiring 报告中的 `time_sync_report` 以及最终验收报告中的 `pps_ptp_wiring_time_sync_report` 解析到同一个 bundle/绝对路径语义后，与 manifest 声明的独立 time sync 证据路径一致。若 wiring 报告状态、topic、数值和人工审计字段均合法但 `time_sync_report` 指向另一份文件，PPS/PTP wiring、field acceptance 和最终 evidence_status 均必须保持 FAIL。

v2.97 补充约束：最终 `field_acceptance_report.txt` 不得只下沉 PPS/PTP wiring 的总字段、人工字段和 `time_sync_report` 路径；还必须下沉并由 `lio_eval_tools` 重验 `pps_ptp_wiring_time_sync_status=PASS`、`pps_ptp_wiring_capture_status=CAPTURED`、`pps_ptp_wiring_pps_status=PASS` 和 `pps_ptp_wiring_clock_offset_status=PASS`。若独立 wiring 证据已通过但最终验收报告把这些下沉状态字段改为 FAIL、UNAVAILABLE、缺失或其它非通过值，`field_acceptance_status` 与最终 `evidence_status` 均必须保持 FAIL。

v2.98 补充约束：最终 `field_acceptance_report.txt` 中的 PPS/PTP wiring 下沉 topic 与数值字段必须与独立 time sync 证据保持同一语义。`lio_eval_tools` 必须要求 `pps_ptp_wiring_time_status_topic` 和 `pps_ptp_wiring_pps_topic` 是合法 ROS topic，且分别等于独立 time sync 的 `time_status_topic/pps_topic`；`pps_ptp_wiring_pps_jitter_ms` 和 `pps_ptp_wiring_mean_offset_ms` 必须可严格解析为数值，PPS jitter 必须非负，且文本值分别等于独立 time sync 的 `pps_jitter_ms/mean_offset_ms`。若最终报告把 wiring 下沉 topic 或数值改成另一组合法值，`field_acceptance_status` 与最终 `evidence_status` 均必须保持 FAIL。

v2.99 补充约束：`record_session.sh` 生成的 `capture_field_acceptance.sh` 对 metrics_report 来源的 `power_loss_resume_verified.txt` 不得只校验 metrics 报告总体 PASS、路径一致和恢复时间小于门槛；还必须用当前 session/scenario 的匹配记录重新抽取 `recovery_time_s`，并用严格数值解析后的等价关系确认该值与 verified 文件下沉的 `recovery_time_s` 一致。若 verified 文件自称 `power_loss_resume_status=PASS` 且 `recovery_time_s=25`，但当前 metrics 记录为 `recovery_time_s=30`，即便两者都满足 `max_recovery_time_s=45`，最终 `power_loss_resume_status`、`field_acceptance_status` 和 evidence_status 均必须保持 FAIL。

v3.00 补充约束：runtime health 快照不得只证明运行态字段是非空有效文本；`record_session.sh` 生成的 `capture_field_acceptance.sh` 和 `lio_eval_tools` evidence manifest 都必须要求 `runtime_health_latest.txt` 中 `systemd_active=active` 且 `docker_container_status=running`，并要求最终 `field_acceptance_report.txt` 下沉的 `runtime_health_systemd_active/runtime_health_docker_container_status` 与独立 runtime health 证据一致。若 runtime health 快照显示 `systemd_active=inactive`、`docker_container_status=exited`，即便 runtime deployment 报告单独证明 systemd/Docker 为 active/running，`runtime_health_status`、`field_acceptance_status` 和 evidence_status 均必须保持 FAIL。

v3.01 补充约束：板端 24 h 长稳采样不得只把 `runtime_health.sh` 成功写出文件当作健康成功。`runtime_ops.sh` 生成的 `runtime_stability_check.sh` 必须对每次采样返回的 health 报告执行脚本级复验：报告必须非空，`runtime_dir` 必须等于当前 runtime，`disk_available_gb` 必须是可解析非负数，`runtime_pid` 必须是严格正整数，且 `systemd_active=active`、`docker_container_status=running`。若 health 报告显示 inactive/exited、缺字段、数值畸形或来自其它 runtime，即使命令退出码为 0，也必须累加 `health_failures` 并使 `runtime_stability_summary.txt` 的 `overall=FAIL`。

v3.02 补充约束：板端 `runtime_stability_check.sh` 的采样参数必须在开始写入 CSV/summary 前完成脚本级校验。`--samples` 必须是严格正整数，`--interval` 必须是非负整数；`--samples 0`、非数字 samples 或负 interval 必须非零退出，不能生成不采样但 `overall=PASS` 的长稳 summary。`--interval 0` 仅保留给离线 dry-run/快速测试路径，最终 field acceptance 仍要求 summary interval 为严格正整数并满足 24 h 时长闭环。

v3.03 补充约束：板端 runtime 路径必须保持 runtime stability CSV 可解析。由于 `runtime_stability.csv` 使用固定表头 `sample,timestamp,disk_guard_status,watchdog_status,health_report` 且不提供 CSV 转义，`runtime_ops.sh` 生成 runtime 前必须拒绝会进入 `runtime_dir/logs/runtime_health_*.txt` 路径的逗号污染。`runtime_root` 不得包含逗号；`runtime_name` 必须继续满足安全 token 约束，因此同样不得包含逗号。否则 `runtime_health.sh` 即便成功输出报告路径，也可能把长稳 CSV 拆成多列，使 session 归档和最终现场验收无法形成稳定证据。

v3.04 补充约束：session 归档入口的非空 `runtime_dir` 必须与板端 runtime 入口保持同一 CSV 安全语义。`record_session.sh` 在创建 session 目录、写入 metadata/evidence manifest、生成 `run_runtime_stability.sh` 和 `capture_runtime_stability.sh` 前必须拒绝 `runtime_dir` 中的逗号，防止现场采集时把会污染 `runtime_stability.csv` 固定 5 列格式的运行目录固化进证据包。该约束仅针对会进入长稳 CSV 路径证据的 runtime 目录，不改变 topic 列表等已有多值参数的空格分隔语义。

v3.05 补充约束：最终证据包校验端必须与生成端保持同一 runtime 目录 CSV 安全语义。`lio_eval_tools` 不得只把 manifest 顶层 `runtime_dir`、`runtime_health.txt` 的 `runtime_dir`、`runtime_deployment.txt` 的 `runtime_dir`、以及 `field_acceptance_report.txt` 下沉的 `runtime_health_runtime_dir/deployment_runtime_dir` 判定为合法绝对路径，还必须拒绝其中的逗号。若伪造证据包让这些字段彼此一致但包含逗号，即便 metrics、time sync、PPS/PTP wiring、runtime health/deployment/stability、section export、power-loss resume 和 field acceptance 其它状态均自称 PASS，最终 `evidence_status` 必须保持 FAIL。

v3.06 补充约束：板端长稳采样脚本必须在写入 CSV 前校验 `runtime_health.sh` 返回的 health 报告路径本身。即便报告文件内容能够证明 `runtime_dir`、磁盘、PID、systemd active 和 Docker running 全部合法，只要返回路径包含逗号，`runtime_stability_check.sh` 就必须把该采样的 `health_report` 写为 `missing`、累加 `health_failures`，并使 `runtime_stability_summary.txt` 的 `overall=FAIL`。这样被现场替换或包裹的 health 脚本不能通过返回污染路径破坏 `runtime_stability.csv` 的固定 5 列结构。

v3.07 补充约束：板端长稳采样脚本必须同时拒绝 `runtime_health.sh` 返回路径中的换行和回车。由于 `runtime_stability.csv` 不做 CSV 转义，包含 LF/CR 的路径即便列数仍看似正确，也会把单条采样拆成多行，导致 session 归档和最终 evidence manifest 看到的采样数、采样行和 summary 语义不一致。`runtime_stability_check.sh` 必须在写入 CSV 前把这类 `health_report` 改写为 `missing`、累加 `health_failures`，并使 summary `overall=FAIL`。

v3.08 补充约束：板端长稳采样脚本还必须拒绝 `runtime_health.sh` 返回路径中的分号。分号不会直接拆开 `runtime_stability.csv` 的列数，但会污染后续分号键值格式报告和证据文本字段，尤其是在 session 归档、field acceptance 下沉或人工排查时造成路径值与新 key/value 片段混淆。`runtime_stability_check.sh` 必须将这类 `health_report` 同样改写为 `missing`、累加 `health_failures`，并使 summary `overall=FAIL`。

v3.09 补充约束：session 侧长稳归档必须绑定本 session 的触发记录。`capture_runtime_stability.sh` 在复制板端 `runtime_stability.csv` 和 `runtime_stability_summary.txt` 前，必须先读取当前 session 的 `logs/runtime_stability_run.log`，要求其中 `runtime_dir`、`samples`、`interval` 与本次配置完全一致，且 `exit_status=0`。若 run log 已存在 `capture_exit_status` 字段，则该字段也必须为字面量 `0`；`capture_exit_status` 缺失只允许用于 `run_runtime_stability.sh` 首次调用归档脚本且尚未回写捕获退出码的执行中状态。若 run log 缺失、参数不匹配、运行失败或既有捕获退出状态非 0，脚本必须写入只有固定表头的 CSV 和 `overall=FAIL/capture_status=RUN_LOG_*` 的 summary，并非零退出，防止 `validate_evidence.sh` 在没有执行 `run_runtime_stability.sh` 或上次捕获失败后误归档板端旧长稳结果。

v3.10 补充约束：最终 evidence manifest 必须把 session 侧长稳触发记录纳入必需证据。`record_session.sh` 生成的 `evidence_manifest.txt` 必须声明 `runtime_stability_run_log=logs/runtime_stability_run.log`；`lio_eval_tools` 在校验 `runtime_stability_summary.txt` 后，必须继续校验 run log 的 `started_at/finished_at` 为 ISO-8601 seconds、`runtime_dir` 与 manifest 有效运行目录一致、`samples/interval` 与 summary 一致且 `exit_status=0`、`capture_exit_status=0`。run log 缺失、为空、路径不安全、时间戳畸形、运行目录不一致、采样数/间隔不一致或任一退出状态非 0 时，即使 CSV 和 summary 自称 PASS，`runtime_stability_status` 与最终 `evidence_status` 也必须保持 FAIL。

v3.11 补充约束：最终 field acceptance 不得只相信长稳 CSV/summary，也必须绑定 session 侧长稳触发记录。`capture_field_acceptance.sh` 必须读取当前 session 的 `logs/runtime_stability_run.log`，将 `runtime_stability_run_log_status`、`runtime_stability_run_log_started_at`、`runtime_stability_run_log_finished_at`、`runtime_stability_run_log_runtime_dir`、`runtime_stability_run_log_samples`、`runtime_stability_run_log_interval`、`runtime_stability_run_log_exit_status` 和 `runtime_stability_run_log_capture_exit_status` 下沉到 `field_acceptance_report.txt`，并要求这些字段与当前 session 运行目录、长稳 summary 和两个零退出状态一致。`lio_eval_tools` 还必须复验 field acceptance 报告中的这些下沉字段；缺失、畸形、运行目录不一致、采样数/间隔不一致或任一退出状态非 0 时，即使独立 run log 和 summary 通过，`field_acceptance_status` 与最终 `evidence_status` 也必须保持 FAIL。

v3.12 补充约束：`lio_eval_tools` 的最终证据包校验不得只检查 field acceptance 下沉的 run log 字段与 manifest/summary 间接一致，还必须与独立 `runtime_stability_run.log` 的原始字段逐项一致。独立 run log 通过后，校验器必须保留 `started_at`、`finished_at`、`runtime_dir`、`samples`、`interval`、`exit_status` 和 `capture_exit_status` 文本值，并要求 `field_acceptance_report.txt` 中对应 `runtime_stability_run_log_*` 字段完全相同。若最终报告使用另一组合法时间戳或合法字段值伪造下沉证据，即使独立 run log 自身 PASS，`field_acceptance_status` 和最终 `evidence_status` 也必须保持 FAIL。

v3.13 补充约束：最终 field acceptance 不得只下沉 `section_export_status=PASS`，还必须下沉 `section_export_report` 来源路径。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须把当前 session 的 `reports/section_export.csv` 写入该字段；`lio_eval_tools` 必须要求该字段为合法 bundle 相对路径或绝对路径，并解析到 evidence manifest 声明的独立 `section_export` 文件。缺失、哨兵值、非法路径或指向另一份 CSV 时，即使独立 section export CSV 自身通过，`field_acceptance_status` 与最终 `evidence_status` 也必须保持 FAIL。

v3.14 补充约束：最终 field acceptance 不得只下沉 `runtime_health_status=PASS` 和健康明细，还必须下沉 `runtime_health_report` 来源路径。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须把当前 session 的 `logs/runtime_health_latest.txt` 写入该字段；`lio_eval_tools` 必须要求该字段为合法 bundle 相对路径或绝对路径，并解析到 evidence manifest 声明的独立 `runtime_health` 快照。缺失、哨兵值、非法路径或指向另一份健康快照时，即使独立 runtime health 快照自身通过，`field_acceptance_status` 与最终 `evidence_status` 也必须保持 FAIL。

v3.15 补充约束：最终 field acceptance 不得只下沉 `runtime_deployment_status=PASS` 和部署明细，还必须下沉 `runtime_deployment_report` 来源路径。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须把当前 session 的 `logs/runtime_deployment_check.txt` 写入该字段；`lio_eval_tools` 必须要求该字段为合法 bundle 相对路径或绝对路径，并解析到 evidence manifest 声明的独立 `runtime_deployment` 检查文件。缺失、哨兵值、非法路径或指向另一份部署检查文件时，即使独立 runtime deployment 检查自身通过，`field_acceptance_status` 与最终 `evidence_status` 也必须保持 FAIL。

v3.16 补充约束：最终 field acceptance 不得只下沉 `runtime_stability_status=PASS`、长稳 CSV 状态、summary 明细和 run log 明细，还必须下沉 `runtime_stability_csv_report`、`runtime_stability_summary_report` 与 `runtime_stability_run_log_report` 来源路径。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须把当前 session 的 `logs/runtime_stability.csv`、`logs/runtime_stability_summary.txt` 和 `logs/runtime_stability_run.log` 分别写入这些字段；`lio_eval_tools` 必须要求三者均为合法 bundle 相对路径或绝对路径，并分别解析到 evidence manifest 声明的独立长稳 CSV、summary 和 run log 文件。任一字段缺失、哨兵值、非法路径或指向另一份长稳证据时，即使独立 runtime stability 证据自身通过，`field_acceptance_status` 与最终 `evidence_status` 也必须保持 FAIL。

v3.17 补充约束：最终 field acceptance 不得只下沉 `power_loss_resume_status=PASS`、`power_loss_resume_source`、恢复时间和人工审计明细，还必须下沉 `power_loss_resume_report` 来源路径。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须把当前 session 的 `reports/power_loss_resume_verified.txt` 写入该字段；`lio_eval_tools` 必须要求该字段为合法 bundle 相对路径或绝对路径，并解析到 evidence manifest 声明的独立 `power_loss_resume` 证据文件。缺失、哨兵值、非法路径或指向另一份断电续建证据时，即使独立 power-loss resume 证据自身通过，`field_acceptance_status` 与最终 `evidence_status` 也必须保持 FAIL。

v3.18 补充约束：最终 field acceptance 不得只下沉 `time_sync_status=PASS`、time sync topic、PPS 状态、clock offset 状态和 jitter/offset 数值，还必须下沉 `time_sync_report` 来源路径。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须把当前 session 的 `logs/time_sync_status.txt` 写入该字段；`lio_eval_tools` 必须要求该字段为合法 bundle 相对路径或绝对路径，并解析到 evidence manifest 声明的独立 `time_sync` 证据文件。缺失、哨兵值、非法路径或指向另一份 time sync 证据时，即使独立 time sync 证据自身通过，`field_acceptance_status` 与最终 `evidence_status` 也必须保持 FAIL。

v3.19 补充约束：最终 field acceptance 不得只下沉 `pps_ptp_wiring_verified=PASS`、PPS/PTP wiring 的 time sync 复验字段和人工接线审计字段，还必须下沉 `pps_ptp_wiring_report` 来源路径。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须把当前 session 的 `reports/pps_ptp_wiring_verified.txt` 写入该字段；`lio_eval_tools` 必须要求该字段为合法 bundle 相对路径或绝对路径，并解析到 evidence manifest 声明的独立 `pps_ptp_wiring` 证据文件。缺失、哨兵值、非法路径或指向另一份 PPS/PTP wiring 证据时，即使独立 wiring 证据自身通过，`field_acceptance_status` 与最终 `evidence_status` 也必须保持 FAIL。

v3.20 补充约束：最终 field acceptance 不得只依赖 evidence manifest 外层的 metrics PASS 结果，也必须在 `field_acceptance_report.txt` 自身下沉 `metrics_status=PASS`。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须对当前 session 的 `reports/validation_metrics_report.txt` 重新执行 `metrics_report_passed` 完整 gate，并把结果写为 `metrics_status=PASS/FAIL`；最终 `field_acceptance_status=PASS` 必须显式依赖该字段。`lio_eval_tools` 必须要求最终报告自身包含 `metrics_status=PASS`。缺失 metrics 状态、metrics 报告失败、重复 key、summary 污染、记录数不合法、缺少当前 session/scenario 匹配 PASS 记录或 `manual_file` 断电恢复路径试图绕过 metrics 时，`field_acceptance_status` 与最终 `evidence_status` 均必须保持 FAIL。

v3.21 补充约束：evidence manifest 中的 replay/HIL `event_file` 不得只作为存在性证据。`lio_eval_tools` 必须解析该文件中的规范化事件流，要求事件可聚合出当前 manifest 的 `session_id/scenario`，聚合指标必须通过默认指标 gate，且事件时间戳必须是可解析有限值。若 event_file 只有占位行、缺少当前 scenario、指向其它 session、包含畸形时间戳或聚合指标失败，即便 metrics report 和 field acceptance 其它证据均为 PASS，最终 `event_file_status` 与 `evidence_status` 也必须保持 FAIL。

v3.22 补充约束：仓库自带 sample evidence bundle 必须持续作为当前 evidence gate 的可执行样例，而不是停留在旧字段格式。`sample_evidence_manifest.txt` 必须包含 `runtime_stability_run_log`，`sample_replay_events.txt` 必须绑定 manifest 的 `sample_static/STATIC_IDLE` 并通过默认 replay 指标 gate，`sample_pps_ptp_wiring_verified.txt` 与 `sample_field_acceptance_report.txt` 必须下沉 time sync topic、PPS/PTP 状态/数值、runtime stability run log、来源报告路径和 `metrics_status=PASS` 等当前交叉校验字段。若后续新增 evidence gate，应同步新增样例 PASS 测试，防止文档中的样例 launch 与实际校验器脱节。

v3.23 补充约束：evidence manifest 中的 replay/HIL `event_file` 不得只用 `event=session_start` 这类空壳事件绑定 `session_id/scenario`。`lio_eval_tools` 必须要求事件流至少包含一个真实验收指标字段，例如 `static_drift_m`、`length_error_percent`、`chainage_m/reference_chainage_m`、`wrong_loop`、`queue_backlog` 或 `pps_jitter_ms`，或者包含完整的 `event=power_loss` 到 `event=recovered`/`event=recovery_complete` 恢复事件对。只有 session/scenario/t 均合法但没有任何指标证据时，`event_file_status` 与最终 `evidence_status` 必须保持 FAIL。

v3.24 补充约束：最终 `field_acceptance_status=PASS` 必须吸收 replay/HIL `event_file` 的独立校验结果，不能只由 `evidence_status` 外层汇总体现。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须读取当前 session 的 `reports/replay_events.txt`，要求每条非空事件记录包含合法 `event/session_id/scenario/t`，当前 `session_id/scenario` 至少包含一个验收指标字段或完整 `power_loss -> recovered/recovery_complete` 恢复事件对，并在 `field_acceptance_report.txt` 下沉 `event_file_status=PASS/FAIL` 与 `event_file_report`。`lio_eval_tools` 必须同时要求独立 `event_file` 解析 PASS、`field_acceptance_report.txt` 下沉 `event_file_status=PASS`，且 `event_file_report` 解析到 evidence manifest 声明的同一事件文件。event_file 缺失、畸形、只含空壳事件、指向其它会话/场景，或最终报告未下沉 event 状态/路径时，`field_acceptance_status` 与最终 `evidence_status` 均必须保持 FAIL。

v3.25 补充约束：`record_session.sh` 生成的 `capture_field_acceptance.sh` 不得允许通过 `FIELD_ACCEPTANCE_MIN_STABILITY_HOURS` 把最终现场验收的长稳要求降到 24 h 以下。该环境变量只允许设为合法数值且大于等于 24，用于提高门槛；若为空则使用默认 24。若设置为小于 24、非数值或污染值，脚本必须在 `field_acceptance_report.txt` 下沉 `runtime_stability_min_duration_h` 和 `runtime_stability_min_duration_status=FAIL`，并使 `runtime_stability_status` 与 `field_acceptance_status` 保持 FAIL。

v3.26 补充约束：最终 evidence manifest 校验器必须复核 v3.25 下沉字段，不能只相信 `runtime_stability_duration_h >= 24`。`lio_eval_tools` 必须要求 `field_acceptance_report.txt` 明确包含 `runtime_stability_min_duration_status=PASS` 和严格数值 `runtime_stability_min_duration_h`，其中最小时长必须 `>= 24`，且报告自称的 `runtime_stability_duration_h` 必须大于等于该最小时长。缺失状态、状态非 PASS、最小时长低于 24、最小时长畸形或实际长稳时长小于该最小时长时，`field_acceptance_status` 与最终 `evidence_status` 均必须保持 FAIL。

v3.27 补充约束：evidence manifest 中的 replay/HIL `event_file` 必须把真实指标证据绑定到当前 `session_id/scenario`。`lio_eval_tools` 在判断是否存在静止漂移、长度误差、错回环、队列堆积、PPS 抖动或完整 `power_loss -> recovered/recovery_complete` 恢复事件证据时，只能检查当前 manifest 的 `session_id/scenario` 事件，并且 replay metrics 聚合也只能使用这些匹配事件。若当前会话只有 `session_start` 等空壳事件，而其它 session 或其它 scenario 提供了合法指标字段，`event_file_status`、`field_acceptance_status` 与最终 `evidence_status` 均必须保持 FAIL。

v3.28 补充约束：`record_session.sh` 生成的 `capture_field_acceptance.sh` 不得只检查当前 replay/HIL `event_file` 是否出现指标字段。对当前 `session_id/scenario` 的指标字段，`static_drift_m`、`length_error_percent`、`chainage_m`、`reference_chainage_m`、`pps_jitter_ms` 必须是严格可解析数值；`wrong_loop`、`queue_backlog` 必须是严格整数；`chainage_m` 与 `reference_chainage_m` 必须成对出现。畸形值、`nan/inf`、尾随污染或缺少配对字段时，`event_file_status`、`field_acceptance_status` 与最终 `evidence_status` 均必须保持 FAIL。

v3.29 补充约束：replay/HIL `event_file` 的物理非负指标不得接受负值。`lio_eval_tools` 聚合 `static_drift_m`、`length_error_percent`、`wrong_loop`、`queue_backlog` 和 `pps_jitter_ms` 时，若当前 `session_id/scenario` 事件出现负值，必须把对应指标置为失败并使 `event_file_status` 与最终 `evidence_status` 保持 FAIL；`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须执行同样规则，使负值 replay 指标不能写出 `event_file_status=PASS` 或 `field_acceptance_status=PASS`。

v3.30 补充约束：replay/HIL `event_file` 每条规范化事件的 `t` 必须严格数值可解析且非负。`lio_eval_tools` 解析到 `t<0` 时必须将该事件视为非法时间戳，并使事件流聚合指标、`event_file_status` 与最终 `evidence_status` 保持 FAIL；`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须执行同样规则，使负时间戳 replay 事件不能写出 `event_file_status=PASS` 或 `field_acceptance_status=PASS`。

v3.31 补充约束：evidence manifest 侧的 replay/HIL `event_file` 必须对所有非空规范化事件逐记录执行结构校验，而不能只过滤当前 `session_id/scenario` 后校验匹配记录。任一事件记录缺失 `event/session_id/scenario/t`、包含重复 key、`session_id/scenario` 不满足合法 token 约束或事件名被空值/哨兵/首尾空白污染时，即使当前会话另有合法指标事件，`event_file_status`、`field_acceptance_status` 与最终 `evidence_status` 也必须保持 FAIL。

v3.32 补充约束：`validation_report_node` 通过 `event_file` 参数直接聚合 replay/HIL 事件时，必须与 evidence manifest 入口采用同样的规范化事件结构语义。每条事件记录都必须包含合法 `event/session_id/scenario/t` 且无重复 key；缺失结构字段、session/scenario token 非法、事件名为空值/哨兵/首尾空白污染或重复 key 时，聚合结果必须 fail closed，不能生成 `overall=PASS` 的 direct event_file 报告。

v3.33 补充约束：`slam_backend_manager` 的参数化 Scan Context 风格几何描述子和 ISC 风格强度上下文描述子不得接受非法配置继续参与回环候选。`ring_edges` 必须至少包含两个有限、非负且严格递增的边界，`sector_count` 必须为正；强度上下文的量化尺度和最小占用点数必须为有效正值；候选选择使用的 `min_loop_score` 必须位于 `[0,1]`，`min_top_score_ratio >= 1`，`min_loop_chainage_separation_m >= 0`，`intensity_descriptor_weight` 必须位于 `[0,1]`。任何违反这些条件的配置都必须不产出描述子或候选，防止现场参数污染制造错误回环或稳定图晋升依据。

v3.34 补充约束：`tca_manager` 的高反标靶检测、上下文签名和锚点台账匹配必须对非法参数和无效锚点 fail closed。检测配置中的 `intensity_threshold/cluster_radius_m` 必须为有限值，`cluster_radius_m > 0`，`min_reflective_points > 0`；`context_ring_edges` 必须至少包含两个有限、非负且严格递增的边界。TCA ledger 的 `min_match_score` 必须位于 `[0,1]`，`min_top_score_ratio >= 1`；anchor 必须包含非空 `anchor_id`、有限 `chainage_m`、有限中心点和非空非负 `context_signature`。任一条件不满足时不得生成检测、签名或匹配，防止损坏台账或现场参数污染触发错误 TCA 恢复锚点。

v3.35 补充约束：`lio_local_odometry` 的 ICP/NDT/GICP 局部注册在进入 PCL 前必须校验多尺度 leaf/resolution、最大对应距离、变换/fitness epsilon、step size、迭代次数和 min_points；非法配置必须直接输出 `invalid_config` 且 `stages_used=0`。局部注册可用性、几何可观测性和子图质量治理必须拒绝非有限 fitness/geometry/observability、非法门控阈值和非有限几何点，不能通过 NaN 比较绕过弱几何、退化或晋升 gate。

v3.36 补充约束：`lio_session_manager` 的断电续建恢复决策必须拒绝非有限或越界的 local/TCA/global 恢复分数和阈值，阈值配置中 `local_relocalize` 不得高于 `local_resume`；非法恢复配置只能保守进入临时会话路径，不得借其它候选分数触发 TCA/global 恢复。稳定图恢复锚点读取必须要求查询链距、最大距离、最低质量门槛和 ledger 条目的 `keyframe_id/chainage_m/section_quality` 合法；稳定锚点恢复精配准 gate 必须拒绝非有限或越界的 `rmse/inlier_ratio/translation/yaw` 和 gate 阈值，任何 NaN 比较不得绕过恢复质量门槛。

v3.37 补充约束：`slam_backend_manager` 的一维链距位姿图和可调 Ceres 非线性 6DoF 位姿图优化入口必须在构图前校验全部节点、约束和配置。节点 `keyframe_id` 必须非空且唯一，链距和 6DoF 位姿全部为有限值；约束端点必须引用已知节点，relative chainage/pose 必须有限，weight 必须位于 `[0,1]`，协方差字段必须有限，Huber 风格鲁棒核 delta 必须为有限非负值；Ceres 配置中的迭代次数和隐式里程计/平滑先验标准差必须合法。任一输入污染时优化器必须返回空结果，不得把 NaN 残差交给 Ceres，也不得输出看似可用的优化图污染回环治理、稳定图晋升或续建恢复。

v3.38 补充约束：`lio_preprocess` 的 IMU 连续时间去畸变必须在使用点内时间和 IMU 角速度前校验 deskew 配置、cloud stamp、点时间字段和 IMU 样本。`reference` 只允许 `start/end`，`point_time_scale` 与 `max_abs_point_time` 必须有限且 `max_abs_point_time >= 0`，point time 候选字段不得为空；IMU `stamp/wx/wy/wz` 必须全部有限。任一条件不满足时必须保持原始点或跳过该点 deskew，并通过 `deskew_invalid_config/deskew_invalid_imu/deskew_invalid_point_time` 诊断下沉，不得输出 NaN 坐标污染局部注册。

v3.39 补充约束：`lio_state_estimator` 的 IMU 滑窗、静止 bias/gravity 估计和短时预积分必须在使用样本前校验 `stamp/ax/ay/az/gx/gy/gz` 全部有限。非法样本不得进入滑窗均值、频率、RMS、健康评分或 bias/gravity 估计，只能累加 `invalid_sample_count` 并使诊断进入 WARN；预积分观测、gyro bias 和 gravity direction 任一字段非有限时必须忽略或计入 `preintegration_rejected_updates`，不得更新 last sample、速度、位置、角速度或重力方向，防止 IMU 污染穿透到 `/lio/state_predict`、局部预测先验和后续注册门控。

v3.40 补充约束：`machine_state_manager` 的 PLC Modbus parser 必须在读取寄存器前校验配置：`left_track_register_index/right_track_register_index/status_register_index >= 0`，`cutting_on_bit` 位于 `[0,15]`，`valid_bit` 只能为负数禁用或位于 `[0,15]`，`track_speed_scale` 必须有限且大于 0。配置非法时必须输出不可用 signals 并通过 `parser_config_valid=false` 诊断下沉，不得访问负索引或发布 NaN 轨速。`classifyMachineState()` 必须拒绝非有限轨速、雷达速度、IMU 振动和非法阈值配置，统一返回 `CONFLICT`，保证污染工况输入不会绕过位姿冻结或稳定图写入门控。

v3.41 补充约束：`mapping_control` 的策略配置必须满足 `section_spacing_m/control_anchor_spacing_m > 0`、`weak_observability_threshold` 位于 `[0,1]`、`max_weak_delta_m >= 0` 且 `max_nominal_delta_m >= max_weak_delta_m`，所有字段必须有限。`decideMappingControl()` 的请求增量、观测度、当前链距和最近截面链距也必须有限，观测度必须位于 `[0,1]`；未知工况、非法配置或非法输入必须输出 `REJECT`、`accepted_delta_m=0`、禁止稳定图写入和截面采样。`needsControlAnchor()` 对非法链距或配置必须返回 false；`gradeSection()` 必须拒绝非有限完整度、非有限/负 RMSE 和越界完整度，避免污染值产生 A/B 质量等级。

v3.42 补充约束：`section_manager` 的截面提取必须在切片前校验 `chainage_m`、`slice_thickness_m`、`angle_bins`、`min_points`、矩形宽高和点坐标；非法配置或非有限链距必须返回空的 C 级观测，非有限点不得进入完整度、RMSE 或点云输出。截面评分必须拒绝非有限/越界完整度和非有限/负 RMSE；历史替换不得接受非有限链距、非法质量或污染指标；CSV 导出必须拒绝非法范围、非法最低质量、非有限数值、负 RMSE、非法质量、空点数和包含逗号/分号/换行的文本字段，防止截面成果污染 session evidence 和最终验收入口。

v3.43 补充约束：`lidar_fusion` 的三雷达融合诊断和重叠区一致性计算不得接受非法数值继续生成健康外观。`computeOverlapResidual()` 的 `max_pair_distance` 必须为有限正值，否则返回零配对残差；节点配置中的 `sync_slop/tf_timeout/voxel_leaf_size/diagnostics_period/overlap_pair_distance/overlap_warn_rmse/overlap_fail_rmse` 必须为有限且满足非负/正值约束，非法值必须重置为保守默认。`/diagnostics/lidar_fusion` 分号键值 payload 必须把非有限或负的 sync span、overlap RMSE/max 归零，并拒绝 `overlap_status`、输入 frame 文本中的分号、换行或回车污染，防止融合诊断、重叠区 RMSE/max/status 和后续 time/status、replay/HIL 证据链被污染。

v3.44 补充约束：`lio_time_manager` 的传感器时间、设备时间和 PPS 事件观测入口必须在进入统计窗口前校验时间戳。`SensorTimeTracker::observe()` 必须拒绝非有限 `sensor_stamp` 或 `receipt_time`；`ClockOffsetEstimator::observe()` 必须拒绝非有限 `device_time` 或 `host_time`；`PpsEventTracker::observe()` 必须拒绝非有限 `event_stamp` 或 `receipt_time`。被拒绝样本不得增加 sample_count、不得触发时间回退、不得更新最新 latency/offset/interval，防止 NaN/Inf 时间戳穿透到 `/time/status`、time sync 证据、PPS/PTP wiring 证据和最终 `field_acceptance_status` gate。

v3.45 补充约束：`lio_time_manager` 的 stale 判定不得把非有限状态时间解释为 fresh。`SensorTimeTracker::status(now)`、`PpsEventTracker::status(now)` 和 `DiagnosticFeedTracker::status(now)` 遇到非有限 `now`、非有限 stale 阈值或负 stale 阈值时必须保守输出 stale；`DiagnosticFeedTracker::observe()` 遇到非有限 `receipt_time` 时必须忽略该条 diagnostics，不得更新 `received/message/level/publish_rate_hz/last_observed_time`。该约束保证 `/time/status` 中 IMU/PLC/PPS 的 fresh/stale 证据不会被 NaN 状态时间、非法阈值或污染 receipt time 误放行。

v3.46 补充约束：`lio_eval_tools` 对长稳证据中的“必须为 0”字段不得使用宽松整数归一化。`runtime_stability_summary.txt` 的 `disk_failures/watchdog_failures/health_failures`、`runtime_stability_run.log` 的 `exit_status/capture_exit_status`、以及 `field_acceptance_report.txt` 下沉的 `runtime_stability_disk_failures/runtime_stability_watchdog_failures/runtime_stability_health_failures` 必须字面量严格等于 `0`。`00`、`000`、带符号、空白或其它污染形式都必须使 `runtime_stability_status`、`runtime_stability_run_log_status` 或 `field_acceptance_status` 保持 FAIL，防止污染文本被数值解析归一化为合格证据。

v3.47 补充约束：`lio_eval_tools` 的 evidence manifest metrics report 校验必须对每条非空、非分隔记录执行重复 key fail-closed。首条 summary、当前 session/scenario 的匹配记录、恢复时间明细行以及其它分号键值行中，只要出现任一重复 key，无论该 key 是否是 `overall/total_records/failed_records/session/scenario/status/failed_checks/recovery_time_s` 等关键字段，都必须使 `metrics_status=FAIL`，并进一步使 metrics_report 来源的 `power_loss_resume_status` 和最终 `field_acceptance_status` 保持 FAIL，防止污染审计字段或非关键文本字段被忽略后制造完整验收外观。

v3.48 补充约束：`lio_eval_tools` 的 evidence manifest 行式 `key=value` 证据校验不得只在关键字段重复时 fail closed。`time_sync.txt`、`pps_ptp_wiring_verified.txt`、`runtime_health.txt`、`runtime_deployment.txt`、`runtime_stability_summary.txt`、`runtime_stability_run.log`、`power_loss_resume_verified.txt` 和 `field_acceptance_report.txt` 中的任一重复 key，包括 `operator` 等非关键审计字段，都必须使对应 `time_sync_status/pps_ptp_wiring_status/runtime_health_status/runtime_deployment_status/runtime_stability_status/power_loss_resume_status/field_acceptance_status` 保持 FAIL。最终证据包不得接受含重复行式 key 的污染证据文件，即使所有必需关键字段仍显示为 PASS。

v3.49 补充约束：`lio_eval_tools` 的 evidence manifest 顶层分号键值记录必须保留重复 key 状态。`parseEvidenceManifestRecord()` 不得只抽取必需字段后丢弃 `operator` 等非必需字段的重复污染；`evaluateEvidenceBundle()` 必须把该状态下沉为 `manifest_duplicate_keys=true/false`，并要求 `manifest_duplicate_keys=false` 才能输出最终 `evidence_status=PASS`。

v3.50 补充约束：`lio_eval_tools` 的 evidence manifest 内容证据路径必须先通过路径 gate，才允许参与内容 gate。对 metrics report、replay event_file、time sync、PPS/PTP wiring、runtime health/deployment/stability、power-loss resume、section export 和 field acceptance 等会产生子状态的证据，manifest 路径不是合法 bundle 相对路径、为空、哨兵、绝对路径、包含 `.`/`..`/反斜杠/分号/换行/回车或文件不存在时，校验器必须把该证据视为 missing，并且不得读取外部或污染路径内容来生成 `metrics_status=PASS`、`event_file_status=PASS`、`time_sync_status=PASS` 等子 gate PASS。这样即使 bundle 外部存在一份自称 PASS 的报告，也不能补齐当前 session 证据链。

v3.51 补充约束：`record_session.sh` 生成的 `capture_field_acceptance.sh` 中，长稳零值判断必须与 `lio_eval_tools` 证据解析保持同一文本语义。`is_zero_integer()` 只能接受字面量 `0`，不得接受 `00`、`000`、带符号、空白或其它前导零形式；否则 `runtime_stability_summary.txt` 的三类 failure counters 或 `runtime_stability_run.log` 的 `exit_status/capture_exit_status` 会先在脚本侧制造 `field_acceptance_status=PASS` 外观，再被 manifest 侧拒绝，破坏统一 evidence gate 的可解释性。

v3.52 补充约束：`record_session.sh` 生成的 `capture_power_loss_resume.sh` 与 `capture_field_acceptance.sh` 对 metrics report 来源执行脚本级 gate 时，必须与 `lio_eval_tools` 的 `lookupStrictZeroInteger()` 口径一致。`failed_records` 和当前 session/scenario 匹配记录的 `failed_checks` 只能接受字面量 `0`，不得接受 `00`、`000`、带符号、空白或其它前导零形式；否则断电续建 verified 报告和最终 field acceptance 报告会在脚本侧生成 PASS 外观，再被 manifest 侧拒绝，削弱 `field_acceptance_status=PASS` 作为统一入口的证据一致性。

v3.53 补充约束：`record_session.sh` 生成的 `capture_power_loss_resume.sh` 与 `capture_field_acceptance.sh` 的 `metrics_report_passed()` 必须对每条非空分号键值记录执行 duplicate-key fail-closed。首条 summary、当前 session/scenario 匹配记录以及其它 session/scenario 的分号记录中，只要任一 key 重复，即使重复项只是 `operator` 等非关键审计字段，也必须使 metrics 来源的断电续建 verified 报告和最终 field acceptance 报告保持 FAIL，避免脚本侧先生成 PASS 外观再被 `lio_eval_tools` manifest 侧拒绝。

v3.54 补充约束：`record_session.sh` 生成的 `capture_power_loss_resume.sh` 与 `capture_field_acceptance.sh` 使用 `metrics_report_recovery_time()` 下沉恢复时间时，必须把当前 session/scenario 匹配记录块内的污染明细视为整块不可借值。若匹配记录之后出现 `operator=qa;operator=qa`、重复 key、缺失 key 或其它导致分号键值解析失败的明细行，即使后续还有单独的 `recovery_time_s=25`，脚本也不得输出该恢复时间；最终报告必须保留 `recovery_time_s=missing` 并使 metrics 来源的 `power_loss_resume_status` 与 `field_acceptance_status` 保持 FAIL。

v3.55 补充约束：`record_session.sh` 生成的 `capture_pps_ptp_wiring.sh` 与 `capture_field_acceptance.sh` 必须对 `logs/time_sync_status.txt` 执行整文件行式 key 去重校验。任一 `key=value` 行的 key 重复，包括 `operator` 等非关键审计字段重复，都必须使脚本侧 time sync gate 保持 FAIL，并进一步阻断 `pps_ptp_wiring_verified=PASS` 和 `field_acceptance_status=PASS`，避免脚本侧生成 PASS 外观后再被 `lio_eval_tools` evidence manifest 侧拒绝。

v3.56 补充约束：`record_session.sh` 生成的现场证据捕获脚本必须把 v3.48 的行式证据重复 key 语义前移到生成侧。`capture_pps_ptp_wiring.sh` 必须对 `reports/pps_ptp_wiring_confirmation.txt` 执行整文件 key 去重，`capture_power_loss_resume.sh` 必须对 `reports/power_loss_resume_confirmation.txt` 执行整文件 key 去重；`capture_field_acceptance.sh` 必须对 runtime health、runtime deployment、runtime stability summary、runtime stability run log、PPS/PTP verified 和 power-loss verified 行式输入执行整文件 key 去重。任一非关键审计字段重复都必须使对应 `*_keys_status=FAIL`，并阻断子 gate 与最终 `field_acceptance_status=PASS`，避免人工源文件或中间 verified 报告先制造 PASS 外观再被 manifest 侧拒绝。

v3.57 补充约束：最终 evidence manifest 校验器必须复核 v3.56 生成侧下沉的 keys 状态，不能只相信各子 gate 自报 PASS。`field_acceptance_report.txt` 必须显式下沉 `time_sync_keys_status=PASS`、`runtime_deployment_keys_status=PASS`、`runtime_health_keys_status=PASS`、`runtime_stability_summary_keys_status=PASS`、`runtime_stability_run_log_keys_status=PASS`、`power_loss_resume_keys_status=PASS` 和 `pps_ptp_wiring_keys_status=PASS`；`pps_ptp_wiring_verified.txt` 必须显式包含 `wiring_confirmation_keys_status=PASS`；当 `power_loss_resume_verified.txt` 来源为 `manual_file` 时，还必须显式包含 `power_loss_resume_confirmation_keys_status=PASS`。缺失或非 PASS 都必须使对应子状态、最终 `field_acceptance_status` 和 `evidence_status` 保持 FAIL。

v3.58 补充约束：PPS/PTP wiring 独立证据必须复核人工确认源文件总字段的下沉结果。`pps_ptp_wiring_verified.txt` 中的 `wiring_confirmation=PASS` 不能单独代表人工接线源文件通过，`lio_eval_tools` evidence manifest 还必须要求 `wiring_confirmation_overall=PASS` 和 `wiring_confirmation_keys_status=PASS` 同时成立。缺失 `wiring_confirmation_overall` 或该字段非 PASS 时，即使 `pps_wiring_verified/ptp_wiring_verified/wiring_verified_by/wiring_verified_at` 均合法，`pps_ptp_wiring_status`、最终 `field_acceptance_status` 和 `evidence_status` 仍必须保持 FAIL。

v3.59 补充约束：最终 field acceptance 不能只依赖独立 PPS/PTP wiring gate 已经复核 `wiring_confirmation_overall=PASS`。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须从 `reports/pps_ptp_wiring_verified.txt` 读取并下沉 `wiring_confirmation_overall`，且只有该字段严格等于 `PASS` 时才允许 `pps_ptp_wiring_verified=PASS` 和最终 `field_acceptance_status=PASS`；`lio_eval_tools` evidence manifest 也必须要求 `field_acceptance_report.txt` 自身包含 `wiring_confirmation_overall=PASS`。缺失或非 PASS 都必须使最终 `field_acceptance_status` 与 `evidence_status` 保持 FAIL，避免过期或伪造的最终报告绕过独立 wiring 证据。

v3.60 补充约束：最终 field acceptance 不能只下沉 PPS/PTP wiring 人工确认总字段，也必须下沉并复核 `wiring_confirmation_keys_status=PASS`。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须从 `reports/pps_ptp_wiring_verified.txt` 读取 `wiring_confirmation_keys_status`，缺失时写为 `missing`，且只有该字段严格等于 `PASS` 时才允许 `pps_ptp_wiring_verified=PASS` 和最终 `field_acceptance_status=PASS`；`lio_eval_tools` evidence manifest 也必须要求 `field_acceptance_report.txt` 自身包含 `wiring_confirmation_keys_status=PASS`。缺失或非 PASS 都必须使最终 `field_acceptance_status` 与 `evidence_status` 保持 FAIL，避免人工确认源文件存在重复 key 污染时被最终报告绕过。

v3.61 补充约束：最终 field acceptance 不能只下沉 PPS/PTP wiring 人工确认总字段和 keys 状态，也必须下沉并复核独立 verified 报告中的 `wiring_confirmation=PASS`。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须从 `reports/pps_ptp_wiring_verified.txt` 读取 `wiring_confirmation`，缺失时写为 `missing`，且只有该字段严格等于 `PASS` 时才允许 `pps_ptp_wiring_verified=PASS` 和最终 `field_acceptance_status=PASS`；`lio_eval_tools` evidence manifest 也必须要求 `field_acceptance_report.txt` 自身包含 `wiring_confirmation=PASS`。缺失或非 PASS 都必须使最终 `field_acceptance_status` 与 `evidence_status` 保持 FAIL，避免独立 wiring 报告的人工确认状态被最终报告绕过。

v3.62 补充约束：最终 field acceptance 不能只下沉断电续建人工确认总字段，也必须在 `power_loss_resume_source=manual_file` 时下沉并复核 `power_loss_resume_confirmation_keys_status=PASS`。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须从 `reports/power_loss_resume_verified.txt` 读取该字段，缺失时写为 `missing`，且只有该字段严格等于 `PASS` 时才允许 `power_loss_resume_status=PASS` 和最终 `field_acceptance_status=PASS`；`lio_eval_tools` evidence manifest 也必须要求最终报告自身包含该字段且为 PASS。缺失或非 PASS 都必须使最终 `field_acceptance_status` 与 `evidence_status` 保持 FAIL，避免人工确认源文件存在重复 key 污染时被最终报告绕过。

v3.63 补充约束：`slam_backend_manager` 的回环几何精验证必须把非有限几何和非法阈值作为拒绝条件。`makeGeometrySummary()` 只允许有限 `x/y/z` 点进入 centroid/span 摘要；`verifyLoopGeometry()` 必须要求 `min_geometric_score` 为有限 `[0,1]` 且 `max_centroid_distance_m` 为有限非负值。任一 NaN/Inf 坐标、非有限摘要或非法阈值都必须返回 rejected，不能让 NaN 比较绕过几何包络精验证并污染 `/backend/loop_verified`、稳定图晋升或断电恢复锚点。

v3.64 补充约束：`lio_session_manager` 的稳定锚点恢复精配准必须在进入多尺度 ICP 和 KD-tree inlier 统计前校验 `current_recovery_cloud` 与 `stable_anchor_cloud`。任一输入点云为空、点数不足或包含 NaN/Inf 坐标时，`estimateStableAnchorAlignment()` 必须保持默认 rejected alignment，不得依赖 ICP 内部过滤后仍生成 `converged=true`、低 RMSE 或高 inlier ratio 的断电恢复证据，避免损坏锚点云污染 `RECOVER_WITH_STABLE_ANCHOR`。

v3.65 补充约束：`lio_session_manager` 的 manifest 快照和 WAL 回放恢复必须绑定当前 session。`recoverManifest(root, session_id, ...)` 读取 `manifest.json` 或 `session.wal` 中的 `manifest_snapshot` 时，候选 manifest 的 `session_id` 必须与请求恢复的 `session_id` 完全一致，且 `created_at/updated_at/chainage_m/last_stable_pose/wal_seq` 等关键字段必须为有限合法值。错 session、空 session、非有限数值或负 WAL 序号的 snapshot 必须跳过，不能覆盖同一 WAL 中更早但有效的当前 session snapshot，避免断电续建时把其它会话的稳定姿态恢复到当前会话。

v3.66 补充约束：`slam_backend_manager` 的位姿图优化必须严格解释协方差 stddev 字段。`LoopConstraint::stddev_m`、`PoseGraphConstraint6D::translation_stddev_m` 和 `rotation_stddev_deg` 只能使用 `-1` 表示“未设置，使用默认权重”，或使用严格正且有限的实测标准差；`0`、其它负数、NaN/Inf 都必须使图优化输入 fail closed。禁止把畸形协方差字段通过 `positiveOrDefault()` 回退为默认值后继续参与一维链距图或 6DoF Ceres/轻量优化，避免伪造高置信约束或破坏协方差加权证据语义。

v3.67 补充约束：`tca_manager` 的 JSON 台账加载必须 fail closed。`TcaLedger::load()` 读取 malformed ledger 时必须清空当前锚点集合并直接返回，不得向调用方抛出异常；单条 anchor 的 `chainage_m/center/context_signature/height_m` 等字段解析失败时必须跳过该条记录，不得中断其它合法锚点加载。损坏 TCA 台账只能导致本次 TCA 匹配不可用，不能导致断电恢复锚点读取、local/TCA/global 三级恢复策略或现场验收证据链崩溃。

v3.68 补充约束：`tca_manager` 的 TCA 检测和上下文签名必须在使用点云前校验所有 `PointXYZI` 样本。`detectReflectiveTargets()` 与 `buildContextSignature()` 只允许有限 `x/y/z/intensity` 进入高反阈值筛选、空间聚类、centroid 计算和 ring 计数；任一 NaN/Inf 坐标或强度污染都必须返回空检测或空签名。禁止从部分合法点中继续拼出 TCA 候选，避免坏点云绕过上下文验证并污染断电续建的 TCA 恢复输入。

v3.69 补充约束：`tca_manager` 的 TCA 强度字段必须满足物理非负语义。`PointXYZI::intensity` 和 `TcaDetectionConfig::intensity_threshold` 只能使用非负有限值；负强度样本必须使当前检测或上下文签名 fail closed，负检测阈值必须使高反检测直接返回空结果。禁止用负阈值把普通低强度点提升为反光标靶候选，也禁止让负强度污染点参与 ring 计数，避免现场驱动解析错误或参数污染制造 TCA 锚点外观。

v3.70 补充约束：`tca_manager` 的 TCA context signature 必须代表真实局部上下文观测。`buildContextSignature()` 在合法 ring 边界下若输入点云为空、所有点都未落入任何 ring，或最终 ring 计数全为 `0`，必须返回空签名；`validContextSignature()` 也必须拒绝全零向量。TCA 台账中的全零上下文 anchor、检测阶段生成的全零签名和匹配请求中的全零签名都不得触发匹配，避免“无上下文验证”被误解释为高相似度 TCA 锚点。

v3.71 补充约束：`tca_manager` 的高反聚类不得单独形成 TCA 检测结果。`detectReflectiveTargets()` 必须在每个 cluster 生成候选中心后立即计算 context signature，并且只有签名非空且满足 v3.70 的真实上下文观测要求时，才允许把该 cluster 放入检测结果。无有效上下文的高反 cluster 必须被跳过，不得通过 `/tca/detection` 发布、不得作为 `/tca/register` 的台账写入来源，也不得进入 `TcaLedger::match()`，确保反光点只提供候选而不是绕过上下文验证的恢复锚点。

v3.72 补充约束：`tca_manager` 的 TCA 锚点台账必须拒绝负物理量。`TcaLedger::addAnchor()`、`TcaLedger::load()` 和 `score()` 共用的 anchor 有效性检查必须要求 `chainage_m >= 0` 且 `height_m >= 0`，同时保持有限值、非空 `anchor_id`、合法中心点和真实 context signature 要求。负链距或负高度锚点只能导致该条锚点被跳过，不能进入匹配结果、稳定恢复锚点或 local/TCA/global 三级恢复策略。

v3.73 补充约束：`tca_manager` 的 TCA 锚点 ID 必须使用安全 token 语义。`anchor_id` 必须非空，只允许字母、数字、`_`、`-` 和 `.`，且不能单独为 `.` 或 `..`；包含分号、空白、换行或其它字符的锚点必须被 `TcaLedger::addAnchor()`、`TcaLedger::load()` 和匹配评分入口跳过。非法 ID 不得进入 `/tca/match` 的分号键值发布文本、JSON 台账、稳定恢复锚点或 local/TCA/global 三级恢复策略，避免污染字段注入和路径段歧义。

v3.74 补充约束：`tca_manager` 的 `/tca/register` 只能注册当前有效检测。节点必须通过可测试的检测缓存管理注册候选：收到非空检测结果时缓存最新候选，收到空检测结果或输入点云缺少 `x/y/z/intensity` 必需字段时必须清空缓存并复位匹配状态；缓存为空、检测中心或上下文无效、或由检测中心推导的链距为负时，`/tca/register` 必须返回失败。上一帧检测不得在当前帧无 TCA 或输入无效后继续写入 JSON 台账、稳定恢复锚点或 local/TCA/global 三级恢复策略。

v3.75 补充约束：`tca_manager` 的 PointCloud2 输入读取必须整帧 fail closed。节点读取点云时必须要求 `x/y/z/intensity` 必需字段各自唯一、类型可读、offset 不越过 `point_step`，且 `point_step/row_step/data` 长度能覆盖声明的有组织或无组织点云；任一必需字段缺失、重复、类型不可读、点记录越界、数据截断、`x/y/z/intensity` 非有限，或 `intensity < 0`，都必须拒绝整帧输入、清空检测缓存并复位匹配状态。节点不得跳过污染点后继续聚类、生成 context signature、发布检测、匹配锚点或允许 `/tca/register` 写入台账，避免坏点云在 TCA 恢复链路中被部分过滤后伪装成有效现场锚点。

v3.76 补充约束：`lio_preprocess` 的连续时间去畸变只能在点内时间字段唯一且无歧义时启用。`findTimeField()` 必须把配置候选 `time/timestamp/t/offset_time` 归一化后整体匹配，且整帧只能命中一个候选字段；重复 `time` 字段、同时存在多个候选别名，或候选列表污染导致的多重命中，都必须返回无可用时间字段。`deskewPointTuples()` 在时间字段歧义时必须保持点坐标不变、不得使用 IMU 角速度旋转点云，并通过 `missing_time_field`/`deskewed_points=0/used_imu=false` 暴露保守状态，防止现场驱动字段重复或污染时把错误点时间静默用于 `/lio/points_deskewed`。

v3.77 补充约束：`lio_preprocess` 的连续时间去畸变必须先验证点 tuple 至少包含 `x/y/z` 三个槽位。即使时间字段唯一且 `pointRelativeTime()` 可解析，缺少完整 XYZ 槽位的点也只能计入 `invalid_point_time`，并保持原 tuple 原样返回；不得访问不存在的 `points[i][1]`、`points[i][2]` 或向 `corrected[i][1]`、`corrected[i][2]` 写入结果。该 gate 用于防止上游 PointCloud2 字段解析、裁剪或字段名污染生成短 tuple 后，在 `/lio/points_deskewed` 中越界读写或制造伪去畸变点。

v3.78 补充约束：`lio_preprocess` 的点云过滤入口必须在计算距离、近机体遮蔽或输出 kept tuple 前验证点 tuple 至少包含 `x/y/z` 三个槽位。短 tuple 必须计入 `input_points`，按 `dropped_nan` 统计并丢弃；不得访问不存在的 `(*it)[0]`、`(*it)[1]`、`(*it)[2]`，也不得把短 tuple 透传到后续连续时间去畸变或 `/lio/points_deskewed`。该约束与 v3.77 共同覆盖过滤阶段和去畸变阶段的短 tuple 污染输入。

v3.79 补充约束：`lio_preprocess` 的过滤配置必须先通过数值合法性 gate。`min_range/max_range` 必须为有限值，且 `min_range >= 0`、`max_range >= min_range`；启用近机体遮蔽时，`body_crop` 的 `x/y/z` 六个边界必须全部有限，并满足各轴 `min <= max`。非法 range 配置必须按 `range` 丢弃，非法 body crop 配置必须按 `body` 丢弃；不得让 NaN/Inf 边界或反序 crop box 因 C++ 比较结果为 false 而把污染点云保留下来。

v3.80 补充约束：`lio_preprocess` 的 PointCloud2 输入读取必须在过滤和连续时间去畸变前整帧校验。`x/y/z` 必需字段必须各自唯一存在；所有字段必须是可读标量数值类型，字段名非空、`count=1`、`offset + sizeof(datatype) <= point_step`；`point_step`、`row_step` 和 `data` 长度必须覆盖 `width*height` 声明的点云布局，并按 `row_step` 读取有组织点云，避免忽略行 padding。任一必需字段缺失/重复、字段不可读、字段越界、行步长不足或数据截断时，节点必须拒绝整帧输入、更新无效点云诊断，不得发布 `/lio/points_deskewed`，也不得让污染字段名进入点时间匹配和 IMU 去畸变。

v3.81 补充约束：`lio_preprocess` 的 PointCloud2 入口必须证明 `x/y/z` 坐标字段可被去畸变结果安全写回。即使整型 `INT32/UINT32/INT16/UINT16/INT8/UINT8` 坐标字段可读为 double，也不得作为 `/lio/points_deskewed` 的输入坐标字段通过；`x/y/z` 必须为 `FLOAT32` 或 `FLOAT64`，否则整帧以 `unwritable_required_field` 拒绝，防止节点接受异常坐标类型后发布未写回修正坐标的输出点云。非坐标辅助字段仍按 v3.80 的可读标量规则处理。

v3.82 补充约束：`lio_preprocess` 的 PointCloud2 reader 在保留点并构造点 tuple 时，必须校验所有读出的字段值为有限值。`x/y/z` 非有限值继续按过滤路径计入 `dropped_nan` 并丢弃该点；但坐标合法且已通过过滤的点，若 `intensity`、`time`、`timestamp`、`ring` 或其它辅助字段读出 NaN/Inf，必须以 `nonfinite_field` 整帧拒绝，清空输出点和点字节缓存。节点不得把非有限辅助字段透传到 `/lio/points_deskewed`，也不得让污染时间字段进入连续时间去畸变或让污染强度字段进入后续 ISC/强度上下文链路。

v3.83 补充约束：`lio_preprocess` 的 PointCloud2 reader 必须要求整帧字段名唯一。`x/y/z` 必需字段重复时继续以 `duplicate_required_field` 拒绝；除此之外，任一辅助字段名重复，包括重复 `time`、`timestamp`、`intensity`、`ring` 或其它自定义字段，都必须在读取点数据和发布 `/lio/points_deskewed` 前以 `duplicate_field` 整帧拒绝。节点不得把重复字段名透传到 `field_names`、点时间匹配、强度上下文或输出 PointCloud2，避免不同 offset 的同名字段在后续链路中被静默解释为同一语义。

v3.84 补充约束：`slam_backend_manager` 的 Scan Context 风格几何描述子和 ISC 风格强度上下文描述子必须对输入点污染整帧 fail closed。几何描述子和基础半径直方图在构造 bins 前必须确认所有输入点 `x/y/z` 坐标有限；强度上下文还必须确认所有输入点 `intensity` 为有限且非负。任一输入点不满足时必须返回空描述子，不得跳过坏点后继续生成候选 bins，也不得把 NaN/Inf 或负强度量化后参与 `descriptorSimilarity()`、Top1/Top2 唯一性判断、全局候选恢复或稳定图晋升。

v3.85 补充约束：`slam_backend_manager` 的全局候选选择不得只信任上游描述子生成器。进入 `chooseLoopCandidate()` 的当前 keyframe 与候选 keyframe 必须具备有限 `chainage_m`，几何 descriptor 和可选强度 descriptor 的所有 bin 必须为非负整数；当前 keyframe 污染时直接返回无候选，候选 keyframe 污染时跳过该候选。`descriptorSimilarity()` 本身也必须在遇到负 bin 时返回 0，防止外部恢复锚点、WAL 回放或台账污染把负 bin 与负 bin 匹配成 1.0 相似度，并绕过链距间隔、Top1/Top2 非唯一拒绝和全局候选恢复门控。

v3.86 补充约束：`slam_backend_manager` 的轻量 ICP 精配准验证必须在迭代前 fail closed。`max_icp_rmse_m`、`min_icp_inlier_ratio`、`icp_inlier_threshold_m`、`icp_iterations` 和 `max_icp_points` 必须合法：RMSE 与 inlier threshold 为有限非负数，inlier ratio 位于 0-1，迭代次数为正整数，最大抽样点数为非负整数；当前点云和候选点云的所有 `x/y/z` 坐标必须有限。非法配置必须返回 `invalid_icp_config`，非法点云必须返回 `invalid_icp_points`，不得让 NaN/Inf 点或污染阈值进入 ICP seed、最近邻、RMSE/inlier ratio 计算，也不得把这类结果作为全局候选恢复或回环精验证通过证据。

v3.87 补充约束：`slam_backend_manager` 的稳定图晋升策略必须校验自身最低质量阈值。`StableMapPolicy` 的 `min_quality`/`min_stable_quality` 只能是 A/B/C；非法值不得被 `qualityRank()` 解释为最差等级后放行 A/B/C 截面，而必须使 `canPromote()` 对所有候选返回 false。该约束独立于 section quality 输入文本解析和稳定图台账写入校验，防止参数污染把稳定图晋升门槛静默扩大，进而污染断电恢复锚点和长期稳定图治理。

v3.88 补充约束：`slam_backend_manager` 的稳定图台账 `keyframe_id` 必须是安全 token。读取台账、晋升写入和保存输出均只能接受字母、数字、下划线、短横线和点号组成的非空 ID，且不得为 `.` 或 `..`；包含分号、换行、回车、斜杠、空白或路径段污染的条目必须跳过，非法晋升不得删除或覆盖已有合法稳定锚点，防止稳定图台账污染诊断字段、恢复锚点引用或后续证据导出。

v3.89 补充约束：`slam_backend_manager` 的一维链距位姿图和可调 Ceres 6DoF 位姿图必须对节点 ID 和约束端点 ID 执行安全 token 校验。节点 `keyframe_id` 与约束 `from/to_keyframe_id` 只能使用字母、数字、下划线、短横线和点号，且不得为 `.` 或 `..`；包含分号、换行、回车、斜杠、空白或其它污染字符时，整张图必须返回空结果，不得输出携带污染 key 的优化位姿或链距，避免污染回环治理、稳定图晋升依据和断电恢复锚点引用。

v3.90 补充约束：`slam_backend_manager` 的全局候选选择必须对当前 keyframe 和候选 keyframe 的 `keyframe_id` 执行安全 token 校验。当前 keyframe ID 污染时 `chooseLoopCandidate()` 必须直接返回无候选；候选 keyframe ID 污染时必须跳过该候选，即使其链距、几何 descriptor 或强度 descriptor 完全匹配也不得输出污染 ID。该约束防止分号键值片段、路径段、空白或换行污染进入 `/backend/loop_candidate`、几何/ICP 精验证、稳定图晋升依据或诊断文本。

v3.91 补充约束：`slam_backend_manager` 的 ROS 节点不得在私有回调内裸读 `sensor_msgs/PointCloud2`。后端 submap reader 必须在生成 Scan Context/ISC 描述子、几何摘要和 ICP 样本前校验 `x/y/z` 必需字段、可选 `intensity` 字段、字段名唯一性、字段 offset/count/datatype、`point_step/row_step/data` 覆盖关系和 organized cloud 行步长；截断数据、重复字段名、字段越界、空点云、非有限坐标、非有限强度或负强度必须整帧 fail closed。`slam_backend_node` 遇到读取失败必须发布 `invalid_submap:<reason>` WARN 诊断并跳过本次 keyframe 生成，防止畸形 submap 污染描述子、候选回环、ICP 精验证、稳定图晋升和诊断文本。

v3.92 补充约束：`lio_session_manager` 不得假设后端稳定图台账一定来自已校验写入路径。`loadStableRecoveryAnchor()` 读取稳定图恢复锚点时必须对 `keyframe_id` 执行安全 token 校验，只允许字母、数字、下划线、短横线和点号，且不得为 `.` 或 `..`；包含分号、换行、回车、斜杠、空白或其它污染字符的条目必须跳过。`decideStableAnchorRecovery()` 对直接传入的 `StableRecoveryAnchor` 也必须复用同一门槛，污染锚点必须返回 `invalid_stable_anchor`，不得进入 `RECOVER_WITH_STABLE_ANCHOR`，避免污染断电恢复动作、WAL `stable_anchor` 记录和现场恢复证据。

v3.93 补充约束：`lio_session_manager` 的 `session_id` 是 session 目录、manifest 和 WAL 归属边界，不能只按普通文本处理。`createSession()`、`loadManifest()`、`recoverManifest()` 和 `appendWal()` 在拼接 session 目录前必须要求 `session_id` 为安全 token：只允许字母、数字、下划线、短横线和点号，且不得为空、`.` 或 `..`；包含分号、换行、回车、斜杠、空白或其它污染字符时必须 fail closed。`validManifestForSession()` 必须同时校验请求 session 与候选 manifest 的 session ID token 合法性，`listSessions()`/`latestSession()` 扫描到污染目录或污染 manifest 时必须跳过，避免断电恢复、WAL 回放或最新 session 选择把污染 ID 解释为合法会话并覆盖真实稳定基线。

v3.94 补充约束：`lio_session_manager` 的 manifest `state` 会通过 `/session/status`、恢复服务响应上下文和 WAL 证据链传播，不能只要求非空。`validManifestForSession()` 必须只接受当前节点实际写入的 `ACTIVE` 与 `TEMP` 两个状态；manifest 文件或 WAL `manifest_snapshot` 中出现空值、分号键值片段、换行、回车、空白污染或未知状态时，必须跳过该 candidate，不得覆盖同一 WAL 中更早的合法 snapshot，也不得被 `listSessions()`/`latestSession()` 选为最新会话，避免磁盘污染把伪状态注入断电恢复链路或现场证据文本。

v3.95 补充约束：`lio_session_manager` 的 manifest 写入路径必须与恢复路径使用同一合法性 gate。`writeManifest()` 和 `commitManifestSnapshot()` 在创建临时 manifest、rename、组装 WAL JSON 或调用 `appendWal()` 前，必须复验 `session_id` token、`state` 枚举、`created_at/updated_at/chainage_m/last_stable_pose` 有限值和非负 `wal_seq`；任一字段污染时必须抛出 `invalid_argument`，不得覆盖既有合法 `manifest.json`，也不得追加污染 `session.wal`。该约束防止直接 API、未来调用方或异常状态机把读侧已会拒绝的污染 manifest 主动写进现场恢复证据。

v3.96 补充约束：`lio_session_manager` 的 WAL 是一行一条 JSON record，`appendWal()` 不能信任调用方提供的字符串已经满足行边界契约。任何包含换行 `\n` 或回车 `\r` 的 `json_record` 必须在创建目录、打开 `session.wal` 或写入前抛出 `invalid_argument`；节点自身生成的 `manifest_snapshot` 与 `recover` 记录必须保持单行 JSON。该 gate 用于防止直接 API 调用把一次追加拆成多条 WAL 记录，插入伪 `manifest_snapshot` 并在断电恢复回放时污染最新 session manifest。

v3.97 补充约束：`lio_session_manager` 的 WAL record 不得只是“单行字符串”。`appendWal()` 在打开 `session.wal` 前必须把 `json_record` 解析为 JSON object，并要求 `event` 字段存在且属于当前 WAL 支持的 `manifest_snapshot` 或 `recover`；坏 JSON、缺失 `event`、`event` 携带分号键值片段或其它未知事件名时必须抛出 `invalid_argument`，且不得创建或追加 WAL 文件。该约束防止无语义单行文本、伪事件或污染事件进入断电恢复证据链；合法但内容语义不可信的 `manifest_snapshot` 仍由恢复侧 manifest gate 跳过。

v3.98 补充约束：`lio_session_manager` 的 WAL JSON record 必须无歧义。`appendWal()` 在解析 JSON 后必须递归检查同级非空 key，不允许重复 `event`、重复 `manifest.session_id` 或任何嵌套重复字段；发现 duplicate key 时必须在创建或追加 `session.wal` 前抛出 `invalid_argument`。该约束防止 Boost property_tree、其它 JSON parser 或离线证据审计工具在遇到重复 key 时取首值/末值不一致，从而把同一条 WAL record 解释成不同恢复事件或不同 manifest。

v3.99 补充约束：`lio_session_manager` 的 `loadManifest()` 不能只负责 JSON 解析，也必须承担 manifest 直接读取边界。读取 `manifest.json` 后必须先拒绝 duplicate key，再要求候选 manifest 与请求 `session_id` 完全一致，且 `session_id` token、`state` 枚举、`created_at/updated_at/chainage_m/last_stable_pose` 有限值和非负 `wal_seq` 全部合法；不满足时必须抛出 `invalid_argument`，不得把污染 manifest 返回给直接 API 调用方。`recoverManifest()` 仍可捕获该异常并回落到合法 WAL snapshot，确保坏 manifest 文件不会阻断断电回放恢复。

v4.00 补充约束：`lio_session_manager` 的 WAL 回放路径必须与写入 gate 对 duplicate key 保持一致。`recoverManifest()` 逐行读取磁盘 `session.wal` 时，任何可解析但含同级重复 JSON key 的 record 都必须跳过，不得读取其 `event` 或 `manifest` 字段，也不得覆盖同一 WAL 中更早的合法 snapshot。该约束覆盖断电、磁盘污染或外部工具绕过 `appendWal()` 后留下的歧义 WAL 行，防止重复 `event`、重复 `manifest.session_id` 等记录在回放时被 Boost property_tree 解释为合法 `manifest_snapshot`。

v4.01 补充约束：`lio_session_manager` 的 `snapshot_event` 也是 WAL 证据文本，不能接受任意字符串。`commitManifestSnapshot()` 在组装 `manifest_snapshot` JSON 前必须要求事件名为安全 token：只允许字母、数字、下划线、短横线和点号，且不得为空、`.` 或 `..`；包含分号、换行、回车、斜杠、空白或其它污染字符时必须抛出 `invalid_argument`，不得创建或追加 `session.wal`，也不得覆盖既有 `manifest.json`。该约束保证 `create_session`、`manual_snapshot`、`periodic_snapshot`、`stable_pose` 等合法事件可写入，同时防止直接 API 调用把 `field_acceptance_status=PASS` 一类片段塞进 WAL 证据链。

v4.02 补充约束：`appendWal()` 作为公开 WAL 写入口，不能只校验 JSON 行和顶层 `event`。写入 `manifest_snapshot` 时必须要求 `snapshot_event` 为安全 token、`stamp` 为有限非负时间、`manifest` 与目标 `session_id` 完全匹配且通过 manifest 写侧合法性校验；写入 `recover` 时必须要求 `stamp` 有限非负，`base_action/action` 属于既定恢复动作集合，`stable_anchor_decision.reason` 属于既定恢复原因集合，`translation_m` 有限非负，`RECOVER_WITH_STABLE_ANCHOR` 必须与 `accepted=true` 和合法稳定锚点一致。任何分号、换行、空白、非法状态、污染 keyframe 或不完整恢复记录都必须在打开 `session.wal` 前抛出 `invalid_argument`。回放路径仍保留对磁盘既有污染 WAL 行的跳过能力，保证断电后不会让外部绕写的坏记录覆盖合法 snapshot。

v4.03 补充约束：`appendWal()` 的 WAL payload gate 还必须验证同一记录内部证据一致性。`manifest_snapshot.stamp` 必须与内嵌 `manifest.updated_at` 数值一致，不允许一条 WAL 行同时声明两个更新时间；`recover` 记录在 `stable_anchor_decision.accepted=false` 时必须保持 `action == base_action`，且不得使用 `stable_anchor_alignment_accepted` 原因；只有 `accepted=true`、`reason=stable_anchor_alignment_accepted`、`base_action` 不是 `CREATE_NEW_SESSION` 且稳定锚点合法时，`action` 才能提升为 `RECOVER_WITH_STABLE_ANCHOR`。这些一致性 gate 防止外部直接追加矛盾 WAL 行制造断电恢复动作或时间戳证据歧义。

v4.04 补充约束：`lio_session_manager` 的 manifest 时间字段不仅要有限，还必须符合单调证据语义。`created_at` 必须为非负时间，`updated_at` 必须大于等于 `created_at`；`writeManifest()`、`commitManifestSnapshot()`、`loadManifest()`、`appendWal()` 的 `manifest_snapshot` payload gate 以及 `recoverManifest()` WAL 回放都必须复用该边界。负创建时间、更新时间早于创建时间或时间倒置的 WAL snapshot 必须 fail closed，不能覆盖 `manifest.json`、不能创建或追加 `session.wal`，也不能在断电恢复时覆盖更早合法 snapshot。

v4.05 补充约束：`decideTieredRecovery()` 作为 `/session/recover` 的三级恢复策略入口，也必须复用 manifest 合法性边界，不能只检查 `updated_at` 是否有限。输入 manifest 的 `session_id/state/created_at/updated_at/chainage_m/last_stable_pose/wal_seq` 必须整体合法，且调用时刻 `now` 必须不早于 `manifest.updated_at`；污染 state、时间倒置、未来时间戳或非法 manifest 即使恢复分数很高也必须返回 `CREATE_TEMP_SESSION`，避免直接调用方或异常时钟把不可信 manifest 恢复为稳定会话。

v4.06 补充约束：`lio_eval_tools` 的 ISO-8601 seconds 证据时间戳不能只做格式和数值范围校验，还必须符合真实日历日期。`validIso8601SecondsTimestamp()` 必须按月份天数和闰年规则验证 `timestamp`、`wiring_verified_at`、`resume_verified_at`、`runtime_stability_run_log_started_at/finished_at` 等字段；`2026-02-31T00:00:00+08:00`、非闰年 2 月 29 日、4/6/9/11 月 31 日等不可能日期必须使对应证据和最终 `field_acceptance_status` fail closed，防止人工审计时间或最终验收时间用格式正确但日历无效的文本伪造 PASS。

v4.07 补充约束：`record_session.sh` 生成的现场证据捕获脚本必须与 `lio_eval_tools` 使用同一 ISO-8601 seconds 真实日历口径。`capture_pps_ptp_wiring.sh` 的 `wiring_verified_at`、`capture_power_loss_resume.sh` 的 `resume_verified_at`，以及 `capture_field_acceptance.sh` 读取的 `runtime_stability_run_log_started_at/finished_at`、`wiring_verified_at`、`resume_verified_at` 都必须按月份天数和闰年规则校验；`2026-02-31T00:00:00+08:00` 这类格式正确但日历不存在的审计时间必须在脚本生成侧保持 FAIL，避免先写出 `pps_ptp_wiring_verified=PASS`、`power_loss_resume_status=PASS` 或 `field_acceptance_status=PASS` 外观后再由最终 manifest 侧拒绝。

v4.08 补充约束：runtime stability run log 的 `started_at/finished_at` 不能只要求格式正确、日期真实，还必须满足时间顺序语义。`lio_eval_tools` 必须把 ISO 秒级时间戳换算到 UTC 秒级并要求 `finished_at >= started_at`；`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须在脚本侧执行同一顺序 gate。`finished_at` 早于 `started_at` 时，`runtime_stability_run_log_status`、`runtime_stability_status` 和最终 `field_acceptance_status` 必须保持 FAIL，防止倒置 run log 时间制造负时长或过期长稳 PASS 外观。

v4.09 补充约束：runtime stability 的 24 h 证明不能只依赖 `samples * interval_s` 声明出的 `runtime_stability_duration_h`。`runtime_stability_run.log` 的真实起止时间必须覆盖配置时长，并只允许最多 `min(interval_s, 60 s)` 的调度容差，用于兼容最后一次采样后不再 sleep 的默认脚本行为；若 run log 仅跨越 1 秒却声明 `samples=1/interval=86400`，`runtime_stability_run_log_status`、`runtime_stability_status`、`field_acceptance_status` 和最终 `evidence_status` 必须保持 FAIL。`capture_field_acceptance.sh` 还必须下沉 `runtime_stability_run_log_elapsed_s`、`runtime_stability_run_log_required_elapsed_s` 和 `runtime_stability_run_log_duration_status`，便于现场证据包解释失败原因。

v4.10 补充约束：runtime stability CSV 不能只证明行数和 PASS 状态。`sample` 列必须是严格正整数并等于采样序号，`timestamp` 列必须是符合 ISO 秒级格式且日历日期真实的时间戳；`2026-02-31T00:00:00+08:00`、单行 `sample=2` 或任一跳号采样都必须使 `runtime_stability_csv_status`、`runtime_stability_status`、`field_acceptance_status` 和最终 `evidence_status` 保持 FAIL。

v4.11 补充约束：runtime stability CSV 必须与本 session 的长稳 run log 时间窗口闭环。每条采样 `timestamp` 在格式、真实日期和序号语义通过后，还必须落在 `runtime_stability_run.log` 的 `started_at <= timestamp <= finished_at` 闭区间内；若 CSV 采样来自 run log 之前、之后或另一段运行窗口，即使 CSV 行状态全部 PASS 且 run log 本身覆盖 24 h，`runtime_stability_csv_status`、`field_acceptance_status` 和最终 `evidence_status` 也必须保持 FAIL。

v4.12 补充约束：最终 field acceptance 报告自身的 `timestamp` 不能只证明格式和真实日期合法，还必须证明报告生成时间不早于已纳入最终 PASS 的关键证据时间。`lio_eval_tools` 必须把最终报告 `timestamp`、`runtime_stability_run_log_finished_at`、`wiring_verified_at` 以及 manual_file 来源的 `resume_verified_at` 全部换算为 UTC 秒级比较；`capture_field_acceptance.sh` 必须在生成报告时输出 `field_acceptance_timestamp_status` 并纳入最终 gate。若最终报告时间早于 24 h 长稳完成、PPS/PTP 接线确认或 manual 断电续建确认，即使其它证据字段均为 PASS，`field_acceptance_status` 和最终 `evidence_status` 也必须保持 FAIL。

v4.13 补充约束：最终 field acceptance 报告不能只由校验器重算时间顺序，还必须显式下沉 `field_acceptance_timestamp_status=PASS`。`lio_eval_tools` manifest 侧必须同时要求该字段为 PASS 并独立复算 `timestamp >= runtime_stability_run_log_finished_at`、`timestamp >= wiring_verified_at`，以及 manual_file 来源下 `timestamp >= resume_verified_at`；缺失、非 PASS 或与实际时间顺序矛盾时，`field_acceptance_status` 和最终 `evidence_status` 都必须保持 FAIL。

v4.14 补充约束：最终 field acceptance 报告下沉的 runtime stability run log 时长字段必须与独立 run log 真实时间一致。`lio_eval_tools` manifest 侧必须把 `runtime_stability_run_log_started_at/finished_at` 换算为 UTC 秒级 elapsed，并按 `samples * interval_s - min(interval_s, 60 s)` 重算 required elapsed；最终报告中的 `runtime_stability_run_log_elapsed_s` 和 `runtime_stability_run_log_required_elapsed_s` 必须为严格非负整数字面量且与重算值完全一致，`runtime_stability_run_log_duration_status` 必须严格等于 `PASS`。缺失、非 PASS、前导零/污染、数值不一致或 elapsed 小于 required 时，`field_acceptance_status` 和最终 `evidence_status` 都必须保持 FAIL。

v4.15 补充约束：最终 field acceptance 报告下沉的 `runtime_stability_duration_h` 不能只满足 `>= 24 h` 或小于等于 `samples * interval_s / 3600`，还必须与最终报告中已绑定 summary 的 `runtime_stability_samples/runtime_stability_interval_s` 推导值一致。`lio_eval_tools` manifest 侧按 `samples * interval_s / 3600` 重算小时数，并按 `capture_field_acceptance.sh` 的两位小数下沉语义比对；若最终报告把 48 h、30 h 等长稳 summary 改写为 24 h 外观，即使独立 CSV、summary 和 run log 均 PASS，`field_acceptance_status` 和最终 `evidence_status` 也必须保持 FAIL。

v4.16 补充约束：独立 time sync 捕获文件不能只提供可被下游重新推导的子字段，还必须显式输出 `time_sync_status=PASS`。`capture_time_sync.sh` 必须在 `logs/time_sync_status.txt` 中写入该总状态；`capture_pps_ptp_wiring.sh`、`capture_field_acceptance.sh` 和 `lio_eval_tools` manifest 侧必须同时要求独立 time sync 总字段为 PASS、`capture_status=CAPTURED`、`pps_status=PASS`、`clock_offset_status=PASS`、topic 合法且数值合法。缺失或非 PASS 时，即使其它子字段看似可推导为 PASS，`time_sync_status`、`pps_ptp_wiring_verified`、`field_acceptance_status` 和最终 `evidence_status` 都必须保持 FAIL。

v4.17 补充约束：独立 time sync 捕获文件的 PASS 不能只依赖手写状态和数值字段，还必须绑定原始 `/time/status` YAML 捕获物。`logs/time_sync_status.txt` 的 `raw` 字段必须是有效 bundle 相对路径或位于当前 evidence bundle 根目录下的绝对路径，且目标必须是非空 regular file；缺失、空文件、`missing/__DUPLICATE_KEY__`、路径逃逸、绝对路径指向 bundle 外部或 raw 文件不存在时，`time_sync_status`、`pps_ptp_wiring_verified`、`field_acceptance_status` 和最终 `evidence_status` 都必须保持 FAIL。

v4.18 补充约束：`record_session.sh` 生成的 `capture_pps_ptp_wiring.sh` 与 `capture_field_acceptance.sh` 必须在脚本侧执行与 evidence manifest 一致的 time sync raw 捕获物 gate。两个脚本都必须要求 `logs/time_sync_status.txt` 的 `raw` 字段为有效文本路径、解析后位于当前 session/evidence bundle 内且目标为非空文件；PPS/PTP wiring 报告必须下沉 `time_sync_raw` 和 `time_sync_raw_status`，最终 field acceptance 报告还必须下沉并复核 `pps_ptp_wiring_time_sync_raw` 和 `pps_ptp_wiring_time_sync_raw_status`。缺失、空文件、路径逃逸或 PPS/PTP wiring 与当前 time sync raw 不一致时，`pps_ptp_wiring_verified`、`field_acceptance_status` 和最终验收入口都必须保持 FAIL。

v4.19 补充约束：`lio_eval_tools` evidence manifest 侧也必须复核脚本下沉的 time sync raw 证据闭环。独立 `pps_ptp_wiring_verified.txt` 必须包含 `time_sync_raw` 且与独立 time sync 证据的 `raw` 原文一致，并包含 `time_sync_raw_status=PASS`；最终 `field_acceptance_report.txt` 必须包含 `time_sync_raw/time_sync_raw_status` 和 `pps_ptp_wiring_time_sync_raw/pps_ptp_wiring_time_sync_raw_status`，且二者都必须与独立 time sync raw 一致。缺失、非 PASS 或下沉 raw 与独立 time sync 原始捕获物不一致时，`pps_ptp_wiring_status`、`field_acceptance_status` 和最终 `evidence_status` 必须保持 FAIL。

v4.20 补充约束：`record_session.sh` 生成的最终 `capture_field_acceptance.sh` 在脚本侧校验规范化 replay/HIL `event_file` 时必须与 `lio_eval_tools` evidence manifest 对齐，对所有非空事件记录执行 `event` 有效文本、`session_id` 安全 token、`scenario` 大写场景 token、`t` 非负数和重复 key fail-closed 校验；即使污染记录不属于当前 session/scenario，也不得被忽略后让 `event_file_status=PASS` 或最终 `field_acceptance_status=PASS`。这避免脚本最终报告自报 PASS、C++ manifest 后续再打 FAIL 的证据口径不一致。

v4.21 补充约束：`record_session.sh` 生成的最终 `capture_field_acceptance.sh` 不能只验证 replay/HIL `event_file` 中指标字段格式合法，还必须按 `lio_eval_tools` 默认验收阈值聚合当前 session/scenario 的事件指标并 fail-closed：`static_drift_m <= 0.05`、`length_error_percent <= 0.5`、`recovery_time_s <= 45.0`、`wrong_loop_count == 0`、`queue_backlog_max <= 10`、`pps_jitter_ms <= 2.0`。超阈值事件不得让 `event_file_status=PASS` 或最终 `field_acceptance_status=PASS`，防止脚本报告与 C++ evidence manifest 在 replay/HIL event_file 语义上分裂。

v4.22 补充约束：`record_session.sh` 生成的最终 `capture_field_acceptance.sh` 在脚本侧解析 replay/HIL `event_file` 时必须与 `lio_eval_tools` 的 `parseReplayEventRecords()` 对齐，trim 后空行和以 `#` 开头的注释行不视为事件记录；带注释的规范化 event_file 只要当前 session/scenario 指标事件合法且通过阈值 gate，`event_file_status` 和 `field_acceptance_status` 可为 PASS；真正的非注释畸形记录仍必须 fail-closed。

v4.23 补充约束：`lio_eval_tools` 解析 replay/HIL `event_file` 时不得静默忽略畸形分号 token。每条非注释事件记录按 `;` 分隔后的 token 必须全部是非空 `key=value`，空 key、无 `=` token、连续分号、前导分号或尾随分号都必须标记为事件结构错误；聚合指标和 evidence manifest 的 `event_file_status` 都必须 fail-closed，保持与 `record_session.sh` 生成脚本侧 awk 解析口径一致。

v4.24 补充约束：`lio_eval_tools` 的 replay/HIL 事件聚合器自身也必须拒绝 `missing` 哨兵文本值。`event/session_id/scenario` 等结构字段进入 `aggregateReplayMetrics()` 前必须通过与 evidence manifest 一致的文本 gate，`event=missing`、`session_id=missing` 或重复 key sentinel 都必须使聚合指标 fail-closed，防止 `validation_report_node` 在未经过最终 manifest 时先生成 PASS 外观指标报告。

v4.25 补充约束：`validation_metrics` 的 `metrics_file` 与 `scenario_validation_thresholds.txt` 分号记录不得静默忽略畸形 token。每条有效记录按 `;` 分隔后的 token 必须全部是非空 `key=value`，空 key、无 `=` token、连续分号、前导分号或尾随分号都必须 fail-closed；metrics 记录毒化对应指标，场景阈值记录毒化阈值覆盖，防止 `validation_report_node` 或最终验收 gate 使用污染指标/阈值生成 PASS 外观报告。

v4.26 补充约束：`lio_eval_tools` 的 evidence manifest 主记录与 metrics report 分号行也不得静默忽略畸形 token。manifest 主记录必须输出并纳入 `manifest_malformed_tokens` 诊断，metrics report 的 summary、record 和 detail 行只要出现无 `=` token、空 key、连续/前导/尾随分号等结构错误，`metrics_status`、`field_acceptance_status` 和最终 `evidence_status` 都必须 fail-closed，防止最终证据入口使用污染 manifest 或污染恢复时间明细生成 PASS。

v4.27 补充约束：`lio_eval_tools` 的逐行 `key=value` 证据文件解析也必须 fail-closed。time sync、PPS/PTP wiring、runtime health/deployment/stability、power-loss resume、runtime stability run log 和最终 field acceptance 等文本证据中，空行可忽略，但任何非空行都必须具备非空 key 和 `=` 分隔；无 `=` 行或空 key 行不得被静默忽略，必须使对应子证据状态以及最终 `field_acceptance_status/evidence_status` 保持 FAIL。

v4.28 补充约束：`record_session.sh` 生成的 `capture_pps_ptp_wiring.sh`、`capture_power_loss_resume.sh` 和最终 `capture_field_acceptance.sh` 的逐行 `key=value` 证据检查必须与 `lio_eval_tools` manifest 口径对齐。空行可忽略，但任何非空行都必须包含非空 key 和 `=` 分隔；无 `=`、空 key 或重复 key 均必须使对应 `*_keys_status`、子证据状态和最终 gate 保持 FAIL，防止脚本先生成 PASS 外观报告、manifest 后续再打 FAIL。

v4.29 补充约束：`runtime_ops.sh` 生成的板端 `runtime_stability_check.sh` 在复验每次 `runtime_health.sh` 输出时，也必须执行整文件逐行 `key=value` 结构校验。空行可忽略，但健康报告中的无 `=` 行、空 key 或任意重复 key，即使不属于 `runtime_dir/disk_available_gb/runtime_pid/systemd_active/docker_container_status` 关键字段，也必须计入 `health_failures` 并使长稳 summary `overall=FAIL`，防止 24 h 长稳证据在板端先生成 PASS 外观后再被最终验收拒绝。

v4.30 补充约束：`runtime_ops.sh` 生成的板端 `runtime_stability_check.sh` 每次执行都必须生成本次运行自洽的 `runtime_stability.csv`，不得复用或追加旧运行遗留的采样行。脚本启动时必须重写固定 CSV 表头，本次循环只写本次 `samples` 条采样，使 summary 中的 `samples` 与 CSV 非空采样记录数一致，防止重复运行长稳检查后板端 summary 自报 PASS、最终 field acceptance 再因 stale CSV 行数不一致而 FAIL。

v4.31 补充约束：`runtime_ops.sh` 生成的板端 `runtime_stability_check.sh` 允许通过 `TUNNEL_LIO_SKIP_WATCHDOG=1` 做离线 dry-run 诊断采样，但该路径不得生成 PASS 外观长稳证据。只要任一采样的 watchdog 被跳过，脚本必须累加 `watchdog_skipped`，CSV 保留 `watchdog_status=SKIP`，summary 必须写出 `overall=FAIL`；只有真实 watchdog 检查均为 `PASS`、health/disk 无失败的采样结果才可作为后续 24 h 长稳和 field acceptance PASS 证据。

v4.32 补充约束：最终现场验收入口也必须显式拒绝被跳过的 watchdog 证据。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须从 `runtime_stability_summary.txt` 读取 `watchdog_skipped`，下沉为 `runtime_stability_watchdog_skipped`，并要求其字面量严格等于 `0`；`lio_eval_tools` evidence manifest 必须同时要求独立 summary 的 `watchdog_skipped=0` 和 field acceptance 下沉的 `runtime_stability_watchdog_skipped=0`。缺字段、前导零、非零或污染值都必须使 `runtime_stability_status`、`field_acceptance_status` 和最终 `evidence_status` 保持 FAIL。

v4.33 补充约束：session 侧 `run_runtime_stability.sh` 的结构化 `runtime_stability_run.log` 必须保持纯逐行 `key=value` 证据文件，不得混入板端 `runtime_stability_check.sh` 或归档脚本的普通 stdout/stderr。长稳命令输出必须隔离归档到 `logs/runtime_stability_command_output.log`，run log 只记录 `started_at/runtime_dir/samples/interval/command_output_log/exit_status/capture_exit_status/finished_at` 等结构化字段，防止实机脚本输出路径行后被最终 `runtime_stability_run_log_keys_status` 自我拒绝。

v4.34 补充约束：最终现场验收和 evidence manifest 必须把 `runtime_stability_run.log` 中的 `capture_exit_status` 作为独立零值证据，而不是只检查主命令 `exit_status`。`capture_field_acceptance.sh` 必须读取并下沉 `runtime_stability_run_log_capture_exit_status`，且 `runtime_stability_run_log_status` 只有在 `exit_status=0` 与 `capture_exit_status=0` 均为字面量严格零时才可 PASS；`lio_eval_tools` 必须同时要求独立 run log 的 `capture_exit_status=0`、field acceptance 下沉字段为 0，并与独立 run log 原始值一致。缺字段、非零、前导零或最终报告与独立 run log 不一致时，`runtime_stability_status`、`field_acceptance_status` 和最终 `evidence_status` 必须保持 FAIL。

v4.35 补充约束：session 侧 `capture_runtime_stability.sh` 在归档板端 CSV/summary 前，必须把当前 session 的 `runtime_stability_run.log` 作为新鲜度 gate。除 `runtime_dir/samples/interval/exit_status` 外，若 run log 已存在 `capture_exit_status` 字段，则其必须为字面量 `0`；`capture_exit_status` 缺失仅允许用于 `run_runtime_stability.sh` 首次调用归档脚本、尚未回写捕获退出码且 run log 尚无 `finished_at` 的执行中状态。若既有 run log 显示 `capture_exit_status` 非 0，归档脚本必须写出 `overall=FAIL/capture_status=RUN_LOG_FAILED` 占位 summary 并拒绝复制板端 PASS 文件，防止失败捕获后的二次归档把 stale 长稳证据重新带入 session。

v4.36 补充约束：session 侧长稳归档不得接受“已完成但缺少捕获退出码”的 run log。`capture_runtime_stability.sh` 读取到 `finished_at` 时，必须同时看到 `capture_exit_status=0` 才能复制板端 CSV/summary；若 `finished_at` 已存在但 `capture_exit_status` 缺失，应按 `RUN_LOG_FAILED` 写出 FAIL 占位 summary 并非零退出。这样可以区分真正的首次执行中归档和事后残缺/伪造的完成态 run log，避免残缺 run log 把 stale 板端 PASS 证据带入 session。

v4.37 补充约束：`capture_exit_status` 缺失的首次归档必须证明来自 `run_runtime_stability.sh` 内部调用。`run_runtime_stability.sh` 在调用 `capture_runtime_stability.sh` 前写入短暂 `runtime_stability_capture_in_progress` marker，并通过 `RUN_RUNTIME_STABILITY_CAPTURE_MARKER` 传入；capture 脚本必须确认 marker 中的 `run_log` 指向当前 session 的 `runtime_stability_run.log`，否则即使 run log 具备 `exit_status=0` 且尚无 `finished_at`，也必须按 `RUN_LOG_FAILED` 拒绝 standalone 归档。该 marker 只用于执行中授权，不进入最终 evidence manifest，最终验收仍只接受回写后的 `capture_exit_status=0`。

v4.38 补充约束：session 侧长稳内部归档 marker 不能只绑定 run log 路径，还必须绑定本次执行的一次性 `capture_token`。`run_runtime_stability.sh` 在 `exit_status=0` 后生成 `capture_token=<date_ns>-<pid>`，同时写入 `runtime_stability_run.log` 和 `runtime_stability_capture_in_progress` marker；`capture_runtime_stability.sh` 在 `capture_exit_status` 缺失时，必须确认 marker 的 `run_log` 指向当前 run log，且 marker 中的 `capture_token` 与 run log 中合法 token 完全一致。缺 token、污染 token 或 token 不一致的 stale marker 必须按 `RUN_LOG_FAILED` 拒绝，防止上次中断残留 marker 授权本次 standalone 归档。

v4.39 补充约束：session 侧长稳归档的新鲜度 gate 必须同时检查授权文件的结构完整性。`capture_runtime_stability.sh` 在读取当前 `runtime_stability_run.log` 前必须执行整文件逐行 `key=value` 校验，拒绝无 `=`、空 key 或重复 key；只有结构合法的 run log 才能继续匹配 `runtime_dir/samples/interval/exit_status/capture_exit_status/capture_token`。当 `RUN_RUNTIME_STABILITY_CAPTURE_MARKER` 存在时，marker 也必须先通过同样的整文件 key 唯一性校验，任何无 `=` 污染行、空 key 或重复 key 都不得授权缺失 `capture_exit_status` 的首次归档，并必须按 `RUN_LOG_FAILED` 输出 FAIL 占位 summary，防止结构污染的临时 marker/run log 复制板端 stale PASS 长稳证据。

v4.40 补充约束：runtime health 证据必须与 runtime deployment 一样证明运行态来源。板端 `runtime_health.sh` 必须输出 `systemd_active_source` 和 `docker_container_status_source`，且只有 `systemctl` 来源的 systemd `active` 与 `docker_inspect` 来源的 Docker `running` 才允许 runtime health 进入最终 PASS；`runtime_stability_check.sh` 每次复验 health 报告时必须同步要求这两个来源字段；`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须下沉 `runtime_health_systemd_active_source` 与 `runtime_health_docker_container_status_source` 并纳入 `runtime_health_status/field_acceptance_status` gate；`lio_eval_tools` evidence manifest 必须同时复核独立 runtime health 文件和最终 field acceptance 下沉字段，来源缺失、`env_override`、`unavailable` 或污染时均保持 FAIL，防止只具备 `active/running` 外观的健康快照绕过实机运行态证据。

v4.41 补充约束：runtime health 快照必须具备可审计的 ISO-8601 seconds 时间戳并绑定最终验收报告。板端 `runtime_health.sh` 输出的 `timestamp` 必须被 `record_session.sh` 生成的 `capture_field_acceptance.sh` 校验和下沉为 `runtime_health_timestamp`；最终 `timestamp` 必须不早于该 health 快照时间。`lio_eval_tools` evidence manifest 必须要求独立 runtime health timestamp 合法、最终 field acceptance 下沉字段与独立 health 完全一致，并复核最终报告时间覆盖该 health 快照。缺失、畸形、不可能日期、未来 health 快照或下沉不一致时，`runtime_health_status`、`field_acceptance_status` 和最终 `evidence_status` 均保持 FAIL。

v4.42 补充约束：runtime deployment 部署检查也必须具备可审计的 ISO-8601 seconds 时间戳并绑定最终验收报告。板端 `runtime_deployment_check.sh` 输出的 `timestamp` 必须被 `record_session.sh` 生成的 `capture_field_acceptance.sh` 校验和下沉为 `runtime_deployment_timestamp`；最终 `timestamp` 必须不早于该 deployment 快照时间。`lio_eval_tools` evidence manifest 必须要求独立 runtime deployment timestamp 合法、最终 field acceptance 下沉字段与独立 deployment 完全一致，并复核最终报告时间覆盖该 deployment 快照。缺失、畸形、不可能日期、未来 deployment 快照或下沉不一致时，`runtime_deployment_status`、`field_acceptance_status` 和最终 `evidence_status` 均保持 FAIL。

v4.43 补充约束：time sync 捕获快照也必须具备可审计的 ISO-8601 seconds 时间戳并绑定最终验收报告。`capture_time_sync.sh` 输出的 `timestamp` 必须被 `record_session.sh` 生成的 `capture_field_acceptance.sh` 校验和下沉为 `time_sync_timestamp`；最终 `timestamp` 必须不早于该 time sync 快照时间。`lio_eval_tools` evidence manifest 必须要求独立 time sync timestamp 合法、最终 field acceptance 下沉字段与独立 time sync 完全一致，并复核最终报告时间覆盖该 time sync 快照。缺失、畸形、不可能日期、未来 time sync 快照或下沉不一致时，`time_sync_status`、`field_acceptance_status` 和最终 `evidence_status` 均保持 FAIL。

v4.44 补充约束：PPS/PTP wiring verified 报告也必须绑定当前 time sync 快照时间戳。`record_session.sh` 生成的 `capture_pps_ptp_wiring.sh` 必须从独立 `time_sync_status.txt` 读取合法 `timestamp` 并下沉为 `time_sync_timestamp`；`capture_field_acceptance.sh` 必须从独立 PPS/PTP wiring 报告读取该字段并下沉为 `pps_ptp_wiring_time_sync_timestamp`，且两个下沉字段都必须与独立 time sync timestamp 完全一致。`lio_eval_tools` evidence manifest 必须执行同样复核，防止 stale wiring 证据复用旧 time sync 快照进入最终 PASS。

v4.45 补充约束：PPS/PTP wiring verified 报告自身也必须具备可审计的 ISO-8601 seconds 生成时间戳。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须从独立 `pps_ptp_wiring_verified.txt` 读取合法 `timestamp` 并下沉为 `pps_ptp_wiring_timestamp`；最终 `timestamp` 必须不早于该 wiring 报告生成时间。`lio_eval_tools` evidence manifest 必须要求独立 PPS/PTP wiring timestamp 合法、最终 field acceptance 下沉字段与独立 wiring 报告完全一致，并复核最终报告时间覆盖该 wiring 报告。缺失、畸形、不可能日期、下沉不一致或时间倒置时，`pps_ptp_wiring_status`、`field_acceptance_status` 和最终 `evidence_status` 均保持 FAIL。

v4.46 补充约束：power-loss resume verified 报告自身也必须具备可审计的 ISO-8601 seconds 生成时间戳。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须从独立 `power_loss_resume_verified.txt` 读取合法 `timestamp` 并下沉为 `power_loss_resume_timestamp`；最终 `timestamp` 必须不早于该 power-loss resume 报告生成时间。`lio_eval_tools` evidence manifest 必须要求独立 power-loss resume timestamp 合法、最终 field acceptance 下沉字段与独立 power-loss resume 报告完全一致，并复核最终报告时间覆盖该报告。缺失、畸形、不可能日期、下沉不一致或时间倒置时，`power_loss_resume_status`、`field_acceptance_status` 和最终 `evidence_status` 均保持 FAIL。

v4.47 补充约束：runtime stability summary 自身也必须具备可审计的 ISO-8601 seconds 生成时间戳。`runtime_ops.sh` 生成的 `runtime_stability_check.sh` 和 `record_session.sh` 的长稳捕获失败/不可用 summary 都必须写出 `timestamp` 字段；`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须从独立 `runtime_stability_summary.txt` 读取合法 `timestamp` 并下沉为 `runtime_stability_summary_timestamp`，要求该时间不早于 `runtime_stability_run.log` 的 `started_at`、不晚于 `finished_at`，且最终 `timestamp` 必须不早于该 summary 生成时间。`lio_eval_tools` evidence manifest 必须要求独立 summary timestamp 合法、最终 field acceptance 下沉字段与独立 summary 完全一致，并复核 run log 时间窗和最终报告覆盖关系。缺失、畸形、不可能日期、越出 run log 时间窗、下沉不一致或时间倒置时，`runtime_stability_status`、`field_acceptance_status` 和最终 `evidence_status` 均保持 FAIL。

v4.48 补充约束：runtime stability CSV 的采样时间也必须形成可审计链。`record_session.sh` 生成的 `capture_field_acceptance.sh` 必须逐行读取独立 `runtime_stability.csv` 的采样 `timestamp`，要求每个 timestamp 合法且按采样顺序非递减，并把独立 CSV 推导出的首末采样时间下沉为 `runtime_stability_csv_first_timestamp` 与 `runtime_stability_csv_last_timestamp`；首末时间必须满足 `first <= last`，均落在 `runtime_stability_run.log` 的 `started_at/finished_at` 闭区间内，且最终 `timestamp` 必须不早于最后一条 CSV 采样时间。`lio_eval_tools` evidence manifest 必须独立复算 CSV 首末采样时间并要求 field acceptance 下沉字段完全一致。缺失、畸形、不可能日期、采样时间倒序、越出 run log 时间窗、下沉不一致或最终报告早于最后采样时间时，`runtime_stability_csv_status`、`runtime_stability_status`、`field_acceptance_status` 和最终 `evidence_status` 均保持 FAIL。

#### M02 时间管理

开发步骤：

1. 以 GUJ120 MSOP 包内 timestamp、azimuth 和点内相对时间为第一优先级，ROS header stamp 只做工程辅助。
2. 建立每台雷达到融合参考时间的映射：`t_ref = a_s * t_sensor + b_s`，首版 `a_s = 1`，离线估计 `b_s`。
3. 对 IMU 驱动增加时间诊断：Modbus 读取开始/结束时间、发布时刻、估计采样时刻；当前已显式发布 `timestamp_source=host_now`、`hardware_time_status=host_time_only`、`pps_status=unconfigured` 并由 `/time/status` 聚合，真实硬件时间/PPS 接入后必须替换为实测来源。
4. 若 PPS 可接入，新增 `/time/pps_event`，建立 IMU/PPS 到主机单调时间的映射，并在 `/time/status` 中输出 PPS 事件频率、最近间隔、间隔 jitter、stale 和时间回退告警。
5. 输出 `/time/status`，包含每路雷达时间跨度、sync slop、IMU 频率、IMU/PLC Modbus RTT、读取错误、无效帧、IMU 饱和样本、PPS 健康和时间回退告警。
6. 在 `lidar_fusion` 的点内时间归一化基础上，后续增强到设备时间级归一化。

验收标准：

| 项目 | 标准 |
|---|---|
| 时间单调 | 2 h 静态运行无时间回退 |
| 同步跨度 | 三雷达同步 span 持续可观测，超阈值告警 |
| PPS 事件 | `/time/pps_event` 接入时频率、最近间隔、间隔 jitter 和 stale 状态持续可观测，时间回退为 ERROR |
| 时间同步证据 | `record_session` 能从 `/time/status` 捕获 PPS 与 clock offset 诊断块，只有匹配 block 的 `level=0` 才允许判定 PASS |
| 点级时间 | 10 Hz 机械扫描点相对时间覆盖一个扫描周期内合理范围 |
| IMU 时间 | 发布频率和读取延迟可统计；无异常跳变 |

#### M03 外参与坐标管理

开发步骤：

1. 冻结根坐标：推荐 `base_link` 为机体系，`lidar_center` 为三雷达几何参考。
2. 建立 `T_base_lidar_center`、`T_lidar_center_lidar_left`、`T_lidar_center_lidar_right`、`T_base_imu` 命名规范。
3. 写入 `calib/extrinsics.yaml`，禁止外参散落在多个 launch 文件中。
4. 编写 C++/roscpp TF 审计服务 `/calib/audit_tf`：检查必需 frame、父子关系、单根 TF 树、多父节点、循环、非有限数值、异常平移量级和 RPY 歧义角；Python 脚本仅作为离线辅助。
5. IMU 原生坐标到 ROS 坐标单独建 `imu_native -> imu_link` 变换，不允许在算法里隐式调轴。
6. 机械安装后由 ME 出具安装台账：支架版本、螺栓状态、减振结构、复测时间。

验收标准：

| 项目 | 标准 |
|---|---|
| 静态叠加 | 三雷达观察同一墙角/边缘无系统性双线 |
| 重复上电 | 多次上电 TF 一致，外参文件版本不变 |
| 方向语义 | 所有矩阵方向有明确命名和审计结果；异常平移量级和 RPY 歧义角会被审计拒绝 |
| 刚性 | 强振动前后外参健康检查无明显偏移 |

#### M04-M08 前端链路

开发步骤：

1. 先跑中心雷达 + IMU 基线，确认去畸变、状态预测和注册收敛。
2. 接入三雷达融合点云，保留 `sensor_id` 或通过字段/来源记录每点来源。
3. 去畸变按点时刻插值 IMU 连续轨迹，输出到统一参考时刻。
4. 对近机体区域、截割头动态区、粉尘/飞散物进行过滤或降权。
5. 局部注册当前采用多尺度 voxel ICP 首版，并提供 NDT 粗配准 + ICP 精配准可选路径、GICP/Surfel 协方差感知注册可选路径；后续继续预留 BIEVR bump-image 高分辨率残差接口。
6. 计算配准 Hessian 或等价可观测性指标，检测纵向弱约束。
7. 弱方向执行冻结/限幅：静止和冲突状态冻结，移动状态限幅，TCA 或明确几何恢复后解除。

验收标准：

| 项目 | 标准 |
|---|---|
| 中心雷达基线 | 低速运动段可连续输出局部里程计 |
| 三雷达融合 | 重叠区残差受控，融合后截面完整度优于单雷达 |
| 静止强振动 | 静止截割时不持续累计位移 |
| 退化保护 | 长直弱几何段能识别弱方向；启用后长度漂移明显下降 |

#### M09 PLC 工况状态机

状态定义：

| 状态 | 进入依据 | 位姿策略 | 写图策略 |
|---|---|---|---|
| `IDLE_STATIC` | 履带无动作、截割停止、LiDAR 位移低、IMU 均值稳定 | 位姿冻结 | 允许高置信静态累积 |
| `CUTTING_STATIC` | 截割电机运行、履带无动作、LiDAR 位移低、IMU 振动高 | 位姿冻结 | 允许低权重截面累积，不扩全局图 |
| `FWD_MOVE` | 前进命令 + LiDAR/IMU 确认真实位移 | 正常更新 | 扩局部图和活动会话图 |
| `REV_MOVE` | 后退命令 + LiDAR/IMU 确认真实位移 | 正常更新 | 可更新活动会话，谨慎写稳定图 |
| `TURNING` | 左右履带状态不一致或角速度确认转向 | 正常更新但降权 | 局部图更新，截面质量降级 |
| `CMD_MOVE_NO_DISP` | PLC 有运动命令但 LiDAR 位移不足 | 冻结或限幅 | 禁止扩图，告警 |
| `CONFLICT` | PLC、IMU、LiDAR 三者矛盾 | 冻结或保守预测 | 只写诊断，不写稳定图 |
| `RELOCALIZING` | 启动/断电恢复/重定位中 | 候选位姿 | 暂不写基线图 |

开发步骤：

1. 建立 PLC 字段映射和回放工具。
2. 离线回放构建状态转移表和滞回时间。
3. 联合 IMU 高频能量、LiDAR ICP 位移、PLC 命令进行三重判定。
4. 输出 `write_policy`：`freeze_pose`、`update_section_only`、`update_session`、`promote_allowed`、`diagnostics_only`。

验收标准：

| 项目 | 标准 |
|---|---|
| 静止识别 | 静止待机、静止截割均不误扩图 |
| 空转识别 | 指令运动未位移识别率 > 95% |
| 状态切换 | 前进/后退/转向切换有滞回，无频繁抖动 |
| 冲突处理 | 冲突状态不污染稳定图 |

#### M10-M12 地图、截面与续建

开发步骤：

1. 局部子图按时间窗/位移窗滚动维护，记录 session_id、chainage、quality、state。
2. 截面数据库区分观测截面和结构截面：
   - 观测截面保留管线、电缆、支护网、锚杆等实际观测。
   - 结构截面按配置剔除附属设施，应用矩形/拱形先验，仅作为产品后处理，不反向强行约束前端。
3. 断面链距由局部里程计、状态机和控制锚点共同维护，不直接用 PLC 运动命令积分。
4. 会话元数据使用 WAL/追加日志；点云块使用分段文件；manifest 原子提交；每次创建、手动快照、周期快照和状态切换都写入完整 manifest 快照 WAL。
5. 断电恢复按三级链路：最后稳定位姿 + 最近局部子图；TCA；全局描述子候选。失败则新建暂存会话。

验收标准：

| 项目 | 标准 |
|---|---|
| 截面一致性 | 控制断面重复观测满足 A/B 级指标 |
| 地图分层 | 活动图、稳定图、截面库互不混淆 |
| 快照恢复 | 强制断电后 manifest 丢失或损坏时，可从 WAL 回放恢复到最近完整状态 |
| 续建保守 | 恢复验证失败时不会继续写坏基线图 |

### 5.3 P1 模块开发步骤与验收

#### M13 TCA 标靶锚点

开发步骤：

1. 反光圆柱检测先基于强度、几何圆柱模型和高度/侧别约束。
2. 单标靶只作为入口，不作为唯一身份。
3. TCA 身份由“标靶中心 + 周边 3-8 m 局部几何/强度/截面上下文 + 历史位姿先验 + 非等距序列”共同确定。
4. 建立施工台账：链距、侧别、高度、安装日期、标靶编号、上下文子图版本。
5. 如果 Top1/Top2 候选分数比不足，必须拒绝，不允许强行匹配。

验收标准：

| 项目 | 标准 |
|---|---|
| 热点恢复 | 有 TCA 的断电热点恢复成功率 > 98% |
| 非唯一处理 | 相似锚点场景必须拒绝误认 |
| 遮挡处理 | 部分遮挡时能降级或拒绝，有健康状态 |

#### M14 保守回环与稳定图治理

开发步骤：

1. 首版只做候选检索和离线报告，不自动闭环。
2. 候选通过局部几何精配准、截面一致性、状态历史、TCA 上下文验证后才接受。
3. 只对稳定区执行低频闭环，活动工作面和重启未验证会话不得直接参与远程闭环。
4. 闭环后必须复核控制断面和控制链距，不得为降低轨迹误差拉坏截面。

验收标准：

| 项目 | 标准 |
|---|---|
| 错回环 | 验收集错误闭环为 0 |
| 闭环收益 | 长度误差下降，截面质量不下降 |
| 可审计 | 每个接受/拒绝候选都有日志和评分 |

## 6. 点云融合策略

### 6.1 坐标与时刻

推荐以中心雷达或 `base_link` 为融合参考坐标系，以中心雷达扫描结束时刻或融合帧参考时刻为 `t_ref_k`。每个点按自己的传感器、扫描内相对时间和时偏映射得到采样时刻，再通过 IMU 连续轨迹去畸变到 `t_ref_k`。

点云融合流程：

1. 驱动层解析 GUJ120 MSOP/DIFOP，保留距离、强度、ring、azimuth、timestamp、回波模式和 sensor_id。
2. 按 GUJ120 说明书中的 azimuth 插值规则恢复每个点的水平角和点级相对时间。
3. 每台雷达用 `T_ref_lidar_i` 投到中心雷达或 `base_link`。
4. 按 IMU 连续时间轨迹将点从采样时刻补偿到统一参考时刻。
5. 对重叠区点计算一致性残差，作为外参/时间健康诊断。
6. 对点附加质量权重：时间不确定度、距离、入射角、振动强度、动态区、粉尘疑似、传感器来源。
7. 输出给前端时保留原始密度；降采样由前端按配准策略执行，`lidar_fusion` 默认不启用 voxel filter。

### 6.2 三雷达分工

| 雷达 | 建议角色 | 权重原则 |
|---|---|---|
| 中心水平雷达 | 主参考、主里程计约束、链距连续性 | 正常权重，时间基准优先 |
| 左倾斜雷达 | 补顶/侧壁/盲区，增强截面完整度 | 与中心重叠一致时正常；遮挡/时间异常时降权 |
| 右倾斜雷达 | 补顶/侧壁/盲区，增强截面完整度 | 同左雷达 |

### 6.3 强度与双回波

一期默认以单回波和几何为主，强度用于诊断、标靶检测和 P1 退化增强。双回波可作为现场 A/B 测试分支，不作为首版主链路强依赖。

强度使用原则：

1. 多雷达强度响应需先做距离归一化和传感器间标定。
2. 几何约束充分时，强度不参与主位姿或低权重参与。
3. 长直极低特征且强度纹理稳定时，可条件激活强度残差。
4. 反光标靶高强度点只提供候选，必须通过 TCA 上下文验证。

### 6.4 静止段多帧累积

静止待机是截面精化的高价值数据源。静止截割是有价值但风险更高的数据源。

| 状态 | 累积策略 |
|---|---|
| `IDLE_STATIC` | 位姿冻结，高置信累积，用于 A 级截面和局部子图精化 |
| `CUTTING_STATIC` | 位姿冻结，点级振动不确定度降权，主要用于补充截面完整度 |
| `FWD_MOVE/REV_MOVE` | 正常扩图，截面质量通常标为 B 或待评估 |
| `CONFLICT/CMD_MOVE_NO_DISP` | 不扩图，仅记录诊断和事件数据 |

## 7. IMU 融合策略

主方案为增强型松耦合：

| 功能 | IMU 作用 | 是否主导位姿 |
|---|---|---|
| 点级去畸变 | 通过连续时间轨迹补偿扫描内运动和振动 | 否 |
| 注册初值 | 提供短时预测，降低 ICP 初值敏感性 | 否 |
| bias/gravity 管理 | 滑窗估计偏置、重力方向和健康分数 | 否 |
| 静止/振动识别 | 计算加速度/角速度均值、频谱、高频能量 | 否 |
| 断电恢复 | 快速恢复后提供姿态先验和静态窗口质量 | 否 |
| 退化短时约束 | 极短距离内辅助外推，配合冻结/限幅 | 有限参与 |

不采用全时全量紧耦合作为主链路的原因：

1. 主工况是长时间静止强振动，不是高速运动。
2. 强惯性残差可能把时间偏差、安装微弹性和振动噪声解释成位移。
3. 三雷达异步、多外参、多时偏会显著提高紧耦合调参难度。
4. 板端算力应优先保障在线最小集，而不是全量滑窗/因子图/回环同时实时运行。

IMU 驱动增强任务：

| 任务 | 当前状态 | 后续要求 |
|---|---|---|
| Modbus 读数 | 已实现 TCP 502 读寄存器 | 保持 |
| 发布频率 | 节点以 400 Hz loop 运行，实际取决于读取耗时 | 统计真实频率和抖动 |
| 时间戳 | 当前使用 `ros::Time::now()`，但已在 IMU 诊断和 `/time/status` 中显式标记 `timestamp_source=host_now`、`hardware_time_status=host_time_only`、`pps_status=unconfigured` | 接入真实硬件时间/PPS/读取延迟估计后，必须更新诊断来源并用现场证据验证 |
| 坐标系 | 已明确 `imu_link`，并通过 `imu_native -> imu_link` 显式轴适配后再经 TF 到 `base_link` | 现场复核安装方向和机械刚性 |
| 协方差 | 已参数化 `orientation/angular_velocity/linear_acceleration` 三组对角协方差，默认值保持原工程硬编码，`imu_modbus.launch` 和 `bringup_sensors.launch` 均可覆盖；IMU diagnostics 已发布 9 个 covariance 字段，`/time/status` 已聚合下沉 | 用现场标定结果替换默认值，并在回放报告和证据包中记录参数快照 |
| 健康诊断 | 已有频率、读取延迟、读取失败、无效样本计数、饱和样本计数、默认关闭的可配置温度/温漂诊断、重连尝试/成功计数和时间来源/PPS 状态诊断；饱和阈值已参数化，默认加速度 150 m/s^2、角速度 1800 deg/s；温度诊断支持 `float32/int32/int16/uint16` 寄存器格式、比例系数、偏置、绝对温度阈值和窗口温升阈值，寄存器解码逻辑已抽为 C++ 纯函数并覆盖边界单测 | 用现场 IMU 说明书/实测数据复核温度寄存器、比例系数、饱和阈值和温漂阈值 |

## 8. 断电续建流程

### 8.1 持久化原则

强制断电不可避免，因此恢复设计不能依赖正常关机。

| 数据对象 | 推荐格式 | 刷盘策略 | 恢复策略 |
|---|---|---|---|
| 会话元数据 | SQLite WAL 或追加日志 | 1 s 或状态切换即时提交 | 启动时重放 WAL |
| 最后稳定位姿 | 小文件 + 原子重命名 | 1 s 或关键状态提交 | 优先加载 |
| 局部子图 | 压缩块文件 | 30 s 或 1-2 m 位移封块 | manifest 恢复最近完整块 |
| 截面数据库 | 元数据即时，点云块延迟 | 5 s 或链距切片提交 | 按 session_id/chainage 补链 |
| 原始 PCAP | 分段文件 | 30-60 s 封段 | 仅恢复完整段 |
| rosbag | 分段文件 | 60 s 封段 | 用于回放，不作为真值 |

### 8.2 重启时序

| 时间窗 | 动作 | 通过条件 |
|---|---|---|
| 0-5 s | IPC 启动、挂载存储、恢复 WAL、启动日志服务 | manifest 和 WAL 校验通过 |
| 5-10 s | 启动 IMU Modbus/PPS，建立静态窗口 | IMU 数据连续，频率稳定 |
| 8-15 s | 启动三雷达驱动和 `lidar_fusion` | 三路点云可见，TF 可用 |
| 12-25 s | 静态质量检查，估计 IMU bias，检查最近子图 | 静态窗口质量达标 |
| 25-35 s | 一级恢复：最后稳定位姿 + 最近局部子图精配准 | 匹配分数通过 |
| 35-45 s | 二级恢复：TCA 锚点 + 上下文验证 | 锚点唯一且精配准通过 |
| 45 s 后 | 三级恢复：全局候选；仍失败则新建暂存会话 | 成功恢复或安全降级 |

### 8.3 续建状态机

```text
BOOT
  -> LOAD_SNAPSHOT
  -> WAIT_SENSOR_STABLE
  -> TRY_LAST_STABLE_POSE
      success -> RECOVERED_ACTIVE_SESSION
      fail    -> TRY_TCA
  -> TRY_TCA
      success -> RECOVERED_ANCHORED_SESSION
      fail    -> TRY_GLOBAL_CANDIDATE
  -> TRY_GLOBAL_CANDIDATE
      success -> RECOVERED_NEW_SESSION
      fail    -> SAFE_TEMP_SESSION
```

关键原则：

1. 一级恢复必须做局部精配准验证，不能直接相信上次位姿。
2. 断电后若底盘有液压回弹或微位移，一级恢复失败是正常情况。
3. 恢复成功前不得写稳定基线图。
4. 新会话不是失败，而是保守机制；后续可通过低频验证晋升。

## 9. 截面控制与长度控制原则

### 9.1 截面产品定义

| 产品 | 定义 | 用途 |
|---|---|---|
| 观测截面 | 实际观测到的点云轮廓，保留管线、电缆、支护网、锚杆和临时物 | 现场实况、障碍/附属设施记录 |
| 结构截面 | 按配置剔除附属设施，并结合矩形/拱形先验拟合巷道主体 | 工程验收、断面变形、进尺质量分析 |

截面质量等级：

| 等级 | 数据来源 | 建议标准 |
|---|---|---|
| A | `IDLE_STATIC` 高置信累积 + 多视角完整 | RMSE <= 25 mm，完整度 >= 90% |
| B | 运动段或 `CUTTING_STATIC` 降权累积 | RMSE <= 40 mm，完整度 >= 80% |
| C | 遮挡、粉尘、冲突、退化明显 | 仅用于参考，不进入关键验收 |

### 9.2 截面生成原则

1. 链距切片间隔根据项目要求配置，建议首版 0.5 m 或 1.0 m。
2. 截面切片方向以局部轨迹切向为准，退化或冲突状态下使用最近稳定切向。
3. 静止累积只提升截面密度，不改变全局位姿。
4. 结构截面先验只用于后处理，禁止反向强行拉动前端轨迹。
5. 支护网、锚杆、管线、电缆应按类别配置保留或剔除。
6. 每个截面必须保存 session_id、chainage、状态来源、点数、完整度、RMSE/拟合残差、质量等级；当前 `section_manager` 已将 `session_id` 参数和 `/mapping/control` 的 `machine_state` 写入结构化输出、诊断和 CSV。现场 session 采集应优先使用 session 归档目录中的 `commands/bringup_fusion_*_session.sh` 启动 fusion 链路，由脚本自动把当前 `session_name` 传入 `section_session_id`；板端长期 runtime 使用 `runtime_ops.sh --launch-file bringup_fusion_*.launch` 时默认把 `runtime_name` 传入 `section_session_id`，也可通过 `--section-session-id` 覆盖；两者都必须拒绝 `missing`、`__DUPLICATE_KEY__`、分号和换行等污染值；禁止正式采集长期使用默认 `unassigned`。
7. 在线历史截面按 `section_spacing_m` 做链距窗口去重；同一窗口内只保留质量等级更高、完整度更高、RMSE 更低或点数更多的观测，避免重复触发污染导出结果。

### 9.3 长度控制原则

长度误差控制不依赖单一前端，而采用多层约束：

| 层级 | 控制手段 |
|---|---|
| 前端 | 退化检测、纵向弱方向冻结/限幅、多分辨率几何注册 |
| 状态机 | 静止不积分，指令动未位移不扩图，冲突不写稳定图 |
| 会话 | 断电后必须重定位验证，新会话先暂存后晋升 |
| 锚点 | 重启热点、长直低特征段、控制断面区布设 TCA |
| 后端 | 只对稳定区做保守回环和链距修正 |
| 验收 | 以 100 m 控制区和控制断面为外部基准定期复核 |

长直弱几何段处理：

1. 配准可观测性下降时，输出 `degeneracy_mode = LONGITUDINAL_WEAK`。
2. 纵向更新限幅，横截面横向/竖向几何仍正常使用。
3. 若进入极端退化，冻结弱方向，最长只允许短距离 IMU 外推。
4. 有 TCA 或明显几何恢复时解除冻结。
5. 后端只在精验证通过后调整长度，不为闭环牺牲截面局部质量。

## 10. 里程碑计划

建议按 28 周组织，实际可根据人员并行度压缩或拉长。

| 阶段 | 周期 | 目标 | 关键交付 | 退出条件 |
|---|---:|---|---|---|
| P0-1 基础接入 | 第 1-2 周 | 三雷达、IMU、PLC 数据链路 | 驱动启动包、双录制脚本、topic 规范 | 数据可稳定录制回放 |
| P0-2 时间/外参 | 第 3-4 周 | 时间管理、TF 审计、标定文件 | `/time/status`、`calib/extrinsics.yaml`、`/calib/audit_tf` C++ 服务、离线审计脚本 | 静态叠加无系统双线 |
| P0-3 中心雷达基线 | 第 5-7 周 | 中心雷达 + IMU 去畸变和局部注册 | 基线前端、IMU 健康诊断 | 低速运动可连续建局部图 |
| P0-4 状态机 | 第 6-8 周 | PLC/IMU/LiDAR 联合工况判断 | 八态状态机、写图策略 | 静止截割不扩图 |
| P0-5 三雷达前端 | 第 8-12 周 | 三雷达融合、连续时间去畸变、退化保护 | `/lio/points_deskewed`、`/lio/odom_local` | 100 m 控制区 <= 1.0% |
| P0-6 截面与局部图 | 第 12-16 周 | 局部子图、截面数据库、质量等级 | 截面导出器、质量报表 | 控制断面重复一致 |
| P0-7 断电恢复 | 第 16-19 周 | 快照、WAL、三级恢复、新会话 | 会话管理器、恢复服务 | 45 s 内恢复或安全降级 |
| P1-1 TCA | 第 19-22 周 | 标靶检测、上下文锚点、台账 | TCA 数据库、检测器 | 热点恢复 > 98% |
| P1-2 保守后端 | 第 22-25 周 | 候选回环、精验证、稳定图治理 | 回环报告、稳定图优化 | 错回环验收为 0 |
| P0/P1 部署验收 | 第 25-28 周 | 板端裁剪、长稳、HIL/Gazebo/现场验收 | 部署包、运维手册、验收报告 | 24 h 稳定运行 |

## 11. 测试与验收矩阵

| 用例 | 场景 | 验证点 | 通过标准 |
|---|---|---|---|
| 静态矩形巷道 | 无动态、规则矩形 | 外参、截面、基础建图 | 轮廓稳定，无双线 |
| 静态拱形巷道 | 无动态、拱形 | 结构截面模式 | 拱形/矩形配置正确 |
| 长直弱特征 | 重复墙面、少细节 | 退化检测和长度限幅 | 启用保护后长度误差下降 |
| 管线/电缆/支护网 | 有附属设施 | 观测/结构截面分离 | 观测保真，结构不过度受污染 |
| 静止强振动 | 车不动、截割振动 | 状态机、位姿冻结 | 不持续扩图 |
| 前进/后退 | 短时移动 | 链距、状态切换 | 方向正确，轨迹连续 |
| 指令动未位移 | PLC 有命令，几何无位移 | 空转/卡滞识别 | 禁止扩图并告警 |
| 断电原地重启 | 中途断电 | 一级恢复 | 45 s 内恢复 |
| 断电微位移 | 重启后 3-20 cm 偏移 | 二/三级恢复 | 不误套上次位姿 |
| TCA 热点 | 稀疏标靶 | 锚点恢复 | 不唯一时拒绝 |
| 错环陷阱 | 重复拓扑 | 保守回环 | 错误闭环不接受 |
| 长稳压力 | 24 h 运行 | 队列、磁盘、看门狗、`runtime_stability_check.sh` 周期采样 | 不崩溃、不堆积、事件保全，输出 `runtime_stability.csv`、`runtime_stability_summary.txt` 和 session 侧 `runtime_stability_run.log` |
| 时间同步闭环 | PPS/PTP 接线后 2 h 静态运行 | `/time/status`、`/time/pps_event`、PPS jitter、host/device offset、`capture_pps_ptp_wiring.sh` | `time_sync_status=PASS`，PPS 与 clock offset 诊断块均为 OK，`pps_jitter_ms` 必须为非负数且 `mean_offset_ms` 必须为合法数值，证据包包含 `time_sync_status.txt`、原始 YAML 和带 `pps_wiring_verified/ptp_wiring_verified/wiring_verified_by/wiring_verified_at` 的 `pps_ptp_wiring_verified.txt` |
| 最终现场验收 gate | Release 候选系统 | metrics、replay/HIL event_file、time sync、runtime health、systemd/Docker 实机运行、24 h 长稳、断电续建、PPS/PTP 接线确认、section export | `field_acceptance_status=PASS`；event_file 必须由当前 session/scenario 的规范化事件流证明，每条事件都必须包含合法 `event/session_id/scenario/t` 且无重复 key，其中 `t` 必须严格可解析且非负，指标或恢复事件证据必须来自当前 session/scenario，指标数值必须严格合法，物理非负指标不得为负值，`chainage_m/reference_chainage_m` 必须成对出现，最终报告必须下沉 `event_file_status=PASS` 与 `event_file_report`；缺任一项时保持 FAIL，不允许用 dry-run 或 env override 替代；time sync 必须严格包含 `time_sync_status=PASS`、`capture_status=CAPTURED`、`pps_status=PASS`、`clock_offset_status=PASS` 和指向当前 evidence bundle 内非空原始 YAML 的 `raw` 字段；runtime health 必须包含非空且非哨兵值的运行目录、可解析非负磁盘余量、严格正整数 PID，并证明 systemd 为 active、Docker 为 running；runtime deployment 总字段 `deployment_status` 必须严格等于 `PASS`；24 h 长稳不得被 `FIELD_ACCEPTANCE_MIN_STABILITY_HOURS` 降级，最终报告必须下沉 `runtime_stability_min_duration_status=PASS`、`runtime_stability_min_duration_h >= 24`、`runtime_stability_duration_h >= runtime_stability_min_duration_h` 且 `runtime_stability_duration_h` 与 `runtime_stability_samples * runtime_stability_interval_s / 3600` 两位小数下沉值一致，缺字段、低于 24、不一致或非法覆盖必须保持最终 FAIL；section export 必须包含固定表头、至少一条数据行、session_id 与 evidence manifest 一致、正式工况枚举、A/B/C 质量等级和合法数值字段；PPS/PTP 接线确认必须来自 `manual_file`，确认文件和 verified 文件总字段 `pps_ptp_wiring_verified` 均必须严格等于 `PASS`，且 PPS、PTP、确认人和确认时间字段为有效文本值；断电续建必须来自 `metrics_report` 或 `manual_file`，其中 `metrics_report` 来源必须带非空 `metrics_report` 字段，且该字段解析后必须与 evidence manifest 的 `metrics_report` 路径一致，verified 文件总字段 `power_loss_resume_status` 必须严格等于 `PASS`；断电续建必须有实测 `recovery_time_s` 且 `recovery_time_s <= max_recovery_time_s`，缺字段、非数字或超时均保持 FAIL |

补充验收约束：PPS/PTP wiring 的独立 PASS 必须绑定 evidence manifest 指向的 `time_sync` 证据实际 PASS，不得只相信 wiring 文件内的 time sync 自报字段；脚本侧 `field_acceptance_report.txt` 中的 `pps_ptp_wiring_verified=PASS` 也必须绑定当前 `logs/time_sync_status.txt` 的实际 PASS 结果，且当前 time sync 证据和最终 field acceptance 报告自身都必须包含有效 `time_status_topic/pps_topic`；`metrics_report` 来源的 `power_loss_resume_status` 必须先证明 metrics 报告自身完整 PASS，再证明 `recovery_time_s` 来自当前 `session/scenario/status=PASS/failed_checks=0` 的匹配记录块，不得从失败 summary、重复 key、无当前匹配记录或其它记录中借值；当前匹配记录缺少恢复时间、恢复时间畸形或超时时，即使其它记录可通过，最终验收仍保持 FAIL。

补充验收约束续：最终 `field_acceptance_report.txt` 必须从独立 `pps_ptp_wiring_verified.txt` 下沉 `wiring_confirmation=PASS`、`wiring_confirmation_overall=PASS` 和 `wiring_confirmation_keys_status=PASS`，并把这三个字段同时纳入 `pps_ptp_wiring_verified=PASS` 和 `field_acceptance_status=PASS` 的脚本侧 gate；evidence manifest 侧也必须复核最终报告自身包含这三个字段且均为 PASS。

补充验收约束续二：runtime health 不能只凭 `systemd_active=active` 和 `docker_container_status=running` 通过最终验收；独立 runtime health 文件和最终 `field_acceptance_report.txt` 下沉字段必须同时证明 `systemd_active_source=systemctl`、`docker_container_status_source=docker_inspect`，缺失、`env_override`、`unavailable` 或污染来源均不得生成 `field_acceptance_status=PASS`。

补充验收约束续三：runtime health 还必须带可审计 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉完全一致的 `runtime_health_timestamp`，且最终报告自身 `timestamp` 不得早于该 health 快照时间。独立 health 缺 timestamp、timestamp 畸形、日历日期不可能、最终报告缺少下沉字段、下沉字段与独立 health 不一致或最终报告时间早于 health 快照时，`field_acceptance_status=PASS` 不能成立。

补充验收约束续四：runtime deployment 还必须带可审计 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉完全一致的 `runtime_deployment_timestamp`，且最终报告自身 `timestamp` 不得早于该 deployment 快照时间。独立 deployment 缺 timestamp、timestamp 畸形、日历日期不可能、最终报告缺少下沉字段、下沉字段与独立 deployment 不一致或最终报告时间早于 deployment 快照时，`field_acceptance_status=PASS` 不能成立。

补充验收约束续五：time sync 也必须带可审计 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉完全一致的 `time_sync_timestamp`，且最终报告自身 `timestamp` 不得早于该 time sync 快照时间。独立 time sync 缺 timestamp、timestamp 畸形、日历日期不可能、最终报告缺少下沉字段、下沉字段与独立 time sync 不一致或最终报告时间早于 time sync 快照时，`field_acceptance_status=PASS` 不能成立。

补充验收约束续六：PPS/PTP wiring verified 报告必须带与独立 time sync 完全一致的 `time_sync_timestamp`，最终 `field_acceptance_report.txt` 必须下沉完全一致的 `pps_ptp_wiring_time_sync_timestamp`。独立 PPS/PTP wiring 缺 timestamp、timestamp 畸形、与独立 time sync 不一致、最终报告缺少下沉字段或下沉字段不一致时，`pps_ptp_wiring_verified=PASS` 和 `field_acceptance_status=PASS` 均不能成立。

补充验收约束续七：PPS/PTP wiring verified 报告自身必须带合法 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉完全一致的 `pps_ptp_wiring_timestamp`，且最终报告自身 `timestamp` 不得早于该 wiring 报告生成时间。独立 PPS/PTP wiring 缺自身 `timestamp`、timestamp 畸形、最终报告缺少下沉字段、下沉字段不一致或最终报告时间早于 wiring 报告时，`pps_ptp_wiring_verified=PASS`、`field_acceptance_status=PASS` 和 `evidence_status=PASS` 均不能成立。

补充验收约束续八：power-loss resume verified 报告自身必须带合法 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉完全一致的 `power_loss_resume_timestamp`，且最终报告自身 `timestamp` 不得早于该 power-loss resume 报告生成时间。独立 power-loss resume 缺自身 `timestamp`、timestamp 畸形、最终报告缺少下沉字段、下沉字段不一致或最终报告时间早于 power-loss resume 报告时，`power_loss_resume_status=PASS`、`field_acceptance_status=PASS` 和 `evidence_status=PASS` 均不能成立。

补充验收约束续九：runtime stability summary 自身必须带合法 `timestamp`，且该时间必须落在 `runtime_stability_run.log` 的 `started_at/finished_at` 闭区间内；最终 `field_acceptance_report.txt` 必须下沉完全一致的 `runtime_stability_summary_timestamp`，且最终报告自身 `timestamp` 不得早于该 summary 生成时间。独立 summary 缺自身 `timestamp`、timestamp 畸形、越出 run log 时间窗、最终报告缺少下沉字段、下沉字段不一致或最终报告时间早于 summary 生成时间时，`runtime_stability_status=PASS`、`field_acceptance_status=PASS` 和 `evidence_status=PASS` 均不能成立。

补充验收约束续十：runtime stability CSV 的每条采样 `timestamp` 必须合法且按 `sample_index` 非递减；最终 `field_acceptance_report.txt` 必须下沉与独立 CSV 完全一致的 `runtime_stability_csv_first_timestamp` 和 `runtime_stability_csv_last_timestamp`，首末采样时间必须满足 `first <= last` 并落在 `runtime_stability_run.log` 的 `started_at/finished_at` 闭区间内，且最终报告自身 `timestamp` 不得早于最后一条 CSV 采样时间。独立 CSV 缺 timestamp、timestamp 畸形、采样时间倒序、越出 run log 时间窗、最终报告缺少下沉字段、下沉字段不一致或最终报告时间早于最后采样时间时，`runtime_stability_csv_status=PASS`、`runtime_stability_status=PASS`、`field_acceptance_status=PASS` 和 `evidence_status=PASS` 均不能成立。

补充验收约束续十一：runtime stability CSV 的每条采样 `health_report` 必须解析到当前 session/evidence bundle 内非空 regular file；相对路径按 CSV 所在目录解析，绝对路径必须仍位于当前 session/evidence bundle 内。session 归档板端长稳 CSV 时，`capture_runtime_stability.sh` 必须把可解析的 runtime health 快照复制到当前 session `logs/` 并把 CSV 字段改写为安全 basename；缺失、空文件、越界绝对路径、含 `.`/`..` 路径段或其它不安全引用时，`runtime_stability_csv_status=PASS`、`runtime_stability_status=PASS`、`field_acceptance_status=PASS` 和 `evidence_status=PASS` 均不能成立。

补充验收约束续十二：runtime stability CSV 的每条采样 `health_report` 指向的 runtime health 快照还必须内容通过 runtime health PASS 语义，并与当前 manifest/session 的 `runtime_dir` 一致。独立采样 health 文件缺 timestamp、runtime_dir、磁盘余量、PID、systemd/Docker 状态或来源字段，字段畸形，`runtime_dir` 不一致，systemd/Docker 非 active/running，或来源不是 `systemctl`/`docker_inspect` 时，`runtime_stability_csv_status=PASS`、`runtime_stability_status=PASS`、`field_acceptance_status=PASS` 和 `evidence_status=PASS` 均不能成立。

测试层级：

1. Gazebo：验证流程、状态机、断电恢复逻辑和接口，不作为真实时序最终证明。
2. HIL 回放：用 PCAP/rosbag 验证时间、去畸变、三雷达融合、截面导出和重启，并用 `lio_eval_tools` 统一生成静止漂移、长度误差、恢复时间、错回环、队列堆积和 PPS 抖动报告；工具同时支持最终指标文件 `metrics_file`、规范化事件文件 `event_file` 和验收证据包 `evidence_manifest_file`，显式提供 `event_file` 时优先从事件流聚合指标；提供 `evidence_manifest_file` 时额外校验 metrics/event/bag/pcap/TF/参数/日志/time sync/pps_ptp_wiring/power_loss_resume/runtime health/runtime deployment/runtime stability/section export/field acceptance 证据是否齐全，检查 time sync 报告包含捕获成功、PPS 诊断 PASS、clock offset 诊断 PASS、PPS jitter 和 host/device mean offset，检查 PPS/PTP wiring 报告由 time sync PASS 和人工接线确认文件共同生成且确认总字段 `pps_ptp_wiring_verified` 严格等于 `PASS`、`wiring_confirmation_source=manual_file`，并包含 `pps_wiring_verified=PASS`、`ptp_wiring_verified=PASS`、`wiring_verified_by` 和 `wiring_verified_at`，并输出独立的 `pps_ptp_wiring_status`；检查 section export CSV 包含固定截面表头和至少一条数据行，且所有非空数据行满足 `session_id` 与 evidence manifest 会话一致、7 字段非空、状态来源为 `IDLE_STATIC/CUTTING_STATIC/FWD_MOVE/REV_MOVE/TURNING/CMD_MOVE_NO_DISP/CONFLICT/RELOCALIZING`、质量等级 A/B/C、数值字段可解析、完整度 0-1、RMSE 非负、点数为正整数，并输出 `section_export_status`；检查 power-loss resume 报告总字段 `power_loss_resume_status` 严格等于 `PASS`、来源和恢复时间门槛且来源不是 `manual_env`，其中 `metrics_report` 来源必须带非空 `metrics_report` 字段，且该字段解析后必须与 evidence manifest 的 `metrics_report` 路径一致，`metrics_report` 与 `manual_file` 来源都必须满足 `recovery_time_s <= max_recovery_time_s`，且 manual_file 来源必须下沉 `power_loss_resume_confirmation_overall=PASS`，最终现场验收报告也必须透传该字段；检查 runtime health 快照包含非空且非哨兵值的运行目录、可解析且非负的磁盘余量、严格正整数 `runtime_pid`、`systemd_active=active` 和 `docker_container_status=running`，检查 runtime deployment 报告包含非空且非哨兵值的 `runtime_dir`、systemd/Docker 文件骨架、启动命令 PASS、`deployment_status=PASS`、`systemd_active=active`、`systemd_active_source=systemctl`、`docker_container_status=running`、`docker_container_status_source=docker_inspect` 和 `runtime_process_status=PASS`，检查 runtime stability CSV 包含固定表头与至少一条采样记录，且所有非空采样记录必须满足 `disk_guard_status=PASS`、`watchdog_status=PASS`、`health_report` 非空且不是 `missing` 或 `__DUPLICATE_KEY__`，且不含分号、换行或回车，并检查 runtime stability summary 的 `overall` 字段严格等于 `PASS`、`samples` 为正整数且等于 CSV 非空采样记录数、`interval_s` 显式存在且为严格正整数、failure counters 显式存在且为 0；field acceptance 必须额外确认 time sync 捕获/PPS/clock offset 状态字段严格 PASS、实机 systemd active、Docker running、runtime deployment 总字段严格 PASS、长稳时长为严格数值且 >= 24 h、必须下沉 `runtime_stability_csv_status=PASS` 且 `runtime_stability_sample_count_match=PASS`，并要求 `runtime_stability_interval_s` 与 summary 一致、断电续建验证 PASS 和 PPS/PTP 接线验证 PASS，且 PPS/PTP verified 文件总字段 `pps_ptp_wiring_verified` 必须严格等于 `PASS`，deployment 来源不是 `env_override`，PPS/PTP 与断电续建来源不是 `manual_env`，断电续建 verified 文件总字段 `power_loss_resume_status` 必须严格等于 `PASS`，并必须再次满足 `recovery_time_s <= max_recovery_time_s`；`record_session.sh` 生成脚本与 C++ evidence manifest 校验器读取 key/value 证据时都必须按 trim 后 key 识别重复键，同时保留 value 原文进入污染检查，不能让前导空格重复 key 或回车污染绕过 gate；`validation_report.launch` 已设置离线节点完成后自动收尾，可用于 `record_session.sh` 生成的 `commands/validate_evidence.sh`；长直弱特征、断电恢复、错回环陷阱等场景使用 `scenario_validation_thresholds.txt` 覆盖默认门槛，未显式覆盖的字段必须继承当前 `validation_thresholds.yaml` 或 ROS 参数默认值，显式覆盖字段畸形时必须 fail closed，任一阈值记录重复 key 也必须 fail closed；`metrics_file` 和 `event_file` 的任一分号键值记录不得包含重复 key，数值或文本 key 重复都必须使对应指标 fail closed。
   v3.62 起，HIL/现场验收包中的 manual_file 断电续建证据还必须在独立 `power_loss_resume_verified.txt` 和最终 `field_acceptance_report.txt` 中同时包含 `power_loss_resume_confirmation_keys_status=PASS`；缺失或非 PASS 时，即使 `power_loss_resume_confirmation_overall=PASS` 和恢复时间合法，最终 `field_acceptance_status/evidence_status` 也必须保持 FAIL。
   v4.40 起，HIL/现场验收包中的 runtime health 证据还必须在独立 `runtime_health` 文件和最终 `field_acceptance_report.txt` 中同时包含 `systemd_active_source=systemctl` 与 `docker_container_status_source=docker_inspect` 语义；来源缺失或来自环境覆盖时，`runtime_health_status/field_acceptance_status/evidence_status` 均必须保持 FAIL。
   v4.41 起，HIL/现场验收包中的 runtime health 证据还必须在独立 `runtime_health` 文件中包含合法 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉一致的 `runtime_health_timestamp`，且最终报告 `timestamp` 必须不早于该 health 快照；缺失、不一致或时间倒置时，`runtime_health_status/field_acceptance_status/evidence_status` 均必须保持 FAIL。
   v4.42 起，HIL/现场验收包中的 runtime deployment 证据还必须在独立 `runtime_deployment` 文件中包含合法 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉一致的 `runtime_deployment_timestamp`，且最终报告 `timestamp` 必须不早于该 deployment 快照；缺失、不一致或时间倒置时，`runtime_deployment_status/field_acceptance_status/evidence_status` 均必须保持 FAIL。
   v4.43 起，HIL/现场验收包中的 time sync 证据还必须在独立 `time_sync` 文件中包含合法 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉一致的 `time_sync_timestamp`，且最终报告 `timestamp` 必须不早于该 time sync 快照；缺失、不一致或时间倒置时，`time_sync_status/field_acceptance_status/evidence_status` 均必须保持 FAIL。
   v4.44 起，HIL/现场验收包中的 PPS/PTP wiring 证据还必须在独立 `pps_ptp_wiring` 文件中包含与独立 time sync 完全一致的 `time_sync_timestamp`，最终 `field_acceptance_report.txt` 必须下沉一致的 `pps_ptp_wiring_time_sync_timestamp`；缺失、不一致或 stale wiring 证据时，`pps_ptp_wiring_status/field_acceptance_status/evidence_status` 均必须保持 FAIL。
   v4.45 起，HIL/现场验收包中的 PPS/PTP wiring 证据还必须在独立 `pps_ptp_wiring` 文件中包含合法自身 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉一致的 `pps_ptp_wiring_timestamp`，且最终报告 `timestamp` 必须不早于该 wiring 报告生成时间；缺失、不一致或时间倒置时，`pps_ptp_wiring_status/field_acceptance_status/evidence_status` 均必须保持 FAIL。
   v4.46 起，HIL/现场验收包中的 power-loss resume 证据还必须在独立 `power_loss_resume` 文件中包含合法自身 `timestamp`，最终 `field_acceptance_report.txt` 必须下沉一致的 `power_loss_resume_timestamp`，且最终报告 `timestamp` 必须不早于该 power-loss resume 报告生成时间；缺失、不一致或时间倒置时，`power_loss_resume_status/field_acceptance_status/evidence_status` 均必须保持 FAIL。
   v4.47 起，HIL/现场验收包中的 runtime stability summary 证据还必须在独立 `runtime_stability_summary` 文件中包含合法自身 `timestamp`，该时间必须落在 `runtime_stability_run_log` 的 `started_at/finished_at` 闭区间内，最终 `field_acceptance_report.txt` 必须下沉一致的 `runtime_stability_summary_timestamp`，且最终报告 `timestamp` 必须不早于该 summary 生成时间；缺失、不一致、越窗或时间倒置时，`runtime_stability_status/field_acceptance_status/evidence_status` 均必须保持 FAIL。
   v4.48 起，HIL/现场验收包中的 runtime stability CSV 证据还必须证明每条采样 `timestamp` 合法且按采样顺序非递减；最终 `field_acceptance_report.txt` 必须下沉与独立 CSV 推导值一致的 `runtime_stability_csv_first_timestamp/runtime_stability_csv_last_timestamp`，首末时间必须落在 `runtime_stability_run_log` 的 `started_at/finished_at` 闭区间内，且最终报告 `timestamp` 必须不早于最后一条 CSV 采样时间；缺失、不一致、倒序、越窗或时间倒置时，`runtime_stability_csv_status/runtime_stability_status/field_acceptance_status/evidence_status` 均必须保持 FAIL。
   v4.49 起，HIL/现场验收包中的 runtime stability CSV 证据还必须证明每条采样 `health_report` 指向当前 session/evidence bundle 内非空 regular file；session 归档应把板端 runtime health 快照复制进 `logs/` 并把 CSV 引用改写为安全 basename；缺失、空文件、越界路径或不安全路径引用时，`runtime_stability_csv_status/runtime_stability_status/field_acceptance_status/evidence_status` 均必须保持 FAIL。
   v4.50 起，HIL/现场验收包中的 runtime stability CSV 证据还必须逐条复验 `health_report` 文件内容本身，要求其 runtime health timestamp 合法、`runtime_dir` 与当前 manifest/session 一致、磁盘/PID 合法、systemd active 来源为 `systemctl` 且 Docker running 来源为 `docker_inspect`；内容缺失、内容 FAIL、来源覆盖或 runtime_dir 不一致时，`runtime_stability_csv_status/runtime_stability_status/field_acceptance_status/evidence_status` 均必须保持 FAIL。
   v4.51 起，实际 bag replay/HIL 入口若 bag 内存在外部速度参考，只允许在离线检查报告中记录首个样本用于初始非零速度审计；rosbag play 输入 topic 不得包含速度参考 topic，验收报告必须显式证明 `velocity_reference_played_to_slam=NO` 和 `continuous_velocity_reference_used=NO`。缺失该证明或连续使用速度参考时，实际 bag replay 证据不得标记为 PASS。
   v4.52 起，实际 bag replay/HIL 命令必须使用自有临时 ROS master，不得依赖默认 11311 master；回放脚本退出后不得遗留 `roslaunch/rosbag/rostopic/roscore/rosmaster` 进程。若 cleanup 证据缺失或存在残留进程，实际 bag replay 证据不得标记为 PASS。
   v4.53 起，实际 bag replay/HIL 诊断 capture 必须覆盖回放窗口，timeout 必须随 `duration/rate` 放大；summary 必须包含 `minimum_fusion_published` 与 `fusion_duration_coverage_status`。慢速回放可用于证明算法链路和证据入口可跑，1.0x 全包若覆盖 gate 未过则必须标记为实时吞吐待优化，不能伪装成最终现场验收 PASS。
   v4.54 起，实际 bag replay/HIL 必须捕获 `/diagnostics/lio_local_odometry` 并在 summary 中下沉局部里程计诊断捕获状态、输入云计数、odom 发布计数和注册拒绝计数；local odometry 诊断缺失或 odom 发布计数不为正时，实际 bag replay 证据不得标记为 PASS。
   v4.55 起，实际 bag replay/HIL 的诊断数值解析必须同时兼容 `data: "key=value;..."` 和 ROS `DiagnosticArray` YAML 的跨行 `key:`/`value:` 格式；同一行 key 左边界必须按非 key 字符处理，避免引号后的首个字段被解析为 0。
   v4.56 起，实际 bag replay/HIL 必须同时证明 local odometry 发布覆盖率，summary 必须包含 `minimum_local_odometry_published` 与 `local_odometry_duration_coverage_status`，且该状态必须为 PASS；仅捕获 local odometry 诊断或 odom 发布计数为正不得生成 replay PASS。外部速度参考 topic 仍只能用于起始审计，不得作为连续 SLAM 输入；完整 0.3x 离线质量 PASS 不能替代 1.0x 实时性能验收，1.0x local odometry 覆盖不足、连续拒绝或弱可观测时必须标记为实时优化未闭环。
   v4.57 起，LiDAR+IMU-only 初测 bag 必须在 inspection、plan 和 summary 中显式声明 `actual_bag_test_scope=INITIAL_LIDAR_IMU_ONLY`、`plc_feedback_status=NOT_PRESENT_NA`、`plc_feedback_gate_status=NA_INITIAL_TEST`、`machine_motion_assumption=CONTINUOUS_MOTION` 和 `field_acceptance_requires_plc_feedback=YES`，且 replay 启动命令必须关闭 PLC 依赖的机器状态、mapping control 和 section manager，防止无 PLC 反馈时把持续运动误判为正式 `CONFLICT` 或静止门控证据。该初测证据不能用于 PLC 状态机、section export、PPS/PTP、断电续建、24 h 长稳或最终 `field_acceptance_status=PASS`。
   v4.58 起，HIL/现场验收包中的 section export CSV `state_source` 必须按八态正式枚举校验：`IDLE_STATIC/CUTTING_STATIC/FWD_MOVE/REV_MOVE/TURNING/CMD_MOVE_NO_DISP/CONFLICT/RELOCALIZING`。脚本侧和 C++ evidence manifest 侧必须一致接受这八态、拒绝其它状态；其中 `CMD_MOVE_NO_DISP` 与 `RELOCALIZING` 仍不得生成扩图或稳定图写入证据。
v4.59 起，实际 bag replay 初测必须生成面向 HIL 工具链的独立 metrics/event 证据文件和验证命令，但这些文件必须声明 `validation_scope=ACTUAL_LIDAR_IMU_FRONTEND_ONLY` 与 `field_acceptance_eligible=NO`。dry-run 证据必须保持 FAIL，实际执行证据也只能在 `actual_bag_replay_status=PASS` 时给出 metrics PASS；该桥接不得被 evidence manifest 或 field acceptance 当作 PLC/PPS/PTP/断电续建/长稳替代证据。

v4.60 起，海底隧道 LiDAR+IMU-only 初测 bag 暴露的局部里程计早期 ICP 拒绝级联必须通过算法恢复而不是降低验收门槛处理：`local_icp_odometry_tunnel_bag.yaml` 显式设置 `cloud_queue_size=50` 与 `reseed_keyframe_after_consecutive_rejections=2`，当连续拒绝但几何可观测性仍通过时 reseed keyframe、清除过期运动先验，并在诊断、actual bag summary、metrics 和 event 中下沉 `local_odometry_keyframe_reseeds`。当前 60 s/0.3x 真实 replay 证据为 `reports/actual_bag_replay_tunnel_v460_reseed_60s_rate03`，结果 `actual_bag_replay_status=PASS`、`fusion_published=576/300`、`local_odometry_published=284/60`、`local_odometry_rejected_registrations=14`、`local_odometry_keyframe_reseeds=7`，生成的 `commands/validate_actual_bag_events.sh` 对 `actual_bag_replay_events.txt` 输出 `overall=PASS`。该证据仍仅证明 `ACTUAL_LIDAR_IMU_FRONTEND_ONLY` 初测链路，`field_acceptance_eligible=NO`，不能替代 PLC 反馈、PPS/PTP 接线、断电续建、systemd/Docker 实机启停或 24 h 长稳。

v4.62 起，用户收集的 LiDAR+IMU-only 实际 bag 在进入现场/HIL 前应先跑 `actual_bag_test_suite.sh`，自动生成 smoke/full replay、两段事件校验、suite metrics/events、初始速度审计和 ROS 残留检查。suite PASS 必须同时证明 `smoke_replay_status=PASS`、`full_replay_status=PASS`、`smoke_event_validation_status=PASS`、`full_event_validation_status=PASS`、`ros_residual_status=PASS`、速度参考未进入 SLAM、PLC 缺失按 `NOT_PRESENT_NA` 记录且 `field_acceptance_eligible=NO`；该 PASS 只表示实际数据初测可用，不能替代 PLC、section export、PPS/PTP、断电续建、实机部署或 24 h 长稳证据。

v4.63 起，actual bag suite PASS 后还必须提供并通过 `commands/validate_actual_bag_test_suite.sh`，将 suite summary、metrics/event、smoke/full replay summary、event validation、初始速度审计、bag inspection 和 ROS 残留报告统一复验到 `actual_bag_test_suite_manifest_validation_status=PASS`。校验脚本读取 `key=value` 时必须按字面量 key 拆分，不能用正则误解析 `topic_count[/plc/...]` 等带方括号字段；缺失任一 smoke/full 子证据、PLC topic 计数非 0、速度参考进入 SLAM 或 `field_acceptance_eligible` 非 NO 时，suite manifest validation 必须 FAIL。

v4.64 起，actual bag suite 还必须生成 `commands/audit_field_acceptance_gap.sh` 和 `reports/field_acceptance_gap_report.txt`。该 audit 必须先复用 suite manifest validation 证明 `actual_bag_initial_evidence_status=PASS`，再固定输出 `field_acceptance_ready=NO`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` 和缺失证据清单；audit 命令非零退出是预期行为，用于阻止 LiDAR+IMU-only 初测 PASS 被误当作最终现场 `field_acceptance_status=PASS`。

v4.65 起，actual bag replay/suite 必须支持用户 bag topic override，并把源 topic 显式 remap 到 canonical replay topic 后再进入现有 SLAM launch。inspection、plan 和 summary 证据必须记录源 topic、canonical topic、play topic 列表和被排除的速度参考 topic；`--initial-velocity-topic` 只能用于 `START_ONLY_AUDIT`，不得出现在 rosbag play 输入 topic 中。非法 topic override、源 topic 缺失、字段不满足当前 replay profile 或 topic override 未透传到 smoke/full replay 时，实际 bag 初测证据不得标记为 PASS。

v4.66 起，actual bag replay/suite 对没有外部 `/time_reference` 的用户 LiDAR+IMU-only 初测 bag 必须支持显式 `--no-time-reference`。该模式必须从 `rosbag play --topics` 中移除 time reference topic，并在 inspection、plan、summary、suite manifest 中记录 `time_reference_topic=NONE`、`time_reference_status=NOT_PRESENT_INITIAL_TEST`、`time_sync_evidence_status=NOT_PRESENT_INITIAL_TEST`；它与 `--time-reference-topic` 必须互斥。该模式只放行初测 replay，不得被 final field acceptance、PPS/PTP wiring 或 time sync evidence manifest 当作已完成时间同步证据。

v4.67 起，`validate_actual_bag_test_suite.sh` 必须把 manifest 的 time reference mode 作为整包一致性契约复验。suite summary、smoke/full replay summary、smoke/full inspection 的 `time_reference_status/time_sync_evidence_status` 必须与 manifest 一致；当 manifest 为 `NOT_PRESENT_INITIAL_TEST` 时，smoke/full replay summary 和 inspection 的 `time_reference_topic` 必须为 `NONE`。任何子证据缺字段、混写 `PRESENT_REQUIRED`、或把 no-time-reference suite 伪装成已有 time reference 的情况，都必须使 suite manifest validation FAIL。

v4.68 起，用户实际 bag 在执行 replay/suite 前必须先可由 `actual_bag_profile.sh` 生成 profile 报告和推荐 suite 命令。profile 必须记录 topic 计数、topic 类型、点云字段、frame_id、中心/左/右雷达、IMU、time reference 模式、初始速度参考状态和 `field_acceptance_eligible=NO`；无 time reference 时只能推荐 `--no-time-reference`，有 time reference 时必须推荐具体 `--time-reference-topic`；速度参考只能标记为起步审计，不得进入 replay play topic 或连续 SLAM 输入。profile PASS 只证明 intake 可用，不能替代 `actual_bag_test_suite_status=PASS`、suite manifest validation，更不能替代最终 `field_acceptance_status=PASS`。

v4.69 起，profile 生成的 `commands/run_recommended_actual_bag_test_suite.sh` 必须能直接驱动 smoke/full replay、事件校验、suite summary、`validate_actual_bag_test_suite.sh` 和 gap audit。只有 suite summary 与 manifest validation 均 PASS、ROS 残留为 PASS、速度参考未进入 SLAM、`field_acceptance_eligible=NO` 同时成立时，才能认为用户实际 bag 初测入口闭环；gap audit 仍必须保持 `field_acceptance_ready=NO`，不能因初测链路 PASS 生成最终 `field_acceptance_status=PASS`。

v4.70 起，actual bag profile 还必须在报告层直接声明 LiDAR+IMU-only 初测边界：`plc_feedback_topic_count`、`plc_feedback_status`、`plc_feedback_gate_status=NA_INITIAL_TEST`、`machine_motion_assumption=CONTINUOUS_MOTION`、`vibration_profile=NORMAL` 和 `field_acceptance_requires_plc_feedback=YES` 必须存在。缺少这些字段时，即使 topic 自动识别成功，也不能把 profile 作为用户实际 bag 初测入口证据。

v4.71 起，actual bag profile 若检测到速度参考 topic，必须尝试捕获首个速度样本并写出 `initial_velocity_reference_status`、`initial_velocity_reference_policy=START_ONLY_AUDIT`、`initial_velocity_reference_topic`、首样本时间和速度分量；若无法解析，只能写 `UNPARSEABLE`，不得伪造速度数值。无论是否捕获成功，profile、suite 和 replay 都必须继续证明 `velocity_reference_played_to_slam=NO` 和 `continuous_velocity_reference_used=NO`。

v4.72 起，actual bag profile 必须生成并通过 `commands/validate_actual_bag_profile.sh`。该自检必须复验 profile 顶层状态、LiDAR+IMU-only 初测边界、无 PLC、time reference 模式、起步速度审计、推荐 suite 命令、`field_acceptance_eligible=NO`、`velocity_reference_played_to_slam=NO` 和 `continuous_velocity_reference_used=NO`；缺字段、重复 key、畸形行、推荐命令缺失、速度连续使用、或把初测 profile 伪装成最终现场验收资格时，profile validation 必须 FAIL。

v4.73 起，profile 生成的 `commands/run_recommended_actual_bag_test_suite.sh` 必须先执行同目录 `validate_actual_bag_profile.sh`，再执行 `actual_bag_test_suite.sh --execute`。若 profile validation 非 PASS，推荐 suite 入口必须因 `set -euo pipefail` 停止，不能启动 smoke/full replay、suite manifest validation 或 field acceptance gap audit。

v4.74 起，`commands/validate_actual_bag_profile.sh` 必须反查 `commands/run_recommended_actual_bag_test_suite.sh` 的入口 gate，要求推荐入口脚本存在严格模式、包含 profile validator 调用、包含 actual bag suite 调用，且 validator 调用顺序早于 suite 调用。若入口脚本缺失、无严格模式、缺 validator、缺 suite 或顺序反转，profile validation 必须 FAIL，并输出 `recommended_suite_entry_gate_status=FAIL`。

v4.75 起，actual bag suite 必须生成 `commands/validate_field_acceptance_gap.sh`，独立校验 `field_acceptance_gap_report.txt` 的缺口语义。该 validator 必须要求 gap audit 总状态仍为 FAIL、初测证据为 PASS、最终 `field_acceptance_ready=NO`、`field_acceptance_eligible=NO`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`，并要求 PLC feedback、section export、PPS/PTP wiring、power-loss resume、runtime deployment、24 h runtime stability、final field acceptance report 和 `required_next_evidence` 缺口清单均完整匹配；任一字段被篡改、缺失、重复 key 或畸形行时，`field_acceptance_gap_validation_status` 必须 FAIL。

v4.76 起，actual bag suite 的 `--execute` 模式必须通过 `commands/run_verified_suite.sh` 执行完整闭环，而不是只运行 smoke/full replay。verified wrapper 必须按顺序运行 `run_suite.sh`、`validate_actual_bag_test_suite.sh`、预期退出 1 的 `audit_field_acceptance_gap.sh` 和 `validate_field_acceptance_gap.sh`；若 gap audit 未以 1 退出、manifest validation 失败或 gap validation 失败，则 `--execute` 必须非零退出。成功时 suite summary 必须追加 `verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_gap_audit_exit=1` 和 `field_acceptance_gap_validation_after_execute=PASS`。

v4.77 起，`validate_actual_bag_test_suite.sh` 必须对 suite summary 中已出现的 verified execute 字段执行严格复验：`verified_suite_status=PASS`、`suite_manifest_validation_after_execute=PASS`、`field_acceptance_gap_audit_exit=1` 和 `field_acceptance_gap_validation_after_execute=PASS`。未执行 verified wrapper 的 dry-run/synthetic suite 可输出 `NOT_PRESENT_NA`，但字段一旦出现，任何 FAIL、错误退出码、缺失值或其它值都必须使 suite manifest validation FAIL。

v4.78 起，`commands/run_verified_suite.sh` 必须在追加 verified execute 字段后再次运行 `validate_actual_bag_test_suite.sh`，使最终落盘的 `reports/actual_bag_test_suite_manifest_validation.txt` 自动包含 `suite_summary_verified_suite_status_status=PASS`、`suite_summary_suite_manifest_validation_after_execute_status=PASS`、`suite_summary_field_acceptance_gap_audit_exit_status=PASS` 和 `suite_summary_field_acceptance_gap_validation_after_execute_status=PASS`。不得依赖人工事后重跑 manifest validator 来补齐这些终态字段。

v4.79 起，actual bag profile 必须在 `reports/actual_bag_profile.txt` 写出 `recommended_suite_verified_execute_required=YES`，且 `commands/validate_actual_bag_profile.sh` 必须校验该字段为 YES。profile 生成的推荐入口不得只指向普通 suite 或 dry-run 计划，必须通过 `actual_bag_test_suite.sh --execute` 进入 verified execute wrapper；若该字段缺失、为 NO、被篡改或推荐入口绕过 profile validator，profile validation 必须 FAIL。

v4.80 起，`commands/validate_actual_bag_profile.sh` 必须反查 `commands/run_recommended_actual_bag_test_suite.sh` 本体包含 `--execute`。仅 profile 报告中的 `recommended_suite_command` 包含 `--execute` 不足以通过；若入口脚本被篡改为删除 `--execute`、绕过 verified wrapper 或只生成 dry-run 计划，profile validation 必须 FAIL，并下沉 `recommended_suite_entry_gate_status=FAIL`。

v4.81 起，`commands/validate_actual_bag_profile.sh` 必须要求 `commands/run_recommended_actual_bag_test_suite.sh` 本体包含 `reports/actual_bag_profile.txt` 中的 `recommended_suite_command` 原文。入口脚本若被篡改为不同 replay rate、不同 `--out`、不同 topic、不同 bag、缺失速度审计隔离参数或其它命令，即使仍包含 `actual_bag_test_suite.sh --execute`，profile validation 也必须 FAIL，并下沉 `recommended_suite_entry_gate_status=FAIL`。

v4.82 起，若 actual bag profile 中 `initial_velocity_reference_topic` 存在且不为 `missing`，`commands/validate_actual_bag_profile.sh` 必须要求 `recommended_suite_command` 包含同一 topic 的 `--initial-velocity-topic` 参数。该参数只用于 `START_ONLY_AUDIT` 起步速度审计，不得进入 rosbag play 的连续 SLAM 输入；但也不得从推荐 suite 入口中丢失，否则 profile validation 必须 FAIL，并下沉 `recommended_suite_command_status=FAIL`。

v4.83 起，actual bag suite manifest validator 必须复验 smoke/full 两段 `reports/initial_velocity_reference.txt` 的 `initial_velocity_reference_status=CAPTURED`。在执行 suite 时若提供了 `--initial-velocity-topic`，仅有 `initial_velocity_reference_policy=START_ONLY_AUDIT`、`velocity_reference_played_to_slam=NO` 和 `continuous_velocity_reference_used=NO` 不足以通过；`MISSING`、`UNPARSEABLE`、`FAIL`、缺字段、重复 key 或畸形行都必须使 `actual_bag_test_suite_manifest_validation_status=FAIL`，并下沉对应 smoke/full 初始速度捕获状态 FAIL。该 gate 只证明初始非零速度样本被捕获并隔离，不允许速度参考作为连续 SLAM 输入。

v4.84 起，actual bag profile/replay/suite 必须区分“有速度参考”和“无速度参考”两种初测路径。有速度参考或显式 `--initial-velocity-topic` 时，`initial_velocity_reference_required=YES`，smoke/full 仍必须 `CAPTURED`；无速度参考时必须显式使用 `--no-initial-velocity-reference`，并在 profile、replay、suite summary、inspection、initial velocity report 和 manifest 中写出 `initial_velocity_reference_required=NO`、`initial_velocity_reference_status=NOT_PRESENT_INITIAL_TEST`、`initial_velocity_reference_topic=NONE`、`initial_velocity_reference_policy=NOT_AVAILABLE_INITIAL_TEST`。`--no-initial-velocity-reference` 与 `--initial-velocity-topic` 必须互斥，且该模式不得检查、推荐或播放默认 `/novatel_data/inspvax`；生成的 `run_suite.sh` 也必须在 `set -u` 下显式注入并写出这些初始速度模式字段。

v4.85 起，actual bag suite 的 `field_acceptance_gap_report.txt` 必须同时包含 `required_next_evidence` 和 7 类缺口采集命令字段：`plc_feedback_collection_command`、`section_export_collection_command`、`pps_ptp_wiring_collection_command`、`power_loss_resume_collection_command`、`runtime_deployment_collection_command`、`runtime_stability_24h_collection_command`、`field_acceptance_collection_command`。`validate_field_acceptance_gap.sh` 必须对这些字段执行严格等值复验；任一字段缺失、篡改、重复 key 或畸形行时 gap validation 必须 FAIL。上述命令仅作为补齐真实现场/HIL 证据的采集指引，不得把 LiDAR+IMU-only 初测 suite 转换为最终 `field_acceptance_status=PASS`。

v4.86 起，actual bag suite 必须生成独立的 `actual_bag_initial_test_readiness` 证据入口。`audit_actual_bag_initial_test_readiness.sh` 只有在 suite summary、manifest validation、field acceptance gap validation、smoke/full 初始速度捕获/隔离、PLC 缺失初测边界和 ROS 残留检查全部通过时，才允许输出 `actual_bag_initial_test_readiness_status=PASS` 与 `actual_bag_user_bag_test_ready=YES`；`validate_actual_bag_initial_test_readiness.sh` 必须拒绝缺失、篡改、重复 key 或畸形 readiness 报告。该 ready 只代表可以用同类 LiDAR+IMU-only bag 做初步软件回放测试，报告中必须继续声明 `field_acceptance_eligible=NO` 与 `field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`，不得替代 PLC/PPS/PTP/断电续建/实机部署/24 h 长稳等最终现场证据。

v4.87 起，actual bag profile 推荐入口必须显式绑定 suite readiness gate。`reports/actual_bag_profile.txt` 必须写出 `recommended_suite_initial_readiness_required=YES`，`commands/validate_actual_bag_profile.sh` 必须校验该字段并反查 `commands/run_recommended_actual_bag_test_suite.sh` 在推荐的 `actual_bag_test_suite.sh --execute` 之后调用推荐 suite 输出目录下的 `commands/validate_actual_bag_initial_test_readiness.sh`。若 profile 报告缺失或篡改该字段、推荐入口缺少 readiness validator、validator 调用顺序早于 suite、或推荐入口只跑 suite 不复验 readiness，则 profile validation 必须 FAIL。该约束只闭环用户初测入口，不得把 LiDAR+IMU-only profile 或 suite 证据升级为最终 `field_acceptance_status=PASS`。

v4.88 起，actual bag suite 必须生成独立的 `field_acceptance_handoff` 交接证据。`generate_field_acceptance_handoff.sh` 必须在 `validate_actual_bag_initial_test_readiness.sh` 与 `validate_field_acceptance_gap.sh` 均 PASS 后，才允许输出 `field_acceptance_handoff_status=PASS` 和 `field_acceptance_handoff_ready=YES`；报告必须包含完整 7 类真实现场/HIL 证据采集命令、`final_gate_command=record_session.sh generated commands/validate_evidence.sh`、`field_acceptance_ready=NO`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` 和完整 `required_next_evidence`。`validate_field_acceptance_handoff.sh` 必须拒绝缺失、重复 key、畸形行、采集命令篡改、final gate 篡改、或把 handoff 报告改成最终 `field_acceptance_status=PASS` 的情况。该 handoff 只表示“可以进入真实现场证据采集”，不得替代 PLC/PPS/PTP/断电续建/实机部署/24 h 长稳或最终 field acceptance 报告。

v4.89 起，actual bag profile 推荐入口必须显式绑定 field acceptance handoff gate。`reports/actual_bag_profile.txt` 必须写出 `recommended_suite_field_acceptance_handoff_required=YES`，`commands/validate_actual_bag_profile.sh` 必须校验该字段并反查 `commands/run_recommended_actual_bag_test_suite.sh` 在 suite verified execute 和 readiness validator 之后调用推荐 suite 输出目录下的 `commands/validate_field_acceptance_handoff.sh`。若 profile 报告缺失或篡改该字段、推荐入口缺少 handoff validator、handoff validator 调用早于 suite/readiness、或推荐入口只跑到 readiness 不复验 handoff，则 profile validation 必须 FAIL。该约束只闭环用户从 profile 启动的初测到现场证据交接入口，不得把 LiDAR+IMU-only profile、suite、readiness 或 handoff 证据升级为最终 `field_acceptance_status=PASS`。

v4.90 起，actual bag suite 必须生成独立的 `field_acceptance_handoff_manifest` bundle manifest。该 manifest 必须列出 suite summary、suite manifest、suite manifest validation、field acceptance gap report/validation、actual bag readiness report/validation、field acceptance handoff report/validation 以及对应 validator 命令；`validate_field_acceptance_handoff_manifest.sh` 必须复验这些文件存在、命令可执行、各级状态一致为 PASS，并继续要求 `field_acceptance_eligible=NO`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY` 和完整 `required_next_evidence`。缺失、篡改、重复 key、畸形行、validator 命令不可执行或任一依赖报告状态不一致时，handoff manifest validation 必须 FAIL。该 bundle manifest 只证明交接证据包完整，不得替代 PLC/PPS/PTP/断电续建/实机部署/24 h 长稳或最终 field acceptance 报告。

v4.91 起，actual bag profile 推荐入口必须显式绑定 handoff bundle manifest gate。`reports/actual_bag_profile.txt` 必须写出 `recommended_suite_field_acceptance_handoff_manifest_required=YES`，`commands/validate_actual_bag_profile.sh` 必须校验该字段并反查 `commands/run_recommended_actual_bag_test_suite.sh` 在 suite verified execute、readiness validator 和 handoff validator 之后调用推荐 suite 输出目录下的 `commands/validate_field_acceptance_handoff_manifest.sh`。若 profile 报告缺失或篡改该字段、推荐入口缺少 handoff manifest validator、validator 调用早于 suite/readiness/handoff、或推荐入口只跑到 handoff 不复验 bundle manifest，则 profile validation 必须 FAIL。该约束只闭环用户从 profile 启动的初测到现场证据交接包，不得把 LiDAR+IMU-only profile、suite、readiness、handoff 或 handoff bundle manifest 证据升级为最终 `field_acceptance_status=PASS`。

v4.92 起，actual bag suite 必须生成独立的 `field_acceptance_collection_plan` 现场采集计划。`generate_field_acceptance_collection_plan.sh` 只有在 `validate_field_acceptance_handoff.sh` 与 `validate_field_acceptance_handoff_manifest.sh` 均 PASS 后，才允许输出 `field_acceptance_collection_plan_status=PASS` 和 `collection_plan_ready=YES`；计划必须固定列出 PLC feedback、section export、PPS/PTP wiring、power-loss resume、runtime deployment、24 h runtime stability、final field acceptance 七步采集顺序和命令，并写出 `final_success_gate=record_session.sh generated commands/validate_evidence.sh => field_acceptance_status=PASS`。`validate_field_acceptance_collection_plan.sh` 必须拒绝缺失、重复 key、畸形行、采集命令篡改、最终成功门槛篡改、引用 handoff/handoff manifest 状态不一致或把初测计划改成最终 PASS 的情况。`actual_bag_profile.sh` 推荐入口必须显式绑定该 gate：profile 报告写出 `recommended_suite_field_acceptance_collection_plan_required=YES`，推荐入口在 handoff manifest validator 后调用推荐 suite 输出目录下的 `commands/validate_field_acceptance_collection_plan.sh`；缺字段、字段篡改或入口缺少 collection plan validator 均必须使 profile validation FAIL。该计划只表示“下一步现场证据该如何收集并最终用哪个 gate 判定”，不得替代真实 PLC/PPS/PTP/断电续建/实机部署/24 h 长稳或最终 field acceptance 报告。

v4.93 起，actual bag suite manifest validator 必须把 collection plan 纳入上层复验。`actual_bag_test_suite_manifest.txt` 必须声明 `field_acceptance_collection_plan_validation=reports/field_acceptance_collection_plan_validation.txt`；当 `actual_bag_test_suite_summary.txt` 中存在 `field_acceptance_collection_plan_after_execute=PASS`，或 collection plan validation 报告已经生成时，`validate_actual_bag_test_suite.sh` 必须重新复验 collection plan 文件存在、collection plan validation 文件存在、计划本体 `field_acceptance_collection_plan_status=PASS`、`collection_plan_ready=YES`、`field_acceptance_eligible=NO`、`field_acceptance_status=NOT_ELIGIBLE_INITIAL_LIDAR_IMU_ONLY`、`field_acceptance_handoff_manifest_validation_status=PASS`、最终成功门槛文本精确匹配，以及 collection plan validation 报告总状态和 `final_success_gate_status` 均为 PASS。任一字段缺失、篡改、重复 key、畸形行或 validation 报告状态 FAIL，必须使 suite manifest validation FAIL。该约束保证只重跑 suite manifest validator 时也能发现现场采集计划污染，但仍不得把 LiDAR+IMU-only 初测证据升级为最终 `field_acceptance_status=PASS`。

v4.94 起，actual bag profile 推荐入口必须显式绑定 collection plan 后置 suite manifest revalidation。`reports/actual_bag_profile.txt` 必须写出 `recommended_suite_collection_plan_manifest_revalidation_required=YES`，`commands/validate_actual_bag_profile.sh` 必须校验该字段，并反查 `commands/run_recommended_actual_bag_test_suite.sh` 在推荐 suite 输出目录下的 `commands/validate_field_acceptance_collection_plan.sh` 之后再次调用 `commands/validate_actual_bag_test_suite.sh`。若 profile 报告缺字段或字段被篡改、推荐入口缺少后置 suite manifest validator、validator 调用早于 collection plan validator、或入口只跑 collection plan validator 不重验 suite manifest，则 profile validation 必须 FAIL。该约束只闭环用户从 profile 启动的 LiDAR+IMU-only 初测到现场采集计划上层复验，不得把 profile、suite、collection plan 或 suite manifest revalidation 证据升级为最终 `field_acceptance_status=PASS`。

v4.95 起，actual bag profile 推荐入口的 validator 目录必须与推荐 suite 输出目录绑定。`reports/actual_bag_profile.txt` 中的 `recommended_suite_out` 必须与 `commands/run_recommended_actual_bag_test_suite.sh` 中的 `suite_out=` 变量完全一致，`commands/validate_actual_bag_profile.sh` 必须把该一致性纳入 `recommended_suite_entry_gate_status`。若入口脚本保留正确 `actual_bag_test_suite.sh --out <recommended_suite_out>` 命令但把 `suite_out` 改到其它目录，或缺失 `suite_out` 绑定行，则 profile validation 必须 FAIL。该约束防止从用户 profile 入口启动真实 suite 后，后续 readiness、handoff、collection plan 和 suite manifest validators 误验其它目录。

v4.96 起，actual bag profile 报告中的推荐 suite 输出目录必须锚定在当前 profile 根目录。`recommended_suite_out` 必须精确等于 `<profile_root>/recommended_suite`，并同时满足 v4.95 的入口脚本 `suite_out=` 一致性；若 profile 报告和入口脚本被同步篡改到外部目录、兄弟目录或其它 session 目录，即使二者彼此一致，`commands/validate_actual_bag_profile.sh` 也必须输出 `recommended_suite_out_status=FAIL` 并拒绝 profile validation PASS。该约束防止 profile 证据与其它 suite 证据目录拼接形成伪闭环。

v4.97 起，actual bag suite manifest 中所有报告路径必须固定为 suite 生成时的相对路径。`summary`、`metrics_report`、`event_file`、`ros_residual_report`、smoke/full replay summary、smoke/full HIL validation、smoke/full initial velocity、smoke/full inspection、`field_acceptance_gap_report`、`actual_bag_initial_test_readiness`、`field_acceptance_handoff`、`field_acceptance_handoff_manifest`、`field_acceptance_collection_plan` 和 `field_acceptance_collection_plan_validation` 等键不得使用绝对路径、`../` 或其它 session/suite 目录；即使外部报告内容均为 PASS，`commands/validate_actual_bag_test_suite.sh` 也必须输出对应 `*_path_status=FAIL` 并拒绝 suite manifest validation PASS。该约束防止 actual bag suite 证据包通过路径拼接复用其它 suite 的 PASS 报告。

v4.98 起，field acceptance handoff bundle manifest 中所有报告路径必须固定为当前 suite 内相对路径。`suite_summary`、`suite_manifest`、`suite_manifest_validation`、`field_acceptance_gap_report`、`field_acceptance_gap_validation`、`actual_bag_initial_test_readiness`、`actual_bag_initial_test_readiness_validation`、`field_acceptance_handoff` 和 `field_acceptance_handoff_validation` 等键不得使用绝对路径、`../` 或其它 session/suite 目录；即使外部报告内容均为 PASS，`commands/validate_field_acceptance_handoff_manifest.sh` 也必须输出对应 `*_path_status=FAIL` 并拒绝 handoff manifest validation PASS。该约束防止 handoff 交接证据包通过路径拼接复用其它 suite 的 PASS 报告。

v4.99 起，field acceptance collection plan 中的 source provenance 路径必须固定为当前 suite `reports/` 下对应报告的绝对路径。`source_field_acceptance_handoff`、`source_field_acceptance_handoff_validation`、`source_field_acceptance_handoff_manifest` 和 `source_field_acceptance_handoff_manifest_validation` 不得指向外部目录、兄弟 suite 或其它 session 报告；即使外部报告内容均为 PASS，`commands/validate_field_acceptance_collection_plan.sh` 也必须输出对应 `source_field_acceptance_*_path_status=FAIL` 并拒绝 collection plan validation PASS。该约束防止现场采集计划通过 source 字段拼接外部 handoff 证据形成伪 provenance。

v5.00 起，actual bag suite manifest validator 必须对 collection plan source provenance 做上层复验。`commands/validate_actual_bag_test_suite.sh` 在 collection plan 已执行或 validation 报告存在时，必须直接检查 `field_acceptance_collection_plan.txt` 中 `source_field_acceptance_handoff`、`source_field_acceptance_handoff_validation`、`source_field_acceptance_handoff_manifest` 和 `source_field_acceptance_handoff_manifest_validation` 精确等于当前 suite `reports/` 下对应报告路径，并复验 `field_acceptance_collection_plan_validation.txt` 中四个 `source_field_acceptance_*_path_status=PASS`。若 plan 本体被篡改但 validation 报告仍是旧 PASS，suite manifest validation 仍必须 FAIL，防止 stale validation 报告掩盖 source provenance 污染。

v5.01 起，actual bag profile 推荐入口 gate 必须只解析非空、非注释的可执行行。`commands/validate_actual_bag_profile.sh` 检查 `commands/run_recommended_actual_bag_test_suite.sh` 时，必须要求 profile validator、`actual_bag_test_suite.sh --execute`、readiness validator、handoff validator、handoff manifest validator、collection plan validator 和 post-collection suite manifest validator 以真实命令行按顺序出现；注释、说明文本、echo 片段或其它非执行文本中的同名脚本不得满足任何 recommended suite entry gate。

v5.02 起，actual bag profile 推荐入口中的 `actual_bag_test_suite.sh --execute` suite 命令必须以非注释可执行行精确等于 profile 报告的 `recommended_suite_command`；`echo`、wrapper、comment、partial text 或其它只包含该命令文本的行不得满足 `recommended_suite_entry_gate`。

v5.03 起，actual bag profile 推荐入口中的 `suite_out=` 可执行赋值行必须唯一存在、精确等于 profile 报告的 `recommended_suite_out`，并且必须早于推荐 suite 命令；任何后续 `suite_out=` 重写都必须使 `recommended_suite_entry_gate` FAIL，防止真实 suite 输出目录与后续 validators 目录脱钩。

v5.04 起，actual bag profile 推荐入口中 readiness、handoff、handoff manifest、collection plan 和 post-collection suite manifest revalidation 的所有后置顺序 gate 必须锚定到 profile 报告中的精确 `recommended_suite_command`，不得使用任意包含 `actual_bag_test_suite.sh` 的行作为 suite 已执行锚点；伪 `echo`、wrapper、comment 或 partial text 均不能使后置 gate PASS。

v5.05 起，actual bag profile 推荐入口从 profile validator 到 post-collection suite manifest revalidation 的闭环窗口不得包含 `exit`、`return` 或 `exec` 提前终止命令。若入口脚本在真实 suite 或后置 validators 前提前退出，即使保留精确 `recommended_suite_command` 和后置 validator 文本，profile validation 也必须 FAIL，并下沉 `recommended_suite_entry_gate_status=FAIL`。

v5.06 起，actual bag profile 推荐入口提前终止 gate 必须识别 shell 控制结构中的提前终止 token。闭环窗口内出现 `if true; then exit 0; fi`、`; return`、`&& exec` 等 shell 边界上的 `exit`、`return` 或 `exec` 时，profile validation 必须 FAIL，并下沉 `recommended_suite_entry_gate_status=FAIL`。

v5.07 起，actual bag profile 推荐入口 strict mode 必须在 profile validator 到 post-collection suite manifest revalidation 的闭环窗口内持续有效。闭环窗口内出现 `set +e`、`set +u` 或 `set +o pipefail` 时，profile validation 必须 FAIL，并下沉 `recommended_suite_entry_gate_status=FAIL`。

v5.08 起，actual bag profile 推荐入口 strict mode relaxation gate 必须识别 shell 控制结构中的 `set` token。闭环窗口内出现 `if true; then set +e; fi`、`; set +u` 或 `&& set +o pipefail` 等包裹式关闭 strict mode 时，profile validation 必须 FAIL，并下沉 `recommended_suite_entry_gate_status=FAIL`。

v5.09 起，actual bag profile 必须允许 PLC topic 在 profile-only 阶段出现。`plc_feedback_topic_count=0` 时必须下沉 `plc_feedback_status=NOT_PRESENT_NA` 与 `plc_feedback_gate_status=NA_INITIAL_TEST`；`plc_feedback_topic_count>0` 时必须下沉 `plc_feedback_status=PRESENT_PROFILE_ONLY` 与 `plc_feedback_gate_status=PRESENT_PROFILE_ONLY_FIELD_VALIDATION_REQUIRED`，同时仍要求 `field_acceptance_eligible=NO`，防止 topic 存在被误解释为最终 PLC 反馈验收 PASS。

3. 控制区现场：验证截面质量、长度误差、TCA 和长稳。

`lio_eval_tools` 规范化事件流约定：

| 字段/事件 | 用途 | 聚合规则 |
|---|---|---|
| `scenario`、`session_id` | 关联场景阈值和本次回放会话 | 取事件流中首个非空值 |
| `static_drift_m` | 静止待机/静止截割漂移 | 取绝对值最大值 |
| `chainage_m` + `reference_chainage_m` | 长度控制误差 | 按 `abs(chainage-reference)/max(abs(reference),1.0)*100` 计算百分比，取最大值 |
| `length_error_percent` | 已由外部真值系统计算出的长度误差 | 若存在则直接参与最大值统计 |
| `event=power_loss` 到 `event=recovered`/`event=recovery_complete` | 断电续建恢复耗时 | 取恢复时间最大值 |
| `wrong_loop` | 错回环验收 | 累加错误闭环次数 |
| `queue_backlog` | 在线链路堆积 | 取最大队列长度 |
| `pps_jitter_ms` | PPS/设备时间稳定性 | 取绝对值最大值 |

示例命令：

```bash
source catkin_ws/devel/setup.bash
roslaunch lio_eval_tools validation_report.launch \
  event_file:=$(rospack find lio_eval_tools)/config/sample_replay_events.txt \
  report_file:=/tmp/tunnel_lio_event_validation_report.txt

roslaunch lio_eval_tools validation_report.launch \
  evidence_manifest_file:=$(rospack find lio_eval_tools)/config/sample_evidence_manifest.txt \
  report_file:=/tmp/tunnel_lio_evidence_validation_report.txt

# 现场/HIL session 归档完成后，在该 session 下生成指标报告和证据完整性报告
roslaunch mine_slam_bringup record_session.launch \
  session_name:=hil_power_loss_001 \
  scenario:=POWER_LOSS_ORIGIN \
  runtime_dir:=/tmp/tunnel_lio_runtime/board_alpha \
  time_status_topic:=/time/status \
  pps_topic:=/time/pps_event \
  start_pcap:=true
# 现场完成 PPS/PTP 接线实物确认后，先写人工确认文件，再由脚本结合 time sync 证据生成 verified 报告。
printf "pps_ptp_wiring_verified=PASS\npps_wiring_verified=PASS\nptp_wiring_verified=PASS\nwiring_verified_by=qa_operator\nwiring_verified_at=$(date --iso-8601=seconds)\n" > /tmp/tunnel_lio_sessions/<session_id>/reports/pps_ptp_wiring_confirmation.txt
/tmp/tunnel_lio_sessions/<session_id>/commands/capture_pps_ptp_wiring.sh
# 断电续建可由 metrics 报告自动证明；人工兜底必须写入实测 recovery_time_s、有效文本确认人和确认时间，再生成 verified 报告。
printf "power_loss_resume_status=PASS\nrecovery_time_s=25\nresume_verified_by=qa_operator\nresume_verified_at=$(date --iso-8601=seconds)\n" > /tmp/tunnel_lio_sessions/<session_id>/reports/power_loss_resume_confirmation.txt
/tmp/tunnel_lio_sessions/<session_id>/commands/capture_power_loss_resume.sh
/tmp/tunnel_lio_sessions/<session_id>/commands/validate_evidence.sh
```

## 12. 风险与对策

| 风险 | 影响 | 对策 | 责任 |
|---|---|---|---|
| ROS 时间被当采样时间 | 去畸变和融合发虚 | 设备时间/PPS/时偏标定优先 | SYS |
| TF 方向或欧拉角顺序错误 | 三雷达重影、截面双线 | P0 TF 审计和静态复测 | FE+SYS |
| IMU 单独软隔振 | LiDAR-IMU 外参在振动下变化 | 三雷达 + IMU 共刚体传感器岛 | ME |
| 静止截割误写位移 | 地图长期被写坏 | 八态状态机、位姿冻结、降权累积 | FE+SYS |
| 长直纵向退化 | 长度漂移 | 弱方向冻结/限幅、TCA、低频后端 | FE+BE |
| 单标靶误认 | 错误续建 | TCA 上下文，非唯一拒绝 | FE+BE+ME |
| 错回环 | 稳定图被拉坏 | 候选强验证，只闭稳定区 | BE |
| 存储不足 | 问题不可复现 | 滚动缓存 + 事件保全 + 开发外接 SSD | SYS+QA |
| 板端算力不足 | 实时堆积 | 在线最小集，回环和重建低频/离线 | SYS |
| 活动工作面污染基线图 | 后续重定位下降 | 活动会话图先暂存，验证后晋升 | BE |

## 13. 仓库落地建议

当前仓库已按统一 catkin 工作区组织，后续应在以下结构内继续扩展。保留现有 ROS 包名，避免影响已有 `roslaunch`、`$(find package)`、include 和 topic 配置：

```text
catkin_ws/
  src/
    imu_modbus_driver/            # 保留并增强：时间、PPS、诊断、frame_id
    lidar_fusion/                 # 保留并增强：sensor_id、设备时间诊断、重叠残差
      src/multi_lidar_fusion.cpp
      config/*.yaml               # 保留 timoo/tmlidar 两套配置
    timoo*/                       # timoo 雷达驱动包族
    lidar_*/                      # tmlidar 雷达驱动包族

    # 后续新增建议包
    lio_time_manager/             # 时间映射、PPS、时偏估计
    lio_calib_manager/            # 外参 YAML、TF 审计、标定健康
    lio_preprocess/               # 点级去畸变、过滤、质量赋权
    lio_frontend/                 # 几何优先注册、退化保护
    machine_state_manager/        # PLC Modbus 和八态状态机
    map_session_manager/          # 局部图、活动图、稳定图、WAL
    section_manager/              # 截面数据库和导出
    tca_manager/                  # 标靶/TCA
    slam_backend_manager/         # 回环候选和稳定图治理
    lio_eval_tools/               # 已落地：回放/HIL 事件聚合、指标判定和报告
```

首轮开发不建议把所有新功能塞进 `lidar_fusion` 或 IMU 驱动。驱动层职责是“正确、完整、可诊断地输出数据”，算法层职责是“基于标准接口处理数据”。这样后续更容易替换雷达驱动族，也更容易定位现场问题。

## 14. 维护与迭代机制

每次迭代必须更新以下内容：

| 内容 | 维护位置 |
|---|---|
| 模块状态、负责人、当前风险 | 本规划文档或派生周报 |
| 外参和时间偏移版本 | `calib/*.yaml` + 标定报告 |
| 参数版本 | `config/` + session 参数快照 |
| 数据集和回放结果 | `reports/` 或外部归档 |
| 验收指标 | QA 报告 |
| 现场问题和复现证据 | 事件切片库 |

建议每周按以下格式维护进度：

| 模块 | 本周完成 | 下周计划 | 阻塞 | 指标变化 | 是否影响里程碑 |
|---|---|---|---|---|---|
| Mxx | 具体交付 | 具体任务 | 人/设备/数据/现场 | 数值化 | 是/否 |

## 15. 近期立即执行清单

1. 固定三雷达驱动族和 topic 命名，确认 `ring/time/intensity` 字段。
2. 用现场 IMU 复核 `catkin_ws/src/imu_modbus_driver` 已落地的 frame_id/topic/rate 参数、真实发布频率统计、读取 RTT 统计、Modbus 失败诊断阈值、饱和阈值、温度寄存器/比例系数/温漂阈值和三组协方差参数，并补充 PPS/硬件时间字段。
3. 建立 `calib/extrinsics.yaml` 和 TF 审计脚本，消除外参方向歧义。
4. 用当前 `lidar_fusion` 跑三雷达静态叠加，记录重叠区残差和同步 span。
5. 用现场 PLC 寄存器表复核 `machine_state_manager` 中 PLC Modbus 节点的寄存器索引、有效位、截割位和履带速度比例系数，并采集状态机离线回放样本。
6. 用现场设备验证 `record_session.sh`/`record_session.launch` 生成的 PCAP + rosbag + 参数 + TF + reports + time sync + runtime health + runtime deployment + runtime stability + field acceptance + `evidence_manifest.txt` 统一 session 归档，按场景传入 `scenario`，运行 `commands/validate_evidence.sh` 生成证据完整性报告，并确认 `/time/status`、`/time/pps_event`、tcpdump 网卡名和权限策略。
7. 采集四类 P0 数据：静止待机、静止截割、短时前进/后退、断电重启。
8. 基于这些数据先验收“静止不漂”和“可复现”，再进入三雷达前端优化。

本规划文档后续应作为开发进度维护和迭代更新的主索引。任何算法增强都应先映射到对应模块、接口和验收项，再进入实现，避免出现“算法能跑但系统不可验收”的情况。
