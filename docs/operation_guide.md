# Elevator-LIO 建图、定位与电梯模式操作指南

本文对应当前源码，包括 `/current_pose` 输出和 `/initial_pose` 手动重定位。以 **Ubuntu 22.04 + ROS 2 Humble + Livox MID-360** 为主要示例，ROS 1 命令见文末。路径中的 `~/ws_elevator_lio_ros2` 请替换为实际工作空间。

手动重定位的代码需要在目标 ROS 环境重新编译后使用；当前 Windows 编辑环境尚未完成该功能的 ROS 编译与运行验证。

## 1. 先分清两种运行模式

| 项目 | mapping 建图模式 | relocation 定位模式 |
| --- | --- | --- |
| 运行配置 | `yaml/runtime/mapping.yaml` | `yaml/runtime/relocation.yaml` |
| `relocation.relocation_enable` | `false` | `true` |
| 参考地图 | 从传感器扫描建立地图 | 启动时读取已有 PCD |
| 初始位姿 | IMU 初始化，位置从原点开始 | 默认等待手动发布 `/initial_pose` |
| 地图更新 | 持续增加点云，维护局部地图 | 保留参考地图，不增量加点、不裁剪 |
| 回环后端 | 当前配置默认开启 | 自动禁用 |
| 正常退出保存地图 | 启用保存配置后保存 | 不保存累计建图结果 |
| 电梯功能 | 可以开启或关闭 | 可以开启或关闭 |

**运行模式和电梯模式是两个独立设置。** `relocation_enable` 选择建图或定位；`elevator.enable` 控制是否处理电梯运动。两种运行模式都可以乘坐电梯。

定位采用“手动初值 + IMU 预测 + 激光点面匹配”。初值需要接近地图中的实际位置和朝向；当前没有任意位置全局搜索功能。

## 2. 配置文件在哪里修改

```text
Elevator-LIO/
├── yaml/
│   ├── root_config.yaml          # 选择传感器、运行和日志配置
│   ├── sensors/livox.yaml        # MID-360 类型、外参和输入话题
│   ├── runtime/mapping.yaml      # 建图及其电梯参数
│   ├── runtime/relocation.yaml   # 定位及其电梯参数
│   └── logging/default.yaml      # 日志开关
└── PCD/                          # 默认地图读取与建图结果保存位置
```

`root_config.yaml` 的三行选择决定本次运行实际加载什么：

```yaml
sensor_config: "sensors/livox.yaml"
runtime_config: "runtime/mapping.yaml"
logging_config: "logging/default.yaml"
```

切换模式时只修改 `runtime_config`。每个字段只保留一个有效值，不要同时启用两行 `runtime_config`。

三个被选中的配置会合并读取，同一配置项不能在多个文件里重复定义。本文的 YAML 片段用于修改已有字段，不是直接追加到文件末尾。

ROS 2 运行时读取的是安装空间里的配置，默认位置为：

```text
<工作空间>/install/lio/share/lio/yaml/
```

使用 `--merge-install` 时通常为 `install/share/lio/yaml/`。可用下面命令确认当前终端找到的软件包：

```bash
ros2 pkg prefix --share lio
```

使用 `--symlink-install` 后，已链接的配置可直接在源码中修改；如果安装空间仍是配置副本，要同步配置或重新执行构建安装。可检查根配置实际指向哪里：

```bash
readlink -f "$(ros2 pkg prefix --share lio)/yaml/root_config.yaml"
```

配置在启动时读取，修改 YAML 后需要重启节点。`/initial_pose` 和电梯控制话题可在运行中使用。

## 3. 首次准备与编译

先完成 ROS、Livox-SDK2、`livox_ros_driver2` 及项目依赖安装，详细步骤见 [项目 README](../README.md)。若驱动在另一个工作空间，还要先 source 驱动工作空间。

```bash
cd ~/ws_elevator_lio_ros2
source /opt/ros/humble/setup.bash

# 驱动独立构建时，按实际路径执行：
# source ~/ws_livox_ros_driver2/install/setup.bash

colcon build --packages-select lio --symlink-install
source install/setup.bash
```

当前 CMake 默认编译回环后端，需要 GTSAM 等依赖。若只使用前端且不需要回环，可在构建时关闭：

```bash
colcon build --packages-select lio --symlink-install \
  --cmake-args -DLIO_ENABLE_LOOP_CLOSURE=OFF
```

此时还应将建图配置中的 `loop_closure.enabled` 设为 `false`，避免配置与构建能力不一致。以后恢复回环需要重新构建并设置 `-DLIO_ENABLE_LOOP_CLOSURE=ON`。

| 修改内容 | 是否需要重新编译 | 是否需要重启节点 |
| --- | --- | --- |
| `.cpp`、`.h`，例如首次加入手动重定位功能 | 需要 | 需要 |
| 切换 mapping/relocation 配置 | 不需要编译代码；确保安装空间配置同步 | 需要 |
| 修改话题、地图文件名、电梯参数等 YAML | 不需要编译代码；确保安装空间配置同步 | 需要 |
| 运行中发布 `/initial_pose` | 不需要 | 不需要 |
| 运行中发布电梯进入/退出消息 | 不需要 | 不需要 |

`--symlink-install` 方便同步配置，不会使 C++ 源码改动自动生效。

## 4. 启动 mapping 建图模式

### 4.1 选择建图配置

在 `yaml/root_config.yaml` 中设置：

```yaml
sensor_config: "sensors/livox.yaml"
runtime_config: "runtime/mapping.yaml"
logging_config: "logging/default.yaml"
```

在 `yaml/runtime/mapping.yaml` 中确认：

```yaml
relocation:
  relocation_enable: false

pcd_save:
  global_map_save_enable: true
  timestamp_prefix: true
  pcd_save_name: "scans.pcd"
```

`pcd_load_name` 在建图模式下不用于加载地图。电梯参数也在本文件中修改，配置方法见第 7 节。

### 4.2 启动传感器和算法

先按现有驱动配置启动 MID-360，确认雷达和 IMU 话题持续有数据。`start_ros2.launch.py` 只启动 `lio` 和可选 RViz，不会启动雷达驱动。

在算法终端执行：

```bash
cd ~/ws_elevator_lio_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lio start_ros2.launch.py
```

不需要 RViz 时：

```bash
ros2 launch lio start_ros2.launch.py use_rviz:=false
```

启动时保持设备静止，让程序收集 200 个 IMU 样本完成初始化，再开始移动。建图模式会用初始若干帧激光建立地图，随后持续发布位姿。

在另一个已 source 相同环境的终端查看：

```bash
ros2 topic echo /current_pose
```

### 4.3 查看建图结果

| 需要查看的内容 | RViz Fixed Frame | 话题及条件 |
| --- | --- | --- |
| 当前扫描和前端轨迹 | `world` | `/LIO/clouds_lidar`、`/lio_node/path_topic_name`，使用默认 ROS 2 节点名时 |
| 当前 ikd-tree 地图 | `world` | `/LIO/ikdtree`；需 `pub.ikdtree_pub_enable: true` |
| 回环优化地图和轨迹 | `map` | `/LIO/loop/map`、`/LIO/loop/path`；需回环后端运行，地图还需 `loop_closure.publish_map: true` |

查看回环结果也可使用专用 RViz 配置：

```bash
ros2 launch lio start_ros2.launch.py rviz_config:=LIO_loop_ros2.rviz
```

地图发布比较耗时，常规运行可保留 `pub.global_map_pub_enable: false` 和 `pub.ikdtree_pub_enable: false`；关闭显示不等于关闭地图保存。

### 4.4 停止并保存地图

在算法终端按 `Ctrl+C`，等待退出保存完成，再结束终端。`global_map_save_enable: true` 时，前端地图保存到**编译时源码目录**下的：

```text
Elevator-LIO/PCD/<时间戳>_scans.pcd
```

具体文件名以终端 `save pcd file:` 的输出为准。`timestamp_prefix: false` 时使用 `PCD/scans.pcd`，同名文件可能被后续运行覆盖。`every_n_frame_save: -1` 表示不单独保存逐帧 PCD，不会关闭退出时的全局地图保存。

回环后端运行时还会在日志会话目录保存 `loop_map.pcd` 和 `loop_trajectory.tum`，默认位于 `temp/<启动时间>/`。两类地图的坐标系不同：

| 地图文件 | 来源及坐标系 | 用于后续定位时的初值 |
| --- | --- | --- |
| `PCD/<时间戳>_scans.pcd` | 前端累计地图，原建图 `world` 系 | 按该前端地图坐标填写 |
| `temp/<启动时间>/loop_map.pcd` | 回环优化地图，原建图 `map` 系 | 按回环地图坐标填写 |

定位模式将所加载 PCD 的坐标直接作为自己的 `world`。如果选用回环地图，初始位姿数值必须来自该回环地图，并在定位消息中填写 `frame_id: world`；不要混用前端坐标和回环优化坐标。

## 5. 启动定位模式

### 5.1 准备参考地图

例如把地图放到源码目录的：

```text
Elevator-LIO/PCD/building_map.pcd
```

修改 `yaml/runtime/relocation.yaml`：

```yaml
relocation:
  relocation_enable: true
  wait_for_initial_pose: true
  initial_pose_topic_name: "/initial_pose"
  pcd_load_name: "building_map.pcd"
```

`pcd_load_name` 的相对路径以**编译时源码目录的 `PCD/`** 为起点，不是 `install` 目录。也可以填写部署机器上的绝对路径，例如：

```yaml
pcd_load_name: "/home/robot/maps/building_map.pcd"
```

地图单位为米；不存在、读取失败或为空时程序会报错退出。加载后地图先做 `0.1 m` 体素降采样。

### 5.2 选择定位配置并启动

修改 `yaml/root_config.yaml`：

```yaml
sensor_config: "sensors/livox.yaml"
runtime_config: "runtime/relocation.yaml"
logging_config: "logging/default.yaml"
```

确认传感器驱动已运行，在算法终端执行：

```bash
cd ~/ws_elevator_lio_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lio start_ros2.launch.py
```

启动日志应包含 `Relocation Map Model`、非零的地图加载点数，以及等待初始位姿的提示。保持设备静止完成 IMU 初始化。等待 `/initial_pose` 期间没有 `/current_pose` 输出属于正常行为。

### 5.3 发布初始位姿，开始定位

初始位姿表示 **参考地图坐标下的车体中心位置和姿态**，与 `/current_pose` 的语义一致，不是 LiDAR 或 IMU 中心。

例如车体位于 `(5, 2, 0)` 米，航向为零：

```bash
ros2 topic pub --once /initial_pose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: world}, pose: {pose: {position: {x: 5.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

平放且只有航向角 `yaw` 时，四元数填写：

```text
x = 0
y = 0
z = sin(yaw / 2)
w = cos(yaw / 2)
```

`yaw` 单位为弧度。例如朝向 `+90°`，填写 `z: 0.70710678, w: 0.70710678`。四元数不能全为零。示例省略协方差，程序会使用默认的位置标准差 `0.5 m` 和姿态标准差 `15°`。

`frame_id` 使用 `world`；空值也按 `world` 处理。其他坐标系不会自动进行 TF 转换。

收到请求后程序在 IMU 时刻设置初值，清除旧积分历史和显示轨迹，使用后续扫描匹配地图。日志 `initial pose applied` 表示初值已生效，不表示地图匹配已经成功；还需检查扫描与地图是否重合。

### 5.4 运行中重新定位和 RViz 操作

定位漂移或需要换一个初值时，保持设备静止，再发布一次上述消息，无需重启或重新编译。请求可在 IMU 初始化前发送，会暂存至初始化完成；实际应用时还需要后续 IMU 和激光数据。

手动重定位也会清零速度和电梯子状态，应在电梯停稳、设备静止时操作。

默认 ROS 2 RViz 配置已添加 **2D Pose Estimate**。Fixed Frame 设置为 `world`，在地图上点击并拖动，指定车体位置和朝向即可。也兼容发布相同消息类型的 `/initialpose`。

二维工具通常设置 `z=0`。其他楼层应通过话题明确填写参考地图中的车体高度。初始位姿消息的时间戳不用于历史位姿回放，初值在下一次处理的 IMU 时刻应用。

查看参考地图时，将 `pub.ikdtree_pub_enable` 设为 `true`，重启节点，在 RViz 中添加 `/LIO/ikdtree`，Fixed Frame 使用 `world`。定位模式没有回环后端，参考地图不会从 `/LIO/loop/map` 发布。

### 5.5 保留从原点直接定位的行为

若每次都从建图起点附近、接近建图初始朝向启动，可设置：

```yaml
relocation:
  wait_for_initial_pose: false
```

此时初始化后直接定位，运行中仍可发布 `/initial_pose` 重置。旧配置缺少 `wait_for_initial_pose` 时也使用这一行为。

## 6. 需要按设备修改的参数

### 6.1 雷达类型、输入话题与外参

在 `yaml/sensors/livox.yaml` 中设置，其他型号选择对应的传感器配置文件：

| 参数 | MID-360 当前值或含义 | 配置要求 |
| --- | --- | --- |
| `lidar_type` | `1`，Livox | 其他值：`2` Ouster、`3` Velodyne、`4` XT32 |
| `topic_sub.lidar_topic_name` | `/livox/lidar` | 必须与驱动点云话题一致；ROS 2 Livox 输入为 `livox_ros_driver2/msg/CustomMsg` |
| `topic_sub.imu_topic_name` | `/livox/imu` | 必须与驱动 IMU 话题一致，类型为 `sensor_msgs/msg/Imu` |
| `offset.imu_R_lidar`、`imu_t_lidar` | LiDAR 到 IMU 的旋转和平移 | 使用设备标定值，平移单位为米 |
| `offset.lidar_R_body`、`lidar_t_body` | 车体到 LiDAR 的旋转和平移 | 使用实际安装关系，决定车体输出和初始位姿换算 |
| `topic_sub.elevator_flag_topic_name` | `/LIO/set_elevator_flag` | 修改后，手动进入/退出命令也要改为同一话题 |

变换约定为：

```text
p_imu = imu_R_lidar × p_lidar + imu_t_lidar
p_lidar = lidar_R_body × p_body + lidar_t_body
world_T_body = world_T_imu × imu_T_lidar × lidar_T_body
```

仓库中的车体外参是示例，不能直接认定适合自己的机器人。建图和定位应使用一致的传感器标定及正确的车体定义。

启动前可检查：

```bash
ros2 topic info /livox/lidar
ros2 topic info /livox/imu
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

### 6.2 降采样、输出与估计器

以下参数修改当前选中的 `yaml/runtime/*.yaml`：

| 参数 | 当前配置 | 作用及修改时机 |
| --- | --- | --- |
| `downsample.blind` | `0.8 m` | 去掉近距离自遮挡点；按安装环境调整 |
| `downsample.point_filter_num` | 建图 `1`，定位 `2` | 按索引保留点的间隔；增大可降低输入点数 |
| `downsample.adaptive.enable` | `true` | 自动调整前端体素大小 |
| `downsample.adaptive.target_points` | `20000` 点/秒 | 不是每帧点数；计算压力大时可降低 |
| `downsample.adaptive.min_voxel` / `max_voxel` | `0.05 / 0.8 m` | 限制自适应体素范围；过大会损失细节 |
| `pub.clouds_lidar_pub_every_n` | `1` | 主点云每隔多少帧发布；增大只减少发布负担 |
| `pub.high_frequency_odom` | `false` | 开启后在激光帧之间同步发布 IMU 先验里程计及 `/current_pose` |
| `pub.lidar_frequency` | `10 Hz` | 开高频输出时必须填写实际雷达帧率 |
| `pub.ikdtree_pub_enable` | `false` | 调试参考地图时开启，发布整个树可能比较耗时 |
| `pub.global_map_pub_enable` | `false` | 建图累计地图显示开关；定位模式下该输出跳过 |
| `topic_pub.current_pose_topic_name` | `/current_pose` | 车体中心 `PoseStamped` 输出 |
| `estimator.lidar_update_mode` | `pose6` | 保留当前前端位姿更新配置即可 |
| `estimator.online_gravity_estimation_enable` | `false` | 默认固定初始化重力 |
| `wheel.wheel_enable` | `false` | 只有接入并适配轮速计消息后才开启 |

回环参数及其地图坐标说明见 [回环后端文档](loop_closure.md)，全部 YAML 字段说明见 [配置参考](../yaml/README.md)。

## 7. 设置电梯模式

电梯配置位于当前选中的运行文件：建图修改 `mapping.yaml`，定位修改 `relocation.yaml`。修改另一份未选中的文件不会影响本次运行。

### 7.1 根据控制方式设置三个开关

| 控制方式 | `elevator.enable` | `elevator.door_detector.enable` | `elevator.self_exit_detector` |
| --- | --- | --- | --- |
| 自动进入、自动退出 | `true` | `true` | `true` |
| 手动进入、自动退出 | `true` | `false` | `true` |
| 手动进入、手动退出 | `true` | `false` | `false` |
| 关闭电梯功能，作为普通 LIO 使用 | `false` | 无效 | 无效 |

例如“手动进入、自动退出”：

```yaml
elevator:
  enable: true
  self_exit_detector: true
  door_detector:
    enable: false
```

`elevator.enable: false` 时自动检测和手动电梯消息均被忽略。定位模式仍然需要初始位姿，关闭电梯功能不会跳过 `/initial_pose` 等待。

### 7.2 手动进入和退出

在设备已经进入轿厢、准备开始电梯运动时，发布一次进入消息：

```bash
ros2 topic pub --once /LIO/set_elevator_flag std_msgs/msg/Bool "{data: true}"
```

采用手动退出时，在电梯停稳后发布一次退出消息：

```bash
ros2 topic pub --once /LIO/set_elevator_flag std_msgs/msg/Bool "{data: false}"
```

不需要持续发布。开关在后续 IMU 处理时反映到内部状态，退出高度提交还需要激光处理，因此应保持传感器数据正常输入。

完全手动控制时要关闭自动进入、自动退出，否则自动检测仍可能改变模式。手动进入后，`self_exit_detector: true` 时仍会根据电梯运动和停稳状态自动退出。

完全手动时还建议把 `elevator.zupt.waiting.enable` 设为 `false`。当前等待阶段判断与自动退出状态机共用阶段状态；关闭 `self_exit_detector` 后阶段不会继续推进，保留等待 ZUPT 可能在运动期间因估计值满足阈值而再次触发。退出时是否执行 ZUPT 仍由 `elevator.zupt.enable` 决定。

### 7.3 自动进入参数

```yaml
elevator:
  enable: true
  self_exit_detector: true
  door_detector:
    enable: true
    dist_threshold: 3.0
    time_threshold: 2.0
    filter_percent: 0.06
    cooldown_time: 5.0
```

上例使用建图配置当前的 `filter_percent`；定位配置当前为 `0.03`。

| 参数 | 单位 | 含义及调整方向 |
| --- | --- | --- |
| `dist_threshold` | m | 去掉最远端部分点后，距离统计量小于此值时认为环境封闭；增大更容易触发，也更容易在走廊误触发 |
| `time_threshold` | s | 封闭条件需要持续满足的时长；增大可抑制短暂误判，但进入检测更慢 |
| `filter_percent` | 比例 | 忽略最远端点的比例，`0.06` 表示最远约 6%；比例过大会忽略开门时看到的远处结构 |
| `cooldown_time` | s | 两次自动触发之间的最短间隔 |

检测依赖激光距离分布，没有识别电梯语义。狭窄走廊和小房间也可能满足条件，遇到误触发可关闭自动进入，改用手动消息。自动退出根据 IMU 估计的运动阶段及停稳状态判断，`self_exit_detector` 是其开关。

### 7.4 ZUPT、到站高度修正与安装先验

下表是仓库当前两套配置，数值不同不表示必须统一：

| 参数 | mapping | relocation | 作用 |
| --- | --- | --- | --- |
| `elevator.enable` | `true` | `true` | 电梯功能总开关 |
| `elevator.zupt.enable` | `true` | `false` | 电梯相关零速、零加速度伪量测更新 |
| `elevator.zupt.waiting.enable` | `true` | `true` | 等待阶段周期性 ZUPT；还受上一行总开关控制 |
| `elevator.exit_icp_z.enable` | `false` | `true` | 退出电梯时，对已有地图只求解高度修正 |
| `elevator.strong_prior_enable` | `false` | `false` | 电梯内相对竖直运动约束先验 |
| `elevator.rebuild_map_on_exit.enable` | `false` | `false` | 退出后清空并重建局部地图，仅建图模式生效 |

等待阶段 ZUPT 的当前参数为：

```yaml
elevator:
  zupt:
    enable: true
    waiting:
      enable: true
      period_s: 3.0
      vz_abs_thresh: 0.1
      az_abs_thresh: 0.1
```

它在进入电梯后的等待阶段、估计的竖直速度和加速度满足阈值时触发；自动退出状态机开启且检测到运动后，会离开等待阶段。完全手动时按第 7.2 节关闭等待 ZUPT。定位预设中 `zupt.enable: false`，即使 `waiting.enable: true` 也不会执行该更新。

到站高度 ICP 的关键参数为：

```yaml
elevator:
  exit_icp_z:
    enable: true
    max_correction_m: 0.3
    max_neighbor_distance_m: 0.5
    min_abs_normal_z: 0.7
```

该功能在退出处理阶段执行，只有启用 ZUPT 时才会先做退出 ZUPT。ICP 仅改变高度，不改水平位置和姿态；累计修正绝对值必须严格小于 `0.3 m` 才接受，且需有足够的水平面对应。它不能代替初始位姿，也不能将错误楼层的大幅高度误差直接修正回来。

当 LiDAR 水平安装、竖直几何约束不足，并且设备相对轿厢基本没有竖直运动时，可按场景启用：

```yaml
elevator:
  strong_prior_enable: true
  strong_prior_acc_var: 0.001
```

该先验将 IMU 竖直加速度更强地解释为电梯加速度。手持设备被举起、上下晃动时不满足假设，应保留 `false`。

跨楼层连续建图通常保留 `rebuild_map_on_exit.enable: false`。开启重建会清空旧局部地图，不适合需要保留原有地图约束的常规建图流程；定位模式不执行该重建。

### 7.5 查看电梯状态和日志

```bash
ros2 topic echo /LIO/in_elevator
ros2 topic echo /LIO/elevator_state
```

`/LIO/in_elevator` 为布尔模式标志；`/LIO/elevator_state` 还包含本段电梯位移 `displacement`、速度 `velocity`、加速度 `acceleration`，分别以米、米/秒、米/秒平方计。位移是本段电梯子状态，不是楼层编号或车体绝对高度；退出后该子状态会清零，累计高度合入车体位姿。

默认 RViz 带电梯状态面板。需要查看自动检测、状态转移和高度 ICP 的详细控制台日志时，在 `yaml/logging/default.yaml` 中修改：

```yaml
log:
  console:
    elevator: true
```

日志总开关和控制台开关也必须开启；当前预设已开启这两个总开关。

## 8. 常用输出与坐标系

| 默认话题 | ROS 2 类型 | 内容及坐标系 |
| --- | --- | --- |
| `/current_pose` | `geometry_msgs/msg/PoseStamped` | 车体中心当前位姿，`world` 系，包含电梯高度补偿 |
| `/LIO/odom_vehicle` | `nav_msgs/msg/Odometry` | 车体里程计，父坐标系 `world` |
| `/LIO/odom_imu` | `nav_msgs/msg/Odometry` | IMU 中心里程计，父坐标系 `world` |
| `/LIO/clouds_lidar` | `sensor_msgs/msg/PointCloud2` | 当前主点云，已转换到 `world` 系 |
| `/LIO/ikdtree` | `sensor_msgs/msg/PointCloud2` | 开启发布时输出参考地图或局部地图，`world` 系 |
| `/LIO/loop/odom` | `nav_msgs/msg/Odometry` | 回环后端运行时的校正后 IMU 位姿，`map` 系 |
| `/LIO/in_elevator` | `std_msgs/msg/Bool` | 当前是否处于电梯模式 |
| `/LIO/elevator_state` | `lio/msg/ElevatorState` | 电梯子状态 |

建图时 `/current_pose` 保持前端 `world` 系，不自动变成回环校正后的 `map` 系车体位姿。固定地图定位时 `/current_pose` 的 `world` 就是所加载参考地图的坐标系。

## 9. 常见问题

| 现象 | 优先检查 |
| --- | --- |
| 修改配置后行为没有变化 | 修改的是否是根配置选中的文件；安装空间是否仍为旧副本；是否已重启节点 |
| 定位启动后一直没有 `/current_pose` | 默认在等待 `/initial_pose`；还需检查 IMU 是否完成初始化、激光与 IMU 是否持续输入 |
| 初始位姿发布后仍无效 | 消息必须为 `PoseWithCovarianceStamped`；`frame_id` 使用 `world`；四元数不能全零；话题名是否改过 |
| 日志显示已应用初值，但位置不对 | 检查初值接近程度、车体外参、地图单位、地图坐标来源，以及楼层高度 |
| 地图加载失败 | 检查部署机器上的文件路径；相对路径指向源码 `PCD/`，也可改用绝对路径 |
| 定位时看不到参考地图 | 开启 `pub.ikdtree_pub_enable`，重启后显示 `/LIO/ikdtree`；不要等 `/LIO/loop/map` |
| 建图结束没有地图文件 | 检查 `global_map_save_enable`、是否正常 `Ctrl+C` 并等待保存，以及终端输出的实际保存路径 |
| 在走廊误进电梯模式 | 关闭 `door_detector.enable` 改用手动进入，或结合距离分布调整自动进入参数 |
| 发布电梯 `true` 没有效果 | 检查 `elevator.enable`、控制话题和 IMU 输入；定位模式是否还在等待初始位姿 |
| 退出电梯后很快又进入 | 自动进入检测可能再次满足条件；完全手动时关闭自动进入，自动模式检查冷却时间及场景 |
| 到站 ICP 拒绝修正 | 可能缺少水平面、近邻不足或高度修正超过限制；查看详细电梯日志，先确认初始楼层及地图匹配正确 |

## 10. ROS 1 对应命令

配置文件及参数含义相同。首次包含新功能时需要编译并 source 工作空间：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch lio start.launch
```

根据根配置选择建图或定位；定位手动初值命令为：

```bash
rostopic pub -1 /initial_pose geometry_msgs/PoseWithCovarianceStamped \
  "{header: {frame_id: world}, pose: {pose: {position: {x: 5.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

手动进入、退出电梯：

```bash
rostopic pub -1 /LIO/set_elevator_flag std_msgs/Bool "{data: true}"
rostopic pub -1 /LIO/set_elevator_flag std_msgs/Bool "{data: false}"
```

查看输出：

```bash
rostopic echo /current_pose
rostopic echo /LIO/in_elevator
rostopic echo /LIO/elevator_state
```

需要使用自定义根配置时，`config_path` 填写相对于软件包 `yaml/` 目录的根配置文件名。不要把 `runtime/relocation.yaml` 直接当成根配置传入。

```bash
# ROS 2
ros2 launch lio start_ros2.launch.py config_path:=root_config.yaml

# ROS 1
roslaunch lio start.launch config_path:=root_config.yaml
```
