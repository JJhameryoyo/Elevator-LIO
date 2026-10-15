# MID360 回环后端

移植来源：本工作区 `mid360-campus-slam-main/src/sc_liorf_localization` 的
`mapOptmization.cpp` 和 `Scancontext.cpp`。回环链路为：关键帧 → 距离候选 /
Scan Context → 历史子图 ICP → Cauchy 鲁棒回环约束 → GTSAM iSAM2。

所需源码和头文件已直接复制到本项目的 `src/loop/` 和 `include/loop/`，
由 `lio` 节点直接编译。这里的来源名称仅用于说明代码出处；构建和运行均不
引用另一个项目的目录、ROS 包或编译产物。可以单独复制 Elevator-LIO 工程使用。

## 构建与启用

在原 Elevator-LIO 的 ROS 1/ROS 2 构建环境中安装 GTSAM（例如已有 MID360
项目使用的 GTSAM 4.x）及 PCL registration/kdtree 开发库，然后正常构建 `lio`。
CMake 默认 `LIO_ENABLE_LOOP_CLOSURE=ON`，`yaml/runtime/mapping.yaml` 默认开启。
无需新增 launch 节点。固定地图重定位模式自动禁用回环；旧配置缺少
`loop_closure` 段时默认关闭。可设 `enabled: false` 停用运行，或构建时传入
`-DLIO_ENABLE_LOOP_CLOSURE=OFF` 完全移除后端及 GTSAM 链接。

ROS 2 示例（先 source ROS 环境）：

```bash
colcon build --packages-select lio --cmake-args -DLIO_ENABLE_LOOP_CLOSURE=ON
source install/setup.bash
ros2 launch lio start_ros2.launch.py
```

## 坐标系和输出

`world` 是连续的前端里程计系，`map` 是回环优化系。前端 IMU 状态、
电梯竖直子状态及局部 ikd-tree 继续在 `world` 中工作。后端使用去畸变的
LiDAR 点云和包含 IMU→LiDAR 外参的位姿创建关键帧。
每次图更新计算 `map_T_world = map_T_lidar * inverse(world_T_lidar)`。
`map → world → IMU → lidar/body` 构成 TF 树。

| 输出 | 内容 |
| --- | --- |
| `/LIO/loop/odom` | 校正后的 IMU 位姿，frame=`map`，child=`IMU`；不是车体位姿 |
| `/LIO/loop/path` | 全部优化后的 LiDAR 关键帧轨迹，1 Hz 发布，可回溯修正历史位姿 |
| `/LIO/loop/map` | 用优化位姿重新拼接的关键帧地图；`publish_map: true` 时发布 |
| TF `map → world` | 最新图优化校正，随里程计时间戳广播 |
| `loop_trajectory.tum` | 校正后的 LiDAR 关键帧轨迹 |
| `loop_map.pcd` | 校正后的关键帧拼接地图，保留帧间叠加点；退出时自动保存 |
| `loop_constraints.csv` | 已接受回环的 current/previous 关键帧编号 |

文件保存到已有日志会话目录，启动时会打印。原 `/LIO/odom_imu`、
`path_topic_name`、`/LIO/global_map` 和原 PCD 输出仍表示前端 world 系结果。
使用 RViz 查看全局回环结果时，将 Fixed Frame 设为 `map`，添加上述 loop Path
和 PointCloud2。历史前端点云不能仅靠最新 TF 消除过去的累计误差，应使用
`loop_map.pcd` 或 `/LIO/loop/map`。

也可以直接启动专用的回环地图 RViz 配置，固定坐标系为 `map`，Loop Map
使用绿色 3 像素点与 Transient Local / Reliable QoS：

该专用视图默认关闭 LiDAR 当前帧和前端 Path 显示项，以便辨认累计的全局地图；
需要对比时可手动打开。原 `LIO_ros2.rviz` 仍保留前端显示。

```bash
ros2 launch lio start_ros2.launch.py rviz_config:=LIO_loop_ros2.rviz
```

地图显示不需要降低 ICP 回环验证阈值。先用
`ros2 topic echo /LIO/loop/map --once --field width --qos-durability transient_local`
检查是否有地图点数，再检查 RViz 的 Loop Map / Status。不要在 RViz 中用
`Map` 显示类型订阅这个话题；该显示类型接收 OccupancyGrid，而本项目发布的是 PointCloud2。

## 电梯与参数

电梯模式内的点云不进入后端，以免移动轿厢形成虚假回环。历史候选先经过
时间间隔、关键帧间隔及高度差筛选，然后才进入 Scan Context KNN；历史子图
也使用这些筛选。`max_height_difference` 应小于实际层高，并允许同层里程计
竖直漂移。相似楼层仍可能误匹配，须用包含乘梯与同层重访的实际数据调参。

ICP 输出是 `previous_T_current`，图中 `BetweenFactor(current, previous)`
使用其逆变换。Scan Context 的 yaw 初始化使用描述子移位的反向旋转。
ICP 同时检查收敛、平均平方误差、对应点覆盖率及校正后的高度差。

输入队列最多保留 8 帧，后端繁忙时丢弃最老的待处理帧；接受回环时日志报告
回环数及丢帧累计数。关键帧/点云随建图时间增长，长时间任务应增大关键帧间隔。
建图配置的 `publish_map` 默认开启，ROS 2 RViz 的 Loop Map/Loop Path 显示项
订阅 `/LIO/loop/map` 和 `/LIO/loop/path`。无需等到首次回环，积累关键帧后即可
看到地图；性能优先时可将 `publish_map` 关闭。退出时仍会排空队列、做最后一次
检测并保存地图。后端异常会停止后端并报告原因，前端继续运行。

`min_source_points` 仅限制 ICP 回环验证，并在 ICP 点云降采样后检查。至少包含
3 个有限坐标点的帧可创建关键帧并生成地图，避免稀疏点云没有地图。
启动日志打印实际读取的运行配置路径及地图发布开关；每 5 秒的 `loop map status`
打印输入帧数、当前帧点数、电梯状态、关键帧数、地图点数和丢帧数。

## 地图密度

已编译的初版后端支持 `voxel_size`，它同时控制关键帧、ICP 和地图体素。
不重新编译也能调密度：将运行配置中的 `voxel_size` 从 0.3 改为 0.1 m，
设置 `point_filter_num: 1`，并缩短关键帧间隔后重启 LIO。当前建图 YAML 使用
0.1 m，以兼容已有程序。还需要更密时可以改为 0.05 m，但初版 ICP 的点数也会增加。

更新后的源码支持独立的 `map_voxel_size`；已有程序尚未编译这一更新时，
单独修改 `map_voxel_size` 不会生效。新版可以设 `map_voxel_size: 0.05`、
`voxel_size: 0.3`，使地图更密而 ICP / Scan Context 使用较粗的点云。
建图配置的输入抽点比例改为 1；关键帧平移、旋转和最大时间间隔分别为
0.3 m、0.1 rad、0.5 s。前端估计的自适应降采样目标仍是每秒 20000 点，它
使用单独的 `downsampled_cloud_`，不会替换后端收到的去畸变输入点云。

### 保持在线计算负载的密集 PCD 导出

当前源码参考 MID360 的 `cloudGlobal.pcd` 保存方式：每个关键帧仍按
`map_voxel_size` 降采样，退出时使用最终优化位姿拼接这些关键帧，
保存 `loop_map.pcd` 时不再对拼接结果做全局体素过滤。因此，多帧在同一
区域观察到的采样点会保留下来，点数可以明显高于在线显示地图。

运行期间的 `/LIO/loop/map` 继续做全局体素过滤；前端每秒目标点数、
ikd-tree、关键帧频率、ICP 和 Scan Context 参数均不变。密集导出只在
停止后端时组装，在线地图缓存即使已经更新到最后一帧，也会被密集结果替换。
这保持了在线算法的处理规模，但不承诺总资源消耗完全不变：退出时可能
需要重新拼接，最终地图占用的内存、PCD 大小及磁盘写入量会增加。

普通 `PCD/*scans.pcd` 也取消了退出合并时固定的 0.1 m 全局过滤，仍然
使用前端有效匹配点；它无法补回前端已经丢弃的点。需要更完整且经过回环
校正的地图时，使用日志会话目录中的 `loop_map.pcd`。`ikdtree_*.pcd`
仍表示前端匹配用的局部地图。增加的是多帧采样密度，不代表定位精度提高。
这项导出行为需要重新编译，修改 YAML 或打开旧 PCD 不会产生新增点。

更密的关键帧与地图会增加内存和地图拼接开销。修改实际运行的 YAML 后只需
重启节点。可直接编辑 `$(ros2 pkg prefix lio)/share/lio/yaml/runtime/mapping.yaml`，
避免普通安装模式下源码 YAML 尚未复制到安装目录。随后重新建图或回放原始
rosbag，生成新的 PCD；已有稀疏 PCD 无法补回丢失的细节。

## 回归验证

提供无需 ROS 的 C++ 后端测试，覆盖空历史、少于 3 个候选、描述子 yaw、
回环约束方向、map/world 校正、跨楼层与时间筛选、电梯帧抑制、关闭后端和保存。
密集导出测试还覆盖在线地图去重、退出时替换已有缓存、重复停止、已发布快照
不被修改，以及重读 PCD 后逐帧采样点数与坐标的保留。
在具备 Eigen/PCL/yaml-cpp/GTSAM 的 Linux 环境中运行：

```bash
cmake -S tests/loop_closure -B /tmp/elevator-loop-tests
cmake --build /tmp/elevator-loop-tests -j2
ctest --test-dir /tmp/elevator-loop-tests --output-on-failure
```

完整验收还需 ROS 1/ROS 2 构建及 rosbag 回放：确认重访后出现 accepted loop
日志、优化轨迹/地图一致、TF 树无冲突，跨楼层及电梯内不形成回环。

本次移植所在 Windows 工作区未提供 ROS、CMake、PCL/GTSAM 编译环境，尚未
运行这些 C++ 测试或 rosbag 回放。已通过 package.xml 解析、CMake 源文件路径、
本地头文件依赖以及全部 20 个 YAML 参数与 C++ 读取字段的一致性检查。
