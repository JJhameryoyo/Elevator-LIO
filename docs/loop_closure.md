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
| `loop_map.pcd` | 校正后的关键帧地图；退出时自动保存 |
| `loop_constraints.csv` | 已接受回环的 current/previous 关键帧编号 |

文件保存到已有日志会话目录，启动时会打印。原 `/LIO/odom_imu`、
`path_topic_name`、`/LIO/global_map` 和原 PCD 输出仍表示前端 world 系结果。
使用 RViz 查看全局回环结果时，将 Fixed Frame 设为 `map`，添加上述 loop Path
和 PointCloud2。历史前端点云不能仅靠最新 TF 消除过去的累计误差，应使用
`loop_map.pcd` 或 `/LIO/loop/map`。

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
`publish_map` 默认关闭以降低地图重拼接开销；退出时仍会排空队列、做最后一次
检测并保存地图。后端异常会停止后端并报告原因，前端继续运行。

## 回归验证

提供无需 ROS 的 C++ 后端测试，覆盖空历史、少于 3 个候选、描述子 yaw、
回环约束方向、map/world 校正、跨楼层与时间筛选、电梯帧抑制、关闭后端和保存。
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
本地头文件依赖以及全部 19 个 YAML 参数与 C++ 读取字段的一致性检查。
