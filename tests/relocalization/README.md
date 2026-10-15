# 手动重定位验证

在安装 Eigen3 的开发环境中，数值测试不依赖 ROS：

```bash
cmake -S tests/relocalization -B build/initial_pose_tests
cmake --build build/initial_pose_tests
ctest --test-dir build/initial_pose_tests --output-on-failure
```

测试覆盖默认协方差、非单位四元数归一化、带旋转和平移外参的车体/IMU位姿往返、协方差雅可比的有限差分验证，以及无效位置、四元数和协方差的拒绝。

ROS 1/ROS 2 运行验证需使用真实传感器或仿真数据及对应参考地图：

1. 选择 `runtime/relocation.yaml`，填写地图，保持 `wait_for_initial_pose: true`。启动传感器及 `lio`，确认 IMU 初始化后仍在等待，尚无 `/current_pose` 输出。
2. 发布错误坐标系（例如 `odom`）、全零四元数或负协方差的 `/initial_pose`，确认日志拒绝请求，节点继续等待。
3. 按 `yaml/README.md` 的命令，发布接近实际位置的 `world` 系车体初始位姿，确认后续扫描匹配参考地图，`/current_pose` 与实际车体位置一致。用非零车体外参验证，避免把 IMU 中心误当成车体中心。
4. 在设备静止时重复发布初始位姿，确认轨迹清空、时间戳继续递增，后续 IMU 预测和扫描匹配不会跳回旧位置。
5. 重启后在 IMU 初始化完成前发送请求，确认请求保留至初始化结束，并只执行最后一个有效初值。
6. 在 RViz 中使用 Fixed Frame `world`，通过 `2D Pose Estimate` 设置初值；另验证 `/initialpose` 输入。其他楼层使用话题显式指定 `z`。
7. 设置 `wait_for_initial_pose: false`，确认从原点直接定位仍可运行，手动消息仍可重置。切换 `mapping.yaml`，确认正常建图，且不订阅重定位输入。

`initial pose applied` 日志只确认初值已设置，不代表地图匹配已成功。需要检查当前扫描与参考地图的重合程度。
