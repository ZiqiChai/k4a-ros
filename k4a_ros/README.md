# k4a_ros

Microsoft Azure Kinect DK 的 ROS 2 驱动包（Azure Kinect Sensor SDK 1.4.2，Ubuntu 22.04 amd64 /
ROS 2 Humble）。

本目录是一个自包含的 ament_cmake ROS 2 包：可直接拷贝（或符号链接）到任意 colcon 工作空间的
`src/` 目录下编译运行。

## 功能

- RGB、Depth、IR 与对应 `CameraInfo`
- Depth→Color、Color→Depth 标定对齐图像
- Depth optical frame 下的有序 XYZRGB 点云（米）
- IMU 与温度
- 设备硬件时间戳映射 ROS clock，同一 capture 消息使用相同时间戳
- 工厂标定静态 TF、`SensorDataQoS`、diagnostics
- standalone node、ROS 2 component、多相机序号选择

## 前置条件

1. 已安装 ROS 2 Humble（`source /opt/ros/humble/setup.bash`）。
2. 已部署 Azure Kinect Sensor SDK 1.4.2，并在当前 shell 设置好 SDK 环境
   （`CMAKE_PREFIX_PATH` 指向 SDK 前缀、运行前设置 `LD_LIBRARY_PATH`）。
   使用本仓库根目录时可直接 `source scripts/env.sh`（先运行 `scripts/setup_sdk.sh` 部署 SDK）；
   若用 `scripts/setup_sdk.sh --method system` 把 SDK 装进了系统，则本步骤可省略。
   Azure Kinect SDK 不在 rosdep 数据库中，`rosdep install` 无法自动安装。
3. udev 规则已安装（参见仓库 `docs/INSTALL.md`）。

## 编译与运行

```bash
mkdir -p ~/ros2_ws/src
cp -r k4a_ros ~/ros2_ws/src/          # 或使用 git clone 后拷贝/符号链接
source /opt/ros/humble/setup.bash
source /path/to/k4a-ros/scripts/env.sh  # SDK 环境
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

## 文档

完整安装、运行、依赖与故障排查文档位于仓库根目录 `docs/`：

- 安装：`docs/INSTALL.md`
- 运行、Topics、参数与多相机：`docs/USAGE.md`
- 依赖锁定、缓存与镜像：`docs/DEPENDENCIES.md`
- 分发边界：`docs/DISTRIBUTION.md`
- 故障排查：`docs/TROUBLESHOOTING.md`

## 许可证

本包使用 [BSD-3-Clause](LICENSE)。Azure Kinect SDK 及其 Depth Engine 保留各自许可，闭源二进制
不随本包分发。
