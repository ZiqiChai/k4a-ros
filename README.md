# k4a-ros

[![CI](https://github.com/ZiqiChai/k4a-ros/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/ZiqiChai/k4a-ros/actions/workflows/ci.yml)

Microsoft Azure Kinect DK 的 ROS 2 驱动，面向 Ubuntu 22.04 amd64 与 ROS 2 Humble，使用系统
安装的 Azure Kinect Sensor SDK 1.4.2。

## 功能

- RGB、Depth、IR 图像与工厂标定 `CameraInfo`
- Depth→Color、Color→Depth 对齐图像
- Color optical frame 下的有序 XYZRGB 点云
- IMU、温度、静态 TF、硬件时间戳与 diagnostics
- `SensorDataQoS`、设备索引/序号选择、standalone node 与 ROS 2 component

## 标准 ROS workspace 流程

前提：Ubuntu 22.04 amd64、ROS 2 Humble、`colcon` 和 `rosdep` 已安装。

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/ZiqiChai/k4a-ros.git

cd k4a-ros
./scripts/setup_sdk.sh

cd ~/ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

`setup_sdk.sh` 是唯一的 SDK 安装入口：它校验并通过 apt 系统安装 `libk4a1.4`、
`libk4a1.4-dev` 和 udev 规则。之后编译、运行只需要标准 ROS 环境，不需要项目专用环境脚本。

官方 `k4a-tools` 是 Ubuntu 18.04 包，依赖 Jammy 不提供的 `libsoundio1`，因此不安装。
`k4aviewer`、`k4arecorder` 不是 ROS 驱动依赖。

## 仓库结构

```text
k4a-ros/
├── k4a_ros/          # 标准 ament_cmake ROS 2 包
├── scripts/          # 系统 SDK 安装、udev 规则和真机检查
├── dependencies/     # SDK runtime/dev URL 与 SHA-256 锁定
├── tools/            # SDK 源码重建与恢复工具
├── docs/             # 安装、使用、故障排查与技术说明
└── .github/          # 标准 ros_ws 构建测试
```

## 文档

- [安装与工作空间编译](docs/INSTALL.md)
- [Topics、参数与运行](docs/USAGE.md)
- [系统 SDK 依赖锁定](docs/DEPENDENCIES.md)
- [故障排查](docs/TROUBLESHOOTING.md)

## 许可证

本项目使用 [BSD-3-Clause](LICENSE)。Azure Kinect SDK、Depth Engine 及其第三方组件保留各自
许可；SDK 二进制由安装脚本在用户接受 Microsoft 许可后从固定官方地址下载并校验。
