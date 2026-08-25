# k4a-ros

[![CI](https://github.com/ZiqiChai/k4a-ros/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/ZiqiChai/k4a-ros/actions/workflows/ci.yml)

Microsoft Azure Kinect DK 的 ROS 2 驱动，基于 Azure Kinect Sensor SDK 1.4.2，面向
Ubuntu 22.04 amd64 与 ROS 2 Humble。

> Azure Kinect Sensor SDK 已被 Microsoft 归档，v1.4.2 是最后一个正式版本。本项目固定上游
> tag、commit、下载 URL 与 SHA-256，由安装脚本在你接受 Microsoft 许可后自动下载部署。

## 功能

- RGB、Depth、IR 图像与 `CameraInfo`（工厂标定）
- Depth→Color、Color→Depth 对齐图像
- 有序 XYZRGB 点云（米；启用 RGB 时使用 Color optical frame，避免遮挡边界错误着色）
- IMU（加速度已旋转至陀螺仪坐标系）与温度
- 硬件时间戳映射 ROS clock，同一 capture 的消息时间戳一致
- 静态 TF、`SensorDataQoS`、diagnostics、多相机序号/序列号选择
- standalone node 与 ROS 2 component 两种运行方式

## 快速开始（三步）

前提：已安装 [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)，
相机通过 USB 3.x 直连。

```bash
git clone https://github.com/ZiqiChai/k4a-ros.git
cd k4a-ros
./scripts/bootstrap.sh          # 安装依赖、部署 SDK、安装 udev、编译 ROS 包
```

运行（含 SDK 许可交互提示；CI 可加 `--accept-sdk-license`）：

```bash
source /opt/ros/humble/setup.bash
source scripts/env.sh
source install/setup.bash
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

看到点云和图像即成功。已配置过 udev 可加 `--skip-udev`；出问题时查
[故障排查](docs/TROUBLESHOOTING.md)。

## 仓库结构

```
k4a-ros/
├── k4a_ros/          # 自包含 ROS 2 包，可直接拷入 colcon 工作空间 src/ 编译运行
├── docs/             # 安装、运行、依赖、分发与故障排查文档
├── scripts/          # 一键安装、SDK 部署、udev、编译、打包与硬件测试脚本
├── dependencies/     # Azure Kinect SDK 版本锁定、SHA-256 与源码补丁
└── .github/          # CI 与发布工作流
```

## 放入自己的工作空间

`k4a_ros/` 是标准 ament_cmake 包。配置好 ROS 2 与 SDK 环境后，可直接拷贝到任意工作空间：

```bash
mkdir -p ~/ros2_ws/src && cp -r k4a_ros ~/ros2_ws/src/
source /opt/ros/humble/setup.bash
source /path/to/k4a-ros/scripts/env.sh   # SDK 不在 rosdep 中，编译和运行前都需要它
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

想让工作流完全标准化（编译和运行都不需要 `env.sh`），把 SDK 装进系统即可：

```bash
./scripts/setup_sdk.sh --method system
```

## 支持基线

| 项目 | 支持范围 |
|---|---|
| OS / ROS | Ubuntu 22.04 / ROS 2 Humble |
| 架构 | amd64（Depth Engine 仅发布 amd64） |
| C++ / CMake | C++17（默认）；CMake >= 3.16 |
| SDK | Azure Kinect Sensor SDK 1.4.2（固定 SHA-256） |

| 分支 | 定位 |
|---|---|
| `main` | 默认开发、文档和发布入口 |
| `ubuntu-22.04-ros2-humble` | Ubuntu 22.04 / ROS 2 Humble 稳定维护线 |
| `ubuntu-18.04-ros1-melodic` | ROS Melodic 历史归档 |

## 文档

- [安装与部署](docs/INSTALL.md)：从全新系统到真机出流
- [使用说明](docs/USAGE.md)：Topics、参数、多相机
- [依赖锁定与离线安装](docs/DEPENDENCIES.md)
- [分发与预编译产物](docs/DISTRIBUTION.md)
- [故障排查](docs/TROUBLESHOOTING.md)
- [彩色点云的坐标系与遮挡处理](docs/COLOR_POINT_CLOUD.md)
- [发布检查清单](docs/RELEASE_CHECKLIST.md)
- [版本记录](CHANGELOG.md)

## 许可证

本项目使用 [BSD-3-Clause](LICENSE)。Azure Kinect SDK 源码、官方 runtime、Depth Engine 及其
第三方组件保留各自许可。闭源 SDK 二进制不进入本仓库，由安装脚本在用户明确接受 Microsoft
许可后从固定来源下载和校验。
