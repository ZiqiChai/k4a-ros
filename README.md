# k4a-ros

[![CI](https://github.com/ZiqiChai/k4a-ros/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/ZiqiChai/k4a-ros/actions/workflows/ci.yml)

Microsoft Azure Kinect DK 的独立 ROS 2 驱动，基于 Azure Kinect Sensor SDK 1.4.2，面向
Ubuntu 22.04 amd64 与 ROS 2 Humble。

> Microsoft 已归档 Azure Kinect Sensor SDK，v1.4.2 是最后一个正式版本。本项目固定上游
> tag、commit、下载 URL 与 SHA-256，并提供二进制部署、源码构建、离线缓存和 CI 验证。

## 功能

- RGB、Depth、IR 与对应 `CameraInfo`
- Depth→Color、Color→Depth 标定对齐图像
- Depth optical frame 下的有序 XYZRGB 点云（米）
- IMU 与温度
- 设备硬件时间戳映射 ROS clock，同一 capture 消息使用相同时间戳
- 工厂标定静态 TF、`SensorDataQoS`、diagnostics
- standalone node、ROS 2 component、多相机序号选择

## 支持基线

| 项目 | 支持范围 |
|---|---|
| OS / ROS | Ubuntu 22.04 / ROS 2 Humble |
| 架构 | amd64 |
| C++ | 默认 C++17；兼容 C++20；Humble 官方最低要求为 C++17 |
| 编译器 | GCC、Clang；CI 同时验证 C++17，并额外验证 GCC/C++20 |
| CMake | 项目声明最低 3.10；发布基线使用 Ubuntu 22.04 自带版本 |
| SDK | Azure Kinect Sensor SDK 1.4.2 |

本机真机基线已验证 RGB 1280×720、Depth/IR 640×576、30 FPS、aligned images、
640×576 organized point cloud、IMU、TF 与 diagnostics。

## 分支

| 分支 | 定位 |
|---|---|
| `main` | 默认开发、文档和发布入口；当前基线为 Ubuntu 22.04 / ROS 2 Humble |
| `ubuntu-22.04-ros2-humble` | Ubuntu 22.04 / ROS 2 Humble 稳定维护线 |
| `ubuntu-18.04-ros1-melodic` | Ubuntu 18.04 / ROS Melodic 历史归档；使用该分支自身文档 |

## 一键安装

先按 [ROS 2 Humble 官方说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
安装 ROS 2，然后：

```bash
git clone https://github.com/ZiqiChai/k4a-ros.git
cd k4a-ros
./scripts/bootstrap.sh
```

脚本会安装系统/ROS 依赖、提示确认 Microsoft SDK runtime 许可、部署 SDK、安装 udev 规则并
编译 ROS 包。完成后：

```bash
source /opt/ros/humble/setup.bash
source scripts/env.sh
source install/setup.bash
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

已配置 udev 时可使用 `./scripts/bootstrap.sh --skip-udev`。完整选项与手动步骤见安装文档。

真机一键检查：

```bash
./scripts/run_hardware_test.sh
```

## 文档

- [完整安装、源码编译与离线安装](docs/INSTALL.md)
- [运行、Topics、参数与多相机](docs/USAGE.md)
- [依赖锁定、缓存与镜像](docs/DEPENDENCIES.md)
- [源码、预编译包与第三方二进制分发边界](docs/DISTRIBUTION.md)
- [故障排查](docs/TROUBLESHOOTING.md)
- [发布检查清单](docs/RELEASE_CHECKLIST.md)
- [版本记录](CHANGELOG.md)

## 许可证

本项目使用 [BSD-3-Clause](LICENSE)。Azure Kinect SDK 源码、官方 runtime、Depth Engine 及其
第三方组件保留各自许可。闭源 SDK 二进制不进入本仓库或本项目预编译包，由安装脚本在用户明确
接受 Microsoft 许可后从固定来源下载和校验。
