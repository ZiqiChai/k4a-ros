# 安装与部署

本文给出从全新 Ubuntu 22.04 amd64 到真机出流的完整流程。所有命令默认在 Bash 中执行，
仓库目录固定使用 Git clone 后的名称 `k4a-ros`，不依赖维护者本机绝对路径。

## 1. 前置条件

- Ubuntu 22.04 amd64
- Azure Kinect DK 与数据规格 USB 3.x 线缆，建议直连主板端口
- ROS 2 Humble
- 可使用 `sudo` 安装系统包和 udev 规则

ROS 2 只在此简述：请按
[ROS 2 Humble Ubuntu deb 官方指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
安装 `ros-humble-desktop` 与开发工具。安装后确认：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
printenv ROS_DISTRO
```

最后一条应输出 `humble`。Humble 的目标平台是 Ubuntu 22.04；不要在 24.04 上强装本分支。

## 2. 获取正确分支

全新克隆推荐直接指定分支：

```bash
git clone --branch ubuntu-22.04-ros2-humble \
  https://github.com/ZiqiChai/k4a-ros.git
cd k4a-ros
git branch --show-current
```

已有 clone：

```bash
cd k4a-ros
git fetch origin
git switch ubuntu-22.04-ros2-humble
```

输出应为 `ubuntu-22.04-ros2-humble`。

## 3. 安装 ROS 包构建依赖

如果希望自动完成本节、SDK、udev 和 ROS 构建，可直接运行：

```bash
./scripts/bootstrap.sh
```

已完成 udev 配置时使用 `./scripts/bootstrap.sh --skip-udev`。以下为全部手动步骤。

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  curl \
  git \
  python3-colcon-common-extensions \
  python3-rosdep
```

仅当本机从未初始化 rosdep 时运行：

```bash
sudo rosdep init
```

然后安装 `package.xml` 声明的依赖：

```bash
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro humble -r -y
```

## 4. 部署 Azure Kinect Sensor SDK

### 路线 A：官方二进制，推荐

```bash
./scripts/setup_sdk.sh
source scripts/env.sh
```

首次执行会显示 Microsoft SDK runtime 许可位置并要求确认。CI 或其他非交互环境必须显式使用
`--accept-sdk-license` 或设置 `ACCEPT_EULA=Y`；这表示执行者已经审阅并接受该许可。

脚本固定使用 SDK 1.4.2，下载并校验三个 Microsoft 官方包：

- `libk4a1.4`：runtime 与闭源 Depth Engine
- `libk4a1.4-dev`：headers 和 CMake config
- `k4a-tools`：`k4arecorder`、`k4aviewer` 等工具

它们只解包到 `.deps/k4a-sdk`，不会修改 `/usr` 或 apt 状态。每个新终端在构建或运行前都要：

```bash
source /opt/ros/humble/setup.bash
source scripts/env.sh
```

> Ubuntu 18.04 版 `k4aviewer` 依赖旧 ABI `libsoundio.so.1`，Jammy 默认只提供
> `libsoundio.so.2`，因此 viewer 不属于本分支的推荐验证路径。`k4arecorder --list`、ROS 2
> driver 和 RViz2 不受影响；不要把 `libsoundio.so.2` 伪装成 `.so.1`。

### 路线 B：源码编译核心 SDK

源码路线用于审计或长期留档。由于 Depth Engine 本身闭源，仍必须从经过 SHA-256 校验的
Microsoft runtime 包部署 `libdepthengine.so.2.0`。本路线编译并替换核心 `libk4a`，其余
headers/tools/runtime 文件来自相同的 1.4.2 官方包。

安装上游 CMake 配置阶段可能检查的开发包：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libgl1-mesa-dev \
  libopencv-dev \
  libsoundio-dev \
  libudev-dev \
  libusb-1.0-0-dev \
  libvulkan-dev \
  libx11-dev \
  libxcursor-dev \
  libxinerama-dev \
  libxrandr-dev \
  libxxf86vm-dev \
  pkg-config
```

构建和私有部署：

```bash
./scripts/setup_sdk.sh --method source
source scripts/env.sh
```

脚本会完成以下可复现动作：

1. 克隆 `v1.4.2` 及全部 submodule；
2. 检查上游 commit `cd2b3e9894b26fbeebfe34a5cab5b26efab0939a`；
3. 应用仓库内 Ubuntu 22.04/OpenSSL 3 最小兼容补丁；
4. 使用 GCC/CMake Release 模式构建 `k4a` target；
5. 将源码产物部署进 `.deps/k4a-sdk`，并保留官方 Depth Engine。

也可使用已经递归克隆的专用 checkout：

```bash
git clone --branch v1.4.2 --recursive \
  https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git /path/to/k4a-sdk-source
./scripts/setup_sdk.sh --method source --source-dir /path/to/k4a-sdk-source
```

脚本会在该 checkout 中应用仓库补丁，因此请使用专门的、可修改的 checkout。

## 5. 安装 udev 规则

仓库内规则来自 Azure Kinect Sensor SDK。执行：

```bash
./scripts/install_udev_rules.sh
```

它会安装 `/etc/udev/rules.d/99-k4a.rules`、重载规则并触发 USB 设备。完成后重新插拔相机。
当前用户通常应在 `plugdev` 组：

```bash
groups
lsusb | grep '045e:'
source scripts/env.sh
k4arecorder --list
```

若 `plugdev` 缺失，可执行 `sudo usermod -aG plugdev "$USER"`，随后注销并重新登录。

## 6. 编译 ROS 2 包

推荐使用路径无关的封装脚本：

```bash
./scripts/build_ros.sh
source install/setup.bash
```

等价的展开命令为：

```bash
source /opt/ros/humble/setup.bash
source scripts/env.sh
colcon build --base-paths "$PWD" --packages-select k4a_ros \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 7. 真机测试

先验证 SDK 能枚举设备：

```bash
source scripts/env.sh
k4arecorder --list
```

启动驱动与 RViz2：

```bash
source /opt/ros/humble/setup.bash
source scripts/env.sh
source install/setup.bash
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

另开终端检查频率和 diagnostics：

```bash
cd k4a-ros
source /opt/ros/humble/setup.bash
source scripts/env.sh
source install/setup.bash
ros2 topic hz /k4a/depth/image_raw
ros2 topic echo /diagnostics --once
```

一键真机数据内容检查会自动启动驱动，等待 Color/Depth/IR/PointCloud/IMU，并在退出时停止驱动：

```bash
./scripts/run_hardware_test.sh
```

若失败，脚本会保留 `/tmp/k4a_ros_hardware_test.*.log`。后续使用见 [USAGE.md](USAGE.md)，
问题定位见 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)。
