# 安装与部署

面向首次使用者：从全新 Ubuntu 22.04 到真机出流。命令默认在仓库根目录 `k4a-ros` 下执行。

## 0. 前提

- Ubuntu 22.04 amd64
- ROS 2 Humble（按
  [官方指南](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 安装
  `ros-humble-desktop`；`printenv ROS_DISTRO` 应输出 `humble`）
- Azure Kinect DK 通过数据规格 USB 3.x 线缆直连主板端口
- 可用 `sudo` 安装系统包和 udev 规则

## 1. 一键安装（推荐）

```bash
git clone https://github.com/ZiqiChai/k4a-ros.git
cd k4a-ros
./scripts/bootstrap.sh
```

脚本依次完成：安装系统/ROS 依赖 → 下载并校验 SDK（首次会提示接受 Microsoft 许可）→
安装 udev 规则 → 编译 ROS 包。已配置过 udev 时加 `--skip-udev`；非交互环境加
`--accept-sdk-license`。

想让 SDK 装进系统、之后完全按标准 ROS 包的方式使用（编译和运行都不需要
`scripts/env.sh`）时，改用：

```bash
./scripts/bootstrap.sh --sdk-method system
```

完成后运行：

```bash
source /opt/ros/humble/setup.bash
source scripts/env.sh
source install/setup.bash
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

RViz 中出现点云和图像即安装成功。以下各节是 bootstrap 背后的手动步骤，排查问题时按节对照。

## 2. 系统与 ROS 依赖

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y build-essential cmake curl git \
  python3-colcon-common-extensions python3-rosdep
sudo rosdep init          # 仅首次，已初始化可跳过
rosdep update
rosdep install --from-paths k4a_ros --ignore-src --rosdistro humble -r -y
```

Azure Kinect SDK 不在 rosdep 数据库中，由下一步的专用脚本部署。

## 3. 部署 Azure Kinect SDK

### 路线 A：官方二进制（默认，推荐）

```bash
./scripts/setup_sdk.sh
source scripts/env.sh
```

脚本固定 SDK 1.4.2，下载并逐个校验 SHA-256 后解包三个官方包到 `.deps/k4a-sdk`
（不改动 `/usr` 与 apt）：`libk4a1.4`（runtime 与闭源 Depth Engine）、`libk4a1.4-dev`
（headers 与 CMake config）、`k4a-tools`（`k4arecorder` 等）。

**每个新终端**在编译或运行前都要：

```bash
source /opt/ros/humble/setup.bash
source scripts/env.sh
```

### 路线 B：源码编译 libk4a（审计/留档用）

Depth Engine 仍来自官方 runtime 包，只有核心 `libk4a` 从固定 commit
`cd2b3e98`（tag `v1.4.2`）编译。先装上游构建依赖：

```bash
sudo apt install -y libgl1-mesa-dev libopencv-dev libsoundio-dev libudev-dev \
  libusb-1.0-0-dev libvulkan-dev uuid-dev libx11-dev libxcursor-dev \
  libxinerama-dev libxrandr-dev libxxf86vm-dev pkg-config
./scripts/setup_sdk.sh --method source
source scripts/env.sh
```

脚本自动克隆、校验 commit、应用 Ubuntu 22.04 兼容补丁并构建。已有递归克隆时可指定：

```bash
./scripts/setup_sdk.sh --method source --source-dir /path/to/k4a-sdk-source
```

> 提示：官方 `k4aviewer` 依赖 Ubuntu 18.04 的旧 `libsoundio.so.1` ABI，在 22.04 上无法运行，
> 属预期现象。验证请用 `k4arecorder --list` 或 ROS 2 + RViz2，不要伪造软链接。

### 路线 C：系统安装（完全标准的 ROS 工作流）

把校验过的官方 `.deb` 安装到系统（需要 sudo，会修改 apt 状态）：

```bash
./scripts/setup_sdk.sh --method system
```

安装后 `k4a` 的 CMake config 和动态库都在系统默认路径上，**编译和运行都不再需要
`source scripts/env.sh`**，与任何标准 ROS 包的使用方式一致：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths "$PWD/k4a_ros" --packages-select k4a_ros \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

许可仍由脚本先确认；官方 runtime 包自带的 EULA 检查由 `ACCEPT_EULA=Y` 满足。
路线 A/B 与路线 C 可以共存，`scripts/env.sh` 检测到私有前缀时会优先使用它。

## 4. 安装 udev 规则

```bash
./scripts/install_udev_rules.sh
```

随后重新插拔相机，并确认当前用户在 `plugdev` 组（否则
`sudo usermod -aG plugdev "$USER"` 后注销重登）：

```bash
groups
source scripts/env.sh
k4arecorder --list        # 应列出相机序号
```

## 5. 编译 ROS 包

```bash
./scripts/build_ros.sh
source install/setup.bash
```

等价的手动命令：

```bash
colcon build --base-paths "$PWD/k4a_ros" --packages-select k4a_ros \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 放入自己的 colcon 工作空间（标准用法）

`k4a_ros/` 是自包含包，可直接拷贝（或符号链接）到任意工作空间的 `src/`：

```bash
mkdir -p ~/ros2_ws/src && cp -r k4a_ros ~/ros2_ws/src/
source /opt/ros/humble/setup.bash
source /path/to/k4a-ros/scripts/env.sh   # 路线 C 系统安装时可省略本行
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 6. 真机验证

```bash
source /opt/ros/humble/setup.bash && source scripts/env.sh && source install/setup.bash
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

另开终端检查数据频率与设备状态：

```bash
ros2 topic hz /k4a/depth/image_raw
ros2 topic echo /diagnostics --once
```

或一键自动化检查（自动启动驱动、验证 Color/Depth/IR/点云/IMU、退出时停止驱动）：

```bash
./scripts/run_hardware_test.sh
```

后续使用见 [USAGE.md](USAGE.md)；遇到问题见 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)。
