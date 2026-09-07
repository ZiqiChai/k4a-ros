# 安装与工作空间编译

本文给出唯一受支持的部署方式：SDK 系统安装，ROS 包在标准 `ros_ws` 中由 `rosdep` 和
`colcon` 编译。

## 1. 前提

- Ubuntu 22.04 amd64
- ROS 2 Humble desktop
- Azure Kinect DK 通过 USB 3.x 直连
- 当前用户可使用 `sudo`

安装标准 ROS 构建工具：

```bash
sudo apt update
sudo apt install -y build-essential git python3-colcon-common-extensions python3-rosdep
```

若 rosdep 尚未初始化，只执行一次：

```bash
sudo rosdep init
rosdep update
```

## 2. 克隆到 ROS workspace

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
git clone https://github.com/ZiqiChai/k4a-ros.git
```

仓库中的 `k4a_ros/` 是标准 `ament_cmake` 包，`colcon` 会从 workspace 的 `src` 下递归发现它。

## 3. 一键系统安装 SDK

```bash
cd ~/ros_ws/src/k4a-ros
./scripts/setup_sdk.sh
```

脚本完成以下操作：

1. 从固定 Microsoft URL 下载 Azure Kinect SDK 1.4.2 runtime/dev；
2. 校验两个 `.deb` 的 SHA-256；
3. 确认 Microsoft runtime 许可；
4. 通过 apt 安装 `libk4a1.4` 和 `libk4a1.4-dev`；
5. 安装并刷新 Azure Kinect udev 规则；
6. 验证 CMake config、Depth Engine 和安装版本。

CI 可使用 `./scripts/setup_sdk.sh --accept-sdk-license`。

官方 `k4a-tools` 依赖 Ubuntu 22.04 不提供的 `libsoundio1`，因此不会安装。这不影响 ROS driver、
RViz2、Depth Engine、图像、IMU 或点云。

## 4. rosdep 与 colcon 编译

打开新终端：

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

SDK 的 CMake config 位于系统默认搜索路径，`find_package(k4a 1.4.2 CONFIG REQUIRED)` 可直接
找到它。Azure Kinect SDK 尚无公共 rosdep key，因此 SDK 安装由第 3 节单独完成；其余依赖全部
由 package.xml 和 rosdep 管理。

## 5. 运行

每个新终端执行标准 ROS underlay 和 workspace overlay：

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

仅启动节点：

```bash
ros2 launch k4a_ros k4a.launch.py
```

停止使用 `Ctrl+C`。不要同时运行第二个相机进程，否则设备独占会导致 `LIBUSB_ERROR_BUSY`。

## 6. 验证

确认系统 SDK：

```bash
dpkg-query -W libk4a1.4 libk4a1.4-dev
test -f /usr/lib/x86_64-linux-gnu/cmake/k4a/k4aConfig.cmake
test -f /usr/lib/x86_64-linux-gnu/libk4a1.4/libdepthengine.so.2.0
```

编译测试：

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select k4a_ros --event-handlers console_direct+
colcon test-result --verbose
```

真机流检查：

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
~/ros_ws/src/k4a-ros/scripts/run_hardware_test.sh
```
