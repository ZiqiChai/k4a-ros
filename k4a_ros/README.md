# k4a_ros

Azure Kinect DK 的标准 ROS 2 `ament_cmake` 驱动包，支持 Ubuntu 22.04 amd64 / ROS 2 Humble。

## 系统依赖

Azure Kinect Sensor SDK 1.4.2 runtime/dev 必须安装在系统默认路径。在完整仓库根目录执行：

```bash
./scripts/setup_sdk.sh
```

若只复制本包到其他仓库，请先确保系统已安装 `libk4a1.4` 和
`libk4a1.4-dev`。

## 标准 workspace 编译与运行

```bash
mkdir -p ~/ros_ws/src
cp -a k4a_ros ~/ros_ws/src/
cd ~/ros_ws

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

## 功能

- RGB、Depth、IR 与 `CameraInfo`
- Depth→Color、Color→Depth 标定对齐图像
- Color optical frame 下的有序 XYZRGB 点云；关闭 RGB 时回退到 Depth optical frame
- IMU、温度、硬件时间戳、静态 TF 与 diagnostics
- standalone node、ROS 2 component 和多设备选择

完整 Topics、参数和故障排查位于仓库根目录 `docs/`。

## 许可证

本包使用 [BSD-3-Clause](LICENSE)。Azure Kinect SDK 和 Depth Engine 保留各自许可。
