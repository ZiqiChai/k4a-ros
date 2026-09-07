# 故障排查

## 系统 SDK 未找到

编译错误通常包含 `Could not find a package configuration file provided by "k4a"`。检查：

```bash
dpkg-query -W libk4a1.4 libk4a1.4-dev
ls -l /usr/lib/x86_64-linux-gnu/cmake/k4a/k4aConfig.cmake
ls -l /usr/lib/x86_64-linux-gnu/libk4a1.4/libdepthengine.so.2.0
```

缺失时重新运行仓库根目录的 `./scripts/setup_sdk.sh`，然后在 `~/ros_ws` 重新执行 `colcon build
--cmake-clean-cache`。不要设置项目专用的 `CMAKE_PREFIX_PATH` 或 `LD_LIBRARY_PATH`。

## `k4a-tools` 依赖 `libsoundio1`

官方工具包面向 Ubuntu 18.04，不能通过 Jammy apt 正常安装。本项目不安装 `k4a-tools`；
`k4aviewer` 和 `k4arecorder` 不是 ROS 驱动依赖。不要把 `libsoundio.so.2` 伪装成 `.so.1`。

## 设备权限或找不到相机

```bash
lsusb | grep -i microsoft
ls -l /etc/udev/rules.d/99-k4a.rules
groups
```

重新运行 `setup_sdk.sh` 会安装并刷新 udev 规则。之后重新插拔相机；当前用户应属于 `plugdev`。

## `LIBUSB_ERROR_BUSY`

Azure Kinect 由单个进程独占。关闭其他 ROS launch、viewer、recorder 或容器中的驱动后再启动：

```bash
pgrep -af 'k4a_driver_node|ros2 launch.*k4a'
```

## 驱动启动后 capture timeout

- 相机直连 USB 3.x 主板端口，避免无源 Hub；
- 恢复默认 `720p + nfov_unbinned + 30 FPS`；
- 关闭高开销派生流进行隔离：

```yaml
point_cloud_enabled: false
aligned_depth_enabled: false
aligned_color_enabled: false
```

## RViz2 没有图像或点云

- 推荐使用 `ros2 launch k4a_ros k4a.launch.py use_rviz:=true`；
- 手动添加 Image/PointCloud2 时，将 Reliability Policy 设为 `Best Effort`；
- `/k4a/points` 启用 RGB 时使用 `k4a_color_camera_optical_frame`；
- 使用 `ros2 topic info /k4a/points --verbose` 检查 publisher/subscriber QoS。

驱动刚启动时 RViz 可能短暂创建默认 Reliable 订阅并打印一次不兼容警告；配置加载完成后实际
订阅应为 Best Effort，且不应持续告警。

## 点云遮挡边界重影

当前驱动在 Color 相机几何下生成彩色点云，避免把前景纹理复制到背景表面。若仍有问题，确认
运行的是当前 workspace 构建结果且 `color_enabled: true`，并检查 `/k4a/points` 的
`frame_id` 为 `k4a_color_camera_optical_frame`。

Color→Depth 对齐图在遮挡边界仍可能有数像素拉伸；黑色/高反射物体也会造成 ToF 深度空洞，
两者都是传感器成像特性，不是点云颜色再次重影。

## 真机测试失败

先 source workspace，再执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
~/ros_ws/src/k4a-ros/scripts/run_hardware_test.sh
```

脚本会输出保留的 `/tmp/k4a_ros_hardware_test.*.log` 路径。
