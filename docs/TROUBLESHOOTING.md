# 故障排查

按“USB 枚举 → 权限 → SDK runtime → ROS 环境 → 参数/带宽”的顺序检查，通常最快。

## 找不到相机

```bash
lsusb | grep '045e:'
source scripts/env.sh
k4arecorder --list
```

- `lsusb` 没有输出：检查供电、数据线、USB 3.x 端口或主板 USB controller。
- `lsusb` 有设备但 recorder 无法打开：重新执行 `./scripts/install_udev_rules.sh`，重新插拔，确认
  当前用户属于 `plugdev`。
- 在 Docker/沙箱内：宿主机 udev 正确并不等于容器可访问，必须显式传入相机的所有 USB interface。

检查规则是否生效：

```bash
ls -l /etc/udev/rules.d/99-k4a.rules
sudo udevadm control --reload-rules
```

## 找不到 SDK 或 `k4aConfig.cmake`

错误通常包含 `Could not find a package configuration file provided by "k4a"`。

```bash
./scripts/setup_sdk.sh
source scripts/env.sh
echo "$K4A_SDK_ROOT"
find "$K4A_SDK_ROOT" -name k4aConfig.cmake
```

每个新终端都必须重新 `source scripts/env.sh`，并且要在调用 `colcon build` 之前执行。

## 找不到 `libdepthengine.so.2.0` 或 `libk4a.so.1.4`

```bash
source scripts/env.sh
ldd "$K4A_SDK_ROOT/lib/x86_64-linux-gnu/libk4a.so.1.4.2"
echo "$LD_LIBRARY_PATH"
```

不要只复制 `libk4a.so`。Depth 模式运行必须保留
`$K4A_SDK_ROOT/lib/x86_64-linux-gnu/libk4a1.4/libdepthengine.so.2.0`。

## `k4aviewer` 缺少 `libsoundio.so.1`

这是 Ubuntu 18.04 工具包与 Jammy `libsoundio.so.2` 的 ABI 差异。不要创建欺骗性软链接。
使用以下已验证路径：

```bash
k4arecorder --list
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

viewer 只是 SDK GUI 工具，不是驱动运行依赖。

## 驱动启动后 capture timeout

- 确认相机没有被 `k4arecorder`、另一个 ROS node 或容器占用。
- 直连 USB 3.x，避免无源 Hub。
- 恢复默认 `720p + nfov_unbinned + 30 FPS`。
- 关闭高开销派生流进行隔离：

```yaml
point_cloud_enabled: false
aligned_depth_enabled: false
aligned_color_enabled: false
```

检查统计：

```bash
ros2 topic echo /diagnostics --once
ros2 topic hz /k4a/depth/image_raw
```

## 参数组合不受支持

SDK 会在 `k4a_device_start_cameras` 阶段拒绝不支持的 resolution/depth mode/FPS 组合。
先使用 [`config/k4a.yaml`](../config/k4a.yaml) 的默认值确认硬件正常，再逐项调整。

## RViz2 看不到图像或点云

- 确认 Fixed Frame 是 `k4a_depth_camera_optical_frame`。
- 图像/点云使用 `SensorDataQoS`；自定义订阅者应使用 best-effort sensor data QoS。
- `ros2 topic list` 和 `ros2 topic hz` 先确认数据实际发布。
- 点云显示昂贵；远程桌面或软件渲染环境先只显示 Depth image。

## `run_hardware_test.sh` 失败

脚本会输出并保留日志路径。查看末尾：

```bash
tail -n 100 /tmp/k4a_ros_hardware_test.*.log
```

若已有 `/k4a/driver` 在运行，先正常停止原进程再执行一键测试，避免设备独占冲突。
