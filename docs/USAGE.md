# 使用说明

## 启动

每个新终端先加载三层环境：

```bash
cd k4a-ros
source /opt/ros/humble/setup.bash
source scripts/env.sh        # SDK 运行库路径
source install/setup.bash    # 工作空间
```

默认启动（可选同时启动项目自带 RViz2 配置）：

```bash
ros2 launch k4a_ros k4a.launch.py
ros2 launch k4a_ros k4a.launch.py use_rviz:=true
```

指定 namespace 和自定义参数文件：

```bash
ros2 launch k4a_ros k4a.launch.py \
  namespace:=front_k4a \
  params_file:=/absolute/path/to/front_k4a.yaml
```

也可以直接运行 standalone node 或 component：

```bash
ros2 run k4a_ros k4a_driver_node --ros-args \
  -r __ns:=/k4a \
  --params-file "$(ros2 pkg prefix k4a_ros)/share/k4a_ros/config/k4a.yaml"
```

component plugin 为 `k4a_ros::K4ADriverNode`，同时安装了可直接运行的
`k4a_driver_component_node`。

## Topics

默认 namespace 是 `/k4a`。图像和点云使用 `SensorDataQoS`。

| Topic | Type | Encoding / frame |
|---|---|---|
| `/k4a/color/image_raw` | `sensor_msgs/msg/Image` | `bgra8`, color optical |
| `/k4a/color/camera_info` | `sensor_msgs/msg/CameraInfo` | color optical |
| `/k4a/depth/image_raw` | `sensor_msgs/msg/Image` | `16UC1` mm, depth optical |
| `/k4a/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | depth optical |
| `/k4a/ir/image_raw` | `sensor_msgs/msg/Image` | `mono16`, depth optical |
| `/k4a/ir/camera_info` | `sensor_msgs/msg/CameraInfo` | depth optical |
| `/k4a/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | `16UC1` mm, color optical |
| `/k4a/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | color optical |
| `/k4a/aligned_color_to_depth/image_raw` | `sensor_msgs/msg/Image` | `bgra8`, depth optical |
| `/k4a/aligned_color_to_depth/camera_info` | `sensor_msgs/msg/CameraInfo` | depth optical |
| `/k4a/points` | `sensor_msgs/msg/PointCloud2` | organized XYZRGB, m；启用 RGB 时为 color optical，否则为 depth optical |
| `/k4a/imu` | `sensor_msgs/msg/Imu` | rad/s、m/s², IMU frame |
| `/k4a/imu_temperature` | `sensor_msgs/msg/Temperature` | °C, IMU frame |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | serial、captures、timeouts、IMU count |

静态 TF 发布 Depth→Color 与 Depth→IMU 的工厂标定外参。frame 使用 ROS optical 约定：
x 向右、y 向下、z 向前。

## 参数

默认值见 [`k4a_ros/config/k4a.yaml`](../k4a_ros/config/k4a.yaml)。

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `device_id` | `0` | 设备索引，必须非负 |
| `serial_number` | `""` | 非空时按序号查找，优先于索引 |
| `color_enabled` | `true` | 发布 RGB |
| `depth_enabled` | `true` | 发布 Depth |
| `ir_enabled` | `true` | 发布 IR |
| `imu_enabled` | `true` | 发布 IMU 与温度 |
| `point_cloud_enabled` | `true` | 发布有序 XYZRGB 点云；要求 Depth |
| `aligned_depth_enabled` | `true` | Depth 对齐 Color；要求 RGB 与 Depth |
| `aligned_color_enabled` | `true` | Color 对齐 Depth；要求 RGB 与 Depth |
| `color_resolution` | `720p` | `720p/1080p/1440p/1536p/2160p/3072p` |
| `depth_mode` | `nfov_unbinned` | 见下文 |
| `fps` | `30` | `5/15/30`；组合必须受 SDK 支持 |
| `capture_timeout_ms` | `1000` | 等待 capture 的超时，必须大于 0 |
| `depth_frame_id` | `k4a_depth_camera_optical_frame` | Depth frame；关闭 RGB 时也是点云 frame |
| `color_frame_id` | `k4a_color_camera_optical_frame` | RGB frame；启用 RGB 时也是点云 frame |
| `imu_frame_id` | `k4a_imu_frame` | IMU frame |
| `publish_tf` | `true` | 发布工厂标定静态 TF |

`depth_mode` 可选：

- `nfov_2x2_binned`
- `nfov_unbinned`
- `wfov_2x2_binned`
- `wfov_unbinned`
- `passive_ir`

并非所有 color resolution、depth mode 和 FPS 组合都可用。启动失败并提示
`Failed to start Azure Kinect cameras` 时，先恢复默认组合，再逐项修改。

## 多相机

先获取序号：

```bash
source scripts/env.sh
k4arecorder --list
```

为每台相机建立独立 YAML，填写不同 `serial_number`，然后分别使用不同 namespace 启动：

```bash
ros2 launch k4a_ros k4a.launch.py \
  namespace:=front_k4a params_file:=/absolute/path/to/front.yaml
ros2 launch k4a_ros k4a.launch.py \
  namespace:=rear_k4a params_file:=/absolute/path/to/rear.yaml
```

多台设备会快速占满 USB 带宽。优先使用不同 USB controller，并按需降低分辨率/FPS或关闭
aligned images、point cloud。当前驱动以 `standalone` wired sync mode 启动，不提供外部同步参数。

## 性能建议

- 只订阅原始深度时，关闭两个 aligned stream 和 point cloud 可显著降低 CPU 与内存带宽。
- 高分辨率 BGRA32 占用较高；默认 720p/30 FPS 是更稳妥的通用配置。
- 点云为 organized cloud，无效深度点写为非有限值；消费者应检查 `isfinite`。启用 RGB 时，
  点云分辨率与 Color 相同（默认 `1280×720`）；关闭 RGB 时与 Depth 相同（默认 `640×576`）。
- 同一 RGB-D capture 的派生消息共享设备时间戳映射后的 ROS 时间；IMU 保留各自设备时间间隔。

彩色点云为何使用 Color 坐标系，以及遮挡边界重影的修复与实机证据，见
[彩色点云的坐标系与遮挡处理](COLOR_POINT_CLOUD.md)。
