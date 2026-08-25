# Changelog

## Unreleased

- 修复彩色点云在 RGB/Depth 遮挡边界将前景纹理投到背景表面的重影：启用 RGB 时先把 Depth
  转换到 Color 相机几何，再与原始 Color 像素逐点配对。
- 启用 RGB 时，`/k4a/points` 由 Depth optical frame/Depth 分辨率调整为 Color optical
  frame/Color 分辨率；关闭 RGB 时保持原有 Depth frame/分辨率。`validate_streams.py`
  新增点云 frame 与尺寸的一致性检查。
- 更新 RViz2 预置为已验证的观察视角；新增 `docs/COLOR_POINT_CLOUD.md` 及修复前后实机对比图。
- 修复 RViz2 预置点云和对齐深度图订阅使用 Reliable QoS、无法接收 SensorDataQoS 数据的问题。
- 修复真机测试误用已有 `/k4a/driver` 数据、在新驱动启动失败时仍报告通过的问题。
- 规范化仓库结构：ROS 包迁移至自包含的 `k4a_ros/` 子目录，可直接拷入 colcon 工作空间
  `src/` 编译运行；仓库根保留文档、SDK 依赖管理与一键脚本。
- CMake 最低版本提升至 3.16；包内新增 README 与 LICENSE 副本，不再向 share 安装 docs。
- 精简文档为面向首次使用者的开箱即用流程；修复文档中失效的配置文件链接。
- `setup_sdk.sh` 新增 `--method system`：将官方 `.deb` 系统安装，之后编译与运行均不再
  需要 `source scripts/env.sh`；`env.sh` 检测到系统安装时不再报错。

## 2.0.0 - 2026-08-10

- 将默认主线重构为 Ubuntu 22.04 / ROS 2 Humble 独立实现。
- 提供独立的 C++17 ROS 2 standalone node 与 component。
- 发布 RGB、Depth、IR、CameraInfo、aligned images、organized XYZRGB point cloud、IMU 与温度。
- 增加工厂标定 TF、硬件时间戳映射、diagnostics、多设备选择和 RViz2 配置。
- 固定 Azure Kinect Sensor SDK 1.4.2、官方包 SHA-256 与上游 commit。
- 增加项目私有 SDK 二进制部署、Ubuntu 22.04 源码构建补丁和 offline cache 支持。
- 增加 udev、构建、真机验证脚本及完整发布文档。
- 增加 GCC/Clang、SDK binary/source、offline replay 与预编译制品 GitHub Actions。
- 清理与当前独立 ROS 2 发布无关的历史实现和说明。
