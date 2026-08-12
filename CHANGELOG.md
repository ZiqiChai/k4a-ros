# Changelog

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
