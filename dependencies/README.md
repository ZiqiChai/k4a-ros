# Azure Kinect SDK 锁定

本目录保存 Azure Kinect Sensor SDK 1.4.2 的固定版本、官方 runtime/dev 校验值、固定源码
revision 和 Ubuntu 22.04 兼容补丁。

- `azure-kinect-sdk-1.4.2.env`：固定版本、官方 URL 与 SHA-256；
- `SHA256SUMS`：两个官方 `.deb` 的校验清单；
- `patches/`：从固定源码重建 `libk4a` 所需的兼容补丁；
- `cache/`：部署包可依法放置已校验的官方 runtime/dev；
- `source/`：部署包可放置包含完整 submodule 的固定 SDK checkout。

普通安装只运行 `scripts/setup_sdk.sh`，结果始终进入系统路径。`cache/` 与 `source/` 只用于完整
离线部署包和维护恢复，不产生仓库私有 runtime，也不需要环境变量注入。

`tools/package_sdk_recovery_bundle.sh` 可在许可允许范围内把上述材料生成独立恢复归档；该归档
用于防止上游源码或包源消失，不是普通用户运行 ROS 节点所需的第二套环境。
