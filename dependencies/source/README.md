# 固定 SDK 源码放置目录

长期维护部署包可在本目录放置 `Azure-Kinect-Sensor-SDK/` Git checkout。它必须位于
`dependencies/azure-kinect-sdk-1.4.2.env` 固定的 commit，并包含完整 recursive submodule。

`tools/build_sdk_system_packages.sh` 会校验 revision、应用仓库跟踪的 Ubuntu 22.04 补丁并生成
系统安装 `.deb`。源码 checkout 默认被 Git 忽略，可按其开源许可纳入内部源码归档。
