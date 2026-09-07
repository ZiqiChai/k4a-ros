# 官方 SDK 包放置目录

完整离线部署包可在本目录放置以下 Microsoft 官方文件：

- `libk4a1.4_1.4.2_amd64.deb`
- `libk4a1.4-dev_1.4.2_amd64.deb`

`scripts/setup_sdk.sh` 会先校验 SHA-256，再通过 apt 安装。`.deb` 默认被 Git 忽略；是否随交付物
保存或分发，以 Microsoft Azure Kinect Sensor SDK 许可为准。
