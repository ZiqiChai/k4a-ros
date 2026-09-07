# 系统 SDK 依赖锁定

Azure Kinect Sensor SDK 已归档，1.4.2 是最后一个正式版本。普通部署使用 Microsoft 官方
Ubuntu 18.04 runtime/dev `.deb`，在 Ubuntu 22.04 上通过 apt 系统安装。

## 固定包

| 包 | SHA-256 |
|---|---|
| `libk4a1.4_1.4.2_amd64.deb` | `1f090b65f696f78739507d0f55f3280d5df6330401b825616ef9eca0125ba8b7` |
| `libk4a1.4-dev_1.4.2_amd64.deb` | `f7e694eae34c4b8216f249f0bf23eb0f3fd9ea00538d41b214c9e8ac2da2b3bc` |

唯一事实来源是
[`dependencies/azure-kinect-sdk-1.4.2.env`](../dependencies/azure-kinect-sdk-1.4.2.env)。安装脚本
只接受校验通过的文件，不使用浮动版本或第三方二进制镜像。

## 系统安装内容

```bash
./scripts/setup_sdk.sh
```

安装后关键路径为：

```text
/usr/include/k4a/
/usr/lib/x86_64-linux-gnu/cmake/k4a/
/usr/lib/x86_64-linux-gnu/libk4a.so.1.4.2
/usr/lib/x86_64-linux-gnu/libk4a1.4/libdepthengine.so.2.0
/usr/share/doc/libk4a1.4/LICENSE.txt
```

下载缓存使用系统约定的用户 cache 目录，不参与编译和运行。若交付目录依法包含
`dependencies/cache/` 中的两个固定 `.deb`，脚本会自动使用它们，因此安装不依赖外部包源。
ROS 包只从系统默认 CMake 和动态链接器路径查找 SDK。

## 上游失效时的完整恢复

完整恢复资料由三部分组成：

1. `dependencies/cache/` 中两个 SHA-256 匹配的官方 runtime/dev 包；
2. `dependencies/source/Azure-Kinect-Sensor-SDK/` 中位于固定 commit、包含完整 submodule 的源码；
3. 仓库跟踪的版本锁和 Ubuntu 22.04 兼容补丁。

前两部分可能受 Microsoft 许可和交付环境限制，公共 Git 仓库默认不跟踪其内容；内部部署包可在
许可允许范围内一并保存。只要 runtime/dev 已随交付目录保存，普通 `setup_sdk.sh` 就能完全离线
安装。源码副本用于上游消失或需重新构建开源 `libk4a` 时的长期维护。

在授权环境准备好上述两类材料后，可生成独立保存的 SDK 恢复包：

```bash
./tools/package_sdk_recovery_bundle.sh
```

该归档包含固定递归源码、补丁、锁文件和两个官方 `.deb`，解压到同版本 `k4a-ros` 仓库根目录
即可恢复 `dependencies/cache/` 与 `dependencies/source/`。它不包含预编译 ROS workspace；ROS
包仍按标准 rosdep/colcon 流程编译。是否保存或分发含 Microsoft 二进制的归档，以 SDK 许可为准。
恢复归档只消除 Azure Kinect 上游归档风险，不替代 Ubuntu 22.04 与 ROS 2 Humble 的系统依赖源。

安装源码构建依赖后，可生成仍由 apt 管理的恢复包：

```bash
sudo apt install -y build-essential cmake git libgl1-mesa-dev libopencv-dev \
  libsoundio-dev libudev-dev libusb-1.0-0-dev libvulkan-dev uuid-dev \
  libx11-dev libxcursor-dev libxinerama-dev libxrandr-dev libxxf86vm-dev pkg-config
./tools/build_sdk_system_packages.sh
```

输出位于 `dist/sdk-system/`。工具从固定源码重新编译 `libk4a`，并与官方 runtime 中不可从公开
源码重建的 Depth Engine 重新组装为 runtime/dev `.deb`，由 apt 安装到 `/usr`。

## 工具包边界

`k4a-tools_1.4.2_amd64.deb` 依赖 Jammy 不提供的 `libsoundio1`，且其中的 viewer/recorder 不是
驱动依赖，因此不下载、不安装、不锁定。不要为其创建 `libsoundio.so.1` 欺骗性软链接。

## 许可

runtime 包包含单独的 Microsoft Azure Kinect Sensor SDK 许可。交互安装需要用户确认；CI 必须
显式使用 `--accept-sdk-license` 或 `ACCEPT_EULA=Y`。公共发布是否可附带官方包和 Depth Engine，
必须以对应许可为准；不能用开源 `libk4a` 假定替代闭源 Depth Engine。

包内 `REDIST.txt` 只列出可在满足条款的应用中分发的 object code。它不等于允许把原始 runtime/
dev `.deb` 作为独立下载公开镜像；公共制品必须另行完成许可审查。本仓库提供的是技术打包和校验
能力，不作分发授权判断。
