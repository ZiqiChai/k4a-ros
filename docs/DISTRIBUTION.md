# 分发与预编译产物

本项目采用“Git 仓库只保存项目源码；GitHub Actions 发布项目自身预编译包；Microsoft SDK 由用户
接受许可后校验下载”的默认策略。这样既能快速部署，也不会把第三方闭源二进制混入 Git 历史。

## 内容边界

| 内容 | 默认公开分发 | 方式 |
|---|---|---|
| `k4a_ros` 源码、配置、脚本、文档 | 是 | Git clone / GitHub source archive |
| Ubuntu 22.04 + Humble 的 `k4a_ros` 编译产物 | 是 | CI artifact；tag release tarball |
| Azure Kinect SDK 源码 | 不随主仓库复制 | 固定 commit 自动 clone；可建立保留许可证的独立镜像 |
| Microsoft SDK `.deb` | 否 | 用户接受许可后按固定 URL/SHA-256 下载 |
| `libdepthengine.so` 等可再分发 object code | 默认否 | 只有经过许可审查的完整应用 bundle 才考虑包含 |
| 私有离线缓存 | 可选 | 访问受控制品库或离线介质，不进入公共 Git 历史 |

Azure Kinect SDK 的 GitHub 源码使用 MIT；官方 Debian runtime 内的 `LICENSE.txt` 是单独的
Microsoft 软件许可。其 `REDIST.txt` 列出 `libk4a`、`libk4arecord` 和 Depth Engine object code，
但分发必须随具有显著主要功能的应用，并满足最终用户条款、notice、indemnification 等要求。
因此本项目不把三个原始 `.deb` 当作普通 release assets 再托管。本说明不是法律意见。

## clone 后最快路径

安装 ROS 2 Humble 后：

```bash
git clone https://github.com/ZiqiChai/k4a-ros.git
cd k4a-ros
./scripts/bootstrap.sh
```

以上默认获取 `main`。需要固定 Ubuntu 22.04 / ROS 2 Humble 稳定维护线时，clone 时增加
`--branch ubuntu-22.04-ros2-humble`。

对于已准备好依赖缓存和 udev 的离线机器：

```bash
./scripts/bootstrap.sh \
  --offline \
  --cache-dir /media/offline/k4a-cache \
  --accept-sdk-license \
  --skip-udev
```

源码 SDK 路线：

```bash
./scripts/bootstrap.sh --sdk-method source
```

## ROS 2 预编译包

`scripts/package_ros.sh` 在独立 build/install 目录中执行非 symlink Release 构建，生成：

```text
dist/k4a_ros-<version>-ubuntu22.04-ros2-humble-amd64.tar.gz
dist/k4a_ros-<version>-ubuntu22.04-ros2-humble-amd64.tar.gz.sha256
```

tarball 包含：

- 可重定位的 `ros2_install`；
- SDK 校验下载、环境配置和 udev 安装脚本；
- 固定 SDK 版本、URL、SHA-256 以及源码兼容补丁；
- 项目 README、LICENSE、CHANGELOG；
- 完整用户文档；
- `BUILD_INFO` 和逐文件 `MANIFEST.sha256`。

它不包含 ROS 2、Azure Kinect SDK 或 Depth Engine。解压预编译包后，使用者仍需安装 ROS 2
Humble，并在包内接受 SDK 许可、执行校验下载：

```bash
cd /path/to/extracted/k4a_ros-<version>-ubuntu22.04-ros2-humble-amd64
./scripts/setup_sdk.sh
source /opt/ros/humble/setup.bash
source scripts/env.sh
source ros2_install/setup.bash  # 发布包的可重定位入口
ros2 launch k4a_ros k4a.launch.py
```

发布包以 Bash 为标准激活方式；请勿直接 source 通用的 `setup.sh`。后者受 POSIX sh 无法可靠获知
被 source 文件路径的限制，必须由调用者预先设置 `COLCON_CURRENT_PREFIX`。

预编译包只承诺 Ubuntu 22.04 amd64 + ROS 2 Humble ABI，不作为跨 Ubuntu/ROS 版本通用二进制。

## GitHub Actions 覆盖

[`ci.yml`](../.github/workflows/ci.yml) 在官方 Humble/Jammy 容器内执行：

1. GCC/Clang 的 C++17 Release 构建及 GCC/C++20 兼容构建；
2. ament lint/test；
3. 官方 SDK binary 下载、SHA-256 与 offline replay；
4. 固定 commit 的完整 SDK source build；
5. 安装空间和 launch 参数检查；
6. ROS 预编译 tarball、SHA-256 和 GitHub artifact attestation。

CI 无法访问物理 USB，因此真机验证仍由 `scripts/run_hardware_test.sh` 和发布检查清单负责。

tag release workflow 只上传本项目 ROS tarball，不上传 Microsoft SDK 包。GitHub Actions 临时 artifact
用于 PR/分支验证，tag release 用于长期公开分发。
