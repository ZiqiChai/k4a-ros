# 依赖锁定、缓存与镜像

Azure Kinect SDK 上游已归档，Microsoft 仅提供 Ubuntu 18.04 Debian 包。为减少未来包源下架造成的
不可部署风险，本项目不使用浮动版本或未经验证的第三方包。

## 锁定内容

唯一依赖清单位于 [`dependencies/azure-kinect-sdk-1.4.2.env`](../dependencies/azure-kinect-sdk-1.4.2.env)。

| 内容 | 固定值 |
|---|---|
| SDK tag | `v1.4.2` |
| SDK commit | `cd2b3e9894b26fbeebfe34a5cab5b26efab0939a` |
| `libk4a1.4_1.4.2_amd64.deb` | `1f090b65f696f78739507d0f55f3280d5df6330401b825616ef9eca0125ba8b7` |
| `libk4a1.4-dev_1.4.2_amd64.deb` | `f7e694eae34c4b8216f249f0bf23eb0f3fd9ea00538d41b214c9e8ac2da2b3bc` |
| `k4a-tools_1.4.2_amd64.deb` | `eb3a62cf307575e960cb68056a2d8818eee26fd6597265dcdcdfe930014a33c7` |

安装脚本下载到临时 `.part` 文件，校验成功后才改为正式文件；缓存损坏会重新下载或在 offline
模式下直接失败，不会静默使用。

## 本地缓存

在线执行一次即可形成可复制缓存：

```bash
./scripts/setup_sdk.sh --cache-dir dependencies/cache --accept-sdk-license
(cd dependencies/cache && sha256sum -c ../SHA256SUMS)
```

`dependencies/cache/` 已加入 `.gitignore`。建议维护者在完成许可审查后，将以下内容存入访问受控的
组织制品库或离线介质，至少保留两个独立副本：

1. 三个固定文件名的官方 `.deb`；
2. 递归克隆且 commit 已验证的 Azure Kinect SDK source checkout，或保留 `.git` 元数据的归档；
3. 本仓库同版本 source archive；
4. 本文中的 SHA-256 manifest。

用户把三个 `.deb` 放入任意目录后可完全离线部署二进制 SDK：

```bash
./scripts/setup_sdk.sh --offline --cache-dir /media/offline/k4a-cache
```

离线源码构建还需要一个已经初始化全部 submodule 的 Git checkout：

```bash
./scripts/setup_sdk.sh \
  --method source \
  --offline \
  --cache-dir /media/offline/k4a-cache \
  --source-dir /media/offline/Azure-Kinect-Sensor-SDK
```

不要使用 GitHub 自动生成的 Source code ZIP：上游 submodule 内容不在其中。

## 镜像覆盖

不修改锁定文件即可切换到受信任镜像：

```bash
export K4A_PACKAGE_BASE_URL=https://artifacts.example.com/microsoft-ubuntu-18.04-prod
export K4A_SDK_GIT_REPOSITORY=https://git.example.com/mirror/Azure-Kinect-Sensor-SDK.git
./scripts/setup_sdk.sh --method source
```

包镜像必须保持锁定文件中的相对路径；无论镜像来源如何，SHA-256 与 Git commit 检查始终执行。

## Microsoft runtime 许可确认

SDK 源码仓库使用 MIT，但官方 Debian runtime 包包含单独的 Microsoft Azure Kinect Sensor SDK
许可。脚本在提取 `.deb` 前要求交互确认；非交互环境必须明确传入：

```bash
./scripts/setup_sdk.sh --accept-sdk-license
# 或者与 Microsoft 官方 Debian preinst 一致：
ACCEPT_EULA=Y ./scripts/setup_sdk.sh
```

安装后的原文位于：

```text
.deps/k4a-sdk/usr/share/doc/libk4a1.4/LICENSE.txt
.deps/k4a-sdk/usr/share/doc/libk4a1.4/REDIST.txt
.deps/k4a-sdk/usr/share/doc/libk4a1.4/ThirdPartyNotices.txt
```

## 为什么不提交或公开镜像 SDK 二进制

- `libdepthengine.so.2.0` 是 Microsoft 闭源组件，不能当作本项目源码重新许可；
- 大型二进制会永久膨胀 Git 历史；
- 保留官方来源、精确哈希和访问受控的灾备缓存，能同时满足可审计性和可恢复性。

官方 `REDIST.txt` 列出了一部分可随具有显著主要功能的应用分发的 object code，但这不等于允许把
原始 `.deb` 作为独立下载镜像。本项目默认只分发自己的源码和 ROS 预编译产物，SDK 由最终用户从
固定来源下载。若要制作包含 SDK runtime 的完整离线应用包，必须同时满足 Microsoft 的最终用户
条款、notice、indemnification 等要求，并进行独立许可审查。详见 [DISTRIBUTION.md](DISTRIBUTION.md)。

## 源码补丁边界

[`0001-disable-unused-tls-backend.patch`](../dependencies/patches/0001-disable-unused-tls-backend.patch)
只处理上游旧 `azure-c-shared-utility` 在 `use_openssl=OFF` 时仍引用 OpenSSL backend 的问题。
核心相机 SDK 不需要网络 TLS；禁用该旧 backend 可避开 Ubuntu 22.04 的 OpenSSL 3 API 不兼容。
构建同时只把 `-Warray-parameter` 从 error 降为 warning，不修改 SDK 公共 API。
