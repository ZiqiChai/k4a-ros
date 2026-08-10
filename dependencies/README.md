# 固定依赖

本目录只保存可审计的版本锁定、兼容补丁和许可证允许再分发的文本文件，不提交 Microsoft
闭源 Depth Engine 或大型二进制包。

- `azure-kinect-sdk-1.4.2.env`：上游 tag、commit、下载路径和 SHA-256 的唯一事实来源。
- `SHA256SUMS`：便于 release asset 和离线介质直接执行 `sha256sum -c`。
- `patches/0001-disable-unused-tls-backend.patch`：Ubuntu 22.04/OpenSSL 3 源码构建兼容补丁。
- `cache/`（可选，不纳入 Git）：可预置三个官方 `.deb` 和带 submodule 的 SDK 源码归档，供离线安装。

缓存生成、镜像地址覆盖和校验方法见 [`docs/DEPENDENCIES.md`](../docs/DEPENDENCIES.md)。
