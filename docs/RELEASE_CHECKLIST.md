# Ubuntu 22.04 分支发布检查清单

本清单供维护者上传分支或创建 release 前使用。任何包含凭据、个人绝对路径或未审计二进制的状态
都不应发布。

## 仓库状态

- [ ] 当前分支为 `ubuntu-22.04-ros2-humble`
- [ ] clone 目录与文档统一使用 `k4a-ros`
- [ ] 全仓库搜索确认无维护者本机绝对路径或旧项目目录名
- [ ] `git diff --check` 通过
- [ ] `.deps/`、`build/`、`install/`、`log/`、录制文件未被跟踪
- [ ] 没有 token、密码、私钥、设备隐私数据或本机用户名

## 依赖与许可证

- [ ] SDK tag、commit、URL 和三个 SHA-256 与 lock file 一致
- [ ] 二进制安装在空缓存条件下成功
- [ ] `--offline` 使用预置缓存成功
- [ ] 源码路线在 Ubuntu 22.04 干净环境构建成功
- [ ] 私有灾备保存三个官方 `.deb` 与保留 `.git` 元数据的递归 SDK 源码归档
- [ ] 公共 release assets 不含 Microsoft `.deb`、Depth Engine 或其他未授权独立二进制
- [ ] 若将可再分发 object code 随应用打包，已完成 Microsoft 条款和最终用户条款审查
- [ ] Git 历史中不包含闭源 Depth Engine 或大型 `.deb`

## ROS 构建与测试

```bash
source /opt/ros/humble/setup.bash
source scripts/env.sh
colcon build --base-paths "$PWD" --packages-select k4a_ros \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select k4a_ros --event-handlers console_direct+
colcon test-result --verbose
```

- [ ] Release 构建成功
- [ ] ament lint/test 全部通过或每个 skip 都有明确原因
- [ ] launch 文件可加载
- [ ] `k4arecorder --list` 可枚举设备
- [ ] `scripts/run_hardware_test.sh` 通过
- [ ] RGB、Depth、IR、aligned images、point cloud、IMU、TF、diagnostics 已抽查
- [ ] 默认 30 FPS 下无持续 capture timeout

## 文档与发布

- [ ] README 快速开始在全新 clone 中逐条执行过
- [ ] `docs/INSTALL.md` 的二进制、源码、udev、编译和真机路径都可复现
- [ ] `docs/DEPENDENCIES.md` 中离线缓存可取得
- [ ] `CHANGELOG.md`、`package.xml`、`CMakeLists.txt` 版本一致
- [ ] GitHub Actions 的 GCC、Clang、SDK source jobs 全部通过
- [ ] 预编译 tarball 的 SHA-256、manifest 和 artifact attestation 可验证
- [ ] 分支保护、tag 和 release notes 已准备
- [ ] 上传前再次查看 `git status` 与最终 diff

提交、推送、创建 tag/release 均是独立发布动作，应由维护者在确认最终 diff 后执行。
