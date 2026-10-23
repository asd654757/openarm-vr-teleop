# OpenArm 双臂 VR 遥操作与视觉数据采集

当前 OpenArm 项目的源码快照，包含 Pico VR 遥操作、ROS 2 双臂控制、CAN FD、重力补偿、D435i RGB 视觉采集、LeRobot 数据集录制/重播及相关仿真代码。

该快照保存了工作区中的本地修改。各上游仓库的来源、基准提交、导出范围和排除项见 [SOURCE_SNAPSHOT.json](SOURCE_SNAPSHOT.json)，导出文件校验值见 [SOURCE_FILES.sha256](SOURCE_FILES.sha256)。未打包上游 Git 历史。

## 当前运行链路

```text
Pico 4 OpenArmX App
  -> Wi-Fi UDP 5100
  -> VR bridge + IK
  -> ROS 2 position controller（100 Hz）
  -> CAN FD -> OpenArm 双臂

D435i RGB（640 x 480，30 FPS）
  + 关节 state / action（30 Hz 采样）
  -> LeRobot 视觉数据集
```

运行环境以现有 Ubuntu / ROS 2 Humble 方案为基准。此仓库没有验证 Windows 原生真机控制。

## 目录

| 目录 | 内容 |
| --- | --- |
| `OpenarmXR/` | 当前主要 VR 工作区，包含 CAN、机器人模型、bringup、控制器和 VR bridge/teleop |
| `ros2_ws/src/` | 另一套 ROS 2 源码；其中包含主要工作区构建时额外引用的 `openarm_gravity_comp` 重力补偿包 |
| `lerobot/` | 带本地修改的 LeRobot，含相机适配和双臂采集/重播脚本 |
| `openarmx_teleop_vr/` | 独立 VR 源码副本和历史操作文档 |
| `openarmx_teleop_vr_apk/` | 头显 App 使用说明和许可；不包含 APK 二进制 |
| `teleop_xr/` | WebXR 路线源码和 ROS 2 适配 |
| `openarm_mujoco/` | MuJoCo 模型和仿真代码 |
| `openarm/` | OpenArm 上游资料和网站源文件 |

保留两套 ROS 工作区是为了完整保存当前代码状态，不能在整个仓库根目录直接执行无范围的 `colcon build`，否则会找到重复的包名。

## 操作文档

- [无实机重力补偿仿真结果与曲线](docs/gravity_simulation/REPORT.md)
- [重力补偿标定具体流程与只读采集/拟合工具](GRAVITY_CALIBRATION_RUNBOOK.md)
- [重力补偿比例辨识与验证方案（待真机实验）](docs/gravity_compensation_calibration.md)
- [开机到带视觉遥操采集 SOP](lerobot/examples/openarm_xr/OPENARMX_VISUAL_TELEOP_RECORDING_SOP.md)
- [项目技术总结](lerobot/examples/openarm_xr/OPENARMX_PROJECT_TECHNICAL_SUMMARY.md)
- [采集、重播与 Rerun 工作流](lerobot/examples/openarm_xr/OPENARMX_LEROBOT_TROUBLESHOOTING_AND_WORKFLOW.md)
- [双臂录制和重播说明](lerobot/examples/openarm_xr/OPENARMX_BIMANUAL_100HZ_RECORD_REPLAY.md)
- [OpenarmXR 工作区说明](OpenarmXR/README.md)

文档中部分路径仍是原开发机的 `~/OPENARM`、`/home/w/OPENARM` 和 `oplerobot310` 环境名。新机器上需要替换为实际克隆位置和环境，不应直接照搬绝对路径。

## 在新机器恢复源码

本仓库是源码备份，未打包系统依赖或 Conda 环境。先按各组件文档安装 ROS 2、编译依赖、相机 SDK 和 Python 依赖，再进行构建。不要在迁移过程中直接升级全部 Python 包；当前 LeRobot 本地版本与在线最新版的依赖要求可能不同。

当前 `OpenarmXR` 的重力补偿包实际来自另一套工作区。构建时需要显式加入该目录，例如从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
cd OpenarmXR
colcon build --symlink-install \
  --base-paths . ../ros2_ws/src/openarm_ros2/openarm_gravity_comp
source install/setup.bash
```

这段命令说明源码范围，不代表已在一台干净机器上完成依赖安装和全量构建验收。后续真实硬件操作以 SOP 为准。

`ros2_ws/src` 中指向独立 VR 包的链接已改成仓库内部相对链接，不再依赖原开发机的绝对路径。机器人 URDF、网格模型和各组件许可文件均随源码保留。

## 备份范围

保留：源码、工作区中的本地修改、配置、模型、脚本、文档、上游许可及来源记录。

未包含：

- `build/`、`install/`、日志、Python 缓存、前端构建产物和依赖目录。
- `lerobot/data/` 等采集数据集、录包、视频、训练权重和 Rerun `.rrd` 文件。
- APK、Android platform tools、Conda 环境和已安装的 Python 编译扩展。
- 本机私钥、证书、`.env` 和账户配置。
- 根目录下载的网页副本、约 86 MiB 的 CAN 探测 CSV。

这些文件继续留在原开发目录。模型文件应继续保留；部分上游 `.gitignore` 会忽略新建的 URDF/STL，后续新增模型时请确认其已经进入 Git 暂存区。

## 上游和许可

此仓库由多个上游项目及本地修改组成，不能将所有文件统一视为一种许可证。各子目录现有的 `LICENSE` / `LICENSE.txt` 文件和源码版权声明继续适用。详见 [THIRD_PARTY.md](THIRD_PARTY.md)。
