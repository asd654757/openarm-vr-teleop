# 上游项目与许可保留

本仓库是多项目工作树的源码快照，包含本地未提交修改。没有统一替换各组件原有许可证。

| 组件 | 上游参考 | 本地许可文件 |
| --- | --- | --- |
| LeRobot | https://github.com/huggingface/lerobot | `lerobot/LICENSE` |
| OpenArm 资料 | https://github.com/enactic/openarm | `openarm/LICENSE` |
| OpenArm ROS 2 | https://github.com/enactic/openarm_ros2 | 两套 `openarm_ros2/LICENSE` 及各包中的许可 |
| OpenArm CAN | https://github.com/enactic/openarm_can | 两套 `openarm_can/LICENSE.txt` |
| OpenArm Description | https://github.com/enactic/openarm_description | 两套 `openarm_description/LICENSE.txt` |
| OpenArm MuJoCo | https://github.com/enactic/openarm_mujoco | `openarm_mujoco/LICENSE` |
| OpenArmX VR | https://github.com/openarmx/openarmx_teleop_vr | 各 VR 目录中的 `LICENSE` |
| TeleopXR | https://github.com/qrafty-ai/teleop_xr | `teleop_xr/LICENSE` |
| OpenArmX Description | 见组件 README | `OpenarmXR/openarmx_description/LICENSE` |
| VR App 使用资料 | 见组件 README | 两套 `openarmx_teleop_vr_apk/LICENSE` |

上表提供来源入口，不保证未保留 Git 元数据的副本与某个上游提交完全相同。具有本地 Git 元数据的组件，其导出时基准提交记录在 `SOURCE_SNAPSHOT.json`。

外部 Python 包 `openarmx-arm-driver`、RealSense SDK 等没有作为已安装二进制打包，需按其文档和许可另行安装。
