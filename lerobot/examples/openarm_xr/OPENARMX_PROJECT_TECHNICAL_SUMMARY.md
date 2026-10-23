# Pico 4 / OpenArmX / LeRobot 项目技术总结与操作手册

> 适用范围：Pico 4 VR 遥操作、OpenArm/OpenArmX 仿真与真实双臂、ROS 2
> Humble、LeRobot 数据采集/重播、D435i 全局 RGB 相机，以及后续 ACT、
> SmolVLA、Pi0 和 MuJoCo 部署。
>
> 本文记录的是当前本地工作区已经实现或验证过的方案。涉及真实机械臂的
> 命令必须在急停可用、工作区清空、操作者能够立即断电的条件下执行。
>
> 文档更新：2026-08-31；“已验证”仅表示当前工作区/已有实验结果，新增硬件
> 配置仍需在用户机器上做短时验收。

## 0. 先看结论

当前项目已经从“让 Pico 4 控制机械臂”进入“建立可用于 VLA 的高质量示教
数据流水线”阶段。推荐保留下面的分层：

```text
Pico 4 / OpenArmX VR              100 Hz
        ↓ UDP 5100 + Pinocchio IK
ROS 2 position command            100 Hz
        ↓ ros2_control + SocketCAN FD
真实 OpenArm 双臂                 100 Hz（当前实机稳定点）
        ↓ 只读 recorder
LeRobot state/action 数据         100 Hz（无视觉原始数据）
        ↓ D435i 全局 RGB（独立 30 FPS）
LeRobot VLA 数据                  30 Hz（图像、状态、动作、任务文本）
        ↓ action chunk / 插值 / 保持
VLA 推理                          约 15～30 Hz
```

不要因为 VLA 常见推理频率约 30 Hz，就把真实机械臂底层控制降到 30 Hz。
30 Hz 是策略输出层的频率，100 Hz 是机器人安全、平滑执行层的频率。

当前路线决策：继续使用已经稳定的 OpenArmX VR（UDP + Pinocchio +
direct-topic）作为示教输入，不需要为了“使用 LeRobot”而立刻改成 physical
leader。LeRobot 的价值先放在数据 schema、相机、训练、评估和策略部署层；
只有在需要完全标准化的 CAN follower/leader 工作流时，才单独迁移到官方
适配器，并重新做控制器和安全验证。

目前最可靠的验证顺序是：

```text
fake hardware / RViz
  → 真实双臂、无重力补偿、无 Rerun
  → 100 Hz 无视觉数据与离线检查
  → 真实轨迹 replay
  → gravity_scale=0.01 的低比例补偿
  → D435i 30 FPS RGB 视觉采集
  → synthetic / rendered sim
  → ACT smoke test、SmolVLA 首个模型
  → MuJoCo rollout
  → 真实策略部署（先单臂，再双臂）
```

### 文档导航

- [项目目标、阶段和成果](#1-项目目标阶段和当前成果)
- [系统架构与进程边界](#2-系统架构与进程边界)
- [环境、目录和 Shell 设置](#3-环境目录和启动前的-shell-设置)
- [迭代过程与问题根因](#4-迭代过程做了什么为什么做解决了什么)
- [稳定参数与设置理由](#5-当前稳定参数与设置理由)
- [重力补偿原理和安全调参](#6-重力补偿原理开关和安全调参)
- [LeRobot 采集、验收和 replay](#7-lerobot-数据采集与数据结构)
- [D435i 全局相机](#8-d435i-全局相机检测配置和视觉采集)
- [Rerun 与 LeRobot viewer](#9-从录制到真实-replay)
- [卡顿/抖动排查表](#10-为什么会卡抖和跳变排查顺序)
- [100 Hz、30 Hz 与 realtime-VLA](#12-100-hz-数据30-hz-vla-和-realtime-vla-思路)
- [VLA、ACT、SmolVLA 和 MuJoCo 路线](#13-vlaactmujoco-后续路线)
- [官方文档与本地方案关系](#14-官方-lerobotopenarm-文档与本地方案的关系)

## 1. 项目目标、阶段和当前成果

### 1.1 目标

项目的最终目标不是只做一次 VR 演示，而是形成可重复的闭环：

1. 用 Pico 4 低门槛示教双臂任务。
2. 以一致的关节、夹爪、相机和任务文本格式保存 LeRobot 数据集。
3. 训练 ACT/VLA 策略，并在仿真中先验证。
4. 将策略以较低的推理频率输出动作块，由 100 Hz ROS 2/硬件层平滑执行。
5. 在真实 OpenArm 上以可回退、可急停的方式部署。

### 1.2 里程碑

| 阶段 | 目的 | 结果 |
| --- | --- | --- |
| Pico 4 + `teleop_xr` | 验证 WebXR、手柄位姿、按钮和 OpenArm IK | 已打通；处理了 `gripPose`、无视频 WebRTC、deadman 等兼容问题 |
| OpenArmX/OpenarmXR | 用 UDP + Pinocchio 建立更适合 Pico 4 的实时遥操 | 已形成当前主用 direct-topic 路线 |
| fake hardware/RViz | 在不接电机时验证 ROS 话题、关节映射和按钮 | 已完成 |
| 真实双臂 CAN FD | 将相同 IK 输出接入真实 OpenArm | 已完成基础遥操和安全 SOP |
| 100 Hz 稳定化 | 降低延迟、抑制 Pico 抖动和网络短断流 | 当前稳定组合为 VR 100 Hz + ros2_control 100 Hz |
| 重力前馈 | 减少持续负载导致的下坠和跟手差 | 已完成低比例实机验证；推荐从 0.01 开始 |
| LeRobot 无视觉采集 | 验证 state/action、episode、metadata、replay | 双臂 16D、100 Hz 已完成 |
| callback 修复 | 消除录制中 3°～4° 的阶梯跳变 | 独立 executor 后大多数逐帧更新，单周期约 1° |
| D435i RGB | 将示教扩展为 VLA 所需的图像+状态+动作+文本 | 已识别为 USB 3.2，并成功请求 RGB 30 FPS；下一步做短视觉 episode |
| VLA/仿真 | 训练和部署前闭环验证 | 已规划并有 `openarm_sim`、MuJoCo follower 骨架 |

## 2. 系统架构与进程边界

### 2.1 当前真实双臂主链路（OpenArmX direct-topic）

```text
Pico 4 OpenArmX APK
        │ UDP 5100
        ▼
openarmx_teleop_bridge_vr_node
        │ /pico_* controller pose/joy
        ▼
openarmx_teleop_vr_node
  Pinocchio IK → 一阶滤波 → 单周期限步 → 位置目标
        ├── /left_forward_position_controller/commands
        └── /right_forward_position_controller/commands
                    │
                    ▼
           OpenArm ROS 2 bringup
          ros2_control + CAN FD 驱动
                    │
                    ▼
              真实左右机械臂
```

夹爪是独立话题：

```text
/left_gripper_controller/joint_trajectory
/right_gripper_controller/joint_trajectory
```

LeRobot 录制器只订阅这些话题和 `/joint_states`，不发布动作；因此录制时
不会与 VR 节点抢控制器。重播器则会发布动作，必须与 VR 节点互斥。

### 2.2 另一条 `teleop_xr`/标准 LeRobot 路线

早期验证使用的是：

```text
Pico 4 WebXR
   → teleop_xr ROS 2 bridge
   → /xr/controller_* /pose、/joy
   → OpenArm IK
   → /joint_trajectory
   → pico_openarm_ik / bi_openarm_ros2_follower
   → fake hardware 或 ROS 2 OpenArm
```

这条路线适合仿真和标准 `lerobot-record` 集成；当前真实 OpenArmX VR
主线直接发布 `forward_position_controller/commands`，不要同时启动两条
路线，也不要把它们的 topic、动作顺序或控制器假设混用。

### 2.3 进程职责

| 进程/节点 | 是否向机械臂发命令 | 主要职责 |
| --- | --- | --- |
| OpenArm bringup | 是（底层） | robot description、ros2_control、CAN FD、控制器 |
| VR bridge | 否（只转发输入） | UDP/WebXR 到 ROS 2 pose/joy |
| `openarmx_teleop_vr_node` | 是 | IK、滤波、限步、arm/gripper command |
| `record_openarmx_bimanual_lerobot.py` | 否 | 读取 snapshot，写 LeRobot episode |
| `replay_openarmx_bimanual_lerobot.py` | 是 | 读取 action，按频率发布真实控制命令 |
| Rerun | 否 | 记录/显示 state 和 action，不能代替控制器 |
| D435i/`RealSenseCamera` | 否 | 采集全局 RGB；当前不启用深度/IMU |

### 2.4 话题、单位和动作顺序

| 数据 | 话题/格式 | 单位 |
| --- | --- | --- |
| 实测关节状态 | `/joint_states` (`sensor_msgs/JointState`) | ROS 内部 rad；录制后转 deg |
| 左/右 arm 目标 | `Float64MultiArray` | 通常 7 个值，rad；不要假设第 8 项是夹爪 |
| 左/右夹爪目标 | `JointTrajectory` | 位置 m；0 m 约为闭合，0.044 m 约为打开 |
| LeRobot arm/state/action | `float32` | 7 个关节用 deg |
| LeRobot gripper/state/action | `float32` | 闭合约 0°，打开约 -65° |

当前自定义双臂 recorder 的固定顺序是：

```text
left_joint_1..7, left_gripper,
right_joint_1..7, right_gripper     （共 16 维）
```

标准 `BiOpenArmROS2Follower` 适配器的历史顺序是 `right 8 + left 8`。训练、
重播、数据合并和模型配置必须使用同一顺序；如果切换适配器，先显式重排，
不能只改 `info.json`。

## 3. 环境、目录和启动前的 Shell 设置

### 3.1 推荐环境

| 项目 | 当前约定 |
| --- | --- |
| ROS | ROS 2 Humble |
| Python | 3.10，conda 环境 `oplerobot310` |
| LeRobot 源码 | `~/OPENARM/lerobot` |
| OpenarmXR 工作区 | `~/OPENARM/OpenarmXR` |
| 默认右/左 CAN | `can0` / `can1` |
| 数据保存 | `~/OPENARM/lerobot/data/...` |

Python 3.12 与 ROS 2 Humble 二进制 ABI 不匹配时会产生难以解释的导入或
运行错误，因此 ROS 相关命令统一在 `oplerobot310`（Python 3.10）中运行。

### 3.2 每个终端的基础初始化

当前 OpenArmX direct-topic 终端使用：

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot

# 清理其他 ROS 工作区留下的路径，避免消息/包版本串线
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

# 只在运行本地 LeRobot Python 脚本的终端添加
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_HUB_OFFLINE=1
```

启动 ROS launch 的终端可以不设置 LeRobot `PYTHONPATH`；但必须 source 同一
个 OpenarmXR install。`teleop_xr` 旧路线可能使用 `~/OPENARM/ros2_ws/install`，
不要在一个终端交替 source 两个不一致的工作区。

`libtinfo.so.6: no version information available` 是当前 conda/bash 的环境
警告，通常不是本项目故障；真正的 ROS、CAN、相机错误应看后续日志。

### 3.3 关键目录

```text
lerobot/examples/openarm_xr/
  record_openarmx_bimanual_lerobot.py       # 只读 recorder，可选 D435i
  replay_openarmx_bimanual_lerobot.py      # 真实双臂 replay
  OPENARMX_BIMANUAL_100HZ_RECORD_REPLAY.md
  OPENARMX_LEROBOT_TROUBLESHOOTING_AND_WORKFLOW.md
  DUAL_ARM_SMOOTH_TELEOP_GUIDE.md

OpenarmXR/openarmx_teleop_vr/
  .../config/teleop_params.yaml             # VR IK 参数
  .../launch/openarm_teleop_vr_{sim,real}.launch.py

OpenarmXR/openarm_ros2/openarm_bringup/
  launch/openarm.bimanual.launch.py         # 真实/仿真 bringup
  config/controllers/openarm_bimanual_controllers.yaml
```

### 3.4 本地新增/修改的关键实现

| 文件/模块 | 作用 |
| --- | --- |
| `record_openarmx_bimanual_lerobot.py` | OpenArmX direct-topic 双臂只读 recorder；可选 D435i RGB |
| `replay_openarmx_bimanual_lerobot.py` | 真实双臂 action replay、首帧偏差检查、可选 Rerun |
| `openarm_ros2_follower` | 将 ROS 2 OpenArm state/action 封装为 LeRobot robot；后台 executor |
| `pico_openarm_ik` | 将 `/joint_trajectory` 的 Pico IK 输出接入 LeRobot；后台 executor |
| `openarm_sim` | fake joint state 驱动的轻量渲染相机，用于视觉 pipeline smoke test |
| `openarm_mujoco_follower` | 对接官方 OpenArm MuJoCo 相机和 16D action 的后续实验骨架 |
| `openarmx_teleop_vr` 参数/launch | 100 Hz Pinocchio IK、滤波、限步和真实/仿真分流 |

这些实现仍属于当前项目的本地扩展；标准 LeRobot upstream 版本未必包含同名
topic、robot type 或动作顺序。升级 LeRobot 后应重新运行 `--help`、编译和
短集 smoke test。

高频双臂采集应使用上表中的新 recorder；早期单臂
`record_openarmx_left_lerobot.py` 保留了同步 `spin_once()` 的旧结构，适合低频
兼容性测试，不作为当前 100 Hz 双臂/VLA master 数据采集器。

### 3.5 每次开机的最短 SOP

```text
电脑开机
  → 检查急停、机械臂姿态、线缆和工作区
  → 打开 Pico 4，确认与电脑同一 Wi-Fi
  → 给机械臂/48 V 上电
  → 配置并确认 can0/can1 为 UP
  → 启动 bringup，确认 controllers active 和 /joint_states
  → 启动 OpenArmX VR，做小幅单臂方向测试
  → 启动 recorder，先录 1 集短数据
```

结束时反向执行：停止 recorder → 松 Grip/回零 → 停止 VR → 停止 bringup →
确认电机 disarmed → 关闭机械臂电源。任何异常优先急停/断电，再保存日志；
不要在运动中拔 CAN、USB 相机或电源线。

## 4. 迭代过程：做了什么、为什么做、解决了什么

### 4.0 早期脚本兼容问题：ROS logger 的参数格式

最初 recorder 启动即报：

```text
TypeError: RcutilsLogger.info() takes 2 positional arguments but 5 were given
```

原因是 ROS 2 的 `RcutilsLogger` 不完全等同于 Python 标准库 logger，不能把
`"%s"` 和多个替换参数作为独立位置参数传给 `info()`。修复方式是先用 f-string
或字符串拼接得到一个完整消息：

```python
# 错误
logger.info("topic=%s", topic)

# 正确
logger.info(f"topic={topic}")
```

这属于启动层 API 兼容问题，与控制频率、重力补偿和数据跳变无关；修好后再
进行时序排查，避免把启动异常误判为机械臂问题。

### 4.1 阶段一：Pico 4 + `teleop_xr`

**目的**：先验证 Pico 4 的 WebXR 输入和 OpenArm IK，而不是一开始就接
真实电机。

主要工作：

- 配置 `teleop_xr` WebXR 前端和 ROS 2 bridge。
- 补齐 OpenArm IK 的 `jax`、`pyroki`、`ballpark`、`yourdfpy` 等依赖。
- 验证官方 `OpenArmRobot` IK 注册和 `/joint_trajectory` 输出。
- Pico 某些固件缺少 `gripPose` 时，回退到普通 pose，避免手柄数据为空。
- 没有视频源时避免 `video_request` 生成无效 WebRTC offer。
- 增加 `--no-require-deadman` 作为仿真排查选项。

验证话题：

```bash
ros2 topic echo /xr/controller_left/pose --once
ros2 topic echo /xr/controller_right/pose --once
ros2 topic echo /xr/controller_left/joy --once
ros2 topic echo /joint_trajectory --once
```

`--no-require-deadman` 只适合 fake hardware/仿真。真实硬件必须恢复
deadman，否则追踪丢失时可能继续使用旧目标。

### 4.2 阶段二：OpenarmXR/OpenArmX UDP 路线

**目的**：使用 OpenArmX APK 的 UDP 输入和 Pinocchio IK，减少 WebXR 中间
层，并获得更稳定的实时控制频率。

```text
Pico 4 APK
  → UDP 5100
  → openarmx_teleop_bridge_vr_node
  → Pico pose/joy topics
  → Pinocchio IK
  → forward_position_controller
```

期间处理了 Pinocchio/NumPy 版本兼容和 OpenArm/OpenArmX 关节命名差异，使用
`joint_name_remapper` 让模型、IK 输出和 ROS 控制器名称一致，并准备了
fake/real launch。这个阶段成为后续真实双臂和录制的主基础。

IK 的基本过程是：先把 Pico 手柄在头部/身体参考系中的 pose 经过
`body_anchor_offset`、轴矩阵、位置比例和姿态矩阵转换到 OpenArm 基座坐标，
再用 Pinocchio 根据末端目标 pose 求满足关节范围的 `q_target`。控制节点不
直接把原始 VR pose 当电机命令，而是对 `q_target` 做滤波、限步和 deadman/超时
检查后才发布。这样“坐标标定错误”“IK 解跳变”和“网络丢帧”可以分别定位，
也是后面调参时必须保持分层的原因。

### 4.3 阶段三：fake hardware/RViz 验证

**目的**：在没有 CAN、电机和负载风险时，先验证完整链路和操作约定。

```text
Pico APK → UDP bridge → Pinocchio IK
         → fake ros2_control → RViz
```

典型启动：

```bash
ros2 launch openarmx_teleop_vr openarm_teleop_vr_sim.launch.py
```

这个仿真 launch 会把 fake hardware、`ros2_control`、左右 position controller、
robot state publisher、RViz、UDP bridge 和 IK 节点一起启动。若要单独复现早期
`teleop_xr` WebXR 路线，则使用独立的 fake bringup 和 bridge（不要和上面的
OpenArmX launch 同时启动）：

```bash
# 终端 A
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/ros2_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller

# 终端 B（~/OPENARM/teleop_xr）
python -m teleop_xr.ros2 \
  --host 0.0.0.0 --port 4443 \
  --input-mode CONTROLLER --mode ik \
  --robot-class openarm \
  --output-topic /joint_trajectory \
  --no-urdf-topic --no-require-deadman
```

在 Pico 浏览器打开 `https://<PC_IP>:4443`。这条路线的 `--no-require-deadman`
只用于仿真排查；真实设备必须启用 deadman。两条路线均可验证 IK，但只保留
一套 ROS publisher，避免 controller 争用。

验证内容：fake hardware、controller manager、左右 position controller、
robot state publisher、RViz、VR bridge 和 IK 节点是否都能同时工作。

操作约定（以当前映射为准，实际以日志/参数为准）：

```text
左 Grip  → 左臂 deadman/控制
右 Grip  → 右臂 deadman/控制
左右 Trigger → 对应夹爪
右 B     → 回零
左 Y     → 举手/预设动作
```

### 4.4 阶段四：真实 OpenArm 双臂 + CAN FD

**目的**：复用已经在 fake hardware 验证的 ROS 控制链，只替换底层硬件。

```text
OpenArmX VR → ROS position target
            → ros2_control
            → SocketCAN FD
            → 48 V OpenArm 电机
```

当前接线约定：右臂 `can0`，左臂 `can1`；CAN 仲裁速率 1 Mbps，数据段
5 Mbps，FD 开启。接线不同只改 launch 参数，不能凭左右名称猜接口。

实机必须先完成 CAN、控制器、`/joint_states`、单臂小动作和急停验证，
再打开双臂 VR。出现 `ERROR-PASSIVE`、`BUS-OFF`、反向运动、异响或持续
振荡时立即停止 bringup/断电，禁止继续试动作。

### 4.5 阶段五：遥操卡顿和参数迭代

**目的**：区分网络、IK、滤波、控制器频率和硬件负载，而不是盲目提高频率。

观察到的问题包括跟手延迟、Pico 抖动被放大、短暂 Wi-Fi 抖动后 idle，以及
把 ros2_control 提到 750 Hz 后反而更卡。最后验证出当前机器的稳定点是
VR IK 100 Hz + controller manager 100 Hz。

### 4.6 阶段六：模型重力补偿

**目的**：为持续负载提供模型前馈，改善抬臂/下放和静态保持的手感；它不
负责平滑 VR 目标，也不能修复 recorder 的 callback 阶梯。

当前独立链路：

```text
/joint_states
   → URDF/KDL 重力模型
   → gravity effort
   → forward_effort_controller / MIT tau_ff
   → 电机前馈
```

已做过左臂低比例（约 0.02）实机输出验证，双臂为安全起见推荐从 0.01 开始。

### 4.7 阶段七：接入 LeRobot

**目的**：把 VR 示教变成可复用的 state/action/episode 数据，而不让 LeRobot
在录制时抢走真实控制器。

自定义 recorder 只读：

```text
/joint_states
+ left/right arm commands
+ left/right gripper commands
         ↓
observation.state + action + task
```

双臂每帧 16 维（14 个 arm joint + 2 个 gripper），100 Hz、20 s episode 约
2000 帧。本地 `HF_HUB_OFFLINE=1` 便于在无网络环境采集。

### 4.8 阶段八：定位 3°～4° 动作跳变

**问题表现**：`ros2 topic hz` 显示约 100 Hz，数据时间戳也像 100 Hz，但 replay
出现 3°～4° 阶梯。

**旧实现**：主循环每 10 ms 只调用一次 `rclpy.spin_once()`，同时订阅
`/joint_states`、左右 arm、左右 gripper 五路高频消息。一次 `spin_once()`
通常只处理一个回调，其他消息排队；于是 snapshot 可能连续几帧不变，随后
一次跳过多个目标。

```text
发布者：100 Hz                    （topic hz 只能看到这一层）
录制器：每帧 spin_once 一次        （回调处理跟不上）
结果：时间戳 100 Hz，但值每数帧才刷新  （3°～4° staircase）
```

如果真实目标每 10 ms 变化约 0.8°～1°，四个采样周期没有被处理时，下一次
读取到的值会近似累积为 `4 × (0.8°～1°)`，所以看起来正好是 3°～4° 的跳变；
时间戳仍按录制循环每 10 ms 写入，因而只看 timestamp 或 `ros2 topic hz` 很难
发现它。

**修复**：ROS 订阅放到独立 `SingleThreadedExecutor` 后台线程持续 spin，主线程
固定 100 Hz 读取加锁 snapshot 并写盘。修复后的 v2 数据中各路 callback 接近
2000，`common_gap` 大多为 1，单周期最大变化约降到 1°。

同一时序原则也应用到了本地 `openarm_ros2_follower` 和
`pico_openarm_ik` adapter：ROS callback 在后台 executor 处理，LeRobot 的
控制/采样循环只读取最新快照。这样不能改变外部 publisher 的频率，但能避免
“主循环偶尔 spin 一次”造成的输入滞后。

**结论**：

- `ros2 topic hz` 测的是发布端，不等于 recorder 实际处理端。
- LeRobot/parquet/Rerun 没有改写 action；阶梯在进入数据集前已经形成。
- 录制器的 callback 计数和逐维差分必须纳入数据验收。
- 用旧版 recorder 录出的 episode 若已经出现这种阶梯，跳变已经写入数据；
  之后只调 replay 频率或 `gravity_scale` 不能恢复丢失的中间目标，应使用修复
  后的 recorder 重新采集。

### 4.9 阶段九：真实 replay 与 Rerun

**目的**：在真正发送动作前检查控制器、状态和首帧姿态，降低误启动风险。

replay 会：

1. 加载指定 episode 和源频率。
2. 检查左右 arm/gripper controller 是否有订阅者。
3. 检查双臂关节状态是否齐全。
4. 比较当前姿态和录制首帧，默认最大 arm 偏差 10°。
5. 等待人工确认后按目标频率发布。

Rerun 只记录/显示 `observation.state` 与 `action`，不补偿重力、不插值控制、
不提高频率。真实机械臂首次 replay 建议关闭；确认稳定后用每 10 或 20 帧
显示一次，避免 GUI 负载影响时序。

### 4.10 阶段十：D435i 全局相机

**目的**：把无视觉低维示教扩展为 VLA 所需的：

```text
observation.images.front + observation.state + action + task
```

D435i 固定在机械臂外部（eye-to-hand），当前第一阶段只采 RGB，不打开深度
和 IMU。相机由 LeRobot `pyrealsense2` 直接打开，不要同时启动另一个
`realsense2_camera` ROS 节点抢设备。

### 4.11 阶段十一：VLA 和仿真

**目的**：先在软件环境验证训练/推理闭环，再把策略送到真实电机。

RViz + fake hardware 只有关节状态可视化，不是带真实场景渲染的视觉仿真器。
真实 RealSense 拍 RViz 画面不能自动变成有意义的 simulated VLA 数据。推荐
三级验证：

1. synthetic video：只验证数据格式、训练和推理 plumbing。
2. `openarm_sim` renderer：图像随 fake joint state 改变，验证 observation。
3. 官方 `enactic/openarm_mujoco` + 本地 `openarm_mujoco_follower`：真实
   物理/渲染场景中的 rollout。

## 5. 当前稳定参数与设置理由

### 5.1 VR IK/指令平滑参数

配置文件：

```text
~/OPENARM/OpenarmXR/openarmx_teleop_vr/openarmx_teleop_vr/config/teleop_params.yaml
```

注意：`openarmx_teleop_vr_node.py` 代码里的声明默认值仍包含早期的
`stream_timeout_sec=0.3`、`command_filter_alpha=0.18` 和较大的 max-step；
表中的稳定值来自 launch 加载的 `teleop_params.yaml`。因此应优先使用
`openarm_teleop_vr_sim/real.launch.py`，不要裸跑可执行文件后误以为已经使用
了稳定参数。若必须裸跑，需显式传入同一组参数，并在日志中核对最终值。

| 参数 | 当前值 | 含义和选择理由 |
| --- | ---: | --- |
| `control_rate` | `100.0` Hz | 与 Pico/ROS/CAN 的验证频率一致；10 ms 一个目标，延迟和计算量平衡 |
| `grip_threshold` | `0.5` | 只有明确握紧才接管对应手臂，防止追踪到但未授权时运动 |
| `stream_timeout_sec` | `1.0` s | 原 `0.3` s 对短暂 Wi-Fi 抖动过敏；1 s 减少误入 idle，但 deadman 和急停仍必须可用 |
| `command_filter_alpha` | `0.5` | 一阶滤波折中；0.18 明显滞后，1.0 会把 Pico 微抖直接传给机械臂 |
| `command_reset_timeout_sec` | `0.5` s | 长时间未更新后从当前测量姿态重新初始化滤波器，避免恢复时大跳 |
| `max_step_deg_joint1..7` | `0.8,0.8,0.8,1.0,1.0,0.8,0.8` | 普通模式单周期限步，抑制 IK/网络瞬时异常；100 Hz 下约对应 80～100°/s 上限，实际通常更低 |
| `fast_max_step_deg_joint1..7` | `1.5,1.5,1.5,1.8,1.8,1.2,1.2` | 快速/按钮动作允许更快，但仍有上限；不能恢复到未限速的 10～20°/周期 |
| `step_limit_enable_threshold_deg_joint1..7` | 全部 `0` | 对每个非零周期差都限步；避免 3～4° 目标一次穿透 |
| `button_motion_step_deg` | `0.5` | 回零/预设按钮动作的独立小步长，降低突然跳变 |
| `button_motion_done_tolerance_deg` | `1.0` | 预设动作在 1° 内结束，避免无限微调 |
| `position_scale_xyz` | `[1.0,1.0,0.9]` | z 方向略缩放，降低上下运动过敏；需按个人身高/安装姿态重新校准 |
| `body_anchor_offset` | `[0,-0.18,0.08]` | 将头部/身体参考系转换到机械臂工作空间；改变安装位置后需重标定 |
| `left_axis_matrix` / `right_axis_matrix` | 当前两侧同为 `[0,0,-1,-1,0,0,0,1,0]` | 将 Pico 坐标轴变换到 OpenArm 坐标；方向错误时只改对应侧并在 fake hardware 验证 |
| `left/right_orientation_matrix` | 当前为 3×3 identity | 保留手柄姿态到末端姿态的标定；机械安装改变后再重新求矩阵 |

滤波和限步的实际顺序可理解为：

```text
q_target (IK)
  → q_filtered = q_last + alpha * (q_target - q_last)
  → clip(q_filtered - q_last, ±max_step)
  → publish q_command
```

提高 `alpha` 或 `max_step` 会更跟手，也会提高抖动和异常目标的风险；降低它们
会更稳，但带来滞后。每次只改一个参数，并在 fake hardware 和小幅实机上比较。

### 5.2 频率分层

| 层 | 推荐频率 | 原因 |
| --- | ---: | --- |
| Pico/VR pose 输入 | 约 100 Hz | 保留手柄连续运动和低延迟 |
| Pinocchio IK/command | 100 Hz | 与限步单位（每周期角度）匹配 |
| `controller_manager.update_rate` | 100 Hz | 当前 CPU/CAN 环境稳定；750 Hz 实测更卡 |
| D435i RGB | 30 FPS | VLA 常用图像频率，USB/编码负载可控 |
| LeRobot 无视觉采样 | 100 Hz | 保留原始动作细节，便于排查和后处理 |
| LeRobot 视觉数据 | 30 Hz | 与图像帧率匹配，避免同一图像重复多次 |
| VLA 推理 | 约 15～30 Hz | 由模型大小、GPU 和实时项目决定 |

`HF_HUB_OFFLINE=1` 只控制 Hugging Face 网络访问；它不会改变 ROS 或相机频率，
也不会自动把数据上传到 Hub。当前阶段使用本地 `local/...` repo/root，先
保证可重复和不依赖网络。

100 Hz 的硬件执行层可以在两次策略输出之间保持最近目标、线性插值或执行
action chunk。不要只把 metadata 的 `fps` 改成 30；真正降采样时要重采样
state/action，并记录时间关系。

例如 action chunk：

```text
30 Hz 策略 + chunk_size=10 → 约 0.33 s 的动作块
100 Hz 策略 + chunk_size=10 → 约 0.10 s 的动作块
```

对于当前项目，建议保存 100 Hz 原始无视觉数据作为 master，再生成 30 Hz
视觉训练副本；这样既保留控制细节，又符合 VLA 输入频率。

### 5.3 Bringup、CAN 和控制器参数

常用参数及理由：

| 参数 | 推荐值 | 理由 |
| --- | --- | --- |
| `use_fake_hardware` | 仿真 `true`；实机 `false` | 明确隔离软件验证和电机风险 |
| `robot_controller` | `forward_position_controller` | 与当前 OpenArmX direct-topic 输出一致 |
| `right_can_interface` | `can0` | 当前接线约定；必须按实物确认 |
| `left_can_interface` | `can1` | 当前接线约定；必须按实物确认 |
| `controller_manager.update_rate` | `100` | 当前系统的稳定点，不追求纸面高频 |
| `enable_effort_controllers` | 默认 `false` | 只在重力/力矩调试需要时启用，减少无关控制器 |

CAN FD 示例（接口名和速率必须与硬件手册一致）：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 \
  dbitrate 5000000 fd on restart-ms 100
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 \
  dbitrate 5000000 fd on restart-ms 100
sudo ip link set can1 up

ip -details link show can0
ip -details link show can1
```

期望看到 `state UP`、FD、1 Mbps arbitration 和 5 Mbps data。若总线进入
`ERROR-PASSIVE`/`BUS-OFF`，先断开动作源并处理物理连接，不要靠重启 recorder
掩盖问题。

### 5.4 底层 MIT 位置环：不要把所有抖动都归因于 VR

当前 `openarm_simple_hardware` 还会把每个关节的 position、velocity 和
effort command 组合成 Damiao MIT 参数后发送。工作区中可见的当前基础增益是：

```text
Kp = [70, 70, 70, 60, 10, 10, 10]
Kd = [2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5]
```

它们决定“同一个目标下电机如何追踪”，而不是 VR 的 `command_filter_alpha`。
Kp/Kd 过高可能放大编码器噪声、机械回差和目标跳变，过低则跟踪软、下坠或
滞后；不同负载和关节姿态不能直接套用一组经验值。排查时先固定 VR 目标、
关闭 gravity/Rerun，确认硬件环本身，再在小范围内一次只改一个增益。不要在
数据采集期间随意改增益，否则不同 episode 的动力学分布会不一致，必须记录
增益版本。

## 6. 重力补偿：原理、开关和安全调参

### 6.1 数学模型

节点从 URDF 的质量、质心、惯量和关节链构建 KDL 动力学模型，根据当前
关节角 `q` 计算抵抗重力的广义力：

```text
g(q) = KDL.JntToGravity(q)
tau_ff = clamp(gravity_scale * g(q), -torque_limits, +torque_limits)
```

位置/阻尼环的概念形式为：

```text
tau = Kp * (q_desired - q)
    + Kd * (0 - q_dot)
    + tau_ff
```

实际 `Kp/Kd` 和底层 MIT/CAN 实现由 OpenArm 驱动器决定；重力节点只是计算
并发布前馈。机械臂“能停住”主要可能来自位置环 Kp，不能据此断定重力补偿
已经生效。

### 6.2 两个布尔开关的区别

在 `openarm.bimanual.launch.py` 中：

- `enable_gravity_compensation:=true`：启动重力计算节点（可只计算/观察）。
- `apply_gravity_compensation:=true`：把计算出的 effort 发给 active effort
  controller，进入 live 前馈。
- 只有计算没有 apply 时，不应把它称作“已经施加补偿”。

推荐先 dry-run/观察，再 live；第一次真实双臂先无补偿验证采集和 replay。

### 6.3 推荐初始值

```text
gravity_scale:          0.01
gravity_ramp_duration_s: 10.0
torque_limits (Nm):      [2.0, 2.0, 2.0, 2.0, 0.5, 0.5, 0.5]
publish_rate_hz:         100
state_timeout_s:         0.1
```

`gravity_scale=0.5` 表示使用模型力矩的 50%，不是简单的“抖动减半”。模型
误差、零位、摩擦、负载和符号错误都会使 0.5 产生过补偿、抬升、振荡或发热。
建议逐级 `0.01 → 0.02 → 0.05`，每级只做短时间、低速、无负载测试。

上表的 `torque_limits` 是当前 bringup launch 中的调试/投运限制
`[2,2,2,2,0.5,0.5,0.5]` Nm；gravity node 源码的通用默认值更高，不能把
源码默认值当作实机 commissioning 上限。

启动示例：

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=false \
  robot_controller:=forward_position_controller \
  right_can_interface:=can0 \
  left_can_interface:=can1 \
  enable_gravity_compensation:=true \
  apply_gravity_compensation:=true \
  gravity_scale:=0.01 \
  gravity_ramp_duration_s:=10.0 \
  gravity_enable_left:=true \
  gravity_enable_right:=true
```

验证：

```bash
ros2 control list_controllers
ros2 topic echo /gravity_compensation/left_effort --once
ros2 topic echo /gravity_compensation/right_effort --once
ros2 topic echo /left_forward_effort_controller/commands --once
ros2 topic echo /right_forward_effort_controller/commands --once
ros2 param get /openarm_gravity_comp gravity_scale
ros2 param get /openarm_gravity_comp apply_effort
```

若出现突然抬升、反向、振荡、噪声或电机明显发热，立刻停止并回到：

```text
enable_gravity_compensation:=false
apply_gravity_compensation:=false
```

### 6.4 录制时有没有重力补偿？

录制器本身没有重力补偿代码。录制时是否存在补偿，完全取决于同时运行的
bringup 参数和 gravity node：

```text
bringup 开启并 apply=true → 实际机械臂可能带前馈
bringup 关闭或 apply=false → recorder 仍能录，但没有该前馈
```

补偿改变的是机械臂实际受力和 `observation.state` 的运动结果，不会把
`action` 目标自动平滑。建议在数据记录元数据/实验日志中标明
`gravity_scale`，不要无标签地混合不同动力学条件的数据。当前 recorder 的
16D feature 不包含 effort/torque 通道；如果以后要让策略显式利用力矩，需要
另加带单位、限幅和同步定义的 observation feature，而不是把 gravity effort
误写进现有 action 向量。

## 7. LeRobot 数据采集与数据结构

### 7.1 无视觉 recorder 的设计

文件：`record_openarmx_bimanual_lerobot.py`。

设计原则：

- 只读 ROS 话题，VR teleop 仍是唯一动作发布者。
- 后台 `SingleThreadedExecutor` 持续处理 ROS callback。
- 主线程按 `--fps` 定时读取最新 snapshot。
- arm 关节 rad→deg；夹爪 m↔-65° 约定统一转换。
- 每个 episode 显式保存，结束后 finalize。

默认特征：

```text
observation.state: float32[16]
action:            float32[16]
task:              字符串
robot_type:        bi_openarm_ros2_follower
```

启用相机后，recorder 会增加 `observation.images.front`（LeRobot `video`
feature；示例默认用 RGB/H.264 MP4，也可按 `--vcodec` 选择其他编码），不会
改变 16D state/action 的左右顺序。

这里 `observation.state` 是 `/joint_states` 的实际测量位置，`action` 是
VR/重播控制器已经生成并发布的目标位置；两者有意分开。这样训练可以学习
“目标如何导致实际状态变化”，也能从 state-action 差异发现延迟、限步、负载
或跟踪误差。若把两者都写成同一个实测值，会丢掉控制目标信息；若把状态误当
成 action，则 replay 和策略监督会失真。

20 s × 100 Hz 理论上是 2000 帧；线程调度可能造成 ±1～数帧差异，但不应
出现系统性少帧或 callback 计数大幅低于 2000。

### 7.2 从开机到开始录制：真实双臂无视觉

#### 终端 0：安全和 CAN

确认急停、机械臂周边、供电、线缆和左右 CAN 映射；必要时执行上面的 CAN
配置命令。第一次只做空载、小幅度动作。

#### 终端 1：真实 bringup（第一次建议无重力补偿）

```bash
conda activate oplerobot310
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=false \
  robot_controller:=forward_position_controller \
  right_can_interface:=can0 \
  left_can_interface:=can1 \
  enable_gravity_compensation:=false \
  apply_gravity_compensation:=false
```

#### 终端 1：bringup 检查

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic info -v /left_forward_position_controller/commands
ros2 topic info -v /right_forward_position_controller/commands
ros2 topic info -v /left_gripper_controller/joint_trajectory
ros2 topic info -v /right_gripper_controller/joint_trajectory
```

至少确认 `joint_state_broadcaster`、左右 position controller、左右 gripper
controller 为 `active`，关节名包括：

```text
openarm_left_joint1..7, openarm_left_finger_joint1
openarm_right_joint1..7, openarm_right_finger_joint1
```

#### 终端 2：OpenArmX VR

```bash
unset PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

ros2 launch openarmx_teleop_vr openarm_teleop_vr_real.launch.py
```

该 launch 会启动/连接 OpenArmX 的 VR bridge、关节名 remapper 和 Pinocchio
teleop 节点；不需要再手动启动另一份同名 teleop。电脑 IP 可用下面命令查看，
并在 Pico APK 中填入同一局域网地址和 UDP 端口：

```bash
ip -4 addr show
```

```text
ROS PC IP Address: <电脑 Wi-Fi IPv4>
UDP Port: 5100
```

Pico 和电脑必须在同一局域网，优先 5 GHz Wi-Fi；APK 保持前台，避免系统
休眠/电池优化。终端应能看到 bridge 正在监听 `0.0.0.0:5100`，teleop 日志
应显示约 `100.0 Hz`。

检查日志应为约 100 Hz；连接 Pico 后先只做小幅运动，确认左右方向、Grip
deadman 和 Trigger 夹爪映射。松开 Grip 后应停止跟随/保持，而不是继续追旧位姿。

开始录制前可在另一个只读终端确认输入和输出频率：

```bash
ros2 topic list | rg 'pico|vr|pose' || true
timeout 15s ros2 topic hz /pico_left_controller/pose
timeout 15s ros2 topic hz /pico_right_controller/pose
timeout 15s ros2 topic hz /left_forward_position_controller/commands
timeout 15s ros2 topic hz /right_forward_position_controller/commands
```

这些命令测的是各 topic 的发布端频率；它们不能替代 recorder 自身的 callback
计数和逐帧数据检查。

#### 终端 3：运行 100 Hz 无视觉采集

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_HUB_OFFLINE=1

python examples/openarm_xr/record_openarmx_bimanual_lerobot.py \
  --repo-id local/openarmx_bimanual_novision_100hz_v2 \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz_v2 \
  --task "Move the object with both OpenArms using Pico 4 VR" \
  --fps 100 \
  --episodes 1 \
  --episode-time-s 20
```

程序会等待关节状态和回车。正常输出约为：

```text
Saved episode 1: 2000 frames
ROS callbacks during episode:
  joint_states≈2000, left_arm≈2000, right_arm≈2000
```

夹爪 callback 可能少于 2000，因为夹爪只有位置变化时才发布；arm 和
joint_states 明显偏低则应废弃该 episode。

这个自定义 recorder 的 episode 控制是“每集回车开始”；它不同于标准
`lerobot-record` 的右箭头保存、左箭头重录、Esc 退出快捷键。不要把两套按键
说明混用，也不要为了获得标准快捷键而在当前 direct-topic 实机上额外启动
标准 recorder。

> 当前实现对录制中途 `Ctrl-C` 的“正在进行 episode”保存能力有限：如果中断
> 发生在 `record_episode()` 内部，可能尚未把 `pending_frames` 赋值。为了保留
> 数据，优先让当前 episode 正常结束，再停止程序；若必须中断，检查数据目录
> 是否真的生成了 episode，不要仅凭终端的 `Stopped by user` 判断。

### 7.3 离线检查 metadata

```bash
cd ~/OPENARM/lerobot
python - <<'PY'
import json
from pathlib import Path

root = Path("data/openarmx_bimanual_novision_100hz_v2")
info = json.loads((root / "meta/info.json").read_text())
print("fps:", info["fps"])
print("episodes:", info["total_episodes"])
print("frames:", info["total_frames"])
print("robot_type:", info["robot_type"])
print("action names:", info["features"]["action"]["names"])
PY
```

当前已验证的一份 v2 数据：`fps=100`、`episodes=1`、`frames=2000`、
`robot_type=bi_openarm_ros2_follower`，动作名为左 8 维加右 8 维。

### 7.4 callback 和连续性验收

验收不只看 `fps`：

1. 记录器输出的各路 callback 计数接近 episode 预期值。
2. 每个 action/state 维度均为有限值，无 NaN/Inf。
3. 相邻样本差分没有大量完全相同的 4 帧块。
4. `common_gap` 大多数为 1；若普遍为 4，先修 callback/负载，不要训练。
5. 目标 action 和实测 state 的延迟、方向和单位合理。

可直接对 parquet 做一个最小的逐帧差分检查（这里只判断数值连续性，不替代
完整的数据集统计）：

```bash
cd ~/OPENARM/lerobot
python - <<'PY'
from pathlib import Path
import numpy as np
import pyarrow.parquet as pq

p = Path("data/openarmx_bimanual_novision_100hz_v2/data/chunk-000/file-000.parquet")
table = pq.read_table(p)
action = np.asarray(table["action"].to_pylist(), dtype=np.float32)
delta = np.abs(np.diff(action, axis=0))
print("frames:", len(action), "dims:", action.shape[1])
print("max step per dim:", np.max(delta, axis=0))
print("global p99 step:", np.percentile(delta, 99))
print("rows with any step > 3 deg:", int(np.sum(np.max(delta, axis=1) > 3.0)))
PY
```

如果连续很多行完全不变、随后出现约 3～4° 的单行跳变，优先回到 recorder
callback/executor 排查；不要先用后处理滤波把问题遮住。

### 7.5 Recorder 参数速查

| 参数 | 当前/示例值 | 作用与理由 |
| --- | --- | --- |
| `--repo-id` | `local/...` | 数据集逻辑名称；不同 fps/视觉特征必须使用新名称 |
| `--root` | `~/OPENARM/lerobot/data/...` | 本地实际落盘目录；便于离线检查和 replay |
| `--task` | 固定任务句子 | VLA 的语言条件；同一批数据应保持语义一致 |
| `--fps` | 无视觉 `100`；视觉 `30` | 数据采样时钟；必须和真实写入频率及视觉帧率匹配 |
| `--episodes` | 先 `1`，再逐步增加 | 先用短集验收，避免一次生成大量坏数据 |
| `--episode-time-s` | 先 `10`～`20` s | 控制单集长度，便于回零、筛选和重录 |
| `--connect-timeout-s` | `10` s | 等待完整 `/joint_states`；超时说明 bringup/topic 未就绪 |
| `--joint-state-topic` | `/joint_states` | 实测状态来源；命名空间变化时显式覆盖 |
| `--left/right-command-topic` | 左右 `forward_position_controller/commands` | 当前 OpenArmX direct-topic 动作来源 |
| `--left/right-gripper-topic` | 左右 `joint_trajectory` | 夹爪独立于 7D arm array |
| `--camera-serial-number-or-name` | D435i 实际 SN | 只在视觉集启用；避免误打开另一台相机 |
| `--camera-width/height/fps` | `640/480/30` | 与相机 profile 对齐；先稳定再降分辨率 |
| `--camera-warmup-s` | `2` s | 给曝光和后台采集线程预热 |
| `--vcodec` | `h264` | 兼容性、体积、编码速度折中 |
| `--streaming-encoding` | 默认关闭 | 保存时再编码更稳；仅在保存太慢且 CPU 足够时打开 |

视觉数据不能追加到已经创建的无视觉 100 Hz dataset：`features`、视频编码
和 `fps` 不同。请新建类似 `local/openarmx_bimanual_vla30_test` 的 repo/root，
保留 `*_novision_100hz` 作为原始控制和 replay 数据。

## 8. D435i 全局相机：检测、配置和视觉采集

### 8.1 相机模式和频率选择

相机固定在机械臂外部，键名固定为 `observation.images.front`。第一阶段只
使用 RGB：`use_depth=false`、`ColorMode.RGB`、不启用 IMU。这样能先验证
VLA 必需的图像链路，避免深度/同步/带宽问题同时出现。

物理安装建议把 D435i 固定在机架或三脚架上（eye-to-hand），一次调整好
俯仰、水平和视场后不要在一批数据中移动。画面应同时覆盖左右末端、工作台、
物体和目标区域，避免夹具、桌沿或线缆遮挡；先用 finder 输出的 PNG 人工确认
取景，再开始录制。相机位姿、曝光、光照和遮挡是数据的一部分，换位置后不要
把新旧数据直接混在同一训练 split 中，除非显式记录相机版本并做校准。

用户当前检测结果（2026-08-31）：

```text
Name: Intel RealSense D435I
Serial: 943222072197
Firmware: 5.16.0.1
USB descriptor: 3.2
Default RGB profile: 1280x720 @ 30 FPS
```

这次结果说明 finder 已识别到 USB 3.2 设备，并列出了默认
`1280×720@30 FPS` profile。此前的 USB 2/profile 风险已经基本排除。finder 打印的
默认 profile 与 recorder 主动请求的 profile 可以不同；当前仍建议先用
`640×480@30`，降低编码、磁盘和训练显存负载。你随后用 LeRobot
`RealSenseCamera` 主动请求 `640×480@30`，成功读取了 `(480, 640, 3)`、
`uint8` RGB 帧，说明该 profile 可用；正式采集前还应做连续 60 帧测试，确认
没有掉帧或 stale frame。

注意：`USB descriptor: 3.2` 是设备/链路能力提示，`lsusb -t` 中 D435i
对应设备行的协商速度显示为 `5000M` 才是最终 USB 3 带宽确认。若以后再次
看到 USB 2 结果，先运行：

```bash
lsusb -t
```

目标看到连接速度 `5000M`（USB 3）。若看到 `480M`/USB 2：更换 USB 3 数据线、
直连主板 USB 3 口、去掉 Hub/延长线，再重新检测。USB 2 下 640×480@30 可能
 协商失败或不稳定，不能把默认 15 FPS 当作最终 VLA 配置。

设备物理路径中的 `/usb2/2-2` 是 Linux 的 USB bus 编号，不代表 USB 2.0
速率；本次 `lsusb -t` 已看到该设备接口为 `5000M`，因此应按 USB 3 链路处理。

finder 显示的 `rgb8` 对应 recorder 期望的 NumPy 图像形状
`(height, width, channels)`，本例为 `(480, 640, 3)`。深度流即使 D435i 支持，
也不会自动出现在当前 `observation.images.front`；启用深度前需单独设计
feature、同步和训练输入。

LeRobot 原生 RealSense 依赖 `pyrealsense2`。项目约束偏向 `<2.57.0`
（常用 2.56.x）；如果当前 RGB 读取正常，不要为版本警告贸然改环境。只有
profile/API 异常时才锁定兼容版本并重新测试。

可先查看当前版本：

```bash
python -c 'import pyrealsense2 as rs; print(getattr(rs, "__version__", "unknown"))'
```

当前环境曾见 `2.57.7.10387`，高于项目约束；这本身不是 RGB 必然失败的
证据。只有在独立读帧/profile 确实异常时，才考虑在有网络和备份环境的前提下
锁定兼容版本，例如：

```bash
python -m pip install --force-reinstall 'pyrealsense2==2.56.5.9235'
```

安装后必须重新执行 finder 和 60 帧读帧测试，不能只看 pip 成功信息。

### 8.2 相机发现命令

不启动机械臂和 VR，先独立确认设备：

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

rs-enumerate-devices -s
rs-enumerate-devices -d
lsusb | grep -i -E 'intel|realsense|8086'
lsusb -t

python -m lerobot.scripts.lerobot_find_cameras realsense \
  --record-time-s 3 \
  --output-dir /tmp/d435i_check
```

检查 serial、USB、profile，并确认 `/tmp/d435i_check/` 中有非黑图像。`lsusb -t`
要看 D435i 所在的嵌套设备行：该行应为 `5000M`，不能只看根 hub 的速率。
不要
同时启动 `realsense2_camera` ROS 节点、`realsense-viewer` 或其他直接打开
相机的程序，因为当前 recorder 通过 `pyrealsense2` 直接占用设备。

finder 保存的 PNG 只是设备诊断截图，不是 LeRobot 视频数据；如果命令没有
正确解析 `--output-dir`，默认目录可能显示为 `outputs/captured_images`。正式
录制时应以 recorder 的 `--root` 为准，视频会写入该数据集的 `videos/` 目录。

如果 `lsusb` 能看到 Intel 设备但 finder 报无设备，再排查权限/udev，而不是
先改 recorder：

```bash
lsusb | grep -i -E 'intel|realsense|8086'
groups
ls -l /dev/bus/usb/*/* 2>/dev/null | head
pgrep -af 'realsense|rs-' || true
ls -l /dev/video* 2>/dev/null || true
```

必要时按 librealsense 的 udev 规则安装说明处理，并重新插拔相机；这一步需
管理员权限，完成后仍要重新做 finder/读帧测试。

若 finder 成功，再用下面的独立读帧测试确认“请求 30 FPS”确实得到了约 30
FPS，而不是仅仅打印了配置值：

```bash
python - <<'PY'
import time
from lerobot.cameras import ColorMode, Cv2Rotation
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

camera = RealSenseCamera(RealSenseCameraConfig(
    serial_number_or_name="943222072197",
    width=640, height=480, fps=30,
    color_mode=ColorMode.RGB, use_depth=False,
    rotation=Cv2Rotation.NO_ROTATION, warmup_s=2,
))
connected = False
try:
    camera.connect()
    connected = True
    # 丢弃几帧预热数据，再测量持续读取时间。
    for _ in range(5):
        camera.read()
    start = time.perf_counter()
    shape = None
    for _ in range(60):
        shape = camera.read().shape
    elapsed = time.perf_counter() - start
    print("shape:", shape, "measured_fps:", round(60 / elapsed, 2))
finally:
    if connected:
        camera.disconnect()
PY
```

预期 `shape=(480, 640, 3)`、实测接近 30；若约 15 或频繁超时，先处理 USB
链路/profile，再开始正式采集。序列号应替换为 finder 输出的实际 `Id`。

### 8.3 先录一个短视觉 episode

机器人 bringup 和 VR 与无视觉录制相同，只把终端 3 改为：

```bash
python examples/openarm_xr/record_openarmx_bimanual_lerobot.py \
  --repo-id local/openarmx_bimanual_vla30_test \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_vla30_test \
  --task "Move the object with both OpenArms using Pico 4 VR" \
  --fps 30 \
  --episodes 1 \
  --episode-time-s 10 \
  --camera-serial-number-or-name "943222072197" \
  --camera-width 640 \
  --camera-height 480 \
  --camera-fps 30 \
  --camera-warmup-s 2 \
  --vcodec h264
```

参数理由：

- `--fps 30`：让 LeRobot state/action 采样与 RGB 帧率一致；VR/机械臂仍为
  100 Hz。
- `640×480`：先使用已经通过单帧读取验证的较低负载 profile，便于排错；稳定
  后可根据目标细节评估 `1280×720`，或使用更低分辨率（如 424×240）降低
  编码和训练显存。
- `--camera-fps 30`：目标 VLA 图像频率；USB 2/profile 不稳定时暂时改为
  15，并把数据集 `--fps` 也改为 15。
- `--camera-warmup-s 2`：等待自动曝光和后台采集线程稳定，减少开头黑帧。
- `--vcodec h264`：兼容性和体积折中；保存慢时可试 `--streaming-encoding`，
  若出现 `Encoder queue full` 则去掉该选项。

短集应约 300 帧，并包含：

```text
observation.state
observation.images.front
action
fps: 30
```

保存后检查目录和视频文件：

```bash
cat ~/OPENARM/lerobot/data/openarmx_bimanual_vla30_test/meta/info.json
find ~/OPENARM/lerobot/data/openarmx_bimanual_vla30_test/videos \
  -type f -name '*.mp4' -ls
```

如果没有 MP4、视频帧数明显少于约 300，或视频黑屏/冻结，先不要录更多集；
检查 USB/profile、编码日志和相机是否被其他进程占用。

当前 `read_latest()` 是非阻塞读取最新帧，没有硬件时间戳严格同步；机器人
状态和图像最多约有一帧（约 33 ms）偏差，第一版 VLA 数据通常可接受。若以
`--fps 100` 同时录相机 30 FPS，则同一图像会被重复约 3～4 次；这种数据只
适合保留 100 Hz 原始 state/action，训练前应明确降采样/去重，不建议作为
首选视觉数据格式。当前读取器对超过约 500 ms 的过期帧会报错/中止当前采集，
所以视觉集出现中断时先排查 USB、带宽和编码，不要简单延长 stale timeout 来
掩盖丢帧。

## 9. 从录制到真实 replay

### 9.1 Replay 前的互斥和安全检查

1. 在 VR 终端按 `Ctrl-C`，确认 `openarm_teleop_vr_real.launch.py` 已停止。
2. 保持 bringup 和控制器运行；不要同时启动另一套 teleop。
3. 清空机械臂周围，确认急停和断电路径。
4. 检查当前姿态接近录制首帧；默认首帧 arm 偏差上限为 10°。
5. 首次只 replay 一段短、低幅度、无负载 episode。

### 9.2 无 Rerun 的首次 replay

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_HUB_OFFLINE=1

python examples/openarm_xr/replay_openarmx_bimanual_lerobot.py \
  --repo-id local/openarmx_bimanual_novision_100hz_v2 \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz_v2 \
  --episode 0 \
  --fps 100 \
  --confirm-real
```

脚本把 LeRobot 的 deg arm action 转回 rad，把 gripper deg 转回约 0～0.044 m，
以 100 Hz 发布。`--fps` 与数据源不同也可以运行，脚本会线性插值 arm/action；
但训练/验收时应优先保持源频率一致。

### 9.3 打开 Rerun（只观察）

确认无 GUI replay 平稳后再执行：

```bash
python examples/openarm_xr/replay_openarmx_bimanual_lerobot.py \
  --repo-id local/openarmx_bimanual_novision_100hz_v2 \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz_v2 \
  --episode 0 \
  --fps 100 \
  --confirm-real \
  --display-data \
  --display-every 10
```

`--display-every 20` 更省负载。Rerun 中看 `observation.state` 和 `action`；
不把 Rerun 的曲线当作控制反馈，也不因为看见曲线就认为重力补偿已开启。
当前 replay 脚本的 `--display-data` 只记录 state/action，不读取
`observation.images.front`；要看 D435i 视频和每帧关节曲线，请使用下面的
`lerobot-dataset-viz` 离线 viewer。

Rerun 对一维向量通常按索引显示（例如 `action_0`、`observation.state_7`），
不是自动把每条曲线改成关节名。按当前 recorder 顺序，索引映射为：

```text
0..6   left_joint_1..7.pos
7      left_gripper.pos
8..14  right_joint_1..7.pos
15     right_gripper.pos
```

因此查看曲线时应结合 `meta/info.json` 的 `features.action.names`，不要仅凭
曲线编号判断左右臂。

### 9.4 离线查看 LeRobot 数据集

如果想看到 LeRobot 的标准界面（每个关节的 action/state 曲线，视觉集还
包括 `front` 图像），建议在 replay 完成后单独打开 dataset viewer，不要让
viewer 和真实控制循环共用同一实时进程：

首次使用且缺少 Rerun 依赖时，在当前环境安装一次：

```bash
python -m pip install -e '.[dataset_viz]'
```

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

lerobot-dataset-viz \
  --repo-id local/openarmx_bimanual_novision_100hz_v2 \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz_v2 \
  --episode-index 0
```

若没有 entrypoint，可使用等价的模块命令：

```bash
python -m lerobot.scripts.lerobot_dataset_viz \
  --repo-id local/openarmx_bimanual_novision_100hz_v2 \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz_v2 \
  --episode-index 0
```

需要把 `.rrd` 文件带到另一台电脑时：

```bash
lerobot-dataset-viz \
  --repo-id local/openarmx_bimanual_novision_100hz_v2 \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz_v2 \
  --episode-index 0 \
  --save 1 \
  --output-dir /tmp/openarmx_rrd
rerun /tmp/openarmx_rrd/local_openarmx_bimanual_novision_100hz_v2_episode_0.rrd
```

`lerobot-dataset-viz` 的曲线和图像是离线观察结果；它不向 ROS controller
发布命令。首次真实 replay 仍建议“无 Rerun replay → 离线 viewer”，确认稳定
后再在 replay 中加 `--display-data`。

如果 viewer 报 Hugging Face cache 的锁/写权限错误，可把可写缓存指向临时目录：

```bash
mkdir -p /tmp/hf_datasets_cache
export HF_DATASETS_CACHE=/tmp/hf_datasets_cache
```

### 9.5 Replay 参数速查

| 参数 | 推荐值/含义 | 为什么这样设 |
| --- | --- | --- |
| `--repo-id`, `--root` | 与已验收数据完全一致 | 防止 replay 读错 episode 或动作顺序 |
| `--episode` | `0` 起，先短集 | LeRobot episode index 从 0 开始；先验证一集 |
| `--fps` | 与源数据同为 `100` | 避免额外重采样；不同频率只在明确需要时使用 |
| `--confirm-real` | 必须显式给出 | 默认拒绝向真实机器人发布，作为安全门 |
| `--yes` | 首次不要给 | 保留人工回车确认和最后一次安全检查 |
| `--max-initial-delta-deg` | 默认 `10`° | 首帧偏差过大时拒绝；不要为了启动而盲目放宽 |
| `--connect-timeout-s` | 默认 `5` s | 等待关节状态和 controller subscriber |
| `--display-data` | 首次关闭 | Rerun 只观察且会增加 GUI/CPU 负载 |
| `--display-every` | `10` 或 `20` | 降低显示开销；不改变实际发布频率 |

### 9.6 正常结束和紧急停止

正常结束：松开 Grip → 必要时回零 → 停止 VR → 停止 recorder/replay → 停止
bringup → 确认电机 disarmed 后断电。

紧急情况（突然跳动、反向、异响、持续振荡、Pico 长时间断流、CAN 错误）：
立即 Ctrl-C 停止动作源；无法确认安全时直接使用急停/断电。不要先继续收集
数据，也不要通过提高 `max-initial-delta` 或 `gravity_scale`“硬顶过去”。

## 10. 为什么会卡、抖和跳变：排查顺序

| 现象 | 首要原因 | 检查/处理 |
| --- | --- | --- |
| 目标跟手但 Pico 抖动明显 | `alpha=1` 或限步太大 | 回到 `alpha=0.5`，保持逐周期 max step |
| 机械臂明显滞后 | `alpha=0.18` 或限步太小 | 先确认回调/网络，再小步提高 alpha；不要直接去掉限步 |
| Wi-Fi 短抖后进入 idle | timeout 太短、Pico 休眠/丢追踪 | 检查 APK/5 GHz/Wi-Fi；当前 timeout 1.0 s；deadman 不可关闭 |
| `ros2 topic hz` 100 Hz 但 replay 阶梯 | recorder callback 堆积 | 使用后台 executor 版本；看 callback 计数和逐维差分 |
| replay 3°～4° 一跳 | 使用旧数据或多发布者竞争 | 换 v2 数据，停止 VR，`ros2 topic info -v` 查 publisher |
| 固定目标仍抖 | 位置环增益、机械回差、CAN 或控制器竞争 | 无 VR/无 Rerun 发送固定小目标做隔离测试 |
| gravity 开启后抬升/振荡 | scale 过高、符号/URDF/零位误差 | 立即关补偿；从 0.01 + 10 s ramp 重测 |
| 重力补偿“没效果” | 只 enable 未 apply，或 torque 被 clamp | 查 `apply_effort`、effort topic、active effort controller |
| controller 无 subscriber | 控制器未 active 或 topic 不匹配 | `ros2 control list_controllers`、`ros2 topic info -v` |
| 相机找不到/profile 失败 | USB 2、线材/Hub、两个进程抢设备 | `lsusb -t` 看 5000M；只保留一个 RealSense owner |
| 视觉图像重复/冻结 | `--fps=100` 配 30 FPS 相机或读取线程异常 | 视觉集用 30 Hz；检查视频帧数和相邻帧差 |
| 保存很慢/编码队列满 | 分辨率、编码线程或 CPU 负载 | 先短集；降低分辨率；去掉 `--streaming-encoding` |
| `torchcodec` 不可用警告 | 平台缺少 torchcodec | LeRobot 回退 pyav，通常不影响 RGB 采集 |
| `libtinfo` 警告 | conda/bash 库版本提示 | 若后续命令正常可忽略 |
| 两臂左右动作对调 | 16D 顺序或 CAN 映射错误 | 固定 `left8+right8`，显式核对 can0/can1 和关节名 |

### 10.1 建议的隔离实验

按以下顺序一次只改变一个变量：

1. 无 VR、无 Rerun、无重力，固定发送一个小目标；确认硬件/控制器本身。
2. 无视觉 recorder，检查 callback 计数和 action 差分。
3. 开 VR、保持 100/100 Hz，检查 pose 与 command topic 频率。
4. 加低比例 gravity，比较 state 受力变化，不把它当滤波器。
5. 最后才打开 Rerun 或相机编码。

### 10.2 还应排除的基础原因

即使 callback 已修复，卡顿仍可能来自下列基础层：

- **多 publisher**：VR、replay、测试脚本或残留节点同时写同一 command topic，
  机械臂会在不同目标之间来回跳。用 `ros2 topic info -v` 只保留预期发布者。
- **CPU/线程调度**：Python IK、视频编码、Rerun 和 ROS executor 同时争用 CPU，
  会让 10 ms timer 产生长尾延迟。录制时先关 viewer，观察 `top`/`htop`，必要时
  降低相机分辨率或编码线程。
- **DDS/网络**：Pico UDP 丢包、Wi-Fi 漫游、ROS domain/工作区串线会造成 pose
  断流或过期数据。检查 Pico 与电脑 IP、`ROS_DOMAIN_ID`、pose 时间间隔和
  teleop 的 gap 日志。
- **CAN/供电**：错误的 bit-rate、终端电阻、线缆、USB-CAN 负载或 48 V 电源
  波动会造成底层重试和控制周期抖动。先看 `ip -details link` 和驱动日志，
  出现 BUS-OFF 时停止动作。
- **机械/编码器**：减速器回差、线缆拖拽、负载变化、零位偏差和编码器噪声会
  让固定目标也抖；这类问题与 LeRobot 数据格式无关。
- **时间基准**：系统休眠、CPU 降频或不同节点使用不一致的时钟会造成时间戳
  漂移。数据验收时同时看 `timestamp`、真实帧数和 topic 到达间隔。

常用只读检查：

```bash
ros2 topic info -v /left_forward_position_controller/commands
ros2 topic info -v /right_forward_position_controller/commands
ros2 node list
top -H -b -n 1 | head -35
ip -details link show can0
ip -details link show can1
dmesg --level=err,warn | tail -50
```

这些命令只能帮助定位，不能替代急停和硬件厂商的 CAN/电机诊断流程。

## 11. 数据质量验收标准

一集数据只有同时满足以下条件才进入训练候选：

### 状态和动作

- `info.json` 的 `fps` 等于真实采样频率；不能手工伪造。
- 100 Hz、20 s 约 2000 帧；30 Hz、10 s 约 300 帧。
- `observation.state`、`action` 形状均为 16，名称和左右顺序一致。
- 所有值有限，单位/夹爪方向正确，关节没有越过硬件安全范围。
- action 与 state 方向一致，延迟可解释；没有大段 NaN、零值或冻结。
- 邻帧差分没有系统性 4 帧台阶；大跳变必须能在原始 VR/目标流找到原因。

### ROS 时序

- `joint_states`、left/right arm callback 计数接近预期。
- `ros2 topic hz` 和 recorder 内部计数都正常；不能只看发布端 hz。
- 录制期间只有一个真正的动作发布者（VR）；replay 时只有 replay。

### 视觉

- `observation.images.front` 不是黑屏、花屏、错误朝向或静止画面。
- 视频帧数与 30 Hz episode 时长大致匹配，编码可被 LeRobot/pyav 读取。
- 相机固定、曝光稳定、任务区域完整；记录相机序列号、分辨率、fps、光照。
- 图像与动作在时间上大致对应；若需要精确同步，后续加入硬件时间戳。

### 实验可追溯性

每批数据至少记录：日期、任务文本、机械臂型号、左右 CAN、VR 参数版本、
gravity 开关/scale、相机 serial/profile、数据 fps、是否有 Rerun、异常 episode
及删除原因。这样才能解释不同批次动力学和视觉差异。

## 12. 100 Hz 数据、30 Hz VLA 和 `realtime-vla` 思路

### 12.1 三个“频率”不是一回事

```text
感知/策略频率：相机帧 + VLA 推理（约 15～30 Hz）
动作输出频率：策略每次产生一个 action 或 action chunk
执行频率：ROS2/控制器/CAN（当前 100 Hz）
```

实时 VLA 项目通常在策略和执行层之间做 action chunk、时间预算、保持或
插值。策略一次推理耗时不应阻塞 100 Hz 的安全执行线程。推理超时应保持
最近安全目标或触发停止策略，而不是发布半更新的数组。

这也是参考 `realtime-vla` 一类项目时最值得借鉴的部分：把“观测采集、策略
推理、动作队列和机器人控制”拆成有界的异步环路，显式测量端到端 latency、
inference time、action age 和 queue 长度。它解决的是实时推理调度问题，
不是把相机或机械臂本身强行改成某个固定频率。对本项目的对应实现是：

```text
相机/状态线程       → 最新观测（带时间戳）
策略线程            → 15～30 Hz action/chunk
100 Hz 执行线程     → 取最新有效目标、插值/限步、watchdog
```

后续部署至少应记录 `observation_timestamp`、`policy_timestamp`、
`command_timestamp` 和动作年龄；当动作超过 timeout 或队列断流时进入安全
保持/停止，而不是让旧目标无限期运行。

### 12.2 推荐的数据策略

保留两种数据：

1. `*_raw_100hz`：无视觉或高频 state/action master，用于时序排查、控制研究
   和后处理。
2. `*_vla30`：D435i RGB 30 FPS 对齐的 state/action/task 训练集。

从 100 Hz 降到 30 Hz 时，按时间戳重采样并明确采用“最近值、线性插值还是
动作平均”；不要只改 metadata。相机每 33 ms 一帧，机器人 10 ms 一次，
视觉集通常取每帧附近的最新状态和对应 action。

### 12.3 推理部署的安全分层

```text
VLA 15～30 Hz
  → action chunk / 目标队列
  → 100 Hz 保持/插值/限幅层
  → 关节范围、速度、加速度、deadman、超时检查
  → position/effort controller
```

模型输出不能绕过现有的限幅、急停和控制器互斥规则。先在 MuJoCo 做
shadow rollout，再 fake hardware，最后真实单臂低速，再双臂。

## 13. VLA/ACT/MuJoCo 后续路线

### 13.1 数据和相机

1. 已确认 D435i USB 3（`5000M`）和 640×480@30 RGB profile 可打开；再做连续读帧验收。
2. 录 1 集 10 s，检查视频、state/action、任务文本。
3. 录 10～20 集短任务，筛掉失败、遮挡、断流和动作阶梯。
4. 再扩大到有足够变化的 50～200 集；保持相机位姿和任务文本一致。
5. 如需 wrist camera，再增加固定键名 `observation.images.left_wrist`、
   `right_wrist`，不要中途改变 `front` 的含义。

### 13.2 训练顺序

| 模型 | 建议用途 |
| --- | --- |
| ACT | 最先做 pipeline smoke test，快速验证 action chunk 和 replay |
| SmolVLA | 当前最实际的首个视觉语言策略，资源和调试成本较低 |
| Pi0/Pi0.5/XVLA | 数据、GPU、部署接口稳定后再尝试的重模型 |

训练前必须先完成数据 viewer/离线检查；模型训练不能修复错误的左右顺序、
相机冻结或 3°～4° staircase。

首个软件 smoke test 可以先用现有 `PICO4_TO_VLA_SIM_PIPELINE.md` 中的
synthetic/rendered 数据，不接真实机械臂：

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_DATASETS_CACHE=/tmp/hf_datasets_cache

# 先生成/检查 synthetic dataset，再做极小 ACT 训练
python examples/openarm_xr/smoke_synthetic_pico_dataset.py \
  --repo-id local/openarm_pico4_smoke \
  --root /tmp/lerobot_openarm_pico4_smoke \
  --episodes 2 --frames 16 --fps 5

HF_HUB_OFFLINE=1 python -m lerobot.scripts.lerobot_train \
  --policy.type=act \
  --dataset.repo_id=local/openarm_pico4_smoke \
  --dataset.root=/tmp/lerobot_openarm_pico4_smoke \
  --output_dir=/tmp/openarm_pico4_act_smoke \
  --batch_size=2 --steps=2 --save_freq=2 --log_freq=1 \
  --num_workers=0 --persistent_workers=false \
  --policy.device=cpu --policy.push_to_hub=false \
  --policy.chunk_size=4 --policy.n_action_steps=4 \
  --policy.dim_model=64 --policy.n_heads=2 \
  --policy.dim_feedforward=128 --policy.n_encoder_layers=1 \
  --policy.n_vae_encoder_layers=1 --policy.use_vae=false \
  --policy.pretrained_backbone_weights=null --wandb.enable=false
```

这个命令只证明数据格式、视频读取、策略加载和训练 plumbing，不能证明真实
任务已学会。视觉数据通过验收后，再按同一配置将 `--dataset.repo_id` 换成
`local/openarmx_bimanual_vla30_test`，先做 SmolVLA 约 1000 step smoke，再
扩大训练；详细超参数和 MuJoCo rollout 命令见链接文档。

参数选择逻辑是：`steps=2/1000` 用于快速发现 schema、显存或导入错误；
`chunk_size` 表示一次预测的连续动作长度，`n_action_steps` 表示实际执行
其中多少步；先用 4～10 步观察延迟和动作连续性，再根据任务时长调大。真实
部署时不要照搬 smoke 的 CPU/低维参数，必须重新测量推理时间和 action age。

### 13.3 仿真分级

| 级别 | 观察来源 | 能证明什么 |
| --- | --- | --- |
| Synthetic video | 固定/生成视频 | 数据格式、编码、训练脚本、推理接口 |
| `openarm_sim` | 根据 fake joint state 渲染简易桌面 | 图像是否随机器人状态变化 |
| OpenArm MuJoCo | `camera_head` 等真实虚拟相机 | 物理、碰撞、动作和视觉闭环 |

纯 RViz 画面不是 VLA 场景；真实 D435i 也不是模拟相机。训练和 rollout 的
相机键名、分辨率、颜色格式必须一致。

### 13.4 真实部署门槛

- 低级控制层保持 100 Hz，并有独立 watchdog/超时。
- policy action 经过关节范围、速度、单周期限步和碰撞约束。
- 推理线程与控制线程解耦，GPU 卡顿不会阻塞急停。
- 先无负载、单臂、低速度；再双臂和真实物体。
- 记录每次 rollout 的模型版本、相机配置、gravity 配置和异常日志。

## 14. 官方 LeRobot/OpenArm 文档与本地方案的关系

参考资料：

- [LeRobot 中文 OpenArm 文档](https://dctx-team.github.io/lerobot-zh/zh/openarm/)
- [LeRobot 相机文档](https://hugging-face.cn/docs/lerobot/cameras)
- [realtime-vla](https://github.com/Dexmal/realtime-vla)
- [官方 OpenArm MuJoCo](https://github.com/enactic/openarm_mujoco)

官方 LeRobot OpenArm 路线通常是：

```text
physical OpenArm leader
  → openarm_leader / bi_openarm_leader
  → openarm_follower / bi_openarm_follower
  → Damiao CAN/CAN-FD
```

官方文档中的 `lerobot-setup-can --mode=setup/test`、`openarm_follower` 校准
和 `bi_openarm_leader` 示例，适合建立官方 CAN 基线；它们不是当前
OpenArmXR bringup 的替代命令。当前实机已经由
`openarm_bringup + openarmx_teleop_vr` 管理，切换到官方 follower 前必须先
停止现有 ROS 控制器并重新核对 CAN 接口、动作顺序和校准文件。

官方基线的典型检查形式（仅在明确要切换到官方 Damiao 路线时执行）是：

```bash
lerobot-setup-can --mode=setup --interfaces=can0,can1
lerobot-setup-can --mode=test --interfaces=can0,can1
```

不要把这两条命令和当前 OpenArmXR 的自定义 CAN/ROS bringup 同时运行。

官方相机配置思路也值得保留，例如把相机键名、序列号、分辨率和 fps 写进
robot config：

```text
front: {type: intelrealsense, serial_number_or_name: "<SN>",
        width: 640, height: 480, fps: 30}
```

这适合未来迁移到 LeRobot 原生 `lerobot-record`；当前 OpenArmX direct-topic
录制仍应使用本地只读 recorder 的 `--camera-serial-number-or-name` 参数。

值得借鉴的是数据 schema、校准、安全限制、相机命名、离线采集和 replay
流程；但当前项目的 Pico/OpenArmX direct-topic 路线不是 physical leader
路线。尤其不能直接把：

- `/joint_trajectory` 的 `pico_openarm_ik`；
- `forward_position_controller/commands` 的 OpenArmX teleop；
- 官方 Damiao follower 的 CAN 端口和动作顺序；

当成同一个适配器。若后续想完全使用标准 `lerobot-record`，应先实现一个
“只读 OpenArmX observation + 不与 VR 冲突的 action source”适配层，再迁移，
而不是在真实控制器上同时运行两个 publisher。

## 15. 当前已验证内容、限制和待办

### 已验证

- Pico 4 → OpenArm IK → fake hardware/RViz。
- OpenArmX UDP bridge → Pinocchio → 真实双臂 position controller。
- 右 `can0`、左 `can1` 的当前硬件映射。
- VR/ros2_control 100 Hz 稳定参数组合。
- gravity node 的低比例 effort 输出链路（左臂先行验证）。
- 双臂 16D、100 Hz、20 s≈2000 帧 LeRobot 无视觉采集。
- 独立 executor 修复 callback 堆积；v2 数据大多数 `common_gap=1`。
- 真实 replay 的 subscriber、首帧偏差和 `--confirm-real` 安全门。
- D435i 被 finder 识别为 serial `943222072197`，USB descriptor 为 `3.2`，默认
  RGB profile 为 `1280×720@30 FPS`。
- 已用 LeRobot `RealSenseCamera` 主动请求 `640×480@30`，成功读取
  `(480, 640, 3)`、`uint8` RGB 帧；还需用连续 60 帧测试确认长时间稳定性。
- 相关 Python 文件 `py_compile` 和 `git diff --check` 已通过（以当前工作区
 版本为准）。

### 尚未完全验证/需要实机完成

- USB 3 线材下 D435i 640×480@30 连续读帧/长时间稳定性。
- 严格硬件时间戳的图像-状态同步。
- 大规模视觉数据的曝光、遮挡和任务多样性。
- VLA 在 MuJoCo 与真实机械臂之间的 domain gap。
- policy 推理线程、action chunk 和 100 Hz safety wrapper 的实机闭环。
- 重力补偿在不同负载、左右臂和全工作空间的最佳 scale。

### 下一步清单

```text
[x] `lsusb -t` 确认 D435i 对应设备行协商为 `5000M`（finder 已报 USB 3.2）
[x] 独立读取 pyrealsense2 RGB 640x480@30 单帧
[ ] 连续 60 帧/短时测试 pyrealsense2 RGB 640x480@30
[ ] 真实双臂录 1 集 10 s 视觉数据
[ ] 检查约 300 帧视频和 front 图像内容
[ ] 录制多集并生成 raw_100hz / vla30 两套数据
[ ] 在 synthetic/openarm_sim 上跑 ACT smoke test
[ ] 在 MuJoCo camera_head 上跑 SmolVLA 首个模型
[ ] 加入 action chunk、watchdog、限幅和 shadow rollout
[ ] 单臂低速真实策略测试，再进入双臂
```

## 16. 快速索引

- 真实双臂 100 Hz 无视觉：`OPENARMX_BIMANUAL_100HZ_RECORD_REPLAY.md`
- 卡顿、重力、Rerun 排查：`OPENARMX_LEROBOT_TROUBLESHOOTING_AND_WORKFLOW.md`
- 开机和 VR 丝滑参数：`DUAL_ARM_SMOOTH_TELEOP_GUIDE.md`
- Pico → VLA → 仿真：`PICO4_TO_VLA_SIM_PIPELINE.md`
- Pico 路线与官方 OpenArm 对比：`PICO4_VS_OFFICIAL_OPENARM.md`
- 当前 recorder：`record_openarmx_bimanual_lerobot.py`
- 当前 replay：`replay_openarmx_bimanual_lerobot.py`
