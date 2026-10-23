# OpenArmX + LeRobot 真实双臂：问题总结与操作流程

适用环境：ROS 2 Humble、Python 3.10 环境 `oplerobot310`、OpenArmX 双臂、Pico 4 VR。本文假定右臂使用 `can0`、左臂使用 `can1`；实际接线不同只修改对应 launch 参数。

## 1. 整体原理

```text
Pico 4 → openarmx_teleop_vr（100 Hz）
       → /left_forward_position_controller/commands
       → /right_forward_position_controller/commands
       → 真实 OpenArm 双臂

/joint_states + 两路 arm command + 两路 gripper command
       → LeRobot 双臂采集器（只读）
```

采集器只订阅 ROS2 话题，不向机械臂发送动作；重播脚本会向真实控制器发送动作，必须和 VR 摇操互斥运行。

## 2. 这次遇到的问题和根因

### ROS2 logger 报错

`RcutilsLogger.info()` 不支持标准 Python logging 的多参数格式化写法：

```python
logger.info("topic=%s", topic)
```

已改为先拼接字符串：

```python
logger.info(f"topic={topic}")
```

### 旧版 100 Hz 采集器导致 3°~4° 跳变

旧版采集循环每 10 ms 只调用一次 `rclpy.spin_once()`，但节点同时订阅 5 路高频话题。一个 `spin_once()` 通常只处理一个回调，消息会排队。结果是时间戳仍为 100 Hz，但动作值经常每 4 帧才更新一次，于是单次变化累积为 3°~4°。

`ros2 topic hz` 测量的是发布者频率，不是录制节点处理回调的频率。

新版采集器改为独立 ROS2 executor 线程持续处理回调，主线程固定 100 Hz 读取快照并写盘。20 秒、100 Hz 时，`joint_states`、`left_arm`、`right_arm` 应接近 2000；夹爪回调较低可能是正常的，因为夹爪只在位置变化时发布。

### LeRobot 是否改写了动作

没有。LeRobot 只是保存采集器传入的 `action`。旧数据的阶梯已经存在于采集器拿到的输入中，不是 parquet 写盘或 Rerun 造成的。

修复后的 `v2` 数据已验证时间戳稳定 100 Hz，大多数维度逐帧更新，最大单周期动作约 1°，没有旧数据的 3°~4° 跳变。

### 重力补偿

采集器本身不启用重力补偿。录制时是否有补偿取决于 bringup 的 `enable_gravity_compensation` 和 `apply_gravity_compensation` 参数。重力补偿影响真实机械臂受力、下坠和跟随感，不会自动平滑动作目标。首次建议 `gravity_scale:=0.01`，不要直接使用 0.5。

### Rerun

Rerun 只显示 `observation.state` 和 `action`，不提供重力补偿，也不会提高控制频率。真实机械臂首次重播建议关闭 Rerun；确认稳定后再用 `--display-data --display-every 10` 或 20 观察，以减少 GUI 负载。

## 3. 从开机到开始录制

### 3.1 检查 CAN

```bash
ip -details link show can0
ip -details link show can1
```

确认接口已配置并处于 `UP`。确认急停可用，机械臂周围无人和障碍物，第一次只做空载小幅运动。

### 3.2 启动真实双臂 bringup（先无重力补偿）

终端 1：

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

### 3.3 检查控制器和状态

```bash
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic info -v /left_forward_position_controller/commands
ros2 topic info -v /right_forward_position_controller/commands
```

应确认以下控制器为 `active`：

```text
joint_state_broadcaster
left_forward_position_controller
right_forward_position_controller
left_gripper_controller
right_gripper_controller
```

### 3.4 启动双臂 Pico 4 VR 摇操

终端 2：

```bash
unset PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
ros2 launch openarmx_teleop_vr openarm_teleop_vr_real.launch.py
```

确认日志中的控制频率为 `100.0 Hz`。连接 Pico 后，左/右 Grip 控制对应手臂，左右扳机控制夹爪；先小幅验证方向和关节顺序。

### 3.5 开始录制双臂 100 Hz 无视觉数据

终端 3：

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

按提示回车开始。20 秒应保存约 2000 帧。结束时检查：

```text
Saved episode 1: 2000 frames
ROS callbacks during episode:
joint_states 接近 2000
left_arm 接近 2000
right_arm 接近 2000
```

如果 arm 或 joint_states 计数明显偏低，不要使用该数据。

### 3.6 离线检查数据

```bash
cat ~/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz_v2/meta/info.json
```

应看到 `fps: 100`、`total_frames: 2000`、`robot_type: bi_openarm_ros2_follower`。动作的 `common_gap` 应大多数为 1，不应普遍为 4。

## 4. 重力补偿模式

### 无重力补偿（第一次验证推荐）

```text
enable_gravity_compensation:=false
apply_gravity_compensation:=false
```

先用它验证采集数据、控制器、CAN 和重播轨迹。

### 启用重力补偿

必须重新启动 bringup：

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=false \
  robot_controller:=forward_position_controller \
  right_can_interface:=can0 \
  left_can_interface:=can1 \
  enable_gravity_compensation:=true \
  apply_gravity_compensation:=true \
  gravity_scale:=0.01 \
  gravity_ramp_duration_s:=10.0
```

确认日志包含 `apply_effort=true` 和 `LIVE gravity feedforward is enabled`，并检查：

```bash
ros2 control list_controllers
ros2 param get /openarm_gravity_comp gravity_scale
ros2 param get /openarm_gravity_comp apply_effort
```

建议逐步测试 `0.01 → 0.02 → 0.05`。出现突然抬升、振荡或发热时立即停止并回到无补偿模式。

## 5. 重播和 Rerun

重播前必须在 VR 终端按 `Ctrl-C` 停止 `openarm_teleop_vr_real.launch.py`；ROS2 bringup 保持运行。不能让 VR 和 replay 同时发布相同话题。

### 无 Rerun（首次真实重播推荐）

```bash
cd ~/OPENARM/lerobot
conda activate oplerobot310
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

脚本会检查控制器订阅者、双臂 `/joint_states` 和首帧姿态偏差，随后等待回车才发送动作。

### 有 Rerun（只用于观察）

确认无 Rerun 重播平稳后再使用：

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

Rerun 中查看 `observation.state` 和 `action`。如果 GUI 造成负载升高，改为 `--display-every 20` 或关闭 Rerun。

## 6. 仍然抖动时的排查顺序

1. 确认 VR 已停止，命令话题没有多个发布者：`ros2 topic info -v <topic>`。
2. 确认使用 `v2` 数据且动作 `common_gap` 大多数为 1。
3. 关闭 Rerun 比较。
4. 关闭重力补偿比较。
5. 检查 replay 话题频率是否接近 100 Hz。
6. 若固定姿态持续发送仍抖，检查真实硬件位置控制增益、CAN 通讯、控制器竞争、机械回差和驱动器参数。

## 7. 推荐验证顺序

```text
无重力补偿 + 无 Rerun
        ↓
确认 v2 数据逐帧且重播稳定
        ↓
gravity_scale=0.01 的低强度补偿
        ↓
最后才启用 Rerun 观察
```

