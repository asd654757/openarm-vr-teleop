# OpenArm 双臂 VR 丝滑摇操指南

本文档用于 OpenArm 双臂实机 + Pico 4 + OpenArmX VR 摇操作，目标是复现左臂已经验证过的“稳定、跟手、不频繁停顿”的配置，并把同一套参数用于双臂。

## 1. 总体数据流

```text
Pico 4 / OpenArmX APK
  -> UDP 5100
  -> openarmx_teleop_bridge_vr_node
  -> /pico_left_controller/pose、/pico_right_controller/pose 等
  -> openarmx_teleop_vr_node（Pinocchio IK + 指令平滑/限速）
  -> /left_forward_position_controller/commands
  -> /right_forward_position_controller/commands
  -> openarm_ros2 控制器 + CAN 硬件接口
  -> 左右机械臂电机
```

同时还有一个独立的重力补偿节点：

```text
/joint_states
  -> openarm_gravity_comp（KDL 重力模型）
  -> /gravity_compensation/left_effort
  -> /gravity_compensation/right_effort
  -> /left_forward_effort_controller/commands
  -> /right_forward_effort_controller/commands
  -> 与位置指令一起叠加发送给电机
```

## 2. 当前已验证参数

下面是左臂已经验证过、并已同步到双臂配置的关键参数。

| 参数 | 值 | 作用 |
| --- | --- | --- |
| `control_rate` | 100 Hz | 遥操 IK 和指令发布频率 |
| `controller_manager.update_rate` | 100 Hz | ros2_control 读写硬件和 CAN 的频率 |
| `command_filter_alpha` | 0.5 | 指令平滑折中值；1.0 关闭，0.18 过慢 |
| `stream_timeout_sec` | 1.0 s | Pico 位姿断流超过 1 秒才进入 idle |
| `max_step_deg_joint1..7` | 0.8/0.8/0.8/1.0/1.0/0.8/0.8 | 普通模式每周期最大角度步长 |
| `fast_max_step_deg_joint1..7` | 1.5/1.5/1.5/1.8/1.8/1.2/1.2 | 快速模式每周期最大角度步长 |
| `step_limit_enable_threshold_deg_joint1..7` | 全部 0 | 每周期都强制限速 |
| `gravity_scale` | 0.01 | 只发送 1% 模型重力前馈，保持低风险 |
| `gravity_ramp_duration_s` | 10.0 s | 重力前馈从 0 缓慢升到目标比例 |
| `torque_limits` | `[2.0, 2.0, 2.0, 2.0, 0.5, 0.5, 0.5]` | 每个关节重力前馈的力矩上限，单位 Nm |

## 3. 开机与硬件准备

### 3.1 开机顺序

1. 打开电脑。
2. 打开 Pico 4。
3. 确认左右机械臂结构完整、线缆安全、附近无障碍物。
4. 给双臂电源上电。
5. 确认右臂接 `can0`，左臂接 `can1`。

> 如果右臂尚未修复或未连接，不要启动双臂 bringup。

### 3.2 配置 CAN FD

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 \
  dbitrate 5000000 fd on restart-ms 100
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 \
  dbitrate 5000000 fd on restart-ms 100
sudo ip link set can1 up
```

验证：

```bash
ip -details link show can0
ip -details link show can1
```

应该看到：

```text
state UP
can <FD>
bitrate 1000000
dbitrate 5000000
```

### 3.3 可选：检查电机

```bash
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

openarm-can-cli -i can0 discover --max-id 8
openarm-can-cli -i can1 discover --max-id 8
```

如果电机未上电或线缆异常，discover 可能无结果。此时不要继续摇操，先处理供电和 CAN 通信。

## 4. 启动双臂实机

### 4.1 终端 1：双臂 bringup

```bash
unset PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

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

启动后双臂会缓慢回零。回零期间不要按 Grip、不要触碰机械臂。

检查控制器：

```bash
ros2 control list_controllers
```

应看到：

```text
joint_state_broadcaster          active
left_forward_position_controller active
right_forward_position_controller active
left_forward_effort_controller   active
right_forward_effort_controller  active
left_gripper_controller          active
right_gripper_controller         active
```

检查重力补偿输出：

```bash
ros2 topic echo /gravity_compensation/left_effort --once
ros2 topic echo /gravity_compensation/right_effort --once
ros2 topic echo /left_forward_effort_controller/commands --once
ros2 topic echo /right_forward_effort_controller/commands --once
```

### 4.2 查询电脑 IP

```bash
ip -4 addr show
```

记录电脑当前 Wi-Fi IPv4 地址，例如：

```text
192.168.31.252
```

### 4.3 配置 Pico 4

打开 Pico 上的 OpenArmX APK，设置：

```text
ROS PC IP Address: 192.168.31.252
UDP Port: 5100
```

要求：

- Pico 和电脑在同一局域网。
- 优先使用 5GHz Wi-Fi。
- Pico 靠近路由器。
- APK 保持前台。
- 关闭 Pico 对 OpenArmX 的电池优化。

电脑端 bridge 日志应出现：

```text
Listening for controller poses on 0.0.0.0:5100
```

### 4.4 终端 2：启动双臂 VR 遥操

```bash
unset PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

ros2 launch openarmx_teleop_vr \
  openarm_teleop_vr_real.launch.py
```

### 4.5 操作方式

1. Pico 进入 VR 会话。
2. 确认左右手柄追踪正常。
3. 按住左手 Grip 控制左臂，按住右手 Grip 控制右臂。
4. 扳机控制对应夹爪。
5. 松开 Grip，机械臂保持当前位置。
6. 右手柄 B 回零位。
7. 左手柄 Y 触发双手上举动作。

首次使用只做小范围低速运动，确认左右臂都正常后再做大动作。

## 5. 运行检查

检查 Pico 输入频率：

```bash
timeout 20s ros2 topic hz /pico_left_controller/pose
timeout 20s ros2 topic hz /pico_right_controller/pose
```

检查机械臂指令频率：

```bash
timeout 20s ros2 topic hz /left_forward_position_controller/commands
timeout 20s ros2 topic hz /right_forward_position_controller/commands
```

正常情况下应稳定在 50 Hz 以上，最好接近 100 Hz。如果日志出现：

```text
VR pose stream gap
Teleop mode idle
```

优先排查：

- Pico Wi-Fi 信号和 APK 是否前台。
- 电脑 Wi-Fi 是否切换或休眠。
- 左右手柄是否追踪丢失。

## 6. 正常结束与紧急停止

正常结束：

1. 松开 Grip。
2. 按 B 回零，等待双臂稳定。
3. `Ctrl+C` 停止 VR 遥操终端。
4. `Ctrl+C` 停止双臂 bringup 终端。
5. 确认电机 DISARMED 后再关闭双臂电源。

紧急情况：

```text
出现振荡、反向拉扯、异响、突然跳动或 Pico 长时间断流
```

应立即 `Ctrl+C` 停止 bringup，必要时直接断电。

## 7. 卡顿改进过程和原理

### 7.1 卡顿的主要来源

最初的问题不是单一原因，而是多个因素叠加：

1. `command_filter_alpha=0.18` 导致指令过度平滑，机械臂明显滞后。
2. `stream_timeout_sec=0.3` 导致 Pico 网络只要抖 300ms，节点就进入 idle。
3. `torque_limits` 过低，重力前馈被截断。
4. 控制器频率从 100 改到 750 后，在这台电脑/CAN 环境下出现明显负担，最终回到 100Hz 才稳定。

### 7.2 改进过程

| 步骤 | 修改 | 结果 |
| --- | --- | --- |
| 1 | 关闭重力前馈的过低限制，提高 `torque_limits` | 重力补偿有机会真正输出 |
| 2 | `command_filter_alpha` 从 0.18 调到 1.0 | 更跟手，但 Pico 抖动也更明显 |
| 3 | `stream_timeout_sec` 从 0.3 调到 1.0 | 短断流不再频繁停顿 |
| 4 | `command_filter_alpha` 调到 0.5 | 跟手和稳定之间取得平衡 |
| 5 | 底层频率改 750 | 当前机器上更卡，回退 |
| 6 | 底层保持 100Hz，遥操保持 100Hz | 稳定流畅 |

### 7.3 100Hz 与 750Hz 的区别

两个频率不是同一个东西：

```text
control_rate: 100 Hz
  遥操节点运行 IK 和发布位置指令的频率

controller_manager.update_rate: 100 Hz
  ros2_control 读取关节状态、发送 CAN 指令的频率
```

原版 OpenArmX 通常是遥操 100Hz + 底层 750Hz。但在当前实机环境中，750Hz 会让 CAN/CPU 负担过大，表现反而更卡，所以本方案使用 100Hz + 100Hz。

## 8. 重力补偿原理

### 8.1 计算方式

重力补偿节点从 URDF 读取机械臂质量、惯量和关节结构，建立 KDL 动力学链：

```text
openarm_left_link0 -> openarm_left_hand_tcp
openarm_right_link0 -> openarm_right_hand_tcp
```

再根据当前 `joint_states` 计算每个关节为抵抗重力需要的力矩：

```text
gravity_effort = KDL.JntToGravity(joint_positions)
```

实际发送：

```text
output = clamp(gravity_scale * gravity_effort, -torque_limit, torque_limit)
```

### 8.2 为什么 0.01 也能让手臂停住

机械臂的 MIT 控制公式为：

```text
tau = Kp * (q_desired - q)
    + Kd * (0 - q_dot)
    + tau_ff
```

其中：

- `Kp` 位置环负责把机械臂拉回目标角度。
- `Kd` 阻尼环负责抑制速度。
- `tau_ff` 是重力前馈，也就是 `gravity_scale * gravity_effort`。

所以“能不能停住”主要靠位置环 Kp，而不是重力补偿。`gravity_scale=0.01` 时，前馈几乎等于零，手臂仍然可以停住。

重力补偿的真正作用：

1. 减小电机为抵抗重力而持续输出的电流。
2. 减小位置环为保持姿态而需要的角度误差。
3. 让抬起、放下和跟随 VR 时更自然、更顺滑。

### 8.3 如何验证重力补偿是否生效

不要用“能不能停住”判断，应该看数值：

```bash
ros2 topic echo /gravity_compensation/left_effort --once
ros2 topic echo /left_forward_effort_controller/commands --once
ros2 topic echo /joint_states --once
```

同一个姿态下，把 `gravity_scale` 从 `0.01` 调到 `0.5`，如果补偿生效，`left_effort` 会按比例增大，`joint_states` 里的电机力矩会下降。

## 9. 各部件作用

| 部件 | 作用 |
| --- | --- |
| Pico 4 + OpenArmX APK | 采集手柄/头部位姿、Grip、扳机、按键，并通过 UDP 发送 |
| `openarmx_teleop_bridge_vr_node` | 接收 UDP，转换为 ROS 2 的 Pico/Vr topic |
| `joint_name_remapper` | 把 `openarm_` 关节名转换为 `openarmx_`，供 IK 使用 |
| `openarmx_teleop_vr_node` | 100Hz IK、模式仲裁、指令平滑、限速、夹爪控制 |
| `left/right_forward_position_controller` | 接收 7 维位置指令，写入位置接口 |
| `left/right_forward_effort_controller` | 接收重力前馈，写入力矩接口 |
| `openarm_gravity_comp` | KDL 重力计算、比例缩放、力矩上限 |
| `openarm_hardware_interface` | CAN FD 通信，把位置/速度/力矩打包成 MIT 协议 |
| DAMIAO 电机 | 执行 MIT 控制，返回位置/速度/力矩状态 |

## 10. 调参建议

如果双臂仍然卡顿：

1. 先确认 Pico 输入频率，不要先调机械臂参数。
2. 如果输入正常但机械臂慢，再小幅提高 `max_step_deg_joint*`。
3. 如果机械臂抖动，先把 `command_filter_alpha` 降到 0.35。
4. 如果觉得回零慢，提高 `button_motion_step_deg`。
5. 每次只改一个参数，重启对应节点后测试。
