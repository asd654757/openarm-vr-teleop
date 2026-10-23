# OpenArmX 双臂 VR 带视觉数据采集 SOP

本文适用于当前已验证的真实双臂方案：

```text
Pico 4 OpenArmX App
  -> Wi-Fi UDP 5100
  -> OpenArmX bridge + Pinocchio IK（100 Hz）
  -> ROS 2 position controller（100 Hz）
  -> OpenArm 双臂 CAN FD

D435i RGB（640x480 @ 30 FPS）
  + 双臂 observation.state/action（30 Hz 采样）
  + task 文本
  -> LeRobot 视觉数据集
```

当前硬件约定：

| 项目 | 当前值 |
| --- | --- |
| 右臂 CAN | `can0` |
| 左臂 CAN | `can1` |
| Pico UDP 端口 | `5100` |
| VR/IK/ros2_control | `100 Hz` |
| D435i serial | `943222072197` |
| D435i 视觉采集 | `640x480 RGB @ 30 FPS` |
| LeRobot 视觉数据集 | `30 Hz` |
| action/state 顺序 | 左 7 关节 + 左夹爪 + 右 7 关节 + 右夹爪 |

## 1. 重力补偿参数说明

重力节点根据当前关节角和 URDF/KDL 模型计算重力前馈：

```text
g(q) = KDL.JntToGravity(q)
tau_ff = clamp(gravity_scale * g(q), -torque_limits, +torque_limits)
```

当前位置环仍然工作，重力补偿只是额外的力矩前馈。它不会过滤 Pico 抖动，
也不会改变 recorder 的采样频率。

关键参数：

| 参数 | 建议值 | 说明 |
| --- | ---: | --- |
| `enable_gravity_compensation` | `true` | 启动重力计算节点 |
| `apply_gravity_compensation` | `true` | 将前馈发送给真实 effort controller |
| `gravity_enable_left/right` | `true` | 对左右臂启用计算和输出 |
| `gravity_scale` | 初始 `0.01` | 模型重力矩比例，`0.01` 表示 1% |
| `gravity_ramp_duration_s` | `10.0`～`20.0` | 从当前比例平滑变化到目标比例 |
| 发布频率 | `100 Hz` | launch 内固定 |
| state timeout | `0.1 s` | 关节状态过期后向零输出回落 |
| torque limits | `[2,2,2,2,0.5,0.5,0.5] Nm` | 当前 launch 的逐关节限幅 |

`gravity_scale=0.5` 表示使用模型计算重力矩的 50%，不是“电机开到 50%”。
模型、零位、负载或符号有误时，0.5 仍可能导致抬升、振荡和发热。因此首次
开机不要直接以 0.5 起动；先从 0.01 启动，再在有人托住机械臂、急停可用、
无负载且小范围动作的条件下逐级升高。

建议阶梯：

```text
0.01 -> 0.02 -> 0.05 -> 0.10 -> 0.20 -> 0.30 -> 0.50
```

每一级至少观察静止保持、缓慢抬放、异响、振荡和电机温升。不同
`gravity_scale` 的 episode 不要无标签地混进同一训练批次；建议把比例写进
数据集名称和实验记录。

## 2. 开机前安全检查

1. 清空双臂工作空间，确认底座、相机和线缆固定。
2. 急停和机械臂总电源必须在操作者伸手可及处。
3. 机械臂先处于安全、低势能姿态；不要手持物体启动。
4. 确认右臂接 `can0`、左臂接 `can1`，不要只凭接口名称猜左右。
5. Pico 与电脑连接同一个局域网，优先使用 5 GHz Wi-Fi。
6. D435i 直连 USB 3 端口，不经过不稳定的 Hub 或延长线。
7. 首次带视觉采集只录 `1` 集、`10` 秒，验收后再扩大规模。

出现突然抬升、反向、持续振荡、异响、明显发热、CAN `BUS-OFF` 或控制源不明
时，立即松开 Grip，并停止 VR 动作源；无法确认安全时使用急停或断电。

## 3. 电脑开机后的基础检查

### 3.1 查看电脑当前 IP

每次电脑重启、切换 Wi-Fi 或 DHCP 重新分配后，IP 都可能变化：

```bash
ip -br -4 addr show scope global
ip route show default
```

找默认路由对应网卡的 IPv4，例如 `192.168.1.23/24`，在 Pico OpenArmX App
中填写不带 `/24` 的地址：

```text
Server/IP: 192.168.1.23
UDP port: 5100
```

不要填写 `127.0.0.1`、Docker 地址、`can0/can1` 地址或旧的 Wi-Fi 地址。

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

检查：

```bash
ip -details link show can0
ip -details link show can1
```

应看到两路均为 `UP`，仲裁速率 `1000000`、数据段 `5000000`、CAN FD 开启。
若出现 `ERROR-PASSIVE` 或 `BUS-OFF`，不要启动 bringup，先检查供电、终端电阻、
线缆和左右接口。

### 3.3 检查 D435i

此时不要启动 `realsense-viewer`、`realsense2_camera` 或其他相机进程。

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

lsusb -t

python -m lerobot.scripts.lerobot_find_cameras realsense \
  --record-time-s 3 \
  --output-dir /tmp/d435i_check
```

当前已验证结果：

```text
Id: 943222072197
Usb type descriptor: 3.2
Default profile: RGB 1280x720 @ 30 FPS
lsusb -t: 对应设备行为 5000M
```

recorder 使用 `640x480@30` 是另一个明确请求的 profile，与 finder 显示的默认
`1280x720@30` 不冲突。相机检查结束后保证 finder 已退出，不要让它继续占用
D435i。

首次建立视觉数据集或更换 USB 线/端口后，再做一次连续 60 帧测试：

```bash
python - <<'PY'
import time
from lerobot.cameras import ColorMode, Cv2Rotation
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

camera = RealSenseCamera(RealSenseCameraConfig(
    serial_number_or_name="943222072197",
    width=640,
    height=480,
    fps=30,
    color_mode=ColorMode.RGB,
    use_depth=False,
    rotation=Cv2Rotation.NO_ROTATION,
    warmup_s=2,
))

connected = False
try:
    camera.connect()
    connected = True
    for _ in range(5):
        camera.read()
    start = time.perf_counter()
    shape = None
    for _ in range(60):
        shape = camera.read().shape
    elapsed = time.perf_counter() - start
    print("shape:", shape)
    print("measured_fps:", round(60 / elapsed, 2))
finally:
    if connected:
        camera.disconnect()
PY
```

期望 `shape=(480, 640, 3)`、实测接近 30 FPS。测试进程结束后再启动 recorder。

## 4. 终端 1：启动真实双臂和重力补偿

先给双臂上电，再执行：

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
  enable_gravity_compensation:=true \
  apply_gravity_compensation:=true \
  gravity_scale:=0.01 \
  gravity_ramp_duration_s:=10.0 \
  gravity_enable_left:=true \
  gravity_enable_right:=true
```

该命令会同时启用左右 position controller 和 gravity 所需的左右 effort
controller。不要另起一套 bringup。

### 4.1 验证控制器和重力节点

在另一个已 source 同一环境的终端执行：

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 param get /openarm_gravity_comp enabled
ros2 param get /openarm_gravity_comp apply_effort
ros2 param get /openarm_gravity_comp gravity_scale
ros2 param get /openarm_gravity_comp ramp_duration_s
ros2 topic echo /gravity_compensation/left_effort --once
ros2 topic echo /gravity_compensation/right_effort --once
ros2 topic echo /left_forward_effort_controller/commands --once
ros2 topic echo /right_forward_effort_controller/commands --once
```

期望以下 controller 为 `active`：

```text
joint_state_broadcaster
left_forward_position_controller
right_forward_position_controller
left_forward_effort_controller
right_forward_effort_controller
left_gripper_controller
right_gripper_controller
```

`enabled=true` 只表示计算已启用；`apply_effort=true` 才表示前馈实际发送给电机。

### 4.2 逐级升到 50%

确认 `0.01` 没有抬升、反向、振荡或发热后，在检查终端中逐级执行。每条命令
之间先完成观察，不要一次全部粘贴运行：

```bash
ros2 param set /openarm_gravity_comp ramp_duration_s 10.0
ros2 param set /openarm_gravity_comp gravity_scale 0.02
```

随后按相同方法分别设置：

```bash
ros2 param set /openarm_gravity_comp gravity_scale 0.05
ros2 param set /openarm_gravity_comp gravity_scale 0.10
ros2 param set /openarm_gravity_comp gravity_scale 0.20
ros2 param set /openarm_gravity_comp gravity_scale 0.30
```

只有前面各级都验证稳定，才设置 50%，并使用更长 ramp：

```bash
ros2 param set /openarm_gravity_comp ramp_duration_s 20.0
ros2 param set /openarm_gravity_comp gravity_scale 0.50
ros2 param get /openarm_gravity_comp gravity_scale
```

动态修改 `gravity_scale` 会从当前比例重新平滑到新比例。`apply_effort` 不能在
运行中动态切换；它由 bringup 启动参数确定。

需要立即退出重力补偿但机械臂仍可安全保持时：

```bash
ros2 param set /openarm_gravity_comp ramp_duration_s 2.0
ros2 param set /openarm_gravity_comp enabled false
```

这会向零前馈 ramp。发生危险动作时不要等待软件 ramp，直接使用急停/断电。

## 5. 终端 2：启动 Pico 4 VR 摇操

```bash
conda activate oplerobot310
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

ros2 launch openarmx_teleop_vr openarm_teleop_vr_real.launch.py
```

日志应包含：

```text
Listening for controller poses on 0.0.0.0:5100
Node initialized - control rate: 100.0 Hz
```

现在打开 Pico 的 OpenArmX App，重新核对电脑 IP 和端口 `5100`。连接后先不按
Grip，先检查 control mode：

```bash
timeout 3s ros2 topic echo /vr/control_mode --once || true
```

该 topic 不是 latched；如果 3 秒内没有新的 mode packet，不代表 bridge 失效。
当前常用的是 `relative` 模式，持续小幅移动对应手柄并优先检查：

```bash
timeout 20s ros2 topic hz /pico_left_controller/pose
timeout 20s ros2 topic hz /pico_right_controller/pose
```

如果 relative pose 没有数据，而 App 明确选择了 `absolute`，再检查：

```bash
timeout 20s ros2 topic hz /vr/left/pose_absolute
timeout 20s ros2 topic hz /vr/right/pose_absolute
```

再检查输出：

```bash
ros2 topic hz /left_forward_position_controller/commands
ros2 topic hz /right_forward_position_controller/commands
ros2 topic info -v /left_forward_position_controller/commands
ros2 topic info -v /right_forward_position_controller/commands
```

左右 command 应接近 100 Hz，并且每个话题只能有预期的 OpenArmX teleop
publisher。此时不能同时运行 replay、另一套 `teleop_xr` mapper 或策略节点。

当前稳定 VR 参数由 `teleop_params.yaml` 加载：

```text
control_rate=100 Hz
command_filter_alpha=0.5
stream_timeout_sec=1.0 s
normal max step=0.8/0.8/0.8/1.0/1.0/0.8/0.8 deg/cycle
fast max step=1.5/1.5/1.5/1.8/1.8/1.2/1.2 deg/cycle
```

先做小幅动作：左/右 Grip 分别授权左右臂，Trigger 控制对应夹爪。松开 Grip
应停止跟随。方向、Grip、夹爪和回零动作都正确后，才能开始录制。

### VR 没反应时的最短排查

```bash
ip -br -4 addr show scope global
ss -lunp | rg ':5100\b'
timeout 3s ros2 topic echo /vr/control_mode --once || true
timeout 20s ros2 topic hz /pico_left_controller/pose
timeout 20s ros2 topic hz /pico_right_controller/pose
ros2 topic hz /left_forward_position_controller/commands
ros2 topic hz /right_forward_position_controller/commands
```

判断顺序：

1. 当前电脑 IP 是否与 Pico App 中一致。
2. Pico 和电脑是否仍在同一局域网，Pico App 是否保持前台。
3. bridge 是否监听 `0.0.0.0:5100`。
4. relative 模式下 `/pico_*_controller/pose` 是否持续有数据；absolute 模式才看
   `/vr/*/pose_absolute`。
5. pose 有数据但 command 没数据时，再查校准、control mode、Grip 和 IK 日志。
6. command 有 100 Hz 但机械臂不动时，再查 controller 状态、CAN 和 publisher。

如果出现“刚能动，随后进入 `Teleop mode idle`”，不要先增大
`stream_timeout_sec`。应先停止完整 VR launch 和真实 bringup，单独启动只接收
UDP、不会发布机械臂 command 的 bridge：

```bash
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node
```

另一个终端中持续移动手柄并测 20 秒：

```bash
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

timeout 20s ros2 topic hz /pico_left_controller/pose
timeout 20s ros2 topic hz /pico_right_controller/pose
```

正常应连续发布且通常高于 50 Hz，不能几秒无消息。bridge 每收到 60 个 UDP
包会打印一条 `RAW UDP data`；正常时该日志应连续出现。若这里已经断流，问题
位于 Pico App、手柄追踪或 Wi-Fi，不在 IK、重力补偿、CAN 或 LeRobot。

## 6. 终端 3：采集 30 Hz 带视觉数据

录制器只订阅 ROS 状态和动作，并读取相机；它不会成为机械臂的第二个动作
publisher。底层 VR/ROS/CAN 继续以 100 Hz 运行，LeRobot 只按 30 Hz 对齐 RGB、
state、action 和任务文本。

每个新批次使用新的 `repo-id` 和空的 `root`。下面名称显式记录了 30 Hz 和
`gravity_scale=0.50`；若实际比例不是 0.50，必须修改名称和实验记录：

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot

unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_HUB_OFFLINE=1

python examples/openarm_xr/record_openarmx_bimanual_lerobot.py \
  --repo-id local/openarmx_bimanual_vla30_g050_test \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_vla30_g050_test \
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

正常启动信息应包含：

```text
Joint states received. Recording global RGB camera 943222072197 at 640x480@30 Hz.
Press Enter to start episode 1/1:
```

开始前把双臂放在任务起始姿态，确认画面包含双臂、夹爪、物体和操作区域，
然后按 Enter。10 秒结束后应保存约 300 帧。

首次不要添加 `--streaming-encoding`，也不要同时开启 Rerun。先让控制和数据
采集负载尽量简单；确认保存耗时确实成为问题后再单独评估流式编码。

## 7. 数据验收

### 7.1 metadata 和视频文件

```bash
cd ~/OPENARM/lerobot

python - <<'PY'
import json
from pathlib import Path

root = Path("data/openarmx_bimanual_vla30_g050_test")
info = json.loads((root / "meta/info.json").read_text())

print("fps:", info["fps"])
print("episodes:", info["total_episodes"])
print("frames:", info["total_frames"])
print("robot_type:", info["robot_type"])
print("features:", list(info["features"]))
print("front:", info["features"].get("observation.images.front"))
print("action names:", info["features"]["action"]["names"])
PY

find data/openarmx_bimanual_vla30_g050_test/videos \
  -type f -name '*.mp4' -ls
```

通过标准：

```text
fps = 30
episodes = 1
frames ≈ 300
robot_type = bi_openarm_ros2_follower
features 包含 observation.state、observation.images.front、action
视频目录存在非空 MP4
action 共 16 维，顺序为 left8 + right8
```

### 7.2 离线 Rerun viewer

录制和真实控制进程停止后再打开：

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

lerobot-dataset-viz \
  --repo-id local/openarmx_bimanual_vla30_g050_test \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_vla30_g050_test \
  --episode-index 0
```

若环境中没有 `lerobot-dataset-viz` entrypoint，使用：

```bash
python -m lerobot.scripts.lerobot_dataset_viz \
  --repo-id local/openarmx_bimanual_vla30_g050_test \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_vla30_g050_test \
  --episode-index 0
```

检查 `front` 图像是否正常、曝光是否稳定、画面是否冻结，以及 state/action
曲线是否连续。Rerun 只显示数据，不负责重力补偿或控制。

### 7.3 不通过时不要继续批量采集

| 现象 | 首先处理 |
| --- | --- |
| 只有约 150 帧 | 相机/数据集是否误设为 15 Hz，检查 USB/profile |
| 约 300 帧但图像大量重复 | 检查相机独立持续帧率和 USB 负载 |
| 没有 MP4 | 检查编码错误、`videos/` 路径和磁盘空间 |
| 黑屏或曝光剧烈变化 | 固定相机位置和照明，重新录短集 |
| action 每数帧跳一次 | 检查 recorder callback 计数和是否使用当前后台 executor 版本 |
| 机械臂抖动 | 先关 viewer/编码干扰，再隔离 gravity、VR 目标和底层增益 |
| gravity 开启后抬升或振荡 | 立即回到低 scale 或关闭 enabled；核对模型、零位和符号 |

## 8. 扩大正式采集

短集完全通过后，使用新的正式数据集名称，例如：

```bash
python examples/openarm_xr/record_openarmx_bimanual_lerobot.py \
  --repo-id local/openarmx_bimanual_vla30_g050_v1 \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_vla30_g050_v1 \
  --task "Move the object with both OpenArms using Pico 4 VR" \
  --fps 30 \
  --episodes 10 \
  --episode-time-s 20 \
  --camera-serial-number-or-name "943222072197" \
  --camera-width 640 \
  --camera-height 480 \
  --camera-fps 30 \
  --camera-warmup-s 2 \
  --vcodec h264
```

每集 20 秒约 600 帧。每个 episode 开始前统一起始姿态、物体位置、任务语言
和重力参数；同时保留一定的物体位置、光照和动作轨迹变化。不要为了数量继续
保留碰撞、遮挡严重、控制丢失或动作失败的 episode。

## 9. 正常停止顺序

1. 让当前 episode 正常结束并确认 `Saved episode ...`。
2. 停止 recorder；确认数据 finalize 完成。
3. 松开左右 Grip，在终端 2 `Ctrl-C` 停止 VR。
4. 正常情况下将重力补偿平滑降为零：

   ```bash
   ros2 param set /openarm_gravity_comp ramp_duration_s 5.0
   ros2 param set /openarm_gravity_comp enabled false
   ```

5. 等待约 5 秒，再在终端 1 `Ctrl-C` 停止 bringup。
6. 确认电机已停止/解除使能，再关闭机械臂电源。
7. 最后关闭 Pico；D435i 可在进程全部退出后拔出。

紧急情况不执行上述等待顺序，直接使用急停或断电。

## 10. 频率与数据含义

```text
Pico/IK/command/controller = 100 Hz
D435i RGB                  = 30 FPS
LeRobot 视觉数据集         = 30 Hz
```

视觉 recorder 每 33.3 ms 取一次最新图像、最新关节实测状态和最新 VR 动作目标。
这不会把底层控制降到 30 Hz。不要把视觉数据集设为 100 Hz 配 30 FPS 相机，
否则同一图像会重复约 3～4 次，视频和训练时间尺度也会变得不清晰。

`observation.state` 来自 `/joint_states`，表示真实测量位置；`action` 来自左右
position command 和 gripper command，表示 VR 控制目标。重力补偿 effort 不在
当前 16 维 action 中，但会改变实际动力学，因此必须在批次名称或实验日志中
记录 `gravity_scale`。
