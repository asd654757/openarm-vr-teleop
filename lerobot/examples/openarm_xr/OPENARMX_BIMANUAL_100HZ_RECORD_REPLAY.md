# OpenArmX 双臂 LeRobot 100 Hz 无视觉采集与重播

适用于真实 OpenArm 双臂、Pico 4/OpenArmX VR 摇操和 ROS 2 Humble。录制脚本只订阅话题；重播脚本会驱动真实机械臂。

## 安全要求

- 第一次只做空载、小幅度动作；急停必须可用。
- 录制时运行 VR 摇操；重播时停止 VR 摇操，不能让两个节点同时发布命令。
- 当前工程约定右臂 `can0`、左臂 `can1`，实际接线不同时只修改 launch 参数。

## 启动真实双臂 ROS2

终端 1：

```bash
unset PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=false \
  robot_controller:=forward_position_controller \
  right_can_interface:=can0 \
  left_can_interface:=can1
```

检查控制器和关节状态：

```bash
ros2 control list_controllers
ros2 topic info /left_forward_position_controller/commands
ros2 topic info /right_forward_position_controller/commands
ros2 topic info /left_gripper_controller/joint_trajectory
ros2 topic info /right_gripper_controller/joint_trajectory
ros2 topic echo /joint_states --once
```

确认 `joint_state_broadcaster`、左右 `forward_position_controller`、左右 `gripper_controller` 都是 `active`。

## 启动双臂 VR 摇操

终端 2：

```bash
unset PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
ros2 launch openarmx_teleop_vr openarm_teleop_vr_real.launch.py
```

确认日志显示控制频率为 `100 Hz`。Pico 4 连接后，左/右 Grip 分别控制左右臂，左右扳机控制对应夹爪。先小幅度验证方向和关节映射。

## 录制 100 Hz 无视觉数据

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
  --repo-id local/openarmx_bimanual_novision_100hz \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz \
  --task "Move the object with both OpenArms using Pico 4 VR" \
  --fps 100 --episodes 10 --episode-time-s 20
```

脚本等待双臂关节状态，然后每个 episode 等待回车。20 秒约 2000 帧。数据为双臂各 7 个关节 + 1 个夹爪，不含图像。中途 `Ctrl-C` 会尽量保存当前 episode。

采集器使用独立 ROS2 executor 线程处理 `/joint_states`、左右臂命令和左右夹爪命令，再由主线程以 100 Hz 取快照。每个 episode 保存后会打印各路 ROS 回调计数；正常情况下 20 秒数据应接近 2000 次/路。若计数明显偏低，不要使用该数据，先检查 ROS2 节点和系统负载。

检查：

```bash
cat ~/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz/meta/info.json
```

应看到 `fps: 100`、`robot_type: bi_openarm_ros2_follower` 和对应帧数。

## 使用 D435i 作为全局相机采集 VLA 数据

当前 OpenArmX VR 节点直接发布左右 `forward_position_controller` 指令，
因此视觉采集继续使用本文件中的 OpenArmX 录制器；不要在同一时间再启动
另一个会向相同控制器发布动作的 teleop 节点。录制器现在支持可选的
RealSense RGB 相机，图像键为 `observation.images.front`。

先在不启动机械臂和 VR 的情况下确认相机。序列号应使用实际输出的 `Id`：

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH

python -m lerobot.scripts.lerobot_find_cameras realsense \
  --record-time-s 3 \
  --output-dir /tmp/d435i_check
```

应看到 D435i 的序列号、USB 类型和默认 profile，并在
`/tmp/d435i_check/` 生成一张非黑 PNG。若 USB 类型不是 USB 3（通常应为
`3.2`/`5000M`），先更换 USB 3 数据线或端口。第一版只记录 RGB，不打开深度
和 IMU。

确认序列号后，先录制一个 10 秒、30 Hz 的短 episode：

```bash
# 终端 1：真实双臂 bringup（保持原来的无/有重力补偿配置）
# 终端 2：openarm_teleop_vr_real.launch.py
# 终端 3：执行下面命令
conda activate oplerobot310
cd ~/OPENARM/lerobot
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_HUB_OFFLINE=1

python examples/openarm_xr/record_openarmx_bimanual_lerobot.py \
  --repo-id local/openarmx_bimanual_vla30_test \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_vla30_test \
  --task "Move the object with both OpenArms using Pico 4 VR" \
  --fps 30 \
  --episodes 1 \
  --episode-time-s 10 \
  --camera-serial-number-or-name "D435I_SERIAL" \
  --camera-width 640 \
  --camera-height 480 \
  --camera-fps 30 \
  --camera-warmup-s 2 \
  --vcodec h264
```

确认 30 Hz 采样和视频完整后，如果单个 episode 保存时间较长，再追加
`--streaming-encoding`；录制过程中若出现 `Encoder queue full`，先去掉该选项。

第一次视觉采集建议不打开 Rerun；确认数据稳定后再单独用 dataset viewer
查看。正常结果应包含约 300 帧，以及：

```text
observation.state
observation.images.front
action
fps: 30
```

这里的 `--fps 30` 只表示 LeRobot 数据采样频率；VR 和真实机械臂控制仍保持
100 Hz。相机配置中的 `fps`、`width`、`height` 必须同时设置，且同一台 D435i
不要同时放到 left/right arm 配置中。

## 停止摇操并重播

先在终端 2 按 `Ctrl-C` 停止 VR 摇操；终端 1 的 ROS2 bringup 保持运行。

终端 4：

```bash
conda activate oplerobot310
cd ~/OPENARM/lerobot
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash
export PYTHONPATH=~/OPENARM/lerobot/src:$PYTHONPATH
export HF_HUB_OFFLINE=1
python examples/openarm_xr/replay_openarmx_bimanual_lerobot.py \
  --repo-id local/openarmx_bimanual_novision_100hz \
  --root ~/OPENARM/lerobot/data/openarmx_bimanual_novision_100hz \
  --episode 0 --fps 100 --confirm-real
```

重播会检查左右控制器订阅者、双臂 `/joint_states` 和首帧姿态偏差（默认最大 `10°`），通过后才发送动作。程序会等待回车确认；偏差过大时先将机械臂放回录制首帧姿态，不要直接放宽阈值。

常见问题：卡顿时确认录制元数据和重播日志都是 `100 Hz`，并且 VR 摇操已停止；无订阅者时检查控制器是否为 `active`；初始姿态偏差过大时先手动回到录制开始姿态。
