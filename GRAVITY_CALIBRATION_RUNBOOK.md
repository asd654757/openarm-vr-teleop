# OpenArm：从采样到确定 gravity_scale 的具体流程

工具：[tools/gravity_calibration.py](tools/gravity_calibration.py)。它只读取 ROS 状态/参数并保存文件，或者进行离线计算，不发布机械臂命令、不修改补偿比例。

目前完成了 Python/ROS 导入检查、真实 URDF 的离线 KDL 调用、已知参数的合成数据拟合和对照比较，以及独立 ROS 域中的 14 关节只读采集测试（会拒绝运动中的反馈）。测试没有连接 CAN 或发布机械臂命令。尚未完成真机录制验证，下面的实际数值必须由新实验产生。

## 第 0 步：确定本次实验范围

- 第一轮固定为空载、固定夹爪开度，不触碰桌面或由人托臂；保留正常位置控制和已验证稳定的补偿设置。
- 左右臂都采样时，先分别检查是否能稳定保持，再采双臂。不要为了本实验解除保护或关闭位置控制。
- 明确输出两项：`alpha_id`（解释力矩数据的模型比例）和 `alpha_deploy`（指定工作区域中经对照验证的部署值）。
- 先核对电机型号、量程、方向和单位。没有独立力矩校核时，结果应称为“基于驱动器力矩估计的辨识”，不能称为真实重力矩的绝对校准。

当前 v1.0 `control_gains.yaml` 中关节 4 的 kd 配置是 40，而当前 CAN 编码器把 kd 限制在 0–5；如果运行的确是这份驱动，发送的值会限幅为 5。采集前核对运行版本和实际增益，固定这套配置。不要在实验中途修改增益；若修正配置，应先单独验证，再新建实验 session。

## 第 1 步：启动已经验证的控制链路并检查反馈

沿用原 SOP 启动 bringup 和 VR。若它们已在运行，不要重复启动，也不要为了辨识先改变当前已验证的比例。不得仅因下面需要“保持姿态”而取消急停和现有控制保护。

在采集终端执行：

```bash
cd ~/OPENARM
unset ROS_DISTRO ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES ROS_ETC_DIR PYTHONPATH
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

ros2 control list_controllers
ros2 param get /openarm_gravity_comp gravity_scale
ros2 param get /openarm_gravity_comp ramp_duration_s
timeout 5s ros2 topic echo /joint_states --once
```

确认 arm 关节顺序正确，每臂 7 个关节的 position、velocity、effort 完整且为有限值。不同姿态下 effort 应有可信的变化；全零、固定占位值、单位不明、或方向相反时先排查反馈，不能直接拟合。

本工具使用 `/usr/bin/python3`，因为本机已验证它在 source ROS 后能导入 NumPy、rclpy 和消息类型；不要混用任意 Conda Python。

## 第 2 步：保存实际模型和参数

新开一个实验目录，后续所有命令在同一个终端中执行，保留 SESSION 变量：

```bash
SESSION="$HOME/OPENARM/experiments/gravity_calibration/empty_hand_$(date +%Y%m%d_%H%M%S)"

/usr/bin/python3 tools/gravity_calibration.py snapshot \
  --session "$SESSION" \
  --payload "empty_hand_fixed_gripper" \
  --torque-note "驱动器力矩反馈；独立Nm校准状态请按实际实验记录补充"
```

填写真实的校核情况，不要保留含糊说明后在论文中声称已校准。工具从 `/openarm_gravity_comp` 读取正在使用的 robot_description、比例、ramp、限幅和控制硬件参数，保存 `robot.urdf` 和 `session.json`。

检查输出的 warnings。保存的模型含质量/质心，不是从图像或 LeRobot action 推测模型。后续若换负载、模型或增益，应新开 session。

## 第 3 步：设计并编号 20 个姿态

这是本项目的建议实验规模，不是行业统一标准：

- 20 个姿态覆盖实际作业区域中的高低、前后、肩肘不同力臂，均选择已有能力安全到达和保持的姿态。
- 先整体设计姿态，再从各区域均匀选 6 个作留出验证，编号 `p15`–`p20`。
- 其余 14 个编号 `p01`–`p14`，用于拟合。
- 同一个姿态的所有重复只能属于同一划分，不能把同一姿态的相邻帧拆到训练/验证两边。
- 每个姿态采 4 次，从两种接近路径 A/B 各采 2 次，例如 A、B、A、B。记录实际角度；A/B 用于检查迟滞，不保证静摩擦已被消除。

可先用 2 个姿态做试采，检查记录是否有效。确认数据质量后再采全量，试采不代替 20 姿态设计。

## 第 4 步：按姿态逐次录制

用原 VR 方法缓慢移动到 `p01`。松开 Grip 后确认位置控制能保持，无人支撑，等待稳定。执行：

```bash
/usr/bin/python3 tools/gravity_calibration.py record \
  --session "$SESSION" \
  --pose p01 --repeat 1 --split train --approach A \
  --arm both --settle 15 --seconds 5
```

`--settle 15` 表示录制前再等 15 秒。若实际 ramp 大于 10 秒，把 settle 改为至少 `ramp + 3` 秒，并确认已稳定。

从另一条路径重新接近 p01、稳定后，分别改为：

```text
--pose p01 --repeat 2 --split train --approach B
--pose p01 --repeat 3 --split train --approach A
--pose p01 --repeat 4 --split train --approach B
```

然后依次采 p02–p14。p15–p20 的命令将 `--split train` 改为 `--split validation`。

这些是手动逐次操作，不是让机械臂自动执行的轨迹列表。若只测单臂，整轮保持相同 `--arm left` 或 `--arm right`。

每次应输出 `ACCEPTED`。工具初始使用 `|dq| <= 0.03 rad/s`、5 秒内角度跨度不超过 0.5°、至少约 20 Hz 的有效样本和新鲜的重力 preview 来筛选稳定窗口；这些是起始质量阈值，需结合量化噪声检查，不是物理准确性的保证。

输出 `REJECTED` 时保留原记录，按原因排查并重新采。重试使用新 tag，例如 `--tag retry1`，避免覆盖失败记录。不得单纯放宽阈值以强行通过。

每次目录包含 `raw.csv` 和 `record.json`。工具记录实际角度、速度、总力矩反馈和前馈 preview；拟合不把前馈 preview 当作真实负载。

## 第 5 步：离线计算模型比例

```bash
/usr/bin/python3 tools/gravity_calibration.py fit \
  --session "$SESSION" \
  --calculator ~/OPENARM/OpenarmXR/install/openarm_gravity_comp/lib/openarm_gravity_comp/gravity_comp_calculate \
  --output "$SESSION/fit_v1"
```

拟合使用已保存 URDF 和各次稳定窗口的实际平均角度，调用 KDL 计算未缩放的模型重力矩。默认对每关节拟合一个诊断偏置、对整套采样拟合一个公共比例，并对姿态分组 bootstrap 计算区间。低重力激励的关节不参与比例估计。

主要结果：

| 字段/文件 | 查看内容 |
| --- | --- |
| `fit_v1/fit_report.json` 的 `alpha` | 模型比例估计，不是自动推荐参数 |
| `alpha_pose_bootstrap_95_ci` | 姿态重采样区间，不含系统性误差 |
| `per_arm_alpha_diagnostic` | 左右臂是否明显不一致 |
| `per_approach_alpha_diagnostic` | A/B 接近路径的估计是否明显不一致，检查迟滞影响 |
| `offsets_nm` | 模型没有解释的固定偏差，当前控制器不会自动应用这些偏置 |
| `held_out_model_rmse_nm_with_offsets` | 独立姿态上的模型预测误差 |
| `deployment_predictions_no_offset` | 按当前限幅预测的剩余力矩和饱和比例，不是真机对照结果 |
| `pose_summary.csv` | 供画模型力矩—反馈力矩散点图和残差图 |

先检查残差、左右臂一致性和区间宽度。估计为负、超出代码允许范围、偏置很大、区间很宽时先排查；不要直接截断后上机。若结果提示激励不足，增加力臂差异明显的安全姿态。

即使区间很窄，静摩擦、未知外力和力矩估算的系统偏差仍可能存在。A/B 路径差异明显时，应进一步做匹配的低速正反向数据或摩擦建模，不能宣称已经得到真实重力比例。

## 第 6 步：比较部署候选值，保持位置目标不变

先根据拟合值及其区间，在**已验证可用的范围内**预先选 3–5 个候选。若合理区间在当前已验证范围以外，应先单独逐级验证扩展范围，而不是直接跳到拟合值。

推荐先在可用范围内粗比较，再在表现接近的邻域细化。所有候选用同样的增益、负载、ramp、限幅、姿态和记录时长。对照次序交错或随机，减少温升和先后顺序的影响。

每个留出姿态（推荐 p15–p20）按如下方式比较：

1. 先保持原基线比例，用 VR 到达目标姿态。
2. 启动下面的 baseline 录制时，最初几秒保持手柄稳定、Grip 有效，让工具看到位置命令；然后松开 Grip，在 settle 结束前确认位置目标不再变化。
3. baseline 第一次成功后，不再移动 VR、发送新位置命令或重新到达该姿态，只改变已选候选比例。这样各候选使用同一个位置目标。

```bash
/usr/bin/python3 tools/gravity_calibration.py record \
  --session "$SESSION" \
  --pose p15 --repeat 1 --split deployment --approach fixed \
  --tag baseline --arm both --settle 15 --seconds 5
```

如果提示目标缺失，说明工具启动后未观察到目标命令，不应拿实际位置冒充目标。重新试采，使用新的 tag，并确保在 settle 阶段捕获位置命令。

在另一个已经 source ROS 的终端手动输入本次候选值：

```bash
read -r -p "输入已验证可用的本次候选比例：" GRAVITY_TRIAL_SCALE
ros2 param set /openarm_gravity_comp gravity_scale "$GRAVITY_TRIAL_SCALE"
```

回到采集终端，将 `--tag` 改成该候选的固定标签，例如用 `a_0_30` 表示 0.30。下面标签仅表示命名方法，数值应与实际候选一致：

```bash
/usr/bin/python3 tools/gravity_calibration.py record \
  --session "$SESSION" \
  --pose p15 --repeat 1 --split deployment --approach fixed \
  --tag a_0_30 --arm both --settle 15 --seconds 5 \
  --target-record "$SESSION/recordings/baseline_p15_r01"
```

`--target-record` 只是读回先前观察到的位置目标，不会重新发送它。它只适用于期间没有改变位置目标的情况；若移动了 VR 或存在其他位置命令源，不能使用旧目标充当当前目标，应重做该姿态的整组对照。

同一姿态每个候选采至少 3 次，并对 baseline 也补齐 repeat 2/3。交错设置候选，每次等待 ramp 和稳定。这里是同一保持任务的重复测量，不是 3 个独立姿态。

完成该姿态后，再转向 p16。每次到新姿态前恢复同一原基线比例，再建立该姿态自己的 baseline 目标。不同候选不要只测不同姿态。

发生持续振荡、异常漂移、异响或温升异常时停止该候选并记录失败，不能通过只保留成功窗口将它评为最佳。

## 第 7 步：离线比较实际保持误差

至少完成相同的 4 个留出姿态，推荐完成全部 6 个，且各候选每姿态有至少 3 次有效记录后：

```bash
/usr/bin/python3 tools/gravity_calibration.py compare \
  --session "$SESSION" \
  --output "$SESSION/compare_v1"
```

输出 `hold_comparison.csv/json`：包含候选比例、等姿态权重的保持 RMSE、最大关节波动、角度跨度和失败记录。不同位置目标、不同非比例参数、不同关节集合或不完整的对照组会被拒绝。

确定部署值时：

1. 先剔除实际出现不稳定、温度或驱动器状态不满足要求的候选；这些条件不能只靠该工具判定。
2. 比较相同目标下的保持 RMSE，并观察各关节波动；在剩余候选中选择满足预先约定指标且表现最好的值。
3. 若候选间差异与重复测量波动相当，报告可用范围，不宣称唯一最优。
4. 若最好结果在所测范围边界，只能称为该范围内最好，需要进一步验证才能扩展结论。
5. 若大量前馈饱和，比例的影响可能已被限幅遮蔽；报告适用工作区域，不能为获得曲线而擅自放宽限幅。
6. 用最后的候选补做固定的低速遥操作轨迹和较长时间运行，确认动态跟踪、振动和温度仍符合任务要求，再确定 `alpha_deploy`。静态试验不能证明高速任务下也最优。

## 最后在论文中交付什么

- 模型/质量/质心/末端负载及反馈校核说明，实际增益与限幅。
- 20 姿态设计、分组方法、4 次重复、A/B 路径和失败记录。
- 模型比例、区间、左右臂诊断、留出集残差；说明未补偿的摩擦/偏置。
- 各候选相同目标下的保持误差与波动对照表，动态和温度验证记录。
- 最终部署比例及适用负载、姿态、速度范围。

采样数量、稳定窗口阈值和模型激励阈值都是本项目的起始设计，实验前应结合噪声和任务要求确认并固定。方法依据见 [MIT 系统辨识讲义](https://underactuated.mit.edu/sysid.html)。

## 不连接机械臂时的离线执行

已提供 `tools/run_gravity_calibration_sim.py`，它不初始化 ROS、不访问 CAN、不发布硬件命令。它用 URDF 建立 MuJoCo 模型，生成带已知噪声/偏置的静态逆动力学观测，再执行实际前向动力学 PD 保持，对照当前限幅、质量偏差和右臂额外负载。

在本机重现完整实验：

```bash
cd ~/OPENARM
source /opt/ros/humble/setup.bash
source ~/OPENARM/OpenarmXR/install/setup.bash

/usr/bin/python3 -c 'import xacro; xacro.main()' \
  OpenarmXR/openarm_description/assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro \
  bimanual:=true ros2_control:=true use_fake_hardware:=false \
  -o /tmp/openarm_calibration_sim_source.urdf

SIM_OUTPUT="$HOME/OPENARM/experiments/gravity_calibration_sim/offline_$(date +%Y%m%d_%H%M%S)"
MUJOCO_GL=egl ~/anaconda3/envs/oplerobot310/bin/python \
  tools/run_gravity_calibration_sim.py \
  --urdf /tmp/openarm_calibration_sim_source.urdf \
  --calculator ~/OPENARM/OpenarmXR/install/openarm_gravity_comp/lib/openarm_gravity_comp/gravity_comp_calculate \
  --output "$SIM_OUTPUT"
```

这里 `use_fake_hardware:=false` 只让离线展开的 URDF 包含真实控制配置供读取增益；上述命令不加载 ROS 硬件插件、不启动 bringup，也不连接电机。

首轮结果在 `experiments/gravity_calibration_sim/offline_20261023_full/REPORT.md`。仿真能验证辨识方法和所指定模型中的参数表现，不能单独确定真实机械臂的最终比例。
