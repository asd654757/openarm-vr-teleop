# OpenArm ROS2 Simulation Bridge

This example drives OpenArm through LeRobot's `Robot` API while OpenArm ROS2 is
running with fake hardware.

## Single Arm

```bash
ros2 launch openarm_bringup openarm.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller
```

In another terminal:

```bash
python examples/openarm_ros2/smoke_sim.py --duration-s 10
```

## Bimanual

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py \
  use_fake_hardware:=true \
  robot_controller:=forward_position_controller
```

In another terminal:

```bash
python examples/openarm_ros2/smoke_sim.py --bimanual --duration-s 10
```

## Minimal Mock

If `ros2_control`, `openarm_description`, or `openarm_bringup` are not installed
yet, test the LeRobot bridge against the bundled minimal ROS2 mock:

```bash
python examples/openarm_ros2/mock_ros2_openarm.py --bimanual
```

In another terminal:

```bash
python examples/openarm_ros2/smoke_sim.py --bimanual --duration-s 10
```

If `control_msgs` is not installed yet, disable the gripper path:

```bash
python examples/openarm_ros2/mock_ros2_openarm.py --bimanual --no-gripper
python examples/openarm_ros2/smoke_sim.py --bimanual --no-gripper --duration-s 10
```

## LeRobot CLI

The registered robot types are:

- `openarm_ros2_follower`
- `bi_openarm_ros2_follower`

They expose the same OpenArm feature names used by the existing CAN follower:
arm joints are degrees and the gripper is `0` closed to `-65` open. The bridge
converts arm commands to ROS2 radians and gripper commands to the OpenArm ROS2
finger joint range `0..0.044m`.
