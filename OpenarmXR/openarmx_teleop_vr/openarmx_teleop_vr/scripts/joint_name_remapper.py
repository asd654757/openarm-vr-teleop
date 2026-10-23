#!/usr/bin/env python3
"""Remap joint names in /joint_states from openarm_ to openarmx_ prefix.

Bridges the naming gap between openarm_hardware (openarm_ prefix)
and openarmx_teleop_vr_node (openarmx_ prefix).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointNameRemapper(Node):
    def __init__(self):
        super().__init__("joint_name_remapper")

        self.declare_parameter("input_topic", "/joint_states_raw")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("from_prefix", "openarm_")
        self.declare_parameter("to_prefix", "openarmx_")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.from_prefix = self.get_parameter("from_prefix").value
        self.to_prefix = self.get_parameter("to_prefix").value

        self.pub = self.create_publisher(JointState, output_topic, 10)
        self.sub = self.create_subscription(JointState, input_topic, self.callback, 10)

        self.get_logger().info(
            f"Remapping '{self.from_prefix}' → '{self.to_prefix}' "
            f"({input_topic} → {output_topic})"
        )

    def callback(self, msg: JointState):
        out = JointState()
        out.header = msg.header
        out.name = [
            n.replace(self.from_prefix, self.to_prefix, 1) if n.startswith(self.from_prefix) else n
            for n in msg.name
        ]
        out.position = msg.position
        out.velocity = msg.velocity
        out.effort = msg.effort
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointNameRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
