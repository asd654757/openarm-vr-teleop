#!/usr/bin/env python

import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyMappingPrinter(Node):
    def __init__(self, topics: list[str], threshold: float) -> None:
        super().__init__("xr_joy_mapping_printer")
        self.threshold = threshold
        self.last_buttons: dict[str, list[int]] = {}
        self.last_axes: dict[str, list[float]] = {}
        for topic in topics:
            self.create_subscription(Joy, topic, lambda msg, t=topic: self._on_joy(t, msg), 10)
            self.get_logger().info(f"Listening on {topic}")

    def _on_joy(self, topic: str, msg: Joy) -> None:
        buttons = list(msg.buttons)
        axes = [float(v) for v in msg.axes]
        last_buttons = self.last_buttons.get(topic)
        last_axes = self.last_axes.get(topic)

        if last_buttons is None or last_axes is None:
            print(f"\n{topic}")
            print(f"  buttons count={len(buttons)} axes count={len(axes)}")
            print(f"  buttons={buttons}")
            print(f"  axes={[round(v, 3) for v in axes]}")
        else:
            button_changes = [
                f"button[{idx}] {old}->{new}"
                for idx, (old, new) in enumerate(zip(last_buttons, buttons, strict=False))
                if old != new
            ]
            axis_changes = [
                f"axis[{idx}] {old:.3f}->{new:.3f}"
                for idx, (old, new) in enumerate(zip(last_axes, axes, strict=False))
                if abs(new - old) >= self.threshold
            ]
            if button_changes or axis_changes:
                print(f"\n{time.strftime('%H:%M:%S')} {topic}")
                for change in button_changes:
                    print(f"  {change}")
                for change in axis_changes:
                    print(f"  {change}")

        self.last_buttons[topic] = buttons
        self.last_axes[topic] = axes


def main() -> None:
    parser = argparse.ArgumentParser(description="Print changed Pico/WebXR Joy button and axis indices.")
    parser.add_argument(
        "--topics",
        nargs="+",
        default=["/xr/controller_left/joy", "/xr/controller_right/joy"],
    )
    parser.add_argument("--threshold", type=float, default=0.05)
    args = parser.parse_args()

    rclpy.init()
    node = JoyMappingPrinter(args.topics, args.threshold)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
