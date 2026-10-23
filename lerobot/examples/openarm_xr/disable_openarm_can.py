#!/usr/bin/env python

import argparse
import logging

from lerobot.robots.openarm_follower.config_openarm_follower import OpenArmFollowerConfig
from lerobot.robots.openarm_follower.openarm_follower import OpenArmFollower
from lerobot.utils.utils import init_logging


def main() -> None:
    parser = argparse.ArgumentParser(description="Connect to an OpenArm CAN follower and disable torque.")
    parser.add_argument("--id", default="openarm_can0")
    parser.add_argument("--port", default="can0")
    parser.add_argument("--side", choices=["left", "right"], default="right")
    parser.add_argument("--retries", type=int, default=8)
    args = parser.parse_args()

    init_logging()
    cfg = OpenArmFollowerConfig(
        id=args.id,
        port=args.port,
        side=args.side,
        disable_torque_on_disconnect=True,
        disable_torque_on_disconnect_retries=args.retries,
    )
    robot = OpenArmFollower(cfg)
    try:
        robot.connect()
        logging.info("Sending disable_torque with %d retries.", args.retries)
        robot.bus.disable_torque(num_retry=args.retries)
    finally:
        if robot.is_connected:
            robot.disconnect()
    print(f"disable_torque sent for {args.side} arm on {args.port}.")


if __name__ == "__main__":
    main()
