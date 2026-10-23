#!/usr/bin/env python

import argparse
import time

import can


DISABLE_FRAME = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]


def main() -> None:
    parser = argparse.ArgumentParser(description="Send raw Damiao disable frames without handshake.")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--interface", default="socketcan")
    parser.add_argument("--ids", nargs="+", type=lambda x: int(x, 0), default=list(range(1, 9)))
    parser.add_argument("--repeat", type=int, default=10)
    parser.add_argument("--sleep-s", type=float, default=0.02)
    parser.add_argument("--can-fd", action="store_true", default=True)
    args = parser.parse_args()

    bus = can.Bus(interface=args.interface, channel=args.channel, fd=args.can_fd)
    try:
        for repeat_idx in range(args.repeat):
            for motor_id in args.ids:
                msg = can.Message(
                    arbitration_id=motor_id,
                    data=DISABLE_FRAME,
                    is_extended_id=False,
                    is_fd=args.can_fd,
                )
                bus.send(msg)
                time.sleep(args.sleep_s)
            print(f"disable pass {repeat_idx + 1}/{args.repeat}: ids={args.ids}")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
