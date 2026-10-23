#!/usr/bin/env python3
import argparse
import time
from dataclasses import dataclass
from typing import Optional, Dict, List

import can
import pandas as pd


@dataclass
class MitRange:
    p_min: float
    p_max: float
    v_min: float
    v_max: float
    t_min: float
    t_max: float


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    max_int = (1 << bits) - 1
    return float(x_int) * span / float(max_int) + x_min


def decode_damiao_feedback(data: bytes, r: MitRange) -> Optional[Dict]:
    """
    DAMIAO MIT-style feedback frame decoder.

    Assumed layout from the motor manual:
      D0: ID | ERR << 4
      D1: POS[15:8]
      D2: POS[7:0]
      D3: VEL[11:4]
      D4: VEL[3:0] | T[11:8]
      D5: T[7:0]
      D6: T_MOS
      D7: T_Rotor

    Note:
      The exact ID/ERR packing can differ by firmware/config.
      Verify decoded ID and ERR against the vendor debug tool.
    """
    if len(data) != 8:
        return None

    b = list(data)

    # Common DAMIAO/MIT style packing.
    # If your IDs are larger than 15, verify this part with raw frames/debug tool.
    motor_id = b[0] & 0x0F
    err_code = (b[0] >> 4) & 0x0F

    p_int = (b[1] << 8) | b[2]
    v_int = (b[3] << 4) | (b[4] >> 4)
    t_int = ((b[4] & 0x0F) << 8) | b[5]

    pos = uint_to_float(p_int, r.p_min, r.p_max, 16)
    vel = uint_to_float(v_int, r.v_min, r.v_max, 12)
    tau = uint_to_float(t_int, r.t_min, r.t_max, 12)

    return {
        "motor_id": motor_id,
        "err_code": err_code,
        "pos": pos,
        "vel": vel,
        "torque": tau,
        "t_mos": b[6],
        "t_rotor": b[7],
        "p_int": p_int,
        "v_int": v_int,
        "t_int": t_int,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--out", default="damiao_feedback_check.csv")

    # These ranges MUST match the motor configuration in the vendor debug tool.
    parser.add_argument("--p-min", type=float, default=-12.5)
    parser.add_argument("--p-max", type=float, default=12.5)
    parser.add_argument("--v-min", type=float, default=-30.0)
    parser.add_argument("--v-max", type=float, default=30.0)
    parser.add_argument("--t-min", type=float, default=-12.0)
    parser.add_argument("--t-max", type=float, default=12.0)

    parser.add_argument(
        "--accept-can-id",
        type=lambda x: int(x, 0),
        default=None,
        help="Optional CAN ID filter, e.g. 0x000. Leave empty to decode all 8-byte frames.",
    )

    args = parser.parse_args()

    r = MitRange(
        p_min=args.p_min,
        p_max=args.p_max,
        v_min=args.v_min,
        v_max=args.v_max,
        t_min=args.t_min,
        t_max=args.t_max,
    )

    bus = can.interface.Bus(
        channel=args.channel,
        interface="socketcan",
        bitrate=args.bitrate,
    )

    print(f"[INFO] Listening on {args.channel} for {args.duration:.1f} s")
    print("[INFO] This script does not send any CAN command.")

    rows: List[Dict] = []
    t0 = time.time()

    while time.time() - t0 < args.duration:
        msg = bus.recv(timeout=1.0)
        if msg is None:
            continue

        if args.accept_can_id is not None and msg.arbitration_id != args.accept_can_id:
            continue

        if msg.is_error_frame or msg.is_remote_frame:
            continue

        if msg.dlc != 8:
            continue

        decoded = decode_damiao_feedback(bytes(msg.data), r)
        if decoded is None:
            continue

        row = {
            "host_time": time.time(),
            "can_id": msg.arbitration_id,
            "dlc": msg.dlc,
            "raw_hex": bytes(msg.data).hex(" "),
            **decoded,
        }
        rows.append(row)

        print(
            f"can_id=0x{msg.arbitration_id:X} "
            f"id={decoded['motor_id']:02d} "
            f"err={decoded['err_code']:X} "
            f"pos={decoded['pos']:+.4f} rad "
            f"vel={decoded['vel']:+.4f} rad/s "
            f"tau={decoded['torque']:+.4f} Nm "
            f"Tmos={decoded['t_mos']} "
            f"Trotor={decoded['t_rotor']}"
        )

    if not rows:
        print("[FAIL] No decodable 8-byte feedback frame was received.")
        return

    df = pd.DataFrame(rows)
    df.to_csv(args.out, index=False)

    print(f"\n[OK] Saved: {args.out}")
    print("\n[SUMMARY]")
    print(f"frames: {len(df)}")
    print(f"unique CAN IDs: {sorted(df['can_id'].unique())}")
    print(f"unique motor IDs: {sorted(df['motor_id'].unique())}")
    print("\nTorque statistics by motor_id:")
    print(df.groupby("motor_id")["torque"].agg(["count", "mean", "std", "min", "max"]))

    print("\nVelocity statistics by motor_id:")
    print(df.groupby("motor_id")["vel"].agg(["count", "mean", "std", "min", "max"]))

    print("\nERR code counts:")
    print(df.groupby(["motor_id", "err_code"]).size())


if __name__ == "__main__":
    main()