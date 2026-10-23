#!/usr/bin/env python3
"""Read-only SocketCAN/CAN-FD probe for OpenArm Damiao feedback."""

import argparse
import csv
import math
import time
from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import can


OPENARM_DEFAULT_SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
OPENARM_DEFAULT_RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18]
OPENARM_DEFAULT_TYPES = [
    "DM8009",
    "DM8009",
    "DM4340",
    "DM4340",
    "DM4310",
    "DM4310",
    "DM4310",
    "DM4310",
]

CSV_FIELDS = [
    "host_time",
    "elapsed_s",
    "channel",
    "arbitration_id",
    "arbitration_id_hex",
    "is_extended_id",
    "is_remote_frame",
    "is_error_frame",
    "is_fd",
    "bitrate_switch",
    "error_state_indicator",
    "dlc",
    "data_len",
    "raw_hex",
    "damiao_classic_valid",
    "damiao_classic_motor_id",
    "damiao_classic_err_code",
    "damiao_classic_position_rad",
    "damiao_classic_velocity_rad_s",
    "damiao_classic_torque_nm",
    "damiao_classic_t_mos",
    "damiao_classic_t_rotor",
    "openarm_valid",
    "openarm_send_can_id",
    "openarm_recv_can_id",
    "openarm_motor_type",
    "openarm_position_rad",
    "openarm_velocity_rad_s",
    "openarm_torque_nm",
    "openarm_t_mos",
    "openarm_t_rotor",
]


@dataclass(frozen=True)
class MitRange:
    p_min: float
    p_max: float
    v_min: float
    v_max: float
    t_min: float
    t_max: float


def parse_int_list(text: str) -> List[int]:
    return [int(item.strip(), 0) for item in text.split(",") if item.strip()]


def parse_str_list(text: str) -> List[str]:
    return [item.strip() for item in text.split(",") if item.strip()]


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    max_int = (1 << bits) - 1
    return float(x_int) * span / float(max_int) + x_min


def decode_damiao_classic(data: bytes, limits: MitRange) -> Optional[Dict[str, float]]:
    if len(data) != 8:
        return None

    b = list(data)
    p_int = (b[1] << 8) | b[2]
    v_int = (b[3] << 4) | (b[4] >> 4)
    t_int = ((b[4] & 0x0F) << 8) | b[5]

    return {
        "motor_id": b[0] & 0x0F,
        "err_code": (b[0] >> 4) & 0x0F,
        "position": uint_to_float(p_int, limits.p_min, limits.p_max, 16),
        "velocity": uint_to_float(v_int, limits.v_min, limits.v_max, 12),
        "torque": uint_to_float(t_int, limits.t_min, limits.t_max, 12),
        "t_mos": b[6],
        "t_rotor": b[7],
    }


class OpenArmDecoder:
    def __init__(self, motor_types: Sequence[str], send_ids: Sequence[int], recv_ids: Sequence[int]):
        try:
            import openarm_can as oa
        except Exception as exc:
            raise RuntimeError(f"failed to import openarm_can: {exc}") from exc

        if not (len(motor_types) == len(send_ids) == len(recv_ids)):
            raise ValueError("--openarm-motor-types, --openarm-send-ids, and --openarm-recv-ids differ")

        self._oa = oa
        self._motors = {}
        for type_name, send_id, recv_id in zip(motor_types, send_ids, recv_ids):
            motor_type = getattr(oa.MotorType, type_name)
            motor = oa.Motor(motor_type, send_id, recv_id)
            self._motors[recv_id] = (type_name, send_id, recv_id, motor)

    def decode(self, arbitration_id: int, data: bytes) -> Optional[Dict[str, object]]:
        entry = self._motors.get(arbitration_id)
        if entry is None or len(data) < 8:
            return None

        type_name, send_id, recv_id, motor = entry
        result = self._oa.CanPacketDecoder.parse_motor_state_data(motor, list(data))
        if not getattr(result, "valid", False):
            return None

        return {
            "motor_type": type_name,
            "send_can_id": send_id,
            "recv_can_id": recv_id,
            "position": result.position,
            "velocity": result.velocity,
            "torque": result.torque,
            "t_mos": result.t_mos,
            "t_rotor": result.t_rotor,
        }


def format_ids(ids: Iterable[int]) -> str:
    return ", ".join(f"0x{can_id:X}" for can_id in sorted(ids))


def make_row(msg: can.Message, start_time: float, channel: str) -> Dict[str, object]:
    host_time = time.time()
    data = bytes(msg.data)
    return {
        "host_time": f"{host_time:.9f}",
        "elapsed_s": f"{host_time - start_time:.9f}",
        "channel": getattr(msg, "channel", None) or channel,
        "arbitration_id": msg.arbitration_id,
        "arbitration_id_hex": f"0x{msg.arbitration_id:X}",
        "is_extended_id": bool(msg.is_extended_id),
        "is_remote_frame": bool(msg.is_remote_frame),
        "is_error_frame": bool(msg.is_error_frame),
        "is_fd": bool(msg.is_fd),
        "bitrate_switch": bool(getattr(msg, "bitrate_switch", False)),
        "error_state_indicator": bool(getattr(msg, "error_state_indicator", False)),
        "dlc": msg.dlc,
        "data_len": len(data),
        "raw_hex": data.hex(" "),
        "damiao_classic_valid": False,
        "damiao_classic_motor_id": "",
        "damiao_classic_err_code": "",
        "damiao_classic_position_rad": "",
        "damiao_classic_velocity_rad_s": "",
        "damiao_classic_torque_nm": "",
        "damiao_classic_t_mos": "",
        "damiao_classic_t_rotor": "",
        "openarm_valid": False,
        "openarm_send_can_id": "",
        "openarm_recv_can_id": "",
        "openarm_motor_type": "",
        "openarm_position_rad": "",
        "openarm_velocity_rad_s": "",
        "openarm_torque_nm": "",
        "openarm_t_mos": "",
        "openarm_t_rotor": "",
    }


def write_csv(path: str, rows: Sequence[Dict[str, object]]) -> None:
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(rows: Sequence[Dict[str, object]], duration_s: float) -> None:
    elapsed = max(duration_s, 1e-9)
    id_counts = Counter(int(row["arbitration_id"]) for row in rows)
    dlc_by_id: Dict[int, Counter] = defaultdict(Counter)
    len_counts = Counter()
    fd_counts = Counter()

    for row in rows:
        can_id = int(row["arbitration_id"])
        dlc_by_id[can_id][int(row["dlc"])] += 1
        len_counts[int(row["data_len"])] += 1
        fd_counts[bool(row["is_fd"])] += 1

    print("\n[SUMMARY]")
    print(f"frames: {len(rows)}")
    print(f"unique arbitration IDs: {format_ids(id_counts)}" if id_counts else "unique arbitration IDs: none")
    print(f"CAN-FD frames: {fd_counts[True]}")
    print(f"classic CAN frames: {fd_counts[False]}")
    print(f"data length distribution: {dict(sorted(len_counts.items()))}")

    if id_counts:
        print("\nPer-ID frame rate and DLC distribution:")
        for can_id in sorted(id_counts):
            rate_hz = id_counts[can_id] / elapsed
            print(f"  0x{can_id:X}: count={id_counts[can_id]} rate={rate_hz:.3f} Hz dlc={dict(dlc_by_id[can_id])}")

    decoded_openarm = sum(1 for row in rows if row["openarm_valid"])
    decoded_classic = sum(1 for row in rows if row["damiao_classic_valid"])
    if decoded_openarm:
        print(f"\nOpenArm decoder valid frames: {decoded_openarm}")
    if decoded_classic:
        print(f"Damiao classic decoder valid frames: {decoded_classic}")


def add_decodes(
    row: Dict[str, object],
    data: bytes,
    limits: MitRange,
    try_damiao_classic: bool,
    openarm_decoder: Optional[OpenArmDecoder],
) -> None:
    if try_damiao_classic:
        classic = decode_damiao_classic(data, limits)
        if classic is not None:
            row.update(
                {
                    "damiao_classic_valid": True,
                    "damiao_classic_motor_id": classic["motor_id"],
                    "damiao_classic_err_code": classic["err_code"],
                    "damiao_classic_position_rad": f"{classic['position']:.12g}",
                    "damiao_classic_velocity_rad_s": f"{classic['velocity']:.12g}",
                    "damiao_classic_torque_nm": f"{classic['torque']:.12g}",
                    "damiao_classic_t_mos": classic["t_mos"],
                    "damiao_classic_t_rotor": classic["t_rotor"],
                }
            )

    if openarm_decoder is not None:
        decoded = openarm_decoder.decode(int(row["arbitration_id"]), data)
        if decoded is not None:
            row.update(
                {
                    "openarm_valid": True,
                    "openarm_send_can_id": f"0x{decoded['send_can_id']:X}",
                    "openarm_recv_can_id": f"0x{decoded['recv_can_id']:X}",
                    "openarm_motor_type": decoded["motor_type"],
                    "openarm_position_rad": f"{decoded['position']:.12g}",
                    "openarm_velocity_rad_s": f"{decoded['velocity']:.12g}",
                    "openarm_torque_nm": f"{decoded['torque']:.12g}",
                    "openarm_t_mos": decoded["t_mos"],
                    "openarm_t_rotor": decoded["t_rotor"],
                }
            )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only SocketCAN/CAN-FD frame probe for OpenArm feedback."
    )
    parser.add_argument("--channel", default="can0", help="SocketCAN channel, e.g. can0 or can1")
    parser.add_argument("--duration", type=float, default=10.0, help="Capture duration in seconds")
    parser.add_argument("--out", default="feedback_probe.csv", help="CSV output path")
    parser.add_argument("--recv-timeout", type=float, default=0.2, help="python-can recv timeout")
    parser.add_argument("--print-frames", action="store_true", help="Print every received frame")
    parser.add_argument(
        "--no-fd-socket",
        action="store_true",
        help="Do not enable CAN-FD receive support on the SocketCAN raw socket.",
    )

    parser.add_argument("--try-damiao-classic-decode", action="store_true")
    parser.add_argument("--p-min", type=float, default=-12.5)
    parser.add_argument("--p-max", type=float, default=12.5)
    parser.add_argument("--v-min", type=float, default=-30.0)
    parser.add_argument("--v-max", type=float, default=30.0)
    parser.add_argument("--t-min", type=float, default=-10.0)
    parser.add_argument("--t-max", type=float, default=10.0)

    parser.add_argument("--use-openarm-decoder", action="store_true")
    parser.add_argument("--openarm-send-ids", default=",".join(f"0x{x:X}" for x in OPENARM_DEFAULT_SEND_IDS))
    parser.add_argument("--openarm-recv-ids", default=",".join(f"0x{x:X}" for x in OPENARM_DEFAULT_RECV_IDS))
    parser.add_argument("--openarm-motor-types", default=",".join(OPENARM_DEFAULT_TYPES))

    args = parser.parse_args()

    limits = MitRange(args.p_min, args.p_max, args.v_min, args.v_max, args.t_min, args.t_max)
    openarm_decoder = None
    if args.use_openarm_decoder:
        try:
            openarm_decoder = OpenArmDecoder(
                parse_str_list(args.openarm_motor_types),
                parse_int_list(args.openarm_send_ids),
                parse_int_list(args.openarm_recv_ids),
            )
            print("[INFO] OpenArm decoder enabled.")
        except Exception as exc:
            print(f"[WARN] OpenArm decoder unavailable, falling back to raw logging only: {exc}")

    print(f"[INFO] Listening on {args.channel} for {args.duration:.3f} s")
    print("[INFO] This script is read-only and sends no CAN command.")

    rows: List[Dict[str, object]] = []
    start_time = time.time()
    end_time = start_time + args.duration

    with can.interface.Bus(
        channel=args.channel,
        interface="socketcan",
        fd=not args.no_fd_socket,
    ) as bus:
        while time.time() < end_time:
            msg = bus.recv(timeout=args.recv_timeout)
            if msg is None:
                continue

            row = make_row(msg, start_time, args.channel)
            data = bytes(msg.data)
            add_decodes(row, data, limits, args.try_damiao_classic_decode, openarm_decoder)
            rows.append(row)

            if args.print_frames:
                frame_type = "FD" if row["is_fd"] else "CAN"
                print(
                    f"{row['elapsed_s']} {row['channel']} {frame_type} "
                    f"id={row['arbitration_id_hex']} dlc={row['dlc']} "
                    f"len={row['data_len']} data={row['raw_hex']}"
                )

    capture_time = time.time() - start_time
    write_csv(args.out, rows)
    print(f"[OK] Saved CSV: {args.out}")
    print_summary(rows, capture_time)

    if not rows:
        print("[FAIL] No raw CAN/CAN-FD frame was received.")
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
