#!/usr/bin/env python3
"""
Drives the A1 admittance calibration tests through the gRPC AdmittanceService.

Tests (see aegis_handoff.md):
    a) static stiffness:  prepare -> hang a known mass -> hold
    b) tracking:          prepare -> track --step 0.002, then --step 0.004
    c) release:           prepare -> hold, deflect the gripper by hand and let go
    d) press on a plate:  prepare -> press (steps until the service clips the reference)

Run inside the container that has `aegis_grpc_client` installed (aegis_gym_toolbx).
Record with PlotJuggler (admittance_calibration_layout.xml) and export CSV for
admittance_calibration_analysis.py. `--log` writes the per-step command log
(wall-clock time, same clock as PlotJuggler's receive time).

Never run admittance_grpc_check.py / admittance_mock_check.py on the real robot.
"""

import argparse
import asyncio
import csv
import sys
import time

import numpy as np
from aegis_grpc_client import AegisRobotClient

AXES = {"x": 0, "y": 1, "z": 2}
MAX_STEP_LIN = 0.004
MAX_DISTANCE = 0.15


class StepLog:
    FIELDS = [
        "t_wall",
        "k",
        "success",
        "clipped",
        "ref_x",
        "ref_y",
        "ref_z",
        "latency_ms",
        "msg",
    ]

    def __init__(self, path: str | None):
        self._file = open(path, "w", newline="") if path else None
        self._writer = csv.writer(self._file) if self._file else None
        if self._writer:
            self._writer.writerow(self.FIELDS)

    def write(self, k, success, clipped, reference, latency_s, msg):
        if self._writer:
            self._writer.writerow(
                [
                    f"{time.time():.6f}",
                    k,
                    int(success),
                    int(clipped),
                    *(f"{v:.6f}" for v in reference[:3]),
                    f"{latency_s * 1e3:.2f}",
                    msg,
                ]
            )

    def close(self):
        if self._file:
            self._file.close()


async def sleep_until(deadline: float) -> None:
    await asyncio.sleep(max(0.0, deadline - time.perf_counter()))


async def step(client, log, k, translation):
    t0 = time.perf_counter()
    success, msg, clipped, reference = await client.admittance_step(
        translation=translation
    )
    latency = time.perf_counter() - t0
    log.write(k, success, clipped, reference, latency, msg)
    if not success:
        print(f"[{k}] rejected: {msg}", file=sys.stderr)
    return success, clipped, reference, latency


async def cmd_prepare(client, args):
    try:
        await client.servo_disable()
    except RuntimeError:
        # ServoDisable fails its controller switch when Servo is already off; Enable re-checks anyway.
        pass
    print("disable:", await client.admittance_disable())
    print("tare:", await client.wrench_bias_set())
    await asyncio.sleep(0.5)
    print("wrench after tare:", np.round(await client.get_wrench(), 3))
    print("enable:", await client.admittance_enable())


async def cmd_disable(client, args):
    print("disable:", await client.admittance_disable())


async def cmd_hold(client, args):
    start = await client.get_tcp_pose()
    deadline = time.perf_counter()
    end = deadline + args.duration
    while deadline < end:
        pose = await client.get_tcp_pose()
        wrench = await client.admittance_get_wrench()
        d = (pose[:3] - start[:3]) * 1e3
        print(
            f"dTCP [mm] {d[0]:+7.2f} {d[1]:+7.2f} {d[2]:+7.2f} | F [N] "
            f"{wrench[0]:+6.2f} {wrench[1]:+6.2f} {wrench[2]:+6.2f}",
            end="\r",
        )
        deadline += args.period
        await sleep_until(deadline)
    print()


async def cmd_track(client, args, log):
    direction = np.zeros(3)
    direction[AXES[args.axis]] = 1.0 if args.sign > 0 else -1.0
    n = int(round(args.distance / args.step))
    legs = [direction, -direction] if args.back else [direction]
    k = 0
    latencies = []
    for leg_idx, leg in enumerate(legs):
        if leg_idx:
            await asyncio.sleep(args.pause)
        deadline = time.perf_counter()
        for _ in range(n):
            _, clipped, _, latency = await step(client, log, k, leg * args.step)
            latencies.append(latency)
            if clipped:
                print(
                    f"[{k}] clipped: the arm lags the reference by more than max_offset_lin"
                )
            k += 1
            deadline += args.period
            await sleep_until(deadline)
    lat = np.array(latencies) * 1e3
    print(
        f"{k} steps, {args.step / args.period * 1e2:.1f} cm/s; call latency [ms] "
        f"median {np.median(lat):.1f}, p95 {np.percentile(lat, 95):.1f}, max {lat.max():.1f}"
    )


async def cmd_press(client, args, log):
    direction = np.zeros(3)
    direction[AXES[args.axis]] = 1.0 if args.sign > 0 else -1.0
    start_pose = await client.get_tcp_pose()
    n_max = int(round(args.max_distance / args.step))
    consecutive = 0
    k = 0
    reason = "max distance"
    deadline = time.perf_counter()
    while k < n_max:
        _, clipped, _, _ = await step(client, log, k, direction * args.step)
        consecutive = consecutive + 1 if clipped else 0
        k += 1
        force = np.linalg.norm((await client.admittance_get_wrench())[:3])
        if force > args.max_force:
            reason = f"|F| {force:.1f} N > --max-force"
            break
        if consecutive >= args.clipped_steps:
            reason = "clipped"
            break
        deadline += args.period
        await sleep_until(deadline)
    wrench = await client.admittance_get_wrench()
    print(f"stopped after {k} steps ({reason}), F = {np.round(wrench[:3], 2)} N")

    await asyncio.sleep(args.hold)

    # Retreat at the approach speed: SetTCPTarget alone would move at the service limit (10 cm/s).
    reference = (await client.admittance_step(translation=np.zeros(3)))[3]
    deadline = time.perf_counter()
    for _ in range(4 * n_max + 50):
        to_start = start_pose[:3] - reference[:3]
        distance = np.linalg.norm(to_start)
        if distance < 5e-4:
            break
        success, _, reference, _ = await step(
            client, log, k, to_start / distance * min(args.step, distance)
        )
        k += 1
        if not success:
            break
        deadline += args.period
        await sleep_until(deadline)
    print("retreated to the start reference")


async def main(args):
    client = AegisRobotClient(server_address=args.server)
    await client.connect()
    log = StepLog(getattr(args, "log", None))
    try:
        match args.command:
            case "prepare":
                await cmd_prepare(client, args)
            case "disable":
                await cmd_disable(client, args)
            case "hold":
                await cmd_hold(client, args)
            case "track":
                await cmd_track(client, args, log)
            case "press":
                await cmd_press(client, args, log)
    finally:
        log.close()
        await client.disconnect()


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--server", default="127.0.0.1:50051")
    sub = p.add_subparsers(dest="command", required=True)

    sub.add_parser(
        "prepare", help="Disable chain, tare F/T (chain inactive), enable chain."
    )
    sub.add_parser(
        "disable", help="Relax the reference and return to the MoveIt controller."
    )

    hold = sub.add_parser(
        "hold", help="No commands; print TCP displacement and wrench (tests a, c)."
    )
    hold.add_argument("--duration", type=float, default=30.0)
    hold.add_argument("--period", type=float, default=0.1)

    for name, help_ in (
        ("track", "Fixed-rate steps in free space (test b)."),
        ("press", "Step towards a surface until the reference is clipped (test d)."),
    ):
        c = sub.add_parser(name, help=help_)
        c.add_argument("--axis", choices=AXES, required=True, help="world frame axis")
        c.add_argument("--sign", type=int, choices=(-1, 1), default=1)
        c.add_argument("--step", type=float, default=0.002, help="[m] per call")
        c.add_argument(
            "--period", type=float, default=0.04, help="[s] between calls (25 Hz)"
        )
        c.add_argument("--log", help="CSV path for the per-step command log")
    track = sub.choices["track"]
    track.add_argument("--distance", type=float, default=0.05, help="[m] per leg")
    track.add_argument("--back", action="store_true", help="return along the same line")
    track.add_argument("--pause", type=float, default=1.0, help="[s] between legs")
    press = sub.choices["press"]
    press.add_argument(
        "--max-distance", type=float, default=0.05, help="[m] give up without contact"
    )
    press.add_argument(
        "--clipped-steps",
        type=int,
        default=10,
        help="consecutive clipped steps = contact",
    )
    press.add_argument(
        "--hold", type=float, default=2.0, help="[s] in contact before retreating"
    )
    press.add_argument(
        "--max-force",
        type=float,
        default=8.0,
        help="[N] stop stepping above this force",
    )

    args = p.parse_args()
    if hasattr(args, "step") and not 0.0 < args.step <= MAX_STEP_LIN:
        p.error(f"--step must be in (0, {MAX_STEP_LIN}] m")
    if (
        max(getattr(args, "distance", 0.0), getattr(args, "max_distance", 0.0))
        > MAX_DISTANCE
    ):
        p.error(f"distance limited to {MAX_DISTANCE} m")
    return args


if __name__ == "__main__":
    asyncio.run(main(parse_args()))
