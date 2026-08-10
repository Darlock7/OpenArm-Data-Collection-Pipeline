#!/usr/bin/env python3
"""Task 2: read live joint position, velocity and torque.

Prints a refreshing table of all 14 joints plus the bus health counters that
tell you whether to trust it.

    python scripts/read_joints.py                    # simulated arm, 10 s
    python scripts/read_joints.py --duration 30
    python scripts/read_joints.py --source hardware  # requires Linux + an arm

The hardware path is the same code with a different CANSource. That is the
whole point of the seam in can/source.py.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from openarm_pipeline.can.assembler import JointStateAssembler
from openarm_pipeline.can.mock import MockCANSource
from openarm_pipeline.config import JOINTS

BOLD, DIM, RED, YELLOW, GREEN, RESET = (
    "\033[1m", "\033[2m", "\033[31m", "\033[33m", "\033[32m", "\033[0m"
)


def make_source(kind: str):
    if kind == "mock":
        return MockCANSource()
    if kind == "hardware":
        from openarm_pipeline.can.socketcan import SocketCANSource
        return SocketCANSource()
    raise ValueError(f"unknown source: {kind}")


def render(state, source, assembler, elapsed: float, rate: float) -> str:
    lines = []

    if source.is_mock:
        lines.append(f"{YELLOW}{BOLD}  ** SIMULATED DATA - NOT A REAL ARM **{RESET}")
    else:
        lines.append(f"{GREEN}{BOLD}  LIVE HARDWARE{RESET}")

    lines.append("")
    lines.append(f"{BOLD}  {'joint':<22}{'pos (rad)':>12}{'vel (rad/s)':>14}"
                 f"{'torque (Nm)':>14}{'':>6}{RESET}")
    lines.append(f"  {DIM}{'-' * 68}{RESET}")

    for i, joint in enumerate(JOINTS):
        if i == 7:
            lines.append("")  # visually separate the two arms

        flag = f"{GREEN}ok{RESET}" if state.valid[i] else f"{RED}STALE{RESET}"
        lines.append(
            f"  {joint.name:<22}"
            f"{state.position[i]:>12.4f}"
            f"{state.velocity[i]:>14.4f}"
            f"{state.torque[i]:>14.4f}"
            f"{'':>4}{flag}"
        )

    # Snapshot spread: how far apart the oldest and newest readings in this
    # "instant" actually were. Small is good. This is the number that says
    # whether calling it a single timestamp is honest.
    spread_ms = state.spread_s * 1e3
    spread_col = GREEN if spread_ms < 2 else (YELLOW if spread_ms < 5 else RED)

    lines.append("")
    lines.append(f"  {DIM}{'-' * 68}{RESET}")
    lines.append(
        f"  elapsed {elapsed:6.1f} s"
        f"   snapshots {assembler.snapshots:>7}"
        f"   {rate:7.1f} Hz"
    )
    lines.append(
        f"  frames {assembler.frames_seen:>9}"
        f"   dropped {source.frames_dropped if source.is_mock else 0:>5}"
        f"   bad {assembler.frames_bad:>3}"
        f"   unknown {assembler.frames_unknown:>3}"
    )
    lines.append(
        f"  snapshot spread {spread_col}{spread_ms:5.2f} ms{RESET}"
        f"   {DIM}(time between oldest and newest reading){RESET}"
    )
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["mock", "hardware"], default="mock")
    ap.add_argument("--duration", type=float, default=10.0, help="seconds")
    ap.add_argument("--refresh", type=float, default=10.0, help="display Hz")
    ap.add_argument("--once", action="store_true", help="print one snapshot and exit")
    args = ap.parse_args()

    source = make_source(args.source)
    assembler = JointStateAssembler()

    t_start = time.monotonic()
    last_draw = 0.0
    first = True
    state = None

    try:
        with source:
            for frame in source.frames():
                state = assembler.push(frame)
                if state is None:
                    continue

                if args.once:
                    print(render(state, source, assembler, 0.0, 0.0))
                    return 0

                now = time.monotonic()
                elapsed = now - t_start

                if now - last_draw >= 1.0 / args.refresh:
                    rate = assembler.snapshots / elapsed if elapsed > 0 else 0.0
                    out = render(state, source, assembler, elapsed, rate)
                    if not first:
                        # Redraw in place rather than scrolling.
                        print(f"\033[{len(out.splitlines())}A", end="")
                    print(out)
                    first = False
                    last_draw = now

                if elapsed >= args.duration:
                    break
    except KeyboardInterrupt:
        print("\n  interrupted")
    except RuntimeError as exc:
        print(f"{RED}  {exc}{RESET}", file=sys.stderr)
        return 1

    if state is None:
        print(f"{RED}  no joint states assembled - nothing arrived on the bus{RESET}",
              file=sys.stderr)
        return 1

    elapsed = time.monotonic() - t_start
    print()
    print(f"  {BOLD}summary{RESET}")
    print(f"    duration          {elapsed:.2f} s")
    print(f"    frames decoded    {assembler.frames_seen}")
    print(f"    snapshots         {assembler.snapshots} "
          f"({assembler.snapshots / elapsed:.1f} Hz)")
    if source.is_mock:
        print(f"    frames dropped    {source.frames_dropped} "
              f"({100 * source.frames_dropped / max(1, source.frames_emitted):.2f} %)")
        print()
        print(f"    {YELLOW}source was SIMULATED{RESET}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
