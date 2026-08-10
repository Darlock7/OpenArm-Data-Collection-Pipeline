#!/usr/bin/env python3
"""Task 3 - capture five streams at once and measure how well they align.

    python scripts/demo_sync.py                 # 5 s
    python scripts/demo_sync.py --duration 20
    python scripts/demo_sync.py --policy nearest

Runs the simulated arm at 1 kHz alongside four simulated cameras at 60/30/30/15
fps, each with its own clock drift, jitter and dropped frames, then aligns them
and reports the timing error. The numbers that matter are BIAS and JITTER, not
the average absolute error - see openarm_pipeline/cameras/sync.py for why.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from openarm_pipeline.can.assembler import JointStateAssembler
from openarm_pipeline.can.mock import MockCANSource
from openarm_pipeline.cameras.mock import make_mock_cameras
from openarm_pipeline.cameras.source import CameraStream, JointStream
from openarm_pipeline.cameras.sync import align

BOLD, DIM, RESET = "\033[1m", "\033[2m", "\033[0m"
GREEN, YELLOW, RED, CYAN = "\033[32m", "\033[33m", "\033[31m", "\033[36m"


def capture(duration: float, scale: float):
    """Run arm and cameras concurrently, each in its own thread."""
    can = MockCANSource()
    cams = make_mock_cameras(scale=scale)

    states, frames = [], {c.name: [] for c in cams}
    stop = threading.Event()

    def run_can():
        asm = JointStateAssembler()
        with can:
            for frame in can.frames():
                s = asm.push(frame)
                if s is not None:
                    states.append(s)
                if stop.is_set():
                    break

    def run_cam(cam):
        with cam:
            for f in cam.frames():
                frames[cam.name].append(f)
                if stop.is_set():
                    break

    threads = [threading.Thread(target=run_can, daemon=True)]
    threads += [threading.Thread(target=run_cam, args=(c,), daemon=True) for c in cams]
    for t in threads:
        t.start()

    time.sleep(duration)
    stop.set()
    for c in cams:
        c.close()
    can.close()
    for t in threads:
        t.join(timeout=2.0)

    joints = JointStream(
        t_mono_ns=np.array([s.t_mono_ns for s in states], dtype=np.int64),
        position=np.stack([s.position for s in states]),
        velocity=np.stack([s.velocity for s in states]),
        torque=np.stack([s.torque for s in states]),
        valid=np.stack([s.valid for s in states]),
        spread_s=np.array([s.spread_s for s in states]),
    )
    streams = {n: CameraStream.from_frames(n, f, keep_images=False)
               for n, f in frames.items() if f}
    return joints, streams, cams, can


def timeline_art(joints: JointStream, streams: dict, width: int = 74) -> str:
    """First 200 ms of every stream, so the rate mismatch is visible."""
    t0 = joints.t_mono_ns[0]
    span_ns = 200_000_000

    def row(label, times, mark):
        line = [" "] * width
        for t in times:
            off = t - t0
            if 0 <= off < span_ns:
                line[int(off / span_ns * (width - 1))] = mark
        return f"  {label:<13}{''.join(line)}"

    out = [f"  {DIM}first 200 ms of each stream{RESET}", ""]
    out.append(row("joints 1kHz", joints.t_mono_ns, "|"))
    for name, s in streams.items():
        out.append(row(name, s.t_mono_ns, "o"))
    out.append(f"  {DIM}{'':<13}{'0 ms':<{width-8}}200 ms{RESET}")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--policy", default="hermite",
                    choices=["hermite", "nearest", "window_mean"])
    ap.add_argument("--scale", type=float, default=0.125,
                    help="image render scale, to keep the demo cheap")
    args = ap.parse_args()

    print(f"\n  {YELLOW}{BOLD}** SIMULATED DATA - NOT A REAL ARM OR REAL CAMERAS **{RESET}\n")
    print(f"  capturing {args.duration:.0f} s ...", flush=True)

    joints, streams, cams, can = capture(args.duration, args.scale)
    if len(joints) == 0 or not streams:
        print(f"{RED}  nothing captured{RESET}", file=sys.stderr)
        return 1

    # ---- what was captured ----------------------------------------------
    print(f"\n  {BOLD}captured{RESET}")
    print(f"  {DIM}{'stream':<14}{'samples':>9}{'nominal':>10}{'measured':>11}"
          f"{'dropped':>9}{'drift':>10}{RESET}")
    print(f"  {DIM}{'-' * 63}{RESET}")
    print(f"  {'joints':<14}{len(joints):>9}{'1000 Hz':>10}"
          f"{joints.measured_rate_hz:>10.1f}{'':>10}{'':>10}")
    for c in cams:
        s = streams.get(c.name)
        if s is None:
            continue
        print(f"  {c.name:<14}{len(s):>9}{c.spec.fps:>9.0f} f{s.measured_fps:>10.2f}"
              f"{s.dropped:>9}{c.drift_ppm:>9.0f}p")

    print()
    print(timeline_art(joints, streams))

    # ---- alignment -------------------------------------------------------
    ep = align(joints, streams, joint_policy=args.policy)
    q = ep.quality()

    print(f"\n  {BOLD}alignment{RESET}  "
          f"{DIM}anchor={q['anchor']}  policy={q['joint_policy']}  "
          f"{q['samples']} samples{RESET}")
    print(f"  {DIM}{'stream':<14}{'bias':>10}{'jitter':>10}{'worst':>10}"
          f"{'usable':>9}{RESET}")
    print(f"  {DIM}{'-' * 53}{RESET}")

    j = q["joints"]
    print(f"  {'joints':<14}{j['bias_ms']:>9.3f}m{j['jitter_ms']:>9.3f}m"
          f"{j['worst_ms']:>9.3f}m{j['coverage'] * 100:>8.1f}%")
    for name, s in q["cameras"].items():
        colour = GREEN if abs(s["bias_ms"]) < 1 else (YELLOW if abs(s["bias_ms"]) < 20 else RED)
        star = "  <- anchor" if name == q["anchor"] else ""
        print(f"  {name:<14}{colour}{s['bias_ms']:>9.3f}m{RESET}{s['jitter_ms']:>9.3f}m"
              f"{s['worst_ms']:>9.3f}m{s['coverage'] * 100:>8.1f}%{DIM}{star}{RESET}")

    print(f"\n  {DIM}bias   = mean signed error. systematic offset. the number that"
          f" damages a policy.{RESET}")
    print(f"  {DIM}jitter = its standard deviation. unlearnable, so worse per ms"
          f" than bias.{RESET}")
    print(f"  {DIM}anchor camera reads exactly 0 by construction: every sample sits"
          f" on a real frame.{RESET}")

    # ---- policy comparison, against ground truth -------------------------
    # The simulated arm is an analytic function of time, so the TRUE joint
    # position at any instant is known exactly. That makes it possible to
    # measure how wrong each interpolation policy actually is, rather than
    # only asserting that one should be better.
    truth = np.array([[can._trajectory(j, t)[0] for j in range(joints.position.shape[1])]
                      for t in (ep.t_mono_ns / 1e9 - can._t_start)])

    print(f"\n  {BOLD}joint policy comparison{RESET}  "
          f"{DIM}(same raw data, three views, scored against the analytic truth){RESET}")
    print(f"  {DIM}{'policy':<14}{'synthetic':>11}{'mean err':>13}{'worst err':>13}{RESET}")
    print(f"  {DIM}{'-' * 51}{RESET}")

    best = None
    for pol in ("nearest", "hermite", "window_mean"):
        e = align(joints, streams, joint_policy=pol)
        err = np.abs(e.position[e.joint_valid] - truth[e.joint_valid])
        mean_e, worst_e = err.mean(), err.max()
        best = mean_e if best is None else min(best, mean_e)
        tag = "yes" if e.joint_synthetic else "no"
        mark = f"  {GREEN}<- best{RESET}" if mean_e == best and pol != "nearest" else ""
        print(f"  {pol:<14}{tag:>11}{mean_e * 1e3:>12.4f}m{worst_e * 1e3:>12.4f}m{mark}")

    print(f"\n  {DIM}error in millirad against the arm's true position at each"
          f" timestamp.{RESET}")
    print(f"  {DIM}'nearest' returns only real measurements but lands up to half a"
          f" sample early{RESET}")
    print(f"  {DIM}or late. 'hermite' uses the reported velocity as the slope at each"
          f" endpoint.{RESET}")
    print(f"  {DIM}'window_mean' trades accuracy for anti-aliasing, and is the right"
          f" choice only{RESET}")
    print(f"  {DIM}when downsampling far below the sampling rate.{RESET}")
    print(f"\n  {DIM}Note the margin is small. At 1 kHz the joints are ~50x"
          f" oversampled relative to{RESET}")
    print(f"  {DIM}human motion, so the residual is dominated by TIMESTAMP jitter"
          f" rather than by{RESET}")
    print(f"  {DIM}interpolation error, and no method can recover what the clock"
          f" got wrong. The{RESET}")
    print(f"  {DIM}choice matters far more for lower-rate streams: see"
          f" tests/test_sync.py, where{RESET}")
    print(f"  {DIM}hermite beats linear by 5x on a signal sampled every 20 ms.{RESET}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
