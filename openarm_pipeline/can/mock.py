"""A simulated OpenArm, emitting real Damiao-format frames on a fake bus.

>>> THIS IS SIMULATED DATA. NOT A REAL ARM. <<<

Every episode recorded from this source is tagged `is_mock=True` in its
metadata and its filename, and the dashboard shows a banner. That labelling
is deliberate and load-bearing: the failure mode I am guarding against is
synthetic data being mistaken for a real demonstration later.

WHAT IS FAITHFUL HERE
  * Frames are real Damiao MIT-mode bytes, produced by protocol.encode_feedback
    and parsed back by the same decoder the hardware path uses. The decoder is
    therefore exercised for real, not bypassed.
  * Motors are polled round-robin at CONTROL_RATE_HZ, one frame per motor, as
    on a real bus. A full-arm snapshot is never atomic.
  * Timing jitter and occasional dropped frames are injected, because a
    pipeline that only ever sees perfectly regular data hides its own bugs.

WHAT IS NOT
  * The motion is a sum of sinusoids, not a real teleoperation demonstration
    and not the output of any dynamics model.
  * Torque is a plausible-looking gravity-plus-inertia stand-in, not the real
    joint torque of this arm.
  * Bus arbitration, electrical faults and controller latency are absent.
"""

from __future__ import annotations

import math
import random
from collections.abc import Iterator

from ..clock import RateLimiter, monotonic_ns
from ..config import CONTROL_RATE_HZ, JOINTS, MOTOR_SPECS
from .protocol import JointFeedback, encode_feedback
from .source import CANFrame, CANSource


class MockCANSource(CANSource):
    """Synthesises frames for all 14 joints across can0 and can1."""

    def __init__(self, rate_hz: float = CONTROL_RATE_HZ,
                 drop_rate: float = 0.001, jitter_s: float = 0.0002,
                 seed: int = 0):
        """
        drop_rate: fraction of frames silently discarded, simulating bus
            contention or a missed reply. Default 0.1%.
        jitter_s: random timing noise added per frame. Real buses are not
            metronomes and downstream code must not assume they are.
        """
        self.rate_hz = rate_hz
        self.drop_rate = drop_rate
        self.jitter_s = jitter_s
        self._rng = random.Random(seed)   # seeded: episodes are reproducible
        self._running = False
        self._t_start = 0.0
        self.frames_emitted = 0
        self.frames_dropped = 0

    @property
    def is_mock(self) -> bool:
        return True

    def open(self) -> None:
        self._running = True
        self._t_start = monotonic_ns() / 1e9

    def close(self) -> None:
        self._running = False

    # -- motion model ------------------------------------------------------

    def _trajectory(self, joint_index: int, t: float) -> tuple[float, float, float]:
        """Position, velocity and torque for one joint at time t.

        Two sinusoids of different frequency per joint, phase-offset by joint
        index. Smooth and bounded, so it looks like a hand guiding the arm
        rather than noise, and differentiable so velocity is exact rather
        than estimated.
        """
        f1 = 0.20 + 0.04 * joint_index      # slow sweep
        f2 = 0.70 + 0.11 * joint_index      # faster overlay
        a1, a2 = 0.45, 0.12
        phase = joint_index * 0.7

        w1, w2 = 2 * math.pi * f1, 2 * math.pi * f2
        pos = a1 * math.sin(w1 * t + phase) + a2 * math.sin(w2 * t)
        vel = a1 * w1 * math.cos(w1 * t + phase) + a2 * w2 * math.cos(w2 * t)
        acc = -a1 * w1 * w1 * math.sin(w1 * t + phase) - a2 * w2 * w2 * math.sin(w2 * t)

        # Stand-in torque: a gravity term that peaks when the joint is
        # extended, plus an inertia term proportional to acceleration.
        # Shoulder joints carry more of the limb, so they are weighted up.
        gravity_scale = 2.5 if joint_index % 7 < 3 else 0.8
        torque = gravity_scale * math.cos(pos) + 0.05 * acc
        return pos, vel, torque

    # -- frame production --------------------------------------------------

    def frames(self) -> Iterator[CANFrame]:
        limiter = RateLimiter(self.rate_hz)

        while self._running:
            t = monotonic_ns() / 1e9 - self._t_start

            # One full round of the bus: every motor reports once. On real
            # hardware these are spread across the cycle rather than
            # simultaneous, which is what gives a snapshot its spread.
            for idx, joint in enumerate(JOINTS):
                if self._rng.random() < self.drop_rate:
                    self.frames_dropped += 1
                    continue

                pos, vel, torque = self._trajectory(idx, t)
                spec = MOTOR_SPECS[joint.motor]

                fb = JointFeedback(
                    can_id=joint.can_id,
                    position_rad=max(-spec.p_max, min(spec.p_max, pos)),
                    velocity_rad_s=max(-spec.v_max, min(spec.v_max, vel)),
                    torque_nm=max(-spec.t_max, min(spec.t_max, torque)),
                    temp_mos_c=38 + int(3 * math.sin(t * 0.05 + idx)),
                    temp_rotor_c=42 + int(4 * math.sin(t * 0.04 + idx)),
                    error="ok",
                )

                jitter_ns = int(self._rng.uniform(-self.jitter_s, self.jitter_s) * 1e9)
                self.frames_emitted += 1

                yield CANFrame(
                    interface=joint.interface,
                    can_id=joint.can_id,
                    data=encode_feedback(fb, spec=spec),
                    t_mono_ns=monotonic_ns() + jitter_ns,
                )

            limiter.sleep()

    @property
    def missed_cycles(self) -> int:
        """Cycles where the host could not keep up with the target rate."""
        return getattr(self, "_limiter_missed", 0)
