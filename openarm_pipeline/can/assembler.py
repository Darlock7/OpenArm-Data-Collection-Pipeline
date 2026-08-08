"""Turns a stream of individual motor frames into whole-arm snapshots.

The problem this solves is easy to overlook. Downstream code wants "the
state of the arm at time t" -- 14 positions, 14 velocities, 14 torques. The
bus does not provide that. It provides one motor's reply at a time, each
arriving a few tens of microseconds after the last.

So a snapshot is a reconstruction, and every reconstruction makes a choice.
The choice here is: hold the most recent reading from each motor, and emit a
snapshot once every motor has reported at least once in the current window.

Two things are recorded alongside the values so the choice stays visible:

  spread_s  how far apart the oldest and newest readings in the snapshot
            were. If a snapshot claims to describe one instant but its
            readings span 8 ms, a consumer deserves to know.
  valid     per joint, whether a fresh reading arrived in this window at
            all. A dropped frame leaves the previous value in place, which
            is the right behaviour -- but it must be marked, not hidden.

A pipeline that skipped both would produce data that looks cleaner than it
is, and the error would be invisible until a policy trained on it behaved
strangely on hardware.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator

import numpy as np

from ..config import JOINTS, MOTOR_SPECS, N_JOINTS
from .protocol import decode_feedback
from .source import CANFrame, JointState

#: (interface, can_id) -> index into the joint arrays. Built once. The
#: interface has to be part of the key because both arms reuse IDs 0x01-0x07
#: on their own bus -- keying on can_id alone would collide the two arms.
_INDEX: dict[tuple[str, int], int] = {
    (j.interface, j.can_id): i for i, j in enumerate(JOINTS)
}


class JointStateAssembler:
    """Accumulates frames; emits a JointState per completed window."""

    def __init__(self, require_all: bool = False):
        """
        require_all: if True, only emit once every joint has reported in the
            current window. Safer, but one dead motor stalls the stream
            entirely. Default False: emit on window completion regardless,
            and mark the missing joints invalid. Losing one joint should
            degrade the recording, not stop it.
        """
        self.require_all = require_all

        self._pos = np.zeros(N_JOINTS, dtype=np.float32)
        self._vel = np.zeros(N_JOINTS, dtype=np.float32)
        self._tau = np.zeros(N_JOINTS, dtype=np.float32)
        self._t_ns = np.zeros(N_JOINTS, dtype=np.int64)
        self._seen = np.zeros(N_JOINTS, dtype=bool)

        self.frames_seen = 0
        self.frames_unknown = 0   # frames from IDs not in our joint map
        self.frames_bad = 0       # frames that failed to decode
        self.snapshots = 0

    def push(self, frame: CANFrame) -> JointState | None:
        """Absorb one frame. Returns a snapshot when the window completes."""
        self.frames_seen += 1

        idx = _INDEX.get((frame.interface, frame.can_id))
        if idx is None:
            # Unknown ID. Could be a different device on a shared bus, or a
            # misconfigured motor. Counted, not raised -- one stray device
            # must not take the recording down.
            self.frames_unknown += 1
            return None

        try:
            fb = decode_feedback(frame.data, spec=MOTOR_SPECS[JOINTS[idx].motor])
        except ValueError:
            self.frames_bad += 1
            return None

        # A joint reporting twice before the others have reported once means
        # the window is over: flush what we have, then start the new window
        # with this frame.
        completing = self._seen[idx]

        if completing:
            snapshot = self._emit()
            self._store(idx, fb, frame.t_mono_ns)
            return snapshot

        self._store(idx, fb, frame.t_mono_ns)

        if self.require_all and self._seen.all():
            return self._emit()
        return None

    def _store(self, idx: int, fb, t_ns: int) -> None:
        self._pos[idx] = fb.position_rad
        self._vel[idx] = fb.velocity_rad_s
        self._tau[idx] = fb.torque_nm
        self._t_ns[idx] = t_ns
        self._seen[idx] = True

    def _emit(self) -> JointState:
        fresh = self._t_ns[self._seen]
        spread_s = float(fresh.max() - fresh.min()) / 1e9 if fresh.size else 0.0

        state = JointState(
            # The snapshot is stamped with its NEWEST reading, not an average.
            # An average would be a number no sensor ever produced; the newest
            # reading is a real measurement, and spread_s carries the rest.
            t_mono_ns=int(fresh.max()) if fresh.size else 0,
            position=self._pos.copy(),
            velocity=self._vel.copy(),
            torque=self._tau.copy(),
            valid=self._seen.copy(),
            spread_s=spread_s,
        )

        self._seen[:] = False   # values persist; freshness does not
        self.snapshots += 1
        return state

    def stream(self, frames: Iterable[CANFrame]) -> Iterator[JointState]:
        """Convenience wrapper: frame stream in, snapshot stream out."""
        for frame in frames:
            state = self.push(frame)
            if state is not None:
                yield state
