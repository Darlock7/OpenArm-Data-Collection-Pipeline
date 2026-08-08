"""The seam between "where CAN frames come from" and everything downstream.

This is the most important design decision in the project, so it gets stated
plainly: nothing downstream of this file knows whether the data came from a
real arm or from a simulation. Both implement CANSource. The recorder takes
one as an argument and cannot tell the difference.

That matters for three reasons:

  1. I have no hardware, so without this seam none of tasks 3-5 could be
     built or tested at all.
  2. It is how the real system should be built anyway. CI cannot plug in a
     robot arm, so the mock is not scaffolding to delete later -- it is the
     test fixture the pipeline is regression-tested against permanently.
  3. Bringing this up on real hardware is a one-line change, and the code
     that would change is already written (socketcan.py).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Iterator
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CANFrame:
    """One raw frame, timestamped as early as we are able to.

    `t_mono_ns` is stamped the instant the frame is handed to us. That is
    NOT when the motor sampled the value -- there is driver, kernel and
    USB latency in between. On real hardware the fix is SocketCAN's
    SO_TIMESTAMPING, which gets the kernel to stamp on arrival instead;
    with a hardware-timestamping adapter it comes from the controller
    itself. Noted in the README as a known limitation.
    """

    interface: str
    can_id: int
    data: bytes
    t_mono_ns: int


@dataclass
class JointState:
    """All joints of the rig at one instant.

    Real hardware does not deliver this atomically -- each motor answers
    with its own frame, so a "snapshot" is assembled from 14 frames that
    arrived at slightly different times. `spread_s` records how far apart
    the oldest and newest of those were, so the dataset carries its own
    sync-quality evidence instead of implying a precision it does not have.
    """

    t_mono_ns: int          # timestamp of the newest frame in the snapshot
    position: np.ndarray    # (N,) rad
    velocity: np.ndarray    # (N,) rad/s
    torque: np.ndarray      # (N,) N*m
    valid: np.ndarray       # (N,) bool -- False where no fresh frame arrived
    spread_s: float         # newest-frame time minus oldest-frame time


class CANSource(ABC):
    """A source of CAN frames. Implemented by MockCANSource and SocketCANSource."""

    @abstractmethod
    def open(self) -> None:
        """Acquire the bus. Raises if unavailable."""

    @abstractmethod
    def frames(self) -> Iterator[CANFrame]:
        """Yield frames until close() is called. Blocking generator."""

    @abstractmethod
    def close(self) -> None:
        """Release the bus. Must be safe to call twice."""

    @property
    @abstractmethod
    def is_mock(self) -> bool:
        """True for simulated sources.

        Recorded episodes store this flag. Simulated data must never be
        mistakable for real data once it is sitting in a dataset directory
        six months from now.
        """

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()
        return False
