"""The camera equivalent of can/source.py: one seam, two implementations.

Same reasoning as the CAN layer. Nothing downstream knows whether frames came
from a real Arducam or from a simulation.

One detail here that has no analogue on the CAN side, and that quietly
corrupts datasets when ignored:

    A CAN frame is an instant. A camera frame is an INTERVAL.

A sensor reading is sampled at a moment. An image integrates light across the
whole exposure, so a 1/60 s exposure smears 16.7 ms of world into one array.
Asking "when was this frame taken" has no exact answer.

The convention used here is **mid-exposure**, which is the standard choice and
the defensible one: it is the centre of mass of the light that formed the
image. Stamping at start of exposure biases every frame early by half the
exposure time, and stamping at readout biases late by a full exposure plus
transfer. Either produces a systematic offset between images and joint states,
which is the single most damaging error in this pipeline (see the README on
why bias beats noise for wrecking a policy).

`Frame.exposure_s` is carried alongside the timestamp so a consumer can see
how wide the interval was rather than having to assume.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Iterator
from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    """One image, timestamped at the middle of its exposure."""

    camera: str
    seq: int                 # monotonic per camera; gaps mean dropped frames
    t_mono_ns: int           # mid-exposure, monotonic domain (see clock.py)
    exposure_s: float        # how much time this image integrates over
    image: np.ndarray        # (H, W, 3) uint8

    @property
    def t_start_ns(self) -> int:
        return self.t_mono_ns - int(self.exposure_s * 1e9 / 2)

    @property
    def t_end_ns(self) -> int:
        return self.t_mono_ns + int(self.exposure_s * 1e9 / 2)


class CameraSource(ABC):
    """A source of frames from one camera."""

    @property
    @abstractmethod
    def name(self) -> str: ...

    @property
    @abstractmethod
    def is_mock(self) -> bool: ...

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def frames(self) -> Iterator[Frame]:
        """Yield frames until close(). Blocking generator."""

    @abstractmethod
    def close(self) -> None: ...

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()
        return False


@dataclass
class CameraStream:
    """Everything captured from one camera during an episode.

    Timestamps are kept in a separate array from the images on purpose. The
    aligner does binary searches over the timeline thousands of times and must
    never touch pixel data to do it; keeping them apart means alignment stays
    fast and can run on metadata alone, without loading images at all.
    """

    name: str
    t_mono_ns: np.ndarray        # (N,) int64, mid-exposure
    seq: np.ndarray              # (N,) int64
    exposure_s: np.ndarray       # (N,) float64
    images: list[np.ndarray] | None = None   # None when metadata only

    def __len__(self) -> int:
        return int(self.t_mono_ns.size)

    @property
    def dropped(self) -> int:
        """Frames the camera skipped, inferred from gaps in the sequence."""
        if self.seq.size < 2:
            return 0
        return int((np.diff(self.seq) - 1).sum())

    @property
    def measured_fps(self) -> float:
        if self.t_mono_ns.size < 2:
            return 0.0
        span_s = (self.t_mono_ns[-1] - self.t_mono_ns[0]) / 1e9
        return (self.t_mono_ns.size - 1) / span_s if span_s > 0 else 0.0

    @classmethod
    def from_frames(cls, name: str, frames: list[Frame],
                    keep_images: bool = True) -> "CameraStream":
        return cls(
            name=name,
            t_mono_ns=np.array([f.t_mono_ns for f in frames], dtype=np.int64),
            seq=np.array([f.seq for f in frames], dtype=np.int64),
            exposure_s=np.array([f.exposure_s for f in frames], dtype=np.float64),
            images=[f.image for f in frames] if keep_images else None,
        )


@dataclass
class JointStream:
    """Every joint snapshot captured during an episode.

    The counterpart to CameraStream. Same shape of idea: timestamps in their
    own array, values in parallel arrays, nothing resampled.
    """

    t_mono_ns: np.ndarray    # (N,) int64
    position: np.ndarray     # (N, J) float32
    velocity: np.ndarray     # (N, J) float32
    torque: np.ndarray       # (N, J) float32
    valid: np.ndarray        # (N, J) bool
    spread_s: np.ndarray     # (N,) float64 -- per-snapshot reconstruction error

    def __len__(self) -> int:
        return int(self.t_mono_ns.size)

    @property
    def measured_rate_hz(self) -> float:
        if self.t_mono_ns.size < 2:
            return 0.0
        span_s = (self.t_mono_ns[-1] - self.t_mono_ns[0]) / 1e9
        return (self.t_mono_ns.size - 1) / span_s if span_s > 0 else 0.0
