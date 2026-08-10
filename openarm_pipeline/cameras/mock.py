"""Four simulated cameras, deliberately imperfect.

>>> SIMULATED. NOT REAL CAMERAS. <<<

A mock that produced perfectly regular frames at exactly the nominal rate
would make the synchroniser look flawless and prove nothing. Real cameras
misbehave in four specific ways, and all four are modelled here because each
one breaks a different naive assumption:

  1. JITTER -- frame intervals vary. Breaks "frame N happened at N/fps".
  2. CLOCK DRIFT -- the camera's crystal is not the host's. A camera nominally
     at 30 fps free-runs at 29.97, and over a 60 s episode that is nearly two
     full frames of accumulated offset. Breaks any code that assumes a
     constant offset measured once at the start.
  3. DROPPED FRAMES -- USB contention, buffer overruns. Breaks "frame index is
     time". This is why Frame carries a sequence number: a gap in `seq` is
     visible, whereas a gap in a Python list is not.
  4. STARTUP SKEW -- cameras are opened one after another and do not begin at
     the same instant. Breaks "stream 0 sample 0 lines up with stream 1
     sample 0".

Together these are the reason alignment has to be timestamp-based rather than
index-based, which is the whole point of Task 3.

IMAGE CONTENT: each frame carries a small synthetic pattern with a marker that
moves with time, so alignment can be checked by eye rather than only by
assertion. Images are rendered at a fraction of the declared sensor
resolution (see `scale`) to keep the demo cheap; the declared resolution is
preserved in metadata. That is a simulation shortcut, not a claim about the
real cameras.
"""

from __future__ import annotations

import random
from collections.abc import Iterator

import numpy as np

from ..clock import monotonic_ns
from ..config import CAMERAS, CameraSpec
from .source import CameraSource, Frame


class MockCamera(CameraSource):
    """One simulated camera with its own imperfect clock."""

    def __init__(self, spec: CameraSpec, scale: float = 0.125,
                 drift_ppm: float | None = None, jitter_s: float = 0.0015,
                 drop_rate: float = 0.002, startup_skew_s: float | None = None,
                 exposure_s: float | None = None, seed: int = 0):
        """
        drift_ppm: clock error in parts per million. Real consumer camera
            crystals are commonly tens to low hundreds of ppm out. Randomised
            per camera when None, which is the realistic case: no two cameras
            drift the same way.
        startup_skew_s: how late this camera begins relative to t=0.
            Randomised when None.
        exposure_s: defaults to half the frame interval.
        """
        self.spec = spec
        self._rng = random.Random(seed ^ (hash(spec.name) & 0xFFFF))

        self.drift_ppm = drift_ppm if drift_ppm is not None else self._rng.uniform(-120, 120)
        self.startup_skew_s = (startup_skew_s if startup_skew_s is not None
                               else self._rng.uniform(0.0, 0.040))
        self.jitter_s = jitter_s
        self.drop_rate = drop_rate
        self.exposure_s = exposure_s if exposure_s is not None else 0.5 / spec.fps

        self.w = max(8, int(spec.width * scale))
        self.h = max(8, int(spec.height * scale))

        self._running = False
        self._t0_ns = 0
        self._seq = 0
        self.frames_emitted = 0
        self.frames_dropped = 0

    @property
    def name(self) -> str:
        return self.spec.name

    @property
    def is_mock(self) -> bool:
        return True

    def open(self) -> None:
        self._running = True
        self._t0_ns = monotonic_ns()
        self._seq = 0

    def close(self) -> None:
        self._running = False

    # -- timing ------------------------------------------------------------

    def _due_ns(self, seq: int) -> int:
        """When frame `seq` should be stamped, mid-exposure.

        Drift is applied as a multiplier on elapsed time, not as a constant
        offset, because that is how a wrong crystal actually behaves: the
        error grows with the length of the episode.
        """
        nominal_s = self.startup_skew_s + seq / self.spec.fps
        drifted_s = nominal_s * (1.0 + self.drift_ppm * 1e-6)
        jitter_s = self._rng.uniform(-self.jitter_s, self.jitter_s)
        return self._t0_ns + int((drifted_s + jitter_s) * 1e9)

    # -- image ------------------------------------------------------------

    def _render(self, t_s: float) -> np.ndarray:
        """Cheap synthetic frame with a time-dependent marker.

        Deliberately not a rendering of the arm. Pretending to simulate what
        the cameras would actually see would be a much larger claim than this
        project can support, and would invite the reader to trust pixels that
        mean nothing.
        """
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)

        # static gradient so each camera looks distinct from the others
        img[:, :, 0] = np.linspace(20, 90, self.w, dtype=np.uint8)[None, :]
        img[:, :, 1] = np.linspace(20, 70, self.h, dtype=np.uint8)[:, None]

        # marker sweeping left to right once per second -- alignment across
        # cameras is checkable by eye from where this sits in each frame
        x = int((t_s % 1.0) * (self.w - 1))
        y = self.h // 2
        img[max(0, y - 2):y + 3, max(0, x - 2):x + 3, 2] = 255
        return img

    # -- capture -----------------------------------------------------------

    def frames(self) -> Iterator[Frame]:
        import time

        while self._running:
            due_ns = self._due_ns(self._seq)
            now_ns = monotonic_ns()
            if due_ns > now_ns:
                time.sleep((due_ns - now_ns) / 1e9)

            seq = self._seq
            self._seq += 1

            if self._rng.random() < self.drop_rate:
                # Dropped. The sequence number still advances, which is what
                # makes the loss visible downstream instead of silent.
                self.frames_dropped += 1
                continue

            t_ns = self._due_ns(seq)
            self.frames_emitted += 1
            yield Frame(
                camera=self.spec.name,
                seq=seq,
                t_mono_ns=t_ns,
                exposure_s=self.exposure_s,
                image=self._render((t_ns - self._t0_ns) / 1e9),
            )


def make_mock_cameras(scale: float = 0.125, seed: int = 0) -> list[MockCamera]:
    """All four cameras from config.CAMERAS, each with its own clock error."""
    return [MockCamera(spec, scale=scale, seed=seed + i)
            for i, spec in enumerate(CAMERAS)]
