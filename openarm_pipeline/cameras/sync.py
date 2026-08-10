"""Aligning streams that were never sampled together.

THE PROBLEM
-----------
Joint states arrive at 1 kHz. The ZED runs at 60 fps, the wrist cameras at 30,
the ceiling camera at 15. None of them tick together, none of their clocks
agree, and all of them jitter. A training sample needs an image and a joint
state that describe *the same moment*, and no such pair exists in the raw data.

THE RULE THIS FILE IS BUILT ON
------------------------------
    Do not resample at record time. Store raw, align at read time.

At record time you do not know what the training code will want. One method
wants a sample per ZED frame with interpolated joints; another wants raw 1 kHz
torque for contact detection; a third wants a fixed 30 Hz grid. Resampling
during capture bakes one of those in and destroys the others irreversibly.

So nothing in this file mutates a stream. Alignment is a query: hand it a
timeline, get back a view. Ask again with a different policy and you get a
different view of the same untouched data.

WHY INTERPOLATE JOINTS BUT NEVER IMAGES
---------------------------------------
Joint position is a continuous physical signal. A joint cannot teleport, so a
value between two samples is a reasonable estimate of something that really
happened.

An image is not continuous in that sense. Blending two frames produces a
translucent ghost of a scene that never existed. Train a vision policy on
manufactured frames and it learns to expect artifacts the real camera will
never produce. So cameras get NEAREST-FRAME ONLY, and the staleness is
recorded rather than hidden.

Torque sits in between and is treated with suspicion: it is near
discontinuous at contact, and contact is usually the most important moment in
a manipulation demonstration. Interpolating across a contact event invents a
gentle ramp where reality had a step.

WHAT IS ACTUALLY BEING DEFENDED AGAINST
---------------------------------------
Ranked by how badly each damages a trained policy:

  1. BIAS -- a systematic offset between image and joint state. The policy
     learns to act on stale observations, then lags and overshoots on real
     hardware. Bias does not average out with more data; it is learned.
  2. JITTER -- an offset that varies. Worse per millisecond than a constant
     one, because a constant offset can be partly absorbed into the learned
     dynamics and a varying one cannot.
  3. ALIASING -- decimating 1 kHz joint data to camera rate without filtering
     folds high-frequency content down into the training signal, where it
     looks like real motion. This is why `window_mean` exists.

Every aligned sample therefore carries its own timing error. That is the
single most important output of this module: it makes 1 and 2 measurable, and
you cannot fix what you cannot see.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from ..config import PRIMARY_CAMERA, SYNC_TOLERANCE_S
from .source import CameraStream, JointStream

JointPolicy = Literal["nearest", "hermite", "window_mean"]


# ---------------------------------------------------------------------------
# primitives
# ---------------------------------------------------------------------------

def nearest_index(t_samples: np.ndarray, t_query: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """For each query, the closest sample index and the SIGNED error.

    Sign convention, and it matters: ``dt = t_sample - t_query``.
      negative -> the sample is OLDER than the query (stale, the usual case)
      positive -> the sample is NEWER than the query

    Signed rather than absolute because the mean of the signed error is the
    systematic bias, which is failure mode 1. Take absolute values first and
    the bias vanishes into the noise, which is exactly the mistake that lets a
    50 ms offset ship undetected.
    """
    if t_samples.size == 0:
        return (np.full(t_query.shape, -1, dtype=np.int64),
                np.full(t_query.shape, np.nan))

    right = np.searchsorted(t_samples, t_query)
    left = np.clip(right - 1, 0, t_samples.size - 1)
    right = np.clip(right, 0, t_samples.size - 1)

    take_right = np.abs(t_samples[right] - t_query) < np.abs(t_samples[left] - t_query)
    idx = np.where(take_right, right, left)
    dt_s = (t_samples[idx] - t_query) / 1e9
    return idx.astype(np.int64), dt_s


def hermite(t_samples: np.ndarray, values: np.ndarray, derivs: np.ndarray,
            t_query: np.ndarray) -> np.ndarray:
    """Cubic Hermite interpolation, using the reported derivative.

    Linear interpolation between two positions ignores something the motor
    already told us: the velocity at each endpoint. Hermite uses it, so the
    curve leaves each sample at the slope actually measured there rather than
    at whatever slope the straight line happens to have.

    Strictly better than linear and free, since velocity is in every frame.

    Queries outside the sample range are held at the endpoint rather than
    extrapolated. Extrapolating a cubic past its data produces confident
    nonsense; the caller sees the large `joint_dt_s` and can discard it.
    """
    n = t_samples.size
    if n == 0:
        return np.full((t_query.size,) + values.shape[1:], np.nan)
    if n == 1:
        return np.repeat(values[:1], t_query.size, axis=0)

    i1 = np.clip(np.searchsorted(t_samples, t_query), 1, n - 1)
    i0 = i1 - 1

    t0 = t_samples[i0].astype(np.float64)
    t1 = t_samples[i1].astype(np.float64)
    h = (t1 - t0) / 1e9                      # seconds
    h = np.where(h <= 0, 1e-12, h)           # guard duplicate timestamps
    s = np.clip((t_query - t0) / 1e9 / h, 0.0, 1.0)

    # standard Hermite basis
    s2, s3 = s * s, s * s * s
    h00 = 2 * s3 - 3 * s2 + 1
    h10 = s3 - 2 * s2 + s
    h01 = -2 * s3 + 3 * s2
    h11 = s3 - s2

    # broadcast (N,) coefficients against (N, J) values
    def col(a):
        return a[:, None] if values.ndim == 2 else a

    return (col(h00) * values[i0]
            + col(h10 * h) * derivs[i0]
            + col(h01) * values[i1]
            + col(h11 * h) * derivs[i1])


def window_mean(t_samples: np.ndarray, values: np.ndarray,
                t_query: np.ndarray, window_s: float) -> tuple[np.ndarray, np.ndarray]:
    """Average every sample within +/- window_s/2 of each query.

    This is the anti-aliasing option, and it is here for one specific case:
    downsampling 1 kHz joint data to a camera rate is DECIMATION, and
    decimating without first low-pass filtering folds everything above the new
    Nyquist frequency down into the result. Contact vibration and sensor noise
    stop looking like noise and start looking like real low-frequency motion
    that never happened.

    A boxcar average is a crude low-pass filter, but crude and applied beats
    ideal and skipped.

    TRADE-OFF, and it is a real one: this smooths genuine sharp transitions
    too. It also means the value handed to the policy is not the instantaneous
    state the policy will see at inference, which is its own mismatch. Default
    to `hermite`; reach for this when downsampling hard.

    Returns (values, n_samples_averaged) so a consumer can see when a window
    was thin.
    """
    # Half-open window [t - w/2, t + w/2). Closing both ends would include
    # both boundary samples, so a 10 ms window at 1 kHz would average 11
    # samples rather than 10 and adjacent windows would share a sample.
    half_ns = int(window_s * 1e9 / 2)
    lo = np.searchsorted(t_samples, t_query - half_ns, side="left")
    hi = np.searchsorted(t_samples, t_query + half_ns, side="left")

    out = np.empty((t_query.size,) + values.shape[1:], dtype=np.float64)
    counts = np.zeros(t_query.size, dtype=np.int64)

    # Cumulative sum makes each window O(1) instead of O(window). At 1 kHz
    # across a 60 s episode with 60 queries per second this matters.
    csum = np.concatenate([np.zeros((1,) + values.shape[1:]),
                           np.cumsum(values, axis=0)], axis=0)
    for k in range(t_query.size):
        a, b = lo[k], hi[k]
        counts[k] = b - a
        out[k] = (csum[b] - csum[a]) / (b - a) if b > a else np.nan
    return out, counts


# ---------------------------------------------------------------------------
# episode alignment
# ---------------------------------------------------------------------------

@dataclass
class AlignedEpisode:
    """A view of an episode on one common timeline. Owns no raw data."""

    t_s: np.ndarray                          # (N,) seconds from episode start
    t_mono_ns: np.ndarray                    # (N,) the query timeline itself

    position: np.ndarray                     # (N, J)
    velocity: np.ndarray                     # (N, J)
    torque: np.ndarray                       # (N, J)
    joint_dt_s: np.ndarray                   # (N,) distance to nearest REAL sample
    joint_valid: np.ndarray                  # (N,)
    joint_policy: str
    joint_synthetic: bool                    # True when values were computed, not measured

    camera_idx: dict[str, np.ndarray] = field(default_factory=dict)
    camera_dt_s: dict[str, np.ndarray] = field(default_factory=dict)
    camera_valid: dict[str, np.ndarray] = field(default_factory=dict)

    anchor: str = ""
    tolerance_s: float = SYNC_TOLERANCE_S

    def __len__(self) -> int:
        return int(self.t_s.size)

    def quality(self) -> dict:
        """Alignment quality, in the terms that actually predict model damage.

        `bias_ms` is the mean SIGNED error: the systematic offset. This is the
        number to watch. `jitter_ms` is its standard deviation.
        """
        def stats(dt: np.ndarray, valid: np.ndarray) -> dict:
            d = dt[valid & np.isfinite(dt)]
            if d.size == 0:
                return {"bias_ms": float("nan"), "jitter_ms": float("nan"),
                        "worst_ms": float("nan"), "coverage": 0.0}
            return {
                "bias_ms": float(np.mean(d) * 1e3),
                "jitter_ms": float(np.std(d) * 1e3),
                "worst_ms": float(np.max(np.abs(d)) * 1e3),
                "coverage": float(valid.mean()),
            }

        return {
            "samples": len(self),
            "anchor": self.anchor,
            "joint_policy": self.joint_policy,
            "joints": stats(self.joint_dt_s, self.joint_valid),
            "cameras": {
                name: stats(self.camera_dt_s[name], self.camera_valid[name])
                for name in self.camera_dt_s
            },
        }


def build_timeline(camera_streams: dict[str, CameraStream],
                   anchor: str = PRIMARY_CAMERA) -> np.ndarray:
    """The timestamps training samples will sit on.

    Anchoring on a camera rather than on a synthetic fixed grid is deliberate.
    In imitation learning the observation IS an image, and at inference the
    policy runs when a frame arrives. Anchoring on real frame times means
    every training sample corresponds to a moment the robot will actually
    experience, and the anchor camera needs no interpolation at all -- its
    error is exactly zero by construction.

    A fixed grid would instead resample every stream including the anchor,
    which buys independence from any one sensor at the cost of making every
    single sample synthetic. `align()` accepts any timeline, so that remains
    available; it is just not the default.
    """
    if anchor not in camera_streams:
        raise KeyError(f"anchor camera {anchor!r} not among {list(camera_streams)}")
    return camera_streams[anchor].t_mono_ns.copy()


def align(joints: JointStream,
          camera_streams: dict[str, CameraStream],
          timeline: np.ndarray | None = None,
          anchor: str = PRIMARY_CAMERA,
          joint_policy: JointPolicy = "hermite",
          tolerance_s: float = SYNC_TOLERANCE_S,
          window_s: float | None = None) -> AlignedEpisode:
    """Produce one aligned view. Mutates nothing.

    joint_policy:
      "hermite"     -- interpolate using position and reported velocity. Default.
      "nearest"     -- closest real measurement, nothing synthesised. Use when
                       only genuine sensor readings are acceptable.
      "window_mean" -- boxcar average, the anti-aliasing option for hard
                       downsampling. `window_s` defaults to the anchor's frame
                       interval, which is the correct width when producing one
                       sample per frame.
    """
    if timeline is None:
        timeline = build_timeline(camera_streams, anchor)
    timeline = np.asarray(timeline, dtype=np.int64)

    # ---- joints ----------------------------------------------------------
    j_idx, j_dt = nearest_index(joints.t_mono_ns, timeline)

    if joint_policy == "nearest":
        pos, vel, tau = joints.position[j_idx], joints.velocity[j_idx], joints.torque[j_idx]
        synthetic = False

    elif joint_policy == "hermite":
        pos = hermite(joints.t_mono_ns, joints.position, joints.velocity, timeline)
        # Velocity's own derivative (acceleration) is not reported, so velocity
        # and torque fall back to linear. Being explicit about the asymmetry:
        # position gets the better method because it is the only channel whose
        # derivative we actually have.
        vel = _linear(joints.t_mono_ns, joints.velocity, timeline)
        tau = _linear(joints.t_mono_ns, joints.torque, timeline)
        synthetic = True

    elif joint_policy == "window_mean":
        if window_s is None:
            window_s = _median_interval_s(timeline)
        pos, _ = window_mean(joints.t_mono_ns, joints.position, timeline, window_s)
        vel, _ = window_mean(joints.t_mono_ns, joints.velocity, timeline, window_s)
        tau, n = window_mean(joints.t_mono_ns, joints.torque, timeline, window_s)
        synthetic = True

    else:
        raise ValueError(f"unknown joint_policy {joint_policy!r}")

    # Valid means: inside the recorded span, and close enough to a real sample.
    # Outside the span the interpolator holds at the endpoint, which is a
    # guess, so it is marked rather than silently trusted.
    in_span = (timeline >= joints.t_mono_ns[0]) & (timeline <= joints.t_mono_ns[-1]) \
        if len(joints) else np.zeros(timeline.shape, dtype=bool)
    j_valid = in_span & (np.abs(j_dt) <= tolerance_s)

    # ---- cameras ---------------------------------------------------------
    cam_idx: dict[str, np.ndarray] = {}
    cam_dt: dict[str, np.ndarray] = {}
    cam_ok: dict[str, np.ndarray] = {}

    for name, stream in camera_streams.items():
        idx, dt = nearest_index(stream.t_mono_ns, timeline)
        # Tolerance scales with the camera's own frame interval. Holding a
        # 15 fps ceiling camera to the same millisecond budget as a 60 fps ZED
        # would reject nearly all of its frames for doing nothing wrong; its
        # best possible match is half a frame away, which is 33 ms.
        tol = max(tolerance_s, 0.5 / max(stream.measured_fps, 1e-6))
        cam_idx[name] = idx
        cam_dt[name] = dt
        cam_ok[name] = np.isfinite(dt) & (np.abs(dt) <= tol)

    t0 = timeline[0] if timeline.size else 0
    return AlignedEpisode(
        t_s=(timeline - t0) / 1e9,
        t_mono_ns=timeline,
        position=np.asarray(pos, dtype=np.float32),
        velocity=np.asarray(vel, dtype=np.float32),
        torque=np.asarray(tau, dtype=np.float32),
        joint_dt_s=j_dt,
        joint_valid=j_valid,
        joint_policy=joint_policy,
        joint_synthetic=synthetic,
        camera_idx=cam_idx,
        camera_dt_s=cam_dt,
        camera_valid=cam_ok,
        anchor=anchor,
        tolerance_s=tolerance_s,
    )


# ---------------------------------------------------------------------------

def _linear(t_samples: np.ndarray, values: np.ndarray, t_query: np.ndarray) -> np.ndarray:
    """Per-column linear interpolation, endpoint-held outside the range."""
    if t_samples.size == 0:
        return np.full((t_query.size,) + values.shape[1:], np.nan)
    if t_samples.size == 1:
        return np.repeat(values[:1], t_query.size, axis=0)
    x = t_samples.astype(np.float64)
    xi = t_query.astype(np.float64)
    if values.ndim == 1:
        return np.interp(xi, x, values)
    return np.column_stack([np.interp(xi, x, values[:, j])
                            for j in range(values.shape[1])])


def _median_interval_s(t: np.ndarray) -> float:
    return float(np.median(np.diff(t)) / 1e9) if t.size > 1 else 0.0
