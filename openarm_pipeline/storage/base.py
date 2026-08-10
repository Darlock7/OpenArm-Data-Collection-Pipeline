"""Episode storage: the write path and the read path are different problems.

THE ARGUMENT
------------
The task says "choose a storage format". The honest answer is that one format
cannot serve both jobs well, because they want opposite things:

  RECORDING                          TRAINING
  append one sample at a time        random access to shuffled timesteps
  length unknown until you stop      length fixed and known
  five streams, all different rates  usually one common timeline
  must survive a power cut           must keep a GPU fed
  -> append-only, crash tolerant     -> chunked, compressed, indexed

So this package does not pick one. It records to an append-only log and
exports to an array store:

    capture ──▶ MCAP ──[ align() ]──▶ HDF5 / LeRobot
                 raw                   one timeline, dataloader ready

That is Task 3's rule made physical. "Store raw, align at read time" becomes
"the alignment policy is a parameter of the export", which means one recording
can produce several training sets without capturing anything twice.

WHY MCAP FOR THE WRITE PATH
  * Append-only, and a file truncated by a crash is still readable up to the
    last complete message. HDF5 can lose the whole file to a bad write.
  * Per-message timestamps and independent channels are native, so five
    streams at five rates need no resampling and no padding.
  * It is the ROS 2 and Foxglove standard, so recordings open in existing
    tools rather than needing bespoke viewers.

WHY HDF5 FOR THE READ PATH
  * Chunked and compressed random access, which is what a shuffling dataloader
    actually does.
  * Widely used in imitation learning, so it drops into existing training code.
  * LeRobot/RLDS is the same shape of target and is what DeepAware's own
    pipeline consumes; see `export.py` for how that would slot in.

KNOWN LIMITATION, stated plainly: images here are stored as raw arrays. A real
deployment must encode video (h264/AV1) with a frame-index-to-timestamp map,
the way LeRobot pairs mp4 with parquet. At four cameras and full resolution,
raw frames are roughly 150 MB/s, which is not a thing you keep. The synthetic
frames in this project are small enough that raw storage is fine for a demo
and wrong for production.
"""

from __future__ import annotations

import json
import time
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from ..config import CAMERAS, JOINT_NAMES

SCHEMA_VERSION = "1.0"


@dataclass
class EpisodeMetadata:
    """Everything needed to interpret an episode without opening the data.

    Written as a sidecar JSON as well as inside the file, so the REST API can
    list a thousand episodes without deserialising a thousand recordings.
    """

    episode_id: str
    created_utc: float
    duration_s: float = 0.0

    # ---- provenance. load-bearing, not decoration --------------------------
    is_mock: bool = True
    source_note: str = "SIMULATED - not a real OpenArm"
    schema_version: str = SCHEMA_VERSION

    # ---- contents ----------------------------------------------------------
    joint_names: list[str] = field(default_factory=lambda: list(JOINT_NAMES))
    camera_names: list[str] = field(default_factory=lambda: [c.name for c in CAMERAS])
    n_joint_samples: int = 0
    n_frames: dict[str, int] = field(default_factory=dict)

    # ---- timing, so quality is visible before download ---------------------
    joint_rate_hz: float = 0.0
    camera_fps: dict[str, float] = field(default_factory=dict)
    dropped_frames: dict[str, int] = field(default_factory=dict)
    mean_snapshot_spread_s: float = 0.0

    # ---- the wall-clock anchor from clock.TimeBase -------------------------
    t0_utc_s: float = 0.0
    t0_monotonic_ns: int = 0

    notes: str = ""

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2, sort_keys=True)

    @classmethod
    def from_json(cls, text: str) -> "EpisodeMetadata":
        raw = json.loads(text)
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in raw.items() if k in known})

    @property
    def filename_stem(self) -> str:
        """Mock episodes carry it in the filename, not only in metadata.

        Someone six months from now will find these files in a directory
        listing with no context. `mock_` in the name is the last line of
        defence against synthetic data being taken for a real demonstration.
        """
        return f"{'mock_' if self.is_mock else ''}{self.episode_id}"


def new_episode_id(now: float | None = None) -> str:
    """Sortable, human readable, collision resistant enough for one rig."""
    now = now if now is not None else time.time()
    return time.strftime("%Y%m%dT%H%M%S", time.gmtime(now)) + f"_{int(now * 1e6) % 1000:03d}"


class EpisodeWriter(ABC):
    """Append-only sink for one episode.

    Deliberately narrow. The recorder pushes samples as they arrive and never
    asks the writer to reorganise, index, or align anything, because doing any
    of that on the capture path risks blocking it.
    """

    @abstractmethod
    def open(self, meta: EpisodeMetadata) -> None: ...

    @abstractmethod
    def write_joint_state(self, t_mono_ns: int, position: np.ndarray,
                          velocity: np.ndarray, torque: np.ndarray,
                          valid: np.ndarray, spread_s: float) -> None: ...

    @abstractmethod
    def write_frame(self, camera: str, t_mono_ns: int, seq: int,
                    exposure_s: float, image: np.ndarray) -> None: ...

    @abstractmethod
    def close(self) -> EpisodeMetadata:
        """Finalise and return the metadata actually written."""

    @property
    @abstractmethod
    def path(self) -> Path: ...

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
