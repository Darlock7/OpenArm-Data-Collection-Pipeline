"""Episode lifecycle: capture threads in, one file out.

THE REAL-TIME CONSTRAINT, which is the whole reason this file is not just a
for-loop:

    The thread reading CAN must never wait on a disk.

A 1 kHz capture loop has 1 ms per cycle. An fsync, a log rotation or a garbage
collection pause can exceed that by an order of magnitude. If the capture
thread does the writing, a disk hiccup does not slow the recording down, it
puts a HOLE in it, and the hole lands in the data rather than in a log.

So capture threads only enqueue, and a single writer thread drains. The queue
is BOUNDED, deliberately:

  * unbounded -> a slow disk turns into unbounded memory growth, and the
    process dies later, harder, and further from the cause
  * bounded   -> back-pressure is visible immediately, and the drop policy is
    an explicit decision rather than an accident

When the queue is full this drops the newest sample and counts it. Blocking
would stall capture, which is the one thing we are protecting. A recording
with 12 counted gaps is usable and honest; a recording with 12 uncounted gaps
is neither.

A note on Python: the GIL makes this acceptable only because every one of
these threads is I/O bound, releasing the GIL while waiting on a socket or a
file. A real 1 kHz control loop belongs in C++ or on a real-time thread. This
process observes the bus, it does not close a loop over it, so the bar is much
lower. Stated here rather than left for a reviewer to wonder about.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .can.assembler import JointStateAssembler
from .can.mock import MockCANSource
from .can.source import CANSource
from .cameras.mock import make_mock_cameras
from .cameras.source import CameraSource, Frame
from .clock import TimeBase
from .config import DATA_DIR, QUEUE_MAXSIZE
from .storage.base import EpisodeMetadata, EpisodeWriter, new_episode_id
from .storage.mcap_backend import McapEpisodeWriter


@dataclass
class LiveState:
    """Most recent sample, for the dashboard. Not part of the recording."""

    t_mono_ns: int = 0
    position: list[float] = field(default_factory=list)
    velocity: list[float] = field(default_factory=list)
    torque: list[float] = field(default_factory=list)
    valid: list[bool] = field(default_factory=list)
    spread_s: float = 0.0
    rate_hz: float = 0.0
    frames: dict[str, int] = field(default_factory=dict)


@dataclass
class RecorderStatus:
    running: bool = False
    episode_id: str | None = None
    started_utc: float | None = None
    elapsed_s: float = 0.0
    joint_samples: int = 0
    frames: dict[str, int] = field(default_factory=dict)
    queue_depth: int = 0
    dropped_queue: int = 0     # lost to back-pressure, NOT to the bus
    is_mock: bool = True
    last_episode: str | None = None


class Recorder:
    """Owns the capture threads and the episode lifecycle."""

    def __init__(self, can_source: CANSource | None = None,
                 cameras: list[CameraSource] | None = None,
                 data_dir: Path | str = DATA_DIR,
                 writer_factory=None):
        self.can = can_source or MockCANSource()
        self.cameras = cameras if cameras is not None else make_mock_cameras()
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self._writer_factory = writer_factory or (lambda: McapEpisodeWriter(self.data_dir))

        self._q: queue.Queue = queue.Queue(maxsize=QUEUE_MAXSIZE)
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._lock = threading.Lock()

        self._writer: EpisodeWriter | None = None
        self._status = RecorderStatus(is_mock=self.can.is_mock)
        self._live = LiveState()
        self._t_start = 0.0
        self._live_count = 0

        #: capture always runs so the dashboard has something to show; this
        #: gates whether samples are persisted
        self._recording = threading.Event()

    # -- lifecycle ---------------------------------------------------------

    def start_capture(self) -> None:
        """Begin reading sensors. Does not record."""
        if self._threads:
            return
        self._stop.clear()
        self._t_start = time.monotonic()

        self._threads.append(threading.Thread(target=self._run_can, daemon=True,
                                              name="can"))
        for cam in self.cameras:
            self._threads.append(threading.Thread(target=self._run_camera, args=(cam,),
                                                  daemon=True, name=f"cam-{cam.name}"))
        self._threads.append(threading.Thread(target=self._run_writer, daemon=True,
                                              name="writer"))
        for t in self._threads:
            t.start()

    def stop_capture(self) -> None:
        if self._recording.is_set():
            self.stop_recording()
        self._stop.set()
        for cam in self.cameras:
            cam.close()
        self.can.close()
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads.clear()

    def start_recording(self, notes: str = "") -> RecorderStatus | None:
        """Begin an episode. Returns None if one is already running.

        The already-running check happens INSIDE the lock and the answer is
        returned to the caller, rather than the caller checking `status()`
        first and then calling this. That ordering matters: two concurrent
        requests both read `running == False`, both proceed, and although the
        lock still ensures only one file is opened, the loser is told it
        succeeded. A client convinced it started a recording that does not
        exist is worse than an error, so the decision and the report have to
        be the same atomic step.
        """
        with self._lock:
            if self._recording.is_set():
                return None
            if not self._threads:
                self.start_capture()

            tb = TimeBase.now()
            meta = EpisodeMetadata(
                episode_id=new_episode_id(tb.t0_utc_s),
                created_utc=tb.t0_utc_s,
                is_mock=self.can.is_mock,
                source_note=("SIMULATED - not a real OpenArm" if self.can.is_mock
                             else "recorded from hardware"),
                camera_names=[c.name for c in self.cameras],
                t0_utc_s=tb.t0_utc_s,
                t0_monotonic_ns=tb.t0_monotonic_ns,
                notes=notes,
            )
            self._writer = self._writer_factory()
            self._writer.open(meta)

            self._status.running = True
            self._status.episode_id = meta.episode_id
            self._status.started_utc = tb.t0_utc_s
            self._status.joint_samples = 0
            self._status.frames = {c.name: 0 for c in self.cameras}
            self._status.dropped_queue = 0
            self._recording.set()
            return self.status()

    def stop_recording(self) -> EpisodeMetadata | None:
        with self._lock:
            if not self._recording.is_set():
                return None
            self._recording.clear()

        # Drain what is still queued before closing, so the tail of the
        # episode is not thrown away by the act of stopping.
        deadline = time.monotonic() + 2.0
        while not self._q.empty() and time.monotonic() < deadline:
            time.sleep(0.01)

        with self._lock:
            meta = self._writer.close() if self._writer else None
            self._writer = None
            self._status.running = False
            self._status.last_episode = meta.episode_id if meta else None
            self._status.episode_id = None
            return meta

    # -- capture threads ---------------------------------------------------

    def _run_can(self) -> None:
        asm = JointStateAssembler()
        with self.can:
            for frame in self.can.frames():
                if self._stop.is_set():
                    break
                state = asm.push(frame)
                if state is None:
                    continue

                self._live = LiveState(
                    t_mono_ns=state.t_mono_ns,
                    position=[round(float(x), 5) for x in state.position],
                    velocity=[round(float(x), 5) for x in state.velocity],
                    torque=[round(float(x), 5) for x in state.torque],
                    valid=[bool(x) for x in state.valid],
                    spread_s=float(state.spread_s),
                    rate_hz=self._live.rate_hz,
                    frames=dict(self._status.frames),
                )
                self._live_count += 1
                elapsed = time.monotonic() - self._t_start
                if elapsed > 0:
                    self._live.rate_hz = self._live_count / elapsed

                if self._recording.is_set():
                    self._enqueue(("joint", state))

    def _run_camera(self, cam: CameraSource) -> None:
        with cam:
            for frame in cam.frames():
                if self._stop.is_set():
                    break
                if self._recording.is_set():
                    self._enqueue(("frame", frame))

    def _enqueue(self, item) -> None:
        """Never blocks. Drops and counts instead."""
        try:
            self._q.put_nowait(item)
        except queue.Full:
            self._status.dropped_queue += 1

    # -- writer thread -----------------------------------------------------

    def _run_writer(self) -> None:
        while not self._stop.is_set():
            try:
                kind, payload = self._q.get(timeout=0.1)
            except queue.Empty:
                continue

            w = self._writer
            if w is None:
                continue
            try:
                if kind == "joint":
                    s = payload
                    w.write_joint_state(s.t_mono_ns, s.position, s.velocity,
                                        s.torque, s.valid, s.spread_s)
                    self._status.joint_samples += 1
                else:
                    f: Frame = payload
                    w.write_frame(f.camera, f.t_mono_ns, f.seq, f.exposure_s, f.image)
                    self._status.frames[f.camera] = self._status.frames.get(f.camera, 0) + 1
            except Exception:  # noqa: BLE001
                # A write failure must not kill the capture threads. The
                # episode is compromised either way; keeping the process alive
                # means the operator finds out now rather than at the end.
                self._status.dropped_queue += 1

    # -- introspection -----------------------------------------------------

    def status(self) -> RecorderStatus:
        s = self._status
        s.queue_depth = self._q.qsize()
        s.elapsed_s = (time.time() - s.started_utc) if s.started_utc and s.running else 0.0
        return s

    def live(self) -> LiveState:
        live = self._live
        live.frames = dict(self._status.frames)
        return live

    def latest_frame(self, camera: str) -> np.ndarray | None:
        """Most recent image from one camera, for the dashboard preview."""
        for cam in self.cameras:
            if cam.name == camera and hasattr(cam, "_render"):
                t = (time.monotonic() - self._t_start)
                return cam._render(t)
        return None
