"""Resilience tests: the failure modes, not the happy path.

Every test here exists because fault injection found something. Two of them
guard real bugs that shipped and were caught by writing these:

  1. A crashed recording returned NOTHING. The README claimed a truncated file
     was still readable; it was not. Two causes: MCAP's 1 MiB default chunk
     meant a short episode sat entirely in one unflushed chunk, and the
     indexed reader refuses a file with no footer.
  2. Two concurrent POST /record/start both returned 200. Only one recording
     actually began, so the data was fine, but the loser was told it had
     started a recording that did not exist.

The rest assert properties claimed in the write-up that were previously
untested.
"""

import threading
import time

import numpy as np
import pytest

from openarm_pipeline.cameras.source import CameraStream, JointStream
from openarm_pipeline.cameras.sync import align
from openarm_pipeline.storage.base import EpisodeMetadata, EpisodeWriter, new_episode_id
from openarm_pipeline.storage.mcap_backend import McapEpisodeReader, McapEpisodeWriter

MS = 1_000_000
J = 14


def _write_episode(directory, n=800):
    meta = EpisodeMetadata(episode_id=new_episode_id(1.8e9), created_utc=1.8e9,
                           is_mock=True, camera_names=["zed_head"])
    w = McapEpisodeWriter(directory)
    w.open(meta)
    for i in range(n):
        w.write_joint_state(i * MS, np.full(J, i * 0.001, np.float32),
                            np.full(J, 0.5, np.float32), np.full(J, -1.0, np.float32),
                            np.ones(J, bool), 3e-4)
    return w, meta


# ------------------------------------------------------- crash recovery

def test_truncated_recording_still_yields_its_prefix(tmp_path):
    """The headline storage claim. It was false before this test existed."""
    w, meta = _write_episode(tmp_path, n=800)
    w.close()
    good = tmp_path / f"{meta.filename_stem}.mcap"

    cut = tmp_path / "cut.mcap"
    raw = good.read_bytes()
    cut.write_bytes(raw[: len(raw) // 2])

    reader = McapEpisodeReader(cut)
    joints, _ = reader.read(with_images=False)

    assert len(joints) > 0, "a half file must yield the half that survived"
    assert len(joints) < 800
    assert reader.truncated is True
    # and what it did recover must be intact, not garbage
    assert np.all(np.diff(joints.t_mono_ns) > 0)


def test_process_killed_mid_recording_loses_only_the_tail(tmp_path):
    """No close(), so no footer and no index: what an actual crash looks like.

    Guards the chunk_size=64 KiB choice. At MCAP's 1 MiB default this recovers
    nothing at all for an episode this size.
    """
    w, _ = _write_episode(tmp_path, n=800)
    w._f.flush()
    path = w.path
    del w                                   # never closed

    reader = McapEpisodeReader(path)
    joints, _ = reader.read(with_images=False)

    assert len(joints) > 600, f"expected most of 800 samples, got {len(joints)}"
    assert reader.truncated is True


def test_crashed_recording_can_still_identify_itself(tmp_path):
    """No sidecar and no summary, but the header survives, so the episode can
    still say what it is and that it was simulated."""
    w, meta = _write_episode(tmp_path, n=400)
    w._f.flush()
    path = w.path
    del w

    recovered = McapEpisodeReader(path).metadata()
    assert recovered.episode_id == meta.episode_id
    assert recovered.is_mock is True
    assert "INCOMPLETE" in recovered.notes


def test_unreadable_file_raises_instead_of_looking_empty(tmp_path):
    """Recovering nothing differs in kind from recovering a short episode.

    Returning an empty stream would let a caller mistake rubble for a take the
    operator stopped immediately.
    """
    bad = tmp_path / "rubble.mcap"
    bad.write_bytes(b"\x00" * 4096)
    with pytest.raises(ValueError, match="unreadable"):
        McapEpisodeReader(bad).read()


# ------------------------------------------------------- back-pressure

class _StalledDisk(EpisodeWriter):
    """A disk that has effectively stopped responding."""

    def __init__(self, path):
        self._p = path

    @property
    def path(self):
        return self._p

    def open(self, meta):
        self._meta = meta

    def write_joint_state(self, *a, **k):
        time.sleep(0.02)

    def write_frame(self, *a, **k):
        time.sleep(0.02)

    def close(self):
        return self._meta


def test_a_stalled_disk_does_not_stall_capture(tmp_path):
    """The reason writing happens on its own thread behind a bounded queue.

    Asserts all three properties at once: capture keeps its rate, the queue
    does not grow without limit, and what could not be written is counted
    rather than silently lost.
    """
    from openarm_pipeline.recorder import Recorder
    from openarm_pipeline.config import QUEUE_MAXSIZE

    rec = Recorder(data_dir=tmp_path,
                   writer_factory=lambda: _StalledDisk(tmp_path / "stalled.mcap"))
    rec.start_capture()
    time.sleep(0.5)
    rec.start_recording()
    time.sleep(2.5)

    status = rec.status()
    rate = rec.live().rate_hz
    rec.stop_recording()
    rec.stop_capture()

    assert rate > 500, f"capture collapsed to {rate:.0f} Hz behind a slow disk"
    assert status.queue_depth <= QUEUE_MAXSIZE, "queue grew past its bound"
    assert status.dropped_queue > 0, "drops happened but were not counted"


# ------------------------------------------------------- concurrency

def test_concurrent_starts_produce_exactly_one_winner(tmp_path):
    """Guards a check-then-act race.

    The endpoint used to ask status() and then start. Under concurrency both
    callers saw "not running" and both were told they had started, even though
    only one file was opened.
    """
    from fastapi.testclient import TestClient

    import openarm_pipeline.api.server as srv
    from openarm_pipeline.recorder import Recorder
    from openarm_pipeline.storage.registry import EpisodeRegistry

    srv.recorder = Recorder(data_dir=tmp_path)
    srv.registry = EpisodeRegistry(tmp_path)

    with TestClient(srv.app) as client:
        codes: list[int] = []
        lock = threading.Lock()

        def hit():
            r = client.post("/api/record/start")
            with lock:
                codes.append(r.status_code)

        threads = [threading.Thread(target=hit) for _ in range(8)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        assert codes.count(200) == 1, f"expected 1 winner, got {codes.count(200)}"
        assert codes.count(409) == 7

        time.sleep(0.8)
        client.post("/api/record/stop")
        assert len(client.get("/api/episodes").json()) == 1


# ------------------------------------------------------- degenerate input

def _joints(times_ms, n=4):
    t = np.array([int(x * MS) for x in times_ms], dtype=np.int64)
    ts = t / 1e9
    pos = np.column_stack([np.sin(2 * np.pi * ts + k) for k in range(n)]) if t.size \
        else np.zeros((0, n))
    vel = np.column_stack([np.cos(2 * np.pi * ts + k) for k in range(n)]) if t.size \
        else np.zeros((0, n))
    return JointStream(t_mono_ns=t, position=pos.astype(np.float32),
                       velocity=vel.astype(np.float32),
                       torque=np.zeros_like(pos, np.float32),
                       valid=np.ones(pos.shape, bool), spread_s=np.zeros(t.size))


def _cam(name, times_ms):
    t = np.array([int(x * MS) for x in times_ms], dtype=np.int64)
    return CameraStream(name=name, t_mono_ns=t, seq=np.arange(t.size, dtype=np.int64),
                        exposure_s=np.full(t.size, 0.008))


@pytest.mark.parametrize("label,joints,cams", [
    ("no joint samples at all", _joints([]), {"zed_head": _cam("zed_head", [0, 16, 33])}),
    ("a single joint sample", _joints([50]), {"zed_head": _cam("zed_head", [0, 50, 100])}),
    ("anchor camera recorded nothing", _joints([0, 10, 20]), {"zed_head": _cam("zed_head", [])}),
    ("duplicate timestamps", _joints([0, 10, 10, 10, 20]), {"zed_head": _cam("zed_head", [5, 15])}),
    ("one camera recorded nothing", _joints(list(range(0, 51))),
     {"zed_head": _cam("zed_head", [10, 30]), "ceiling": _cam("ceiling", [])}),
])
def test_degenerate_episodes_align_without_crashing(label, joints, cams):
    """A broken episode must produce a marked-invalid result, never an
    exception and never a plausible-looking wrong answer.

    The contract is "finite WHERE VALID", not "finite everywhere". Samples
    with nothing behind them come back as NaN, which is deliberate: NaN
    propagates loudly if a consumer ignores the valid mask, whereas 0.0 would
    sit there looking exactly like a legitimate joint angle.
    """
    ep = align(joints, cams)
    assert ep.joint_valid.shape[0] == len(ep)
    assert np.isfinite(ep.position[ep.joint_valid]).all(), \
        f"non-finite value on a sample marked valid: {label}"


def test_a_nan_in_one_joint_does_not_spread_to_the_others():
    joints = _joints(list(range(0, 51)))
    joints.position[10, 0] = np.nan
    ep = align(joints, {"zed_head": _cam("zed_head", [5, 25, 45])})
    assert np.isfinite(ep.position[:, 1:]).all(), "NaN leaked across joints"


def test_a_dead_motor_degrades_the_recording_rather_than_stopping_it():
    from openarm_pipeline.can.assembler import JointStateAssembler
    from openarm_pipeline.can.protocol import JointFeedback, encode_feedback
    from openarm_pipeline.can.source import CANFrame
    from openarm_pipeline.config import JOINTS, MOTOR_SPECS

    asm = JointStateAssembler()
    DEAD = 5
    snapshots = []

    for cycle in range(10):
        for idx, joint in enumerate(JOINTS):
            if idx == DEAD and cycle >= 3:
                continue                                # motor 5 stops replying
            fb = JointFeedback(joint.can_id, 0.1 * cycle, 0.0, 0.0, 40, 40, "ok")
            s = asm.push(CANFrame(joint.interface, joint.can_id,
                                  encode_feedback(fb, MOTOR_SPECS[joint.motor]),
                                  cycle * MS + idx * 1000))
            if s is not None:
                snapshots.append(s)

    assert len(snapshots) >= 8, "the stream stopped when one motor died"
    assert not snapshots[-1].valid[DEAD], "the dead joint was not marked stale"
    others = [i for i in range(J) if i != DEAD]
    assert snapshots[-1].valid[others].all(), "healthy joints were marked stale"


# ------------------------------------------------------- the disk gives out

class _FailingDisk(EpisodeWriter):
    """Accepts writes for a while, then the filesystem refuses."""

    def __init__(self, directory, fail_after=200):
        self.real = McapEpisodeWriter(directory)
        self.n = 0
        self.fail_after = fail_after

    @property
    def path(self):
        return self.real.path

    def open(self, meta):
        self.real.open(meta)

    def write_joint_state(self, *a, **k):
        self.n += 1
        if self.n > self.fail_after:
            raise OSError(28, "No space left on device")
        self.real.write_joint_state(*a, **k)

    def write_frame(self, *a, **k):
        if self.n > self.fail_after:
            raise OSError(28, "No space left on device")
        self.real.write_frame(*a, **k)

    def close(self):
        return self.real.close()


def test_a_failing_disk_is_survived_and_reported_distinctly(tmp_path):
    """"The disk cannot keep up" and "the disk refused the write" need
    completely different responses, so they get different counters.

    They shared one before this test, which meant an operator watching the
    dashboard could not tell a slow disk from a broken one.
    """
    from openarm_pipeline.recorder import Recorder

    rec = Recorder(data_dir=tmp_path,
                   writer_factory=lambda: _FailingDisk(tmp_path, fail_after=200))
    rec.start_capture()
    time.sleep(0.4)
    rec.start_recording()
    time.sleep(2.0)

    status = rec.status()
    rate = rec.live().rate_hz
    meta = rec.stop_recording()          # must not raise
    rec.stop_capture()

    assert meta is not None, "stop_recording raised or lost the episode"
    assert rate > 500, f"capture collapsed to {rate:.0f} Hz on write failures"
    assert status.write_errors > 0, "write failures were not counted"
    assert status.dropped_queue == 0, "write failures were miscounted as back-pressure"

    # and whatever landed before the failure is still readable
    joints, _ = McapEpisodeReader(rec.data_dir / f"{meta.filename_stem}.mcap") \
        .read(with_images=False)
    assert len(joints) > 0


def test_rapid_start_stop_cycles_do_not_leak_threads_or_ids(tmp_path):
    from openarm_pipeline.recorder import Recorder

    rec = Recorder(data_dir=tmp_path)
    rec.start_capture()
    time.sleep(0.3)
    expected_threads = len(rec._threads)

    ids = []
    for _ in range(4):
        rec.start_recording()
        time.sleep(0.2)
        m = rec.stop_recording()
        if m:
            ids.append(m.episode_id)

    assert len(rec._threads) == expected_threads, "cycling spawned extra threads"
    rec.stop_capture()

    assert len(set(ids)) == 4, f"episode ids collided: {ids}"
    assert len(list(tmp_path.glob("*.mcap"))) == 4


# ------------------------------------------------------- exporting damage

def test_a_truncated_episode_exports_and_says_it_was_truncated(tmp_path):
    """A damaged recording is still worth training on, but the resulting file
    must not be indistinguishable from a clean short episode."""
    import h5py

    meta = EpisodeMetadata(episode_id=new_episode_id(1.8e9), created_utc=1.8e9,
                           is_mock=True, camera_names=["zed_head"])
    w = McapEpisodeWriter(tmp_path)
    w.open(meta)
    for i in range(4000):                       # interleaved, as a real rig writes
        w.write_joint_state(i * MS, np.full(J, i * 1e-4, np.float32),
                            np.full(J, 0.5, np.float32), np.full(J, -1.0, np.float32),
                            np.ones(J, bool), 3e-4)
        if i % 16 == 0:
            w.write_frame("zed_head", i * MS, i // 16, 0.008,
                          np.full((8, 8, 3), (i // 16) % 250, np.uint8))
    w.close()

    full = tmp_path / f"{meta.filename_stem}.mcap"
    cut = tmp_path / "cut.mcap"
    cut.write_bytes(full.read_bytes()[: int(full.stat().st_size * 0.6)])

    from openarm_pipeline.storage.export import export_hdf5
    out = export_hdf5(cut, policy="hermite")

    with h5py.File(out, "r") as h:
        assert h["timing/t_s"].shape[0] > 0, "nothing survived the export"
        assert bool(h.attrs["source_truncated"]) is True
        assert "INCOMPLETE" in h.attrs["warning"]


# ------------------------------------------------------- stereo

def test_the_zed_actually_emits_a_stereo_pair():
    """The brief lists it as "ZED stereo".

    A mono image behind a stereo=True flag would be the flag lying, and would
    silently drop half the sensor.
    """
    from openarm_pipeline.cameras.mock import make_mock_cameras

    cams = {c.name: c for c in make_mock_cameras()}
    zed, wrist = cams["zed_head"], cams["wrist_left"]

    assert zed.spec.stereo is True
    assert wrist.spec.stereo is False

    stereo_frame = zed._render(0.30)
    mono_frame = wrist._render(0.30)

    assert stereo_frame.shape[1] == 2 * zed.mono_w, "the ZED frame is not double width"
    assert mono_frame.shape[1] == wrist.mono_w

    # the two eyes must differ, otherwise it is one image printed twice
    left = stereo_frame[:, : zed.mono_w]
    right = stereo_frame[:, zed.mono_w:]
    assert not np.array_equal(left, right), "both eyes are identical; disparity is missing"
