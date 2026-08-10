"""Storage and API tests.

The claims worth proving:
  * a recording survives a round trip with its values and timestamps intact
  * dropped frames are counted from SEQUENCE GAPS, not from arrivals, because
    counting arrivals reports zero losses no matter how many were lost
  * simulated data is labelled in the filename, not only in metadata
  * the export applies the requested policy and records that it did
  * exporting never modifies the recording
  * the API fails loudly on conflicts rather than silently doing nothing
"""

import numpy as np
import pytest

from openarm_pipeline.storage.base import EpisodeMetadata, new_episode_id
from openarm_pipeline.storage.export import export_hdf5, summarise_hdf5
from openarm_pipeline.storage.mcap_backend import (
    McapEpisodeReader, McapEpisodeWriter, pack_image, unpack_image,
)
from openarm_pipeline.storage.registry import EpisodeRegistry

MS = 1_000_000
J = 14


def make_episode(tmp_path, n_joints=200, cam_seqs=None, is_mock=True):
    """A small synthetic episode written through the real writer."""
    cam_seqs = cam_seqs if cam_seqs is not None else {"zed_head": list(range(20))}
    meta = EpisodeMetadata(
        episode_id=new_episode_id(1_800_000_000.0),
        created_utc=1_800_000_000.0,
        is_mock=is_mock,
        camera_names=list(cam_seqs),
    )
    w = McapEpisodeWriter(tmp_path)
    w.open(meta)

    for i in range(n_joints):
        t = i * MS
        w.write_joint_state(
            t_mono_ns=t,
            position=np.full(J, i * 0.001, dtype=np.float32),
            velocity=np.full(J, 0.5, dtype=np.float32),
            torque=np.full(J, -1.25, dtype=np.float32),
            valid=np.ones(J, dtype=bool),
            spread_s=0.0003,
        )
    for cam, seqs in cam_seqs.items():
        for s in seqs:
            w.write_frame(cam, t_mono_ns=s * 16 * MS, seq=s, exposure_s=0.008,
                          image=np.full((8, 8, 3), s % 256, dtype=np.uint8))
    return w.close(), w.path


# ------------------------------------------------------------------ round trip

def test_image_header_round_trips():
    img = np.random.randint(0, 255, (12, 9, 3), dtype=np.uint8)
    np.testing.assert_array_equal(unpack_image(pack_image(img)), img)


def test_recording_round_trips_with_timestamps_intact(tmp_path):
    meta, path = make_episode(tmp_path, n_joints=50)
    joints, streams = McapEpisodeReader(path).read()

    assert len(joints) == 50
    np.testing.assert_array_equal(joints.t_mono_ns, np.arange(50) * MS)
    np.testing.assert_allclose(joints.position[10], 0.010, atol=1e-6)
    np.testing.assert_allclose(joints.torque[0], -1.25, atol=1e-6)
    assert len(streams["zed_head"]) == 20


def test_metadata_sidecar_matches_the_recording(tmp_path):
    meta, path = make_episode(tmp_path)
    side = EpisodeMetadata.from_json(path.with_suffix(".json").read_text())

    assert side.episode_id == meta.episode_id
    assert side.n_joint_samples == meta.n_joint_samples == 200
    assert side.is_mock is True


def test_simulated_episodes_are_labelled_in_the_filename(tmp_path):
    """Metadata alone is not enough. Someone will find these in a directory
    listing with no context."""
    _, mock_path = make_episode(tmp_path, is_mock=True)
    assert mock_path.name.startswith("mock_")

    _, real_path = make_episode(tmp_path / "real", is_mock=False)
    assert not real_path.name.startswith("mock_")


def test_dropped_frames_come_from_sequence_gaps_not_arrival_counts(tmp_path):
    """The whole reason Frame carries a sequence number.

    Nine frames arrive, but they are numbered 0..11 with three missing.
    Counting arrivals reports zero drops; reading the gaps reports three.
    """
    meta, _ = make_episode(tmp_path, cam_seqs={"zed_head": [0, 1, 2, 4, 5, 7, 8, 9, 11]})
    assert meta.n_frames["zed_head"] == 9
    assert meta.dropped_frames["zed_head"] == 3


# ------------------------------------------------------------------ export

def test_export_records_which_policy_produced_it(tmp_path):
    """A file that cannot say how it was made is not reproducible."""
    _, path = make_episode(tmp_path, n_joints=400,
                           cam_seqs={"zed_head": list(range(20))})

    for policy, synthetic in (("nearest", False), ("hermite", True)):
        out = export_hdf5(path, policy=policy)
        s = summarise_hdf5(out)
        assert s["policy"] == policy
        assert s["joints_synthetic"] is synthetic


def test_export_keeps_timing_so_quality_is_filterable(tmp_path):
    """The group most pipelines drop, and the reason to keep it: a training
    run can reject episodes whose alignment was poor."""
    import h5py
    _, path = make_episode(tmp_path, n_joints=400,
                           cam_seqs={"zed_head": list(range(20))})
    out = export_hdf5(path)

    with h5py.File(out, "r") as h:
        assert "timing/joint_dt_s" in h
        assert "timing/camera_dt_s/zed_head" in h
        assert "bias_ms/zed_head" in h.attrs
        # anchor camera needs no interpolation, so its bias is exactly zero
        assert h.attrs["bias_ms/zed_head"] == pytest.approx(0.0, abs=1e-9)
        assert bool(h.attrs["is_mock"]) is True


def test_export_does_not_modify_the_recording(tmp_path):
    """Re-export under a different policy a year later; the raw file is
    untouched."""
    _, path = make_episode(tmp_path, n_joints=400,
                           cam_seqs={"zed_head": list(range(20))})
    before = path.read_bytes()

    export_hdf5(path, policy="hermite")
    export_hdf5(path, policy="nearest")
    export_hdf5(path, policy="window_mean")

    assert path.read_bytes() == before


def test_export_falls_back_when_the_anchor_camera_is_missing(tmp_path):
    """A crashed recording can lose the anchor camera entirely, since its
    frames may sit in the destroyed tail.

    Refusing to export then would discard a partly good episode over a choice
    that has an obvious fallback. It anchors on the fastest surviving camera
    instead, and records the substitution so nobody finds out by accident.
    """
    import h5py
    _, path = make_episode(tmp_path, n_joints=400,
                           cam_seqs={"ceiling": list(range(20))})

    out = export_hdf5(path, anchor="zed_head")     # zed_head is not present
    with h5py.File(out, "r") as h:
        assert h.attrs["align_anchor"] == "ceiling"
        assert h.attrs["anchor_fallback_from"] == "zed_head"


def test_export_refuses_an_episode_with_no_camera_frames_at_all(tmp_path):
    """With no frames there is no timeline to anchor on, and joint states
    alone are not an episode. That one has to fail."""
    _, path = make_episode(tmp_path, n_joints=100, cam_seqs={"zed_head": []})
    with pytest.raises(ValueError, match="no camera frames"):
        export_hdf5(path)


# ------------------------------------------------------------------ registry

def test_registry_lists_newest_first_and_survives_junk(tmp_path):
    make_episode(tmp_path, n_joints=10)
    m2, _ = make_episode(tmp_path, n_joints=10)
    m2.created_utc += 500
    (tmp_path / f"{m2.filename_stem}.json").write_text(m2.to_json())
    (tmp_path / "not_an_episode.mcap").write_bytes(b"garbage")

    refs = EpisodeRegistry(tmp_path).list()
    assert len(refs) >= 1
    assert refs[0].meta.created_utc >= refs[-1].meta.created_utc


# ------------------------------------------------------------------ API

@pytest.fixture
def client(tmp_path):
    from fastapi.testclient import TestClient

    import openarm_pipeline.api.server as srv
    from openarm_pipeline.recorder import Recorder

    srv.recorder = Recorder(data_dir=tmp_path)
    srv.registry = EpisodeRegistry(tmp_path)
    with TestClient(srv.app) as c:
        yield c


def test_api_reports_that_it_is_simulated(client):
    assert client.get("/api/status").json()["is_mock"] is True
    assert client.get("/api/live").json()["is_mock"] is True


def test_api_refuses_to_start_two_recordings(client):
    """Silently ignoring the second Start would leave a UI convinced it is
    recording when it is not."""
    assert client.post("/api/record/start").status_code == 200
    assert client.post("/api/record/start").status_code == 409
    client.post("/api/record/stop")


def test_api_refuses_to_stop_when_not_recording(client):
    assert client.post("/api/record/stop").status_code == 409


def test_api_404s_name_the_missing_thing(client):
    r = client.get("/api/episodes/does_not_exist")
    assert r.status_code == 404
    assert "does_not_exist" in r.json()["detail"]


def test_api_rejects_an_unknown_export_policy(client):
    assert client.post("/api/episodes/x/export?policy=bogus").status_code == 422


def test_api_records_and_lists_an_episode(client):
    import time
    eid = client.post("/api/record/start?notes=pytest").json()["episode_id"]
    time.sleep(1.2)
    stopped = client.post("/api/record/stop").json()

    assert stopped["episode_id"] == eid
    assert stopped["joint_samples"] > 100

    eps = client.get("/api/episodes").json()
    assert any(e["episode_id"] == eid for e in eps)
    assert eps[0]["is_mock"] is True
    assert client.get(f"/api/episodes/{eid}/download").status_code == 200
