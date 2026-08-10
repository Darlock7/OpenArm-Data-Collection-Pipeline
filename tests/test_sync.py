"""Synchronisation tests.

The claims worth proving here are not "the code runs" but:
  * the error we report has the right SIGN, so bias is detectable
  * interpolation is exact where it should be, and better than linear
  * out-of-range queries are marked, never silently extrapolated
  * the anchor camera needs no interpolation at all
  * a slow camera is not punished for being slow
"""

import numpy as np
import pytest

from openarm_pipeline.cameras.source import CameraStream, JointStream
from openarm_pipeline.cameras.sync import (
    align, build_timeline, hermite, nearest_index, window_mean,
)

MS = 1_000_000  # ns


def joints_at(times_ms, n_joints=2, fn=None):
    """A JointStream sampling `fn` (default: a sine) at the given times."""
    t = np.array([int(x * MS) for x in times_ms], dtype=np.int64)
    ts = t / 1e9
    if fn is None:
        w = 2 * np.pi * 1.0
        pos = np.column_stack([np.sin(w * ts + j) for j in range(n_joints)])
        vel = np.column_stack([w * np.cos(w * ts + j) for j in range(n_joints)])
    else:
        pos, vel = fn(ts)
    return JointStream(
        t_mono_ns=t,
        position=pos.astype(np.float32),
        velocity=vel.astype(np.float32),
        torque=np.zeros_like(pos, dtype=np.float32),
        valid=np.ones(pos.shape, dtype=bool),
        spread_s=np.zeros(t.size),
    )


def camera_at(name, times_ms):
    t = np.array([int(x * MS) for x in times_ms], dtype=np.int64)
    return CameraStream(name=name, t_mono_ns=t,
                        seq=np.arange(t.size, dtype=np.int64),
                        exposure_s=np.full(t.size, 0.008))


# ---------------------------------------------------------------- primitives

def test_nearest_error_is_signed_so_bias_is_visible():
    """dt = sample - query. A stale sample must read NEGATIVE.

    If this returned absolute error, a systematic 50 ms lag would average to
    +50 instead of -50 and look identical to a 50 ms lead. Direction is the
    whole point: it tells you which way the offset runs.
    """
    samples = np.array([0, 10, 20], dtype=np.int64) * MS
    idx, dt = nearest_index(samples, np.array([12], dtype=np.int64) * MS)

    assert idx[0] == 1                      # the sample at 10 ms
    assert dt[0] == pytest.approx(-0.002)   # 2 ms OLDER than the query


def test_nearest_picks_the_closer_of_two_neighbours():
    samples = np.array([0, 100], dtype=np.int64) * MS
    _, dt = nearest_index(samples, np.array([40, 60], dtype=np.int64) * MS)
    assert dt[0] == pytest.approx(-0.040)   # 40 -> back to 0
    assert dt[1] == pytest.approx(+0.040)   # 60 -> forward to 100


def test_nearest_on_empty_stream_is_marked_not_crashed():
    idx, dt = nearest_index(np.array([], dtype=np.int64),
                            np.array([5], dtype=np.int64) * MS)
    assert idx[0] == -1
    assert np.isnan(dt[0])


def test_hermite_is_exact_at_the_sample_points():
    """Interpolation must not perturb data it was handed."""
    js = joints_at([0, 10, 20, 30])
    out = hermite(js.t_mono_ns, js.position, js.velocity, js.t_mono_ns)
    np.testing.assert_allclose(out, js.position, atol=1e-5)


def test_hermite_beats_linear_on_a_smooth_signal():
    """The reason to use velocity rather than ignore it.

    Sample a sine coarsely, reconstruct at 10x the density, compare against
    ground truth. Hermite knows the slope at each endpoint; linear does not.
    """
    coarse = joints_at(list(range(0, 201, 20)))            # every 20 ms
    q = np.arange(0, 200, 2, dtype=np.int64) * MS          # every 2 ms

    truth = np.sin(2 * np.pi * (q / 1e9))
    herm = hermite(coarse.t_mono_ns, coarse.position, coarse.velocity, q)[:, 0]
    lin = np.interp(q.astype(float), coarse.t_mono_ns.astype(float),
                    coarse.position[:, 0])

    err_h = np.abs(herm - truth).max()
    err_l = np.abs(lin - truth).max()
    assert err_h < err_l / 5, f"hermite {err_h:.5f} vs linear {err_l:.5f}"


def test_hermite_holds_at_the_endpoint_instead_of_extrapolating():
    """A cubic run past its data produces confident nonsense. Clamp instead."""
    js = joints_at([0, 10, 20])
    far = np.array([500], dtype=np.int64) * MS
    out = hermite(js.t_mono_ns, js.position, js.velocity, far)
    np.testing.assert_allclose(out[0], js.position[-1], atol=1e-5)


def test_window_mean_averages_only_inside_the_window():
    t = np.arange(0, 100, dtype=np.int64) * MS          # 0..99 ms, 1 kHz
    vals = np.arange(100, dtype=np.float64)[:, None]
    out, n = window_mean(t, vals, np.array([50], dtype=np.int64) * MS, window_s=0.010)

    assert n[0] == 10                        # 45..54 inclusive
    assert out[0, 0] == pytest.approx(49.5)


def test_window_mean_attenuates_high_frequency_content():
    """The anti-aliasing claim, actually demonstrated.

    A 200 Hz component sampled at 1 kHz survives decimation to 60 Hz as a
    spurious low-frequency signal. Averaging over the window removes most of
    it before that can happen.
    """
    t = np.arange(0, 1000, dtype=np.int64) * MS // 1     # 1 kHz for 1 s
    t = np.arange(0, 1000, dtype=np.int64) * (MS // 1)
    ts = t / 1e9
    clean = np.sin(2 * np.pi * 1.0 * ts)
    noisy = clean + 0.5 * np.sin(2 * np.pi * 200.0 * ts)

    q = np.arange(0, 1000, 17, dtype=np.int64) * (MS // 1)   # ~59 Hz
    truth = np.sin(2 * np.pi * 1.0 * (q / 1e9))

    raw_idx, _ = nearest_index(t, q)
    err_raw = np.abs(noisy[raw_idx] - truth).mean()
    smoothed, _ = window_mean(t, noisy[:, None], q, window_s=0.017)
    err_smooth = np.abs(smoothed[:, 0] - truth).mean()

    assert err_smooth < err_raw / 2, f"raw {err_raw:.4f} vs smoothed {err_smooth:.4f}"


# ---------------------------------------------------------------- alignment

def test_anchor_camera_needs_no_interpolation():
    """The reason to anchor on a camera instead of a synthetic grid.

    Every sample lands exactly on a real frame, so the anchor's timing error
    is identically zero. A fixed grid would make every stream synthetic,
    including this one.
    """
    js = joints_at(list(range(0, 101)))
    cams = {"zed_head": camera_at("zed_head", [0, 16, 33, 50, 66, 83, 100]),
            "ceiling": camera_at("ceiling", [5, 71])}

    ep = align(js, cams, anchor="zed_head")
    np.testing.assert_array_equal(ep.camera_dt_s["zed_head"], 0.0)
    assert ep.camera_valid["zed_head"].all()


def test_slow_camera_is_not_penalised_for_being_slow():
    """A 15 fps camera's best possible match is half a frame away, ~33 ms.

    Judging it against the 5 ms joint tolerance would reject nearly every
    frame for doing nothing wrong, so the camera tolerance scales with its
    own measured rate.
    """
    js = joints_at(list(range(0, 201)))
    cams = {"zed_head": camera_at("zed_head", list(range(0, 201, 16))),
            "ceiling": camera_at("ceiling", list(range(0, 201, 66)))}

    ep = align(js, cams, anchor="zed_head")
    assert ep.camera_valid["ceiling"].mean() > 0.9
    # ...and the staleness is still reported honestly, not hidden
    assert np.abs(ep.camera_dt_s["ceiling"]).max() > 0.020


def test_queries_outside_the_joint_span_are_marked_invalid():
    js = joints_at([100, 110, 120])
    cams = {"zed_head": camera_at("zed_head", [50, 105, 115, 300])}

    ep = align(js, cams, anchor="zed_head")
    assert not ep.joint_valid[0]     # 50 ms, before the joints start
    assert not ep.joint_valid[-1]    # 300 ms, after they end
    assert ep.joint_valid[1:3].all()


def test_interpolated_output_is_flagged_as_synthetic():
    """An interpolated value is not a measurement and must not look like one."""
    js = joints_at(list(range(0, 51)))
    cams = {"zed_head": camera_at("zed_head", [10, 20, 30])}

    assert align(js, cams, joint_policy="hermite").joint_synthetic is True
    assert align(js, cams, joint_policy="nearest").joint_synthetic is False


def test_quality_report_separates_bias_from_jitter():
    """A constant offset must show up as bias, not disappear into jitter."""
    js = joints_at(list(range(0, 201)))
    # camera frames sitting a constant 7 ms after each joint sample
    cams = {"zed_head": camera_at("zed_head", [x + 0.4 for x in range(0, 200, 16)]),
            "wrist_left": camera_at("wrist_left", [x - 7 for x in range(0, 200, 16)])}

    q = align(js, cams, anchor="zed_head").quality()
    assert q["cameras"]["wrist_left"]["bias_ms"] == pytest.approx(-7.4, abs=1.5)
    assert q["cameras"]["wrist_left"]["jitter_ms"] < 1.0


def test_alignment_does_not_mutate_the_streams():
    """Alignment is a query. Ask twice with different policies, get two views
    of identical underlying data."""
    js = joints_at(list(range(0, 101)))
    cams = {"zed_head": camera_at("zed_head", list(range(0, 101, 16)))}

    before_t = js.t_mono_ns.copy()
    before_p = js.position.copy()

    align(js, cams, joint_policy="hermite")
    align(js, cams, joint_policy="nearest")
    align(js, cams, joint_policy="window_mean", window_s=0.02)

    np.testing.assert_array_equal(js.t_mono_ns, before_t)
    np.testing.assert_array_equal(js.position, before_p)


def test_build_timeline_rejects_an_unknown_anchor():
    with pytest.raises(KeyError, match="not among"):
        build_timeline({"ceiling": camera_at("ceiling", [0, 10])}, anchor="zed_head")
