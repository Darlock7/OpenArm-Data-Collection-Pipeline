"""MCAP -> HDF5: the read path, and the place alignment happens.

This is the hinge of the whole design. Task 3 argued "store raw, align at read
time"; this module is read time. The alignment policy is an *argument* here,
which means one recording can produce several training sets:

    export(ep, policy="hermite")      # default, interpolated joints
    export(ep, policy="nearest")      # only genuine measurements
    export(ep, policy="window_mean")  # anti-aliased for hard downsampling

None of those touch the MCAP. Disagree with a choice a year from now and you
re-export rather than re-record.

LAYOUT follows the shape imitation learning code expects, close to ALOHA/ACT
and to LeRobot:

    /observations/joint_position   (N, 14)  float32
    /observations/joint_velocity   (N, 14)
    /observations/joint_torque     (N, 14)
    /observations/images/<camera>  (N, H, W, 3)  uint8
    /timing/t_s                    (N,)     seconds from episode start
    /timing/joint_dt_s             (N,)     distance to nearest real sample
    /timing/joint_valid            (N,)     bool
    /timing/camera_dt_s/<camera>   (N,)     signed staleness
    /timing/camera_valid/<camera>  (N,)     bool

The `/timing` group is the part most pipelines omit, and the reason to keep it
is that it makes alignment quality a *filterable property of the dataset*. A
training run can drop episodes whose bias exceeds a threshold. You cannot do
that if the error was discarded at export.

CHUNKING: images are chunked one frame per chunk. A shuffling dataloader reads
random timesteps, so chunking across time would force it to decompress a whole
block to reach a single frame. One frame per chunk means one decompression per
read, which is the access pattern that actually happens.

COMPRESSION: lzf, which is fast and cheap. gzip compresses image data barely
better while costing far more CPU, and the real answer for images is a video
codec rather than a general purpose compressor. See base.py on that gap.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ..cameras.sync import align
from ..config import PRIMARY_CAMERA, SYNC_TOLERANCE_S
from .base import EpisodeMetadata
from .mcap_backend import McapEpisodeReader


def export_hdf5(mcap_path: Path | str,
                out_path: Path | str | None = None,
                policy: str = "hermite",
                anchor: str = PRIMARY_CAMERA,
                tolerance_s: float = SYNC_TOLERANCE_S,
                compression: str | None = "lzf") -> Path:
    """Align an episode onto one timeline and write it as HDF5."""
    import h5py

    mcap_path = Path(mcap_path)
    out_path = Path(out_path) if out_path else mcap_path.with_suffix(f".{policy}.h5")

    reader = McapEpisodeReader(mcap_path)
    meta = reader.metadata()
    joints, streams = reader.read(with_images=True)

    if len(joints) == 0:
        raise ValueError(f"{mcap_path} contains no joint states")

    # A crashed recording may have lost the anchor camera entirely, since its
    # frames could sit in the destroyed tail. Refusing to export at that point
    # throws away a partially good episode over a choice that has an obvious
    # fallback: anchor on whichever surviving camera ran fastest, and record
    # that a substitution happened so nobody discovers it by surprise later.
    usable = {n: st for n, st in streams.items() if len(st) > 0}
    if not usable:
        raise ValueError(
            f"{mcap_path.name} has no camera frames at all, so there is no "
            f"timeline to anchor on. Joint states alone cannot form an episode."
        )
    anchor_fallback = None
    if anchor not in usable:
        anchor_fallback = anchor
        anchor = max(usable, key=lambda n: usable[n].measured_fps)

    ep = align(joints, usable, anchor=anchor,
               joint_policy=policy, tolerance_s=tolerance_s)
    q = ep.quality()
    n = len(ep)

    with h5py.File(out_path, "w") as h:
        # ---- provenance, at the root so it cannot be missed ----------------
        h.attrs["episode_id"] = meta.episode_id
        h.attrs["is_mock"] = bool(meta.is_mock)
        h.attrs["source_note"] = meta.source_note
        h.attrs["schema_version"] = meta.schema_version
        h.attrs["t0_utc_s"] = meta.t0_utc_s
        h.attrs["duration_s"] = meta.duration_s
        h.attrs["joint_names"] = np.array(meta.joint_names, dtype=object)

        # ---- how this view was produced. without it the file is unreproducible
        h.attrs["align_policy"] = policy
        h.attrs["align_anchor"] = anchor
        h.attrs["align_tolerance_s"] = tolerance_s
        h.attrs["joints_synthetic"] = bool(ep.joint_synthetic)
        h.attrs["source_mcap"] = mcap_path.name

        # Provenance for damage. Without these a truncated recording exports
        # to something indistinguishable from a clean short episode, and a
        # training run has no way to know it is missing the end of a
        # demonstration.
        h.attrs["source_truncated"] = bool(reader.truncated)
        if anchor_fallback:
            h.attrs["anchor_fallback_from"] = anchor_fallback
        if reader.truncated:
            h.attrs["warning"] = (
                "Exported from an INCOMPLETE recording. The source was "
                "interrupted, so this episode ends earlier than intended."
            )

        # ---- alignment quality, so episodes can be filtered on it ----------
        h.attrs["joint_bias_ms"] = q["joints"]["bias_ms"]
        h.attrs["joint_jitter_ms"] = q["joints"]["jitter_ms"]
        for cam, s in q["cameras"].items():
            h.attrs[f"bias_ms/{cam}"] = s["bias_ms"]
            h.attrs[f"jitter_ms/{cam}"] = s["jitter_ms"]
            h.attrs[f"worst_ms/{cam}"] = s["worst_ms"]
            h.attrs[f"coverage/{cam}"] = s["coverage"]

        obs = h.create_group("observations")
        obs.create_dataset("joint_position", data=ep.position, compression=compression)
        obs.create_dataset("joint_velocity", data=ep.velocity, compression=compression)
        obs.create_dataset("joint_torque", data=ep.torque, compression=compression)

        imgs = obs.create_group("images")
        for cam, stream in usable.items():
            if not stream.images:
                continue
            idx = ep.camera_idx[cam]
            valid = ep.camera_valid[cam]
            h_, w_ = stream.images[0].shape[:2]
            c_ = stream.images[0].shape[2] if stream.images[0].ndim == 3 else 1

            ds = imgs.create_dataset(
                cam, shape=(n, h_, w_, c_), dtype=np.uint8,
                chunks=(1, h_, w_, c_),      # one frame per chunk; see docstring
                compression=compression)

            for k in range(n):
                # An invalid match means no frame was close enough. Writing
                # zeros and flagging it beats silently reusing a distant frame,
                # which would look like data and be a lie.
                if valid[k] and 0 <= idx[k] < len(stream.images):
                    img = stream.images[idx[k]]
                    ds[k] = img.reshape(h_, w_, c_)
            ds.attrs["source_fps"] = float(stream.measured_fps)
            ds.attrs["dropped_frames"] = int(stream.dropped)

        t = h.create_group("timing")
        t.create_dataset("t_s", data=ep.t_s)
        t.create_dataset("t_mono_ns", data=ep.t_mono_ns)
        t.create_dataset("joint_dt_s", data=ep.joint_dt_s)
        t.create_dataset("joint_valid", data=ep.joint_valid)
        cdt = t.create_group("camera_dt_s")
        cok = t.create_group("camera_valid")
        for cam in ep.camera_dt_s:
            cdt.create_dataset(cam, data=ep.camera_dt_s[cam])
            cok.create_dataset(cam, data=ep.camera_valid[cam])

    return out_path


def summarise_hdf5(path: Path | str) -> dict:
    """Read back the attrs a consumer would filter on."""
    import h5py
    with h5py.File(path, "r") as h:
        n = h["timing/t_s"].shape[0]
        return {
            "episode_id": h.attrs["episode_id"],
            "is_mock": bool(h.attrs["is_mock"]),
            "samples": int(n),
            "policy": h.attrs["align_policy"],
            "anchor": h.attrs["align_anchor"],
            "joints_synthetic": bool(h.attrs["joints_synthetic"]),
            "cameras": sorted(h["observations/images"].keys()),
            "bias_ms": {k.split("/")[1]: float(v)
                        for k, v in h.attrs.items() if k.startswith("bias_ms/")},
        }
