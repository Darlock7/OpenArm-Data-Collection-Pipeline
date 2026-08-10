"""REST API over the recorder and the episode store.

    uvicorn openarm_pipeline.api.server:app --reload
    ->  http://127.0.0.1:8000        dashboard
        http://127.0.0.1:8000/docs   generated OpenAPI docs

DESIGN NOTES

Capture runs continuously from process start; recording is a separate flag.
The dashboard therefore shows live joint state whether or not an episode is
being written, and Start does not have to spin up hardware and wait for it to
settle. Pressing Start on a rig that is already streaming is instant and
cannot miss the first samples.

Errors are explicit. Asking for an episode that does not exist returns 404
with a message naming the id, not an empty list. Starting a recording while
one is running returns 409 rather than silently doing nothing, because a UI
that thinks it started a second recording and did not is worse than an error.

Live frames are served as raw RGB behind a 12 byte header rather than as PNG.
The browser can build an ImageData directly from that, which avoids an image
encoding dependency on the server and a decode on the client. Same header
format the storage layer uses, so there is one image convention in the project
rather than two.
"""

from __future__ import annotations

from pathlib import Path

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse, HTMLResponse, Response

from ..config import DATA_DIR
from ..recorder import Recorder
from ..storage.export import export_hdf5, summarise_hdf5
from ..storage.mcap_backend import pack_image
from ..storage.registry import EpisodeRegistry

app = FastAPI(
    title="OpenArm 2.0 Data Collection Pipeline",
    description="Teleoperation episode recorder. NOTE: this instance runs a "
                "SIMULATED arm and simulated cameras unless configured otherwise.",
    version="1.0",
)

recorder = Recorder()
registry = EpisodeRegistry(DATA_DIR)
_STATIC = Path(__file__).parent / "static"


@app.on_event("startup")
def _startup() -> None:
    recorder.start_capture()


@app.on_event("shutdown")
def _shutdown() -> None:
    recorder.stop_capture()


# ---------------------------------------------------------------- dashboard

@app.get("/", response_class=HTMLResponse, include_in_schema=False)
def dashboard() -> str:
    index = _STATIC / "index.html"
    if not index.exists():
        return "<h1>dashboard not built</h1>"
    return index.read_text()


# ---------------------------------------------------------------- status

@app.get("/api/status", tags=["recorder"])
def get_status() -> dict:
    """Recorder state, including queue depth and back-pressure drops."""
    s = recorder.status()
    return {
        "running": s.running,
        "episode_id": s.episode_id,
        "elapsed_s": round(s.elapsed_s, 2),
        "joint_samples": s.joint_samples,
        "frames": s.frames,
        "queue_depth": s.queue_depth,
        "dropped_queue": s.dropped_queue,
        "write_errors": s.write_errors,
        "is_mock": s.is_mock,
        "last_episode": s.last_episode,
        "episode_count": len(registry.list()),
    }


@app.get("/api/live", tags=["recorder"])
def get_live() -> dict:
    """Most recent joint snapshot. Not part of any recording."""
    live = recorder.live()
    return {
        "t_mono_ns": live.t_mono_ns,
        "position": live.position,
        "velocity": live.velocity,
        "torque": live.torque,
        "valid": live.valid,
        "spread_ms": round(live.spread_s * 1e3, 4),
        "rate_hz": round(live.rate_hz, 1),
        "is_mock": recorder.can.is_mock,
    }


@app.get("/api/cameras", tags=["recorder"])
def list_cameras() -> list[dict]:
    return [{"name": c.name, "fps": c.spec.fps, "width": c.spec.width,
             "height": c.spec.height, "is_mock": c.is_mock}
            for c in recorder.cameras]


@app.get("/api/cameras/{name}/frame", tags=["recorder"])
def camera_frame(name: str) -> Response:
    """Latest frame as raw RGB: uint32 h, uint32 w, uint32 c, then pixels."""
    img = recorder.latest_frame(name)
    if img is None:
        raise HTTPException(404, f"no camera named {name!r}")
    return Response(content=pack_image(img),
                    media_type="application/octet-stream",
                    headers={"Cache-Control": "no-store"})


# ---------------------------------------------------------------- recording

@app.post("/api/record/start", tags=["recorder"])
def start_recording(notes: str = Query("", description="free text stored in metadata")) -> dict:
    # No pre-check here on purpose. Asking `status()` first and then starting
    # is a check-then-act race: under concurrent requests both callers see
    # "not running" and both are told they succeeded. The recorder decides
    # inside its own lock and returns None to the loser.
    s = recorder.start_recording(notes=notes)
    if s is None:
        raise HTTPException(409, "a recording is already running")
    return {"started": True, "episode_id": s.episode_id, "is_mock": s.is_mock}


@app.post("/api/record/stop", tags=["recorder"])
def stop_recording() -> dict:
    meta = recorder.stop_recording()
    if meta is None:
        raise HTTPException(409, "no recording is running")
    return {"stopped": True, "episode_id": meta.episode_id,
            "duration_s": round(meta.duration_s, 3),
            "joint_samples": meta.n_joint_samples,
            "n_frames": meta.n_frames}


# ---------------------------------------------------------------- episodes

@app.get("/api/episodes", tags=["episodes"])
def list_episodes() -> list[dict]:
    """Newest first. Reads sidecar metadata only, never the recordings."""
    return [ref.summary() for ref in registry.list()]


@app.get("/api/episodes/{episode_id}", tags=["episodes"])
def get_episode(episode_id: str) -> dict:
    ref = registry.get(episode_id)
    if ref is None:
        raise HTTPException(404, f"no episode {episode_id!r}")
    out = ref.summary()
    out["exports_detail"] = [
        summarise_hdf5(ref.mcap_path.parent / name) for name in ref.exports
    ]
    return out


@app.get("/api/episodes/{episode_id}/download", tags=["episodes"])
def download_episode(episode_id: str, fmt: str = Query("mcap", pattern="^(mcap|h5)$"),
                     policy: str = Query("hermite")) -> FileResponse:
    """Download the raw recording, or an aligned HDF5 view of it.

    `fmt=h5` exports on demand if that view does not exist yet. That is the
    read-time alignment argument made concrete: the raw episode is one file,
    and any number of aligned views can be generated from it later without
    re-recording anything.
    """
    ref = registry.get(episode_id)
    if ref is None:
        raise HTTPException(404, f"no episode {episode_id!r}")

    if fmt == "mcap":
        return FileResponse(ref.mcap_path, media_type="application/octet-stream",
                            filename=ref.mcap_path.name)

    out = ref.mcap_path.with_suffix(f".{policy}.h5")
    if not out.exists():
        try:
            export_hdf5(ref.mcap_path, out, policy=policy)
        except (ValueError, KeyError) as exc:
            raise HTTPException(422, f"cannot export: {exc}") from exc
    return FileResponse(out, media_type="application/x-hdf5", filename=out.name)


@app.post("/api/episodes/{episode_id}/export", tags=["episodes"])
def export_episode(episode_id: str,
                   policy: str = Query("hermite",
                                       pattern="^(hermite|nearest|window_mean)$")) -> dict:
    """Produce an aligned HDF5 view under a chosen policy.

    One recording, several training sets. Nothing is re-captured and the MCAP
    is not modified.
    """
    ref = registry.get(episode_id)
    if ref is None:
        raise HTTPException(404, f"no episode {episode_id!r}")
    try:
        out = export_hdf5(ref.mcap_path, policy=policy)
    except (ValueError, KeyError) as exc:
        raise HTTPException(422, f"cannot export: {exc}") from exc
    return {"exported": out.name, "policy": policy, **summarise_hdf5(out)}
