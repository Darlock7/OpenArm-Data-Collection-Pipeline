"""Finding episodes on disk.

Listing reads the sidecar JSON, never the recording. That is the entire reason
the sidecar exists: `GET /episodes` on a directory of a thousand episodes must
not mean opening a thousand multi-gigabyte files. The sidecar is a few hundred
bytes and holds exactly what a listing needs.

The recording remains the source of truth. A sidecar can be regenerated from
its MCAP; the reverse is not true.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .base import EpisodeMetadata
from .mcap_backend import McapEpisodeReader


@dataclass
class EpisodeRef:
    episode_id: str
    mcap_path: Path
    meta: EpisodeMetadata
    size_bytes: int
    exports: list[str]      # already-generated HDF5 views

    def summary(self) -> dict:
        m = self.meta
        return {
            "episode_id": m.episode_id,
            "created_utc": m.created_utc,
            "duration_s": round(m.duration_s, 3),
            "is_mock": m.is_mock,
            "source_note": m.source_note,
            "n_joint_samples": m.n_joint_samples,
            "n_frames": m.n_frames,
            "joint_rate_hz": round(m.joint_rate_hz, 2),
            "camera_fps": {k: round(v, 2) for k, v in m.camera_fps.items()},
            "dropped_frames": m.dropped_frames,
            "mean_snapshot_spread_ms": round(m.mean_snapshot_spread_s * 1e3, 4),
            "size_bytes": self.size_bytes,
            "exports": self.exports,
            "notes": m.notes,
        }


class EpisodeRegistry:
    def __init__(self, data_dir: Path | str):
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)

    def list(self) -> list[EpisodeRef]:
        """Newest first. Skips anything unreadable rather than failing the
        whole listing, since one corrupt episode should not hide the rest."""
        refs: list[EpisodeRef] = []
        for path in sorted(self.data_dir.glob("*.mcap")):
            try:
                meta = McapEpisodeReader(path).metadata()
            except Exception:  # noqa: BLE001
                continue
            refs.append(EpisodeRef(
                episode_id=meta.episode_id,
                mcap_path=path,
                meta=meta,
                size_bytes=path.stat().st_size,
                exports=sorted(p.name for p in self.data_dir.glob(f"{path.stem}.*.h5")),
            ))
        refs.sort(key=lambda r: r.meta.created_utc, reverse=True)
        return refs

    def get(self, episode_id: str) -> EpisodeRef | None:
        for ref in self.list():
            if ref.episode_id == episode_id:
                return ref
        return None
