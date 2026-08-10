"""MCAP writer and reader: the record path.

Chosen for capture because of one property the alternatives lack: **a file
truncated by a crash is still readable up to the last complete message.**
Recording is the one moment in this pipeline where data is irreplaceable. A
training export can be regenerated; a demonstration cannot be re-performed.

Beyond that, MCAP gives per-message timestamps on independent channels, which
means five streams at five different rates need no resampling, no padding and
no common clock at write time. That is exactly the constraint Task 3 argued
for, so the write format stops fighting the design instead of enabling it.

ENCODING CHOICES, and the trade in each
  * Joint states -> JSON with a registered JSON Schema. Self describing,
    inspectable with any MCAP tool, opens in Foxglove. Costs roughly 1 KB per
    message at 1 kHz. Wasteful, and worth it for a research rig where being
    able to open a file and read it beats saving a megabyte a second.
  * Images -> raw bytes behind a 12 byte header. JSON would base64 encode
    pixels, inflating them by a third for no benefit.

PRODUCTION GAP, stated rather than hidden: real deployments must encode video
(h264/AV1) and store a frame-index-to-timestamp map, as LeRobot does with mp4
plus parquet. Four cameras of raw full resolution frames is around 150 MB/s,
which is not something you keep. The synthetic frames here are small enough
that raw storage is fine for a demo and wrong for anything real.
"""

from __future__ import annotations

import json
import struct
from pathlib import Path

import numpy as np

from .base import EpisodeMetadata, EpisodeWriter

TOPIC_JOINTS = "/joint_states"
TOPIC_META = "/episode_metadata"


def _camera_topic(name: str) -> str:
    return f"/camera/{name}"


JOINT_SCHEMA = {
    "type": "object",
    "properties": {
        "t_mono_ns": {"type": "integer"},
        "position": {"type": "array", "items": {"type": "number"}},
        "velocity": {"type": "array", "items": {"type": "number"}},
        "torque": {"type": "array", "items": {"type": "number"}},
        "valid": {"type": "array", "items": {"type": "boolean"}},
        "spread_s": {"type": "number"},
    },
    "required": ["t_mono_ns", "position", "velocity", "torque", "valid", "spread_s"],
}

#: 12 byte image header: height, width, channels, all uint32 little endian.
_IMG_HEADER = struct.Struct("<III")


def pack_image(img: np.ndarray) -> bytes:
    a = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = a.shape[0], a.shape[1]
    c = a.shape[2] if a.ndim == 3 else 1
    return _IMG_HEADER.pack(h, w, c) + a.tobytes()


def unpack_image(blob: bytes) -> np.ndarray:
    h, w, c = _IMG_HEADER.unpack_from(blob, 0)
    arr = np.frombuffer(blob, dtype=np.uint8, offset=_IMG_HEADER.size)
    return arr.reshape((h, w, c) if c > 1 else (h, w))


class McapEpisodeWriter(EpisodeWriter):
    """Append-only episode log."""

    def __init__(self, directory: Path | str):
        self._dir = Path(directory)
        self._dir.mkdir(parents=True, exist_ok=True)
        self._path: Path | None = None
        self._f = None
        self._w = None
        self._joint_chan = None
        self._cam_chan: dict[str, int] = {}
        self._meta: EpisodeMetadata | None = None

        self._n_joints = 0
        self._n_frames: dict[str, int] = {}
        self._spread_sum = 0.0
        self._t_first: int | None = None
        self._t_last: int = 0
        self._cam_first: dict[str, int] = {}
        self._cam_last: dict[str, int] = {}
        self._cam_seq_first: dict[str, int] = {}
        self._cam_seq_last: dict[str, int] = {}

    @property
    def path(self) -> Path:
        if self._path is None:
            raise RuntimeError("writer not open")
        return self._path

    def open(self, meta: EpisodeMetadata) -> None:
        from mcap.writer import Writer

        self._meta = meta
        self._path = self._dir / f"{meta.filename_stem}.mcap"
        self._f = open(self._path, "wb")
        self._w = Writer(self._f)
        self._w.start(profile="openarm", library="openarm-data-collection-pipeline")

        joint_schema = self._w.register_schema(
            name="openarm.JointState", encoding="jsonschema",
            data=json.dumps(JOINT_SCHEMA).encode())
        self._joint_chan = self._w.register_channel(
            topic=TOPIC_JOINTS, message_encoding="json", schema_id=joint_schema,
            metadata={"joint_names": ",".join(meta.joint_names)})

        img_schema = self._w.register_schema(
            name="openarm.RawImage", encoding="",
            data=b"12-byte header: uint32 height, uint32 width, uint32 channels; "
                 b"then height*width*channels uint8 pixels, row major.")
        for cam in meta.camera_names:
            self._cam_chan[cam] = self._w.register_channel(
                topic=_camera_topic(cam), message_encoding="openarm/raw_image",
                schema_id=img_schema, metadata={"camera": cam})
            self._n_frames[cam] = 0

        # Metadata is written up front as well as at close, so a crashed
        # recording still says what it was and that it was simulated.
        self._w.add_metadata("episode", {
            "episode_id": meta.episode_id,
            "is_mock": str(meta.is_mock),
            "source_note": meta.source_note,
            "schema_version": meta.schema_version,
            "t0_utc_s": repr(meta.t0_utc_s),
            "t0_monotonic_ns": str(meta.t0_monotonic_ns),
        })

    def write_joint_state(self, t_mono_ns, position, velocity, torque,
                          valid, spread_s) -> None:
        payload = json.dumps({
            "t_mono_ns": int(t_mono_ns),
            "position": [round(float(x), 6) for x in position],
            "velocity": [round(float(x), 6) for x in velocity],
            "torque": [round(float(x), 6) for x in torque],
            "valid": [bool(x) for x in valid],
            "spread_s": float(spread_s),
        }).encode()
        self._w.add_message(channel_id=self._joint_chan, log_time=int(t_mono_ns),
                            publish_time=int(t_mono_ns), data=payload,
                            sequence=self._n_joints)
        self._n_joints += 1
        self._spread_sum += float(spread_s)
        if self._t_first is None:
            self._t_first = int(t_mono_ns)
        self._t_last = int(t_mono_ns)

    def write_frame(self, camera, t_mono_ns, seq, exposure_s, image) -> None:
        chan = self._cam_chan.get(camera)
        if chan is None:
            raise KeyError(f"camera {camera!r} was not declared in metadata")
        self._w.add_message(channel_id=chan, log_time=int(t_mono_ns),
                            publish_time=int(t_mono_ns), data=pack_image(image),
                            sequence=int(seq))
        self._n_frames[camera] = self._n_frames.get(camera, 0) + 1
        self._cam_first.setdefault(camera, int(t_mono_ns))
        self._cam_last[camera] = int(t_mono_ns)
        self._cam_seq_first.setdefault(camera, int(seq))
        self._cam_seq_last[camera] = int(seq)

    def close(self) -> EpisodeMetadata:
        if self._w is None or self._meta is None:
            raise RuntimeError("writer not open")

        m = self._meta
        m.n_joint_samples = self._n_joints
        m.n_frames = dict(self._n_frames)
        m.duration_s = (self._t_last - self._t_first) / 1e9 if self._t_first else 0.0
        m.joint_rate_hz = (self._n_joints - 1) / m.duration_s if m.duration_s > 0 else 0.0
        m.mean_snapshot_spread_s = (self._spread_sum / self._n_joints) if self._n_joints else 0.0

        for cam, n in self._n_frames.items():
            span = (self._cam_last.get(cam, 0) - self._cam_first.get(cam, 0)) / 1e9
            m.camera_fps[cam] = (n - 1) / span if span > 0 else 0.0
            # Sequence numbers advance even when a frame is dropped, so the
            # gap between span and count is the loss. Counting arrivals alone
            # would report zero drops no matter how many were lost.
            expected = self._cam_seq_last.get(cam, 0) - self._cam_seq_first.get(cam, 0) + 1
            m.dropped_frames[cam] = max(0, expected - n)

        self._w.add_metadata("episode_final", {"summary": m.to_json()})
        self._w.finish()
        self._f.close()
        self._w = None

        (self._path.with_suffix(".json")).write_text(m.to_json())
        return m


# ---------------------------------------------------------------------------

class McapEpisodeReader:
    """Reads an episode back into the stream objects Task 3 aligns."""

    def __init__(self, path: Path | str):
        self.path = Path(path)

    def metadata(self) -> EpisodeMetadata:
        """Prefer the sidecar. It exists so listing 1000 episodes does not
        mean opening 1000 recordings."""
        side = self.path.with_suffix(".json")
        if side.exists():
            return EpisodeMetadata.from_json(side.read_text())

        from mcap.reader import make_reader
        with open(self.path, "rb") as f:
            for record in make_reader(f).iter_metadata():
                if record.name == "episode_final":
                    return EpisodeMetadata.from_json(record.metadata["summary"])
        raise FileNotFoundError(f"no metadata in {self.path}")

    def read(self, with_images: bool = True):
        """Returns (JointStream, {camera: CameraStream})."""
        from mcap.reader import make_reader

        from ..cameras.source import CameraStream, JointStream

        jt, jp, jv, jq, jvalid, jspread = [], [], [], [], [], []
        cams: dict[str, dict[str, list]] = {}

        with open(self.path, "rb") as f:
            for _schema, channel, message in make_reader(f).iter_messages():
                if channel.topic == TOPIC_JOINTS:
                    d = json.loads(message.data)
                    jt.append(d["t_mono_ns"])
                    jp.append(d["position"])
                    jv.append(d["velocity"])
                    jq.append(d["torque"])
                    jvalid.append(d["valid"])
                    jspread.append(d["spread_s"])
                elif channel.topic.startswith("/camera/"):
                    name = channel.topic.removeprefix("/camera/")
                    c = cams.setdefault(name, {"t": [], "seq": [], "img": []})
                    c["t"].append(message.log_time)
                    c["seq"].append(message.sequence)
                    if with_images:
                        c["img"].append(unpack_image(message.data))

        joints = JointStream(
            t_mono_ns=np.array(jt, dtype=np.int64),
            position=np.array(jp, dtype=np.float32),
            velocity=np.array(jv, dtype=np.float32),
            torque=np.array(jq, dtype=np.float32),
            valid=np.array(jvalid, dtype=bool),
            spread_s=np.array(jspread, dtype=np.float64),
        )
        streams = {
            name: CameraStream(
                name=name,
                t_mono_ns=np.array(c["t"], dtype=np.int64),
                seq=np.array(c["seq"], dtype=np.int64),
                exposure_s=np.zeros(len(c["t"]), dtype=np.float64),
                images=c["img"] if with_images else None,
            )
            for name, c in cams.items()
        }
        return joints, streams
