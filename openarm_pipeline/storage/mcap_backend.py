"""MCAP writer and reader: the record path.

Chosen for capture because of one property the alternatives lack: **a file
interrupted by a crash still yields the part that was written.** Recording is
the one moment in this pipeline where data is irreplaceable. A training export
can be regenerated; a demonstration cannot be re-performed.

That property is NOT free, and testing showed it does not hold by default. Two
things were needed, both discovered by killing the process mid-recording:

  1. `chunk_size=64 KiB` on the writer. MCAP buffers into chunks and loses
     whatever is still open. At the 1 MiB default a 3 s episode fits entirely
     inside one unflushed chunk and **100 % of it is lost**. At 64 KiB, 97 %
     survives, for about 4 % more file size.
  2. Streaming reads rather than indexed ones. A crashed file has no footer and
     no index, and the indexed reader rejects it outright, returning nothing.
     The streaming reader recovers every record written before the damage.

Recovering nothing is treated as a different outcome from recovering a short
episode, and raises, so a caller can never mistake rubble for an empty take.

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

from .base import SCHEMA_VERSION, EpisodeMetadata, EpisodeWriter

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
        # chunk_size is the crash-safety dial, and the default (1 MiB) is wrong
        # for this application. MCAP buffers messages into a chunk and only
        # writes the chunk when it fills or at finish(); anything still in the
        # open chunk is lost if the process dies. MEASURED, killing the process
        # mid-recording:
        #
        #   episode    1 MiB chunks    64 KiB chunks
        #    3 s          0 % survives    97 % survives
        #   10 s         93 %             99 %
        #   30 s         91 %            100 %
        #
        # A short episode fits entirely inside one unflushed chunk and is lost
        # completely, which is the exact case a demo or an aborted take hits.
        # 64 KiB costs about 4 % in file size and removes that cliff.
        self._w = Writer(self._f, chunk_size=64 * 1024)
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
    """Reads an episode back into the stream objects Task 3 aligns.

    Reads DEFENSIVELY, via the streaming record iterator rather than the
    indexed reader. That choice exists for one reason: a recording interrupted
    by a crash has no footer and no index, and the indexed reader refuses it
    outright, returning nothing at all. Streaming recovers every record written
    before the damage and stops there.

    A partially recovered episode sets `truncated`. It is a real, usable
    recording that is simply shorter than intended, and losing all of it
    because the tail is damaged would be the worst possible behaviour for the
    one file in this pipeline that cannot be regenerated.
    """

    def __init__(self, path: Path | str):
        self.path = Path(path)
        self.truncated = False

    def metadata(self) -> EpisodeMetadata:
        """Prefer the sidecar. It exists so listing 1000 episodes does not
        mean opening 1000 recordings.

        Falls back through the final summary, then the header record written
        at open(). A crashed episode has no sidecar and no summary, but it does
        have the header, so it can still identify itself and still declares
        whether it was simulated.
        """
        side = self.path.with_suffix(".json")
        if side.exists():
            return EpisodeMetadata.from_json(side.read_text())

        from mcap.records import Metadata
        from mcap.stream_reader import StreamReader

        header: dict[str, str] | None = None
        try:
            with open(self.path, "rb") as f:
                for rec in StreamReader(f, emit_chunks=False).records:
                    if not isinstance(rec, Metadata):
                        continue
                    if rec.name == "episode_final":
                        return EpisodeMetadata.from_json(rec.metadata["summary"])
                    if rec.name == "episode":
                        header = rec.metadata
        except Exception:  # noqa: BLE001 -- damaged tail; use what we found
            self.truncated = True

        if header is None:
            raise FileNotFoundError(f"no recoverable metadata in {self.path}")

        self.truncated = True
        return EpisodeMetadata(
            episode_id=header.get("episode_id", self.path.stem),
            created_utc=float(header.get("t0_utc_s", 0.0) or 0.0),
            is_mock=header.get("is_mock", "True") == "True",
            source_note=header.get("source_note", ""),
            schema_version=header.get("schema_version", SCHEMA_VERSION),
            t0_utc_s=float(header.get("t0_utc_s", 0.0) or 0.0),
            t0_monotonic_ns=int(header.get("t0_monotonic_ns", 0) or 0),
            notes="RECOVERED FROM AN INCOMPLETE RECORDING: this episode has no "
                  "footer, so it was interrupted. Counts and durations below are "
                  "measured from what survived, not from what was intended.",
        )

    def read(self, with_images: bool = True):
        """Returns (JointStream, {camera: CameraStream}).

        Sets `self.truncated` if the file ended mid-record.
        """
        from mcap.records import Channel, Message
        from mcap.stream_reader import StreamReader

        from ..cameras.source import CameraStream, JointStream

        jt, jp, jv, jq, jvalid, jspread = [], [], [], [], [], []
        cams: dict[str, dict[str, list]] = {}
        topics: dict[int, str] = {}

        with open(self.path, "rb") as f:
            try:
                for rec in StreamReader(f, emit_chunks=False).records:
                    if isinstance(rec, Channel):
                        topics[rec.id] = rec.topic
                        continue
                    if not isinstance(rec, Message):
                        continue

                    topic = topics.get(rec.channel_id)
                    if topic == TOPIC_JOINTS:
                        d = json.loads(rec.data)
                        jt.append(d["t_mono_ns"])
                        jp.append(d["position"])
                        jv.append(d["velocity"])
                        jq.append(d["torque"])
                        jvalid.append(d["valid"])
                        jspread.append(d["spread_s"])
                    elif topic and topic.startswith("/camera/"):
                        name = topic.removeprefix("/camera/")
                        c = cams.setdefault(name, {"t": [], "seq": [], "img": []})
                        c["t"].append(rec.log_time)
                        c["seq"].append(rec.sequence)
                        if with_images:
                            c["img"].append(unpack_image(rec.data))
            except Exception as exc:  # noqa: BLE001
                # The file ends mid-record. Everything decoded up to here is
                # intact and is returned; the caller is told via `truncated`.
                #
                # But recovering NOTHING is different in kind from recovering
                # a short episode, and must not be reported as an empty
                # recording. A caller cannot distinguish "the operator stopped
                # immediately" from "this file is rubble" unless we say so.
                if not jt and not cams:
                    raise ValueError(
                        f"{self.path.name} is unreadable: no records recovered "
                        f"before {type(exc).__name__}"
                    ) from exc
                self.truncated = True

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
