<div align="center">

# OpenArm 2.0 — Data Collection Pipeline

**A teleoperation recording platform for the [OpenArm 2.0](https://docs.openarm.dev) bimanual robot arm.**
Reads joint state over CAN FD, synchronises it with four camera streams, stores episodes for
robot learning, and serves them over a REST API with a live dashboard.

<img src="docs/openarm-topology.svg" alt="OpenArm 2.0 rig: 14 joints across two CAN FD buses, four cameras" width="100%">

</div>

---

## Status

| | Task | State | Notes |
|:--:|---|:--|---|
| 1 | **CAN interface setup** | 📝 **Documented, not run** | No hardware. Full reasoning below |
| 2 | **CAN data reading** | ✅ **Working** | 995.8 Hz sustained against a simulated arm |
| 3 | Multi-camera synchronisation | ⬜ Not started | |
| 4 | Storage backend + REST API | ⬜ Not started | |
| 5 | Monitoring dashboard | ⬜ Not started | |

> ### ⚠️ Read this first
>
> **I had no OpenArm, no CAN adapter, and no cameras.** Every data source in this project has
> two implementations behind one interface: a **simulated** one that runs anywhere, and a
> **hardware** one written against the SocketCAN and python-can documentation but **never
> executed**.
>
> Simulated data is labelled as such in episode metadata, in filenames, and on screen. Nothing
> here should ever be mistaken for a real demonstration.

---

## Quick start

```bash
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt

./.venv/bin/python -m pytest tests/ -q          # 13 tests
./.venv/bin/python scripts/read_joints.py       # live joint state, simulated arm
```

---

## Task 1 — CAN FD interface setup

> *"Follow the setup guide to configure the CAN FD interfaces. Run `openarm-can-cli
> can_configure` and set the zero position on both can0 and can1. Show a screenshot or terminal
> output confirming the interfaces are UP and zero position is set."*

**Status: not performed.** Bringing up a CAN interface needs a physical adapter, and zeroing
writes a value into physical motors. I had neither, and I was on macOS, which has no SocketCAN
at all. The only thing I could have submitted was a fabricated screenshot.

What follows is the procedure I would run and why. Everything marked **[UNVERIFIED]** is
reasoning from documentation rather than something I observed.

### Why the rig needs two buses

The single most useful thing I worked out while reading the spec. The task hands you `can0` and
`can1` without saying why, and the reason falls out of arithmetic.

Each motor exchanges **two messages per control cycle** — a command from the host and a reply
from the motor — and both travel on the same shared pair of wires. At the 1 kHz control loop
OpenArm 2.0 runs:

```
  7 motors  ×  2 messages  ×  1000 Hz          =  14,000 messages/sec  per arm
  14,000 msg/sec  ×  ~50 µs per CAN FD frame   ≈  0.70 s of wire time per second
                                               ≈  70 % bus load
```

70 % is already past the point where arbitration delay makes worst-case latency unpredictable;
the usual design guidance is to stay under 50–60 %. **Both arms on one bus would need ~140 %,
which is not physically possible.** Split one arm per bus and each sits near **35 %**, with
headroom for retransmission.

> **[UNVERIFIED]** — this is my own estimate, not a figure from the OpenArm docs, and the frame
> timing is approximate. If the rig uses a single broadcast sync frame instead of per-motor
> commands, the count drops substantially. The conclusion holds either way: one bus cannot carry
> both arms.

### Bringing the interfaces up

SocketCAN presents a CAN adapter as an ordinary Linux network interface, so it is configured
with `ip link` — the same tool used for ethernet.

```bash
sudo ip link set can0 down                     # must be down to change parameters

sudo ip link set can0 type can \
     bitrate  1000000  sample-point  0.75 \    # arbitration phase
     dbitrate 5000000  dsample-point 0.75 \    # data phase — the 5 Mbit/s from the brief
     fd on                                     # without this it falls back to classic CAN

sudo ip link set can0 txqueuelen 1000          # default of 10 drops frames at 1 kHz
sudo ip link set can0 up
```

Then identically for `can1`.

<details>
<summary><b>Why each flag is there, and two deliberate omissions</b></summary>

<br>

| Flag | Reason |
|---|---|
| `bitrate 1000000` | Arbitration phase. Every node must agree on it, and it is where bus arbitration happens, so it stays slow and robust. |
| `dbitrate 5000000` | Data phase. CAN FD's core trick: once a node has won arbitration nobody is competing, so the payload can clock much faster. |
| `fd on` | Enables CAN FD. Without it the controller stays in classic mode — 8-byte payloads, no data-phase switch. |
| `sample-point 0.75` | Where in each bit the controller samples. The CiA-recommended value at these rates; trades noise margin against tolerance for clock mismatch between nodes. |
| `txqueuelen 1000` | Kernel transmit buffer. The default of 10 is sized for a low-rate bus and will drop outbound frames at 1 kHz across 7 motors. |

**No `restart-ms`.** Automatic bus-off recovery sounds helpful and is wrong for a data recorder.
A bus-off event means the recording is already compromised, and silently recovering hides that
from the operator. Fail visibly instead.

**Not done from Python.** Reconfiguring an interface needs `CAP_NET_ADMIN`, and a recording
process should not run privileged. `socketcan.py::setup_commands()` returns these commands as
strings rather than executing them.

</details>

### What the screenshot would show

```console
$ ip -details -statistics link show can0

2: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 72 qdisc pfifo_fast state UP
    can <FD> state ERROR-ACTIVE restart-ms 0
      bitrate 1000000 sample-point 0.750
      dbitrate 5000000 dsample-point 0.750
      re-started bus-errors arbit-lost error-warn error-pass bus-off
      0          0          0          0          0          0
```

Four things I would actually check, in order:

1. **`state UP`** — the interface is running.
2. **`mtu 72`** — 64 data bytes + 8 header, so FD is active. **`mtu 16` means FD did not take**
   and the bus is in classic mode, which will not carry full Damiao frames.
3. **`ERROR-ACTIVE`** — the healthy state. `ERROR-PASSIVE` or `BUS-OFF` points at wiring:
   termination, grounding, or a bitrate mismatch.
4. **Error counters at zero.** Non-zero and climbing on an idle bus is a physical-layer fault,
   not a software one.

### Zero position

The Damiao motors are **quasi-direct-drive**, so the arm is backdrivable and gets moved by hand.
Each encoder reports an angle against an internal reference with no relationship to the arm's
geometry. Zeroing declares the **current physical pose** to be 0 rad on every joint.

It is the same operation as taring a scale, and it matters for the same reason: without it,
"joint 4 at 0.5 rad" means something different every session, and a policy trained on Monday's
data is handed a different coordinate frame on Tuesday.

Underneath the CLI, zeroing is one reserved 8-byte payload sent to each motor ID:

```
FF FF FF FF FF FF FF FE     set zero position
FF FF FF FF FF FF FF FC     enable
FF FF FF FF FF FF FF FD     disable
FF FF FF FF FF FF FF FB     clear error
```

These are implemented in [`can/protocol.py`](openarm_pipeline/can/protocol.py) as `CMD_SET_ZERO`
and friends, and [`tests/test_protocol.py`](tests/test_protocol.py) asserts the byte sequence.

<details>
<summary><b>Physical procedure, and what I expect to go wrong</b></summary>

<br>

Before running it: power the motors but leave them **disabled** so the arm moves freely; move
both arms into the documented home pose against their mechanical hard stops (OpenArm 2.0 has
stops on every axis, which is what makes the pose repeatable rather than eyeballed); **support
the arms**, because a QDD arm has almost no gearbox friction and will fall under its own weight
the instant it is released; then zero, enable, and confirm every joint reads ~0.

Ranked by how likely I think they are:

| Failure | Symptom | Cause |
|---|---|---|
| Bitrate mismatch | `ERROR-PASSIVE`, counters climbing, no valid frames | Motors configured for a different data-phase rate |
| FD not enabled | `mtu 16`, frames truncated or rejected | `fd on` omitted, or adapter lacks FD support |
| Missing termination | Intermittent CRC errors, worse under load | No 120 Ω, or three terminators instead of two |
| ID collision | Two joints reporting identical values | Two motors flashed with the same CAN ID |
| **Wrong scaling constants** | **Plausible but physically wrong angles** | **See below — the one that worries me** |

</details>

### 🔴 The part of my own work I cannot vouch for

Damiao motors do not transmit floating-point numbers. They transmit integers, and both ends must
agree on what physical range those integers span. Those limits live in
[`config.py`](openarm_pipeline/config.py) as `MOTOR_SPECS`, and they follow the **public MIT-mode
convention** — I could not check them against a Damiao datasheet.

**If they are wrong, nothing breaks.** The decoder returns smooth, believable numbers in the
wrong units, with no error and no warning. It is the only failure mode on the list that is
completely silent, which is exactly why it is the one I would chase first.

**First test with hardware:** move one joint to a physically measured angle and check the decoder
agrees. Ten minutes, and it either validates the whole read path or invalidates it.

### What I built instead

- The **real Damiao frame format**, tested by round trip — proving my parser self-consistent,
  though not that it matches a real motor
- A `CANSource` interface between the bus and everything downstream, with `MockCANSource` and
  `SocketCANSource` behind it, so hardware bring-up is a one-line change
- [`SocketCANSource`](openarm_pipeline/can/socketcan.py) written in full against the SocketCAN and
  python-can docs, clearly marked untested, with the places I expect to be wrong called out in
  comments

📄 **Full detail:** [`docs/01-can-setup.md`](docs/01-can-setup.md) — physical layer, termination and
grounding, every flag, the complete failure table, and references.

---

## Task 2 — CAN data reading

✅ **Working.** Joint position, velocity and torque for all 14 joints, decoded from real Damiao
MIT-mode frames produced by a simulated arm.

```console
$ python scripts/read_joints.py --duration 3

  ** SIMULATED DATA - NOT A REAL ARM **

  joint                    pos (rad)   vel (rad/s)   torque (Nm)
  --------------------------------------------------------------------
  left_j1_shoulder_pitch     -0.1940        0.0110        2.4396    ok
  left_j2_shoulder_roll      -0.3256       -0.1429        2.2813    ok
  ...
  right_j7_wrist_yaw         -0.0158       -1.5458       -0.1099    ok
  --------------------------------------------------------------------
  frames     41807   dropped    26   bad   0   unknown   0
  snapshot spread  0.32 ms   (time between oldest and newest reading)
```

| Measured over 3 s | |
|---|---|
| Sustained rate | **995.8 Hz** against a 1 kHz target |
| Frames decoded | 41,807 — **0 bad, 0 unknown** |
| Snapshot spread | 0.32 ms |
| Tests | 13 passing |

*(This section will be expanded — architecture and design decisions still to be written up.)*

---

## Tasks 3–5

Not yet started. Sections to follow.

---

## Repository layout

```
openarm_pipeline/
  config.py        rig description: 14 joints, 4 cameras, bus rates
  clock.py         the single timestamp policy every subsystem uses
  can/
    protocol.py    Damiao MIT-mode frame encode/decode
    source.py      the mock-vs-hardware seam
    mock.py        simulated arm
    socketcan.py   real hardware path (untested)
    assembler.py   individual motor frames -> whole-arm snapshots
scripts/
  read_joints.py   Task 2 live reader
docs/
  01-can-setup.md  Task 1, in full
tests/
```

## Licence

MIT
