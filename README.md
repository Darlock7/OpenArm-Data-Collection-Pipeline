<div align="center">

# OpenArm 2.0-Data Collection Pipeline

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
| 3 | **Multi-camera synchronisation** | ✅ **Working** | 5 streams aligned, timing error measured per sample |
| 4 | **Storage backend + REST API** | ✅ **Working** | MCAP capture, HDF5 export, 8 endpoints |
| 5 | **Monitoring dashboard** | ✅ **Working** | Live joints, 4 camera previews, Start/Stop |

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

./.venv/bin/python -m pytest tests/ -q          # 44 tests
./.venv/bin/python scripts/read_joints.py       # live joint state, simulated arm
./.venv/bin/python scripts/demo_sync.py         # 5 streams captured and aligned
./.venv/bin/uvicorn openarm_pipeline.api.server:app   # API + dashboard on :8000
```

---

## Architecture

Five tasks, one pipeline. Data flows left to right and each stage consumes the previous one.

```
   ARM                    CAPTURE                 STORE              SERVE
   ───                    ───────                 ─────              ─────
   14 motors  ─can0/1─▶  CANSource ──┐
                         assembler   │
                                     ├──▶ bounded ──▶ MCAP ──┐
   4 cameras  ────────▶  CameraSource┘     queue      raw    │
                                                             │
                                              align() ◀──────┤
                                                 │           │
                                                 ▼           ▼
                                               HDF5      REST API ──▶ dashboard
                                            training                   live view
```

Three principles recur at every layer, and they are the actual design:

**1. One seam per data source.** `CANSource` and `CameraSource` each have a simulated and a
hardware implementation. Nothing downstream can tell which is running. Without this, none of
tasks 3 to 5 could have been built at all; with it, hardware bring-up is a one-line change and
the simulator remains the permanent CI fixture, since a robot arm cannot be plugged into a test
runner.

**2. Record raw, align at read time.** Nothing is resampled during capture. Alignment is a query
over untouched streams, so the policy is chosen at export and one recording yields several
training sets. Resampling during capture would bake in one consumer's needs and destroy the
others irreversibly.

**3. Measure your own error and publish it.** Snapshot spread, per-sample timing error, bias,
jitter, dropped frames, queue back-pressure. Where a true value is unavailable, the pipeline
reports a real one plus its uncertainty rather than a synthetic one that hides it. That is what
makes alignment quality a filterable property of the dataset instead of an assumption.

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
# docs.openarm.dev/software/setup/can-setup/
sudo ip link set can0 down                                               # must be down to change parameters
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on    # 1 Mbit arbitration, 5 Mbit data, FD on
sudo ip link set can0 up

openarm-can-cli -i can0 can_configure                                    # then the zero-position tare
```

Then identically for `can1`. Note `-i` precedes the subcommand.

On top of the documented command I would add `sample-point 0.75`, `dsample-point 0.75` and
`txqueuelen 1000`. **Those three are mine, not OpenArm's** — reasoning in the fold below.

<details>
<summary><b>Why each flag is there, and two deliberate omissions</b></summary>

<br>

| Flag | Reason |
|---|---|
| `bitrate 1000000` | Arbitration phase. Every node must agree on it, and it is where bus arbitration happens, so it stays slow and robust. |
| `dbitrate 5000000` | Data phase. CAN FD's core trick: once a node has won arbitration nobody is competing, so the payload can clock much faster. |
| `fd on` | Enables CAN FD. Without it the controller stays in classic mode — 8-byte payloads, no data-phase switch. |
| `sample-point 0.75` | **Mine, not in the OpenArm guide.** Where in each bit the controller samples. The CiA-recommended value at these rates; trades noise margin against tolerance for clock mismatch between nodes. |
| `txqueuelen 1000` | **Mine, not in the OpenArm guide.** Kernel transmit buffer. The default of 10 is sized for a low-rate bus and will drop outbound frames at 1 kHz across 7 motors. |

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

The arm is not posed by hand for this. It goes into a **purpose-built calibration jig** — press
the arm pipe into the jig recess, seat the trigger flat against the jig face, clamp the body,
fasten with two 12 mm M2 screws, then mirror on the other side. That is the point of the jig:
the reference pose is repeatable to a machined surface rather than to someone's judgement.
OpenArm budgets roughly 45 minutes for calibration and homing.

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

**Sources:** [CAN setup guide](https://docs.openarm.dev/software/setup/can-setup/) ·
[calibration workflow](https://docs.openarm.dev/hardware/openarm-ker/calibration-workflow) ·
[enactic/openarm_can](https://github.com/enactic/openarm_can)

📄 **Full detail:** [`docs/01-can-setup.md`](docs/01-can-setup.md) — physical layer, termination and
grounding, every flag, the complete failure table, and references.

---

## Task 2 - CAN data reading

> *"Read live joint position, velocity, and torque from the arm over CAN FD. If you don't have
> hardware access, mock the CAN data stream in software and note that clearly."*

✅ **Working.** All 14 joints, decoded from genuine Damiao MIT-mode frames produced by a
simulated arm, sustained at **995.8 Hz** against the 1 kHz target.

### The decision that shapes everything: where to fake

With no arm, you are writing a simulation. The question is *at which layer*, and it is the only
choice here that is hard to reverse.

```
  OPTION A  (rejected)                 OPTION B  (chosen)

  fake ──▶ position = 0.34             fake ──▶ 0x04 0x90 0x15 0x88 ...
           velocity = 1.20                            │
           torque   = 2.44                            ▼
                                              REAL decoder runs
                                                      │
                                                      ▼
                                              position = 0.34
```

**Option A produces a demo that proves nothing.** The decoder, which is the actual hard part,
never executes. You would have tested your imagination.

**Option B means every frame in the simulation is real Damiao format, parsed by the real
parser.** The bit-unpacking, the scaling, the error handling and the frame assembler are all
exercised on every run. Attach hardware and none of that code path changes.

The cost is that the mock has to know the wire format, which is more work. Worth it: it is the
difference between a stub and a test fixture the pipeline can be regression-tested against
permanently, since CI cannot plug in a robot arm.

### What is actually in a frame

Eight bytes carry position, velocity, torque, two temperatures, the motor ID and an error code,
a thousand times a second. There is no room for anything readable.

```
 byte   0      1      2      3      4      5      6      7
      ┌─────┬─────────────┬─────┬─────┬─────┬─────┬─────┐
      │ ID  │  POSITION   │ VEL │ V|T │ TRQ │Tmos │Trot │
      │+err │   16 bit    │     │     │     │     │     │
      └─────┴─────────────┴─────┴─────┴─────┴─────┴─────┘
                                  ▲
                     byte 4 is SHARED: high nibble finishes
                     velocity, low nibble starts torque
```

That shared byte is where a hand-written parser goes wrong, so
[`tests/test_protocol.py`](tests/test_protocol.py) attacks it directly: drive velocity to its
maximum while holding torque at its minimum, then reverse the pairing. A swapped nibble cannot
pass both.

### The values are integers, and that has consequences

A motor does not transmit `0.34 radians`. It transmits an integer, and both ends must already
agree what range it spans. Two consequences:

**Resolution differs per field.** Position gets 16 bits, velocity and torque only 12:

```console
sent:     pos 1.5708 rad    vel 2.0 rad/s     torque 1.25 Nm
decoded:  pos 1.5707 rad    vel 2.0000 rad/s  torque 1.2479 Nm
          └─ 16 bit, ~0.0004 rad ─┘           └─ 12 bit, ~0.005 Nm ─┘
```

Torque resolves to roughly 4,000 steps across its full range, not 65,000. Worth knowing before
trusting a torque figure to three decimals.

**And the scaling constants are the silent failure mode** described in Task 1: wrong limits give
smooth, believable, incorrect values with no error at all.

### There is no such thing as "the arm right now"

The most interesting thing I learned building this. Downstream code wants a snapshot: 14
positions at one instant. **The bus cannot provide that.** Each motor answers separately,
microseconds apart, so a snapshot is a *reconstruction*, and every reconstruction makes choices.

| Question | Options | Chosen | Why |
|---|---|---|---|
| A joint missed this cycle | stall / carry forward | **carry forward, mark `valid=False`** | one dead motor should degrade a recording, not end it |
| Readings span real time | present as instantaneous / measure it | **measure it, publish `spread_s`** | the dataset should carry evidence of its own precision |
| Which timestamp to use | average them / newest reading | **newest** | an average is a number no sensor ever produced |

That last row is a small decision with a large principle behind it: **when you cannot have the
true value, report a real one plus its error, never a synthetic one that hides it.**

Measured spread is **0.32 ms**. For scale, a joint moving at 2 rad/s travels 0.00064 rad in that
window, which is about 1.7 least-significant bits of position resolution. Comparable to
quantisation noise, so interpolating within a snapshot would be false precision.

<details>
<summary><b>Why interpolate at all, and where it stops being valid</b></summary>

<br>

The same question returns much larger in Task 3, where camera frames sit tens of milliseconds
apart rather than fractions of one. The answer splits by signal type:

| | Interpolate? | Reason |
|---|:--:|---|
| Joint position | ✅ | Continuous physical signal. A joint cannot teleport. Better still, the motor reports velocity too, so cubic Hermite interpolation is available for free and beats linear |
| Joint torque | ⚠️ | Not smooth. Contact events are near discontinuous, and interpolating across one invents a gentle ramp where reality had a step |
| Camera frames | ❌ | Blending two images produces a ghost of something that never existed. Training a vision policy on manufactured frames teaches it artifacts the real camera never produces |

Scale of the problem, joint at 2 rad/s:

```
  snapshot spread   0.32 ms  ->  0.037 deg of arm movement   negligible
  ceiling camera    66.7 ms  ->  7.6 deg of arm movement     enormous
```

Which drives the Task 3 architecture: **do not resample at record time.** At record time you do
not yet know what the training code will want, and discarding raw samples makes that choice
permanent. Store native rates with honest timestamps; align at read time under a declared
policy; mark anything interpolated as interpolated.

</details>

### Proving it works with no arm to compare against

What cannot be proven without hardware: that the decoder matches a real Damiao motor. That
requires moving a joint to a measured angle and checking agreement.

What can be proven, and is: the decoder is self-consistent, handles the split byte correctly,
saturates instead of wrapping on overflow, and rejects malformed frames rather than guessing.
**13 tests, named so the intent reads without opening them.**

```console
$ python -m pytest tests/ -q
.............                                                    [100%]
13 passed in 0.01s
```

### Results

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
| Frames decoded | 41,807, with **0 bad and 0 unknown** |
| Frames dropped | 26 (0.06%), injected deliberately |
| Snapshot spread | 0.32 ms |

<details>
<summary><b>What the simulated arm does and does not model</b></summary>

<br>

**Faithful:**

- Frames are real Damiao MIT-mode bytes, produced by `encode_feedback` and parsed by the same
  decoder the hardware path uses
- Motors are polled round robin, one frame each, as on a real bus. A snapshot is never atomic
- 0.1% of frames are dropped and timing jitter is injected, so the `valid` mask and the spread
  measurement are actually exercised rather than sitting untested
- Seeded, so runs are reproducible

**Not faithful:**

- Motion is two sine waves per joint, chosen because it is smooth and analytically
  differentiable, so velocity is exact rather than estimated. It is not a real teleoperation
  demonstration
- Torque is a gravity plus inertia stand-in, not this arm's dynamics
- No bus arbitration, electrical faults, or controller latency

Every episode recorded from this source is tagged `is_mock=True` in metadata and in its
filename, and the reader prints a banner. Synthetic data must never be mistakable for a real
demonstration six months later.

</details>

---

## Task 3 - multi-camera synchronisation

> *"Capture (or simulate) synchronized frames from all 4 cameras. Describe how you handle frame
> timestamps and alignment with joint state data. How do you deal with cameras running at
> different frame rates?"*

✅ **Working.** Four simulated cameras at 60/30/30/15 fps captured concurrently with the 1 kHz
joint stream, then aligned on a common timeline with the timing error recorded per sample.

### The problem, drawn

```
  joints 1kHz  |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
  wrist_left           o           o            o           o
  wrist_right            o            o          o            o
  ceiling             o                       o
  zed_head            o      o     o    o      o     o    o      o
               0 ms                                               200 ms
```

Five streams, none ticking together, none sharing a clock. A training sample needs an image and
a joint state describing **the same moment**, and no such pair exists in the raw data. Worse,
the ceiling camera is blind for 66 ms at a stretch, during which a joint moving at 2 rad/s
travels **7.6 degrees**.

### The rule everything follows

> **Do not resample at record time. Store raw, align at read time.**

At capture time you do not yet know what the training code will want. One method wants a sample
per ZED frame with interpolated joints; another wants raw 1 kHz torque for contact detection; a
third wants a fixed 30 Hz grid. Resampling during capture bakes one of those in and destroys the
others **irreversibly**.

So `sync.py` mutates nothing. Alignment is a *query*: hand it a timeline, get a view. Ask again
with a different policy and you get a different view of the same untouched data. A test asserts
this directly.

### Joints interpolate. Pixels never do.

| Signal | Interpolate? | Why |
|---|:--:|---|
| Joint **position** | ✅ Cubic Hermite | Continuous physical signal, a joint cannot teleport. The motor already reports **velocity**, which is the derivative, so the curve can leave each sample at the slope actually measured there. Strictly better than linear and free |
| Joint **torque** | ⚠️ Linear, with suspicion | Near discontinuous at contact, and contact is usually the most important moment in a demonstration. Interpolating across one invents a ramp where reality had a step |
| **Camera frames** | ❌ Never | Blending two images makes a translucent ghost of a scene that never existed. A vision policy trained on manufactured frames learns to expect artifacts the real camera will never produce |

Cameras get **nearest frame only**, and the staleness is recorded rather than hidden.

### A camera frame is an interval, not an instant

A CAN reading is sampled at a moment. An image **integrates light across its whole exposure**, so
1/60 s of world is smeared into one array. "When was this frame taken" has no exact answer.

Frames are stamped at **mid-exposure**, the centre of mass of the light that formed the image.
Stamping at start of exposure biases every frame early by half the exposure; stamping at readout
biases late by an exposure plus transfer. Either produces a systematic image-to-joint offset,
which is the single most damaging error in this pipeline. `Frame.exposure_s` travels alongside
so a consumer can see how wide the interval was instead of assuming.

### What is actually being defended against

Ranked by how badly each damages a trained policy:

| | Failure | Why it ranks there |
|:--:|---|---|
| **1** | **Bias** - a systematic image-to-joint offset | The policy learns to act on stale observations, then lags and overshoots on hardware. Bias does not average out with more data. It is *learned* |
| **2** | **Jitter** - an offset that varies | Worse per millisecond than bias. A constant offset can be partly absorbed into the learned dynamics; a wandering one cannot |
| **3** | **Aliasing** - decimating 1 kHz data without filtering | Content above the new Nyquist folds down and stops looking like noise, starts looking like real motion that never happened |

Every aligned sample therefore carries its **signed** timing error, `dt = t_sample - t_query`.
Signed, not absolute, because the mean of the signed error *is* the bias. Take absolute values
first and a 50 ms systematic offset averages to +50 and becomes indistinguishable from a 50 ms
lead. That is exactly how a fatal misalignment ships undetected.

`window_mean` exists for failure 3: a boxcar average is a crude low-pass filter, and crude and
applied beats ideal and skipped.

### Anchoring on a camera, not a grid

The timeline that training samples sit on is **the primary camera's real frame timestamps**.

In imitation learning the observation *is* an image, and at inference the policy runs when a
frame arrives. Anchoring on real frame times means every training sample corresponds to a moment
the robot will actually experience, and the anchor camera needs no interpolation at all. **Its
error is identically zero by construction.**

A fixed grid would buy independence from any one sensor at the cost of making every sample in
every stream synthetic. `align()` accepts any timeline, so that remains available; it is simply
not the default.

### Results

```console
$ python scripts/demo_sync.py --duration 5

  stream          samples   nominal   measured  dropped     drift
  ---------------------------------------------------------------
  joints             4968   1000 Hz      993.0
  wrist_left          150       30 f      29.79        1      -61p
  wrist_right         150       30 f      29.80        1      -18p
  ceiling              76       15 f      15.00        0      -90p
  zed_head            300       60 f      59.99        0      117p

  alignment  anchor=zed_head  policy=hermite  300 samples
  stream              bias    jitter     worst   usable
  -----------------------------------------------------
  joints           -0.001m    0.356m    1.990m    99.7%
  wrist_left       -5.414m    8.509m   16.702m    99.3%
  wrist_right      -0.443m    8.489m   11.624m    99.3%
  ceiling           3.385m   19.001m   33.213m    96.7%
  zed_head          0.000m    0.000m    0.000m   100.0%   <- anchor
```

Three things worth reading off that table:

- **The anchor is exactly 0.000**, not approximately. Every sample sits on a real frame.
- **The ceiling camera's worst case is 33.2 ms**, which is precisely half a frame at 15 fps. Not
  a bug, a floor. It is reported rather than hidden.
- **Bias and jitter are separate columns.** `wrist_left` carries a systematic -5.4 ms offset
  *and* 8.5 ms of jitter. Different problems, different fixes, and a single "mean absolute
  error" column would have concealed both.

A slow camera is also not punished for being slow: the acceptance tolerance scales with each
camera's own measured rate. Holding a 15 fps ceiling camera to the same millisecond budget as a
60 fps ZED would reject nearly all of its frames for doing nothing wrong.

### Scoring the policies against ground truth

The simulated arm is an analytic function of time, so its **true** position at any instant is
known. That turns "hermite should be better" into a measurement:

```console
  policy          synthetic     mean err    worst err
  ---------------------------------------------------
  nearest                no      0.6039m      4.6917m
  hermite               yes      0.5509m      3.9041m   <- best
  window_mean           yes      0.5878m     10.2253m
```

Millirad against truth. Hermite wins on both mean and worst case. `window_mean` has the worst
peak error because smoothing clips extremes, which is the trade it exists to make.

**The margin is only 9%, and that deserves an explanation rather than a victory lap.** At 1 kHz
the joints are roughly 50x oversampled relative to human motion, so the residual is dominated by
**timestamp jitter**, not interpolation error, and no method recovers what the clock got wrong.
The choice matters far more at lower rates: `tests/test_sync.py` shows hermite beating linear by
**5x** on a signal sampled every 20 ms.

<details>
<summary><b>What the simulated cameras model, and why each imperfection is there</b></summary>

<br>

A mock producing perfectly regular frames would make the synchroniser look flawless and prove
nothing. Each modelled defect breaks a different naive assumption:

| Modelled | Breaks the assumption that |
|---|---|
| **Jitter**, ~1.5 ms | frame N happened at N/fps |
| **Clock drift**, randomised per camera to ±120 ppm | the offset measured at startup stays valid. At 30 fps over 60 s, 120 ppm is nearly two full frames of accumulated error |
| **Dropped frames**, 0.2% | frame index is time. This is why `Frame` carries a sequence number: a gap in `seq` is visible, a gap in a Python list is not |
| **Startup skew**, 0 to 40 ms | stream 0 sample 0 lines up with stream 1 sample 0 |

Together these are the reason alignment must be timestamp-based rather than index-based, which
is the whole of Task 3.

**Not modelled:** rolling shutter, real image content, compression, USB bandwidth contention. The
frames carry a synthetic pattern with a marker that sweeps once per second, so cross-camera
alignment is checkable by eye. Images render at a fraction of the declared sensor resolution to
keep the demo cheap; the declared resolution is preserved in metadata. That is a simulation
shortcut, not a claim about the real cameras.

</details>

---

## Task 4 - storage backend and REST API

> *"Store episodes (joint states + camera frames) in a structured format. Choose a storage format
> and justify your choice (e.g. HDF5, MCAP, zarr, custom). Include a simple REST API to list
> episodes, retrieve metadata, and download an episode."*

✅ **Working.** MCAP for capture, HDF5 for training, eight REST endpoints.

### The question contains an assumption worth rejecting

"Choose a storage format" assumes one format should serve both writing and reading. Those jobs
want opposite things:

| | While **recording** | While **training** |
|---|---|---|
| Access pattern | Append, one sample at a time | Random access to shuffled timesteps |
| Length | Unknown until you stop | Fixed and known |
| Rates | Five streams, all different, none resampled | One common timeline |
| Biggest risk | Power dies mid-episode | Dataloader starves the GPU |
| Therefore wants | Append-only, crash tolerant | Chunked, compressed, indexed |

Scored against that split:

| | MCAP | HDF5 | Zarr | LeRobot/RLDS |
|---|:--:|:--:|:--:|:--:|
| Append during capture | ✅ built for it | ❌ writer lock | 🟡 | ❌ export target |
| Survives a crash mid-write | ✅ truncated file still reads | ❌ can lose the file | 🟡 per chunk | n/a |
| Native heterogeneous rates | ✅ per-message timestamps | 🟡 parallel datasets | 🟡 | 🟡 fixed steps |
| Random access for training | ❌ it is a log | ✅ | ✅ | ✅ |
| Ecosystem | ROS 2, Foxglove | ALOHA/ACT | Diffusion Policy | **DeepAware's own pipeline** |

### So: record to MCAP, export to HDF5

```
   capture ──▶ MCAP           raw, native rates, nothing resampled
                 │
                 │  ◀── align() runs HERE
                 ▼
              HDF5            one timeline, dataloader ready
```

**This is Task 3's rule made physical.** "Store raw, align at read time" becomes "the alignment
policy is an argument to the export", which means one recording produces several training sets:

```bash
POST /api/episodes/{id}/export?policy=hermite       # interpolated joints
POST /api/episodes/{id}/export?policy=nearest       # only real measurements
POST /api/episodes/{id}/export?policy=window_mean   # anti-aliased
```

A test asserts the MCAP is **byte-identical** after all three. Disagree with a choice a year from
now and you re-export rather than re-record.

It also places RLDS/LeRobot correctly. It is the **export target**, not the recording format:
matching what the consuming pipeline speaks, without crippling capture to do it.

<details>
<summary><b>Why MCAP specifically, and what it costs</b></summary>

<br>

Recording is the one moment in this pipeline where data is irreplaceable. A training export can
be regenerated; a demonstration cannot be re-performed. So the write format is chosen for
survival first:

- **A file truncated by a crash is still readable** up to the last complete message. HDF5 can
  lose the entire file to one bad write.
- **Per-message timestamps on independent channels**, so five streams at five rates need no
  resampling, no padding, and no common clock at write time. The format stops fighting the Task 3
  design and starts enabling it.
- **ROS 2 and Foxglove standard**, so recordings open in existing tooling.

Costs, taken deliberately: joint states are stored as JSON with a registered schema, roughly 1 KB
per message at 1 kHz. Wasteful, and worth it on a research rig where opening a file and reading
it beats saving a megabyte a second. Images bypass JSON entirely, since base64 would inflate
pixels by a third for nothing.

</details>

### The recorder never lets a disk touch the capture thread

A 1 kHz loop has **1 ms per cycle**. An fsync, a log rotation or a GC pause can exceed that by an
order of magnitude. If the capture thread writes, a disk hiccup does not slow the recording down,
it puts a **hole** in it.

So capture threads only enqueue and one writer thread drains. The queue is **bounded** on purpose:

```
  unbounded  ->  a slow disk becomes unbounded memory growth. The process dies
                 later, harder, and further from the cause.
  bounded    ->  back-pressure is visible immediately, and the drop policy is an
                 explicit decision rather than an accident.
```

When full it drops the newest sample and **counts it**, exposed as `dropped_queue` in the API.
Blocking would stall capture, which is the thing being protected. A recording with 12 counted
gaps is usable and honest; 12 uncounted gaps is neither.

*On Python: this is acceptable only because every thread is I/O bound and releases the GIL. A real
1 kHz control loop belongs in C++. This process observes the bus rather than closing a loop over
it, so the bar is much lower. Stated in `recorder.py` rather than left to be discovered.*

### /timing is the group most pipelines throw away

Every exported episode carries its per-sample timing error, plus bias and jitter in the file
attributes:

```
/observations/joint_position   (N, 14)
/observations/images/<camera>  (N, H, W, 3)   chunked one frame per chunk
/timing/joint_dt_s             (N,)           distance to nearest real sample
/timing/camera_dt_s/<camera>   (N,)           signed staleness
attrs: align_policy, align_anchor, bias_ms/<cam>, jitter_ms/<cam>, is_mock
```

Which makes **alignment quality a filterable property of the dataset**: a training run can reject
episodes whose bias exceeds a threshold. That is the payoff for measuring it in Task 3 rather than
assuming it. Images are chunked one frame per chunk because a shuffling dataloader reads random
timesteps, so chunking across time would force decompressing a whole block to reach one frame.

### The API

| Method | Path | Purpose |
|---|---|---|
| `GET` | `/api/episodes` | List, newest first. Reads sidecars only, never recordings |
| `GET` | `/api/episodes/{id}` | Full metadata plus existing exports |
| `GET` | `/api/episodes/{id}/download` | Raw MCAP, or `?fmt=h5&policy=...` for an aligned view |
| `POST` | `/api/episodes/{id}/export` | Generate an aligned HDF5 under a chosen policy |
| `POST` | `/api/record/start` | Begin an episode. **409** if one is running |
| `POST` | `/api/record/stop` | Finalise. **409** if none is running |
| `GET` | `/api/status` | Queue depth, sample counts, back-pressure drops |
| `GET` | `/api/live` | Latest joint snapshot, whether or not recording |

Two deliberate choices. **Capture runs continuously from process start; recording is a separate
flag**, so the dashboard shows live state before anyone presses Start, and Start cannot miss the
first samples. And **conflicts return 409 rather than silently succeeding** — a UI convinced it
started a second recording when it did not is worse than an error.

Listing reads the sidecar JSON, never the recording. `GET /episodes` across a thousand episodes
must not mean opening a thousand multi-gigabyte files.

### Measured

```console
episode      20260810T030208_050   mock=True
duration     3.00 s
joints       3000 @ 998.0 Hz
frames       {'wrist_left': 90, 'wrist_right': 90, 'ceiling': 44, 'zed_head': 180}
dropped      {'ceiling': 1}          <- from sequence gaps, not arrival counts
queue drops  0

export hermite  -> mock_...hermite.h5   180 samples  synthetic=True
export nearest  -> mock_...nearest.h5   180 samples  synthetic=False
```

### Two things running it revealed

**The export duplicates frames.** That 3 s episode holds 404 unique frames and the export writes
**720 image slots**, because the 15 fps ceiling camera is repeated roughly 4x to fill a 60 fps
anchor timeline. Inherent to anchoring, and precisely why LeRobot stores video separately and
indexes into it rather than materialising every frame at every timestep. With more time that is
the first thing I would change.

**Raw image storage does not scale.** Four cameras at full resolution is roughly **150 MB/s**,
which is not something you keep. Fine for 80x60 synthetic frames, wrong for production, where
this needs h264/AV1 with a frame-index-to-timestamp map. Both gaps are written into
`storage/base.py` rather than left to be discovered.

---

## Task 5 - monitoring dashboard

> *"A simple web dashboard showing live (or simulated) joint states, a camera feed preview,
> episode count, and a Start / Stop recording button. Any frontend framework is fine."*

✅ **Working.** Served at `/` by the same process as the API.

<img src="docs/dashboard.png" alt="Dashboard: live joint states, four camera previews, episode list, recording controls" width="100%">

<div align="center"><sub>Mid-recording. Live joint states at 10 Hz, four camera previews at their real
relative rates, one completed episode with download links.</sub></div>

```bash
./.venv/bin/uvicorn openarm_pipeline.api.server:app
#  http://127.0.0.1:8000        dashboard
#  http://127.0.0.1:8000/docs   generated OpenAPI docs
```

| Panel | Shows |
|---|---|
| **Control bar** | Start/Stop, live joint rate, snapshot spread, sample count, **queue depth**, back-pressure drops, episode count |
| **Joint states** | All 14 joints at 10 Hz with signed bars, split left arm from right. A joint whose `valid` flag is false turns red |
| **Camera preview** | Four live canvases at their real relative rates |
| **Episodes** | Newest first, with download links for the raw MCAP and for aligned HDF5 under either policy |

### Three deliberate choices

**No build step, no framework.** One HTML file with inline CSS and vanilla JS. Node is not
installed on my machine and adding a toolchain to serve one page would be a poor trade. It also
means the dashboard cannot rot: no lockfile, no dependency to update, and it runs from a clean
checkout with nothing but `pip install -r requirements.txt`.

**Camera frames are raw RGB decoded straight into a canvas.** The endpoint returns a 12-byte
header (height, width, channels) followed by pixels, and the browser builds an `ImageData` from
it. No image encoder on the server, no decoder in the client, and it reuses the exact header
format the storage layer already uses, so there is one image convention in the project rather
than two.

**Queue depth and drops are on screen, not buried in logs.** They are the numbers that tell an
operator the recording is in trouble *while it is still happening*, which is the only time the
information is worth anything. A dashboard that shows only what is going well is decoration.

The simulated-data banner is driven by the API's `is_mock` flag rather than hardcoded, so it
disappears by itself the day this runs against a real arm.

### Verified over HTTP

```console
GET  /                 200   11471 bytes
GET  /docs             200
GET  /api/live         200
GET  /api/cameras/zed_head/frame   200

POST /api/record/start  {"started":true,"episode_id":"20260810T031613_812","is_mock":true}
POST /api/record/stop   {"stopped":true,"duration_s":3.012,"joint_samples":3009,
                         "n_frames":{"wrist_left":89,"wrist_right":90,
                                     "ceiling":45,"zed_head":181}}
```

Exactly the calls the buttons make.

---

## What I would do next

### First hour with hardware, in order

1. **Verify `MOTOR_SPECS`.** Move one joint to a physically measured angle and check the decoder
   agrees. This is first because it is the only failure on the list that is completely silent:
   wrong scaling limits produce smooth, believable, incorrect values with no error. Ten minutes,
   and it either validates the whole read path or invalidates it.
2. **Run `canbusload`** and check the ~70 % per-arm estimate that justifies two buses. It is my
   arithmetic, not OpenArm's, and it is measurable in one command.
3. **Switch to kernel timestamping.** Frames are currently stamped when Python receives them,
   which includes driver and scheduling latency. `SO_TIMESTAMPING` stamps on arrival instead.
   The interesting part is not the fix but the measurement: the difference between the two is
   exactly the bias this pipeline is built to eliminate, and I would like to know its size.
4. **Execute `SocketCANSource` for the first time.** I expect its round-robin `recv(timeout=)`
   loop to be too slow for two buses at 1 kHz and to need `can.Notifier` or `select()` instead.
   That is flagged in the source rather than guessed at, because I could not benchmark it.

### With more time, in priority order

| | Why |
|---|---|
| **Video encoding** instead of raw frames | Fixes the two biggest storage problems at once: 150 MB/s of raw pixels, and the frame duplication the export currently produces. h264/AV1 plus a frame-index-to-timestamp map, which is what LeRobot does with mp4 and parquet |
| **Direct LeRobot/RLDS export** | The HDF5 layout is already close. Matching the format DeepAware's pipeline actually consumes removes a conversion step from the training side |
| **Automatic quality gating** | Every episode already carries its bias and jitter. Rejecting recordings above a threshold at ingest is a small addition on top of data that is already there |
| **A more faithful camera mock** | Exposure-time motion blur and rolling shutter. Right now timestamp jitter is uncorrelated with image content, which flatters the synchroniser |
| **Multi-machine clock sync** | Everything here assumes one host. Two machines need PTP, and the alignment code would need a clock-domain concept it currently does not have |

### Known limitations, stated plainly

- **Nothing has touched hardware.** Every number in this README came from a simulation.
- **`MOTOR_SPECS` is unverified** and fails silently if wrong.
- **Raw image storage does not scale**; see above.
- **The export duplicates frames** to fill the anchor timeline, roughly 1.8x on a typical episode.
- **Python is acceptable here only because every thread is I/O bound.** This process observes the
  bus; it does not close a loop over it. A real 1 kHz controller belongs in C++.
- **`SocketCANSource` has never been executed.** It is written against documentation, and the
  places I expect to be wrong are marked in the source.

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
  cameras/
    source.py      Frame, CameraStream, JointStream; mid-exposure timestamps
    mock.py        four cameras with drift, jitter, drops and startup skew
    sync.py        alignment: nearest / hermite / window_mean, error per sample
  storage/
    base.py        the write-path vs read-path argument; EpisodeWriter interface
    mcap_backend.py  append-only capture format, plus reader
    export.py      MCAP -> HDF5; where align() runs
    registry.py    episode listing from sidecars, never opening recordings
  api/
    server.py      REST API
    static/        the dashboard, one file, no build step
  recorder.py      capture threads, bounded queue, episode lifecycle
scripts/
  read_joints.py   Task 2 live reader
  demo_sync.py     Task 3 five-stream capture and alignment report
docs/
  01-can-setup.md  Task 1, in full
  dashboard.png    Task 5 screenshot
tests/             44 tests
```

## Licence

MIT
