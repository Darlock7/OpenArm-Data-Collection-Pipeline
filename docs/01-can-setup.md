# Task 1: CAN FD interface setup

**Status: not performed. No hardware.**

I had no OpenArm, no CAN adapter, and no Linux machine. `openarm-can-cli`
and the zero-position tare write to physical motors, so there is nothing I could have
produced here except a fabricated screenshot.

What follows is the procedure I would run, why each part of it is there, how I would
verify it, and what I expect to go wrong. Everything marked **[UNVERIFIED]** is reasoning
from documentation rather than something I have observed.

---

## 1. The physical setup

CAN is a two-wire differential bus: CAN High and CAN Low, twisted together. All nodes
share the same pair; there is no wire per motor. Each frame carries an ID, and each motor
acts only on frames matching its own.

```
robot PC ──[CAN adapter]──┬──────┬──────┬──────┬──────┬──────┬──────┐
                          │      │      │      │      │      │      │
                       motor1  motor2  ...                       motor7   [120 Ω]
   can0  =  left arm
   can1  =  right arm     (same topology, second adapter)
```

Two physical checks before any software, both of which silently ruin a bus if wrong:

1. **Termination.** 120 Ω at each end of the bus, and only at the ends. Missing
   termination gives reflections that look like random CRC errors under load, an
   intermittent fault that is easy to misdiagnose as a software bug.
2. **Common ground** between the adapter and the motor supply. CAN is differential but
   not isolated; without a shared reference the common-mode voltage drifts out of range.

### Why two buses rather than one

The brief specifies two interfaces. My reading of why follows.
**[UNVERIFIED, this is my own estimate, not from the OpenArm docs]**

A CAN FD frame with an 8-byte payload spends roughly 30 bits in the arbitration phase at
1 Mbit/s (~30 µs) and roughly 85 bits in the data phase at 5 Mbit/s (~17 µs), plus
interframe space. Call it **~50 µs per frame**.

One arm at 1 kHz needs a command and a reply for each of 7 motors:

```
7 motors × 2 frames × 1000 Hz            = 14,000 frames/s
14,000 frames/s × 50 µs                  ≈ 0.70 s of bus time per second
                                         ≈ 70 % bus load
```

70 % is past the point where arbitration delay becomes significant and worst-case latency
stops being predictable. The usual design guidance is to stay below 50 to 60 %. Putting
both arms on one bus would mean ~140 %, which is simply impossible.

Split one arm per bus and each sits near **35 %**, with headroom for retransmission.
That is the constraint that forces `can0` and `can1` to exist.

---

## 2. Bringing the interfaces up

SocketCAN presents a CAN adapter as an ordinary Linux network interface, so it is
configured with `ip link`, the same tool used for ethernet.

The OpenArm guide gives this directly:

```bash
# From docs.openarm.dev/software/setup/can-setup/ -- CAN FD at 5 Mbps
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

Then identically for `can1`. A helper script wraps the same thing:

```bash
openarm-can-configure-socketcan can0 -fd
```

I would add two things their guide does **not** include. Marking them as mine rather than
theirs, because that distinction is the whole point of this document:

```bash
sudo ip link set can0 type can \
     bitrate  1000000  sample-point  0.75 \
     dbitrate 5000000  dsample-point 0.75 \
     fd on
sudo ip link set can0 txqueuelen 1000
```

| Flag | Why |
|---|---|
| `bitrate 1000000` | Arbitration phase. Every node must agree, and this phase is where bus arbitration happens, so it stays slow and robust. |
| `dbitrate 5000000` | Data phase, the 5 Mbit/s from the brief. CAN FD's core trick: once a node has won arbitration nobody else is competing, so the payload can be clocked much faster. |
| `fd on` | Enables CAN FD. Without it the controller stays in classic mode: 8-byte payloads, no data-phase switch. |
| `sample-point 0.75` | **My addition, not in the OpenArm guide.** Where in each bit the controller samples. 0.75 is the CiA-recommended value at these rates; it trades noise margin against tolerance for clock mismatch between nodes. |
| `txqueuelen 1000` | **My addition, not in the OpenArm guide.** Kernel-side transmit buffer. The default of 10 is sized for a low-rate bus and will drop outbound frames at 1 kHz across 7 motors. |

Two deliberate omissions:

- **No `restart-ms`.** Automatic bus-off recovery sounds helpful and is wrong for a data
  recorder. A bus-off event means the recording is already compromised; silently
  recovering hides that from the operator. Fail visibly instead.
- **Not done from Python.** Reconfiguring an interface needs `CAP_NET_ADMIN`. A recording
  process should not run privileged, so setup stays a separate, deliberate, root step.
  `socketcan.py::setup_commands()` returns these commands as strings rather than
  executing them.

---

## 3. Verification: what the screenshot would show

```bash
ip -details -statistics link show can0
```

Expected, with the parts that matter marked:

```
2: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 72 qdisc pfifo_fast state UP  ← UP, and mtu 72
    link/can  promiscuity 0 allmulti 0 minmtu 0 maxmtu 0                 = CAN FD
    can <FD> state ERROR-ACTIVE restart-ms 0                         ← FD, ERROR-ACTIVE
      bitrate 1000000 sample-point 0.750
      tq 12 prop-seg 29 phase-seg1 30 phase-seg2 20 sjw 1
      dbitrate 5000000 dsample-point 0.750                           ← data phase live
      clock 80000000
      re-started bus-errors arbit-lost error-warn error-pass bus-off
      0          0          0          0          0          0       ← all zero
```

Four things I would actually be checking, in order:

1. **`state UP`**: the interface is running.
2. **`mtu 72`**: 64 data bytes + 8 header. An mtu of 16 means FD did not take and the
   bus is in classic mode, which will not carry full Damiao frames.
3. **`ERROR-ACTIVE`**: the healthy state. `ERROR-PASSIVE` or `BUS-OFF` means wiring:
   termination, grounding, or a bitrate mismatch.
4. **Error counters at zero.** Non-zero and climbing on an idle bus is a physical-layer
   fault, not a software one.

Then confirm traffic is really flowing:

```bash
candump -td -x can0            # live frames with delta timestamps
canbusload can0@1000000 -r -t  # measured load, to check the ~35 % estimate above
```

`canbusload` is the one that would tell me whether my 70 % calculation was right.

---

## 4. Zero position

### What it is

The Damiao motors are quasi-direct-drive, so the arm is backdrivable and gets moved by
hand during teleoperation. Each motor's encoder reports an angle against an internal reference that has no
relationship to the arm's geometry. Zeroing declares the **current physical pose** to be
0 rad on every joint.

It is the same operation as taring a scale, and it matters for the same reason: without
it, "joint 4 at 0.5 rad" means something different every session, and a policy trained on
one session's data is being fed a different coordinate frame at inference time.

### Procedure

```bash
openarm-can-cli -i can0 can_configure
openarm-can-cli -i can1 can_configure

# Classic CAN at 1 Mbps instead, if needed:
openarm-can-cli -i can0 can_configure -d 1000000 --no-fd
```

Note the interface flag is `-i` and it precedes the subcommand.

Physically, the arm does not get posed by hand for this. It goes into a **purpose-built
calibration jig**. Per the OpenArm calibration workflow:

1. Power the motors but leave them disabled, so the arm is free to move.
2. Press the arm pipe into the recess of the jig, seat the trigger's flat surface against
   the jig's flat surface, press the body against the jig and secure it firmly, then fasten
   with two 12 mm M2 screws. Mirror for the other arm.
3. Support the arms throughout. A QDD arm has almost no gearbox friction and will fall
   under its own weight the instant it is released.
4. Zero, then enable and confirm every joint reads ~0.

The jig is the whole point: it makes the reference pose repeatable **to a machined surface**
rather than to someone's judgement, which is what lets two sessions share a coordinate
frame. OpenArm's setup guide budgets roughly 45 minutes for calibration and homing.

### What it does on the wire

Underneath the CLI, zeroing is one reserved 8-byte payload sent to each motor ID:

```
FF FF FF FF FF FF FF FE     set zero position
FF FF FF FF FF FF FF FC     enable
FF FF FF FF FF FF FF FD     disable
FF FF FF FF FF FF FF FB     clear error
```

These are implemented in `openarm_pipeline/can/protocol.py` as `CMD_SET_ZERO` and
friends, and `tests/test_protocol.py` asserts the byte sequence. Manually, the equivalent
of zeroing motor 1 on `can0` is:

```bash
cansend can0 001##1ffffffffffffffe
```

### Verifying it took

Zero, then read back. Every joint should report within a few milliradians of zero:

```bash
python -m openarm_pipeline.scripts.read_joints --interface can0 --once
```

Then move one joint by hand and confirm the sign and magnitude match the physical
direction. A joint that reads zero but counts the wrong way is a sign convention error,
and it will not show up until a trained policy drives that joint backwards.

---

## 5. What I expect to go wrong

Ranked by how likely I think they are:

| Failure | Symptom | Cause |
|---|---|---|
| Bitrate mismatch | `ERROR-PASSIVE`, error counters climbing, no valid frames | Motors configured for a different data-phase rate than the interface |
| FD not enabled | `mtu 16`, frames truncated or rejected | `fd on` omitted, or the adapter does not support FD |
| Missing termination | Intermittent CRC errors that worsen with load | No 120 Ω, or three terminators instead of two |
| ID collision | Two joints reporting identical values | Two motors flashed with the same CAN ID |
| Wrong scaling constants | Plausible but physically wrong angles | `MOTOR_SPECS` in `config.py` is **UNVERIFIED**, see below |

The last one is the one that worries me, because it is the only one that produces no
error at all. The position/velocity/torque limits in `config.py` follow the public MIT
mode convention, but I could not check them against a Damiao datasheet. If they are wrong
the decoder still returns smooth, believable numbers in the wrong units. **First thing I
would do with hardware: move one joint to a known angle, measure it physically, and check
the decoder agrees.**

---

## 6. What I did instead

Since none of the above could be executed, `openarm_pipeline/can/`:

- implements the real Damiao frame format, and tests it by round trip
  (`tests/test_protocol.py`), which proves my parser self-consistent, though not that it
  matches a real motor;
- puts a `CANSource` interface between the bus and everything downstream, with
  `MockCANSource` and `SocketCANSource` behind it, so hardware bring-up changes one line;
- writes `SocketCANSource` in full against the SocketCAN and python-can documentation,
  clearly marked untested, with the places I expect to be wrong called out in comments.

## References

- [OpenArm CAN setup guide](https://docs.openarm.dev/software/setup/can-setup/) -- source for the
  `ip link` and `openarm-can-cli` commands above
- [OpenArm zero position calibration workflow](https://docs.openarm.dev/hardware/openarm-ker/calibration-workflow)
- [enactic/openarm_can](https://github.com/enactic/openarm_can) -- the official CAN control library
- [OpenArm documentation](https://docs.openarm.dev)
- [SVRC OpenArm 101 setup guide](https://www.roboticscenter.ai/en/hardware/openarm/setup)
- [SVRC Damiao motor reference](https://www.roboticscenter.ai/wiki/damiao-motors)
- Linux kernel SocketCAN documentation, `Documentation/networking/can.rst`
