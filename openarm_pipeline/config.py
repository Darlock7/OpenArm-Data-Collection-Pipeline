"""Physical description of the OpenArm 2.0 rig.

Everything here is a statement about the hardware, not about our software.
Keeping it in one file means the rest of the codebase never hardcodes a
joint count or a frame rate.

SOURCES / CONFIDENCE: read this before trusting a number.
  * 7 DOF per arm, bimanual (14 total), CAN-FD control bus, Damiao QDD
    motors: stated in the OpenArm 2.0 docs and the task brief.
  * 5 Mbit/s data phase: stated in the task brief.
  * 1 kHz control loop: stated in OpenArm 2.0 material.
  * Motor parameter ranges (P_MAX, V_MAX, T_MAX) below: these follow the
    public MIT / Damiao convention. They are motor-model specific and I
    could NOT verify them against a datasheet without hardware. They are
    marked UNVERIFIED and must be confirmed before use on a real arm.
  * Camera frame rates: assumed, see CAMERAS. Chosen to be realistic and,
    deliberately, mutually non-harmonic so the sync logic gets exercised.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# --------------------------------------------------------------------------
# CAN bus
# --------------------------------------------------------------------------

#: Arbitration phase bitrate (the slower phase that carries the message ID).
CAN_BITRATE = 1_000_000

#: Data phase bitrate. CAN FD's whole point: the payload moves faster than
#: the header. The task brief specifies 5 Mbit/s.
CAN_DATA_BITRATE = 5_000_000

#: One bus per arm. Splitting the two arms keeps each bus's load low enough
#: that a 1 kHz loop stays deterministic -- see docs/01-can-setup.md.
CAN_INTERFACES = ("can0", "can1")

#: Commanded/reported joint state rate, per motor.
CONTROL_RATE_HZ = 1000.0


@dataclass(frozen=True)
class MotorSpec:
    """Scaling limits for one motor model.

    Damiao motors in MIT mode do not transmit floating point numbers. They
    transmit integers, and both ends must agree on what range those integers
    span. Get these wrong and the arm still "works" -- it just reports
    physically wrong values, which is the worst kind of bug.
    """

    model: str
    p_max: float  # rad, position range is [-p_max, +p_max]
    v_max: float  # rad/s
    t_max: float  # N*m
    verified: bool = False  # True only once checked against a datasheet


#: UNVERIFIED -- see module docstring. Placeholder values following the
#: public MIT-mode convention for these Damiao families.
MOTOR_SPECS = {
    "DM43": MotorSpec("Damiao 43-series", p_max=12.5, v_max=30.0, t_max=10.0),
    "DM8009P": MotorSpec("Damiao 8009P", p_max=12.5, v_max=45.0, t_max=54.0),
}


@dataclass(frozen=True)
class JointSpec:
    name: str
    can_id: int
    interface: str
    motor: str


def _arm_joints(side: str, interface: str) -> list[JointSpec]:
    """Seven joints per arm, CAN IDs 0x01..0x07 on that arm's own bus.

    The larger 8009P motors sit at the shoulder where they carry the whole
    limb; the lighter 43-series run the forearm and wrist. Which model sits
    where is an assumption -- the ID-to-joint mapping on real hardware comes
    from the motor configuration, not from us.
    """
    layout = ["j1_shoulder_pitch", "j2_shoulder_roll", "j3_shoulder_yaw",
              "j4_elbow", "j5_wrist_roll", "j6_wrist_pitch", "j7_wrist_yaw"]
    return [
        JointSpec(
            name=f"{side}_{n}",
            can_id=0x01 + i,
            interface=interface,
            motor="DM8009P" if i < 3 else "DM43",
        )
        for i, n in enumerate(layout)
    ]


JOINTS: list[JointSpec] = _arm_joints("left", "can0") + _arm_joints("right", "can1")
JOINT_NAMES: list[str] = [j.name for j in JOINTS]
N_JOINTS = len(JOINTS)


# --------------------------------------------------------------------------
# Cameras
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class CameraSpec:
    name: str
    width: int
    height: int
    fps: float
    stereo: bool = False


#: ASSUMED rates. The exact numbers matter less than the fact that they do
#: not divide evenly into each other or into the 1 kHz joint rate -- that is
#: precisely the condition that makes naive "just zip them together" sync
#: silently wrong, so the defaults keep us honest.
CAMERAS: tuple[CameraSpec, ...] = (
    CameraSpec("wrist_left", 640, 480, fps=30.0),
    CameraSpec("wrist_right", 640, 480, fps=30.0),
    CameraSpec("ceiling", 1280, 720, fps=15.0),
    CameraSpec("zed_head", 1280, 720, fps=60.0, stereo=True),
)

CAMERA_NAMES: list[str] = [c.name for c in CAMERAS]

#: The camera whose frames define the training timeline. See
#: cameras/sync.py -- imitation learning wants one sample per
#: observation, and the observation is an image.
PRIMARY_CAMERA = "zed_head"


# --------------------------------------------------------------------------
# Recording
# --------------------------------------------------------------------------

#: Where episodes land. Gitignored.
DATA_DIR = "data/episodes"

#: A joint state is only considered a valid match for a camera frame if it
#: falls within this window. At 1 kHz, states arrive every 1 ms, so 5 ms is
#: generous; exceeding it means samples were genuinely dropped and we would
#: rather record that fact than paper over it.
SYNC_TOLERANCE_S = 0.005

#: Bounded queue depth between capture threads and the writer. Bounded on
#: purpose: an unbounded queue converts a disk stall into unbounded memory
#: growth, which fails later and worse. See recorder.py.
QUEUE_MAXSIZE = 2048
