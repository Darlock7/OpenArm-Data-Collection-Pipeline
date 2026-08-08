"""Damiao motor frame encoding and decoding (MIT mode).

A CAN frame carries at most a handful of bytes, and a motor needs to report
position, velocity, torque and two temperatures in every one of them, a
thousand times a second. There is no room for anything human readable, so
the values are bit-packed integers and both ends must agree on the layout.

FEEDBACK FRAME -- 8 bytes, motor -> host:

    byte  0   1   2   3   4   5   6   7
         +---+---+---+---+---+---+---+---+
         |ID |  POS  | VEL |V/T|TRQ|Tm |Tr |
         +---+---+---+---+---+---+---+---+

    byte 0      : low nibble = motor ID, high nibble = error code
    bytes 1-2   : position, 16 bits
    bytes 3-4   : velocity, 12 bits (byte 3, plus the HIGH nibble of byte 4)
    bytes 4-5   : torque,   12 bits (LOW nibble of byte 4, plus byte 5)
    byte 6      : MOSFET temperature, degrees C, signed
    byte 7      : rotor temperature, degrees C, signed

Byte 4 is shared: its top half finishes the velocity and its bottom half
starts the torque. That is where a hand-written parser usually goes wrong.

Those integers are not physical units. A 16-bit position is just a number
from 0 to 65535 spanning the motor's full travel, so converting it back to
radians requires knowing that travel -- see config.MotorSpec. Using the
wrong limits yields values that look plausible and are wrong, which is why
config.py marks them UNVERIFIED until checked against a datasheet.

CONFIDENCE: this layout is the public MIT / Damiao convention, cross-checked
against the openly documented protocol. It is NOT verified against real
hardware, because I had none. See README "What I could not verify".
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from ..config import MOTOR_SPECS, MotorSpec

# --------------------------------------------------------------------------
# Special command frames
# --------------------------------------------------------------------------
# Damiao reserves a few all-0xFF payloads as out-of-band commands. These are
# the frames `openarm-can-cli` sends underneath its friendly subcommands --
# in particular SET_ZERO is what "set the zero position" in Task 1 actually
# puts on the wire.

CMD_ENABLE      = bytes([0xFF] * 7 + [0xFC])
CMD_DISABLE     = bytes([0xFF] * 7 + [0xFD])
CMD_SET_ZERO    = bytes([0xFF] * 7 + [0xFE])
CMD_CLEAR_ERROR = bytes([0xFF] * 7 + [0xFB])


ERROR_CODES = {
    0x0: "ok",
    0x8: "overvoltage",
    0x9: "undervoltage",
    0xA: "overcurrent",
    0xB: "mos_over_temperature",
    0xC: "rotor_over_temperature",
    0xD: "lost_communication",
    0xE: "overload",
}


@dataclass(frozen=True)
class JointFeedback:
    """One motor's reported state, in physical units."""

    can_id: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    temp_mos_c: int
    temp_rotor_c: int
    error: str

    @property
    def healthy(self) -> bool:
        return self.error == "ok"


def _uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    """Spread an unsigned integer back across a physical range.

    The motor did the inverse: it took a value in [x_min, x_max] and mapped
    it onto the integers 0 .. 2**bits - 1. So 0 means x_min, the maximum
    integer means x_max, and everything else interpolates linearly.
    """
    span = x_max - x_min
    return x_int * span / float((1 << bits) - 1) + x_min


def _float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """Inverse of _uint_to_float, clamped so we never overflow the field."""
    x = max(x_min, min(x_max, x))
    span = x_max - x_min
    return int((x - x_min) * float((1 << bits) - 1) / span)


def decode_feedback(data: bytes, spec: MotorSpec | None = None,
                    motor_key: str = "DM43") -> JointFeedback:
    """Turn 8 raw bytes from the bus into physical units.

    Raises ValueError on a short frame rather than returning garbage. A
    truncated frame means something is wrong on the bus, and silently
    decoding it would push corrupt numbers into the dataset.
    """
    if len(data) < 6:
        raise ValueError(f"feedback frame too short: {len(data)} bytes, need >= 6")

    spec = spec or MOTOR_SPECS[motor_key]

    can_id = data[0] & 0x0F
    error = ERROR_CODES.get((data[0] >> 4) & 0x0F, "unknown")

    # The bit-unpacking. Note byte 4 being split between two fields.
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0x0F) << 8) | data[5]

    return JointFeedback(
        can_id=can_id,
        position_rad=_uint_to_float(p_int, -spec.p_max, spec.p_max, 16),
        velocity_rad_s=_uint_to_float(v_int, -spec.v_max, spec.v_max, 12),
        torque_nm=_uint_to_float(t_int, -spec.t_max, spec.t_max, 12),
        # Temperatures are plain signed bytes, not scaled.
        temp_mos_c=struct.unpack("b", bytes([data[6]]))[0] if len(data) > 6 else 0,
        temp_rotor_c=struct.unpack("b", bytes([data[7]]))[0] if len(data) > 7 else 0,
        error=error,
    )


def encode_feedback(fb: JointFeedback, spec: MotorSpec | None = None,
                    motor_key: str = "DM43") -> bytes:
    """Build a feedback frame. Used by the mock arm and by the round-trip test.

    Having the exact inverse available is what lets tests/test_protocol.py
    prove the decoder is right without hardware: encode a known value, decode
    it, check it survives.
    """
    spec = spec or MOTOR_SPECS[motor_key]

    p_int = _float_to_uint(fb.position_rad, -spec.p_max, spec.p_max, 16)
    v_int = _float_to_uint(fb.velocity_rad_s, -spec.v_max, spec.v_max, 12)
    t_int = _float_to_uint(fb.torque_nm, -spec.t_max, spec.t_max, 12)

    err_nibble = 0x0 if fb.error == "ok" else 0xE
    return bytes([
        (fb.can_id & 0x0F) | (err_nibble << 4),
        (p_int >> 8) & 0xFF,
        p_int & 0xFF,
        (v_int >> 4) & 0xFF,
        ((v_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F),
        t_int & 0xFF,
        fb.temp_mos_c & 0xFF,
        fb.temp_rotor_c & 0xFF,
    ])
