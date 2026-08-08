"""Protocol tests.

Without hardware I cannot prove the decoder matches a real Damiao motor.
What I CAN prove is that the decoder is self-consistent, handles the awkward
split-byte field correctly, and fails loudly on malformed input. Those are
the bugs most likely to be mine; the remaining risk is that the documented
layout itself is wrong, which is stated plainly in the README.
"""

import struct

import pytest

from openarm_pipeline.config import MOTOR_SPECS
from openarm_pipeline.can.protocol import (
    CMD_SET_ZERO,
    JointFeedback,
    decode_feedback,
    encode_feedback,
    _float_to_uint,
    _uint_to_float,
)

SPEC = MOTOR_SPECS["DM43"]


def _fb(pos=0.0, vel=0.0, tau=0.0, can_id=1):
    return JointFeedback(can_id, pos, vel, tau, 40, 45, "ok")


@pytest.mark.parametrize("pos,vel,tau", [
    (0.0, 0.0, 0.0),
    (1.5708, 2.0, 1.25),
    (-1.5708, -2.0, -1.25),
    (12.5, 30.0, 10.0),      # exactly at the limits
    (-12.5, -30.0, -10.0),
    (0.001, -0.001, 0.001),  # near zero, where quantisation bites hardest
])
def test_roundtrip_preserves_values(pos, vel, tau):
    """Encode then decode must return what went in, within quantisation error.

    Tolerances are derived from the field widths, not guessed. Position gets
    16 bits over +/-12.5 rad, so one step is 25/65535 rad. Velocity and torque
    get 12 bits, so their steps are 4096ths of their range and are coarser --
    a real and unavoidable property of the wire format, worth knowing about.
    """
    decoded = decode_feedback(encode_feedback(_fb(pos, vel, tau), SPEC), SPEC)

    assert decoded.position_rad == pytest.approx(pos, abs=2 * SPEC.p_max / 65535)
    assert decoded.velocity_rad_s == pytest.approx(vel, abs=2 * SPEC.v_max / 4095)
    assert decoded.torque_nm == pytest.approx(tau, abs=2 * SPEC.t_max / 4095)


def test_byte4_is_split_between_velocity_and_torque():
    """The field most likely to be parsed wrong.

    Velocity uses byte 4's HIGH nibble, torque its LOW nibble. If a decoder
    mixes them up, both values are wrong together -- so this drives velocity
    to its maximum while holding torque at its minimum and checks they stay
    independent. A nibble swap cannot pass this.
    """
    frame = encode_feedback(_fb(vel=SPEC.v_max, tau=-SPEC.t_max), SPEC)
    decoded = decode_feedback(frame, SPEC)

    assert decoded.velocity_rad_s == pytest.approx(SPEC.v_max, rel=1e-3)
    assert decoded.torque_nm == pytest.approx(-SPEC.t_max, rel=1e-3)

    # And the reverse pairing, so the test cannot pass by symmetry.
    frame = encode_feedback(_fb(vel=-SPEC.v_max, tau=SPEC.t_max), SPEC)
    decoded = decode_feedback(frame, SPEC)
    assert decoded.velocity_rad_s == pytest.approx(-SPEC.v_max, rel=1e-3)
    assert decoded.torque_nm == pytest.approx(SPEC.t_max, rel=1e-3)


def test_scaling_endpoints_are_exact():
    """Integer 0 must mean x_min, and all-ones must mean x_max."""
    assert _uint_to_float(0, -12.5, 12.5, 16) == pytest.approx(-12.5)
    assert _uint_to_float(65535, -12.5, 12.5, 16) == pytest.approx(12.5)
    assert _float_to_uint(-12.5, -12.5, 12.5, 16) == 0
    assert _float_to_uint(12.5, -12.5, 12.5, 16) == 65535


def test_out_of_range_clamps_rather_than_wrapping():
    """Overflow must saturate, never wrap.

    A wrap turns +13 rad into roughly -12 rad: a full reversal, silently.
    Clamping is wrong by a little; wrapping is wrong by everything.
    """
    assert _float_to_uint(999.0, -12.5, 12.5, 16) == 65535
    assert _float_to_uint(-999.0, -12.5, 12.5, 16) == 0


def test_can_id_and_error_share_byte_zero():
    frame = bytearray(encode_feedback(_fb(can_id=7), SPEC))
    frame[0] = 0x07 | (0xA << 4)          # ID 7, overcurrent
    decoded = decode_feedback(bytes(frame), SPEC)

    assert decoded.can_id == 7
    assert decoded.error == "overcurrent"
    assert not decoded.healthy


def test_short_frame_raises():
    """A truncated frame means bus trouble. Refuse it rather than guess."""
    with pytest.raises(ValueError, match="too short"):
        decode_feedback(b"\x01\x02\x03", SPEC)


def test_negative_temperatures_decode_as_signed():
    frame = bytearray(encode_feedback(_fb(), SPEC))
    frame[6] = struct.pack("b", -10)[0]
    assert decode_feedback(bytes(frame), SPEC).temp_mos_c == -10


def test_set_zero_command_is_the_documented_sequence():
    """Task 1's zero-position tare, at the byte level."""
    assert CMD_SET_ZERO == b"\xff\xff\xff\xff\xff\xff\xff\xfe"
    assert len(CMD_SET_ZERO) == 8
