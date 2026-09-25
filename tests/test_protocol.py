"""
Pure, fast unit tests for the low-level #TP protocol encoding/decoding
(no network involved).
"""

import pytest

from skydroid_c12.protocol import (
    PTZ,
    AttitudeSample,
    build_command,
    checksum,
    gimbal_attitude_push_cmd,
    gimbal_pan_absolute_angle_cmd,
    gimbal_pan_speed_cmd,
    gimbal_tilt_absolute_angle_cmd,
    gimbal_tilt_speed_cmd,
    int2hex,
    parse_gac_packet,
    short2hex,
)


def test_checksum_is_mod_256_hex_of_byte_sum():
    # Sum of ord() for "AB" = 65 + 66 = 131 = 0x83
    assert checksum("AB") == "83"


def test_build_command_appends_checksum():
    cmd = "#TPUG2wPTZ00"
    built = build_command(cmd)
    assert built == (cmd + checksum(cmd)).encode("utf-8")


@pytest.mark.parametrize(
    "value,expected",
    [(0, "00"), (1, "01"), (-1, "FF"), (99, "63"), (-99, "9D")],
)
def test_int2hex_two_digit_twos_complement(value, expected):
    assert int2hex(value) == expected


@pytest.mark.parametrize("bad_value", [100, -100, 1000])
def test_int2hex_rejects_out_of_range(bad_value):
    with pytest.raises(ValueError):
        int2hex(bad_value)


@pytest.mark.parametrize(
    "value,expected",
    [(0, "0000"), (100, "0064"), (-100, "FF9C"), (3000, "0BB8")],
)
def test_short2hex_four_digit_twos_complement(value, expected):
    assert short2hex(value) == expected


def test_ptz_constants_match_verified_hardware_table():
    # These specific values were confirmed against real hardware after an
    # earlier, generic table turned out to be wrong (see PTZ docstring).
    assert PTZ.STOP.endswith("PTZ00")
    assert PTZ.UP.endswith("PTZ01")
    assert PTZ.DOWN.endswith("PTZ02")
    assert PTZ.LEFT.endswith("PTZ03")
    assert PTZ.RIGHT.endswith("PTZ04")
    assert PTZ.BACK_MID.endswith("PTZ05")


def test_pan_speed_zero_is_stop():
    assert gimbal_pan_speed_cmd(0) == PTZ.STOP


def test_tilt_speed_zero_is_stop():
    assert gimbal_tilt_speed_cmd(0) == PTZ.STOP


def test_pan_speed_encodes_sign_and_magnitude():
    assert gimbal_pan_speed_cmd(25).endswith("GSY" + int2hex(25))
    assert gimbal_pan_speed_cmd(-25).endswith("GSY" + int2hex(-25))


def test_pan_absolute_angle_flips_sign_internally():
    """
    GAY's raw wire sign is the opposite of what the public API promises
    (positive degrees = right). This test locks in that correction so a
    future refactor can't silently reintroduce the direction bug that
    was found and fixed on real hardware.
    """
    cmd_right = gimbal_pan_absolute_angle_cmd(30, speed=30)
    cmd_left = gimbal_pan_absolute_angle_cmd(-30, speed=30)
    # +30 requested (right) -> raw encoded angle is NEGATIVE (-30*100)
    assert short2hex(-3000) in cmd_right
    # -30 requested (left) -> raw encoded angle is POSITIVE (+30*100)
    assert short2hex(3000) in cmd_left


def test_tilt_absolute_angle_does_not_flip_sign():
    """GAP's sign matches the public API directly - no correction needed."""
    cmd_up = gimbal_tilt_absolute_angle_cmd(10, speed=25)
    cmd_down = gimbal_tilt_absolute_angle_cmd(-10, speed=25)
    assert short2hex(1000) in cmd_up
    assert short2hex(-1000) in cmd_down


def test_attitude_push_cmd_encodes_rate_as_two_hex_digits():
    assert gimbal_attitude_push_cmd(10) == "#TPUG2wGAA0A"
    assert gimbal_attitude_push_cmd(0) == "#TPUG2wGAA00"


def test_attitude_push_cmd_clamps_to_valid_range():
    assert gimbal_attitude_push_cmd(255) == "#TPUG2wGAA64"  # clamped to 100
    assert gimbal_attitude_push_cmd(-5) == "#TPUG2wGAA00"    # clamped to 0


def test_parse_gac_packet_roundtrip():
    # yaw=12.34, pitch=-5.00, roll=0.10 (degrees*100, signed 16-bit hex)
    payload = "GAC" + "04D2" + "FE0C" + "000A"
    sample = parse_gac_packet(("#tp" + payload + "00").encode("ascii"))
    assert isinstance(sample, AttitudeSample)
    assert sample.yaw_deg == pytest.approx(12.34)
    assert sample.pitch_deg == pytest.approx(-5.00)
    assert sample.roll_deg == pytest.approx(0.10)


def test_parse_gac_packet_returns_none_for_non_gac_data():
    assert parse_gac_packet(b"#tpDU3rVER0105025A") is None
    assert parse_gac_packet(b"") is None
    assert parse_gac_packet(b"garbage") is None


def test_parse_gac_packet_returns_none_for_truncated_payload():
    assert parse_gac_packet(b"#tpGAC04D2") is None
