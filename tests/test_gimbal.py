"""
Integration tests for GimbalController against the mock UDP camera:
covers both open-loop (no IMU) and closed-loop (IMU-confirmed) movement,
plus the tilt safety limit.
"""

import time

import pytest

from skydroid_c12.gimbal import GimbalController
from skydroid_c12.protocol import TPTransport
from skydroid_c12.telemetry import AttitudeReader


def _transport(mock_camera) -> TPTransport:
    return TPTransport("127.0.0.1", mock_camera.port, bind_source_port=False)


def test_open_loop_goto_pan_angle_updates_known_position(mock_camera):
    """Without an AttitudeReader, goto_pan_angle falls back to a
    time-estimated sleep and just trusts the commanded angle."""
    t = _transport(mock_camera)
    gimbal = GimbalController(t)
    try:
        gimbal.goto_pan_angle(20, speed=80, wait=True)
        assert gimbal.known_pan_angle == 20
    finally:
        t.close()


def test_closed_loop_goto_pan_angle_waits_for_real_imu_confirmation(mock_camera):
    """With a fresh AttitudeReader, goto_pan_angle should report the IMU's
    actual measured angle, which must match the commanded target once the
    mock camera has converged."""
    t = _transport(mock_camera)
    imu = AttitudeReader(t, rate_hz=20)
    imu.start()
    gimbal = GimbalController(t, attitude_reader=imu, attitude_tolerance_deg=0.5)
    try:
        gimbal.goto_pan_angle(15, speed=80, wait=True)
        assert gimbal.known_pan_angle == pytest.approx(15, abs=0.5)
        assert mock_camera.state.pan_deg == pytest.approx(15, abs=0.5)
    finally:
        imu.stop()
        t.close()


def test_closed_loop_goto_tilt_angle(mock_camera):
    t = _transport(mock_camera)
    imu = AttitudeReader(t, rate_hz=20)
    imu.start()
    gimbal = GimbalController(t, attitude_reader=imu, attitude_tolerance_deg=0.5)
    try:
        gimbal.goto_tilt_angle(10, speed=80, wait=True)
        assert gimbal.known_tilt_angle == pytest.approx(10, abs=0.5)
    finally:
        imu.stop()
        t.close()


def test_goto_pan_tilt_moves_both_axes_together(mock_camera):
    t = _transport(mock_camera)
    imu = AttitudeReader(t, rate_hz=20)
    imu.start()
    gimbal = GimbalController(t, attitude_reader=imu, attitude_tolerance_deg=0.5)
    try:
        gimbal.goto_pan_tilt(20, -10, speed=80, wait=True)
        assert gimbal.known_pan_angle == pytest.approx(20, abs=0.5)
        assert gimbal.known_tilt_angle == pytest.approx(-10, abs=0.5)
    finally:
        imu.stop()
        t.close()


def test_move_right_is_relative_to_current_position(mock_camera):
    t = _transport(mock_camera)
    gimbal = GimbalController(t)
    try:
        gimbal.goto_pan_angle(10, speed=80, wait=True)
        gimbal.move_right(5, speed=80, wait=True)
        assert gimbal.known_pan_angle == 15
    finally:
        t.close()


def test_tilt_outside_safety_limit_is_rejected_before_sending(mock_camera):
    t = _transport(mock_camera)
    gimbal = GimbalController(t, max_tilt_deg=45.0, min_tilt_deg=-45.0)
    try:
        with pytest.raises(ValueError):
            gimbal.goto_tilt_angle(60, wait=False)
        # The mock camera should not have moved at all.
        time.sleep(0.1)
        assert mock_camera.state.tilt_deg == 0.0
    finally:
        t.close()


def test_center_resets_known_position(mock_camera):
    t = _transport(mock_camera)
    gimbal = GimbalController(t)
    try:
        gimbal.goto_pan_angle(30, speed=80, wait=True)
        gimbal.center(wait=False)
        assert gimbal.known_pan_angle == 0.0
        assert gimbal.known_tilt_angle == 0.0
    finally:
        t.close()
