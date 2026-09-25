"""
Integration tests for AttitudeReader against the mock UDP camera
(tests/conftest.py). These exercise the real network path (UDP send +
background receiver thread), not just parsing.
"""

import time

from skydroid_c12.protocol import TPTransport
from skydroid_c12.telemetry import AttitudeReader, get_attitude_once


def _transport(mock_camera) -> TPTransport:
    # bind_source_port=False: avoid trying to bind to the mock's own
    # (randomly assigned) port from the client side.
    return TPTransport("127.0.0.1", mock_camera.port, bind_source_port=False)


def test_attitude_reader_receives_packets(mock_camera):
    t = _transport(mock_camera)
    reader = AttitudeReader(t, rate_hz=20)
    try:
        reader.start()
        deadline = time.monotonic() + 3.0
        while reader.get_latest() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        sample = reader.get_latest()
        assert sample is not None
        assert reader.is_fresh()
        assert reader.packet_count > 0
    finally:
        reader.stop()
        t.close()


def test_attitude_reader_tracks_camera_state(mock_camera):
    mock_camera.state.pan_deg = 12.5
    mock_camera.state.tilt_deg = -7.25
    t = _transport(mock_camera)
    reader = AttitudeReader(t, rate_hz=20)
    try:
        reader.start()
        deadline = time.monotonic() + 3.0
        sample = None
        while time.monotonic() < deadline:
            sample = reader.get_latest()
            if sample is not None:
                break
            time.sleep(0.05)
        assert sample is not None
        assert sample.yaw_deg == 12.5
        assert sample.pitch_deg == -7.25
    finally:
        reader.stop()
        t.close()


def test_get_attitude_once_returns_a_sample(mock_camera):
    sample = get_attitude_once("127.0.0.1", mock_camera.port, timeout_sec=3.0)
    assert sample is not None


def test_get_attitude_once_times_out_when_nothing_listens():
    # Port 1 should not have anything answering during the test run.
    sample = get_attitude_once("127.0.0.1", 1, timeout_sec=0.3)
    assert sample is None
