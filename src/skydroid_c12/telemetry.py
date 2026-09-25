"""
High-level API for reading attitude (yaw/pitch/roll) telemetry.

VERIFIED ON REAL HARDWARE: with GAA(10Hz) streaming on, sending a
"tilt to 20" command made the pitch reading climb in real time from 0 to
20.00 degrees, matching the commanded value exactly.

Usage 1 - continuous reading (recommended, e.g. inside a control loop
that checks the IMU every iteration):

    from skydroid_c12 import TPTransport, AttitudeReader

    t = TPTransport("192.168.144.108", 5000)
    reader = AttitudeReader(t, rate_hz=10)
    reader.start()
    ...
    sample = reader.get_latest()          # cheap, non-blocking, raw value
    if sample and reader.is_fresh():
        print(sample.yaw_deg, sample.pitch_deg, sample.roll_deg)
    ...
    reader.stop()
    t.close()

Usage 2 - one-shot reading (e.g. "what's the current angle" from a quick
script or a Python console):

    from skydroid_c12 import get_attitude_once

    sample = get_attitude_once("192.168.144.108")
    print(sample.yaw_deg, sample.pitch_deg, sample.roll_deg)

Neither needs HTTP or any network beyond the direct UDP connection to the
camera - both return the GAC packet's contents completely unprocessed (no
filtering, smoothing or correction).
"""

from __future__ import annotations

import logging
import threading
import time

from .protocol import (
    TPTransport,
    AttitudeSample,
    gimbal_attitude_push_cmd,
    parse_gac_packet,
)

logger = logging.getLogger(__name__)


class AttitudeReader:
    """
    Starts requesting attitude (GAC) packets from the camera via the GAA
    command, and keeps the latest yaw/pitch/roll value thread-safe.

    This is not raw gyro data - it's whatever angle the camera's own
    internal IMU/stabilization system computes and pushes out (when
    available). It isn't a hard encoder-grade position feedback, but it
    can be far more reliable than the time-based dead-reckoning estimate
    used by goto_pan()/goto_tilt() - *if* it's working correctly on your
    unit.
    """

    def __init__(self, transport: TPTransport, rate_hz: int = 10):
        self._t = transport
        self._rate_hz = rate_hz
        self._lock = threading.Lock()
        self._latest: AttitudeSample | None = None
        self._last_update_ts = 0.0
        self._packet_count = 0
        self._started = False

    def start(self) -> None:
        """Start the receiver thread and send the GAA command (a few
        times, for reliability)."""
        if self._started:
            logger.warning("AttitudeReader is already started.")
            return
        self._t.start_receiver(self._on_packet)
        cmd = gimbal_attitude_push_cmd(self._rate_hz)
        for _ in range(3):
            self._t.send(cmd)
            time.sleep(0.15)
        self._started = True
        logger.info("AttitudeReader started (rate=%d Hz).", self._rate_hz)

    def stop(self) -> None:
        """Turn off GAA streaming (rate=0) and stop the receiver thread."""
        if not self._started:
            return
        try:
            self._t.send(gimbal_attitude_push_cmd(0))
        except Exception:
            logger.exception("Failed to send GAA stop command.")
        self._t.stop_receiver()
        self._started = False
        logger.info("AttitudeReader stopped.")

    def _on_packet(self, data: bytes) -> None:
        sample = parse_gac_packet(data)
        if sample is None:
            return
        with self._lock:
            self._latest = sample
            self._last_update_ts = time.monotonic()
            self._packet_count += 1

    def get_latest(self) -> AttitudeSample | None:
        """Return the most recent yaw/pitch/roll sample (None if none yet)."""
        with self._lock:
            return self._latest

    def is_fresh(self, max_age_sec: float = 1.0) -> bool:
        """Whether the latest sample is newer than max_age_sec seconds.
        Returns False if no packet has ever been received."""
        with self._lock:
            if self._latest is None:
                return False
            return (time.monotonic() - self._last_update_ts) < max_age_sec

    @property
    def packet_count(self) -> int:
        """Number of GAC packets successfully parsed so far (for diagnostics)."""
        with self._lock:
            return self._packet_count


def get_attitude_once(
    camera_ip: str,
    udp_port: int = 5000,
    timeout_sec: float = 2.0,
    bind_source_port: bool = True,
) -> AttitudeSample | None:
    """
    One-shot helper: connects to the camera, sends GAA, waits for the
    first GAC packet, returns the raw yaw/pitch/roll, and closes the
    connection.

    No HTTP, no server - just a plain function call. Handy for a quick
    "what's the current angle" check from a script or a Python console.

    If you're going to read repeatedly (e.g. every frame of a control
    loop), use AttitudeReader instead - reconnecting and re-sending GAA
    on every call is wasteful.

    bind_source_port: see TPTransport. Leave this True for real hardware
        (the default). Set False when pointing this at a local/mock
        camera on the same machine (e.g. in tests) to avoid a source-port
        collision with the mock's own socket.

    Returns None if no GAC packet arrives within timeout_sec.
    """
    t = TPTransport(camera_ip, udp_port, bind_source_port=bind_source_port)
    reader = AttitudeReader(t, rate_hz=10)
    reader.start()
    try:
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            sample = reader.get_latest()
            if sample is not None:
                return sample
            time.sleep(0.05)
        return None
    finally:
        reader.stop()
        t.close()
