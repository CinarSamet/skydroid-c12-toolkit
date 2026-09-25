"""
skydroid_c12 - Reverse-engineered SDK for the Skydroid C12 gimbal camera.

Skydroid does not publish an official SDK for the C12 (and the related
C10/C11/C13/C14/C20 family). This package is a from-scratch, real-hardware
verified reimplementation of the "#TP" UDP text protocol used by the
official Android FPV app, covering:

  - Pan/tilt movement: speed-based (continuous) and absolute-angle
    (single-shot, precise) commands.
  - IMU/attitude telemetry: real-time yaw/pitch/roll readout pushed by
    the camera itself.
  - Closed-loop control: movement commands can wait for the IMU to
    confirm the gimbal actually reached the target angle, instead of
    just sleeping for an estimated duration.

Quick start
-----------

    from skydroid_c12 import Camera

    with Camera("192.168.144.108") as cam:
        cam.center()
        cam.goto_pan(30)          # absolute angle, closed-loop if IMU is on
        cam.goto_tilt(10)
        sample = cam.get_attitude()
        if sample:
            print(sample.yaw_deg, sample.pitch_deg, sample.roll_deg)

See the README for the full API and the verified protocol notes.
"""

from .protocol import (
    AttitudeSample,
    PTZ,
    Record,
    Capture,
    TPTransport,
    checksum,
    build_command,
    parse_gac_packet,
)
from .telemetry import AttitudeReader, get_attitude_once
from .gimbal import GimbalController
from .camera import Camera
from .config import AppConfig, NetworkConfig, GimbalConfig, DEFAULT_CONFIG

__all__ = [
    "Camera",
    "GimbalController",
    "AttitudeReader",
    "get_attitude_once",
    "AttitudeSample",
    "TPTransport",
    "PTZ",
    "Record",
    "Capture",
    "checksum",
    "build_command",
    "parse_gac_packet",
    "AppConfig",
    "NetworkConfig",
    "GimbalConfig",
    "DEFAULT_CONFIG",
]

__version__ = "0.1.0"
