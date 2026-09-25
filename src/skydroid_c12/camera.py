"""
`Camera` - a single, friendly entry point that wires together
`TPTransport` + `AttitudeReader` + `GimbalController` for the common case.

If you want finer control (e.g. sharing one transport across several
objects, or a custom IMU polling rate per call), use the lower-level
classes directly - `Camera` is a convenience wrapper, not the only way to
use this SDK.
"""

from __future__ import annotations

from .config import GimbalConfig
from .gimbal import GimbalController
from .protocol import AttitudeSample, TPTransport
from .telemetry import AttitudeReader


class Camera:
    """
    High-level, single-object API for a Skydroid C12 gimbal camera.

    Example
    -------
        from skydroid_c12 import Camera

        with Camera("192.168.144.108") as cam:
            cam.center()
            cam.goto_pan(30)                # absolute, closed-loop if IMU is on
            cam.goto_tilt(10)
            cam.goto(pan=-20, tilt=5)        # both axes at once

            cam.move_right(10)               # relative to current position
            cam.move_tilt_down(5)

            sample = cam.get_attitude()      # raw yaw/pitch/roll, may be None
            if sample:
                print(sample.yaw_deg, sample.pitch_deg, sample.roll_deg)

            print(cam.pan_deg, cam.tilt_deg)  # known position (real if IMU is on)

            cam.stop_now()
    """

    def __init__(
        self,
        camera_ip: str,
        udp_port: int = 5000,
        config: GimbalConfig | None = None,
        enable_imu: bool = True,
    ):
        self._config = config or GimbalConfig()
        self._transport = TPTransport(camera_ip, udp_port, min_interval_sec=self._config.min_command_interval_sec)
        self._imu: AttitudeReader | None = None
        if enable_imu:
            self._imu = AttitudeReader(self._transport, rate_hz=self._config.attitude_gac_rate_hz)
        self._gimbal = GimbalController(
            self._transport,
            max_tilt_deg=self._config.max_tilt_deg,
            min_tilt_deg=self._config.min_tilt_deg,
            attitude_reader=self._imu if (self._imu and self._config.enable_attitude_feedback) else None,
            attitude_tolerance_deg=self._config.attitude_tolerance_deg,
            attitude_timeout_multiplier=self._config.attitude_timeout_multiplier,
        )
        self._default_speed = self._config.default_speed

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> "Camera":
        """Start the IMU reader (if enabled). Movement commands work even
        without calling start() first - but you won't get closed-loop
        confirmation or `get_attitude()` data until the IMU is running."""
        if self._imu is not None:
            self._imu.start()
        return self

    def close(self) -> None:
        """Stop the IMU and close the socket. Always call this at the end
        of your program (or use `with Camera(...) as cam:`)."""
        if self._imu is not None:
            self._imu.stop()
        self._transport.close()

    def __enter__(self) -> "Camera":
        self.start()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    def goto_pan(self, degrees: float, speed: int | None = None, wait: bool = True) -> None:
        """Absolute pan angle (positive=right, negative=left)."""
        self._gimbal.goto_pan_angle(degrees, speed or self._default_speed, wait)

    def goto_tilt(self, degrees: float, speed: int | None = None, wait: bool = True) -> None:
        """Absolute tilt angle (positive=up, negative=down)."""
        self._gimbal.goto_tilt_angle(degrees, speed or self._default_speed, wait)

    def goto(
        self,
        pan: float | None = None,
        tilt: float | None = None,
        speed: int | None = None,
        wait: bool = True,
    ) -> None:
        """Absolute pan and/or tilt, moving both axes at once when both
        are given."""
        if pan is not None and tilt is not None:
            self._gimbal.goto_pan_tilt(pan, tilt, speed=speed or self._default_speed, wait=wait)
        elif pan is not None:
            self.goto_pan(pan, speed, wait)
        elif tilt is not None:
            self.goto_tilt(tilt, speed, wait)

    def move_right(self, degrees: float, speed: int | None = None, wait: bool = True) -> None:
        self._gimbal.move_right(degrees, speed or self._default_speed, wait)

    def move_left(self, degrees: float, speed: int | None = None, wait: bool = True) -> None:
        self._gimbal.move_left(degrees, speed or self._default_speed, wait)

    def move_tilt_up(self, degrees: float, speed: int | None = None, wait: bool = True) -> None:
        self._gimbal.move_tilt_up(degrees, speed or self._default_speed, wait)

    def move_tilt_down(self, degrees: float, speed: int | None = None, wait: bool = True) -> None:
        self._gimbal.move_tilt_down(degrees, speed or self._default_speed, wait)

    def center(self, wait: bool = True) -> None:
        self._gimbal.center(wait)

    def stop_now(self) -> None:
        self._gimbal.stop()

    def take_photo(self) -> None:
        self._gimbal.take_photo()

    # ------------------------------------------------------------------
    # IMU / attitude
    # ------------------------------------------------------------------

    def get_attitude(self) -> AttitudeSample | None:
        """Latest raw yaw/pitch/roll sample (None if IMU is off or no
        packet has arrived yet)."""
        if self._imu is None:
            return None
        return self._imu.get_latest()

    def attitude_is_fresh(self, max_age_sec: float = 1.0) -> bool:
        if self._imu is None:
            return False
        return self._imu.is_fresh(max_age_sec)

    @property
    def imu_packet_count(self) -> int:
        return self._imu.packet_count if self._imu is not None else 0

    @property
    def pan_deg(self) -> float:
        """Known pan position - the real IMU measurement if it's fresh,
        otherwise a command-history estimate."""
        return self._gimbal.known_pan_angle

    @property
    def tilt_deg(self) -> float | None:
        """Known tilt position - same rule as pan_deg."""
        return self._gimbal.known_tilt_angle

    # ------------------------------------------------------------------
    # Escape hatch to the lower-level objects, if you need them directly.
    # ------------------------------------------------------------------

    @property
    def transport(self) -> TPTransport:
        return self._transport

    @property
    def gimbal(self) -> GimbalController:
        return self._gimbal

    @property
    def imu(self) -> AttitudeReader | None:
        return self._imu
