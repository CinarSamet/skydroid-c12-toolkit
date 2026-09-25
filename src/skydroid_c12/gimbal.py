"""
High-level gimbal control API.

Wraps the raw #TP commands from `protocol.py` into meaningful calls like
`goto_pan_angle`, `move_right`, `center`, `stop`.

Two pan (horizontal) control methods are available:
  1. goto_pan_angle() - ABSOLUTE angle command (GAY). Verified on real
     hardware, precise and drift-free. The RECOMMENDED default.
  2. pan_at_speed() - continuous speed command (GSY), needs time-based
     estimation. Still useful for a continuous sweep, but goto_pan_angle()
     is usually the better choice.

Tilt has the same two options: goto_tilt_angle() (GAP, absolute, verified,
recommended) and tilt_at_speed() / move_up_by_duration() /
move_down_by_duration() (speed/duration-based, kept for cases where a
continuous motion is actually what you want).

IMU (GAC) closed-loop feedback
-------------------------------
GimbalController can optionally take an `AttitudeReader` (telemetry.py).
When given, movement switches from OPEN LOOP (time estimate) to CLOSED
LOOP (real measurement):

  1. Real arrival confirmation: goto_pan_angle/goto_tilt_angle wait for
     the IMU-reported angle to reach the target (within tolerance)
     instead of sleeping for a fixed estimated duration. The time
     estimate becomes only a SAFETY upper bound (timeout) - if the IMU
     never responds or its data goes stale, control falls back to the
     old fixed-sleep behaviour.
  2. Drift never accumulates: known_pan_angle/known_tilt_angle read the
     IMU's current measurement directly whenever it's fresh, instead of
     relying on command history. In a system running for hours, the
     error that would normally build up in a pure command-history
     estimate simply doesn't - every read is a real measurement.
  3. External interference becomes visible: a natural consequence of #2
     - if someone bumps or manually moves the gimbal, the next
     known_pan_angle/known_tilt_angle read (even without us sending any
     command) reflects that. An open-loop system would never see this.

If no AttitudeReader is given (attitude_reader=None, the default), the
controller behaves exactly like the old open-loop system - nothing
breaks, this is fully backward compatible.
"""

from __future__ import annotations

import logging
import time

from .protocol import (
    TPTransport,
    PTZ,
    Capture,
    gimbal_pan_speed_cmd,
    gimbal_tilt_speed_cmd,
    gimbal_pan_absolute_angle_cmd,
    gimbal_tilt_absolute_angle_cmd,
)
from .telemetry import AttitudeReader

logger = logging.getLogger(__name__)

# Calibrated for the pan axis only (measured on real hardware: 11.86
# deg/sec at speed=25). Can be overridden per-instance in the constructor.
DEFAULT_PAN_DEG_PER_SEC_AT_SPEED_25 = 11.86
DEFAULT_CALIBRATION_SPEED = 25
# SAFETY LIMIT (kept in sync with config.GimbalConfig's defaults so
# GimbalController is safe to use standalone, without importing config).
# A physical limit was observed around +-45 degrees on real hardware.
DEFAULT_MAX_TILT_DEG = 45.0
DEFAULT_MIN_TILT_DEG = -45.0


class GimbalController:
    def __init__(
        self,
        transport: TPTransport,
        pan_deg_per_sec_at_calib_speed: float = DEFAULT_PAN_DEG_PER_SEC_AT_SPEED_25,
        calibration_speed: int = DEFAULT_CALIBRATION_SPEED,
        max_tilt_deg: float = DEFAULT_MAX_TILT_DEG,
        min_tilt_deg: float = DEFAULT_MIN_TILT_DEG,
        attitude_reader: AttitudeReader | None = None,
        attitude_tolerance_deg: float = 0.5,
        attitude_timeout_multiplier: float = 2.5,
    ):
        """
        attitude_reader: if given (a telemetry.AttitudeReader), IMU
            feedback is used for CLOSED-LOOP control (see module
            docstring). None (default) keeps the old OPEN-LOOP (time
            estimate) behaviour unchanged.
        attitude_tolerance_deg: how close (in degrees) the IMU reading
            must be to the target to count as "arrived".
        attitude_timeout_multiplier: max wait for closed-loop arrival =
            time-estimated duration * this multiplier. If the IMU
            doesn't converge within that time (e.g. data drops out),
            control falls back to the open-loop behaviour (sleep for the
            fixed duration and continue) - it never waits forever.
        """
        self._max_tilt_deg = max_tilt_deg
        self._min_tilt_deg = min_tilt_deg
        self._t = transport
        self._current_pan_speed = 0
        self._current_tilt_speed = 0
        # Known/estimated absolute pan position (degrees, relative to
        # center). WITHOUT attitude_reader (or with stale data), this is
        # an ESTIMATE based on goto_angle() calls (not a real encoder).
        # WITH attitude_reader and fresh data, the known_pan_angle
        # property returns the direct IMU measurement instead (see the
        # property definitions below) - this internal variable then only
        # matters as a fallback if the IMU briefly drops out.
        self._known_pan_angle = 0.0
        self._pan_deg_per_sec = pan_deg_per_sec_at_calib_speed
        self._calib_speed = calibration_speed
        # Known absolute tilt angle - only meaningful once goto_tilt_angle
        # has been successfully used. Duration-based move_up_by_duration/
        # move_down_by_duration do NOT update this (the real arrival
        # angle isn't known, only an estimated duration was run).
        self._known_tilt_angle: float | None = None

        self._attitude_reader = attitude_reader
        self._attitude_tolerance_deg = attitude_tolerance_deg
        self._attitude_timeout_multiplier = attitude_timeout_multiplier

    # ------------------------------------------------------------------
    # Absolute-angle control (RECOMMENDED - precise, verified on real hw)
    # ------------------------------------------------------------------

    def goto_pan_angle(self, degrees: float, speed: int = 30, wait: bool = True) -> None:
        """
        Send the pan axis directly to an absolute angle (relative to
        center, positive=right, negative=left). A single command - no
        continuous stream needed.

        If wait=True AND an AttitudeReader was given: waits for the IMU
        to confirm the target angle was actually reached (within
        tolerance) - CLOSED LOOP. Without an AttitudeReader, or with
        stale data: sleeps for a time estimate based on the calibrated
        speed - OPEN LOOP, the old behaviour.
        """
        current_before = self.known_pan_angle
        cmd = gimbal_pan_absolute_angle_cmd(degrees, speed)
        self._t.send(cmd)
        self._current_pan_speed = speed if degrees > current_before else -speed

        if wait:
            distance = abs(degrees - current_before)
            effective_deg_per_sec = self._pan_deg_per_sec * (abs(speed) / self._calib_speed)
            est_duration = distance / effective_deg_per_sec if effective_deg_per_sec > 0 else 0.0

            arrived = self._wait_for_arrival(axis="pan", target_deg=degrees, est_duration=est_duration)
            if not arrived:
                # Open-loop fallback: no IMU / stale / timed out - sleep
                # for the estimated duration and continue (old, safe
                # behaviour).
                time.sleep(est_duration + 0.3)

        self._known_pan_angle = degrees
        self._current_pan_speed = 0

    def _wait_for_arrival(self, axis: str, target_deg: float, est_duration: float) -> bool:
        """
        CLOSED-LOOP helper: if an attitude_reader is set and producing
        fresh data, waits for that axis' IMU reading to reach the target
        (within attitude_tolerance_deg).

        Returns True on success (target reached in time). Returns False
        if there's no attitude_reader, no fresh data is coming in, or the
        target wasn't reached within the timeout - the caller
        (goto_pan_angle/goto_tilt_angle) then falls back to open-loop
        (fixed sleep) behaviour.
        """
        if self._attitude_reader is None:
            return False

        timeout = max(est_duration * self._attitude_timeout_multiplier, 1.0) + 0.5
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if self._attitude_reader.is_fresh():
                sample = self._attitude_reader.get_latest()
                if sample is not None:
                    current = sample.yaw_deg if axis == "pan" else sample.pitch_deg
                    if abs(current - target_deg) <= self._attitude_tolerance_deg:
                        logger.debug(
                            "%s axis reached target via IMU: target=%.2f measured=%.2f (%.2fs)",
                            axis, target_deg, current, time.monotonic() - start,
                        )
                        return True
            else:
                # IMU link missing/dropped - don't wait pointlessly, fall
                # back to open loop right away.
                logger.debug("%s axis: IMU data not fresh, falling back to open loop.", axis)
                return False
            time.sleep(0.03)

        logger.warning(
            "%s axis: IMU did not reach target (%.2f, tolerance=%.2f) within %ss - "
            "falling back to open loop.",
            axis, target_deg, self._attitude_tolerance_deg, f"{timeout:.1f}",
        )
        return False

    def goto_pan_tilt(
        self,
        pan_degrees: float,
        tilt_degrees: float,
        speed: int = 30,
        wait: bool = True,
        pan_speed: int | None = None,
        tilt_speed: int | None = None,
    ) -> None:
        """
        Send pan AND tilt to absolute target angles at the same time, in
        one call (e.g. "30 right, 10 up").

        The two commands (GAY and GAP) are sent back-to-back, separated
        only by TPTransport's own min_interval_sec delay - the gimbal
        drives both axes independently, so in practice they move nearly
        simultaneously (neither waits for the other to finish).

        With wait=True, waits for whichever axis takes LONGER (pan and
        tilt distances/speeds can differ).

        pan_degrees: absolute pan target (positive=right, negative=left)
        tilt_degrees: absolute tilt target (positive=up, negative=down)
        speed: shared speed for BOTH axes (-99..99), used unless
            pan_speed/tilt_speed are given.
        pan_speed: speed for pan only (overrides speed for that axis).
        tilt_speed: speed for tilt only (overrides speed for that axis).
        """
        self._check_tilt_limit(tilt_degrees)
        current_pan = self.known_pan_angle
        current_tilt_raw = self.known_tilt_angle
        current_tilt = current_tilt_raw if current_tilt_raw is not None else 0.0
        eff_pan_speed = pan_speed if pan_speed is not None else speed
        eff_tilt_speed = tilt_speed if tilt_speed is not None else speed

        pan_cmd = gimbal_pan_absolute_angle_cmd(pan_degrees, eff_pan_speed)
        tilt_cmd = gimbal_tilt_absolute_angle_cmd(tilt_degrees, eff_tilt_speed)
        self._t.send(pan_cmd)
        self._t.send(tilt_cmd)

        if wait:
            pan_distance = abs(pan_degrees - current_pan)
            tilt_distance = abs(tilt_degrees - current_tilt)
            pan_deg_per_sec = self._pan_deg_per_sec * (abs(eff_pan_speed) / self._calib_speed)
            tilt_deg_per_sec = self._pan_deg_per_sec * (abs(eff_tilt_speed) / self._calib_speed)
            pan_duration = pan_distance / pan_deg_per_sec if pan_deg_per_sec > 0 else 0.0
            tilt_duration = tilt_distance / tilt_deg_per_sec if tilt_deg_per_sec > 0 else 0.0
            est_duration = max(pan_duration, tilt_duration)

            arrived = self._wait_for_pan_tilt_arrival(pan_degrees, tilt_degrees, est_duration)
            if not arrived and est_duration > 0:
                time.sleep(est_duration + 0.3)

        self._known_pan_angle = pan_degrees
        self._known_tilt_angle = tilt_degrees

    def _wait_for_pan_tilt_arrival(
        self, pan_target: float, tilt_target: float, est_duration: float
    ) -> bool:
        """goto_pan_tilt's helper: waits until BOTH pan and tilt IMU
        readings reach their own targets (within tolerance). See
        _wait_for_arrival."""
        if self._attitude_reader is None:
            return False

        timeout = max(est_duration * self._attitude_timeout_multiplier, 1.0) + 0.5
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            if self._attitude_reader.is_fresh():
                sample = self._attitude_reader.get_latest()
                if sample is not None:
                    pan_ok = abs(sample.yaw_deg - pan_target) <= self._attitude_tolerance_deg
                    tilt_ok = abs(sample.pitch_deg - tilt_target) <= self._attitude_tolerance_deg
                    if pan_ok and tilt_ok:
                        return True
            else:
                return False
            time.sleep(0.03)

        logger.warning(
            "pan+tilt: IMU did not reach target on both axes within %.1fs - "
            "falling back to open loop.", timeout,
        )
        return False

    def move_right(self, degrees: float, speed: int = 30, wait: bool = True) -> None:
        """Turn N degrees right, relative to the current known (real, if
        IMU is on) position."""
        self.goto_pan_angle(self.known_pan_angle + abs(degrees), speed, wait)

    def move_left(self, degrees: float, speed: int = 30, wait: bool = True) -> None:
        """Turn N degrees left, relative to the current known (real, if
        IMU is on) position."""
        self.goto_pan_angle(self.known_pan_angle - abs(degrees), speed, wait)

    def center(self, wait: bool = True) -> None:
        """Return the gimbal to its mechanical/logical center (home)."""
        self._t.send(PTZ.BACK_MID)
        self._current_pan_speed = 0
        self._current_tilt_speed = 0
        self._known_pan_angle = 0.0
        self._known_tilt_angle = 0.0
        if wait:
            time.sleep(2.0)

    # Kept for backward compatibility with older code that used this name.
    def recenter(self, wait: bool = True) -> None:
        self.center(wait=wait)

    # ------------------------------------------------------------------
    # Speed-based control (needs a continuous ~20Hz stream to keep moving)
    # ------------------------------------------------------------------

    def pan_at_speed(self, speed: int) -> None:
        """speed: -99..99. Positive=right, negative=left. The firmware
        auto-stops within tens of milliseconds if not refreshed at
        ~20Hz."""
        speed = max(-99, min(99, int(speed)))
        self._t.send(gimbal_pan_speed_cmd(speed))
        self._current_pan_speed = speed

    def tilt_at_speed(self, speed: int) -> None:
        """speed: -99..99. Positive=up, negative=down."""
        speed = max(-99, min(99, int(speed)))
        self._t.send(gimbal_tilt_speed_cmd(speed))
        self._current_tilt_speed = speed

    # ------------------------------------------------------------------
    # Tilt - absolute angle
    # ------------------------------------------------------------------

    def goto_tilt_angle(self, degrees: float, speed: int = 25, wait: bool = True) -> None:
        """
        VERIFIED ON REAL HARDWARE: sends the tilt axis to an absolute
        angle (GAP command). Same call shape as goto_pan_angle() - a
        single command, no continuous stream needed.

        Confirmed behaviour: positive degrees = UP, negative = DOWN
        (same direction as GSP - unlike GAY, no sign flip here).
        +5, -5 and +10 test moves all landed consistently.

        A mechanical limit was observed around +-45 degrees on this axis
        - be careful with large target angles.

        wait=True uses the pan calibration (pan_deg_per_sec) as a rough
        estimate for the open-loop timing - tilt wasn't calibrated
        separately, so the *wait duration* may be a bit off (early/late),
        but since the command itself is absolute this only affects how
        long wait() blocks, not the angle actually reached.
        """
        self._check_tilt_limit(degrees)
        current_before = self.known_tilt_angle
        start_angle = current_before if current_before is not None else 0.0
        cmd = gimbal_tilt_absolute_angle_cmd(degrees, speed)
        self._t.send(cmd)

        if wait:
            distance = abs(degrees - start_angle)
            effective_deg_per_sec = self._pan_deg_per_sec * (abs(speed) / self._calib_speed)
            est_duration = distance / effective_deg_per_sec if effective_deg_per_sec > 0 else 0.0

            arrived = self._wait_for_arrival(axis="tilt", target_deg=degrees, est_duration=est_duration)
            if not arrived:
                time.sleep(est_duration + 0.3)

        self._known_tilt_angle = degrees

    def _check_tilt_limit(self, degrees: float) -> None:
        """SAFETY: rejects a tilt target outside the observed
        mechanical-limit range BEFORE sending any command (ValueError)."""
        if not (self._min_tilt_deg <= degrees <= self._max_tilt_deg):
            raise ValueError(
                f"Tilt target ({degrees} deg) is outside the safety range "
                f"[{self._min_tilt_deg}, {self._max_tilt_deg}]. A mechanical "
                f"limit was observed around +-45 degrees on real hardware. "
                f"If you intentionally want to exceed this, construct "
                f"GimbalController(..., max_tilt_deg=..., min_tilt_deg=...)."
            )

    def move_tilt_up(self, degrees: float, speed: int = 25, wait: bool = True) -> None:
        """Turn N degrees up, relative to the current known (real, if IMU
        is on) tilt position (uses goto_tilt_angle - verified, see above)."""
        current = self.known_tilt_angle
        base = current if current is not None else 0.0
        self.goto_tilt_angle(base + abs(degrees), speed, wait)

    def move_tilt_down(self, degrees: float, speed: int = 25, wait: bool = True) -> None:
        """Turn N degrees down, relative to the current known (real, if
        IMU is on) tilt position (uses goto_tilt_angle - verified, see
        above)."""
        current = self.known_tilt_angle
        base = current if current is not None else 0.0
        self.goto_tilt_angle(base - abs(degrees), speed, wait)

    # ------------------------------------------------------------------
    # Tilt - duration-based (verified, works - but no degree guarantee.
    # Names deliberately end in "by_duration" so anyone using them knows
    # it's time-based, not angle-based.)
    # ------------------------------------------------------------------

    def move_up_by_duration(self, seconds: float, speed: int = 25) -> None:
        """
        DURATION-based (NOT degree-based). Prefer goto_tilt_angle() /
        move_tilt_up() unless you specifically want a continuous,
        time-limited motion. A physical limit was observed around ~45
        degrees - be careful with large duration values.
        """
        self._move_tilt_for_duration(abs(speed), seconds)

    def move_down_by_duration(self, seconds: float, speed: int = 25) -> None:
        """DURATION-based (NOT degree-based). See move_up_by_duration()."""
        self._move_tilt_for_duration(-abs(speed), seconds)

    def _move_tilt_for_duration(self, signed_speed: int, seconds: float) -> None:
        start = time.monotonic()
        step = 0.05  # 20Hz
        while time.monotonic() - start < seconds:
            self.tilt_at_speed(signed_speed)
            time.sleep(step)
        self.tilt_at_speed(0)
        # After a duration-based move, the real angle reached is unknown -
        # mark known_tilt_angle as "unknown" so it doesn't get confused
        # with a goto_tilt_angle-based estimate.
        self._known_tilt_angle = None

    def stop(self) -> None:
        """Stop both axes immediately."""
        self._t.send(gimbal_pan_speed_cmd(0))
        self._t.send(gimbal_tilt_speed_cmd(0))
        self._current_pan_speed = 0
        self._current_tilt_speed = 0

    def take_photo(self) -> None:
        self._t.send(Capture.TAKE_PHOTO)

    @property
    def current_speed(self) -> int:
        """Backward compatibility: older code used this name (pan speed)."""
        return self._current_pan_speed

    @property
    def known_pan_angle(self) -> float:
        """
        Known absolute pan position (degrees).

        If attitude_reader IS SET and data is FRESH: returns the IMU's
        (GAC) current measurement DIRECTLY - the real position, no drift
        accumulates, and it reflects external interference (even without
        us sending any command).

        If attitude_reader isn't set, or data is stale (IMU link may be
        down): falls back to the old behaviour - an ESTIMATE based on the
        last goto_pan_angle/move_right/move_left/center call.
        """
        if self._attitude_reader is not None and self._attitude_reader.is_fresh():
            sample = self._attitude_reader.get_latest()
            if sample is not None:
                return sample.yaw_deg
        return self._known_pan_angle

    @property
    def known_tilt_angle(self) -> float | None:
        """
        Known absolute tilt position (degrees) - same logic as
        known_pan_angle: if attitude_reader is set and fresh, returns the
        REAL IMU (GAC pitch) measurement; otherwise a command-history
        ESTIMATE (which becomes None after a duration-based
        move_up_by_duration/move_down_by_duration call, since the real
        arrival angle was never known - but if the IMU is on, this can
        still return the REAL angle even then).
        """
        if self._attitude_reader is not None and self._attitude_reader.is_fresh():
            sample = self._attitude_reader.get_latest()
            if sample is not None:
                return sample.pitch_deg
        return self._known_tilt_angle
