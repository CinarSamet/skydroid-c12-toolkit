"""
Skydroid "#TP" wire protocol.

Skydroid does not publish an official SDK for the C12 (or the related
C10/C11/C13/C14/C20 family). This module reimplements the "#TP" ASCII/UDP
protocol used internally by the official Android FPV app
(com.skydroid.camerafpv), cross-checked against an independent
reverse-engineering write-up for the C12:
https://github.com/ngthanhvinh1996/WebAppControlC12

Facts below were confirmed on a real C12 unit:

  - Transport: UDP. Control port **5000** - not 9002, which is what most
    generic Skydroid documentation assumes and what this project also
    assumed at first. Port 5000 was found by scanning the device and
    getting a real reply (`#tpDU3rVER0105025A`, a version response).
    Your firmware may differ; verify before relying on this.
  - Command format: ASCII string + 2-digit uppercase hex checksum.
  - Checksum: sum of all bytes in the command string, mod 256.
  - PTZ fixed commands: STOP=00 UP=01 DOWN=02 LEFT=03 RIGHT=04 CENTER=05.
    (An earlier, generic table had UP=00 DOWN=01 LEFT=02 RIGHT=03 STOP=04
    MID=11 - that table is wrong for this firmware and was the source of
    several early bugs; see PTZ's docstring below.)
  - GSY: yaw (pan) *speed* command, positive = right, negative = left.
  - GSP: pitch (tilt) *speed* command, positive = up, negative = down.
  - GAY: yaw (pan) *absolute angle* command (degrees * 100 encoding).
    Its raw sign is the OPPOSITE of GSY's: a positive raw GAY value turns
    the gimbal LEFT. `gimbal_pan_absolute_angle_cmd()` corrects this
    internally so the public API is consistent (positive = right,
    matching GSY).
  - GAP: pitch (tilt) *absolute angle* command, same encoding as GAY, but
    its sign matches GSP directly (positive = up) - no correction needed.
    Verified with +5/-5/+10 degree test moves.
  - GAA / GAC: attitude telemetry. `GAA(rate_hz)` tells the camera to
    start pushing `GAC` packets (yaw/pitch/roll) at the given rate.
    Verified live: sending a "tilt to 20 deg" command while GAC streaming
    was on showed `GAC.pitch` climb from 0 to exactly 20.00 in real time,
    matching the commanded value 1:1.
  - A mechanical/physical limit was observed on the tilt axis around
    +-45 degrees.
"""

from __future__ import annotations

import dataclasses
import logging
import socket
import threading
import time

logger = logging.getLogger(__name__)


def checksum(command: str) -> str:
    """Mod-256 hex checksum of a command string's ASCII byte sum."""
    total = sum(ord(c) for c in command) & 0xFF
    return f"{total:02X}"


def build_command(command: str) -> bytes:
    """Append the checksum and encode as UTF-8 bytes ready to send."""
    return (command + checksum(command)).encode("utf-8")


def int2hex(value: int) -> str:
    """
    Signed int -> 2-digit hex byte string (two's complement).
    Used for speed bytes; the protocol only uses the range -99..99.
    """
    if not -99 <= value <= 99:
        raise ValueError(f"int2hex: value out of range: {value}")
    v = value if value >= 0 else value + 256
    return f"{v & 0xFF:02X}"


def short2hex(value: int) -> str:
    """
    Signed int -> 4-digit hex string (16-bit two's complement).
    Used to encode degrees*100 in absolute-angle commands (GAY/GAP),
    i.e. a resolution of 0.01 degree - confirmed on real hardware.
    """
    if not -32768 <= value <= 32767:
        raise ValueError(f"short2hex: value out of range: {value}")
    v = value if value >= 0 else value + 65536
    return f"{v & 0xFFFF:04X}"


class TPTransport:
    """
    Thin UDP transport for sending #TP commands to the C12 and (optionally)
    receiving packets pushed back by the camera (e.g. GAC attitude
    packets). Thread-safe: send() can be called from multiple threads.
    """

    def __init__(
        self,
        camera_ip: str,
        udp_port: int,
        min_interval_sec: float = 0.04,
        bind_source_port: bool = True,
    ):
        self._addr = (camera_ip, udp_port)
        self._min_interval = min_interval_sec
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if bind_source_port:
            # The official app opens its UDP pipeline as
            # createUDPPipeline(9002, host, 9002) - i.e. its *source* port
            # is also 9002. Some firmware versions may only accept control
            # packets from that specific source port, so we bind our local
            # socket to the same port too. If the bind fails (port already
            # in use) we silently fall back to a random source port.
            try:
                self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._sock.bind(("", udp_port))
                logger.info("Local UDP socket bound to source port %d.", udp_port)
            except OSError as exc:
                logger.warning(
                    "Could not bind source port %d (%s) - continuing with "
                    "a random source port.", udp_port, exc,
                )
        self._lock = threading.Lock()
        self._last_send_ts = 0.0
        # Receiver thread - off by default, only starts if start_receiver()
        # is called. The socket timeout lets the recv loop exit cleanly
        # via stop_receiver()/close().
        self._sock.settimeout(0.5)
        self._recv_thread: threading.Thread | None = None
        self._recv_stop = threading.Event()

    def send(self, command: str) -> None:
        """Send one #TP command. The protocol recommends >= 40ms between
        commands; this is enforced here."""
        with self._lock:
            elapsed = time.monotonic() - self._last_send_ts
            if elapsed < self._min_interval:
                time.sleep(self._min_interval - elapsed)
            packet = build_command(command)
            try:
                self._sock.sendto(packet, self._addr)
                logger.debug("TP sent: %s -> %s", packet, self._addr)
            except OSError as exc:
                logger.error("TP send error: %s", exc)
            self._last_send_ts = time.monotonic()

    def start_receiver(self, callback) -> None:
        """
        Start a background thread that listens for UDP packets pushed by
        the camera (e.g. GAC attitude packets). callback(data: bytes) is
        invoked for every packet received.

        The same socket is used for both sending and receiving - normal
        for a connectionless UDP socket.
        """
        if self._recv_thread and self._recv_thread.is_alive():
            logger.warning("Receiver is already running.")
            return
        self._recv_stop.clear()
        self._recv_thread = threading.Thread(
            target=self._recv_loop, args=(callback,), name="TPReceiver", daemon=True
        )
        self._recv_thread.start()

    def stop_receiver(self) -> None:
        self._recv_stop.set()
        if self._recv_thread:
            self._recv_thread.join(timeout=2.0)
        self._recv_thread = None

    def _recv_loop(self, callback) -> None:
        while not self._recv_stop.is_set():
            try:
                data, _addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                # Socket was closed (close()) - exit quietly.
                break
            try:
                callback(data)
            except Exception:
                logger.exception("Telemetry/receiver callback error")

    def close(self) -> None:
        self.stop_receiver()
        self._sock.close()


# ---------------------------------------------------------------------------
# Command constants
# ---------------------------------------------------------------------------

class PTZ:
    """
    CORRECTED (verified on real hardware): the first table used here was
    wrong - copied from generic/unverified Skydroid documentation. This
    unit's firmware uses a different ordering. The values below come from
    an independent, checksum-verified C12-specific reverse-engineering
    project: https://github.com/ngthanhvinh1996/WebAppControlC12

    Old (WRONG) table: UP=00 DOWN=01 LEFT=02 RIGHT=03 STOP=04 MID=11
    New (CORRECT) table: STOP=00 UP=01 DOWN=02 LEFT=03 RIGHT=04 MID=05

    This mismatch exactly explained two early symptoms: sending "LEFT"
    made the gimbal tilt down (old LEFT=02 == new DOWN), and sending
    "STOP" appeared to do nothing (old STOP=04 == new RIGHT, but as a
    tiny fixed step it went unnoticed).

    Codes 06-14 are unverified/unused.
    """
    STOP = "#TPUG2wPTZ00"
    UP = "#TPUG2wPTZ01"
    DOWN = "#TPUG2wPTZ02"
    LEFT = "#TPUG2wPTZ03"
    RIGHT = "#TPUG2wPTZ04"
    BACK_MID = "#TPUG2wPTZ05"  # return to center/home


class Record:
    STOP = "#TPUD2wREC00"
    START = "#TPUD2wREC01"


class Capture:
    TAKE_PHOTO = "#TPUD2wCAP01"


def gimbal_pan_speed_cmd(speed: int) -> str:
    """
    Spin the pan (yaw) axis at a continuous speed.

    speed > 0: right, speed < 0: left, speed == 0: stop.
    Speed range is -99..99. The firmware auto-stops the motor a few tens
    of milliseconds after the last command if it isn't refreshed at
    roughly 20Hz.
    """
    if speed == 0:
        return PTZ.STOP
    return "#TPUG2wGSY" + int2hex(speed)


def gimbal_tilt_speed_cmd(speed: int) -> str:
    """
    Spin the tilt (pitch) axis at a continuous speed.
    speed > 0: up, speed < 0: down, speed == 0: stop.
    """
    if speed == 0:
        return PTZ.STOP
    return "#TPUG2wGSP" + int2hex(speed)


def gimbal_pan_absolute_angle_cmd(degrees: float, speed: int = 30) -> str:
    """
    Send the pan (yaw) axis directly to an absolute target angle.

    VERIFIED ON REAL HARDWARE: the `#TPUG6wGAY` command takes a 4-digit
    hex angle (degrees*100, i.e. 0.01-degree resolution) plus a 2-digit
    hex speed. 10 and 90 degree test moves landed correctly.

    SIGN NOTE: GAY's own raw sign is the OPPOSITE of the GSY speed
    command - a positive raw GAY value turns the gimbal LEFT. This
    function flips the sign internally so the rest of the SDK stays
    consistent (positive = right, same convention as GSY). In other
    words: calling this function with degrees > 0 always turns RIGHT.

    degrees: target absolute angle relative to center/home (-180..180
        recommended).
    speed: movement speed, -99..99 (default 30).

    Much more accurate than dead-reckoning with the speed command: the
    gimbal's own controller drives to the target angle, so no drift
    accumulates. Combine with the IMU (see `telemetry.py`) to also get a
    real arrival confirmation instead of just trusting the command was
    accepted.
    """
    angle_raw = round(-degrees * 100)  # sign flip: correct for GAY's inversion
    speed_clamped = max(-99, min(99, int(speed)))
    return "#TPUG6wGAY" + short2hex(angle_raw) + int2hex(abs(speed_clamped))


def gimbal_tilt_absolute_angle_cmd(degrees: float, speed: int = 30) -> str:
    """
    Send the tilt (pitch) axis directly to an absolute target angle.

    VERIFIED ON REAL HARDWARE: `#TPUG6wGAP` uses the same format as GAY -
    4-digit hex angle (degrees*100) + 2-digit hex speed.

    SIGN NOTE (different from GAY): GAP's sign matches GSP directly -
    positive = UP, negative = DOWN. Unlike GAY, no sign flip is applied
    here; the raw value is sent as-is. Confirmed with:
      +5  -> up, ~5 degrees
      -5  -> down, ~5 degrees (absolute from the previous position, so
             going from -5 to +5 to -10 means a real 10-degree physical
             move - this is expected/correct behaviour)
      +10 -> up, ~10 degrees (scale confirmed)

    A mechanical limit was observed around +-45 degrees on this axis -
    be careful with large target angles.

    degrees: target absolute angle relative to center/home, positive=up.
    speed: movement speed, -99..99 (default 30).
    """
    angle_raw = round(degrees * 100)  # no sign flip - raw value is already correct
    speed_clamped = max(-99, min(99, int(speed)))
    return "#TPUG6wGAP" + short2hex(angle_raw) + int2hex(abs(speed_clamped))


# ---------------------------------------------------------------------------
# GAA/GAC - attitude (yaw/pitch/roll) telemetry
# VERIFIED ON REAL HARDWARE: sending "tilt to 20" while GAC streaming was
# on showed GAC.pitch climb from 0 to 20.00 in real time, matching the
# commanded value via GAP.
# ---------------------------------------------------------------------------

def gimbal_attitude_push_cmd(rate_hz: int) -> str:
    """
    Ask the camera to start (or stop) automatically pushing GAC
    (attitude: yaw/pitch/roll) packets at rate_hz Hz.

    VERIFIED ON REAL HARDWARE: sending GAA(10) made the camera push ~10
    GAC packets/second, with correct, real-time values.

    rate_hz: 0-100, how many GAC packets per second are requested
        (0 = stop).
    """
    rate_clamped = max(0, min(100, int(rate_hz)))
    return "#TPUG2wGAA" + f"{rate_clamped:02X}"


@dataclasses.dataclass
class AttitudeSample:
    """Yaw/pitch/roll (degrees) parsed from one GAC packet."""
    yaw_deg: float
    pitch_deg: float
    roll_deg: float


def _signed_hex16(hex_str: str) -> int:
    """4-digit hex string -> signed 16-bit int (two's complement)."""
    value = int(hex_str, 16)
    if value >= 0x8000:
        value -= 0x10000
    return value


def parse_gac_packet(data: bytes) -> AttitudeSample | None:
    """
    Try to parse a raw UDP packet as a GAC (attitude) packet. Returns
    None if it isn't one, or can't be parsed.

    FORMAT VERIFIED ON REAL HARDWARE: right after the "GAC" marker come
    three 4-hex-digit signed int16 values (yaw, pitch, roll), each in the
    same degrees*100 encoding as GAY/GAP but read the other way. The
    pitch channel was numerically verified (0 -> 20.00 degrees, matching
    a commanded goto_tilt exactly). Yaw/roll are parsed with the same
    format but weren't separately calibrated (yaw showed a small,
    roughly constant offset of about -0.7 to -0.9 degrees at rest).
    """
    try:
        text = data.decode("ascii", errors="ignore")
    except Exception:
        return None
    idx = text.find("GAC")
    if idx < 0:
        return None
    payload = text[idx + 3:]
    if len(payload) < 12:
        return None
    try:
        yaw_raw = _signed_hex16(payload[0:4])
        pitch_raw = _signed_hex16(payload[4:8])
        roll_raw = _signed_hex16(payload[8:12])
    except ValueError:
        return None
    return AttitudeSample(yaw_raw / 100.0, pitch_raw / 100.0, roll_raw / 100.0)
