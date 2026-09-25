"""
Shared pytest fixtures - most importantly a small, self-contained mock
UDP "camera" that understands enough of the #TP protocol (GAY/GAP/GAA)
to let the tests exercise real network round-trips (TPTransport,
AttitudeReader, GimbalController) without any real hardware.

It is intentionally simple: it does not validate checksums or try to be
a faithful firmware emulator, it just tracks a pan/tilt angle, converges
towards absolute-angle targets over a few iterations, and streams GAC
packets back when attitude push is enabled - enough to prove the SDK's
own protocol encoding/decoding and control-loop logic are correct.
"""

from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass

import pytest


@dataclass
class _MockState:
    pan_deg: float = 0.0
    tilt_deg: float = 0.0
    gac_on: bool = False
    gac_rate_hz: int = 10


class MockCamera:
    """A minimal fake C12 that speaks just enough #TP to be useful in tests."""

    def __init__(self):
        self.state = _MockState()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(0.2)
        self.port = self._sock.getsockname()[1]
        self._client_addr = None
        self._running = True
        self._threads = [
            threading.Thread(target=self._recv_loop, daemon=True),
            threading.Thread(target=self._gac_loop, daemon=True),
        ]
        for t in self._threads:
            t.start()

    # -- command handling --------------------------------------------------

    def _recv_loop(self) -> None:
        while self._running:
            try:
                data, addr = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            self._client_addr = addr
            self._handle(data.decode("ascii", errors="ignore"))

    def _handle(self, text: str) -> None:
        if "GAY" in text:
            idx = text.index("GAY")
            raw = _signed16(text[idx + 3:idx + 7])
            target_pan = -(raw / 100.0)  # GAY's raw sign is inverted, same as real hw
            threading.Thread(target=self._converge, args=("pan", target_pan), daemon=True).start()
        elif "GAP" in text:
            idx = text.index("GAP")
            raw = _signed16(text[idx + 3:idx + 7])
            target_tilt = raw / 100.0  # GAP is not inverted
            threading.Thread(target=self._converge, args=("tilt", target_tilt), daemon=True).start()
        elif "GAA" in text:
            idx = text.index("GAA")
            rate = int(text[idx + 3:idx + 5], 16)
            self.state.gac_rate_hz = rate if rate > 0 else self.state.gac_rate_hz
            self.state.gac_on = rate > 0
        elif "PTZ05" in text:  # BACK_MID / center
            self.state.pan_deg = 0.0
            self.state.tilt_deg = 0.0

    def _converge(self, axis: str, target: float) -> None:
        while self._running:
            current = self.state.pan_deg if axis == "pan" else self.state.tilt_deg
            if abs(current - target) <= 0.05:
                break
            step = max(-3.0, min(3.0, target - current))
            if axis == "pan":
                self.state.pan_deg += step
            else:
                self.state.tilt_deg += step
            time.sleep(0.02)
        if axis == "pan":
            self.state.pan_deg = target
        else:
            self.state.tilt_deg = target

    # -- telemetry push -------------------------------------------------

    def _gac_loop(self) -> None:
        while self._running:
            if self.state.gac_on and self._client_addr:
                yaw_raw = _to_u16(round(self.state.pan_deg * 100))
                pitch_raw = _to_u16(round(self.state.tilt_deg * 100))
                roll_raw = 0
                payload = f"GAC{yaw_raw:04X}{pitch_raw:04X}{roll_raw:04X}"
                packet = f"#tp{payload}00".encode("ascii")
                try:
                    self._sock.sendto(packet, self._client_addr)
                except OSError:
                    pass
                time.sleep(1.0 / max(1, self.state.gac_rate_hz))
            else:
                time.sleep(0.05)

    def close(self) -> None:
        self._running = False
        self._sock.close()


def _signed16(hex_str: str) -> int:
    value = int(hex_str, 16)
    if value >= 0x8000:
        value -= 0x10000
    return value


def _to_u16(value: int) -> int:
    return value & 0xFFFF


@pytest.fixture
def mock_camera():
    cam = MockCamera()
    yield cam
    cam.close()
