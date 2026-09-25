"""
Default configuration values.

Every value here was measured or observed on a real C12 unit (see the
README's "Verified protocol notes" section). Your unit's firmware may
differ - override what you need via the dataclass constructors or by
passing explicit arguments to `Camera(...)`.
"""

from __future__ import annotations

import dataclasses


@dataclasses.dataclass
class NetworkConfig:
    #: Default C12 IP on its Ethernet subnet (192.168.144.x).
    camera_ip: str = "192.168.144.108"
    #: Control UDP port. NOTE: widely-cited docs assume 9002, but the
    #: unit this SDK was developed against only answered on port 5000
    #: (confirmed via a live UDP port scan). Verify on your own unit
    #: before relying on this default.
    tp_udp_port: int = 5000


@dataclasses.dataclass
class GimbalConfig:
    #: Default pan/tilt speed for absolute-angle commands (-99..99).
    default_speed: int = 25
    #: Measured degrees/second at default_speed=25 (see README
    #: "Calibration"). Only used to size the open-loop fallback wait
    #: and the closed-loop timeout - it does not affect accuracy of
    #: absolute-angle commands themselves.
    deg_per_sec_at_default_speed: float = 11.86
    #: Minimum delay between two consecutive UDP commands (protocol
    #: docs suggest >= 40ms).
    min_command_interval_sec: float = 0.05
    #: SAFETY LIMIT: a mechanical/physical limit was observed around
    #: +-45 degrees on the tilt axis. goto_tilt() rejects targets
    #: outside this range with a ValueError. Adjust if your unit
    #: differs, testing carefully in small steps first.
    max_tilt_deg: float = 45.0
    min_tilt_deg: float = -45.0
    #: IMU (GAC) closed-loop feedback - verified on real hardware. When
    #: True, absolute-angle moves wait for the IMU to confirm arrival
    #: instead of sleeping for an estimated duration.
    enable_attitude_feedback: bool = True
    #: Degrees of tolerance for "target reached" when using the IMU.
    attitude_tolerance_deg: float = 0.5
    #: Closed-loop max wait = estimated open-loop duration * this
    #: multiplier. If the IMU doesn't converge within that time, the
    #: gimbal falls back to the open-loop (fixed sleep) behaviour
    #: instead of waiting forever.
    attitude_timeout_multiplier: float = 2.5
    #: Requested GAC push rate (Hz) when the IMU reader is started.
    attitude_gac_rate_hz: int = 10


@dataclasses.dataclass
class AppConfig:
    network: NetworkConfig = dataclasses.field(default_factory=NetworkConfig)
    gimbal: GimbalConfig = dataclasses.field(default_factory=GimbalConfig)


#: Ready-to-use default instance. Copy it (`dataclasses.replace(...)`) or
#: build your own `AppConfig(...)` rather than mutating this one if you
#: need different settings in different parts of your program.
DEFAULT_CONFIG = AppConfig()
