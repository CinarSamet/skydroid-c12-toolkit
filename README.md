# skydroid-c12-toolkit

[![CI](https://github.com/CinarSamet/skydroid-c12-toolkit/actions/workflows/ci.yml/badge.svg)](https://github.com/CinarSamet/skydroid-c12-toolkit/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](pyproject.toml)

A reverse-engineered Python SDK for the **Skydroid C12** gimbal camera:
precise pan/tilt movement and real-time IMU (yaw/pitch/roll) telemetry
over its undocumented UDP protocol - no official SDK required, because
there isn't one.

```python
from skydroid_c12 import Camera

with Camera("192.168.144.108") as cam:
    cam.center()
    cam.goto_pan(30)          # absolute angle, closed-loop confirmed by the IMU
    cam.goto_tilt(10)

    sample = cam.get_attitude()
    print(sample.yaw_deg, sample.pitch_deg, sample.roll_deg)
```

Or from a terminal, no code required:

```bash
skydroid-c12 192.168.144.108 goto 30
skydroid-c12 192.168.144.108 watch 10
```

[Türkçe rehber için buraya bakın / Turkish guide here](docs/README.tr.md).

## Why this exists

Skydroid does not publish an SDK, an API reference, or even a full
command table for the C12 (or the related C10/C11/C13/C14/C20 family).
The only "documentation" is the official Android FPV app's compiled APK.
This project reimplements what that app actually sends on the wire - the
`#TP` UDP text protocol - **from scratch, verified against real
hardware**, not copied from an assumption.

Along the way, several widely-repeated but *wrong* assumptions about
this protocol were found and corrected (see "Verified protocol notes"
below) - if you've been fighting a Skydroid gimbal that moves the wrong
way or ignores your STOP command, this is probably why.

## Features

- **Absolute-angle movement** (`goto_pan`, `goto_tilt`, `goto`) - single
  command, the gimbal's own controller drives to the target, no drift
  accumulates.
- **Speed-based movement** (continuous pan/tilt at a given speed) for
  when a smooth sweep is what you actually want.
- **Real IMU telemetry** (`get_attitude`) - live yaw/pitch/roll, pushed
  directly by the camera.
- **Closed-loop control** - when the IMU is enabled, movement commands
  wait for the camera to actually *report* arrival at the target angle
  instead of just sleeping for an estimated duration; falls back to a
  time-based wait automatically if the IMU is unavailable or the reading
  goes stale.
- **A safety limit on tilt** (configurable), because this gimbal has a
  real mechanical limit around 45 degrees and will happily grind against
  it if you let it.
- **A CLI** (`skydroid-c12`) for driving the gimbal and reading the IMU
  without writing any code.
- **Zero runtime dependencies** - pure Python 3.8+ standard library
  (`socket`, `threading`, `dataclasses`).

## Installation

```bash
git clone https://github.com/CinarSamet/skydroid-c12-toolkit.git
cd skydroid-c12-toolkit
pip install .
# or, for local development:
pip install -e ".[dev]"
```

Your computer needs to be on the same subnet as the camera. The C12
defaults to `192.168.144.x` over its Ethernet port:

```bash
sudo ip addr add 192.168.144.10/24 dev eth0
ping 192.168.144.108   # default camera IP - yours may differ
```

## Quick start (Python)

```python
from skydroid_c12 import Camera

with Camera("192.168.144.108") as cam:
    cam.center()

    cam.goto_pan(30)                 # absolute angle, positive = right
    cam.goto_tilt(10)                # absolute angle, positive = up
    cam.goto(pan=-20, tilt=5)        # both axes at once

    cam.move_right(10)               # relative to current position
    cam.move_tilt_down(5)

    sample = cam.get_attitude()      # None until the first packet arrives
    if sample:
        print(sample.yaw_deg, sample.pitch_deg, sample.roll_deg)

    print(cam.pan_deg, cam.tilt_deg)  # known position (real IMU reading if fresh)

    cam.stop_now()
```

`Camera` is a convenience facade over three lower-level pieces you can
also use directly if you need finer control: `TPTransport` (raw UDP
send/receive), `AttitudeReader` (IMU polling), and `GimbalController`
(movement logic). See `skydroid_c12/camera.py` for how they're wired
together.

One-shot IMU read, no persistent connection needed:

```python
from skydroid_c12 import get_attitude_once

sample = get_attitude_once("192.168.144.108")
print(sample.yaw_deg, sample.pitch_deg, sample.roll_deg)
```

## Command-line usage

```bash
skydroid-c12 192.168.144.108 right 30      # relative, 30 deg right
skydroid-c12 192.168.144.108 left 20       # relative, 20 deg left
skydroid-c12 192.168.144.108 tilt 10       # absolute tilt, up
skydroid-c12 192.168.144.108 tilt -10      # absolute tilt, down
skydroid-c12 192.168.144.108 goto 45       # absolute pan
skydroid-c12 192.168.144.108 point 30 10   # pan AND tilt, together
skydroid-c12 192.168.144.108 center
skydroid-c12 192.168.144.108 stop

skydroid-c12 192.168.144.108 imu           # one-shot IMU read
skydroid-c12 192.168.144.108 watch 10      # stream IMU readings for 10s
```

Turkish command aliases (`sag`, `sol`, `egim`, `nokta`, `merkez`, `dur`,
`izle`) work too - this project started life for a Turkish-speaking
team. Full option list: `skydroid-c12 --help`.

## Closed-loop vs. open-loop movement

Sending an absolute-angle command doesn't tell you whether the gimbal
actually got there. With the IMU enabled (the default), this SDK closes
that loop:

1. **Real arrival confirmation** - `goto_pan`/`goto_tilt`/`goto` wait for
   the IMU-reported angle to reach the target (within tolerance) instead
   of a fixed sleep.
2. **No drift accumulation** - `cam.pan_deg` / `cam.tilt_deg` read the
   IMU directly when fresh, so error never builds up over a long-running
   session the way a pure command-history estimate would.
3. **External interference becomes visible** - if someone bumps the
   gimbal, the next position read reflects that, even without sending
   any command.

If the IMU never responds, or its data goes stale mid-move, control
falls back automatically to a time-estimated open-loop wait - it never
blocks forever. Pass `enable_imu=False` to `Camera(...)` to disable this
entirely and only get the original open-loop behaviour.

## Verified protocol notes

Everything below was confirmed on a real C12 unit, not assumed from
generic documentation:

| Item | Finding |
|---|---|
| Control port | **UDP 5000** - not 9002, which most generic Skydroid docs (and this project, at first) assume. Found via a live port scan; verify on your own unit. |
| Checksum | ASCII byte sum of the command string, mod 256, as 2-digit hex. |
| PTZ table | `STOP=00 UP=01 DOWN=02 LEFT=03 RIGHT=04 CENTER=05`. An earlier, generic table (`UP=00 DOWN=01 LEFT=02 RIGHT=03 STOP=04`) is **wrong** for this firmware and explains a specific bug: sending "LEFT" tilted the gimbal down instead. |
| `GSY` / `GSP` | Speed-based pan/tilt. Positive GSY = right, positive GSP = up. |
| `GAY` | Absolute pan angle (degrees × 100). Its raw sign is the **opposite** of GSY - a positive raw value turns the gimbal left. This SDK corrects the sign internally so the public API stays consistent. |
| `GAP` | Absolute tilt angle, same encoding as GAY, but its sign matches GSP directly - no correction needed. |
| `GAA` / `GAC` | Attitude telemetry. `GAA(rate_hz)` starts the camera pushing `GAC` packets. Verified live: commanding a tilt to 20° made `GAC.pitch` climb from 0 to exactly 20.00 in real time. |
| Tilt limit | A mechanical/physical limit was observed around ±45°, enforced in software as a configurable safety check. |

Source cross-referenced against an independent, checksum-verified C12
reverse-engineering project:
[ngthanhvinh1996/WebAppControlC12](https://github.com/ngthanhvinh1996/WebAppControlC12).

**Your firmware may differ.** This SDK was developed and tested against
one physical C12 unit; Skydroid does not guarantee protocol stability
across firmware versions. Test carefully, in small steps, before relying
on this in any unattended or safety-relevant setup.

## Testing

```bash
pip install -e ".[dev]"
pytest -v
```

Tests run entirely against a small in-process mock UDP camera
(`tests/conftest.py`) - no real hardware needed to run the suite.

## Project layout

```
src/skydroid_c12/
├── protocol.py    # raw #TP command encoding/decoding, checksum, UDP transport
├── telemetry.py   # AttitudeReader - background IMU (GAC) listener
├── gimbal.py       # GimbalController - movement logic, open/closed loop
├── camera.py       # Camera - single-object facade over the above
├── cli.py           # `skydroid-c12` command-line tool
└── config.py        # default configuration values
tests/                # pytest suite + mock UDP camera fixture
examples/             # standalone usage examples
```

## Contributing

Issues and pull requests are welcome - especially reports of what does
or doesn't hold on other C12 units, firmware versions, or the wider
C10/C11/C13/C14/C20 family. Please include your firmware version and
exactly what you observed.

## License

MIT - see [LICENSE](LICENSE).

## Disclaimer

This is an independent, community reverse-engineering project. It is
not affiliated with, endorsed by, or supported by Skydroid. Use at your
own risk; always test movement commands (especially tilt) carefully
before trusting them near people, property, or expensive hardware.
