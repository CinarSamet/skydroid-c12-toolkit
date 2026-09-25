# Changelog

All notable changes to this project are documented here.
This project follows [Semantic Versioning](https://semver.org/).

## [0.1.0] - 2026-09-24

### Added
- Initial public release.
- Reverse-engineered `#TP` UDP protocol layer (`protocol.py`): checksum,
  PTZ fixed commands, speed-based pan/tilt (GSY/GSP), absolute-angle
  pan/tilt (GAY/GAP), attitude telemetry push/parse (GAA/GAC).
- `AttitudeReader` and `get_attitude_once()` for reading IMU (yaw/pitch/
  roll) telemetry (`telemetry.py`).
- `GimbalController` high-level movement API with optional closed-loop
  (IMU-confirmed) control (`gimbal.py`).
- `Camera` single-object facade combining transport + IMU + gimbal
  control (`camera.py`).
- `skydroid-c12` command-line tool for manual movement and IMU readout
  from a terminal (`cli.py`).
- Test suite (`pytest`) with a mock UDP camera fixture, covering protocol
  encoding/decoding and open-loop/closed-loop control logic.
- GitHub Actions CI (tests on Python 3.8-3.12, lint via ruff/mypy).

### Notes
- All protocol details (port, checksum, command tables, sign
  conventions) were verified against a real C12 unit; see the README's
  "Verified protocol notes" section. Your firmware version may differ.
