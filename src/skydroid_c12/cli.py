"""
Command-line tool for manually moving the gimbal and reading the IMU from
a terminal, without writing any Python.

Installed as the `skydroid-c12` console script (see pyproject.toml), or
run directly:

    python -m skydroid_c12.cli 192.168.144.108 right 30

See `skydroid-c12 --help` for the full command list.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time

from .camera import Camera
from .config import DEFAULT_CONFIG

# English command name -> internal action. Turkish aliases are accepted
# too (this project was originally written for a Turkish-speaking team),
# so either works.
_DIRECTION_ALIASES = {
    "right": "right", "sag": "right",
    "left": "left", "sol": "left",
    "tilt": "tilt", "egim": "tilt",
    "goto": "goto",
    "point": "point", "nokta": "point",
    "center": "center", "merkez": "center",
    "stop": "stop", "dur": "stop",
    "imu": "imu",
    "watch": "watch", "izle": "watch",
}


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="skydroid-c12",
        description="Skydroid C12 - manual gimbal movement + IMU reading from the terminal.",
        epilog=(
            "Examples:\n"
            "  skydroid-c12 192.168.144.108 right 30\n"
            "  skydroid-c12 192.168.144.108 tilt 10 --speed 20\n"
            "  skydroid-c12 192.168.144.108 point 30 10\n"
            "  skydroid-c12 192.168.144.108 center\n"
            "  skydroid-c12 192.168.144.108 imu\n"
            "  skydroid-c12 192.168.144.108 watch 15\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("camera_ip", help="Camera IP address, e.g. 192.168.144.108")
    parser.add_argument(
        "command", choices=sorted(_DIRECTION_ALIASES.keys()),
        help="Action to perform",
    )
    parser.add_argument(
        "value", nargs="?", type=float, default=None,
        help="Degrees for right/left/tilt/goto, PAN degrees for point, "
             "seconds for watch (not needed for center/stop/imu)",
    )
    parser.add_argument(
        "value2", nargs="?", type=float, default=None,
        help="point only: TILT degrees (e.g. 'point 30 10')",
    )
    parser.add_argument(
        "--speed", type=int, default=None,
        help=f"Movement speed (1..99, default {DEFAULT_CONFIG.gimbal.default_speed}).",
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_CONFIG.network.tp_udp_port,
        help=f"Control UDP port (default {DEFAULT_CONFIG.network.tp_udp_port}).",
    )
    parser.add_argument(
        "--no-imu", action="store_true",
        help="Disable the IMU entirely (open-loop movement, no attitude data).",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable debug logging.",
    )
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = _build_parser()
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s: %(message)s",
    )

    action = _DIRECTION_ALIASES[args.command]

    if action in ("right", "left", "tilt", "goto") and args.value is None:
        parser.error(f"'{args.command}' needs a value, e.g.: {args.command} 30")
    if action == "point" and (args.value is None or args.value2 is None):
        parser.error("'point' needs TWO values: point <pan> <tilt>, e.g. point 30 10")

    speed = args.speed or DEFAULT_CONFIG.gimbal.default_speed

    with Camera(args.camera_ip, udp_port=args.port, enable_imu=not args.no_imu) as cam:
        try:
            if action == "center":
                print("Returning to center...")
                cam.center()
                print("Done.")

            elif action == "stop":
                print("Stopping...")
                cam.stop_now()
                print("Done.")

            elif action == "right":
                print(f"Turning {args.value} degrees right (speed={speed})...")
                cam.move_right(args.value, speed=speed)
                print(f"Done. pan={cam.pan_deg:.1f}  tilt={cam.tilt_deg:.1f}")

            elif action == "left":
                print(f"Turning {args.value} degrees left (speed={speed})...")
                cam.move_left(args.value, speed=speed)
                print(f"Done. pan={cam.pan_deg:.1f}  tilt={cam.tilt_deg:.1f}")

            elif action == "goto":
                print(f"Going to absolute pan={args.value} degrees (speed={speed})...")
                cam.goto_pan(args.value, speed=speed)
                print(f"Done. pan={cam.pan_deg:.1f}  tilt={cam.tilt_deg:.1f}")

            elif action == "tilt":
                print(f"Going to absolute tilt={args.value} degrees (speed={speed})...")
                cam.goto_tilt(args.value, speed=speed)
                print(f"Done. pan={cam.pan_deg:.1f}  tilt={cam.tilt_deg:.1f}")

            elif action == "point":
                print(
                    f"Going to absolute pan={args.value}, tilt={args.value2} "
                    f"degrees TOGETHER (speed={speed})..."
                )
                cam.goto(pan=args.value, tilt=args.value2, speed=speed)
                print(f"Done. pan={cam.pan_deg:.1f}  tilt={cam.tilt_deg:.1f}")

            elif action == "imu":
                print("Reading raw IMU data (may take a couple of seconds)...")
                sample = cam.get_attitude()
                t0 = time.monotonic()
                while sample is None and time.monotonic() - t0 < 3.0:
                    time.sleep(0.1)
                    sample = cam.get_attitude()
                if sample is None:
                    print("ERROR: no IMU data received within 3 seconds.")
                    print("Check that the camera supports GAA/GAC and that it's reachable.")
                    sys.exit(1)
                print(
                    f"yaw={sample.yaw_deg:+.2f}  pitch={sample.pitch_deg:+.2f}  "
                    f"roll={sample.roll_deg:+.2f}  degrees"
                )

            elif action == "watch":
                watch_sec = args.value if args.value is not None else 8.0
                print(f"Watching IMU data for {watch_sec:.0f}s (Ctrl+C to stop early)...\n")
                t0 = time.monotonic()
                try:
                    while time.monotonic() - t0 < watch_sec:
                        sample = cam.get_attitude()
                        if sample is not None:
                            print(
                                f"  yaw={sample.yaw_deg:+7.2f}  pitch={sample.pitch_deg:+7.2f}  "
                                f"roll={sample.roll_deg:+7.2f}  "
                                f"(fresh={cam.attitude_is_fresh()}, packets={cam.imu_packet_count})"
                            )
                        else:
                            print("  no IMU data yet...")
                        time.sleep(0.3)
                except KeyboardInterrupt:
                    print("\n(stopped early)")
                print("\nDone.")

        except KeyboardInterrupt:
            print("\nCancelled, stopping...")
            cam.stop_now()
        except ValueError as exc:
            print(f"ERROR: {exc}")
            sys.exit(1)


if __name__ == "__main__":
    main()
