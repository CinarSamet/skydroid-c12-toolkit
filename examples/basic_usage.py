"""
Minimal end-to-end example: move the gimbal through a few absolute
positions and print the IMU reading after each one.

Usage:
    python examples/basic_usage.py 192.168.144.108
"""

from __future__ import annotations

import sys

from skydroid_c12 import Camera


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python examples/basic_usage.py <camera_ip>")
        sys.exit(1)

    camera_ip = sys.argv[1]

    with Camera(camera_ip) as cam:
        print("Centering...")
        cam.center()

        for pan, tilt in [(30, 0), (30, 15), (-30, -10), (0, 0)]:
            print(f"Moving to pan={pan}, tilt={tilt}...")
            cam.goto(pan=pan, tilt=tilt)

            sample = cam.get_attitude()
            if sample is not None:
                print(
                    f"  IMU reading: yaw={sample.yaw_deg:.2f} "
                    f"pitch={sample.pitch_deg:.2f} roll={sample.roll_deg:.2f}"
                )
            else:
                print("  (no IMU reading available)")

        print("Done.")


if __name__ == "__main__":
    main()
