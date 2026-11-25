"""Check the expected motor IDs on an assembled SO-101 follower with LeRobot 0.7."""

import argparse
import sys

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/tty.usbmodem5C931284341")
    args = parser.parse_args()
    bus = SO101Follower(SO101FollowerConfig(port=args.port)).bus
    print(f"Checking {args.port} at {bus.default_baudrate:,} baud")
    try:
        # Skip the full-arm handshake so missing IDs can be reported individually.
        bus._connect(handshake=False)
    except ConnectionError as error:
        print(error, file=sys.stderr)
        return 2

    replies = 0
    try:
        for joint, motor in bus.motors.items():
            model = bus.ping(joint, num_retry=1)
            status = "no reply" if model is None else f"model number {model}"
            print(f"ID {motor.id} ({joint}): {status}")
            if model is not None:
                replies += 1
    finally:
        bus._close()

    print(f"{replies} of {len(bus.motors)} expected IDs replied.")
    return 0 if replies == len(bus.motors) else 1


if __name__ == "__main__":
    sys.exit(main())
