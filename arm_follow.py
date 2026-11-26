"""Use a Leap Motion pinch to enable a small SO-101 shoulder-pan test."""
import argparse
from pathlib import Path
import math
import subprocess
import sys
from time import monotonic

from live_leap import hand_samples, open_hand_stream


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Controller serial port (auto-detected on Mac, COM3 on Windows)")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--seconds", type=int, default=25, choices=range(1, 61), metavar="1-60")
    parser.add_argument("--travel-deg", type=int, default=9, choices=range(1, 19), metavar="1-18")
    args = parser.parse_args()
    if sys.platform == "darwin":
        command = [sys.executable, str(Path(__file__).resolve().with_name("mac_motors.py")), "pan"]
        if args.port:
            command.extend(["--port", args.port])
        if args.enable_motion:
            command.append("--enable-motion")
        command.extend(["--seconds", str(args.seconds), "--travel-deg", str(args.travel_deg)])
        flags = 0
    elif sys.platform == "win32":
        port_number = (args.port or "COM3").upper().removeprefix("COM")
        if not (args.port or "COM3").upper().startswith("COM") or not port_number.isascii() or not port_number.isdecimal():
            parser.error("Use a Windows COM port such as COM3.")

        motion = "$true" if args.enable_motion else "$false"
        command = (
            "Add-Type -Path './shoulder_pan.cs' -ErrorAction Stop; "
            f"[ShoulderPan]::Run('COM{port_number}', {motion}, {args.seconds}, {args.travel_deg})"
        )
        command = ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command]
        flags = subprocess.CREATE_NO_WINDOW
    else:
        parser.error("Use macOS or Windows.")
    with subprocess.Popen(
        command,
        cwd=Path(__file__).resolve().parent, stdin=subprocess.PIPE, text=True,
        creationflags=flags,
    ) as motor:
        try:
            if not args.enable_motion:
                return motor.wait()

            print("Show an open hand, then pinch and move sideways. Release to stop.", flush=True)
            deadline = monotonic() + args.seconds
            with open_hand_stream() as tracking:
                for sample in hand_samples(tracking.stdout, pinch_release_threshold=0.6):
                    if motor.poll() is not None or monotonic() >= deadline:
                        break
                    if sample is None or not math.isfinite(sample.x_mm):
                        value = "missing"
                    elif not sample.pinched:
                        value = "open"
                    else:
                        value = format(sample.x_mm, ".4f")
                    try:
                        motor.stdin.write(value + "\n")
                        motor.stdin.flush()
                    except BrokenPipeError:
                        break
        except KeyboardInterrupt:
            pass
        finally:
            # EOF requests a normal shutdown so the motor bridge can release torque.
            try:
                motor.stdin.close()
            except BrokenPipeError:
                pass
        return motor.wait()


if __name__ == "__main__":
    raise SystemExit(main())
