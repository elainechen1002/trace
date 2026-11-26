"""Print live hand data from the Ultraleap service."""
import argparse
from contextlib import contextmanager, ExitStack
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from time import monotonic

from hand_input import HandSample
from main import format_sample


@contextmanager
def open_hand_stream(*, motor_tracking=False):
    root = Path(__file__).resolve().parent
    with ExitStack() as stack:
        if sys.platform == "darwin":
            sdk = Path("/Applications/Ultraleap Hand Tracking.app/Contents/LeapSDK")
            if not (sdk / "include/LeapC.h").is_file():
                raise SystemExit("Install Ultraleap Hand Tracking in /Applications first.")
            binary = Path(stack.enter_context(TemporaryDirectory())) / "leap-stream"
            subprocess.run(["cc", "-std=c11", "-I", str(sdk / "include"),
                            str(root / "leap_stream_mac.c"), "-L", str(sdk / "lib"),
                            "-lLeapC", "-Wl,-rpath," + str(sdk / "lib"), "-o", str(binary)],
                           check=True)
            command = [str(binary)]
            if motor_tracking:
                command.append("--motor-tracking")
            flags = 0
        elif sys.platform == "win32":
            script = "Add-Type -Path './leap_stream.cs' -ErrorAction Stop; [LeapStream]::Run()"
            command = ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script]
            flags = subprocess.CREATE_NO_WINDOW
        else:
            raise SystemExit("Live tracking requires macOS or Windows.")
        process = subprocess.Popen(command, cwd=root, stdout=subprocess.PIPE, text=True,
                                   creationflags=flags)
        try:
            yield process
        finally:
            if process.poll() is None:
                process.terminate()
            process.wait()
            process.stdout.close()


def hand_samples(lines, *, pinch_release_threshold=0.8):
    pinched = False
    for line in lines:
        if line.strip() == "MISSING":
            pinched = False
            yield None
        else:
            x, y, z, pinch_strength = map(float, line.split(","))
            threshold = pinch_release_threshold if pinched else 0.8
            pinched = pinch_strength >= threshold
            yield HandSample(x, y, z, pinched)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, help="Stop after this many seconds.")
    args = parser.parse_args()
    deadline = None if args.seconds is None else monotonic() + args.seconds
    try:
        with open_hand_stream() as process:
            for sample in hand_samples(process.stdout):
                print(format_sample(sample), flush=True)
                if deadline is not None and monotonic() >= deadline:
                    return 0
            return process.wait()
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
