"""Print live hand data from the Windows Ultraleap service."""
import argparse
from contextlib import contextmanager
from pathlib import Path
import subprocess
import sys
from time import monotonic

from hand_input import HandSample
from main import format_sample


@contextmanager
def open_hand_stream():
    # PowerShell uses the installed LeapC DLL through the adjacent C# source.
    command = "Add-Type -Path './leap_stream.cs' -ErrorAction Stop; [LeapStream]::Run()"
    process = subprocess.Popen(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command],
        cwd=Path(__file__).resolve().parent,
        stdout=subprocess.PIPE,
        text=True,
        creationflags=subprocess.CREATE_NO_WINDOW,
    )
    try:
        yield process
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait()
        process.stdout.close()


def hand_samples(lines):
    for line in lines:
        if line.strip() == "MISSING":
            yield None
        else:
            x, y, z, pinch_strength = map(float, line.split(","))
            yield HandSample(x, y, z, pinch_strength >= 0.8)


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
