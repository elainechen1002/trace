"""Drive a centered SO-101 follower from Leap Motion palm position on Windows."""
import argparse
from contextlib import contextmanager
import errno
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import math
from pathlib import Path
from secrets import token_urlsafe
import subprocess
from threading import Event, Thread

from arm_ik import ARM_JOINTS, ArmModel, add, subtract
from live_leap import hand_samples, open_hand_stream

TICKS_PER_RADIAN = 4096 / math.tau
TRAVEL_TICKS = 64
TOOL_TRAVEL_M = 0.025


@contextmanager
def stop_control(port):
    stop = Event()
    armed = Event()
    token = token_urlsafe(24)
    start_path = "/enable/" + token
    stop_path = "/stop/" + token

    class StopHandler(BaseHTTPRequestHandler):
        def local_request(self):
            host = f"127.0.0.1:{self.server.server_port}"
            return self.headers.get("Host") == host and self.headers.get("Origin") in (
                None, f"http://{host}"
            )

        def do_GET(self):
            if self.path != "/" or not self.local_request():
                self.send_error(404)
                return
            enable_control = (
                '<p><strong>Enabled.</strong> Show an open hand, then pinch and move.</p>'
                if armed.is_set() else
                f'<form method="post" action="{start_path}"><button style="padding:.75rem 1rem;font:inherit">'
                'Enable hand control</button></form>'
            )
            page = (
                '<!doctype html><html lang="en"><meta charset="utf-8">'
                '<meta name="viewport" content="width=device-width,initial-scale=1">'
                '<title>SO-101 control</title><main style="font:1.2rem system-ui;max-width:36rem;margin:3rem auto;padding:1rem">'
                '<h1>SO-101 hand control</h1><p>Secure the base, clear the arm’s path, '
                'and take your hands off the arm. Enable motion, then show an open hand '
                'to the Leap Motion before pinching and moving.</p>'
                '<p>Release your pinch to pause movement. Stop ends hand control '
                'and leaves the arm holding its pose.</p>'
                f'{enable_control}'
                f'<form method="post" action="{stop_path}"><button style="padding:.75rem 1rem;font:inherit">'
                'Stop hand control</button></form></main></html>'
            )
            self.respond(page)

        def do_POST(self):
            if not self.local_request():
                self.send_error(403)
                return
            if self.path == start_path and not stop.is_set():
                armed.set()
                self.send_response(303)
                self.send_header("Location", "/")
                self.end_headers()
                return
            if self.path != stop_path:
                self.send_error(404)
                return
            stop.set()
            self.respond("<p>Stop requested. The controller will hold the arm in place.</p>")

        def respond(self, text):
            body = text.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Security-Policy", "frame-ancestors 'none'; form-action 'self'")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    with ThreadingHTTPServer(("127.0.0.1", port), StopHandler) as server:
        worker = Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            print(f"Hand controls: http://127.0.0.1:{server.server_port}/", flush=True)
            yield armed, stop
        finally:
            server.shutdown()
            worker.join()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--stop-port", type=int, default=8014)
    args = parser.parse_args()
    port = args.port.upper()
    if not port.startswith("COM") or not port[3:].isascii() or not port[3:].isdecimal():
        parser.error("Use a Windows COM port such as COM3.")

    model = ArmModel()
    motion = "$true" if args.enable_motion else "$false"
    command = (
        "$ErrorActionPreference = 'Stop'; "
        "Add-Type -Path './shoulder_pan.cs','./ik_drive.cs'; "
        f"[IkDrive]::Run('{port}', {motion}, {TRAVEL_TICKS})"
    )
    with subprocess.Popen(
        ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command],
        cwd=Path(__file__).resolve().parent,
        stdin=subprocess.PIPE,
        bufsize=0,
        creationflags=subprocess.CREATE_NO_WINDOW,
    ) as motors:
        try:
            if args.enable_motion:
                with stop_control(args.stop_port) as (armed, stop):
                    follow(model, motors, armed, stop)
        except KeyboardInterrupt:
            pass
        except OSError as error:
            if error.errno not in (errno.EPIPE, errno.EINVAL):
                raise
            # Windows can report a closed subprocess pipe as EINVAL.
            # Preserve the motor bridge's exit status and original fault message.
            motors.wait(timeout=1)
        finally:
            # EOF stops the motion loop and holds the arm at its measured position.
            motors.stdin.close()
        return motors.wait()


def follow(model, motors, armed, stop):
    neutral_angles = [0.0] * len(ARM_JOINTS)
    neutral_tool = model.forward(neutral_angles)[0][-1]
    angles = neutral_angles
    anchor = None
    saw_open_hand = False

    def send(message):
        motors.stdin.write((message + "\n").encode("ascii"))

    with open_hand_stream() as tracking:
        for hand in hand_samples(tracking.stdout, pinch_release_threshold=0.6):
            if stop.is_set() or motors.poll() is not None:
                break
            if not armed.is_set():
                continue
            palm = None if hand is None else (hand.x_mm, hand.y_mm, hand.z_mm)
            if palm is None or not all(map(math.isfinite, palm)) or not hand.pinched:
                saw_open_hand = palm is not None and not hand.pinched
                anchor = None
                send("HOLD")
                continue
            if not saw_open_hand:
                continue
            if anchor is None:
                anchor = palm, model.forward(angles)[0][-1]
            initial_palm, initial_tool = anchor
            x, y, z = subtract(palm, initial_palm)
            target = add(initial_tool, (x * 0.00025, -z * 0.00025, y * 0.00025))
            displacement = subtract(target, neutral_tool)
            distance = math.dist(target, neutral_tool)
            if distance > TOOL_TRAVEL_M:
                target = add(neutral_tool, tuple(v * TOOL_TRAVEL_M / distance for v in displacement))
            proposal, error = model.solve(target, angles, joint_window=TRAVEL_TICKS / TICKS_PER_RADIAN)
            if error > 0.006:
                send("HOLD")
                continue
            angles = proposal
            send(",".join(str(round(angle * TICKS_PER_RADIAN)) for angle in angles))


if __name__ == "__main__":
    raise SystemExit(main())
