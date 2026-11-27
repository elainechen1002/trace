"""Drive an SO-101 follower from Leap Motion palm position."""
import argparse
from contextlib import contextmanager
import errno
from html import escape
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
from secrets import token_urlsafe
import subprocess
import sys
from threading import Event, Thread
from time import monotonic

from arm_ik import ARM_JOINTS, ArmModel, PalmIK, add, palm_displacement, subtract
from live_leap import hand_samples, open_hand_stream

TICKS_PER_RADIAN = 4096 / math.tau
MAC_TICKS_PER_RADIAN = 4095 / math.tau  # LeRobot degree-mode conversion
WINDOWS_TRAVEL_TICKS = 64
WINDOWS_TOOL_TRAVEL_M = 0.025


@contextmanager
def stop_control(port, status):
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
            mac = sys.platform == "darwin"
            gestures = ("Move one open hand to guide the arm. Pinch to close the gripper; "
                        "release to open it. Remove your hand to pause."
                        if mac else "Show an open hand, then pinch and move. Release to pause.")
            enable_control = (
                '<p><strong>Enabled.</strong></p>'
                if armed.is_set() else
                f'<form method="post" action="{start_path}"><button style="padding:.75rem 1rem;font:inherit">'
                'Enable hand control</button></form>'
            )
            refresh = '<meta http-equiv="refresh" content="2">' if armed.is_set() else ""
            fault = f'<p role="alert">{escape(status["fault"])}</p>' if status.get("fault") else ""
            page = (
                '<!doctype html><html lang="en"><meta charset="utf-8">'
                '<meta name="viewport" content="width=device-width,initial-scale=1">'
                f'{refresh}'
                '<title>SO-101 control</title><main style="font:1.2rem system-ui;max-width:36rem;margin:3rem auto;padding:1rem">'
                '<h1>SO-101 hand control</h1><p>Secure the base, clear the arm’s path, '
                'and keep clear of the gripper. Enable motion when ready.</p>'
                f'<p>{gestures}</p><p>Stop ends hand control '
                'and leaves the arm holding its pose.</p>'
                f'{fault}{enable_control}'
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
    parser.add_argument("--port", help="Controller serial port (auto-detected on Mac, COM3 on Windows)")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--stop-port", type=int, default=8014)
    args = parser.parse_args()
    if sys.platform == "darwin":
        command = [sys.executable, str(Path(__file__).resolve().with_name("mac_motors.py")), "ik"]
        if args.port:
            command.extend(["--port", args.port])
        if args.enable_motion:
            command.append("--enable-motion")
        flags = 0
    elif sys.platform == "win32":
        port = (args.port or "COM3").upper()
        if not port.startswith("COM") or not port[3:].isascii() or not port[3:].isdecimal():
            parser.error("Use a Windows COM port such as COM3.")

        motion = "$true" if args.enable_motion else "$false"
        command = (
            "$ErrorActionPreference = 'Stop'; "
            "Add-Type -Path './shoulder_pan.cs','./ik_drive.cs'; "
            f"[IkDrive]::Run('{port}', {motion}, {WINDOWS_TRAVEL_TICKS})"
        )
        command = ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", command]
        flags = subprocess.CREATE_NO_WINDOW
    else:
        parser.error("Use macOS or Windows.")
    if not args.enable_motion:
        return subprocess.call(command, cwd=Path(__file__).resolve().parent, creationflags=flags)

    status = {}
    with stop_control(args.stop_port, status) as (armed, stop):
        try:
            while not stop.is_set():
                if not armed.wait(0.1):
                    continue
                status.clear()
                try:
                    reason = run_hand_session(command, flags, armed, stop)
                except (RuntimeError, OSError, ValueError) as error:
                    reason = str(error)
                if stop.is_set():
                    break
                status["fault"] = f"Hand control stopped. {reason} Check the arm before enabling again."
                print(status["fault"], file=sys.stderr, flush=True)
                armed.clear()
        except KeyboardInterrupt:
            stop.set()
    return 0


def run_hand_session(command, flags, armed, stop):
    last_error = []
    with subprocess.Popen(
        command, cwd=Path(__file__).resolve().parent, stdin=subprocess.PIPE,
        stdout=subprocess.PIPE if sys.platform == "darwin" else None,
        stderr=subprocess.PIPE, bufsize=0, creationflags=flags,
    ) as motors:
        def log_motor_feedback():
            for line in motors.stderr:
                message = line.decode(errors="replace").strip()
                print(message, file=sys.stderr, flush=True)
                if message:
                    last_error[:] = [message]

        logger = Thread(target=log_motor_feedback, daemon=True)
        logger.start()
        try:
            follow(ArmModel(), motors, armed, stop)
        except OSError as error:
            if error.errno not in (errno.EPIPE, errno.EINVAL):
                raise
        finally:
            motors.stdin.close()
            result = motors.wait()
            logger.join()
        if result and last_error:
            return last_error[0]
        return "The motor or hand tracking stream ended."


def follow(model, motors, armed, stop):
    neutral_angles = [0.0] * len(ARM_JOINTS)
    neutral_tool = model.forward(neutral_angles)[0][-1]
    simulation = PalmIK(model) if sys.platform == "darwin" else None
    angles = neutral_angles
    anchor = None
    saw_open_hand = False
    last_input_report = 0.0

    def send(message):
        motors.stdin.write((message + "\n").encode("ascii"))

    with open_hand_stream(motor_tracking=True) as tracking:
        for hand in hand_samples(tracking.stdout, pinch_release_threshold=0.4 if simulation is not None else 0.6):
            if stop.is_set() or motors.poll() is not None:
                break
            if not armed.is_set():
                continue
            palm = None if hand is None else (hand.x_mm, hand.y_mm, hand.z_mm)
            if palm is None or not all(map(math.isfinite, palm)):
                saw_open_hand = False
                anchor = None
                if simulation is not None:
                    simulation.accept(None)
                send("HOLD")
                continue
            if simulation is not None:
                if simulation.anchor is None:
                    # Require an open hand to start or reacquire control.
                    if hand.pinched:
                        send("HOLD")
                        continue
                    send("POSE")
                    from select import select
                    if not select([motors.stdout], [], [], 1.5)[0]:
                        raise RuntimeError("Motor feedback timed out while anchoring the hand.")
                    line = motors.stdout.readline()
                    if not line:
                        break
                    pose = json.loads(line)
                    centers = [(low + high) / 2 for _, low, high in pose[:5]]
                    simulation.angles = []
                    simulation.joint_bounds = []
                    for joint, (raw, low, high), center in zip(model.joints, pose, centers):
                        angle = (raw - center) / MAC_TICKS_PER_RADIAN
                        simulation.angles.append(angle)
                        # Respect both sets of limits. If already past a model
                        # limit, allow holding or moving back from this pose.
                        lower = max(joint.lower, (low - center) / MAC_TICKS_PER_RADIAN)
                        upper = min(joint.upper, (high - center) / MAC_TICKS_PER_RADIAN)
                        simulation.joint_bounds.append((min(lower, angle), max(upper, angle)))
                simulation.accept(hand)
                ticks = [round(center + simulation.latest["joints"][name] * MAC_TICKS_PER_RADIAN)
                         for name, center in zip(ARM_JOINTS, centers)]
                _, closed, opened = pose[5]
                # The calibrated closed position is the requested pinch endpoint.
                inset = max(1, round((opened - closed) * 0.02))
                ticks.append(closed if hand.pinched else opened - inset)
                send(",".join(map(str, ticks)))
                now = monotonic()
                if now - last_input_report >= 1:
                    print(f"Hand (mm): x={hand.x_mm:.1f} y={hand.y_mm:.1f} z={hand.z_mm:.1f}, "
                          f"pinched={hand.pinched}", file=sys.stderr, flush=True)
                    last_input_report = now
                continue
            if not hand.pinched:
                saw_open_hand = True
                anchor = None
                send("HOLD")
                continue
            if not saw_open_hand:
                continue
            if anchor is None:
                anchor = palm, model.forward(angles)[0][-1]
            initial_palm, initial_tool = anchor
            target = add(initial_tool, palm_displacement(palm, initial_palm))
            joint_window = None
            if sys.platform == "win32":
                displacement = subtract(target, neutral_tool)
                distance = math.dist(target, neutral_tool)
                if distance > WINDOWS_TOOL_TRAVEL_M:
                    target = add(neutral_tool, tuple(v * WINDOWS_TOOL_TRAVEL_M / distance for v in displacement))
                joint_window = WINDOWS_TRAVEL_TICKS / TICKS_PER_RADIAN
            proposal, error = model.solve(target, angles, joint_window=joint_window)
            if error > 0.006:
                send("HOLD")
                continue
            angles = proposal
            ticks = [round(angle * TICKS_PER_RADIAN) for angle in angles]
            send(",".join(map(str, ticks)))


if __name__ == "__main__":
    raise SystemExit(main())
