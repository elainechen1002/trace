"""Serve only the viewer directory on this computer. Ctrl+C stops it."""
import argparse
from contextlib import ExitStack
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
from threading import Thread
from time import monotonic


class HandFeed:
    def __init__(self, lines, ik=None):
        self.latest = (0.0, None)
        self.ik = ik
        Thread(target=self.read, args=(lines,), daemon=True).start()

    def read(self, lines):
        from live_leap import hand_samples

        try:
            for sample in hand_samples(lines, pinch_release_threshold=0.6 if self.ik else 0.8):
                received_at = monotonic()
                if self.ik:
                    if received_at - self.latest[0] >= 0.6:
                        self.ik.accept(None)
                    self.ik.accept(sample)
                self.latest = (received_at, sample)
        finally:
            if self.ik:
                self.ik.accept(None)
            self.latest = (0.0, None)

    def snapshot(self):
        received_at, sample = self.latest
        if sample is None or monotonic() - received_at >= 0.6:
            return None
        return {"x_mm": sample.x_mm, "y_mm": sample.y_mm, "pinched": sample.pinched}

    def ik_snapshot(self):
        state = self.ik.latest
        if self.snapshot() is None:
            return {**state, "target": None, "error_mm": None, "tracking": False, "active": False}
        return state


class ViewerHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, hand_feed, preview_only, **kwargs):
        self.hand_feed = hand_feed
        self.preview_only = preview_only
        super().__init__(*args, **kwargs)

    def do_GET(self):
        if self.path in ("/hand", "/ik"):
            if self.hand_feed is None or (self.path == "/ik" and self.hand_feed.ik is None):
                self.send_error(404)
                return
            state = self.hand_feed.ik_snapshot() if self.path == "/ik" else self.hand_feed.snapshot()
            body = json.dumps(state, allow_nan=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            return
        if self.path == "/" and self.preview_only:
            self.path = "/hand-preview.html"
        super().do_GET()

    def log_request(self, code="-", size="-"):
        if self.path not in ("/hand", "/ik"):
            super().log_request(code, size)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--hand", action="store_true", help="Show a live Leap Motion target.")
    parser.add_argument("--ik", action="store_true", help="Simulate gripper position from the Leap Motion.")
    args = parser.parse_args()
    use_hand = args.hand or args.ik
    directory = Path(__file__).resolve().parent / "viewer"
    viewer_ready = all((directory / path).is_file() for path in (
        "node_modules/urdf-loader/src/URDFLoader.js",
        "node_modules/three/build/three.module.js",
        "models/SO101/so101_new_calib.urdf",
        "models/SO101/assets/base_so101_v2.stl",
    ))
    if not viewer_ready and not use_hand:
        parser.exit(1, "First run: npm --prefix viewer ci\n")
    if use_hand and sys.platform not in ("darwin", "win32"):
        parser.error("--hand and --ik require macOS or Windows")
    with ExitStack() as stack:
        hand_feed = None
        if use_hand:
            from live_leap import open_hand_stream

            ik = None
            if args.ik:
                from arm_ik import ArmModel, PalmIK

                ik = PalmIK(ArmModel())
            process = stack.enter_context(open_hand_stream())
            hand_feed = HandFeed(process.stdout, ik)
        handler = partial(ViewerHandler, directory=str(directory), hand_feed=hand_feed,
                          preview_only=not viewer_ready)
        try:
            server = stack.enter_context(ThreadingHTTPServer(("127.0.0.1", args.port), handler))
        except OSError as error:
            parser.exit(1, f"Cannot start viewer: {error}. Try --port 8001\n")
        print(f"SO-101 viewer: http://127.0.0.1:{server.server_port} — Ctrl+C to stop", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
