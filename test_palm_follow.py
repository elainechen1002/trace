"""Exercise the physical controller's gesture protocol without a serial connection."""
from contextlib import nullcontext
import io
import json
import math
import os
from threading import Event
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from arm_follow_ik import follow
from arm_ik import ArmModel, PalmIK, palm_displacement

# Positions and limits from this arm after the reported shoulder drift.
POSE = [(2051,1099,2656), (2389,841,3216), (2123,838,3074),
        (2087,844,2845), (2045,0,4095), (2038,2031,3497)]


def replay(frames):
    armed = Event()
    armed.set()
    read_fd, write_fd = os.pipe()
    with os.fdopen(write_fd, "wb") as stream:
        stream.write((json.dumps(POSE) + "\n").encode() * 2)
    with os.fdopen(read_fd, "rb") as feedback:
        motors = SimpleNamespace(stdin=io.BytesIO(), stdout=feedback, poll=lambda: None)
        with patch("arm_follow_ik.sys.platform", "darwin"), patch(
            "arm_follow_ik.open_hand_stream",
            return_value=nullcontext(SimpleNamespace(stdout=io.StringIO(frames))),
        ):
            follow(ArmModel(), motors, armed, Event())
        return motors.stdin.getvalue().decode().splitlines()


class Gestures(unittest.TestCase):
    def test_open_hand_moves_base_and_pinch_only_changes_jaw(self):
        lines = replay("0,200,0,0\n60,200,0,0\n60,200,0,1\n60,200,0,0\n")
        self.assertEqual(lines.pop(0), "POSE")
        home, right, pinch, release = [list(map(int, line.split(","))) for line in lines]
        self.assertEqual(home[:5], [row[0] for row in POSE[:5]])
        self.assertGreater(right[0] - home[0], 10)
        self.assertEqual(right[:5], pinch[:5])
        self.assertEqual(pinch[5], POSE[5][1])
        self.assertEqual(right, release)

    def test_motor_targets_match_preview_angles_from_recorded_pose(self):
        frames = [(0, 200, 0), (35, 200, 0), (35, 230, -40), (-30, 215, 20)]
        _, *commands = replay("".join(f"{x},{y},{z},0\n" for x, y, z in frames))
        preview = PalmIK(ArmModel())
        centers = [(lo + hi) / 2 for _, lo, hi in POSE[:5]]
        ticks_per_rad = 4095 / math.tau
        preview.angles = [(raw - center) / ticks_per_rad
                          for (raw, _, _), center in zip(POSE, centers)]
        for (x, y, z), command in zip(frames, commands):
            preview.accept(SimpleNamespace(x_mm=x, y_mm=y, z_mm=z, pinched=False))
            for raw, center, angle in zip(map(int, command.split(",")), centers, preview.angles):
                self.assertLessEqual(abs(raw - (center + angle * ticks_per_rad)), 0.5)

    def test_simulated_servos_reach_targets_via_group_writes(self):
        from mac_motors import run
        from test_mac_motors import FakeBus

        class FollowingBus(FakeBus):
            def sync_write_register(self, joints, name, values):
                packets.append((name, tuple(joints)))
                self.positions.update(zip(joints, values))

        bus = FollowingBus()
        packets = []
        bus.positions.update({j: row[0] for j, row in enumerate(POSE, 1)})
        _, home, target = replay("0,200,0,0\n30,225,-10,1\n")
        with patch("mac_motors.commands", return_value=iter([home] + [target] * 100)):
            run(bus, ik=True, travel=None, seconds=None, motion=True)
        self.assertTrue(any(name == "goal_position" and len(ids) > 1 for name, ids in packets))
        self.assertEqual(list(bus.positions.values()), list(map(int, target.split(","))))

    def test_one_to_one_translation_on_all_axes(self):
        origin = (10, 200, 25)
        moved = (30, 215, -5)
        self.assertEqual(palm_displacement(moved, origin), (0.020, 0.030, 0.015))
        controller = PalmIK(ArmModel())
        for x, y, z in (origin, moved):
            controller.accept(SimpleNamespace(x_mm=x, y_mm=y, z_mm=z, pinched=False))
        initial_tool = controller.anchor[1]
        for start, end, delta in zip(initial_tool, controller.latest["target"], (0.030, -0.020, 0.015)):
            self.assertAlmostEqual(end - start, delta)

    def test_palm_lift_from_recorded_shoulder_pose(self):
        pose = POSE.copy()
        pose[1] = (2596, 841, 3216)
        with patch(__name__ + ".POSE", pose):
            _, anchor, lift = replay("0,200,0,0\n0,260,0,0\n")
        model = ArmModel()
        def tool_position(command):
            angles = [(int(tick) - (low + high) / 2) * math.tau / 4095
                      for tick, (_, low, high) in zip(command.split(","), pose[:5])]
            return model.forward(angles)[0][-1]
        start = tool_position(anchor)
        expected = (start[0], start[1], start[2] + 0.06)
        self.assertLess(math.dist(tool_position(lift), expected), 0.002)

    def test_tracking_loss_holds_and_reanchors(self):
        lines = replay("0,200,0,0\nMISSING\n100,240,0,1\n100,240,0,0\n")
        self.assertEqual(lines[2:5], ["HOLD", "HOLD", "POSE"])
        self.assertEqual(lines[1], lines[-1])
