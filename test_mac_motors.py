"""Exercise Mac motor guards without opening a serial port."""
import unittest
from unittest.mock import patch

from mac_motors import run


class FakeBus:
    def __init__(self):
        self.writes = []
        self.positions = dict.fromkeys(range(1, 7), 2048)

    def sync_write_register(self, joints, name, values):
        for joint, value in zip(joints, values):
            self.write_register_with_error(joint, name, value)

    def read_register_with_error(self, joint, name):
        registers = {"torque_limit": 500, "model": 777, "mode": 0, "torque_enable": 0, "present_speed": 0,
                     "min_position_limit": 0, "max_position_limit": 4095,
                     "present_position": self.positions[joint]}
        return registers[name], 0

    def write_register_with_error(self, joint, name, value):
        self.writes.append((joint, name, value))
        return 0


class MacMotorTests(unittest.TestCase):
    def setUp(self):
        self.bus = FakeBus()

    def run_ik(self, commands=(), motion=True):
        with patch("mac_motors.commands", return_value=iter(commands)):
            run(self.bus, ik=True, travel=None, seconds=None, motion=motion)

    def test_preflight_never_writes(self):
        self.run_ik(motion=False)
        self.assertFalse(self.bus.writes)
        self.bus.positions[2] = 842
        self.run_ik(motion=False)
        self.bus.positions[2] = -1
        with self.assertRaisesRegex(ValueError, "Motor 2"):
            self.run_ik(motion=False)
        self.assertFalse(self.bus.writes)

    def test_missing_or_invalid_first_target_never_enables(self):
        self.run_ik(["HOLD", None])
        with self.assertRaises(ValueError):
            self.run_ik(["99999,0,0,0,0,0"])
        self.assertFalse(self.bus.writes)

    def test_timeout_holds_measured_pose_and_discards_later_targets(self):
        with self.assertRaisesRegex(RuntimeError, "deadline"):
            self.run_ik(["2112,2048,2048,2048,2048,2048", None, "1984,2048,2048,2048,2048,2048"])
        pan_goals = [v for j, reg, v in self.bus.writes if j == 1 and reg == "goal_position"]
        self.assertIn(2058, pan_goals)
        self.assertEqual(pan_goals[-1], 2048)
        self.assertNotIn(2038, pan_goals)
        self.assertTrue(all(j <= 6 for j, _, _ in self.bus.writes))

    def test_goal_can_travel_past_original_joint_window(self):
        def samples():
            for _ in range(10):
                yield "2148,2048,2048,2048,2048,2048"
                self.bus.positions[1] = self.bus.writes[-1][2]
        self.run_ik(samples())
        self.assertIn((1, "goal_position", 2148), self.bus.writes)

    def test_one_tick_lag_allows_partial_next_step(self):
        def samples():
            yield "2148,2048,2048,2048,2048,2048"
            self.bus.positions[1] = 2057  # first goal was 2058
            yield "2148,2048,2048,2048,2048,2048"
        self.run_ik(samples())
        self.assertIn((1, "goal_position", 2068), self.bus.writes)

    def test_stationary_target_does_not_follow_measured_drift(self):
        def samples():
            yield "2048,2048,2048,2048,2048,2048"
            self.bus.positions[2] += 35
            yield "2048,2048,2048,2048,2048,2048"
            self.assertEqual([v for j, reg, v in self.bus.writes
                              if j == 2 and reg == "goal_position"], [2048])

        self.run_ik(samples())

    def test_transient_crossing_of_old_84_tick_boundary_continues(self):
        def samples():
            yield "2048,2048,2048,2048,2048,2048"
            self.bus.positions[2] += 87
            yield "2048,2048,2048,2048,2048,2048"
            self.bus.positions[2] -= 87
            yield "2058,2048,2048,2048,2048,2048"
        self.run_ik(samples())
        self.assertIn((1, "goal_position", 2058), self.bus.writes)

    def test_new_hand_target_supersedes_lag_at_watchdog_deadline(self):
        now = [0.0]
        def samples():
            for t in (0.0, 0.1, 0.2, 0.3):
                now[0] = t
                yield "2048,1700,2048,2048,2048,2048"
            # The previous lift goal was never reached. The operator now
            # requests movement on the other side of the measured position.
            now[0] = 1.4
            for _ in range(7):
                yield "2048,2100,2048,2048,2048,2048"
                self.bus.positions[2] = self.bus.writes[-1][2]
                now[0] += 0.1
        with patch("mac_motors.monotonic", side_effect=lambda: now[0]):
            self.run_ik(samples())
        self.assertGreater(self.bus.positions[2], 2048)

    def test_shoulder_stall_at_lead_limit_ends_control(self):
        self.bus.positions[2] = 2363
        now = [0.0]
        def samples():
            for frame in range(20):
                now[0] = frame / 10
                yield "2048,1707,2048,2048,2048,2048"
        with patch("mac_motors.monotonic", side_effect=lambda: now[0]):
            with self.assertRaisesRegex(ValueError, "Motor 2 (stalled|failed to track)"):
                self.run_ik(samples())
        goals = [v for j, register, v in self.bus.writes if j == 2 and register == "goal_position"]
        self.assertEqual(min(goals), 2327)
        self.assertEqual(goals[-1], 2363)

    def test_lift_overcomes_eighteen_tick_load_and_paces_elbow(self):
        self.bus.positions[2] = 2649
        def samples():
            for frame in range(10):
                yield "2048,1069,3057,2048,2048,2048"
                goals = {j: v for j, reg, v in self.bus.writes if reg == "goal_position"}
                if frame == 2:
                    self.assertEqual(goals[3], 2066)
                # Simulated load takes 18 ticks of position error to move.
                if frame >= 2:
                    self.bus.positions[2] = min(self.bus.positions[2], goals[2] + 18)
                self.bus.positions[3] = goals[3]
        self.run_ik(samples())
        self.assertLess(self.bus.positions[2], 2600)
        self.assertGreater(self.bus.positions[3], 2068)

    def test_loaded_elbow_with_pan_and_jaw_reversal(self):
        self.bus.positions.update({1: 2112, 3: 2703, 6: 2032})
        now = [0.0]
        def samples():
            for frame in range(50):
                now[0] = frame / 10
                opening = frame < 25
                pan, jaw = (2232, 3468) if opening else (2112, 2032)
                yield f"{pan},2048,1772,2048,2048,{jaw}"
                goals = {j: v for j, reg, v in self.bus.writes if reg == "goal_position"}
                self.bus.positions[1] = goals[1]
                self.bus.positions[3] = min(self.bus.positions[3], goals[3] + 23)
                actual = self.bus.positions[6]
                self.bus.positions[6] = max(goals[6] - 25, min(goals[6] + 25, actual))
                if frame == 24:
                    self.assertEqual(self.bus.positions[1], 2232)
                    self.assertGreater(self.bus.positions[6], 2150)
        with patch("mac_motors.monotonic", side_effect=lambda: now[0]):
            self.run_ik(samples())
        self.assertEqual(self.bus.positions[1], 2112)
        self.assertLess(self.bus.positions[6], 2080)
        self.assertLess(self.bus.positions[3], 2500)

    def test_persistent_tracking_error_stops(self):
        def samples():
            yield "2048,2048,2048,2048,2048,2048"
            self.bus.positions[2] += 87
            yield "2048,2048,2048,2048,2048,2048"
            yield "2048,2048,2048,2048,2048,2048"
        with patch("mac_motors.monotonic", side_effect=[0, 0, 0, 1.1]):
            with self.assertRaisesRegex(ValueError, "Motor 2 failed to track"):
                self.run_ik(samples())

    def test_shoulder_requires_open_hand_and_releases_torque(self):
        with patch("mac_motors.commands", return_value=["2", "open", "2", "52", "missing"]):
            run(self.bus, ik=False, travel=64, seconds=3, motion=True)
        torque = [v for j, reg, v in self.bus.writes if reg == "torque_enable"]
        self.assertEqual(torque, [1, 0])
        self.assertIn((1, "goal_position", 2064), self.bus.writes)


class MotorPipeTests(unittest.TestCase):
    def test_startup_backlog_waits_for_fresh_target(self):
        import os
        from mac_motors import commands

        r, w = os.pipe()
        with os.fdopen(r, "rb") as reader, os.fdopen(w, "wb", buffering=0) as writer:
            with patch("mac_motors.sys.stdin", reader):
                writer.write(b"first\n")
                stream = commands(None)
                self.assertEqual(next(stream), "first")
                writer.write(b"old1\nold2\n")
                # Supply a fresh frame only after the queued batch was read.
                read = os.read
                def read_and_send(fd, size):
                    data = read(fd, size)
                    if data.endswith(b"old2\n"):
                        writer.write(b"fresh\n")
                    return data
                with patch("mac_motors.os.read", side_effect=read_and_send):
                    self.assertEqual(next(stream), "fresh")
                writer.write(b"old\nHOLD\nPOSE\n")
                self.assertEqual(next(stream), "HOLD")
                self.assertEqual(next(stream), "POSE")
                self.assertIsNone(next(stream))
                writer.close()
                self.assertEqual(list(stream), [])
