"""macOS SO-101 motor bridge. The parent supplies palm or IK commands on stdin."""
import argparse
from contextlib import contextmanager
import json
import math
import os
import select
import sys
from time import monotonic



def controller_port(port=None):
    if port:
        return port
    from serial.tools.list_ports import comports

    ports = [p.device for p in comports() if (p.vid, p.pid) == (0x1A86, 0x55D3)]
    if len(ports) != 1:
        raise SystemExit("Specify --port /dev/cu.… (expected one SO-101 USB serial adapter).")
    return ports[0]


@contextmanager
def motor_bus(port):
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    bus = SO101Follower(SO101FollowerConfig(port=port)).bus
    bus._connect(handshake=False)
    try:
        yield bus._connected_bus
    finally:
        bus._close()


def read(bus, joint, register):
    value, status = bus.read_register_with_error(joint, register)
    if status:
        raise RuntimeError(f"Motor {joint} reported status {status:#x} reading {register}.")
    return value


def write(bus, joint, register, value):
    status = bus.write_register_with_error(joint, register, value)
    if status:
        raise RuntimeError(f"Motor {joint} reported status {status:#x} writing {register}.")


def starting_pose(bus, joints, travel, ik):
    home = {}
    torque_states = set()
    for joint in joints:
        position = read(bus, joint, "present_position")
        torque = read(bus, joint, "torque_enable")
        print(f"Motor {joint}: position {position}, torque {torque}", flush=True, file=sys.stderr)
        if joint != 6:
            torque_states.add(torque)
        # A previous IK session leaves all five joints holding. Attach only
        # to a settled pose, with each measured position close to its goal.
        holding = ik and torque == 1 and abs(position - read(bus, joint, "goal_position")) <= 10
        margin = 0 if ik else travel
        if (read(bus, joint, "model") != 777 or read(bus, joint, "mode") != 0
                or (torque != 0 and not holding) or read(bus, joint, "present_speed") != 0
                or not 0 <= read(bus, joint, "min_position_limit") <= position - margin
                or not position + margin <= read(bus, joint, "max_position_limit") <= 4095):
            raise ValueError(f"Motor {joint} failed the stationary starting-pose check.")
        home[joint] = position
    if len(torque_states) != 1:
        raise ValueError("Arm has mixed torque states; cannot start hand control.")
    return home


def commands(seconds):
    """Consume pipe input without read-ahead; yield None on a tracking timeout."""
    end = monotonic() + seconds if seconds else math.inf
    pending = b""
    while monotonic() < end:
        deadline = min(end, monotonic() + 0.45)
        while True:
            remaining = deadline - monotonic()
            if remaining <= 0 or not select.select([sys.stdin], [], [], remaining)[0]:
                yield None
                break
            chunk = os.read(sys.stdin.fileno(), 4096)
            if not chunk:
                return
            lines = (pending + chunk).split(b"\n")
            pending = lines.pop()
            if len(lines) == 1:
                yield lines[0].decode("ascii")
                break
            if len(lines) > 1:
                # Motor setup can span several tracking frames. Drop queued
                # movement, preserve control transitions, then wait for a fresh
                # target. A batch of positions is not a missed deadline.
                controls = [line for line in lines if line in (b"HOLD", b"POSE", b"open", b"missing")]
                if controls:
                    for control in controls:
                        yield control.decode("ascii")
                    break
                # Keep the same deadline while discarding queued targets.


def clamp(value, low, high):
    return min(high, max(low, value))


def run(bus, *, ik, travel, seconds, motion):
    joints = range(1, 7 if ik else 2)
    home = starting_pose(bus, joints, travel, ik)
    if not motion:
        return
    limits = {}
    lag_since = {}
    blocked_since = {}
    lag_reference = {}
    previous_shoulder = None
    enabled = set()
    last_report = 0.0
    following = False
    armed = False
    goals = home.copy()

    def stop():
        failures = []
        for joint in sorted(enabled):
            try:
                if ik:
                    position = read(bus, joint, "present_position")
                    position = clamp(position, *limits[joint])
                    write(bus, joint, "goal_position", position)
                    goals[joint] = position
                else:
                    write(bus, joint, "torque_enable", 0)
            except Exception as error:
                failures.append(str(error))
        if failures:
            raise RuntimeError("Motor stop failed: " + ", ".join(failures))

    try:
        for command in commands(seconds):
            if ik and command == "POSE":
                pose = []
                for j in joints:
                    low = read(bus, j, "min_position_limit")
                    high = read(bus, j, "max_position_limit")
                    position = read(bus, j, "present_position")
                    if not (0 <= low < high <= 4095 and low <= position <= high):
                        raise ValueError(f"Motor {j} has an invalid calibrated range or position.")
                    pose.append((position, low, high))
                print(json.dumps(pose), flush=True)
                continue
            if command in (None, "HOLD", "open", "missing"):
                if following:
                    stop()
                    if not ik:
                        enabled.clear()
                following = False
                lag_since.clear()
                blocked_since.clear()
                lag_reference.clear()
                previous_shoulder = None
                armed = command == "open"
                # After a broken input stream require a fresh control session.
                if command is None and enabled:
                    raise RuntimeError("Stopped: hand controller missed its motor update deadline.")
                continue
            if ik:
                targets = [int(value) for value in command.split(",")]
                if len(targets) != len(joints) or any(not 0 <= v <= 4095 for v in targets):
                    raise ValueError("Invalid encoder targets.")
            else:
                x = float(command)
                if not math.isfinite(x) or not armed:
                    continue
            if not enabled:
                checked = starting_pose(bus, joints, travel, ik)
                if ik:
                    home = checked
                    limits = {j: (read(bus, j, "min_position_limit"),
                                  read(bus, j, "max_position_limit")) for j in joints}
                elif abs(checked[1] - home[1]) > travel:
                    raise ValueError("Shoulder moved outside the starting range.")
                goals = checked.copy()
                for joint in joints:
                    if joint == 6:
                        # Volatile STS torque limit: cap effort for the jaw only.
                        limit = read(bus, joint, "torque_limit")
                        write(bus, joint, "torque_limit", min(limit, 150))
                    write(bus, joint, "acceleration", 4)
                    write(bus, joint, "goal_time", 0)
                    write(bus, joint, "goal_speed", 400 if joint == 6 else (220 if ik else 90))
                    write(bus, joint, "goal_position", checked[joint])
                for joint in joints:
                    enabled.add(joint)
                    write(bus, joint, "torque_enable", 1)
                if not ik:
                    origin, pinch_position = x, checked[1]
            following = True
            measured = [read(bus, joint, "present_position") for joint in joints]
            # Give the loaded shoulder time to lift before advancing the other
            # arm joints. The jaw remains independently controlled.
            lifting = ik and targets[1] < goals[2]
            waiting_for_lift = (lifting and measured[1] - goals[2] >= 18
                                and previous_shoulder is not None and measured[1] >= previous_shoulder)
            if ik:
                previous_shoulder = measured[1]
            next_goals = {}
            for joint, actual in zip(joints, measured):
                if ik:
                    low, high = limits[joint]
                    if not low <= actual <= high:
                        raise ValueError(f"Motor {joint} at {actual} outside configured limits {low}–{high}.")
                    # A transient shift from home is fine. A joint that persistently
                    # cannot hold its commanded pose needs the operator's attention.
                    target = clamp(targets[joint - 1], low, high)
                    # Time only a lag that the current hand target still asks
                    # us to close. On reversal the bounded ramp retires the
                    # previous goal over the next updates.
                    same_direction = (target - actual) * (goals[joint] - actual) > 0
                    if abs(actual - goals[joint]) > 20 and abs(target - actual) > 20 and same_direction:
                        now = monotonic()
                        # A moving servo can trail a moving goal by a constant
                        # distance. Time lack of measured progress toward it.
                        direction = 1 if goals[joint] > actual else -1
                        origin, prior_direction = lag_reference.get(joint, (actual, direction))
                        if (joint not in lag_reference or direction != prior_direction
                                or direction * (actual - origin) >= 3):
                            lag_reference[joint] = actual, direction
                            lag_since[joint] = now
                        if now - lag_since[joint] >= 1.0:
                            raise ValueError(f"Motor {joint} failed to track goal {goals[joint]} "
                                             f"for one second (measured {actual}).")
                    else:
                        lag_since.pop(joint, None)
                        lag_reference.pop(joint, None)
                elif abs(actual - home[joint]) > travel + 8:
                    raise ValueError(f"Motor {joint} exceeded the test range: start {home[joint]}, "
                                     f"last goal {goals[joint]}, measured {actual}.")
                if not ik:
                    target = pinch_position + round(clamp((x - origin) / 50, -1, 1) * travel)
                    target = clamp(target, home[joint] - travel, home[joint] + travel)
                step = 36 if joint == 6 else (10 if ik else 16)
                if ik and joint != 6 and abs(target - goals[joint]) > 150:
                    step = 18
                previous = goals[joint]
                goal = previous + clamp(target - previous, -step, step)
                # Allow a partial step when feedback trails by even one tick.
                # Never shift a stationary goal just because the arm sagged.
                lead = 60 if joint == 6 else (18 if ik else step)
                if ik and (joint == 3 or (joint == 2 and lifting)):
                    # Ramp by the same 10 ticks/update, with up to ~3 degrees
                    # of position error available to the position servo under load.
                    lead = 36
                if goal > previous:
                    goal = max(previous, min(goal, actual + lead))
                elif goal < previous:
                    goal = min(previous, max(goal, actual - lead))
                # At the lead limit the old 20-tick lag check could never
                # trip for an arm joint (18 ticks), even with a distant target.
                if ik and goal == previous and target != previous:
                    now = monotonic()
                    direction = 1 if target > previous else -1
                    old_direction, since = blocked_since.get(joint, (direction, now))
                    if direction != old_direction:
                        since = now
                    blocked_since[joint] = direction, since
                    if now - since >= 1.0:
                        raise ValueError(f"Motor {joint} stalled at {actual} while tracking {target}.")
                else:
                    blocked_since.pop(joint, None)
                if waiting_for_lift and joint in (1, 3, 4, 5):
                    continue
                if goal != previous:
                    next_goals[joint] = goal
            if next_goals:
                if ik:
                    bus.sync_write_register(list(next_goals), "goal_position", list(next_goals.values()))
                else:
                    write(bus, 1, "goal_position", next_goals[1])
                goals.update(next_goals)
            if ik and monotonic() - last_report >= 1:
                print(f"Palm: target {targets}, commanded {list(goals.values())}, measured {measured}",
                      file=sys.stderr, flush=True)
                last_report = monotonic()
    finally:
        if enabled:
            stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("ik", "pan"))
    parser.add_argument("--port")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--seconds", type=int, choices=range(1, 61), default=25)
    parser.add_argument("--travel-deg", type=int, choices=range(1, 19), default=9)
    args = parser.parse_args()
    ik = args.mode == "ik"
    port = controller_port(args.port)
    print(f"Checking {port}", flush=True, file=sys.stderr)
    try:
        with motor_bus(port) as bus:
            run(bus, ik=ik, travel=None if ik else args.travel_deg * 4096 // 360,
                seconds=None if ik else args.seconds, motion=args.enable_motion)
    except (RuntimeError, ValueError, OSError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
