"""Position-only SO-101 inverse kinematics using the vendored follower URDF."""
from dataclasses import dataclass
from math import cos, isfinite, sin, sqrt
from pathlib import Path
from xml.etree import ElementTree

MODEL = Path(__file__).resolve().parent / "viewer/models/SO101/so101_new_calib.urdf"
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
IDENTITY = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
AXIS = (0, 0, 1)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def apply(rotation, vector):
    return tuple(dot(row, vector) for row in rotation)


def multiply(a, b):
    columns = tuple(zip(*b))
    return tuple(tuple(dot(row, column) for column in columns) for row in a)


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def subtract(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def z_rotation(angle):
    c, s = cos(angle), sin(angle)
    return ((c, -s, 0), (s, c, 0), AXIS)


def urdf_rotation(roll, pitch, yaw):
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    pitch_roll = ((cp, sp * sr, sp * cr), (0, cr, -sr), (-sp, cp * sr, cp * cr))
    return multiply(z_rotation(yaw), pitch_roll)


def solve_linear(matrix, vector):
    rows = [list(row) + [value] for row, value in zip(matrix, vector)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda i: abs(rows[i][column]))
        rows[column], rows[pivot] = rows[pivot], rows[column]
        scale = rows[column][column]
        rows[column] = [value / scale for value in rows[column]]
        for i, row in enumerate(rows):
            if i != column:
                factor = row[column]
                rows[i] = [a - factor * b for a, b in zip(row, rows[column])]
    return tuple(row[-1] for row in rows)


@dataclass(frozen=True)
class Joint:
    translation: tuple
    rotation: tuple
    lower: float
    upper: float


class ArmModel:
    def __init__(self, model_path=MODEL):
        robot = ElementTree.parse(model_path).getroot()
        joints = {element.get("name"): element for element in robot.findall("joint")}
        self.joints = []
        for name in ARM_JOINTS + ("gripper_frame_joint",):
            joint = joints[name]
            origin = joint.find("origin")
            xyz = tuple(map(float, origin.get("xyz").split()))
            rotation = urdf_rotation(*map(float, origin.get("rpy").split()))
            limit = joint.find("limit")
            if limit is None:
                self.tool_offset = xyz
                continue
            if tuple(map(float, joint.find("axis").get("xyz").split())) != AXIS:
                raise ValueError(f"Unexpected axis for {name}")
            self.joints.append(Joint(xyz, rotation, float(limit.get("lower")), float(limit.get("upper"))))

    def forward(self, angles):
        point = (0, 0, 0)
        rotation = IDENTITY
        points = [point]
        axes = []
        for joint, angle in zip(self.joints, angles):
            point = add(point, apply(rotation, joint.translation))
            rotation = multiply(rotation, joint.rotation)
            axes.append(apply(rotation, AXIS))
            points.append(point)
            rotation = multiply(rotation, z_rotation(angle))
        tool = add(point, apply(rotation, self.tool_offset))
        return points + [tool], axes

    def solve(self, target, seed, joint_window=None, joint_bounds=None):
        angles = list(seed)
        bounds = [(joint.lower, joint.upper) for joint in self.joints]
        if joint_bounds is not None:
            bounds = joint_bounds
        if joint_window is not None:
            bounds = [(max(lower, -joint_window), min(upper, joint_window)) for lower, upper in bounds]
        best_angles = angles[:]
        best_error = float("inf")
        for _ in range(28):
            points, axes = self.forward(angles)
            error = subtract(target, points[-1])
            remaining = sqrt(dot(error, error))
            if remaining < best_error:
                best_error, best_angles = remaining, angles[:]
            if remaining <= 0.002:
                break
            columns = [cross(axis, subtract(points[-1], pivot)) for axis, pivot in zip(axes, points[1:])]
            columns = [column if lo < hi else (0, 0, 0)
                       for column, (lo, hi) in zip(columns, bounds)]
            rows = tuple(zip(*columns))
            normal = [
                [dot(a, b) + (0.018 ** 2 if i == j else 0) for j, b in enumerate(rows)]
                for i, a in enumerate(rows)
            ]
            correction = solve_linear(normal, error)
            for i, (column, (lower, upper)) in enumerate(zip(columns, bounds)):
                step = max(-0.12, min(0.12, dot(column, correction)))
                angles[i] = max(lower, min(upper, angles[i] + step))
        return best_angles, best_error


def palm_displacement(palm, origin):
    """Map millimetres of hand travel to metres in the arm's coordinate frame."""
    dx, dy, dz = subtract(palm, origin)
    return (dx / 1000, -dz / 1000, dy / 1000)


class PalmIK:
    """Track one palm continuously; pinching operates the jaw independently."""

    def __init__(self, model):
        self.model = model
        self.angles = [0.0] * len(ARM_JOINTS)
        self.anchor = None
        self.joint_bounds = None
        self.latest = self.state()

    def state(self, target=None, residual=None, tracking=False):
        points, _ = self.model.forward(self.angles)
        return {
            "joints": dict(zip(ARM_JOINTS, self.angles)),
            "points": points,
            "target": target,
            "error_mm": None if residual is None else residual * 1000,
            "tracking": tracking,
            "active": target is not None,
        }

    def accept(self, sample):
        if sample is None:
            self.anchor = None
            self.latest = self.state(tracking=sample is not None)
            return
        palm = (sample.x_mm, sample.y_mm, sample.z_mm)
        if not all(map(isfinite, palm)):
            self.accept(None)
            return
        if self.anchor is None:
            points, _ = self.model.forward(self.angles)
            self.anchor = (palm, points[-1])
        initial_palm, initial_tool = self.anchor
        # The neutral tool points along base +X. Facing forward with the
        # arm, right is -Y, forward is +X and upward is +Z.
        dx, dy, dz = subtract(palm, initial_palm)
        target = add(initial_tool, (-dz / 1000, -dx / 1000, dy / 1000))
        self.angles, residual = self.model.solve(target, self.angles, joint_bounds=self.joint_bounds)
        self.latest = self.state(target, residual, tracking=True)
        self.latest["joints"]["gripper"] = 0.0 if sample.pinched else 1.5
