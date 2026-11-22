"""Seat a pinned brass chime cylinder in a walnut music box well and close its maple resonator lid."""

from __future__ import annotations

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, ObjectState

SCENE = "station"
TOP = float(TABLE_TOP[SCENE])
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# Music box movement well centre on the station midline (metres)
FX = 0.250
FY = 0.000

# Case & component dimensions (metres)
BED_SIZE = (0.054, 0.054, 0.014)      # 54 x 54 x 14 mm brass bedplate inside the movement well
BED_TOP = TOP + BED_SIZE[2]
WALL_H = 0.034                        # 34 mm walnut case rim height (flush with seated cylinder top)
WALL_T = 0.012                        # 12 mm case wall thickness
RIM_TOP = TOP + WALL_H

CYL_R = 0.023                         # 48 mm diameter pinned brass chime cylinder (95 g)
CYL_H = 0.020                         # 20 mm tall (14 mm bedplate + 20 mm cylinder = 34 mm rim)
CYL_MASS = 0.095

LID_SIZE = (0.082, 0.064, 0.012)      # 82 x 64 x 12 mm maple resonator lid (60 g)
LID_MASS = 0.060

# Starting positions on left (cylinder) and right (lid) sides of the station table
CYL_START_XY = (0.220, 0.300)
LID_START_XY = (0.220, -0.300)

# Open-loop wrist compliance trims for YAM trajectory tracking
CYL_TRIM_XY = (0.0020, -0.0005)
LID_TRIM_XY = (0.0020, 0.0005)

# Colours (sRGB)
C_WALNUT = (0.36, 0.20, 0.11)
C_BRASS = (0.85, 0.68, 0.20)
C_MAPLE = (0.76, 0.52, 0.30)


def _build_objects() -> list[Box | Cylinder]:
    inner_half = BED_SIZE[0] / 2.0        # 0.027 m
    wall_offset = inner_half + WALL_T / 2.0  # 0.033 m
    outer_span = BED_SIZE[0] + 2.0 * WALL_T  # 0.078 m

    return [
        # Loose rigid bodies
        Cylinder(
            name="cylinder",
            radius=CYL_R,
            height=CYL_H,
            pos=(CYL_START_XY[0], CYL_START_XY[1], TOP + CYL_H / 2.0),
            color=C_BRASS,
            mass=CYL_MASS,
        ),
        Box(
            name="lid",
            size=LID_SIZE,
            pos=(LID_START_XY[0], LID_START_XY[1], TOP + LID_SIZE[2] / 2.0),
            color=C_MAPLE,
            mass=LID_MASS,
            yaw=math.pi / 2.0,
        ),
        # Fixed walnut music box housing with 14 mm brass movement bedplate
        Box(
            name="bedplate",
            size=BED_SIZE,
            pos=(FX, FY, TOP + BED_SIZE[2] / 2.0),
            color=C_BRASS,
        ),
        # Split left/right walls leave a 32 mm central finger-relief notch along x for clean vertical seating
        Box(
            name="wall_left_front",
            size=(0.021, WALL_T, WALL_H),
            pos=(FX - 0.0285, FY + wall_offset, TOP + WALL_H / 2.0),
            color=C_WALNUT,
        ),
        Box(
            name="wall_left_rear",
            size=(0.021, WALL_T, WALL_H),
            pos=(FX + 0.0285, FY + wall_offset, TOP + WALL_H / 2.0),
            color=C_WALNUT,
        ),
        Box(
            name="wall_right_front",
            size=(0.021, WALL_T, WALL_H),
            pos=(FX - 0.0285, FY - wall_offset, TOP + WALL_H / 2.0),
            color=C_WALNUT,
        ),
        Box(
            name="wall_right_rear",
            size=(0.021, WALL_T, WALL_H),
            pos=(FX + 0.0285, FY - wall_offset, TOP + WALL_H / 2.0),
            color=C_WALNUT,
        ),
        Box(
            name="wall_front",
            size=(WALL_T, BED_SIZE[1], WALL_H),
            pos=(FX - wall_offset, FY, TOP + WALL_H / 2.0),
            color=C_WALNUT,
        ),
        Box(
            name="wall_rear",
            size=(WALL_T, BED_SIZE[1], WALL_H),
            pos=(FX + wall_offset, FY, TOP + WALL_H / 2.0),
            color=C_WALNUT,
        ),
    ]


OBJECTS = _build_objects()


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


_SIGNS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)


def _box_corners(state: ObjectState, size: tuple[float, float, float]) -> np.ndarray:
    return np.asarray(state.pos, dtype=float) + (_SIGNS * (np.asarray(size, dtype=float) / 2.0)) @ _rot(state.quat_xyzw).T


def _cylinder_seated(cyl: ObjectState) -> bool:
    """Brass chime cylinder resting upright inside the 54 x 54 mm well on the 14 mm brass bedplate."""
    if not cyl.resting():
        return False
    cx, cy, cz = (float(v) for v in cyl.pos)
    if math.hypot(cx - FX, cy - FY) > 0.0045:
        return False
    ez = _rot(cyl.quat_xyzw)[:, 2]
    tilt_deg = math.degrees(math.acos(min(1.0, abs(float(ez[2])))))
    if tilt_deg > 3.0:
        return False
    bottom_z = cz - CYL_H / 2.0
    top_z = cz + CYL_H / 2.0
    return abs(bottom_z - BED_TOP) <= 0.002 and abs(top_z - RIM_TOP) <= 0.002


def _lid_closed(lid: ObjectState) -> bool:
    """Maple resonator lid resting flat across the 34 mm walnut case rim."""
    if not lid.resting():
        return False
    cx, cy, _ = (float(v) for v in lid.pos)
    if abs(cx - FX) > 0.004 or abs(cy - FY) > 0.004:
        return False
    ex = _rot(lid.quat_xyzw)[:, 0]
    yaw_err = math.degrees(math.acos(min(1.0, abs(float(ex[0])))))
    if yaw_err > 5.0:
        return False
    zs = np.sort(_box_corners(lid, LID_SIZE)[:, 2])
    bottom_ok = bool(np.all(np.abs(zs[:4] - RIM_TOP) <= 0.002))
    top_ok = bool(np.all(np.abs(zs[4:] - (RIM_TOP + LID_SIZE[2])) <= 0.002))
    return bottom_ok and top_ok


def score(objects: dict[str, ObjectState]) -> float:
    """0.0 unless cylinder is seated in the well; 0.5 for cylinder seated; 1.0 when lid closes flat over it."""
    if not _cylinder_seated(objects["cylinder"]):
        return 0.0
    return 1.0 if _lid_closed(objects["lid"]) else 0.5


def success(objects: dict[str, ObjectState]) -> bool:
    return score(objects) >= 1.0


TRAVEL = TOP + 0.145
CYL_GRIP_Z = TOP + 0.017
CYL_PLACE_Z = BED_TOP + 0.020
CYL_ENTRY = 0.014
LID_GRIP_Z = TOP + 0.016
LID_PLACE_Z = RIM_TOP + 0.018
LID_ENTRY = 0.014
SQUEEZE = 0.008
PARK_LEFT = (0.200, 0.300)
WAIT_RIGHT = (0.220, -0.260)


def _pick_place(
    pick: tuple[float, float, float],
    place: tuple[float, float, float],
    *,
    width: float,
    entry: float,
    travel_z: float,
    yaw: float = 0.0,
    place_yaw: float | None = None,
    pick_open: float = FINGER_OPEN,
    release_open: float = FINGER_OPEN,
) -> list[Waypoint]:
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2.0
    return [
        Waypoint(1.5, (px, py, travel_z), pick_open, "to the object", yaw),
        Waypoint(1.5, (px, py, pz), pick_open, "descend", yaw),
        Waypoint(0.6, (px, py, pz), pick_open, "settle", yaw),
        Waypoint(1.4, (px, py, pz), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel_z), grip, "lift", yaw),
        Waypoint(1.8, (qx, qy, travel_z), grip, "carry", end),
        Waypoint(0.6, (qx, qy, travel_z), grip, "settle over place", end),
        Waypoint(1.2, (qx, qy, qz + entry), grip, "lower to entry", end),
        Waypoint(0.7, (qx, qy, qz + entry), grip, "settle at entry", end),
        Waypoint(1.8, (qx, qy, qz), grip, "set in", end),
        Waypoint(0.7, (qx, qy, qz), grip, "settle", end),
        Waypoint(1.4, (qx, qy, qz), release_open, "release", end),
        Waypoint(1.0, (qx, qy, travel_z), release_open, "retreat", end),
    ]


def demo() -> dict[str, list[Waypoint]]:
    """Left arm seats the brass chime cylinder in the well; right arm closes the maple resonator lid."""
    left_cyl = _pick_place(
        (CYL_START_XY[0], CYL_START_XY[1], CYL_GRIP_Z),
        (FX + CYL_TRIM_XY[0], FY + CYL_TRIM_XY[1], CYL_PLACE_Z),
        width=2.0 * CYL_R,
        entry=CYL_ENTRY,
        travel_z=TRAVEL,
        yaw=0.0,
        place_yaw=0.0,
    )
    left_plan = left_cyl + [
        Waypoint(1.4, (PARK_LEFT[0], PARK_LEFT[1], TRAVEL), FINGER_OPEN, "park left", 0.0),
    ]
    left_s = sum(wp.duration for wp in left_plan)

    right_lid = _pick_place(
        (LID_START_XY[0], LID_START_XY[1], LID_GRIP_Z),
        (FX + LID_TRIM_XY[0], FY + LID_TRIM_XY[1], LID_PLACE_Z),
        width=LID_SIZE[1],
        entry=LID_ENTRY,
        travel_z=TRAVEL,
        yaw=-math.pi / 2.0,
        place_yaw=0.0,
    )
    right_plan = [
        Waypoint(1.5, (WAIT_RIGHT[0], WAIT_RIGHT[1], TRAVEL), FINGER_OPEN, "to wait", 0.0),
        Waypoint(max(0.5, left_s - 1.5), (WAIT_RIGHT[0], WAIT_RIGHT[1], TRAVEL), FINGER_OPEN, "wait for left", 0.0),
    ] + right_lid

    return {"left": left_plan, "right": right_plan}


def _make_state(
    pos: tuple[float, float, float],
    yaw_deg: float = 0.0,
    pitch_deg: float = 0.0,
    roll_deg: float = 0.0,
    lin_vel: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ang_vel: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> ObjectState:
    cy, sy = math.cos(math.radians(yaw_deg) / 2.0), math.sin(math.radians(yaw_deg) / 2.0)
    cp, sp = math.cos(math.radians(pitch_deg) / 2.0), math.sin(math.radians(pitch_deg) / 2.0)
    cr, sr = math.cos(math.radians(roll_deg) / 2.0), math.sin(math.radians(roll_deg) / 2.0)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return ObjectState(
        pos=pos,
        quat_xyzw=(qx, qy, qz, qw),
        lin_vel=lin_vel,
        ang_vel=ang_vel,
        low=(0.0, 0.0, 0.0),
        high=(0.0, 0.0, 0.0),
    )


def self_test() -> int:
    cyl_start = _make_state((CYL_START_XY[0], CYL_START_XY[1], TOP + CYL_H / 2.0))
    lid_start = _make_state((LID_START_XY[0], LID_START_XY[1], TOP + LID_SIZE[2] / 2.0), yaw_deg=90.0)
    cyl_ok = _make_state((FX, FY, BED_TOP + CYL_H / 2.0))
    lid_ok = _make_state((FX, FY, RIM_TOP + LID_SIZE[2] / 2.0))

    cases: list[tuple[str, dict[str, ObjectState], float, bool]] = [
        ("start_state", {"cylinder": cyl_start, "lid": lid_start}, 0.0, False),
        ("cylinder_seated_only", {"cylinder": cyl_ok, "lid": lid_start}, 0.5, False),
        ("cylinder_and_lid_closed", {"cylinder": cyl_ok, "lid": lid_ok}, 1.0, True),
        ("lid_reversed_180_valid", {"cylinder": cyl_ok, "lid": _make_state((FX, FY, RIM_TOP + LID_SIZE[2] / 2.0), yaw_deg=180.0)}, 1.0, True),
        ("lid_upside_down_valid", {"cylinder": cyl_ok, "lid": _make_state((FX, FY, RIM_TOP + LID_SIZE[2] / 2.0), roll_deg=180.0)}, 1.0, True),
        # Precedence & failure modes
        ("lid_closed_without_cylinder", {"cylinder": cyl_start, "lid": lid_ok}, 0.0, False),
        ("cylinder_on_table_outside_box", {"cylinder": _make_state((0.250, 0.140, TOP + CYL_H / 2.0)), "lid": lid_ok}, 0.0, False),
        ("cylinder_cocked_on_wall_20mm_proud", {"cylinder": _make_state((FX, FY, RIM_TOP + CYL_H / 2.0)), "lid": _make_state((FX, FY, RIM_TOP + CYL_H + LID_SIZE[2] / 2.0), pitch_deg=15.0)}, 0.0, False),
        ("cylinder_tilted_5deg", {"cylinder": _make_state((FX, FY, BED_TOP + CYL_H / 2.0 + 0.002), pitch_deg=5.0), "lid": lid_ok}, 0.0, False),
        ("cylinder_shifted_6mm", {"cylinder": _make_state((FX + 0.006, FY, BED_TOP + CYL_H / 2.0)), "lid": lid_ok}, 0.0, False),
        ("cylinder_moving", {"cylinder": _make_state((FX, FY, BED_TOP + CYL_H / 2.0), lin_vel=(0.0, 0.0, 0.02)), "lid": lid_ok}, 0.0, False),
        ("lid_tilted_4deg", {"cylinder": cyl_ok, "lid": _make_state((FX, FY, RIM_TOP + LID_SIZE[2] / 2.0 + 0.003), pitch_deg=4.0)}, 0.5, False),
        ("lid_yawed_8deg", {"cylinder": cyl_ok, "lid": _make_state((FX, FY, RIM_TOP + LID_SIZE[2] / 2.0), yaw_deg=8.0)}, 0.5, False),
        ("lid_shifted_6mm", {"cylinder": cyl_ok, "lid": _make_state((FX + 0.006, FY, RIM_TOP + LID_SIZE[2] / 2.0))}, 0.5, False),
        ("lid_held_8mm_above_rim", {"cylinder": cyl_ok, "lid": _make_state((FX, FY, RIM_TOP + LID_SIZE[2] / 2.0 + 0.008))}, 0.5, False),
        ("lid_moving", {"cylinder": cyl_ok, "lid": _make_state((FX, FY, RIM_TOP + LID_SIZE[2] / 2.0), ang_vel=(0.0, 0.0, 0.8))}, 0.5, False),
    ]

    for label, state_map, want_score, want_ok in cases:
        got_score = score(state_map)
        got_ok = success(state_map)
        assert math.isclose(got_score, want_score, abs_tol=1e-6) and got_ok == want_ok, (
            label,
            got_score,
            got_ok,
            want_score,
            want_ok,
        )
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
