"""Seat a blue 1-gang old-work electrical box flush in a drywall cutout and lock both swing-out clamps."""

from __future__ import annotations

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, ObjectState

SCENE = "station"
TOP = float(TABLE_TOP[SCENE])
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# Cutout pocket centre on the station midline (metres)
FX = 0.250
FY = 0.000

# Part dimensions (metres)
BOX = (0.100, 0.060, 0.016)       # 100 x 60 x 16 mm blue PVC 1-gang old-work box (95 g)
BOX_MASS = 0.095
CLAMP = (0.060, 0.016, 0.016)     # 60 x 16 x 16 mm orange swing-out mounting clamp (10 g)
CLAMP_MASS = 0.010
CLEAR = 0.003                     # 3 mm cutout clearance per side
COLLAR_H = 0.005                  # 5 mm cutout collar wall height
COLLAR_T = 0.010                  # 10 mm cutout collar wall thickness
FLANGE = (0.160, 0.080, 0.016)    # 160 x 80 x 16 mm drywall mounting flange (flush with seated box top)
FLANGE_Y = BOX[1] / 2 + CLEAR + COLLAR_T + FLANGE[1] / 2  # 0.083 m
FLANGE_TOP = TOP + FLANGE[2]
RIB_H = 0.010                     # 10 mm alignment/lock rib height above flange

# Key positions
BOX_START_XY = (0.220, 0.320)
STOW_Y = 0.103
SLOT_PLUS_X = 0.222
SLOT_MINUS_X = 0.278
SLOT_RIB_Y = 0.061
SLOT_OUTER_Y = SLOT_RIB_Y + 0.005  # 0.066 m
CLAMP_GOAL_Y = 0.036               # spans y = 0.006..0.066 m (24 mm on box, 13 mm bridge, 23 mm on flange)

# Colours (sRGB)
C_BOX = (0.14, 0.42, 0.78)
C_CLAMP = (0.95, 0.52, 0.08)
C_DRYWALL = (0.76, 0.74, 0.69)
C_COLLAR = (0.36, 0.39, 0.43)


def _build_objects() -> list[Box]:
    _cy = BOX[1] / 2 + CLEAR + COLLAR_T / 2  # 0.038 m
    _cx = BOX[0] / 2 + CLEAR + COLLAR_T / 2  # 0.058 m
    rib_z = FLANGE_TOP + RIB_H / 2

    objs: list[Box] = [
        Box(
            name="box",
            size=BOX,
            pos=(BOX_START_XY[0], BOX_START_XY[1], TOP + BOX[2] / 2),
            color=C_BOX,
            mass=BOX_MASS,
            yaw=math.pi / 2,
        ),
        Box(
            name="clamp_plus",
            size=CLAMP,
            pos=(FX, STOW_Y, FLANGE_TOP + CLAMP[2] / 2),
            color=C_CLAMP,
            mass=CLAMP_MASS,
        ),
        Box(
            name="clamp_minus",
            size=CLAMP,
            pos=(FX, -STOW_Y, FLANGE_TOP + CLAMP[2] / 2),
            color=C_CLAMP,
            mass=CLAMP_MASS,
        ),
        # Fixed cutout collar ring (106 x 66 mm inner pocket around 100 x 60 mm box)
        Box(name="collar_plus", size=(BOX[0] + 2 * (CLEAR + COLLAR_T), COLLAR_T, COLLAR_H), pos=(FX, _cy, TOP + COLLAR_H / 2), color=C_COLLAR),
        Box(name="collar_minus", size=(BOX[0] + 2 * (CLEAR + COLLAR_T), COLLAR_T, COLLAR_H), pos=(FX, -_cy, TOP + COLLAR_H / 2), color=C_COLLAR),
        Box(name="collar_front", size=(COLLAR_T, BOX[1] + 2 * CLEAR, COLLAR_H), pos=(FX - _cx, FY, TOP + COLLAR_H / 2), color=C_COLLAR),
        Box(name="collar_rear", size=(COLLAR_T, BOX[1] + 2 * CLEAR, COLLAR_H), pos=(FX + _cx, FY, TOP + COLLAR_H / 2), color=C_COLLAR),
        # Fixed left and right drywall mounting flanges
        Box(name="flange_plus", size=FLANGE, pos=(FX, FLANGE_Y, TOP + FLANGE[2] / 2), color=C_DRYWALL),
        Box(name="flange_minus", size=FLANGE, pos=(FX, -FLANGE_Y, TOP + FLANGE[2] / 2), color=C_DRYWALL),
    ]

    # Stowed clamp keyways on outer flanges (4 ribs per side, 20 mm inner y-gap)
    for sign, tag in ((1.0, "plus"), (-1.0, "minus")):
        for idx, (rx, ry) in enumerate(
            (
                (FX - 0.027, sign * (STOW_Y - 0.012)),
                (FX - 0.027, sign * (STOW_Y + 0.012)),
                (FX + 0.027, sign * (STOW_Y - 0.012)),
                (FX + 0.027, sign * (STOW_Y + 0.012)),
            )
        ):
            objs.append(
                Box(
                    name=f"keyway_{tag}_{idx}",
                    size=(0.010, 0.004, RIB_H),
                    pos=(rx, ry, rib_z),
                    color=C_COLLAR,
                )
            )

    # Staggered clamp-lock pocket ribs (inner span 26 mm, y = +-0.056..+-0.066 m)
    objs.extend(
        [
            Box(name="slot_plus_a", size=(0.004, 0.010, RIB_H), pos=(SLOT_PLUS_X - 0.015, SLOT_RIB_Y, rib_z), color=C_COLLAR),
            Box(name="slot_plus_b", size=(0.004, 0.010, RIB_H), pos=(SLOT_PLUS_X + 0.015, SLOT_RIB_Y, rib_z), color=C_COLLAR),
            Box(name="slot_minus_a", size=(0.004, 0.010, RIB_H), pos=(SLOT_MINUS_X - 0.015, -SLOT_RIB_Y, rib_z), color=C_COLLAR),
            Box(name="slot_minus_b", size=(0.004, 0.010, RIB_H), pos=(SLOT_MINUS_X + 0.015, -SLOT_RIB_Y, rib_z), color=C_COLLAR),
        ]
    )
    return objs


OBJECTS = _build_objects()


def _quat_to_rot(q: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], ...]:
    x, y, z, w = (float(v) for v in q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _corners(state: ObjectState, size: tuple[float, float, float]) -> list[tuple[float, float, float]]:
    rot = _quat_to_rot(tuple(state.quat_xyzw))
    cx, cy, cz = (float(v) for v in state.pos)
    hx, hy, hz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    pts: list[tuple[float, float, float]] = []
    for sx in (-hx, hx):
        for sy in (-hy, hy):
            for sz in (-hz, hz):
                wx = cx + rot[0][0] * sx + rot[0][1] * sy + rot[0][2] * sz
                wy = cy + rot[1][0] * sx + rot[1][1] * sy + rot[1][2] * sz
                wz = cz + rot[2][0] * sx + rot[2][1] * sy + rot[2][2] * sz
                pts.append((wx, wy, wz))
    return pts


def _long_axis_xy(state: ObjectState) -> tuple[float, float]:
    rot = _quat_to_rot(tuple(state.quat_xyzw))
    ax, ay = rot[0][0], rot[1][0]
    norm = math.hypot(ax, ay)
    if norm < 1e-6:
        return (1.0, 0.0)
    return (ax / norm, ay / norm)


def _flat_tilt_deg(state: ObjectState) -> float:
    rot = _quat_to_rot(tuple(state.quat_xyzw))
    max_align = max(abs(rot[2][0]), abs(rot[2][1]), abs(rot[2][2]))
    return math.degrees(math.acos(min(1.0, max(0.0, max_align))))


def _box_seated(box: ObjectState) -> bool:
    if not box.resting():
        return False
    cx, cy, _ = (float(v) for v in box.pos)
    if abs(cx - FX) > 0.004 or abs(cy - FY) > 0.004:
        return False
    ax, _ = _long_axis_xy(box)
    yaw_err = math.degrees(math.acos(min(1.0, abs(ax))))
    if yaw_err > 3.0:
        return False
    zs = sorted(p[2] - TOP for p in _corners(box, BOX))
    if any(abs(z) > 0.002 for z in zs[:4]):
        return False
    if any(abs(z - BOX[2]) > 0.002 for z in zs[4:]):
        return False
    return True


def _clamp_locked(clamp: ObjectState, box: ObjectState, slot_x: float, side_sign: float) -> bool:
    if not clamp.resting():
        return False
    if _flat_tilt_deg(clamp) > 3.0:
        return False
    _, ay = _long_axis_xy(clamp)
    yaw_err = math.degrees(math.acos(min(1.0, abs(ay))))
    if yaw_err > 10.0:
        return False
    cx = float(clamp.pos[0])
    if abs(cx - slot_x) > 0.004:
        return False

    pts = _corners(clamp, CLAMP)
    min_z = min(p[2] - TOP for p in pts)
    if abs(min_z - FLANGE[2]) > 0.002:
        return False

    ys = [p[1] for p in pts]
    if side_sign > 0:
        outer_y = max(ys)
        inner_y = min(ys)
        if abs(outer_y - SLOT_OUTER_Y) > 0.004:
            return False
        box_edge_y = max(p[1] for p in _corners(box, BOX))
        overlap = box_edge_y - inner_y
    else:
        outer_y = min(ys)
        inner_y = max(ys)
        if abs(outer_y + SLOT_OUTER_Y) > 0.004:
            return False
        box_edge_y = min(p[1] for p in _corners(box, BOX))
        overlap = inner_y - box_edge_y

    return overlap >= 0.018


def score(objects: dict[str, ObjectState]) -> float:
    box = objects["box"]
    if not _box_seated(box):
        return 0.0

    clamps = (objects["clamp_plus"], objects["clamp_minus"])
    plus_filled = any(_clamp_locked(c, box, SLOT_PLUS_X, 1.0) for c in clamps)
    minus_filled = any(_clamp_locked(c, box, SLOT_MINUS_X, -1.0) for c in clamps)
    filled_count = int(plus_filled) + int(minus_filled)

    if filled_count == 2:
        return 1.0
    if filled_count == 1:
        return 0.7
    return 0.4


def success(objects: dict[str, ObjectState]) -> bool:
    return score(objects) >= 1.0


TRAVEL = TOP + 0.150
BOX_GRIP_Z = TOP + 0.018
BOX_PLACE_Z = TOP + 0.024
BOX_ENTRY = 0.012
BOX_RELEASE = -0.036
BOX_TRIM = (0.0020, -0.0005, math.radians(-2.9))
CLAMP_GRIP_Z = FLANGE_TOP + 0.018
CLAMP_PLACE_Z = FLANGE_TOP + 0.024
CLAMP_ENTRY = 0.015
CLAMP_OPEN = -0.020
SQUEEZE = 0.008
PARK_LEFT = (0.200, 0.300)
WAIT_RIGHT = (0.220, -0.260)


def _pick_place(pick, place, *, width, entry, travel_z, yaw=0.0, place_yaw=None, pick_open=FINGER_OPEN, release_open=FINGER_OPEN):
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2
    return [
        Waypoint(1.5, (px, py, travel_z), pick_open, "to the object", yaw),
        Waypoint(1.5, (px, py, pz), pick_open, "descend", yaw),
        Waypoint(0.6, (px, py, pz), pick_open, "settle", yaw),
        Waypoint(1.5, (px, py, pz), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel_z), grip, "lift", yaw),
        Waypoint(2.0, (qx, qy, travel_z), grip, "carry", end),
        Waypoint(0.6, (qx, qy, travel_z), grip, "settle over the place", end),
        Waypoint(1.2, (qx, qy, qz + entry), grip, "lower to the entry", end),
        Waypoint(0.8, (qx, qy, qz + entry), grip, "settle at the entry", end),
        Waypoint(2.0, (qx, qy, qz), grip, "set in", end),
        Waypoint(0.8, (qx, qy, qz), grip, "settle", end),
        Waypoint(1.5, (qx, qy, qz), release_open, "release", end),
        Waypoint(1.0, (qx, qy, travel_z), release_open, "retreat", end),
    ]


def demo() -> dict[str, list[Waypoint]]:
    left_box = _pick_place(
        (BOX_START_XY[0], BOX_START_XY[1], BOX_GRIP_Z),
        (FX + BOX_TRIM[0], FY + BOX_TRIM[1], BOX_PLACE_Z),
        width=BOX[1],
        entry=BOX_ENTRY,
        travel_z=TRAVEL,
        yaw=-math.pi / 2,
        place_yaw=BOX_TRIM[2],
        release_open=BOX_RELEASE,
    )
    left_clamp = _pick_place(
        (FX, STOW_Y, CLAMP_GRIP_Z),
        (SLOT_PLUS_X, CLAMP_GOAL_Y, CLAMP_PLACE_Z),
        width=CLAMP[1],
        entry=CLAMP_ENTRY,
        travel_z=TRAVEL,
        yaw=0.0,
        place_yaw=-math.pi / 2,
        pick_open=CLAMP_OPEN,
        release_open=CLAMP_OPEN,
    )
    left_plan = left_box + left_clamp + [
        Waypoint(1.5, (PARK_LEFT[0], PARK_LEFT[1], TRAVEL), CLAMP_OPEN, "park left", -math.pi / 2),
    ]
    left_s = sum(wp.duration for wp in left_plan)

    right_clamp = _pick_place(
        (FX, -STOW_Y, CLAMP_GRIP_Z),
        (SLOT_MINUS_X, -CLAMP_GOAL_Y, CLAMP_PLACE_Z),
        width=CLAMP[1],
        entry=CLAMP_ENTRY,
        travel_z=TRAVEL,
        yaw=0.0,
        place_yaw=math.pi / 2,
        pick_open=CLAMP_OPEN,
        release_open=CLAMP_OPEN,
    )
    right_plan = [
        Waypoint(1.5, (WAIT_RIGHT[0], WAIT_RIGHT[1], TRAVEL), CLAMP_OPEN, "to wait", 0.0),
        Waypoint(max(0.5, left_s - 1.5), (WAIT_RIGHT[0], WAIT_RIGHT[1], TRAVEL), CLAMP_OPEN, "wait for left", 0.0),
    ] + right_clamp

    return {"left": left_plan, "right": right_plan}


def _make_state(
    pos: tuple[float, float, float],
    yaw_deg: float = 0.0,
    pitch_deg: float = 0.0,
    roll_deg: float = 0.0,
    lin_vel: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ang_vel: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> ObjectState:
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cp, sp = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
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
    box_ok = _make_state((FX, FY, TOP + BOX[2] / 2), yaw_deg=0.0)
    c_plus_start = _make_state((FX, STOW_Y, FLANGE_TOP + CLAMP[2] / 2), yaw_deg=0.0)
    c_minus_start = _make_state((FX, -STOW_Y, FLANGE_TOP + CLAMP[2] / 2), yaw_deg=0.0)

    c_plus_ok = _make_state((SLOT_PLUS_X, CLAMP_GOAL_Y, FLANGE_TOP + CLAMP[2] / 2), yaw_deg=-90.0)
    c_minus_ok = _make_state((SLOT_MINUS_X, -CLAMP_GOAL_Y, FLANGE_TOP + CLAMP[2] / 2), yaw_deg=90.0)

    cases: list[tuple[str, dict[str, ObjectState], float]] = [
        ("start_state", {"box": _make_state((BOX_START_XY[0], BOX_START_XY[1], TOP + BOX[2] / 2), yaw_deg=90.0), "clamp_plus": c_plus_start, "clamp_minus": c_minus_start}, 0.0),
        ("box_seated_only", {"box": box_ok, "clamp_plus": c_plus_start, "clamp_minus": c_minus_start}, 0.4),
        ("box_plus_left_clamp", {"box": box_ok, "clamp_plus": c_plus_ok, "clamp_minus": c_minus_start}, 0.7),
        ("box_plus_right_clamp", {"box": box_ok, "clamp_plus": c_plus_start, "clamp_minus": c_minus_ok}, 0.7),
        ("all_seated_full_score", {"box": box_ok, "clamp_plus": c_plus_ok, "clamp_minus": c_minus_ok}, 1.0),
        ("clamps_swapped_valid", {"box": box_ok, "clamp_plus": c_minus_ok, "clamp_minus": c_plus_ok}, 1.0),
        ("box_end_for_end_180", {"box": _make_state((FX, FY, TOP + BOX[2] / 2), yaw_deg=180.0), "clamp_plus": c_plus_ok, "clamp_minus": c_minus_ok}, 1.0),
        ("box_upside_down_180", {"box": _make_state((FX, FY, TOP + BOX[2] / 2), roll_deg=180.0), "clamp_plus": c_plus_ok, "clamp_minus": c_minus_ok}, 1.0),
        ("box_proud_5mm_on_collar", {"box": _make_state((FX, FY, TOP + BOX[2] / 2 + 0.005)), "clamp_plus": c_plus_ok, "clamp_minus": c_minus_ok}, 0.0),
        ("box_cocked_4deg", {"box": _make_state((FX, FY, TOP + BOX[2] / 2 + 0.003), roll_deg=4.0), "clamp_plus": c_plus_ok, "clamp_minus": c_minus_ok}, 0.0),
        ("box_shifted_6mm", {"box": _make_state((FX + 0.006, FY, TOP + BOX[2] / 2)), "clamp_plus": c_plus_ok, "clamp_minus": c_minus_ok}, 0.0),
        ("box_yawed_4deg", {"box": _make_state((FX, FY, TOP + BOX[2] / 2), yaw_deg=4.0), "clamp_plus": c_plus_ok, "clamp_minus": c_minus_ok}, 0.0),
        ("box_moving", {"box": _make_state((FX, FY, TOP + BOX[2] / 2), lin_vel=(0.02, 0.0, 0.0)), "clamp_plus": c_plus_ok, "clamp_minus": c_minus_ok}, 0.0),
        ("clamp_at_midline_not_slot", {"box": box_ok, "clamp_plus": _make_state((FX, CLAMP_GOAL_Y, FLANGE_TOP + CLAMP[2] / 2), yaw_deg=-90.0), "clamp_minus": c_minus_start}, 0.4),
        ("clamp_tilted_4deg", {"box": box_ok, "clamp_plus": _make_state((SLOT_PLUS_X, CLAMP_GOAL_Y, FLANGE_TOP + CLAMP[2] / 2 + 0.002), pitch_deg=4.0, yaw_deg=-90.0), "clamp_minus": c_minus_start}, 0.4),
        ("clamp_aligned_along_x", {"box": box_ok, "clamp_plus": _make_state((SLOT_PLUS_X, CLAMP_GOAL_Y, FLANGE_TOP + CLAMP[2] / 2), yaw_deg=0.0), "clamp_minus": c_minus_start}, 0.4),
        ("clamp_outer_end_5mm_short", {"box": box_ok, "clamp_plus": _make_state((SLOT_PLUS_X, CLAMP_GOAL_Y - 0.005, FLANGE_TOP + CLAMP[2] / 2), yaw_deg=-90.0), "clamp_minus": c_minus_start}, 0.4),
        ("clamp_outer_end_5mm_past", {"box": box_ok, "clamp_plus": _make_state((SLOT_PLUS_X, CLAMP_GOAL_Y + 0.005, FLANGE_TOP + CLAMP[2] / 2), yaw_deg=-90.0), "clamp_minus": c_minus_start}, 0.4),
        ("clamp_overlap_only_17mm", {"box": _make_state((FX, -0.003, TOP + BOX[2] / 2)), "clamp_plus": _make_state((SLOT_PLUS_X, 0.0395, FLANGE_TOP + CLAMP[2] / 2), yaw_deg=-90.0), "clamp_minus": c_minus_start}, 0.4),
        ("clamp_held_8mm_above", {"box": box_ok, "clamp_plus": _make_state((SLOT_PLUS_X, CLAMP_GOAL_Y, FLANGE_TOP + CLAMP[2] / 2 + 0.008), yaw_deg=-90.0), "clamp_minus": c_minus_start}, 0.4),
        ("clamp_moving", {"box": box_ok, "clamp_plus": _make_state((SLOT_PLUS_X, CLAMP_GOAL_Y, FLANGE_TOP + CLAMP[2] / 2), yaw_deg=-90.0, ang_vel=(0.0, 0.0, 0.8)), "clamp_minus": c_minus_start}, 0.4),
    ]

    for label, state_map, expected in cases:
        got = score(state_map)
        assert math.isclose(got, expected, abs_tol=1e-6), f"{label}: expected {expected}, got {got}"
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
