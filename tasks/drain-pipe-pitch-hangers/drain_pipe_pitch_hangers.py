"""On the station table, on the midline, a 1 g bench analogue of commercial plumbing drain-line installation
(plumbers, pipefitters, and steamfitters, O*NET 47-2152.00): a square-bodied PVC drain conduit is seated flush into its
cradle trench between two staggered hanger plates, then two orange retaining keeper clips are lifted from their stowed
keyways and laid flat across the conduit edges into two longitudinally offset hanger slots (upstream at x = 0.215 m on
the left plate, downstream at x = 0.285 m on the right plate), extending at least 18 mm onto the conduit top face.
Because each keeper clip bridges the hanger plate and the conduit, its flat seat exists only once the conduit is seated
flush in the trench: a clip placed before the conduit is seated tips about 23.4 degrees into the empty trench, and a
conduit cocked on the 5 mm cradle rim tilts the clip past 3 degrees. The score reads the conduit from its centre and all
eight corners, and each keeper clip from its centre, its two ends, and its quaternion.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/drain-pipe-pitch-hangers/drain_pipe_pitch_hangers.py \
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
           --instruction "Seat the white PVC drain conduit flush in the hanger trench, then lay each orange retaining clip flat across the conduit edge into its offset hanger slot, at least 18 mm onto the conduit." \
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
F = (0.250, 0.000)  # the hanger trench centre, on the midline
PIPE = (0.180, 0.060, 0.016)  # square-bodied PVC drain conduit: 180 x 60 x 16 mm
PIPE_MASS = 0.110
CLIP = (0.060, 0.016, 0.016)  # retaining keeper clip bar: 60 x 16 x 16 mm
CLIP_MASS = 0.010
CLEAR = 0.003  # conduit to cradle ring wall, per side
RING_H = 0.005
RING_T = 0.010
PLATE = (0.240, 0.080, 0.016)  # hanger support plate; top is flush with a seated conduit's top (16 mm)
PLATE_Y = PIPE[1] / 2 + CLEAR + RING_T + PLATE[1] / 2  # 83 mm, hanger plate centre from the midline
PLATE_TOP = TOP + PLATE[2]
RIB_H = 0.010  # every keyway and slot rib stands 10 mm tall on a hanger plate
STOW_Y = 0.103  # a stowed clip's centre, from the midline
SLOT_Y = 0.036  # a set clip's centre, from the midline: spans 6 to 66 mm, putting 24 mm on a centred conduit
SLOT_RIB_Y = 0.061  # slot rib centres, spanning 56 to 66 mm from the midline
SLOT_OUTER = SLOT_RIB_Y + 0.005  # 66 mm: the ribs' outer faces, where a set clip's outer end lies
SLOT_OFFSET_X = 0.035  # longitudinal stagger: left slot upstream at x = 0.215, right slot downstream at x = 0.285
SLOT_GAP = CLIP[1] + 2 * 0.003  # 22 mm between the slot ribs' inner faces, 3 mm clearance per side
KEY_GAP = CLIP[1] + 2 * 0.002  # 20 mm between the keyway ribs' inner faces, 2 mm clearance per side
PIPE_START = (0.220, 0.320)  # flat on left side of table, long axis along y

PVC_WHITE = (0.88, 0.90, 0.92)
ORANGE = (0.95, 0.55, 0.10)
STEEL_GREY = (0.38, 0.40, 0.44)

_ring_y = PIPE[1] / 2 + CLEAR + RING_T / 2  # 38 mm
_ring_x = PIPE[0] / 2 + CLEAR + RING_T / 2  # 98 mm
_rib_z = PLATE_TOP + RIB_H / 2
RING = [
    Box("ring_left", pos=(F[0], F[1] + _ring_y, TOP + RING_H / 2), size=(PIPE[0] + 2 * (CLEAR + RING_T), RING_T, RING_H), color=STEEL_GREY),
    Box("ring_right", pos=(F[0], F[1] - _ring_y, TOP + RING_H / 2), size=(PIPE[0] + 2 * (CLEAR + RING_T), RING_T, RING_H), color=STEEL_GREY),
    Box("ring_far", pos=(F[0] + _ring_x, F[1], TOP + RING_H / 2), size=(RING_T, PIPE[1] + 2 * CLEAR, RING_H), color=STEEL_GREY),
    Box("ring_near", pos=(F[0] - _ring_x, F[1], TOP + RING_H / 2), size=(RING_T, PIPE[1] + 2 * CLEAR, RING_H), color=STEEL_GREY),
]
PLATES = [
    Box(f"plate_{side}", pos=(F[0], s * PLATE_Y, TOP + PLATE[2] / 2), size=PLATE, color=STEEL_GREY)
    for side, s in (("left", 1), ("right", -1))
]
KEYWAYS = [
    Box(f"keyway_{side}_{k}", pos=(F[0] + dx, s * (STOW_Y + dy), _rib_z), size=(0.010, 0.004, RIB_H), color=STEEL_GREY)
    for side, s in (("left", 1), ("right", -1))
    for k, (dx, dy) in enumerate(((-0.027, -(KEY_GAP / 2 + 0.002)), (-0.027, KEY_GAP / 2 + 0.002), (0.027, -(KEY_GAP / 2 + 0.002)), (0.027, KEY_GAP / 2 + 0.002)))
]
SLOT_RIBS = [
    Box(f"slot_{side}_{k}", pos=(sx + dx, sy_rib, _rib_z), size=(0.004, 0.010, RIB_H), color=STEEL_GREY)
    for side, sx, sy_rib in (
        ("left", F[0] - SLOT_OFFSET_X, SLOT_RIB_Y),
        ("right", F[0] + SLOT_OFFSET_X, -SLOT_RIB_Y),
    )
    for k, dx in enumerate((-(SLOT_GAP / 2 + 0.002), SLOT_GAP / 2 + 0.002))
]
PIPE_BOX = Box("pipe", pos=(PIPE_START[0], PIPE_START[1], TOP + PIPE[2] / 2), size=PIPE, mass=PIPE_MASS, color=PVC_WHITE, yaw=math.pi / 2)
CLIPS = [
    Box(f"clip_{side}", pos=(F[0], s * STOW_Y, PLATE_TOP + CLIP[2] / 2), size=CLIP, mass=CLIP_MASS, color=ORANGE)
    for side, s in (("plus", 1), ("minus", -1))
]
OBJECTS = [PIPE_BOX, *CLIPS, *RING, *PLATES, *KEYWAYS, *SLOT_RIBS]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

PIPE_XY_TOL = 0.004  # centre to F; the cradle ring allows 3 mm
PIPE_YAW_TOL = math.radians(3.0)  # the cradle ring allows about 1.9 degrees
FLUSH_TOL = 0.002  # every bottom corner to the table, every top corner to the plate tops
CLIP_YAW_TOL = math.radians(10.0)  # long axis to y
CLIP_TILT_TOL = math.radians(3.0)  # a face flat
CLIP_BOTTOM_TOL = 0.002  # lowest corner to the plate tops
CLIP_XY_TOL = 0.004  # centre to the slot's centre line across it (x); outer end to the ribs' outer faces along it (y)
ONTO = 0.018  # a set clip's minimum overlap onto the conduit top face (nominal centred overlap is 24 mm)
EPS = 1e-6
SLOTS = ((F[0] - SLOT_OFFSET_X, F[1] + SLOT_Y), (F[0] + SLOT_OFFSET_X, F[1] - SLOT_Y))
LADDER = (0.4, 0.7, 1.0)  # conduit seated with 0, 1 or 2 offset hanger slots filled


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


_SIGNS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)


def _corner_heights(state, size):
    """The z of the box's eight corners, lowest first, from its centre and quaternion."""
    corners = np.asarray(state.pos, float) + (_SIGNS * (np.asarray(size) / 2)) @ _rot(state.quat_xyzw).T
    return np.sort(corners[:, 2])


def _seated(pipe) -> bool:
    """Resting in the hanger trench, flat on the table and flush: centre within 4 mm of F in x and y, long axis
    within 3 degrees of x either way round, the four bottom corners within 2 mm of the table and the four top
    corners within 2 mm of the plate tops, so a conduit cocked on a cradle wall fails."""
    if not pipe.resting():
        return False
    if abs(float(pipe.pos[0]) - F[0]) > PIPE_XY_TOL + EPS or abs(float(pipe.pos[1]) - F[1]) > PIPE_XY_TOL + EPS:
        return False
    ex = _rot(pipe.quat_xyzw)[:, 0]
    off = abs(math.atan2(float(ex[1]), float(ex[0])))
    if min(off, math.pi - off) > PIPE_YAW_TOL:
        return False
    z = _corner_heights(pipe, PIPE)
    return bool(np.all(np.abs(z[:4] - TOP) <= FLUSH_TOL + EPS) and np.all(np.abs(z[4:] - PLATE_TOP) <= FLUSH_TOL + EPS))


def _set_in(clip, slot, pipe) -> bool:
    """Resting flat across the conduit edge in this offset hanger slot: long axis within 10 degrees of y either
    way round, a face within 3 degrees of flat, lowest corner within 2 mm of the plate tops, centre within 4 mm
    of the slot's centre line across it (x), outer end within 4 mm of the ribs' outer faces along it (y), and its
    inner end at least 18 mm inside the conduit's side edge."""
    if not clip.resting():
        return False
    r = _rot(clip.quat_xyzw)
    ex = r[:, 0]
    off = abs(math.atan2(float(ex[0]), float(ex[1])))  # angle of the long axis from y
    if min(off, math.pi - off) > CLIP_YAW_TOL:
        return False
    if max(abs(float(r[2, 1])), abs(float(r[2, 2]))) < math.cos(CLIP_TILT_TOL):  # a side face down, level
        return False
    if abs(float(_corner_heights(clip, CLIP)[0]) - PLATE_TOP) > CLIP_BOTTOM_TOL + EPS:
        return False
    if abs(float(clip.pos[0]) - slot[0]) > CLIP_XY_TOL + EPS:  # across the offset slot (x)
        return False
    side = 1.0 if slot[1] > F[1] else -1.0
    ends = [np.asarray(clip.pos, float) + s * CLIP[0] / 2 * ex for s in (-1, 1)]
    inner, outer = sorted(ends, key=lambda p: abs(float(p[1]) - F[1]))  # toward the trench, toward the ribs
    if abs(side * (float(outer[1]) - F[1]) - SLOT_OUTER) > CLIP_XY_TOL + EPS:  # along the slot (y)
        return False
    local = _rot(pipe.quat_xyzw).T @ (inner - np.asarray(pipe.pos, float))
    return bool(PIPE[1] / 2 - abs(float(local[1])) >= ONTO - EPS and abs(float(local[0])) <= PIPE[0] / 2)


def _filled(objects) -> int:
    """Offset hanger slots holding a set keeper clip; either clip may fill either slot, and each slot counts once."""
    pipe = objects["pipe"]
    return sum(any(_set_in(objects[c.name], slot, pipe) for c in CLIPS) for slot in SLOTS)


def success(objects) -> bool:
    """The conduit seated flush and both offset hanger slots filled, everything at rest; holds for 1 s."""
    return _seated(objects["pipe"]) and _filled(objects) == 2


def distance(objects) -> float:
    """Metres still to go: the conduit's centre to F, plus each offset slot's nearest clip centre to that slot."""
    p = objects["pipe"]
    total = math.hypot(float(p.pos[0]) - F[0], float(p.pos[1]) - F[1])
    for sx, sy in SLOTS:
        total += min(math.hypot(float(objects[c.name].pos[0]) - sx, float(objects[c.name].pos[1]) - sy) for c in CLIPS)
    return total


def score(objects) -> float:
    """0 unless the conduit is seated flush; then 0.4, plus 0.3 for each offset slot holding a set clip: 0, 0.4,
    0.7 or 1.0."""
    if not _seated(objects["pipe"]):
        return 0.0
    return LADDER[_filled(objects)]


TRAVEL = TOP + 0.150
PIPE_GRIP_Z = TOP + 0.018  # TCP 10 mm above the conduit centre, pad tips 5 mm above the table
PIPE_PLACE_Z = TOP + 0.024  # conduit bottom 6 mm above the table, 1 mm above the ring; pad tips 6 mm above the ring
PIPE_ENTRY = 0.012  # pause above the trench: conduit bottom 13 mm above the ring
PIPE_RELEASE = -0.036  # 72 mm opening: 6 mm clear of the 60 mm conduit per side, staying inside the 86 mm plate gap
PIPE_TRIM = (0.0020, -0.0005, math.radians(-2.9))
CLIP_GRIP_Z = PLATE_TOP + 0.018  # TCP 10 mm above a stowed clip's centre, pad tips 5 mm above the plate
CLIP_PLACE_Z = PLATE_TOP + 0.024  # clip bottom 6 mm above the conduit and plate; pad tips 1 mm above the ribs
CLIP_ENTRY = 0.015  # pause above the slot: clip bottom 11 mm above the ribs
CLIP_OPEN = -0.020  # 40 mm opening at the stowed clip, 12 mm clear of the clip and slot ribs
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part so each finger pushes ~20 N (< 50 N cap)
PARK_LEFT = (0.200, 0.300)
WAIT_RIGHT = (0.220, -0.260)


def _pick_place(pick, place, *, width, entry, travel_z, yaw=0.0, place_yaw=None, pick_open=FINGER_OPEN, release_open=FINGER_OPEN):
    """A top-down pick and place paced for the simulated arm, whose soft wrist trails a moving target by several
    millimetres. The arm settles before every close and release, stops ``entry`` above the place with the part
    clear of every fixture, and makes the last descent slowly. The fingers close to 8 mm narrower than the part's
    ``width`` across the pads rather than fully shut: each finger then pushes 4 mm x 5000 N/m = 20 N, under its
    50 N cap, so the grip centres the part."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2  # finger joint target, m
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


def demo():
    """Left arm: picks the PVC drain conduit from its start at yaw -90 across its 60 mm width, turns it to yaw 0
    less the measured trim, seats it into the hanger trench releasing to 72 mm so the fingers stay inside the
    hanger plate edges, then lifts clip_plus from its keyway with a 40 mm opening, turns it to yaw -90, lays it
    into the upstream left hanger slot at x = 0.215 m, and parks. Right arm: waits out of the way in front of its
    base until the left arm has set its clip and parked, then sets clip_minus into the downstream right hanger
    slot at x = 0.285 m at yaw +90."""
    left = _pick_place(
        (PIPE_START[0], PIPE_START[1], PIPE_GRIP_Z),
        (F[0] + PIPE_TRIM[0], F[1] + PIPE_TRIM[1], PIPE_PLACE_Z),
        width=PIPE[1],
        entry=PIPE_ENTRY,
        travel_z=TRAVEL,
        yaw=-math.pi / 2,
        place_yaw=PIPE_TRIM[2],
        release_open=PIPE_RELEASE,
    )
    left += _pick_place(
        (F[0], STOW_Y, CLIP_GRIP_Z),
        SLOTS[0] + (CLIP_PLACE_Z,),
        width=CLIP[1],
        entry=CLIP_ENTRY,
        travel_z=TRAVEL,
        yaw=0.0,
        place_yaw=-math.pi / 2,
        pick_open=CLIP_OPEN,
    )
    left += [Waypoint(1.5, (PARK_LEFT[0], PARK_LEFT[1], TRAVEL), FINGER_OPEN, "park", -math.pi / 2)]
    wait = (WAIT_RIGHT[0], WAIT_RIGHT[1], TRAVEL)
    right = [
        Waypoint(1.5, wait, FINGER_OPEN, "out of the way"),
        Waypoint(32.5, wait, FINGER_OPEN, "wait for the left arm to set its clip and park"),
    ]
    right += _pick_place(
        (F[0], -STOW_Y, CLIP_GRIP_Z),
        SLOTS[1] + (CLIP_PLACE_Z,),
        width=CLIP[1],
        entry=CLIP_ENTRY,
        travel_z=TRAVEL,
        yaw=0.0,
        place_yaw=math.pi / 2,
        pick_open=CLIP_OPEN,
    )
    return {"left": left, "right": right}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0, roll=0.0, pitch=0.0):
        r = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        r = r @ np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        r = r @ np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        w = math.sqrt(max(1e-12, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
        return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])

    def body(size, x, y, bottom, q, vel=(0.0, 0.0, 0.0)):
        half = np.abs(_rot(q)) @ (np.asarray(size) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def pipe(dx=0.0, dy=0.0, bottom=TOP, yaw=0.0, roll=0.0, pitch=0.0, vel=(0.0, 0.0, 0.0)):
        return body(PIPE, F[0] + dx, F[1] + dy, bottom, quat(yaw, roll, pitch), vel)

    def clip(k, along=0.0, across=0.0, bottom=PLATE_TOP, yaw=math.pi / 2, roll=0.0, pitch=0.0, vel=(0.0, 0.0, 0.0)):
        """A clip in offset slot k; ``along`` moves it outward along y, ``across`` shifts it along x."""
        sx, sy = SLOTS[k]
        s = 1.0 if sy > F[1] else -1.0
        return body(CLIP, sx + across, sy + s * along, bottom, quat(yaw, roll, pitch), vel)

    start_pipe = body(PIPE, PIPE_START[0], PIPE_START[1], TOP, quat(math.pi / 2))
    stowed = [body(CLIP, F[0], s * STOW_Y, PLATE_TOP, quat()) for s in (1, -1)]

    def scene(p=None, plus=None, minus=None):
        return {"pipe": p or start_pipe, "clip_plus": plus or stowed[0], "clip_minus": minus or stowed[1]}

    cock = math.asin(RING_H / PIPE[0])
    cocked = body(PIPE, F[0] + 0.0035, F[1], TOP, quat(0.0, 0.0, -cock))
    tip = math.atan2(PLATE[2], (PLATE_Y - PLATE[1] / 2) - (SLOT_Y - CLIP[0] / 2))
    tipped = body(CLIP, SLOTS[0][0], SLOT_Y - 0.004, TOP, quat(math.pi / 2, 0.0, -tip))
    proud = math.atan2(RING_H, (SLOT_Y + CLIP[0] / 2) - PIPE[1] / 2)
    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(pipe(), clip(0), clip(1)), 1.0, True),
        "pipe seated, clips stowed": (scene(pipe()), 0.4, False),
        "pipe seated, clip_plus set in upstream slot": (scene(pipe(), clip(0)), 0.7, False),
        "pipe seated, clip_minus set in downstream slot": (scene(pipe(), None, clip(1)), 0.7, False),
        "clips swapped between upstream and downstream slots": (scene(pipe(), clip(1), clip(0)), 1.0, True),
        "clips set, pipe still at its start": (scene(None, clip(0), clip(1)), 0.0, False),
        "no pipe, a clip tipped into the trench": (scene(None, tipped), 0.0, False),
        "pipe cocked on the far ring wall, clips set": (scene(cocked, clip(0), clip(1)), 0.0, False),
        "pipe cocked, clips tilted 7.9 degrees on it": (
            scene(cocked, clip(0, bottom=TOP + 0.009, pitch=-proud), clip(1, bottom=TOP + 0.009, pitch=proud)),
            0.0,
            False,
        ),
        "pipe 5 mm proud, flat": (scene(pipe(bottom=TOP + RING_H)), 0.0, False),
        "pipe seated, moving": (scene(pipe(vel=(0.0, 0.0, 0.02)), clip(0), clip(1)), 0.0, False),
        "pipe held 10 mm above the trench": (scene(pipe(bottom=TOP + 0.010), clip(0), clip(1)), 0.0, False),
        "pipe seated against a ring wall, 3 mm off in x and y": (scene(pipe(dx=0.003, dy=-0.003), clip(0), clip(1)), 1.0, True),
        "pipe seated, yawed 2 degrees": (scene(pipe(yaw=math.radians(2))), 0.4, False),
        "pipe yawed 4 degrees": (scene(pipe(yaw=math.radians(4))), 0.0, False),
        "pipe 5 mm off in y, flat on the table": (scene(pipe(dy=0.005)), 0.0, False),
        "pipe upside down, seated": (scene(pipe(roll=math.pi), clip(0), clip(1)), 1.0, True),
        "pipe reversed end for end, seated": (scene(pipe(yaw=math.pi), clip(0), clip(1)), 1.0, True),
        "pipe on a hanger plate": (scene(body(PIPE, F[0], PLATE_Y, PLATE_TOP, quat()), clip(0)), 0.0, False),
        "clip_plus set at midline instead of upstream offset slot": (scene(pipe(), clip(0, across=SLOT_OFFSET_X)), 0.4, False),
        "clip_minus set at midline instead of downstream offset slot": (scene(pipe(), None, clip(1, across=-SLOT_OFFSET_X)), 0.4, False),
        "clip_plus set, tilted 4 degrees": (scene(pipe(), clip(0, pitch=math.radians(4))), 0.4, False),
        "clip_plus set, rolled 4 degrees": (scene(pipe(), clip(0, roll=math.radians(4))), 0.4, False),
        "clip_plus set, tilted 2 degrees": (scene(pipe(), clip(0, pitch=math.radians(2))), 0.7, False),
        "clip_plus across the joint along x": (scene(pipe(), clip(0, yaw=0.0)), 0.4, False),
        "clip_plus yawed 8 degrees in its slot": (scene(pipe(), clip(0, yaw=math.pi / 2 + math.radians(8))), 0.7, False),
        "clip_plus yawed 12 degrees": (scene(pipe(), clip(0, yaw=math.pi / 2 + math.radians(12))), 0.4, False),
        "clip_plus reversed end for end": (scene(pipe(), clip(0, yaw=-math.pi / 2)), 0.7, False),
        "clip_plus rolled a quarter turn about its long axis": (scene(pipe(), clip(0, roll=math.pi / 2)), 0.7, False),
        "clip_plus 3 mm off in x, against a slot rib": (scene(pipe(), clip(0, across=0.003)), 0.7, False),
        "clip_plus 5 mm inward, outer end 5 mm short of the rib faces": (scene(pipe(), clip(0, along=-0.005)), 0.4, False),
        "clip_plus 3.8 mm inward": (scene(pipe(), clip(0, along=-0.0038)), 0.7, False),
        "clip_plus 4.5 mm outward, outer end past the rib faces": (scene(pipe(), clip(0, along=0.0045)), 0.4, False),
        "clip_plus against a rib and 3 mm outward": (scene(pipe(), clip(0, across=0.003, along=0.003)), 0.7, False),
        "clip_plus laid outside the ribs, 23 mm across": (scene(pipe(), clip(0, across=0.023)), 0.4, False),
        "clip_plus 3 mm outward, 21 mm on the pipe": (scene(pipe(), clip(0, along=0.003)), 0.7, False),
        "pipe 3.5 mm away, clip_plus 3 mm outward: only 17.5 mm on the pipe (< 18 mm)": (
            scene(pipe(dy=-0.0035), clip(0, along=0.003)),
            0.4,
            False,
        ),
        "clip_plus flat on the pipe, fully inside it": (scene(pipe(), clip(0, along=-0.026)), 0.4, False),
        "clip_plus on the rib tops": (scene(pipe(), clip(0, bottom=PLATE_TOP + RIB_H)), 0.4, False),
        "clip_plus still held 8 mm over its slot, moving": (
            scene(pipe(), clip(0, bottom=PLATE_TOP + 0.008, vel=(0.0, 0.0, -0.02))),
            0.4,
            False,
        ),
        "clip_minus stacked on a set clip_plus": (scene(pipe(), clip(0), clip(0, bottom=PLATE_TOP + CLIP[2])), 0.7, False),
        "clip_plus on the table beside the frame": (scene(pipe(), body(CLIP, 0.420, 0.100, TOP, quat())), 0.4, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then both arms' plans replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
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
