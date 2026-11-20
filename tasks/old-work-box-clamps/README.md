# Seat an old-work electrical box in a drywall cutout and lock both swing-out clamps

A blue PVC 1-gang old-work electrical box (`100 x 60 x 16 mm`, `95 g`) lies on the left side of the station table, while a fixed drywall cutout frame (`160 x 246 mm`) stands on the midline. The frame forms an open-floored rectangular cutout pocket (`106 x 66 mm` inside `5 mm` collar walls) flanked by two `16 mm` drywall mounting flanges whose top surfaces sit flush with a seated electrical box. Two orange swing-out mounting clamps (`60 x 16 x 16 mm`, `10 g` each) rest stowed along `x` in keyways on the outer flanges. To complete the task, the robot must seat the electrical box flush inside the drywall cutout and set both mounting clamps flat across the box edges into two longitudinally offset clamp-lock pockets—an upstream pocket at `x = 0.222 m` on the left flange and a downstream pocket at `x = 0.278 m` on the right flange—with at least `18 mm` of each clamp overlapping the electrical box top face.

What it tests: sequential electrical rough-in assembly with physical precedence, bimanual workspace coordination, and precision bridging placement across staggered mounting flanges.

Physics enforces the assembly sequence without artificial constraints. Each `60 mm` mounting clamp spans `24 mm` onto a centred electrical box, `13 mm` across the `3 mm` cutout gap and `10 mm` collar wall, and `23 mm` onto the drywall flange (with its outer `10 mm` guided between two lock-ear ribs). Because the clamp's centre (`36 mm` from the midline) overhangs the inner edge of the drywall flange (`43 mm` from the midline) by `7 mm`, a clamp placed into a lock pocket before the electrical box is seated tips `23.4 degrees` down into the empty cutout and fails the `3 degree` flatness check. Similarly, an electrical box left cocked `5 mm` proud on the collar wall tilts the bridging clamp `7.9 degrees`. Placing a clamp at the midline (`x = 0.250 m`) instead of the staggered upstream or downstream lock pockets earns no credit for that clamp.

Instruction given to the policy: `Seat the old-work electrical box flush in the drywall cutout, then swing and seat both orange mounting clamps flat across the rear drywall flanges.`

Files in this folder: `task.json`, `old_work_box_clamps.py` (scene, success check, graded partial credit, offline unit tests, and scripted demo), `README.md`.

## Environment

Built on Isaac Lab-Arena 0.3.1 (Isaac Lab 3.0, Isaac Sim 6.1) with the `yam_arena` bimanual embodiment:

- `inspect-robots` 1.0.0 (`inspect-robots --version`)
- `inspect-robots-dream` client (`dreamlabs/dream-1-beta`)
- Task file SHA-256, first 12 hex: `5cfb020d23b3` (`sha256sum tasks/old-work-box-clamps/old_work_box_clamps.py | cut -c1-12`; matches `eval.environment_revision` in both logs)

The Dream Labs API key is stored in `~/.dreamlabs/credentials` (`DREAM_API_KEY=...`).

## Offline check, no GPU

From the repository root with `yam_arena` importable:

```bash
python tasks/old-work-box-clamps/old_work_box_clamps.py
```

This runs `self_test()` across 21 hand-built states (start `0.0`, electrical box seated `0.4`, one offset clamp pocket locked `0.7`, both locked `1.0`, clamps swapped between upstream and downstream pockets `1.0`; box proud `5 mm`, cocked `4 deg`, shifted `6 mm`, yawed `4 deg`, or moving `0.0` regardless of clamp positions; clamp at midline, tilted `4 deg`, aligned along `x`, outer end `5 mm` short or past the lock ribs, only `17 mm` onto the box, held `8 mm` above, or moving `0.4`; upside-down or end-for-end reversed box still valid) and replays both arms' `demo()` trajectories through the 20 Hz URDF inverse kinematics, confirming joint limits and IK error under `0.5 mm`.

## Run and score one Dream-1 trial

From the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/old-work-box-clamps/old_work_box_clamps.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Seat the old-work electrical box flush in the drywall cutout, then swing and seat both orange mounting clamps flat across the rear drywall flanges." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/old-work-box-dream
```

`task_score` returns `1.0` and terminates the episode with `termination_reason="success"` once `success()` holds continuously for `1.0 s` (20 steps at `20 Hz`); otherwise it reports `score()` at the end of the episode (`0.0`, `0.4`, `0.7`, or `1.0`).

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/old-work-box-clamps/old_work_box_clamps.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Seat the old-work electrical box flush in the drywall cutout, then swing and seat both orange mounting clamps flat across the rear drywall flanges." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/old-work-box-demo
```

The scripted demo (`demo()`) is open-loop and deterministic:
1. **Box seating (left arm):** Picks the blue old-work electrical box across its `60 mm` width at `yaw = -90 deg`, rotates it over the midline cutout (`yaw = 0 deg` minus a `-2.9 deg` / `2.0 mm` wrist-compliance trim), lowers it onto the table, and opens the fingers to `72 mm` inside the `86 mm` gap between the drywall flanges.
2. **Upstream mounting clamp (left arm):** Lifts `clamp_plus` from its keyway using a `40 mm` opening to clear the ribs, turns it to `yaw = -90 deg`, lays it across the box edge into the upstream left lock pocket (`x = 0.222 m`), and parks at `(0.200, 0.300)`.
3. **Downstream mounting clamp (right arm):** Waits clear of the workspace until the left arm parks, then lifts `clamp_minus` with a `40 mm` opening, turns it to `yaw = +90 deg`, and lays it across the box edge into the downstream right lock pocket (`x = 0.278 m`).

The demo reaches `task_score: 1.0` (`success`) in `981` steps.

## Scoring (`score()` at episode end; `success()` held for 1.0 s)

All bodies must be at rest (linear speed `< 10 mm/s` and angular speed `< 0.5 rad/s`):

| Score | Condition |
|---|---|
| `0.0` | Electrical box not seated flush in the cutout (centre `> 4 mm` from `(0.250, 0.000)`, yaw `> 3 deg` from `x`, any bottom corner `> 2 mm` off the table, any top corner `> 2 mm` from `16 mm`, or moving), regardless of clamp positions |
| `0.4` | Electrical box seated flush (`P`), zero offset clamp pockets locked |
| `0.7` | Electrical box seated flush (`P`) and one offset clamp pocket (`x = 0.222 m` left or `x = 0.278 m` right) holding a flat, level mounting clamp (`<= 3 deg` tilt, `<= 10 deg` yaw, outer end within `4 mm` of `y = +-0.066 m`, inner end `>= 18 mm` onto the electrical box) |
| `1.0` | Electrical box seated flush (`P`) and both offset clamp pockets locked with set mounting clamps, held continuously for `1.0 s` |

Real-bench measurement: verify flush box seating with a straightedge across both drywall flanges, check clamp flatness (`<= 3 deg`) with a digital level, verify outer-end alignment (`<= 4 mm`) against the lock ribs' outer faces, and measure `>= 18 mm` box overlap with a steel rule.

## Run limit, runtime and seeds

- **Run limit:** `2,400` control steps at `20 Hz` (`120 s` of simulated time), `--max-steps 2400`.
- **Runtime per trial (RTX 5090):** ~`4 min` wall clock for the scripted demo; ~`12 min` wall clock for the `2,400`-step Dream-1 trial.
- **Seeds:** `--seed 0` with `-E object_jitter=0` for a fixed starting layout.

## Submitted videos and scores

- **Dream-1 run:** log `adhoc_0a1b8adb.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after `2400` steps.
- **Demo:** log `adhoc_806c995b.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after `981` steps.
- **Setup screenshot:** frame `0` of `top_cam` rendered at `640 x 360` (`--max-steps 1`).

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials, fidelity and reset

- **Physical build:** 3D-print the cutout collar ring and both drywall mounting flanges with their ribs as one open-floored PLA frame (`160 x 246 mm`, or two `160 x 123 mm` halves split on the midline) and tape it to the table along a marked outline; print the electrical box (`100 x 60 x 16 mm`, `95 g`) and both clamps (`60 x 16 x 16 mm`, `10 g` each) in PLA.
- **Fidelity:** Rigid box primitives with shared friction and matte colours; the cutout collar ring, drywall flanges, and ribs are fixed colliders standing in for one printed drywall cutout frame. A real old-work electrical box rotates plastic ears via #6-32 machine screws; here the electrical box seats flush in a cutout opening and loose swing-out clamps bridge across the joint into staggered lock pockets under gravity.
- **Reset between trials:** Nothing is consumed. Return the electrical box to `(0.220, 0.320)` with its long axis along `y`, and return both mounting clamps to their keyways along `x` at `(0.250, +-0.103)`.

## Licence

MIT. `old_work_box_clamps.py`, `task.json`, and `README.md` are original code built from `yam_arena` box primitives and include no third-party meshes or assets.
