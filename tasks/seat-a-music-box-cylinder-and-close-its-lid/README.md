# Seat a pinned brass chime cylinder in a walnut music box well and close its maple resonator lid

A pinned brass chime cylinder (`46 mm` diameter $\times$ `20 mm` tall, `95 g`) stands on the left side of the station table, a maple resonator lid (`82 x 64 x 12 mm`, `60 g`) lies on the right side, and an open-top walnut music box case (`78 x 78 mm` outer rim, `34 mm` tall) stands fixed on the midline. Inside the case, notched `12 mm` walnut walls (with `36 mm` side finger-relief notches) enclose a `54 x 54 mm` movement well over a `14 mm` brass bedplate. To complete the task, the robot must seat the brass chime cylinder upright inside the movement well—bringing its top flush with the `34 mm` walnut rim—and then lay the maple resonator lid flat across the housing rim.

What it tests: sequential mechanical assembly with physical enclosure precedence, symmetric bimanual coordination, and flush rim seating.

Physics enforces the assembly sequence without artificial constraints. Closing the maple resonator lid onto the `34 mm` walnut rim first seals the movement well and blocks the brass cylinder from entering (`score = 0.0`). Conversely, leaving the `20 mm`-tall brass cylinder cocked on top of a `34 mm` case wall props the lid `20 mm` proud and tilts it `> 14 degrees`, failing both the rim-height and `3 degree` flatness checks.

Instruction given to the policy: `Seat the brass chime cylinder upright inside the walnut music box movement well, then close the maple resonator lid flat across the case rim.`

Files in this folder: `task.json`, `seat_music_box_cylinder.py` (scene, graded scorer, offline unit tests, and scripted demo), `README.md`.

## Environment

Built on Isaac Lab-Arena 0.3.1 (Isaac Lab 3.0, Isaac Sim 6.1) with the `yam_arena` bimanual embodiment:

- `inspect-robots` 1.0.0 (`inspect-robots --version`)
- `inspect-robots-dream` client (`dreamlabs/dream-1-beta`)
- Task file SHA-256, first 12 hex: `e066b2a0ee13` (`sha256sum tasks/seat-a-music-box-cylinder-and-close-its-lid/seat_music_box_cylinder.py | cut -c1-12`; matches `eval.environment_revision` in both logs)

The Dream Labs API key is stored in `~/.dreamlabs/credentials` (`DREAM_API_KEY=...`).

## Offline check, no GPU

From the repository root with `yam_arena` importable:

```bash
python tasks/seat-a-music-box-cylinder-and-close-its-lid/seat_music_box_cylinder.py
```

This runs `self_test()` across 16 hand-built states (start `0.0`, cylinder seated `0.5`, cylinder seated and lid closed `1.0`, lid reversed `180 deg` or upside down `1.0`; lid closed without cylinder, cylinder outside box, cylinder cocked on rim `20 mm` proud, cylinder tilted `5 deg`, shifted `6 mm`, or moving `0.0`; lid tilted `4 deg`, yawed `8 deg`, shifted `6 mm`, held `8 mm` above rim, or moving `0.5`) and replays both arms' `demo()` trajectories through the 20 Hz URDF inverse kinematics, confirming joint limits and IK error under `0.5 mm`.

## Run and score one Dream-1 trial

From the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/seat-a-music-box-cylinder-and-close-its-lid/seat_music_box_cylinder.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Seat the brass chime cylinder upright inside the walnut music box movement well, then close the maple resonator lid flat across the case rim." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/music-box-dream
```

`task_score` returns `1.0` and terminates the episode with `termination_reason="success"` once `success()` holds continuously for `1.0 s` (20 steps at `20 Hz`); otherwise it reports `score()` at the end of the episode (`0.0`, `0.5`, or `1.0`).

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/seat-a-music-box-cylinder-and-close-its-lid/seat_music_box_cylinder.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Seat the brass chime cylinder upright inside the walnut music box movement well, then close the maple resonator lid flat across the case rim." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/music-box-demo
```

The scripted demo (`demo()`) is open-loop and deterministic:
1. **Cylinder seating (left arm):** Grasps the `46 mm` brass chime cylinder from `(0.220, 0.300)`, carries it over the midline movement well `(0.250, 0.000)`, lowers it onto the `14 mm` brass bedplate, releases, and parks at `(0.200, 0.300)`.
2. **Resonator lid closure (right arm):** Waits clear at `(0.220, -0.260)` until the left arm parks, then grasps the `64 mm`-wide maple lid from `(0.220, -0.300)`, rotates it `90 deg` (`yaw = 0 deg`), and sets it flat across the `34 mm` walnut case rim.

The demo reaches `task_score: 1.0` (`success`) in `616` steps.

## Scoring (`score()` at episode end; `success()` held for 1.0 s)

All bodies must be at rest (linear speed `< 10 mm/s` and angular speed `< 0.5 rad/s`):

| Score | Condition |
|---|---|
| `0.0` | Brass chime cylinder not seated upright inside the movement well (`> 4.5 mm` from `(0.250, 0.000)`, axis `> 3 deg` off vertical, bottom `> 2 mm` from `14 mm` bedplate, top `> 2 mm` from `34 mm` rim, or moving), regardless of lid position |
| `0.5` | Brass chime cylinder seated upright flush in the movement well (`P`), resonator lid not closed flat on the rim |
| `1.0` | Brass chime cylinder seated (`P`) and maple resonator lid resting flat across the `34 mm` walnut case rim (`<= 2 mm` corner error, `<= 4 mm` centre offset, `<= 5 deg` yaw), held continuously for `1.0 s` |

## Run limit, runtime and seeds

- **Run limit:** `2,400` control steps at `20 Hz` (`120 s` of simulated time), `--max-steps 2400`.
- **Runtime per trial (RTX 5090):** ~`4 min` wall clock for the scripted demo; ~`12 min` wall clock for the `2,400`-step Dream-1 trial.
- **Seeds:** `--seed 0` with `-E object_jitter=0` for a fixed starting layout.

## Submitted videos and scores

- **Dream-1 run:** log `adhoc_1129b46f.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after `2400` steps.
- **Demo:** log `adhoc_49a5c2a5.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after `616` steps.
- **Setup screenshot:** frame `0` of `top_cam` rendered at `640 x 360` (`--max-steps 1`).

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials, fidelity and reset

- **Physical build:** 3D-print or laser-cut the open-top walnut housing (`78 x 78 x 34 mm` outer shell with `12 mm` walls and a `14 mm` inner bedplate) and tape it to the table along a marked outline; print or turn the brass cylinder (`46 mm` diameter $\times$ `20 mm` tall, `95 g`) and maple lid (`82 x 64 x 12 mm`, `60 g`).
- **Fidelity:** Rigid cylinder and box primitives with shared friction and matte colours; the walnut housing walls and brass bedplate are fixed colliders standing in for one open-top music box case.
- **Reset between trials:** Nothing is consumed. Lift the maple lid off and lay it flat on `(0.220, -0.300)` along `y`; lift the brass cylinder out of the well and stand it upright at `(0.220, 0.300)`.

## Licence

MIT. `seat_music_box_cylinder.py`, `task.json`, and `README.md` are original code built from `yam_arena` primitives and include no third-party meshes or assets.
