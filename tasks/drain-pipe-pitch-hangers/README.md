# Seat a PVC drain conduit in a hanger trench and set both offset retaining clips

A square-bodied white PVC drain conduit (`180 x 60 x 16 mm`, `110 g`) lies on the left side of the station table, while a fixed steel-grey hanger frame (`240 x 246 mm`) stands on the midline. The frame forms an open-floored cradle trench (`186 x 66 mm` inside `5 mm` walls) flanked by two `16 mm` hanger support plates whose top surfaces sit flush with a seated conduit. Two orange retaining keeper clips (`60 x 16 x 16 mm`, `10 g` each) rest stowed along `x` in keyways on the hanger plates. To complete the task, the robot must seat the conduit flush inside the trench and set both keeper clips across the conduit edges into two longitudinally offset hanger slots—an upstream slot at `x = 0.215 m` on the left plate and a downstream slot at `x = 0.285 m` on the right plate—with at least `18 mm` of each clip overlapping the conduit top face.

What it tests: sequential assembly with physical precedence, bimanual workspace coordination, and precision bridging placement across staggered support stations[cite: 49, 53].

Physics enforces the assembly order without artificial constraints. Each `60 mm` keeper clip spans `24 mm` onto a centred conduit, `13 mm` across the clearance gap and cradle wall, and `23 mm` onto the hanger plate (with its outer `10 mm` guided between two slot ribs). Because the clip's centre (`36 mm` from the midline) overhangs the hanger plate's inner edge (`43 mm` from the midline) by `7 mm`, a clip placed into an empty hanger slot before the conduit is seated tips `23.4 degrees` down into the trench and fails the `3 degree` flatness check[cite: 49, 50]. Similarly, a conduit left cocked `5 mm` proud on a cradle wall tilts the bridging clip `7.9 degrees`[cite: 49, 50]. Placing a clip at the midline (`x = 0.250 m`) instead of the staggered upstream or downstream hanger slots earns no credit for that clip[cite: 50].

Instruction given to the policy: `Seat the white PVC drain conduit flush in the hanger trench, then lay each orange retaining clip flat across the conduit edge into its offset hanger slot, at least 18 mm onto the conduit.`

Files in this folder: `task.json`, `drain_pipe_pitch_hangers.py` (scene, success check, graded partial credit, offline unit tests, and scripted demo), `README.md`.

## Environment

Built on Isaac Lab-Arena 0.3.1 (Isaac Lab 3.0, Isaac Sim 6.1) with the `yam_arena` bimanual embodiment:

- `inspect-robots` 1.0.0 (`inspect-robots --version`)[cite: 54]
- `inspect-robots-dream` client (`dreamlabs/dream-1-beta`)
- Task file SHA-256, first 12 hex: `19438ce97777` (`sha256sum tasks/drain-pipe-pitch-hangers/drain_pipe_pitch_hangers.py | cut -c1-12`; matches `eval.environment_revision` in both logs)

The Dream Labs API key is stored in `~/.dreamlabs/credentials` (`DREAM_API_KEY=...`).

## Offline check, no GPU

From the repository root with `yam_arena` importable:

```bash
python tasks/drain-pipe-pitch-hangers/drain_pipe_pitch_hangers.py