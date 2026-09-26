# P0.3 stage C — episodes spawned in Gazebo and fetched — 2026-09-26

Branch `p03c-episode-gazebo`. Package code at `7309dbe` for every run
(`meta.txt` `head=`); the `dirty_paths` each run records (1 for the first,
5 after) are this directory's uncommitted harness scripts and docs, not
package source. Headless, never `--fast`, fresh simulator per run, torn
down by process name, ROS domain 64, overlay `~/coco_p03c_ws` built from
this tree (`resolve.txt`). Full logs stay in `~/coco_nav_runs/p03c_matrix/`
(`sim.log`, `mission.log`, `cmdpath/trace.csv`, `hrec.csv`); `matrix/`
here keeps the small files, curated by `p03c_curate.sh`.

## How each run was made

`../p03c_episode_run.sh OUT LEVEL SEED COLOUR` — the C2-NAV.44 runner that
every M6 number since C2-NAV.44 was measured with, plus:

1. `coco_episode generate` writes `manifest.json` **before** anything runs;
   `coco_episode inputs` writes what the robot side will be told.
2. FIXED runs use the **default command lines** — no episode argument
   reaches either launch except `episode_record` (which only records).
   COLOUR/POSITION hand the same `manifest.json` to both launches.
3. `coco_episode readback` compares gz with the manifest (`gz model -p`,
   tolerances xy 5 mm, z 5 mm, tilt 0.05 rad, then `validate_episode` on
   the OBSERVED poses — separation, corridor, region). The spawned
   manifest must be byte-identical to the generated one (FIXED: equal
   targets; the default world launch draws its own requested colour).
4. `region_map` read back from `/mission_executive` **and** `/ramp_driver`
   with `ros2 param get`; must equal the manifest's (FIXED: empty). No
   node parameter may contain a manifest path.
5. The C2-NAV.44 bring-up checks, unchanged, plus exactly one publisher
   on `/diff_drive_controller/cmd_vel`.
6. `/mission/start`; wait for COMPLETE or ABORT; `coco_episode result`
   writes the `EpisodeResult` (timings named by clock: `mission_sim_s`
   from `/clock` at start and at terminal detection, `mission_wall_s`).

## The matrix, and why these seeds

`../p03c_episode_matrix.sh`. FIXED × the four colours; COLOUR and
POSITION at seeds **1, 2, 4**. The rule was fixed before any run: the
first three seeds ≥ 1 with distinct colour permutations (1 and 3 draw the
same one), each requesting the colour the permutation moved **farthest**
from its frozen lane — a colour left in its own lane is one the
pre-episode mission would also fetch, and would test nothing. Same
assignment at both levels per seed, so POSITION differs from COLOUR only
by the placement. Modes interleaved (fixed, colours, positions, …) so
drift over the sweep is not confounded with mode.

## Results — the matrix (10 runs, all measured)

`matrix_report.md` / `matrix_report.json` are `../p03c_matrix_report.py`'s
output over `~/coco_nav_runs/p03c_matrix`; every cell comes from a file.
FIXED episode ids are the harness manifests' (seed 0, colour pinned).

| MODE | SEED | episode_id | requested → region (frozen) | target x, y (m) | RESULT | ERROR / reason | sim s | wall s | approach stop x / y (m) | home err (m) | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|
| FIXED | 0 | ep-7916a30396de | red → lane_1 (lane_1) | 4.0500, -0.7500 | complete | -- | 149.1 | 304 | 0.1545 / -0.0000 | 0.119 | default command lines |
| FIXED | 0 | ep-e89ab8905964 | green → lane_2 (lane_2) | 4.0500, -0.2500 | complete | -- | 198.5 | 389 | 0.1544 / -0.0000 | 0.019 | RETURN_HOME 98 s sim |
| FIXED | 0 | ep-be0f51920fb2 | blue → lane_3 (lane_3) | 4.0500, +0.2500 | complete | -- | 155.5 | 285 | 0.1542 / +0.0000 | 0.104 | runner check FAIL was the read-back checker (10 µm settle); passes re-judged |
| FIXED | 0 | ep-83b636dd724b | yellow → lane_4 (lane_4) | 4.0500, +0.7500 | complete | -- | 318.6 | 767 | 0.1539 / -0.0000 | 0.058 | RETURN_HOME 189 s sim |
| COLOUR | 1 | ep-25c3b9083208 | yellow → lane_1 (lane_4) | 4.0500, -0.7500 | complete | -- | 153.6 | 297 | 0.1543 / -0.0000 | 0.075 | yellow moved 1.5 m |
| COLOUR | 2 | ep-8236c0e9a885 | red → lane_4 (lane_1) | 4.0500, +0.7500 | complete | -- | 157.0 | 360 | 0.1545 / -0.0000 | 0.113 | red moved 1.5 m; climb +0.182 m |
| COLOUR | 4 | ep-216ea18e9365 | green → lane_4 (lane_2) | 4.0500, +0.7500 | failed | DESCENT_TIMEOUT | 146.5 | 278 | -- | 6.575 | climb +0.233 m; green not found ×3; descent stalled at x 4.50 |
| POSITION | 1 | ep-2ba30b0dccf6 | yellow → lane_1 (lane_4) | 4.1441, -0.7503 | complete | -- | 164.1 | 309 | 0.1543 / -0.0000 | 0.082 | +94 mm along; approach 1.260 m |
| POSITION | 2 | ep-8ef4f54d7715 | red → lane_4 (lane_1) | 4.0634, +0.7609 | failed | TARGET_NOT_FOUND | 180.8 | 478 | -- | 0.087 | climb −0.206 m; red not found ×3 |
| POSITION | 4 | ep-422c6f25f3a7 | green → lane_4 (lane_2) | 4.4186, +0.7235 | complete | -- | 318.7 | 625 | 0.1543 / -0.0004 | 0.079 | +369 mm / −26.5 mm; found at 1.565 m; RETURN_HOME 200 s sim |

**8 of 10 COMPLETE (result=fetch): FIXED 4/4, COLOUR 2/3, POSITION 2/3.**
Ten runs is not a rate, and three per randomised mode says nothing about
one.

### What each check measured

| check | result (measured) |
|---|---|
| gz spawned what the manifest says | **10/10** within 10 µm in xy, \|dz\| ≤ 2.4 µm, tilt ≤ 26.2 µrad; observed layout valid (separation, corridor, region) — `readback_reeval.txt` |
| spawned manifest = generated manifest | COLOUR/POSITION byte-identical 6/6; FIXED targets equal 4/4 |
| robot side told the right thing | `region_map` on `/mission_executive` **and** `/ramp_driver` equal to the manifest's 10/10 (FIXED: empty); no node parameter carries a manifest path 10/10 |
| mission climbed the episode's lane | Nav2 arrival at the pre-ramp pose 0.005–0.080 m from the **episode's** lane in all 10; in the 6 moved episodes 1.013–1.513 m from the colour's frozen lane |
| climb held to the episode's lane | `ramp_driver`'s cross-track is measured against the region lane (COLOUR s1: yellow in lane_1 reported +0.080 m; against its frozen lane_4 it would read ≈ −1.42) |
| approach and grasp | 8/8 runs that reached APPROACH stopped at base-x 0.1539–0.1545 (window centre 0.1537), lateral ≤ 0.4 mm; approach travel 1.147–1.168 m on the nominal row, 1.260 m at +94 mm, 1.555 m at +369 mm |
| fetched target home | 8/8 COMPLETE: target on the floor near home (z 0.079 = half its height), robot 0.019–0.119 m from home |
| wheel ownership | exactly one publisher on `/diff_drive_controller/cmd_vel` 10/10; raw-controller bypass 0 and wheels driven during a PolygonStop 0 in all 10 (`cmdpath_summary.json`) |

### The two failures

Both are lane_4 episodes that put a **thinner** target in the outer +y
lane, and both lost it at SEARCH_TARGET after an unusually large climb
drift:

| run | target | climb end cross-track | search range / bearing | outcome |
|---|---|---|---|---|
| COLOUR s4 | green 24 mm, nominal (4.0500, +0.7500) | **+0.233 m** (lane hold at its 0.8 clamp) | 1.213 m / −15.6° | not found ×3 (15 s each) → safe unwind; the scripted descent then **timed out at x = 4.50** with the robot at y ≈ 1.0, near the ramp's side → `DESCENT_TIMEOUT` |
| POSITION s2 | red 20 mm, (4.0634, +0.7609) | **−0.206 m** | 1.222 m / +13.2° | not found ×3 → descend, return home → `TARGET_NOT_FOUND` |

In both, the untouched target was still standing at its spawn pose at
the end. Every completed run's climb ended 0.016–0.182 m off its lane.

**What is known.** The region mapping was right in both (Nav2 put the
robot 0.029 / 0.045 m from the episode lane; ramp_driver's datum was the
episode lane). The placement offset was small (POSITION s2: +13 mm /
+10.9 mm) or zero (COLOUR s4) next to the robot's ~0.2 m off-lane climb
end. The COLOUR twin of POSITION s2 (same assignment, nominal pose)
completed from a +0.182 m climb at −12.3°.

**Prior evidence that the drift is not new.** C2-NAV.49 (FIXED, same
policy and lane hold) recorded lane_4 climbs of +0.095, +0.094 and
**+0.209 m** — and completed all three, with **yellow (32 mm, the
thickest target)**, which the FIXED layout always puts in lane_4 while
red (20 mm) always climbs lane_1. Randomising the colours removes that
pairing.

**What the instrumented repeats add**: the next section. Cause **not
attributed**.

### Observations that are not failures

- **RETURN_HOME is the variable leg**: 42–49 s of sim time in 7 runs,
  98 s (FIXED green), 189 s (FIXED yellow), 200 s (POSITION s4); all
  arrived within 0.058–0.119 m. `state_durations.txt`. Every other leg
  is steady: climb 12–14 s, approach 12–14 s, grasp 25 s, place 16 s.
- **One runner check failed spuriously**: FIXED blue's read-back, because
  the checker had no slack on the region's near edge and gz settled every
  target 10 µm toward the crest. Fixed in `df796dd`; the recorded poses
  pass when re-judged. The mission completed.
- **The default world launch records its own requested colour** (drawn
  from seed 0: yellow) because it is not told the mission's. FIXED runs
  therefore compare targets, not whole manifests.
- `target_finder`'s status reports the colour's **frozen** lane (e.g.
  `sel=red … lane=-0.750` while red stood in lane_4) — measured in the
  instrumented repeat, as documented in the stage C survey. Nothing
  consumes it.

## The instrumented repeats (`diag/`)

After the matrix the runner gained `/perception/status`, `/ramp/status`
and `/approach/status` stream recorders (`110f080`), and each failed
episode was run once more from the **same manifest** (same seed, level,
colour), fresh simulator, recorded `head=110f080` (package code as
`df796dd`: the matrix code plus the read-back fix; the dirty paths each
records are docs and evidence only).

| run | climb end cross-track | search | outcome |
|---|---|---|---|
| POSITION s2 (red, lane_4) | **+0.101 m** (was −0.206) | found; approach stopped, grasped, VERIFY_GRASP passed | **void**: Nav2 aborted RETURN_HOME at 10:27:13 UTC (`Goal failed`); the executive's retry 1/2 was in progress, robot at (0.34, +0.99), when the runner's 900 s wall budget expired at 10:31:01 (380.2 s sim). A second `Goal failed` at 10:31:32 falls in the teardown and is not attributed |
| COLOUR s4 (green, lane_4) | **+0.152 m** (was +0.233) | found | **COMPLETE, result=fetch** |

Both spawns re-judged: within 4.7 µm in xy (`readback_reeval.txt`
method). The first attempt at the COLOUR s4 repeat **refused to start**
(`diag/colours_s4_r2_first_attempt_refused.log`): `ros_clean.sh --list`
matched one of this session's own background shells, whose command text
contained a process-name pattern — the runner's pre-flight doing its
job. Nothing ran; the repeat was launched again with nothing matching.

**What the perception stream shows.** The requested target is first
detected *during the climb*, from about 1.40 m, as a blob of **3 × 4 px
(red, 20 mm)** and **4 × 4 px (green, 24 mm)**; for red it grew to 4 × 26
px by 1.253 m. From the slope the crest edge hides most of the 158 mm
cylinder, so at the climb end a thin target is a few pixels wide and a
few rows tall above the crest line. Detection there is marginal by
construction, and which pixels clear the crest depends on where the climb
ends.

**Reading, and its limit.** Four runs of these two episodes: the two with
climb-end cross-track 0.206/0.233 m lost the target; the two with
0.101/0.152 m found it. Every completed matrix run's climb ended within
0.182 m. That is consistent with "SEARCH_TARGET fails when a thin target
is viewed from a ~0.2 m off-lane climb end", and n = 2 per side does not
establish it. Nothing in the failures points at the episode layer: the
region mapping, the spawn, and the placement (0 and +13 mm / +10.9 mm)
were all as specified. **Not attributed.** The next measurement is a
stationary one: stand the robot at the climb-end pose at controlled
cross-tracks with each target in lane_4 and log `/perception/status`
(no mission, no climb variance).
