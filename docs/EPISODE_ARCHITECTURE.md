# Episodes, simulator backends, and what P0.2 already assumes

**§0 is stage C (branch `p03c-episode-gazebo`, 2026-09-26) and is the
current state. §1–§6 are stage B as written on 2026-09-24 and are kept as
the record they were; where §0 supersedes them it says so.**

## 0. Stage C — the episode drives the Gazebo world

Branch `p03c-episode-gazebo`, from `p03-episode-spec` @ `b3c6598` (the
verified episode baseline). Written 2026-09-26. **(measured)** = produced
in this session; **(derived)** = computed from `coco_config`.

**What stage C is.** `EpisodeSpec` is now the source of truth for where
the fetch targets stand. `full_world_robo.launch.py` spawns them from a
manifest through a Gazebo backend; the mission is told which *region*
the requested colour stands in, by name, and resolves the lane from a
static table. The default is `fixed`, which is the P0.2 world byte for
byte. Nothing searches yet: the mission still drives to a pre-ramp pose
chosen before it can see the platform. That is stage F.

**This branch is not `main`.** `main` (`b15d445`) and this lineage
diverged at `d317d85`. `main` carries the expanded 24 × 18 m arena, whose
four colours stand in four separate bays at y = −6/−2/+2/+6 on a 1.2 m
platform; it has neither the P0.2 web platform nor the episode
specification. This branch has both and the original single-platform
world. They cannot be merged by `git` alone — the target geometry
differs — and nothing here tries to. The region model below is the
seam that merge will use: on `main`'s world a region is a bay instead of
a lane, and nothing that consumes regions has to change.

### 0.1 Every remaining fixed-target assumption, classified

Surveyed at `b3c6598`. A = simulator generation only, B = mission /
navigation dependency, C = perception dependency, D = manipulation
safety constraint, E = test or diagnostic fixture, F = obsolete.

| Where | What | Class | Stage C |
|---|---|---|---|
| `coco_config/robot.py` `TARGETS[*].lane_y`, `TARGET_ROW_X`, `lane_for_colour`, `colour_for_lane` | the frozen colour→lane table and row | A + B (source) | **kept**; regions are derived from it, and `resolve_lane(colour, {})` *is* `lane_for_colour` |
| `full_world_robo.launch.py` inline target SDF + `(TARGET_ROW_X, lane_y)` spawn | where gz puts the targets | A | **replaced** by `GazeboBackend` over the manifest; fixed output asserted byte-identical |
| `magnet_release.py --models` | releases each target by model name | A | now from the backend's model list — the same four names |
| `mission_states.MissionPlan` `lane_for_colour` → pre-ramp goal, ALIGN lane check, CLIMB cross-track fallback | the lane the robot climbs | **B** | resolved through the episode's region map; empty map = the frozen table |
| `mission_executive` `lane` param, `--lane`, `_on_colour` plan rebuild | lane override / colour change | **B** | `region_map` parameter; `_on_colour` carries it (it would otherwise fall back to the table mid-episode) |
| `coco_rl/ramp_driver.py` `_on_colour` → `lane_for_colour` | the climb's lateral-hold datum and the published cross-track | **B** — a control input, not a report | `region_map` parameter. Without it a COLOUR episode's climb is steered toward the colour's *frozen* lane, and the executive's CLIMB check reads that cross-track |
| `target_finder` status `lane=lane_for_colour(sel)` | reported lane | C, report-only (no consumer found in `coco_mission`, `coco_web`, `custom_teleop`) | **unchanged** — `coco_perception` is CLAUDE.md rule-4 frozen. In COLOUR/POSITION episodes the `lane=` field names the frozen lane. Known, recorded, harmless to control |
| `target_finder` / `target_pose` / `grasp_server` `target_by_colour` | diameter, model name, magnet | D (identity, not location) | unchanged: a colour's object keeps its model, diameter and magnet wherever it stands |
| `approach_server` `approach_stop_x(colour)`, `GRASP_*`, `approach_window` | the 5.5 mm grasp window | D | unchanged; the POSITION area is bounded so the approach it relies on is the one measured |
| `episode.py` envelope (row, band, separation, corridor) | p03's safety geometry | D | unchanged and still checked first; region checks run after it |
| `traverse_demo.py`, `nav_round_trip.py`, `plan_compare.py` `lane_for_colour` | measurement harnesses | E | unchanged; FIXED-only by design (the M4–M6 numbers were measured with them) |
| `vision_check.py` lanes, `TARGET_ROW_X` | perception diagnostic | E | unchanged (rule 4) |
| `platform_server.world_geometry()` targets at the frozen lanes | the browser's drawing | A (display for a human) | **unchanged, and wrong in COLOUR/POSITION**: the page draws the frozen layout. Nothing reaches the robot from it |
| `scripts/browser_check/fakestack.py`, tests pinning lanes 0.25/0.75 | fixtures | E | unchanged |
| p03's `positions` envelope (±0.234 m about a lane, x over [3.39, 4.40]) | generator | F | **replaced** by the region placement area (§0.3). No `positions` episode had been spawned or run |

### 0.2 Target regions

A region is a named place a target may stand, with no colour attached:

```
TargetRegion(region_id, platform, lane_y, row_x)        coco_config.robot
    lane_1  crest  -0.75  4.05        (ordered by y, derived from TARGETS)
    lane_2  crest  -0.25  4.05
    lane_3  crest  +0.25  4.05
    lane_4  crest  +0.75  4.05
```

- `region_id` — what an episode assigns a colour to.
- `platform` — which platform (and the ramp up to it) it is on.
- navigation approach — the line up the ramp at `lane_y`, entered from
  the mission's flat-ground pre-ramp pose `(PRE_RAMP_X, lane_y)`.
- allowed target area — `coco_sim.episode.region_area(region, diameter)`.
- manipulation constraints — `approach_window`, unchanged, plus the p03
  envelope.

The table is static world knowledge of the same kind as the map. What
an episode adds is the **assignment** colour → region, which lives in the
manifest (`TargetSpec.region_id`) and is privileged.

```
requested colour ──► episode region map ──► region ──► lane_y ──► Nav2 / climb
      (task)          (manifest, compat)     (name)    (static table)
```

With no region map the middle step is skipped and this is exactly
`lane_for_colour` — that is how FIXED stays FIXED.

### 0.3 The three modes

| mode | `episode_level` | colour → region | pose in region |
|---|---|---|---|
| FIXED | `fixed` (default) | the frozen table | nominal `(row_x, lane_y)` |
| COLOUR | `colours` | seeded permutation (first draw) | nominal |
| POSITION | `positions` | the same permutation as COLOUR for that seed | drawn in `region_area` |

**The POSITION area** (derived), and why each side is where it is:

- **Across the lane: ±0.030 m** (`REGION_LATERAL_LIMIT`). The largest
  lateral offset the approach has been measured to absorb: +0.030 m
  commanded reached the grasp as −3.0 mm and grasped (C2-M4.1,
  `PROJECT_STATE.md`, n = 1). It is where the evidence stops, not a
  characterised limit.
- **Along the lane: from the frozen row outward only**, to
  `PLACEMENT_FILL` (0.85) of the platform's far bound: x ∈ [4.0500,
  4.4240] for the 20 mm target, [4.0500, 4.4189] for the 32 mm one
  (derived). A nearer target would shorten the blind crest drive's
  clearance and the camera's stand-off at servo start below what every
  measured fetch had; a farther one only lengthens the closed-loop servo
  and stays inside `target_finder`'s 2.0 m range gate from the end of the
  climb. **No target off the frozen row had been driven to when this was
  set** — the matrix below is the first measurement.

The p03 envelope is still checked first, so every p03 rejection message
is unchanged; the region checks (known region, one target per region,
inside the area) run last.

### 0.4 The switch

```bash
# FIXED — the default. Both files behave exactly as before episodes.
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py target_colour:=red

# COLOUR / POSITION from a seed — give BOTH launches the same values
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false \
    episode_level:=colours episode_seed:=1 episode_colour:=yellow
ros2 launch coco_mission mission.launch.py episode_level:=colours \
    episode_seed:=1 target_colour:=yellow

# Better: one recorded manifest to both (what the matrix did)
coco_episode generate --level positions --seed 4 --colour green --out m.json
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false \
    episode_manifest:=m.json episode_record:=spawned.json
ros2 launch coco_mission mission.launch.py episode_manifest:=m.json
```

`full_world_robo.launch.py` resolves an episode only under
`traverse:=true`; asking for one without the platform is refused, and the
curriculum worlds (`ramp_angle:=12/24`, no traverse) never resolve one.
A manifest resolved at another grade than the world is being built at is
refused. `episode_record` writes the manifest the world was built from.

### 0.5 What the robot receives, and what it never does

**The robot side** (every node under `mission.launch.py`) receives:

| input | carries | from |
|---|---|---|
| `target_colour` | the task — `task_view()["requested_colour"]` | launch argument / manifest |
| `region_map` parameter on `mission_executive` and `ramp_driver` **only** | colour → region **names**, e.g. `blue=lane_2,green=lane_1,red=lane_4,yellow=lane_3` | `compat_mission_inputs(spec)`; empty in FIXED |
| the static region table | region → `lane_y` | `coco_config`, like the map |

It never receives a target x, y or z, an obstacle pose, the seed, the
level, the backend, or the manifest (a test asserts no Node block in
`mission.launch.py` mentions it, and that nothing the resolver sets
carries a coordinate). `task_view()` is unchanged:
`{episode_id, requested_colour}`.

**The region map is privileged.** It tells the robot which lane a colour
is in, which is what it should eventually find out for itself. It is a
*compatibility* channel for a mission that must commit to a lane before
the crest occludes nothing — named `compat_…`, kept out of `task_view()`,
and the thing stage F deletes.

**The simulator and the evaluator receive the manifest**: every target's
pose, region, diameter, model; the seed, level, backend, ramp grade,
robot start. `GazeboBackend` turns it into spawns; `coco_episode
readback` compares gz with it; `EpisodeResult` embeds it.

### 0.6 Reproducing an episode from a seed

```bash
coco_episode generate --level positions --seed 4 --colour green --out m.json
coco_episode check --manifest m.json      # "reproducible" or "DRIFTED"
```

Same seed, level, backend, world and requested colour → the same
manifest, byte for byte (measured: sha256 identical at all three levels,
`docs/data/p03c_episode_gazebo/episode_evidence.txt`). The layout does
not depend on the requested colour (it is drawn after every placement),
so the world launch and the mission launch agree on where everything is
even if only one of them pins the colour. A recorded manifest replays
exactly without its seed (`episode_manifest:=`), including a hand-edited
one — it must still validate.

**Generator change.** The `positions` level's poses changed at this
branch (region-local area). The colour assignment per seed did not (the
permutation is still the first draw). A result recorded against the p03
generator fails `check_reproducible` — as it should — and a p03 manifest
without `region_id` loads but does not validate.

### 0.7 One manifest, two backends

```
                  EpisodeSpec (coco_sim.episode)       semantics, once
                            │
           coco_sim.backends.common.TargetBody        mass, inertia, friction,
                            │                          colour, pose — once
              ┌─────────────┴──────────────┐
      GazeboBackend.translate()     IsaacBackend.translate()
      SDF + `create` argv           USD prim specs (data only)
              │                            │
      full_world_robo.launch.py     (not wired: see below)
              └──────── ROS 2 contract (§4) ────────┘
                            │
              arbiter · Nav2 · perception · MoveIt · executive
```

A backend never draws a number, picks a target or moves one; it refuses
a manifest recorded for another engine and re-validates. The friction
coefficient (1.5) and the inertia formula moved from the launch file into
`backends/common.py`, so both engines build the same body.

**What Isaac needs from an episode, and where each piece stands:**

| piece | source | status |
|---|---|---|
| robot | generated from `coco_config` | exists on `p03-isaac-backend` (`ffb3fc6`) |
| initial condition | `EpisodeSpec.robot_start` | consumed there today (`generate_episode(level='fixed', backend='isaac')`) |
| targets | `IsaacBackend.translate(spec).targets` | **this branch; not consumed by any Isaac code** |
| world geometry (arena, ramp up, platform, ramp down) | none for Isaac | **missing** — the Isaac runtime builds a fixture world; `IsaacScene.missing` says so and a test pins it |
| task | `task_view()` + the same compat region map | the ROS contract is unchanged |

So the adapter boundary exists and is tested, and **Isaac cannot
instantiate a `coco_world` fetch episode yet.** The next Isaac step is a
generated USD of the ramp/platform from `coco_config` (rule 3, as the
MJCF is), then `build_scene(stage, episode)` iterating
`IsaacBackend.translate(episode).targets`. Nothing on this branch touches
`~/coco-isaac-backend` or `coco_sim/isaac/`.

### 0.8 Measured

**Tests** (per package, cwd inside it, private ROS domain, overlay built
from this tree, MoveIt prefix on the path): **1877 passed, 0 failed, 0
skipped**, against **1740 / 0 / 0 measured on `b3c6598` in the same
session**. Every pre-existing test still passes; one pre-existing
assertion was extended (`test_the_manifest_is_complete` gains
`region_id`). New: coco_config +22, coco_rl +11, gazebo_models +13,
coco_sim +71, coco_mission +20.

**Generator** (`docs/data/p03c_episode_gazebo/episode_evidence.txt`):
same seed → byte-identical manifest at all three levels; **10000/10000**
seeds valid and inside their own region at every level; 0 blocked
corridors; COLOUR and POSITION reach all **24** colour→region
assignments, and the requested colour is off its frozen lane in 7482 /
7454 of 10000; POSITION dx ∈ [0, +0.3739] m, dy ∈ [−0.0300, +0.0300] m.

**Gazebo** (`docs/data/p03c_episode_gazebo/README.md`, ten fresh runs,
headless, the C2-NAV.44 runner plus the episode checks):

- **gz built every manifest**: 10/10 within 10 µm in xy, |dz| ≤ 2.4 µm,
  tilt ≤ 26.2 µrad, observed layout valid.
- **The mission resolved every mapping**: region map on the executive
  and ramp_driver 10/10 as expected; Nav2 arrival 0.005–0.080 m from the
  episode's lane in 10/10, 1.013–1.513 m from the frozen lane in the six
  moved episodes; ramp_driver's cross-track measured against the episode
  lane.
- **8 of 10 COMPLETE**: FIXED 4/4, COLOUR 2/3, POSITION 2/3. Approach
  stop 0.1539–0.1545 (window centre 0.1537) in all 8 that reached it,
  including the +369 mm target (found at 1.565 m).
- **Both failures** were lane_4 episodes with a thinner target (green 24
  mm nominal; red 20 mm +13/+10.9 mm) lost at SEARCH_TARGET after climbs
  that ended 0.233 / 0.206 m off-lane; one then hit `DESCENT_TIMEOUT` at
  the platform's far edge. Each was repeated once from the same manifest
  with perception recorded: both found their target (climbs 0.152 /
  0.101 m); one COMPLETE, one `void` (return leg outlasted the 900 s wall
  budget after a Nav2 abort). First detections were 3 × 4 and 4 × 4 px
  blobs at ~1.40 m. **Not attributed**; not an episode-layer fault by any
  evidence gathered.
- **Safety**: one publisher on `/diff_drive_controller/cmd_vel` in every
  run; raw-controller bypass 0 and wheels driven during a PolygonStop 0
  in all ten.

### 0.9 Limitations (evidence-backed only)

- **Thin targets in lane_4 after a large climb drift** — the two failures
  above. The FIXED layout always puts the 32 mm target in lane_4 and the
  20 mm one in lane_1; randomising colours exposes a pairing FIXED never
  exercised. C2-NAV.49 already measured a +0.209 m lane_4 climb (with
  yellow, completed).
- **RETURN_HOME is slow and variable** (42–200 s sim), with one Nav2 abort
  in the repeats — the return leg's known class (C2-NAV.46, P0.2), seen
  in FIXED runs too.
- **`target_finder` reports the frozen lane** in its status line under
  COLOUR/POSITION (measured: `sel=red … lane=-0.750` with red in lane_4).
  Rule 4 keeps `coco_perception` untouched; nothing consumes the field.
- **The browser draws the frozen layout** in every mode.
- **POSITION is conservative**: ±30 mm across (the largest measured
  absorbed offset) and outward-only along. Nothing here measures beyond.
- **Isaac cannot instantiate a fetch episode**: the adapter translates
  targets; the world geometry does not exist for Isaac.
- **10 runs + 2 repeats is not a rate.**

### 0.10 Next stage

The chain `EpisodeSpec → GazeboBackend → ROS 2 contract → mission` is
built and exercised in all three modes; `EpisodeSpec → IsaacBackend` is a
tested translation with no consumer. Ready for, in order:

1. **The lane_4 search failure, measured stationary** — the robot placed
   at the climb-end pose at controlled cross-tracks with each target in
   lane_4, `/perception/status` logged; no climb variance.
2. **Isaac world geometry** generated from `coco_config` (rule 3), then
   `build_scene` consuming `IsaacBackend.translate()` — the first time an
   Isaac run instantiates an episode.
3. **Perception-driven target execution (stage F)** — replace the region
   map with a search; delete `compat_mission_inputs`.

Not before those: dynamic obstacles, Isaac Lab, a full randomised
end-to-end mission. The `main` arena (four bays) joins as a region table
of bays, once this branch and `main` are reconciled by their owner.

---

Branch `p03-episode-spec`, from `p02-release-candidate` @ `c40098f`.
Written 2026-09-24. Every number marked **(measured)** was produced in that
session; **(derived)** means computed from `coco_config`; anything else is
design, and says so.

This is stage B of the post-P0.2 roadmap — the *reproducible episode
specification* — plus the assessment it rests on. P0.2 is frozen: nothing
here changes a spawned pose, a mission transition, a topic or a parameter
of the running stack.

---

## 1. Where P0.2 embeds world knowledge (surveyed at `c40098f`)

`coco_config/coco_config/robot.py` is already the single source of the
fetch world. The survey found almost nothing re-typed; it found the
*direction* of knowledge flow is the problem — the robot is handed
coordinates.

| Assumption | Where | Nature |
|---|---|---|
| colour → lane y | `robot.py:326-335` (`TARGETS[*].lane_y`), `lane_for_colour()` `robot.py:348` | **the** hidden-knowledge lookup |
| robot is sent to a lane before it can see anything | `coco_mission/scripts/mission_states.py:715-717, 734-736` (`plan.lane`, `pre_ramp`) | mission consumes the lookup |
| perception reports a lane, not only a detection | `coco_perception/coco_perception/target_finder.py:522, 529` | perception consumes the lookup |
| target row x | `robot.py:325` `TARGET_ROW_X = 4.05` | fixed |
| spawn positions | `gazebo_models/launch/full_world_robo.launch.py:276-281` spawns from `TARGETS`, inline SDF `:246-272` | Gazebo-only, not from a manifest |
| ramp/platform geometry | `robot.py:182-199`; launch `:200-224` | parameterised by `ramp_angle:=` only |
| static arena | `gazebo_models/worlds/coco_world.world` (walls, 2 boxes, cylinder, gate cubes, pilasters) | frozen literal world |
| world identity | `full_world_robo.launch.py:319` `world:=`; **`mission.launch.py` does not expose it**; `coco_rl/ramp_env.py:63` `WORLD = 'coco_world'` | no variant abstraction |
| seed | `coco_sim.yard.sample_yard` and the MuJoCo envs only | **the Gazebo/ROS fetch path has none** |
| episode | RL-internal only (`coco_rl`, `coco_sim`); `YardSample` is terrain + dynamics, no fetch content | no mission-level episode |
| browser geometry | `coco_web/coco_web/platform_server.py:876-911` `world_geometry()` sends target positions to the UI | fine for a human; must never reach the robot |

Duplicates that will drift (reported, **not** changed — P0.2 is frozen):
`PRE_RAMP_X = 0.5` (`mission_states.py:209`, `gazebo_models/scripts/traverse_demo.py:87`);
`WORLD_TO_MAP_X = 2.0` literal (`gazebo_models/scripts/nav_round_trip.py:64`);
the lane table re-typed in `scripts/browser_check/fakestack.py:59-60`;
`FALLBACK_COLOURS` (`coco_web/coco_web/protocol.py:60`, test-guarded);
HSV bands only in `target_finder.py:120-123` (deliberate);
`gen_ramp.py:111,113` CLI defaults `--run 2.5 --width 2.0` are the swapped
pre-fetch values (the launch always overrides them);
`coco_rl` imports `coco_sim` without declaring it in `package.xml`.

## 2. Assessment

1. **Can stay.** `coco_config` as the physical source of truth; the grasp
   envelope as measured (`GRASP_SELF_COLLISION_X`, the 5.5 mm window);
   the mission executive's pure state machine; Nav2 / MoveIt / perception;
   the arbiter as sole wheel publisher; the `coco.v1` protocol; `coco_rl`
   and the shipped policy (`phase5_24deg_s0p0.zip`) as the terrain
   specialist baseline; `coco_sim.yard` as the terrain generator;
   `coco_world.world` frozen as the v1 world.
2. **Must become parameterised.** Target poses (spawn from a manifest, not
   from `TARGETS`); the world variant through `mission.launch.py`; the
   requested colour's *location* must stop being a mission input (stage F).
3. **Must become generated.** Target spawn SDF (today inline f-strings in
   the launch file); world variants (as `coco_yard.world` already is);
   episode-added obstacles.
4. **Must stay deterministic.** `level='fixed'` — the P0.2 layout — is the
   default until a randomised level is validated in simulation. The
   measured constants are never sampled. The four `TIP_LIMIT`s stay four.
   Difficulty is never made by changing Nav2 tolerances.
5. **Shared by Gazebo and Isaac.** The episode manifest; `coco_config` as
   the robot's dimensions (a USD must be *generated* from it, rule 3,
   exactly as the MJCF is); the ROS 2 contract in §4; the result record.
6. **RL only.** Physics domain randomisation (friction, torque scale,
   payload, yaw gain, IMU noise — `yard_params.yaml` `randomisation`),
   rewards and terminators, MuJoCo, and later Isaac Lab tasks. These are
   privileged by design: reward and evaluation may read them, observations
   may not.
7. **Web platform only.** Session lifecycle, health, the `coco.v1`
   protocol; later, which episode parameters a user may *request*
   (level, seed, backend, world). It may render the manifest's geometry for
   a human; it must forward only `task_view()` toward the robot.

## 3. The episode specification (implemented)

`coco_sim/coco_sim/episode.py`. It lives in `coco_sim` because that package
already owns the only seeded generator and depends on `coco_config` alone —
the graph stays acyclic (rule 6). Imports: stdlib + `coco_config`. No numpy,
no ROS, so the web platform or an Isaac process can read a manifest.

```
seed ──► generate_episode(seed, level, backend, world_variant, …)
            │  one random.Random(seed); every draw from it
            ▼
        EpisodeSpec  (frozen dataclass)
            ├── manifest()   PRIVILEGED: poses, obstacles, seed …
            └── task_view()  ROBOT: {episode_id, requested_colour}
            ▼
        validate_episode()  raises InvalidEpisode
            ▼
        record_result(spec, outcome, …) ──► EpisodeResult (embeds manifest)
                                              └── check_reproducible()
```

| Field | Meaning |
|---|---|
| `episode_id` | `ep-` + 12 hex of sha256(seed, level, backend, world, requested colour) — reproducible, so an id in a table regenerates the run |
| `seed` | the only input to layout |
| `level` | `fixed` (default, P0.2 exactly) · `colours` (permute lanes) · `positions` (permute + jitter inside the envelope) |
| `backend` | `gazebo` · `isaac` · `mujoco` — recorded, changes nothing about the layout (tested), changes the id |
| `world_variant` | `coco_world` · `coco_yard` |
| `requested_colour` | seeded draw from the episode's own targets unless pinned |
| `targets` | `TargetSpec(colour, model, x, y, z, diameter, height)` |
| `obstacles` | `ObstacleSpec(name, kind, x, y, z, yaw, size, motion, enabled, waypoints, speed)` — **represented, never generated**; the frozen world's own obstacles stay the world file's business |
| `robot_start` | `SPAWN_XY`, `SPAWN_Z` |

**The boundary.** `task_view()` is exactly `{episode_id, requested_colour}`.
A test serialises it for 50 seeds × 3 levels and asserts no target
coordinate (repr or 2-dp) and no `lane`/`backend` appears. The robot is
told *what* to find, never *where*.

**The envelope, all (derived) from `coco_config`:**

| Bound | Derivation | Value |
|---|---|---|
| half footprint x | `WHEELBASE/2 + WHEEL_RADIUS` | 0.1485 m (= the literal in `test_targets.py`, now derived) |
| half footprint y | `WHEEL_SEPARATION/2 + WHEEL_WIDTH/2` | 0.1570 m |
| target x | crest + `approach_stop_x` + half footprint x … far edge − radius | [3.3022, 4.4900] (20 mm), [3.3022, 4.4840] (32 mm) |
| target y | ±(`RAMP_WIDTH/2` − half footprint x) | ±1.1015 m |
| pairwise | half footprint y + both radii | per pair |
| **approach corridor** | no nearer cylinder within half footprint y + its radius of a target's +x approach line | per pair |
| grasp | `approach_window(colour)` non-empty | all four |
| z | `RAMP_RUN·tan(RAMP_ANGLE_DEG)` + height/2 | resting on the platform |

The corridor rule was **added after measuring its absence**: the first
`positions` generator (commit `f971078`) blocked a corridor in **1012 of
10000** episodes, **276** of them the requested target's (measured). After
`16e575b`: **0 / 10000** at every level (measured).

**Results.** `EpisodeResult(episode_id, outcome, manifest, failure_reason,
timings, software_commit, policy_version, measurements)`. Outcomes
`complete|failed|aborted|void`; a failure must say why; **every timing key
must end `_sim_s` or `_wall_s`** (P0.2's sim clock runs at RTF ≈ 0.46).
`check_reproducible(result)` regenerates from the seed and compares — False
means the generator drifted and the old and new runs are different tasks.

Evidence: `docs/data/p03_episode_spec/`.

## 4. The common simulator contract (design — not implemented)

```
                 EpisodeSpec.manifest()
                 ┌──────────┴──────────┐
        Gazebo world builder     Isaac stage builder      (stage C / L)
                 │                     │
                 └──── ROS 2 contract ─┘   ◄── the only thing COCO sees
                          │
           arbiter · Nav2 · perception · MoveIt · executive
                          │
                 EpisodeResult (per run)
```

A backend is *any* process that, given a manifest, produces this graph:

| Contract | Topic / interface | Notes already paid for |
|---|---|---|
| command | `/diff_drive_controller/cmd_vel`, **`TwistStamped`**, from the arbiter only | a `Twist` subscriber is silently blind |
| odometry | `/diff_drive_controller/odom`; model odometry for liveness | liveness = COCO's odometry *arriving* |
| TF | `odom → base_footprint → …` + `robot_state_publisher` from the xacro | |
| LiDAR | `/scan` | |
| RGB / depth | camera topics, **BEST_EFFORT**; RPY (0,0,0) | a RELIABLE subscriber is silently blind |
| IMU | as `coco_config.SENSOR_TOPICS` | |
| clock | `/clock`, `use_sim_time` | foreign `/clock` publishers exist on this machine |
| arm | `ros2_control` joint trajectory controllers as in `coco_controllers.yaml` | |
| task | `task_view()` only | never the manifest |

Physics is not required to match; the difference is to be **measured**
(the MuJoCo cross-engine parity work, recorded in `PROJECT_STATE.md`,
is the template).

## 5. Isaac Sim on this machine — what was measured

Full record: `docs/data/isaac_foundation/README.md`. Summary:

- NVIDIA's current minimum ([requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html),
  page updated 2026-09-18, 6.x docs): RTX 4080,
  **16 GB VRAM, 32 GB RAM**, 50 GB SSD, driver **595.58.03**. This machine:
  RTX 4050 Laptop **6141 MiB**, **15696 MiB** RAM, driver **580.173.02**
  (measured). Below minimum on four counts, so **Isaac Sim 6.1 was not
  downloaded.**
- An existing **Isaac Sim 4.5.0** pip install (`~/isaac-sim/venv`) was
  tested instead. Default `SimulationApp(headless)` **never returns**: a
  stack dump at 360 s shows it in the unbounded `_wait_for_viewport` loop
  (`simulation_app.py:520`) (measured).
- With `create_new_stage=False` it starts in **12.4 s** and **physics
  works**: 60 s, 5114 steps, peak RSS **4.77 GB**, clean exit (measured).
- **Rendering is not viable**: forced app updates took ~401 s for 10, then
  `LLVM ERROR: out of memory` (measured). RTX cameras/LiDAR — COCO's
  sensor contract — therefore cannot run here.
- **ROS 2**: the bridge falls back to bundled Humble `rclpy` in a clean
  environment; a Jazzy process on a private domain **discovered** Isaac's
  endpoints; **no message was delivered** — 2 of 2 bridge-loaded runs
  aborted (`LLVM ERROR: out of memory`) seconds into publishing, while the
  same loop without the bridge ran clean. Cause **unknown**.

**Consequence for the roadmap:** stage L (Isaac as a second ROS 2 backend)
is **blocked on hardware**, not on design. Everything up to it — stages
C–K — is Gazebo work and does not need Isaac.

## 6. Next stage

**Done in §0 (stage C, 2026-09-26).** Kept as written:

**Stage C — seeded Gazebo world generation.** Make
`full_world_robo.launch.py` spawn targets from an `EpisodeSpec` instead of
from `TARGETS`, behind an explicit switch whose default is `level=fixed`,
so the P0.2 mission is byte-for-byte unchanged:

```
ros2 launch coco_mission mission.launch.py episode_seed:=1827 episode_level:=fixed
```

then prove the spawned poses equal `manifest()` (read back from gz), then
re-run one fetch per colour on `fixed` before touching `colours`. The
`colours` and `positions` levels are **geometrically** validated only; no
randomised episode has been spawned or driven — and the P0.2 mission
*cannot* complete a `colours` episode, because it still navigates by
`lane_for_colour()`. That is stage F, and it is why the switch defaults to
`fixed`.
