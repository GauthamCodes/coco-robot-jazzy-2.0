# Episodes, simulator backends, and what P0.2 already assumes

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

**Correction, same day (branch `isaac-compat-4x`,
`docs/data/isaac_compat/README.md`).** The bullets above are measurements;
the conclusions drawn from them were wrong. Re-measured on the same 4.5
install: the viewport "never returns" was a first-launch RTX shader compile
(**385.7 s** cold, **20.7 s** warm GUI, **21.7 s** warm headless) that the
earlier runs were killed during; rendering works (1280×720 at 49.9 fps,
VRAM 2 111 MiB); the LLVM abort is the bundled **Humble Fast-DDS**
misreading a **Jazzy** peer's discovery `Gid` (gdb backtrace), not memory.
With **CycloneDDS** — already COCO's RMW — Jazzy received RGB 320×240,
depth 320×240, a 480-beam scan, `/clock`, odometry and TF, and Twist moved
the body within 1 % of the commanded distance (measured). So stage L is
**not hardware-blocked** at COCO's sensor scale, as an
experimental, unsupported configuration (no Isaac release supports Jazzy or
Ubuntu 24.04). What remains unmeasured is COCO's own robot, world and
stack on it.

## 6. Next stage

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
