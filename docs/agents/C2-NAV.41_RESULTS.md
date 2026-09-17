# C2-NAV.41 — the shipped configuration toured in topology B: REJECTED as a candidate; the shipping command path is measured unsafe

**Agent:** implementation → validation → decision. 2026-09-17 UTC.
Everything below was measured in this session unless it says otherwise.
The topology-A arm is C2-NAV.39's three fresh tours, re-analysed here and
not re-run.

## 0. Premise

The sprint brief asked for "the C2-NAV.41 result" to be classified. **There
was no C2-NAV.41 result.**

- No results file, no commits, no run directories.
- Nothing on `jazzy2/worktree-c2nav0-diagnosis` after `46191ef`
  (C2-NAV.40's results commit).

C2-NAV.40 §11 had specified the work and it had not been run: give
`nav_tour_run.sh` a topology-B mode, run `baseline.yaml` × 3 on fresh
simulators, and judge the result against C2-NAV.39's topology-A 17/21. This
session did that, then classified the result.

## 1. Commits (`worktree-c2nav0-diagnosis`)

| commit | what |
|---|---|
| `2358789` | topology-B tour mode: `topology` experiment key, the arbiter started with `initial_mode:=nav`, `verify-topology` live readback, `baseline_topology_b.yaml`, `c2nav41_topology.py` |
| `221b7dd` | analysis: explicit opt-in (`--a-legacy`) for runs whose manifests predate the topology key |
| `f9ef036` | analysis: `stop_breach` and `bypass_source` — what the wheels follow when the monitor is bypassed |
| `949f2b6` | analysis: arm summary, plus the committed data bundle `docs/data/c2nav41_topology.json` |

## 2. What was built

- **`topology: A|B`** in experiment files. The default is A, so every
  existing experiment keeps its meaning. It is neither a nav2 parameter nor
  a benchmark goal, so it never produces a merged parameter file.
- **`nav_tour_run.sh`, topology B:**
  - starts `custom_teleop arbiter.launch.py initial_mode:=nav`, waits for
    `/cmd_vel_arbiter/status`, then starts `nav.launch.py arbiter:=true`;
  - `initial_mode:=nav` is not optional: nothing publishes `/mission/mode`
    in a tour, and an idle arbiter forwards nothing (C2-NAV.21: 0.000 m).
- **`nav_params_overlay.py verify-topology`** reads the wheel topic's
  publishers off the **live graph**. A run fails with exit 8 unless:
  - there is exactly 1 publisher;
  - that publisher is the one the topology names (`cmd_vel_relay` for A,
    `cmd_vel_arbiter` for B);
  - B's arbiter reports `mode=nav`, or no arbiter status exists for A.

  Topology A's "no arbiter" check succeeds on seeing nothing. Its positive
  control is the non-empty publisher list read in the same pass, and a test
  asserts that an unreadable graph fails.
- **`docs/data/c2nav41_topology.py`** imports C2-NAV.39's and C2-NAV.40's
  per-leg definitions rather than restating them. It adds:
  - `monitor_authority`
  - `stop_breach`
  - `bypass_source`
  - `summarise`
- **Resolve check.** `baseline.yaml` and `baseline_topology_b.yaml` both
  resolve to params sha256 `6f61e499…` with 0 parameter changes and 0 goal
  changes. **The only variable between the arms is which processes own the
  wheels.**

## 3. Build and tests (measured this session)

- **Build:** `colcon build --symlink-install --packages-select gazebo_models custom_teleop`
  → 2 packages, rc 0. The installed `nav2_params.yaml` sha256 equals source.
- **Tests** (cwd = package directory, clean ROS graph):

| suite | result |
|---|---|
| `gazebo_models` (`--ignore=test_integration`) | **103 / 0** (was 80; +23 topology tests) |
| `custom_teleop` | 67 / 0 |
| `coco_config` | 70 / 0 |
| `c2nav41_topology.py selftest` | 18 / 18 |
| `c2nav39_tour_report.py selftest` | 27 / 0 |
| `c2nav36_diag.py selftest` (capture validation) | 51 / 0 |

`coco_mission` was not re-run: nothing it imports changed.

## 4. Runs (real COCO simulation, fresh simulator each)

### Topology B — this session

| run | UTC | git | legs | live topology | live params | driven goals | nav_bench exit | `ros_clean` |
|---|---|---|---|---|---|---|---|---|
| `baseline_topology_b_r01` | 04:36:33–04:43:07 | `2358789` | **5/7** | OK: 1 publisher, `cmd_vel_arbiter`, `mode=nav` | 12/12 | 7/7 | 0 | 0 matched |
| `baseline_topology_b_r02` | 04:43:21–04:49:57 | `221b7dd` | **4/7** | OK | 12/12 | 7/7 | 0 | 0 matched |
| `baseline_topology_b_r03` | 04:50:20–(killed) | `221b7dd` | **VOID** | OK | 12/12 | — | — | — |
| `baseline_topology_b_r04` | 05:02:46–05:08:31 | `221b7dd` | **2/7** | OK | 12/12 | 7/7 | 0 | 0 matched |

`2358789` → `221b7dd` changed only the offline analysis script, not the
runner, launch files or parameters.

**r03 is VOID.** The Claude Code session that launched it ended mid-tour.
The leg `open_space` SUCCEEDED, then the run was killed during
`wall_adjacent`. The manifest has no `finished_utc` or `exit_code`, and there
is no results JSON, outcome file or teardown log. The directory holds a
`VOID.txt` with this evidence. It is excluded from every number, and r04
replaces it.

**Orphans.** Checked at 05:01:56 and 05:08:47 UTC. Zero processes matched
`gz sim`, `component_container_isolated`, `nav_bench`, `parameter_bridge`,
`cmd_vel_arbiter`, `cmd_vel_relay` or `robot_state_publisher`.

**The competing-simulator guard was tested live.** A second
`nav_tour_run.sh` was started while r01 ran. It exited **3**, refused, and
listed the live stack (including `nav.launch.py arbiter:=true`) without
touching it.

### Topology A — C2-NAV.39, not re-run

`baseline_r01..r03`, 2026-09-15 20:01:48–20:23:33 UTC, same params sha256.

- Their manifests predate the topology key, so they are aggregated with
  `--a-legacy` and labelled legacy.
- At that commit the runner hard-coded `arbiter:=false` and started no
  arbiter. That is an assertion from source, not a live readback.
- nav_bench exit codes: r01 **139** (the known shutdown segfault, after its
  JSON was written), r02 0, r03 0.

## 5. Topology A vs topology B (Step 1)

`python3 -P docs/data/c2nav41_topology.py compare --a-legacy --a ~/coco_nav_runs/baseline/baseline_r0{1,2,3} --b ~/coco_nav_runs/baseline_topology_b/baseline_topology_b_r0{1,2,4}`

| measure | A (harness: `nav.launch.py` alone) | B (shipping path: through the arbiter) |
|---|---|---|
| total successful legs | **17/21** | **11/21** |
| ordinary legs | **15/15** | **11/15** |
| `enclosure_entry` | 2/3 | **0/3** |
| `enclosure_exit` | 0/3 | 0/3 |
| statuses | 17 SUCCEEDED, 4 TIMEOUT | 11 SUCCEEDED, 5 TIMEOUT, **5 ABORTED** |
| tour sim s (median) | 263.67, 234.83, 191.21 (234.83) | 257.63, 262.55, 185.31 (257.63) |
| minimum true geometric clearance | 0.2423 m | **0.2266 m** |
| PolygonStop activations / seconds | 3 / **176.38** | 3 / 5.2 |
| deadlocks | **3** (every `enclosure_exit`) | 0 |
| terminal position error, SUCCEEDED legs, median | 0.090 m | 0.108 m |
| terminal \|heading\| error, SUCCEEDED legs, median | 0.414 rad | 0.385 rad |
| wheels exceeded collision monitor (> 0.02 m/s) | **0 / 6900** | **1498 / 7029 (21.31 %)** |
| median speed while moving (module definition) | 0.0934 m/s | 0.0842 m/s |
| process / shutdown | 3/3 completed; 1 shutdown segfault after results were written | 3/3 valid runs exit 0, 0 segfaults; 1 run VOID (session end, environment) |

- Both minimum clearances are above the 0.2051 m circumscribed radius.
- r04's tour time is short because its last three legs aborted in about
  1.5 s each, not because B drove faster.
- The speed statistic is `c2nav41_topology.speeds`. It is internally
  comparable between these arms only, **not** with DESIGN_DECISIONS'
  0.155 / 0.208 m/s pair.

### Per leg

| leg | arm | succeeded | median s | goal err m | abs yaw err rad | true clear m | PolygonStop s (legs) | DWB zero-vx | cmd<0.05 |
|---|---|---|---|---|---|---|---|---|---|
| `open_space` | A | 3/3 | 15.17 | 0.093 | 0.352 | 0.5169 | 0.00 (0) | 0.300 | 0.390 |
| `open_space` | B | 3/3 | 14.94 | 0.105 | 0.389 | 0.4982 | 0.00 (0) | 0.244 | 0.471 |
| `wall_adjacent` | A | 3/3 | 25.99 | 0.119 | 0.414 | 0.3946 | 0.00 (0) | 0.476 | 0.841 |
| `wall_adjacent` | B | **2/3** | 30.46 | 0.145 | 0.408 | 0.3651 | 0.00 (0) | 0.667 | 0.842 |
| `wall_parallel` | A | 3/3 | 19.58 | 0.087 | 0.240 | 0.4509 | 0.00 (0) | 0.190 | 0.460 |
| `wall_parallel` | B | 3/3 | 16.68 | 0.091 | 0.191 | 0.4275 | 0.00 (0) | 0.235 | 0.381 |
| `obstacle_corner` | A | 3/3 | 18.56 | 0.090 | 0.458 | 0.4771 | 0.00 (0) | 0.246 | 0.347 |
| `obstacle_corner` | B | **2/3** | 20.96 | 0.137 | 0.413 | 0.3706 | 0.00 (0) | 0.281 | 0.355 |
| `corridor_gate` | A | 3/3 | 21.31 | 0.086 | 0.411 | 0.4112 | 0.00 (0) | 0.368 | 0.577 |
| `corridor_gate` | B | **1/3** | 30.10 | 0.150 | 1.769 | 0.4055 | 1.32 (1) | 0.118 | 0.598 |
| `enclosure_entry` | A | 2/3 | 55.12 | 0.086 | 0.511 | 0.2832 | 0.00 (0) | 0.335 | 0.560 |
| `enclosure_entry` | B | **0/3** | 74.84 | 0.553 | 1.672 | 0.2567 | 0.00 (0) | 0.077 | 0.715 |
| `enclosure_exit` | A | 0/3 | 72.60 | 3.134 | 1.750 | 0.2423 | 176.38 (3) | 0.033 | 0.398 |
| `enclosure_exit` | B | 0/3 | 41.12 | 2.743 | 1.688 | 0.2266 | 3.88 (1) | 0.344 | 0.706 |

## 6. Command-path safety: the collision monitor does not own the wheels in topology B

`/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py` contains 6
`('cmd_vel', 'cmd_vel_nav')` remaps (counted this session).

- **Who uses the topic** is not re-derived here, node by node. It is taken
  from `PROJECT_STATE.md` KNOWN LIMITATIONS 0 and C2-M5.0's
  `c2m5_topology.txt`: `controller_server` and `behavior_server` publish on
  it, and it is the velocity smoother's input.
- **The loop.** `nav.launch.py arbiter:=true` points `cmd_vel_relay`'s
  output at that **same** topic, and `cmd_vel_arbiter` reads it.

**Wheels against the monitor's output.** A row counts when |v_wheel| exceeds
|v_cmdvel| by more than 0.02 m/s. That tolerance sits just above C2-NAV.0's
measured jitter floor of 0.016 m/s.

| arm | samples | exceeded | frac | worst gap | monitor ≈ 0 while wheels driven | worst wheel then |
|---|---|---|---|---|---|---|
| A | 6900 | **0** | 0.00 % | — | 417 | 0.0189 m/s |
| B | 7029 | **1498** | **21.31 %** | **0.300 m/s** | 1354 | **0.300 m/s** |

At tolerance 0, the counts are A 7 and B 2144.

**What the wheels were following.** Rows counted: monitor ≈ 0 and wheels
above 0.2 m/s.

| arm | rows | wheel = raw controller (`v_nav`) | wheel = smoother (`v_smoothed`) | median raw | median smoother | collision-monitor action |
|---|---|---|---|---|---|---|
| A | **0** | — | — | — | — | — |
| B | **643** | **349** | **0** | 0.2842 m/s | 0.0003 m/s | SLOWDOWN 492, none 121, LIMIT 21, blank 9, **STOP 0** |

- **In topology B the arbiter forwards the controller's raw command.** The
  wheels matched the velocity smoother's output in 0 of 643 rows.
- **Two stages are bypassed:** the smoother's acceleration limits, and the
  monitor's SLOWDOWN.
- **Unattributed rows:** 294 of the 643 rows matched neither stage within
  0.02 m/s.

**While PolygonStop was active** (the monitor's STOP state):

| arm | STOP rows | wheels > 0.01 m/s | worst wheel during STOP |
|---|---|---|---|
| A | 1763 | 20 | 0.0853 m/s |
| B | 52 | 23 | 0.0947 m/s |

- **STOP itself was not driven through in either arm:** the worst wheel
  command during STOP is below 0.1 m/s in both.
- **B's absence of deadlocks is therefore not the robot ignoring STOP.** B
  was in STOP for 5.2 s against A's 176.38 s, because it rarely reached the
  trap pose (see §10).

## 7. What each topology-B failure did (`c2nav39_tour_report.py legs`, `FAILURE_CONTEXT`)

### r01 — 5/7

`FAILURE_CONTEXT` fields: `t_transit` / `t_terminal` (s), `PolygonSlow` (s),
`gated` = fraction of time the monitor was gating the command, `DWB zero-vx`
= DWB's zero-forward-speed fraction.

| leg | status | s | goal err m | yaw err rad | true clear m | t_transit / t_terminal | PolygonSlow s | gated | DWB zero-vx |
|---|---|---|---|---|---|---|---|---|---|
| `enclosure_entry` | TIMEOUT | 74.84 | 0.117 | −1.594 | 0.2644 | 29.64 / **45.20** | 57.20 | 0.909 | 0.474 |
| `enclosure_exit` | TIMEOUT | 74.98 | 0.184 | −0.721 | 0.2763 | **72.15** / 2.83 | 61.55 | 0.824 | 0.000 |

The other five legs SUCCEEDED, with 0 STOP activations in the whole tour.

### r02 — 4/7

| leg | status | s | goal err m | yaw err rad | true clear m | t_transit / t_terminal | PolygonSlow / Stop s | gated | DWB zero-vx |
|---|---|---|---|---|---|---|---|---|---|
| `corridor_gate` | TIMEOUT | 75.40 | 0.150 | −1.769 | 0.5574 | **68.77** / 6.62 | 39.21 / 1.32 | 0.605 | 0.118 |
| `enclosure_entry` | TIMEOUT | 75.03 | 0.553 | 1.672 | 0.2567 | never arrived | 39.32 / 0 | 0.726 | 0.077 |
| `enclosure_exit` | ABORTED | 41.12 | 2.794 | −1.688 | 0.2266 | never arrived | 37.19 / 3.88 | **1.000** | **0.688** |

The exit leg started **outside** the pocket, at (−3.644, 2.316), 0.2557 m
from `wall_west`, because the entry never arrived. It drove 0.272 m, its
DWB illegal-trajectory fraction was 0.581, and a 3.9 s STOP hold lasted to
the end of the leg.

### r04 — 2/7

| leg | status | s | goal err m | yaw err rad | true clear m | t_transit / t_terminal | PolygonSlow s | gated | DWB zero-vx |
|---|---|---|---|---|---|---|---|---|---|
| `wall_adjacent` | TIMEOUT | 70.49 | 0.079 | 1.013 | 0.3651 | 4.44 / **66.05** | 54.80 | 0.837 | 0.580 |
| `obstacle_corner` | ABORTED | 51.40 | 1.414 | 2.070 | 0.3706 | never arrived | 44.64 | 0.927 | 0.693 |
| `corridor_gate` | ABORTED | **1.56** | 3.183 | 2.070 | 0.4055 | path 0.0 m | — | — | — |
| `enclosure_entry` | ABORTED | **1.45** | 5.884 | 2.070 | 0.4055 | path 0.0 m | — | — | — |
| `enclosure_exit` | ABORTED | **1.09** | 2.743 | 2.070 | 0.4055 | path 0.0 m | — | — | — |

**The last three legs are one cascade, read from `nav.log`.**

- **The refusal.** For every goal after `obstacle_corner`, `planner_server`
  logged `GridBased plugin failed to plan from (2.14, -1.71) … "Start occupied"`,
  and `bt_navigator` aborted.
- **Where the robot was.** nav_bench's `min_clearance_m` (the ground-truth
  driven path against occupied map cells) was **0.128 m**, inside the 0.20 m
  robot radius. `obstacle_corner` left the robot at a pose the global
  costmap treats as occupied.
- **Not an infrastructure failure.** The `CRITICAL FAILURE: SERVER map_server IS DOWN`
  in the same log came **0.2 s after** `signal_handler(SIGINT/SIGTERM)`,
  in both `nav.log` and `sim.log`. That is the runner tearing down a
  completed run.

## 8. Failure classification (Step 5 — from observed evidence only)

Classes:

- **A** planner / path-selection
- **B** controller behaviour
- **C** collision-monitor / PolygonStop interaction
- **D** terminal pose / heading convergence
- **E** infrastructure / lifecycle

| arm | run | leg | status | class | evidence |
|---|---|---|---|---|---|
| A | r01 | `enclosure_entry` | TIMEOUT | **D** | C2-NAV.39: its entry misses are terminal-yaw timeouts |
| A | r01, r02, r03 | `enclosure_exit` ×3 | TIMEOUT | **C** | PolygonStop deadlock each tour (62.16 / 65.48 / 48.74 s); min true clearance 0.2423–0.2463 m, inside the 0.25 m STOP circle |
| B | r01 | `enclosure_entry` | TIMEOUT | **D** | arrived to 0.117 m in 29.64 s, then 45.20 s terminal, yaw err −1.594 rad |
| B | r01 | `enclosure_exit` | TIMEOUT | **C** | 72.15 s transit, PolygonSlow 61.55 s, gated 0.824, DWB zero-vx **0.000** (DWB wanted to move), 0 STOP |
| B | r02 | `corridor_gate` | TIMEOUT | **C** | 68.77 s transit, PolygonSlow 39.21 s, gated 0.605, 24 stops; arrived to 0.150 m |
| B | r02 | `enclosure_entry` | TIMEOUT | **C** | never arrived (0.553 m), PolygonSlow 39.32 s, gated 0.726, 28 stops, DWB zero-vx 0.077 |
| B | r02 | `enclosure_exit` | ABORTED | **B** (C contributing) | started 0.2557 m from `wall_west` after the failed entry; DWB zero-vx 0.688, illegal 0.581, gated 1.000 |
| B | r04 | `wall_adjacent` | TIMEOUT | **D** (C contributing) | arrived to 0.079 m in 4.44 s, then 66.05 s terminal, yaw err 1.013 rad, gated 0.837 |
| B | r04 | `obstacle_corner` | ABORTED | **B** (C contributing) | DWB zero-vx 0.693, PolygonSlow 44.64 s, gated 0.927; ended at 0.128 m map clearance |
| B | r04 | `corridor_gate`, `enclosure_entry`, `enclosure_exit` | ABORTED | **A** (cascade of B) | planner "Start occupied" from the pose `obstacle_corner` left, 1.09–1.56 s, 0.0 m driven |

| class | A (4 failures) | B (10 failures) |
|---|---|---|
| A planner | 0 | 3 (one cascade) |
| B controller | 0 | 2 |
| C collision monitor | 3 | 3 |
| D terminal heading | 1 | 2 |
| E infrastructure | 0 | 0 |

**No failure in either valid arm is class E.** The one run lost to
infrastructure (r03) was lost to the harness session ending, not to the
stack, and it is VOID rather than counted.

## 9. Safety verification (Step 4)

| check | result | evidence |
|---|---|---|
| validated nav2 defaults installed | **OK** | `nav2_params.yaml` byte-identical to `docs/data/c2nav11_ntp_params.yaml`, sha256 `6f61e499…`; installed copy matches source |
| `BaseObstacle.scale` = 8.0 | **OK** | file, and live on every run |
| local costmap CSF = 65 | **OK** | file 65.0; live 12/12 on every run |
| validated NavigateThroughPoses BT | **OK** | `navigate_through_poses_w_replanning_and_recovery.xml`, file and live |
| PolygonStop not weakened | **OK** | radius 0.25, min_points 4 (file and live); PolygonSlow 0.3 |
| diagnostics disabled by default | **OK** | `amcl_diag.enabled: false` in `baseline.yaml` and `baseline_topology_b.yaml`; only `baseline_amcl_diag.yaml` enables it, by name |
| capture validation strict | **OK** | `c2nav36_diag.py selftest` 51/0 |
| runner refuses a competing simulator | **OK, live** | exit 3 while r01 ran; the running tour was untouched |
| cleanup does not kill unrelated processes | **OK** | every `ros_clean.sh` pattern bracketed; `0 matched, 0 still running` after r01, r02, r04; no orphans afterwards |
| no experiment silently modifies safety config | **OK** | `collision_monitor` overrides refused unless `allow_safety_change: true`; both baseline arms resolve with 0 changes |
| wheel ownership as requested | **OK (new)** | `verify-topology` on every B run: 1 publisher, `cmd_vel_arbiter`, `mode=nav` |

**None of those gates was weakened, and none of them is the problem.** In
topology B the collision monitor is configured correctly, loaded correctly,
and publishing correctly. **Its output simply does not decide what the wheels
receive.**

## 10. Decision: topology B **REJECTED** as the integration candidate

**Topology A remains the runner default** (`topology` defaults to A).

**Why:**

- **Worse on every success measure.** 11/21 against 17/21; ordinary legs
  11/15 against 15/15; entry 0/3 against 2/3; 5 ABORTED against 0.
- **A safety defect A does not have.** The collision monitor's output
  decided the wheel command in topology A (0 of 6900 exceedances) and did
  not in topology B (1498 of 7029). The wheels followed the raw controller
  at up to 0.300 m/s while the monitor commanded 0.
- **Not decided on the total alone.** Four ordinary legs that pass 15/15 in
  A failed in B, and none of them came from a parameter difference: both
  arms ran sha256 `6f61e499…` with 0 changes.
  - Three were monitor-gated transit, controller or terminal-heading
    failures (§7).
  - The fourth (r04 `corridor_gate`) was the planner cascade from one of
    those three.

**What topology B changed at the enclosure.** A's exit deadlock (3/3, 176.38
s of PolygonStop) did not occur in B, but **B did not solve it**:

- B's entry succeeded 0/3, so the robot mostly never reached the pose that
  traps.
- r01 is the only B exit that started at the pocket. Its entry arrived to
  0.117 m but never finished the terminal rotation (yaw err −1.594 rad). It
  ended at true clearance 0.2644 m, outside the 0.25 m STOP circle, and its
  exit was not held.
- A's trapped tours reached 0.2423–0.2463 m, inside the circle.
- Observed once. Not established.

**What rejection means here.** Topology B is not an option that can be
declined: `mission.launch.py` runs it. So this decision says:

- **The accepted configuration's 17/21 does not transfer to the command path
  the robot ships in.** It must not be described as validated for
  `mission.launch.py`.
- **That path's collision monitor does not control the wheels,** re-measured
  here on the shipped configuration.

**Step 2 was not taken.** No candidate launch mode was added and nothing was
integrated.

**Step 3 cleanup: nothing removed, deliberately.** The topology-B runner
mode, `baseline_topology_b.yaml`, `verify-topology` and
`c2nav41_topology.py` are not topology-B-only experiment scaffolding:

- they are the only instrument that measures the command path the robot
  ships in;
- they are the regression test §12 needs.

No topology was invented.

## 11. Remaining concrete failures

1. **Topology B, safety (dominant):** the `/cmd_vel_nav` ownership loop. The
   arbiter forwards the raw controller command, bypassing the velocity
   smoother and the collision monitor's SLOWDOWN (§6).
2. **Topology A:** the `enclosure_exit` PolygonStop deadlock (class C, 3/3),
   the C2-NAV.5/.6 geometry, unchanged.
3. **Both arms:** terminal-heading convergence (class D: A 1, B 2).

## 12. Exactly one next implementation action

**Remove the `/cmd_vel_nav` ownership loop.**

**What to change.** No nav2 parameter and no safety gate changes.

- In arbiter mode, give `cmd_vel_relay` its own output topic. A suggested
  name is `/cmd_vel_gated`; the name must not contain `nav2_`
  (`ros_clean.sh`).
- Point `cmd_vel_arbiter`'s `nav_topic` at that topic, everywhere
  `arbiter:=true` is used:
  - `gazebo_models/launch/nav.launch.py` (the `relay_output` expression);
  - `custom_teleop/launch/arbiter.launch.py`;
  - `coco_mission/launch/mission.launch.py`;
  - `coco_web/launch/web.launch.py`;
  - the `nav_topic` default in `cmd_vel_arbiter.py`.
- Re-stamp in `cmd_vel_relay`. It republishes the original `header.stamp`,
  which DESIGN_DECISIONS records as the cause of topology A's 233 stale-dropped
  commands.

**Why this one.**

- It is the dominant measured difference between the arms.
- It is a safety defect.
- Its fix is already written down (`PROJECT_STATE.md` KNOWN LIMITATIONS 0).
- Tuning terminal heading or the enclosure on a command path that bypasses
  the smoother and the monitor would tune against the wrong plant.

**Tests to add.** No test pins this wiring today: 0 files under `*/test`
reference `cmd_vel_nav`. Add a launch-description test that, in arbiter
mode:

- the relay's output is **not** `/cmd_vel_nav`;
- the relay's output **equals** the arbiter's `nav_topic`.

**Validate.** Run `baseline_topology_b.yaml` × 3 on fresh simulators, then
`c2nav41_topology.py compare` against this session's arms.

- **Pass (safety):** 0 `bypass_source` rows, and wheels-exceeded-monitor at
  topology A's control level.
- **Then compare legs** against A's 17/21 and B's 11/21.

**Known knock-on: C2-M5.1's relocalization spin.** It currently reaches the
wheels through the arbiter reading `/cmd_vel_nav` directly. See
`mission_states.py` 412–415 and `mission_executive.py` 283–287 and 658–660.

- After the fix it passes through the smoother, monitor and relay like every
  other command. That is the intended behaviour.
- Those comments become false.
- C2-M5.1 recovery needs a live re-check.

**Comparability.** M6's standing 19/20 was measured with the loop in place.
A comparability statement is owed.

**Gate.** CLAUDE.md rule 4 lists `cmd_vel_arbiter` as do-not-touch without
an explicit request, and `PROJECT_STATE.md` records the wheel path as
frozen. **This sprint did not implement it.** It needs the owner's explicit
go-ahead.

## 13. Limitations

- **n = 3 per arm.** Legs are not independent, and r04's last three legs are
  a single cascade.
- **Arm A's topology is asserted from runner source,** not read back live.
  Those runs are labelled legacy.
- **The arms were not interleaved.** A ran 2026-09-15 and B 2026-09-17, so
  machine state is a possible confound.
- **Map and world disagree at r04's `obstacle_corner` end pose.** nav_bench's
  map-cell clearance is 0.128 m; the world-file geometry gives 0.3706 m.
  Not resolved here.
- **Some bypass rows are unattributed.** 294 of the 643 matched neither the
  raw controller nor the smoother within 0.02 m/s.
- **Mechanism not isolated.** Which part of the loop (the raw command, the
  interleaving, or the feedback into the smoother) turned B's gated driving
  into ordinary-leg failures is not separated.
- **`mission.launch.py` itself was not toured.** Topology B here is
  `nav.launch.py arbiter:=true` plus `arbiter.launch.py initial_mode:=nav`,
  without the mission nodes. `mission.launch.py` bring-up was checked in
  C2-NAV.40 §8.
