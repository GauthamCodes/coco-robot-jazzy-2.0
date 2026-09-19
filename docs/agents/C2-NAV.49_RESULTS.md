# C2-NAV.49 — the colour matrix, re-measured on `robot_radius` 0.25

**Date:** 2026-09-19 · **Branch:** `c2nav49-integration` · **Base:** `main` @ `1425e6c`

Integrate C2-NAV.48's local-costmap fix and replace C2-NAV.46's 11-of-12 with
a figure measured on the corrected configuration. No investigation was opened.

---

## 1. Result

**12 of 12 fetches. 12 valid runs, 0 void. Every colour 3/3.**

| colour | run 1 | run 2 | run 3 | result |
|---|---|---|---|---|
| red | FETCH | FETCH | FETCH | **3/3** |
| green | FETCH | FETCH | FETCH | **3/3** |
| blue | FETCH | FETCH | FETCH | **3/3** |
| yellow | FETCH | FETCH | FETCH | **3/3** |

Against C2-NAV.46 (`robot_radius` 0.20): 11/12, with blue 2/3. Blue is 3/3 here.

Every run: fresh simulator, headless, never `--fast`, depth fusion off,
`dirty_paths=0`, 22 runner checks passed / 0 failed, clean shutdown, 16
nominal mission states, 0 state re-entries, Nav2 goals **2 succeeded / 0
failed / 0 aborted**, **0** recoveries, **0** relocalizations.

**No runtime code was changed to produce this.** The only production change in
the sprint is the one C2-NAV.48 already made and this branch integrates.

---

## 2. Per-run

`pre` and `home` are ground-truth position error at the arrival gates, `near`
is the closest the base came to `cylinder_obstacle`'s **surface**.

| run | result | pre (m) | gate | home (m) | sim (s) | wall (s) | lift (mm) | base-x (m) | near (m) |
|---|---|---|---|---|---|---|---|---|---|
| r1_red | COMPLETE | 0.113 | clean | 0.133 | 152.5 | 198 | 36.4 | 0.1541 | 0.6218 |
| r1_green | COMPLETE | 0.298 | **accepted** | 0.114 | 151.1 | 176 | 36.6 | 0.1537 | 0.5343 |
| r1_blue | COMPLETE | 0.114 | clean | 0.052 | 165.7 | 183 | 34.1 | 0.1543 | **0.2894** |
| r1_yellow | COMPLETE | 0.040 | clean | 0.074 | 150.1 | 163 | 34.8 | 0.1535 | 0.4542 |
| r2_red | COMPLETE | 0.088 | clean | 0.072 | 151.6 | 169 | 35.4 | 0.1545 | 0.6261 |
| r2_green | COMPLETE | 0.307 | **accepted** | 0.044 | 154.5 | 168 | 35.5 | 0.1545 | 0.4789 |
| r2_blue | COMPLETE | 0.087 | clean | 0.084 | 160.8 | 178 | 34.6 | 0.1547 | 0.4024 |
| r2_yellow | COMPLETE | 0.058 | clean | 0.087 | 149.6 | 252 | 35.7 | 0.1544 | 0.4628 |
| r3_red | COMPLETE | 0.131 | clean | 0.099 | 145.6 | 241 | 36.0 | 0.1545 | 0.6214 |
| r3_green | COMPLETE | 0.330 | **accepted** | 0.065 | 156.1 | 262 | 34.9 | 0.1535 | 0.4054 |
| r3_blue | COMPLETE | 0.080 | clean | 0.088 | 192.4 | 214 | 35.2 | 0.1546 | 0.3335 |
| r3_yellow | COMPLETE | 0.040 | clean | 0.114 | 155.4 | 175 | 35.4 | 0.1536 | 0.4532 |

Ranges: pre-ramp **0.040–0.330 m**, home **0.044–0.133 m**, mission
**145.6–192.4 s** sim, lift **34.1–36.6 mm**, grasp base-x
**0.1535–0.1547 m** — 12 of 12 inside the `[0.1510, 0.1565]` window.

**Two HEADs, and the difference is not runtime.** `r1_red` ran at `3a22201`,
the other eleven at `553f219`. The only change between them is
`docs/data/c2nav49_clearance.py`, an offline reader. No parameter, launch
file or node differs.

---

## 3. The blue regression (Phase 8) — it did not recur

C2-NAV.46's `r2_blue` held PolygonStop **595.5 s** on the return leg at
**0.2486 m** from `cylinder_obstacle`, 1.4 mm inside the 0.25 m stop circle.

Measured here over all twelve traces, by `docs/data/c2nav49_clearance.py`:

| quantity | value |
|---|---|
| traces read | **12 of 12** |
| PolygonStop rows (`cm_action == '1'`) | **0**, every run |
| PolygonStop episodes | **0** |
| Stop duration | **0.0 s** |
| Rows with a stop active and the wheels driven | **0** |
| `controller_failed_progress` | **0** (C2-NAV.46 `r2_blue`: 40) |
| Costmap clears | **0** |
| Closest approach to `cylinder_obstacle`, any run | **0.2894 m** (`r1_blue`) |
| — margin outside the stop circle | **+39.4 mm** |

The twelve traces are what makes the zeros mean something: a "0 PolygonStop
rows" from 0 traces reads identically and is worth nothing, so the reader
says so in those words when it reads nothing.

**Blue is the exposed lane, and that is geometric, not random.** Only blue
goes near the obstacle at all — blue **0.2894 / 0.4024 / 0.3335 m**, green
0.4054–0.5343, yellow 0.4532–0.4628, red never closer than **0.6214 m**. For
blue all three closest approaches are **on the return leg**, exactly where
C2-NAV.46 failed. This is consistent with the defect having been blue's
route, not a lane-independent coin flip.

### What this does NOT establish

**The corrective mechanism was never exercised.** C2-NAV.48's fix works by
making the 0.2059–0.25 m band inscribed-lethal so the planner refuses poses
in it. No run entered that band: the closest approach, 0.2894 m, is outside
even the **old** 0.205879 m inscribed radius. So none of these twelve runs
would have deadlocked on `robot_radius` 0.20 either, and this matrix is
**not** an A/B of the fix. It shows **no recurrence and no regression on the
corrected value**; it does not show the fix catching anything.

The case for KEEP therefore still rests where C2-NAV.48 put it — a root cause
measured from C2-NAV.46's own trace and a mechanism verified in Nav2's source
— with twelve clean missions added on the new value. C2-NAV.46's base rate
was 1 in 12. **Twelve runs is not a rate, and 12/12 is not a 100 % figure.**

**`min_scan_m` cannot be used for this.** It reads 0.1500–0.1504 m in all
twelve — the LiDAR's minimum-range floor, saturated in every run. It does not
discriminate clearance. The ground-truth geometry above is the measure.

---

## 4. Safety (Phase 9)

Metrics as defined in `c2nav41_topology.py`, not redefined.

| metric | result |
|---|---|
| Raw Nav2 → wheel bypass | **0**, all 12 |
| PolygonStop active with wheels moving | **0**, all 12 |
| Stale command drops | **0**, all 12 |
| Wheels above the monitor | **8 of 7,781** nav-active samples = **0.1028 %**, worst gap **0.1250 m/s** |

**The residual is not claimed fixed, and it is not claimed improved.** At
0.1028 % it is *higher* than C2-NAV.46's 4 of 9,744 = 0.0411 %, and sits
inside the 0.088–2.92 % short-streak band that has been unattributed since
C2-NAV.42. Nothing in this sprint addressed it.

`process_died` lines appear 2–7 times per run and are **teardown noise**: they
follow the terminal state, the runner's "no launched process died during the
mission" check passed in all twelve, and every shutdown was clean. C2-NAV.44
documented the same signature.

---

## 5. The arrival gate — green's discrepancy reproduces, and stays green's

Second independent sweep, same conclusion as C2-NAV.46.

| colour | pre-ramp GT error (m) | verdict |
|---|---|---|
| green | **0.298 / 0.307 / 0.330** | discrepancy accepted, inside the consistency band |
| red | 0.088 / 0.113 / 0.131 | clean |
| blue | 0.080 / 0.087 / 0.114 | clean |
| yellow | 0.040 / 0.040 / 0.058 | clean |

C2-NAV.46 measured green at 0.315–0.348 m; here 0.298–0.330 m. The outer
0.50 m band was **never reached** and futile retries were **0 in all 12**.
Red, blue and yellow would all have passed the old single-threshold gate;
only green would not. **Do not generalise green's number to other lanes.**

---

## 6. Configuration, read back from the live nodes (Phase 5)

Every run wrote `params_live.txt`; all rows `OK`, file vs live.

| parameter | file | live |
|---|---|---|
| `local_costmap.robot_radius` | 0.25 | **0.25** |
| `global_costmap.robot_radius` | 0.2 | **0.2** |
| `local_costmap` `inflation_layer.cost_scaling_factor` | 65.0 | 65.0 |
| `global_costmap` `inflation_layer.cost_scaling_factor` | 5.0 | 5.0 |
| `FollowPath.BaseObstacle.scale` | 8.0 | 8.0 |
| `PolygonStop.radius` / `.min_points` | 0.25 / 4 | 0.25 / 4 |
| `goal_checker.xy_goal_tolerance` / `.yaw` | 0.25 / 0.25 | 0.25 / 0.25 |
| `FollowPath.xy_goal_tolerance` | 0.05 | 0.05 |
| `bt_navigator` through-poses tree | `navigate_through_poses_w_replanning_and_recovery.xml` | same |
| `amcl.resample_interval` | 1 | 1 |

`nav2_params.yaml` sha256 `06c308aff78d4e7327212bbca280f62be7a7baef604ae41676e886b72301db8c`
in all twelve. Depth fusion **off**: no depth-cloud process, all four
observation sources `scan`. Exactly **one** arbiter. Command chain verified
link by link (topology B) in every run.

---

## 7. Build, tests, environment

- **9/9 packages built**, colcon exit 0. Only stderr is the pre-existing
  setuptools `pytest-repeat` unbuilt-egg warning.
- **Tests 1004 passed, 0 failed, 0 skipped**, per package, cwd = package dir,
  clean ROS graph: `coco_config` 70, `custom_teleop` 75, `coco_rl` 164,
  `coco_perception` 139, `gazebo_models` 178 (`--ignore=test_integration`),
  `coco_moveit_config` 12, `coco_sim` 55, `coco_mission` 311. Was 1001 before
  this sprint's three new tests.
- `coco_web` has no `test/` directory. pytest exits **5** ("no tests ran")
  on this pytest; `CLAUDE.md` records 4. Version difference, not a failure.

**`<ws>/install` still cannot launch Gazebo, and this was re-measured, not
inherited.** It holds **2** unresolvable ament-index entries — `red_ball_nav`
and `turtlebot3_teleop` — against **0 of 462** for an isolated overlay.
C2-NAV.48 measured 1; it is 2 today. `ros_gz_sim`'s `GazeboRosPaths.get_paths()`
enumerates every package, so one bad entry kills every gz launch. Runs
therefore used `$HOME/c2nav49_overlay`, built from this worktree, selected
with `COCO_WS`. This is the user's environment and is reported, not changed.

---

## 8. `ros_clean.sh` no longer sweeps other people's simulators

The flaw was real and already paid for: C2-NAV.44 measured an unrelated
`eyantra_kepler_colony` simulator from `~/ros2_ws` being killed mid-run by the
bare `g[z] sim` pattern.

    'g[z] sim'  ->  'g[z] sim.*gazebo_models/worlds'

**Scoping the sweep to the current experiment was rejected**, not overlooked.
`ros_clean.sh` exists to kill orphans of *previous* runs, which are never in
the current process group; a session-scoped sweep could not kill one of them,
and its header records what surviving orphans cost. The world path separates
ours from theirs without giving that up: `full_world_robo.launch.py` always
passes a world out of `gazebo_models/worlds/`, in both `gui:=true` and
`gui:=false` form, so every coco orphan from any overlay or past run still
matches. Verified live against this sprint's simulator.

Three tests, asserting **both** directions — a coco decoy *is* matched
(positive control), a foreign decoy is *not*, and no pattern may be a bare
`gz sim` again. Against the pre-fix script in an isolated copy: **2 failed,
1 passed**, the pass being the positive control. Against the fixed script:
**3 passed**.

**Not fixed, not claimed:** a `gz sim -g` client started by hand carries no
world path and is not swept. No launch file here starts one.

---

## 9. Stale artifacts removed

`<repo>/install` (148K, 2 packages, newest file **2026-07-27 21:35**) and
`<repo>/build` (56K, 5 package dirs, newest **2026-07-28 16:35**), both in the
main checkout, both `COLCON_IGNORE`d so colcon never refreshed them, both
holding a seven-week-old `coco_rl`. They existed only to be sourced by
mistake — which C2-NAV.48's guard bug nearly did. Authoritative artifacts are
`<ws>/install` and `<ws>/build`, dated 2026-09-19.

**`<ws>/install` was NOT touched** — it carries the unrelated turtlebot3 and
`red_ball_nav` prefixes; 19 prefixes before and after.

Still present and **not** removed: this worktree's own `install/` and
`build/` (2026-09-19 02:55). They are gitignored build output, they are not
what C2-NAV.48 flagged, and nothing now sources them — the runners take their
overlay from `COCO_WS`.

---

## 10. Decision

**KEEP `local_costmap.robot_radius = 0.25`.** The corrected M6 configuration
is regression-validated: 12 of 12 fetches, 0 void, bypass 0, stale drops 0,
PolygonStop 0, recoveries 0, and the C2-NAV.46 deadlock did not recur.
PolygonStop is unchanged at 0.25 / 4. The global costmap stays at 0.20, and
that asymmetry is deliberate and measured.

Carried forward, unresolved and unclaimed:

1. The fix's mechanism was **not exercised** — no run entered the band.
2. `gated_zero_moving` / wheels-above-the-monitor remains **unattributed**,
   and is 0.1028 % here against C2-NAV.46's 0.0411 %.
3. Green's pre-ramp discrepancy **reproduces** (0.298–0.330 m) and is still
   green's alone. Reported, not retried.
4. `<ws>/install` cannot launch a simulator until the turtlebot3 and
   `red_ball_nav` prefixes are repaired or removed. User's environment.
5. Severe confident AMCL divergence is still detected but not reliably
   recovered (C2-M5 limitation, untouched).

## Next engineering action

The COCO core is stable across all four lanes on the corrected configuration.
The next step is **productization**, not another C2-NAV investigation:

```bash
git checkout main && git merge --ff-only c2nav49-integration
```

Merging is the owner's call, not the agent's.
