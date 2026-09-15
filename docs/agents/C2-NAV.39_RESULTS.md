# C2-NAV.39 — implementation mode: accepted config, capture integrity, reproducible tours

**Agent:** implementation, integration, test and simulation validation. The
C2-NAV investigation loop is paused by the human engineer (2026-09-16). No
new mechanism was investigated. No parameter was tuned. No safety gate moved.

## 1. What the branch actually looked like before this session

A read-only survey at `efde9a0` found the branch was not in the state the
C2-NAV.37/.38 write-ups implied:

| Item | Expected (accepted) | Found in `gazebo_models/config/nav2_params.yaml` |
|---|---|---|
| local costmap `cost_scaling_factor` | 65 (C2-NAV.4/.5) | **5.0** |
| `default_nav_through_poses_bt_xml` | `navigate_through_poses_w_replanning_and_recovery.xml` (C2-NAV.11) | **the single-pose tree** |
| `FollowPath.BaseObstacle.scale` | 8.0 | **2.0 — the C2-NAV.2 value, measured and REJECTED, never reverted** |
| PolygonStop radius / min_points, PolygonSlow ratio | 0.25 / 4, 0.3 | 0.25 / 4, 0.3 (correct) |

The validated values existed only in `docs/data/c2nav11_ntp_params.yaml`,
loaded explicitly by the experiment runners. Every plain `nav.launch.py` or
`mission.launch.py` start used the rejected default. So did C2-NAV.37's
83/105 legs (see §6).

Also found:

- The C2-NAV.37 component-registration fix and the C2-NAV.37/.38 documents
  were uncommitted.
- `amcl_component_load.py`, which performed every C2-NAV.37 live swap, existed
  only in another job's transient tmp directory.

Environment breakage after the workspace was renamed `ros2_ws` →
`ros2_ws(personal)`:

- The worktree's git pointers still named the old path, so plain `git` failed.
- Every CMake cache recorded the old source directory, so the build failed.
- `.navbench/env.sh` sourced a path that no longer exists.
- `ros_gz_sim` runs `gz sim` through a shell, so the unquoted `(` in the
  world path stopped the simulator from starting.

## 2. What changed (commits on `worktree-c2nav0-diagnosis`)

| Commit | Change | Why (established evidence) |
|---|---|---|
| `a765369` | Commit the C2-NAV.37 `rclcpp_components_register_nodes` fix + C2-NAV.37/.38 docs | The component-container swap C2-NAV.37 validated live across 5 runs |
| `ad2b8b8` | `nav2_params.yaml` := `docs/data/c2nav11_ntp_params.yaml`, byte-identical (sha256 `6f61e499…`): local CSF 65, multi-pose BT, BaseObstacle 8.0. New `test_nav2_params_guard.py` | CSF 65 accepted (C2-NAV.4/.5); BT pointer fix validated (C2-NAV.11); BaseObstacle 2.0 rejected (C2-NAV.2). The guard pins the accepted values, the collision-monitor gates, the goal tolerances and `resample_interval: 1`, so a rejected value cannot silently ship again |
| `57f75d8` | `shlex.quote` the world path in `full_world_robo.launch.py` | Concrete blocker: the simulator could not start from the renamed workspace |
| `ee0f2e2` | Capture integrity (details in §3) | The differential-review checklist in the C2-NAV.39 brief |
| `6466a35` | `docs/data/c2nav39_tour_report.py` | Compares against the established baselines with the series' own metrics |
| `0b91330` | Reproducible tour runner, experiment files, parameter overlay with live readback, additive `nav_bench` fields | Phase B infrastructure (§4) |
| `f144852` | Anchor the `amcl_diag` patterns in `ros_clean.sh`; the runner refuses to start if the sweep would already match anything | Found in simulation: the runner killed itself in its own teardown (§6b) |
| `fbb4dc5` | `amcl_diag_swap.py` brings `/amcl` to active from its CURRENT lifecycle state instead of an unconditional configure/activate | Found in simulation: nav2's lifecycle manager activated the reloaded node first (§6b) |

No change to: PolygonStop/PolygonSlow/PolygonLimit/FootprintApproach, any
goal tolerance, any AMCL parameter, the map, the TOUR goals, or DWB critic
weights beyond restoring the accepted 8.0.

## 3. Capture integrity (commit `ee0f2e2`)

**Recorder, at the source (`coco_nav_diag`).**

- `DiagRecorder` writes a `diag_session_start` header at configure and a
  `diag_session_end` footer at `stop()`. The footer carries `recorded`,
  `dropped`, `written`, `write_failures` and `last_update_index`. Both
  bypass the bounded queue, and the schema is now version 2.
- Drops used to exist only as a WARN on stderr at shutdown. They are now in
  the file.
- The capture stays default-disabled and bounded, and all file I/O still
  happens on the writer thread.

**GT sidecar.** On SIGINT or SIGTERM it writes `<gt.csv>.meta.json`: rows
written and dropped, first/last stamp, stamp regressions, duplicate stamps,
the frame pairs seen, and `clean_shutdown`.

**Offline validation (`docs/data/c2nav36_diag.py`), before any number is
reported.**

Capture-level checks. A failure makes the status **INVALID**:

- the footer is present and last, its counts agree with the file, and it
  reports zero drops and zero write failures;
- the header appears exactly once, which catches concatenated captures;
- no duplicate `(type, update_index)`;
- `update_index` and scan stamp strictly increase in file order, and the
  node clock never goes backwards;
- one base frame and one odom frame throughout;
- GT stamps strictly increase in *file* order, checked before sorting;
- GT has one frame pair, and its body frame equals AMCL's base frame (the
  model-to-base correspondence check);
- the GT meta file shows no drops, a matching row count and a clean shutdown.

Row-level checks. A failure makes that update unobservable, with a named
reason:

- **Strict TF pairing:** a paired `odom_tf_lookup` exists and succeeded, with
  the same scan stamp, a returned stamp equal to the scan stamp, and the pose
  the model consumed.
- **Anchor consistency:** the anchor precedes the update in index and stamp,
  and its pose is the one recorded at the anchor update.
- **Delta:** `delta == pose − anchor`.
- **Decomposition:** `(rot1, trans, rot2)` is the differential split of the
  delta and recomposes to it.
- **GT bracket:** each GT bracket is no wider than `--max-gt-gap`, default
  0.1 s, about 5× the measured 48–50 Hz period.

Statuses are OK, UNOBSERVABLE, INVALID and LEGACY_UNVERIFIED. The last one
covers a schema-1 capture or GT without a meta file: every check that can run
did run, but drop accounting is unverifiable, so the capture is never
reported as a plain OK.

**Motion decomposition from the installed-model oracle.** The split moved
into `computeDifferentialDecomposition()` (identical arithmetic, same call
site). `test_motion_decomposition_oracle` runs the **installed**
`nav2_amcl::DifferentialMotionModel::odometryUpdate` with all alphas at
zero. It asserts each particle moves by exactly that split. Fixtures cover:
forward, sideways, reverse, below the 1 cm cutoff, pure rotation, and a
heading across ±π.

**Reproducible component swap.** `coco_nav_diag/scripts/amcl_diag_swap.py`
(installed) replaces the transient script. In order, it:

1. finds `/amcl` in `/nav2_container`, unloads only it, and verifies every
   other component is still loaded;
2. loads `coco_nav_diag::AmclNode` with the params file's flattened `amcl:`
   block plus the `diag_*` overrides;
3. runs configure/activate;
4. reads back `diag_*`, `resample_interval` and `robot_model_type` off the
   running node.

`unload` triggers the destructor, which writes the footer. `ros_clean.sh`
now also sweeps `amcl_dia[g]` and `c2nav36_gt_sideca[r]`.

**Regression on real data (measured this session).** Re-analysing all four
C2-NAV.37 captures with every new check:

| Run | Status | Candidates | Correlated | Unobservable | Along-track mean (m/update) |
|---|---|---|---|---|---|
| 1 | LEGACY_UNVERIFIED | 533 | 532 | 1 (GT does not bracket) | +0.000810 |
| 2 | LEGACY_UNVERIFIED | 599 | 598 | 1 | +0.000720 |
| 3 | LEGACY_UNVERIFIED | 582 | 581 | 1 | +0.000693 |
| 4 | LEGACY_UNVERIFIED | 563 | 562 | 1 | +0.000773 |

That is 2273 correlated updates, pooled +0.000747. C2-NAV.37's numbers are
reproduced exactly: **none** of the new pairing, anchor, delta,
decomposition, bracket, ordering, duplicate or frame checks rejected a single
real update. Those invariants now hold by measurement on 2,273 rows, not by
assumption.

## 4. Tuning and validation infrastructure (Phase B)

- `gazebo_models/config/experiments/{baseline,baseline_amcl_diag}.yaml` are
  explicit experiment files. Each carries only the nav_bench arguments, the
  diagnostic on/off switch, and a `nav2_overrides` block of *differences*.
- `gazebo_models/scripts/nav_params_overlay.py` has two modes:
  - `resolve` merges overrides into the shipped params. Unknown keys and
    type/structure changes are refused, and so is anything under
    `collision_monitor` without an explicit `allow_safety_change: true`. It
    writes `experiment_resolved.json`, plus `params_merged.yaml` only when
    something changes.
  - `verify-live` reads the accepted and safety parameters back off the
    running nodes and fails the run on any mismatch.
- `gazebo_models/scripts/nav_tour_run.sh <experiment.yaml>` does one fresh
  simulator and one bounded tour per call. In order, it:
  - refuses to start if any sim, container or nav_bench is already running;
  - checks the params file nav2 will load resolves into this worktree;
  - allocates a deterministic run directory `~/coco_nav_runs/<exp>/<exp>_rNN`
    (never overwritten) with `manifest.json` (git sha, dirty count, params
    sha256, exit codes, UTC times);
  - brings up in stages (sim → `/scan` → Nav2 → all lifecycle nodes active)
    and runs the live readback;
  - optionally swaps in the diagnostic and sidecar;
  - runs nav_bench with a wall-clock budget;
  - closes and validates the capture;
  - tears down its own process groups, then sweeps with `ros_clean.sh`.
- `nav_bench.py` changes are additive only:
  - `final_yaw_err_rad` per leg, since goals are yaw 0 and the checker judges
    yaw;
  - one greppable `FAILURE_CONTEXT` JSON line for every non-SUCCEEDED leg,
    built from existing fields.

## 5. Build and tests (measured this session)

**Build.** `colcon build --symlink-install --packages-select coco_config
custom_teleop gazebo_models coco_nav_diag` into this worktree's overlay:
**4 packages finished**, after moving the stale pre-rename `build/` and
`install/` aside.

**Tests.**

| Suite | Result |
|---|---|
| `coco_nav_diag` (`colcon test`) | **79 tests, 0 errors, 0 failures, 9 skipped** — the 9 are ament_cppcheck's own "skipped due to cppcheck 2.13.0 performance issues" |
| ↳ gtest `test_diag_recorder` | 15/0 |
| ↳ gtest `test_motion_decomposition_oracle` | 2/0 |
| ↳ gtest `test_amcl_diag_node` | 1/0 |
| ↳ pytest `test_amcl_diag_swap` | 28/0 |
| `gazebo_models` pytest (`--ignore=test_integration`) | **68/0** (41 before + 7 guard + 20 overlay) |
| `coco_config` pytest | **70/0** |
| `custom_teleop` pytest | **67/0** |
| `coco_mission` pytest, clean graph | **281/0** |
| `c2nav36_diag.py selftest` | **51/0** |
| `c2nav39_tour_report.py selftest` | **13/0** — reproduces C2-NAV.5's committed 18/21 from `c2nav5_bench.json` |

Not run this session: `coco_rl`, `coco_perception`, `coco_moveit_config` and
`coco_sim`. These packages are untouched and not built in this worktree's
overlay.

## 6. Simulation validation (measured 2026-09-15 UTC, real COCO simulation)

**Protocol.** `nav_tour_run.sh gazebo_models/config/experiments/baseline.yaml`
× 3, each a fresh headless Gazebo, topology A, the shipped
`nav2_params.yaml` (sha256 `6f61e499…`), the committed seven-leg TOUR, one
repeat, 75 s wall-clock per leg — the C2-NAV.5 protocol.

**Runs.**

- Run directories: `~/coco_nav_runs/baseline/baseline_r01..r03`.
- `r01` ran at `6466a35` with one uncommitted file (the `nav_bench.py`
  additions); `r02` and `r03` ran at `0b91330`, clean.
- Every run reached `all Nav2 lifecycle nodes active` in 28–34 s.
- Every run passed all 12 live read-back checks (`params_live.txt`): local
  CSF 65.0, global CSF 5.0, multi-pose BT, BaseObstacle 8.0, both goal
  tolerances, PolygonStop 0.25/4, PolygonSlow 0.3, `resample_interval` 1.
- Every teardown left `ros_clean: 0 matched`.
- `r01`'s `nav_bench.py` exited 139, a segfault after writing its JSON. This
  predates the session: 9 of 80 `.navbench` logs show `bench exit 139`. The
  runner now records it as `nav_bench_shutdown_crash` instead of failing a
  complete tour. `r02` and `r03` exited 0.

**Per-leg outcome.**

| leg | r01 | r02 | r03 |
|---|---|---|---|
| `open_space` | SUCCEEDED 21.92 s | SUCCEEDED 15.17 s | SUCCEEDED 14.13 s |
| `wall_adjacent` | SUCCEEDED 36.19 s | SUCCEEDED 25.99 s | SUCCEEDED 11.01 s |
| `wall_parallel` | SUCCEEDED 20.96 s | SUCCEEDED 19.17 s | SUCCEEDED 19.58 s |
| `obstacle_corner` | SUCCEEDED 18.30 s | SUCCEEDED 19.28 s | SUCCEEDED 18.56 s |
| `corridor_gate` | SUCCEEDED 21.31 s | SUCCEEDED 27.50 s | SUCCEEDED 20.27 s |
| `enclosure_entry` | TIMEOUT 68.59 s | SUCCEEDED 55.12 s | SUCCEEDED 51.91 s |
| `enclosure_exit` | TIMEOUT 76.40 s, PolygonStop 62.16 s | TIMEOUT 72.60 s, PolygonStop 65.48 s | TIMEOUT 55.75 s, PolygonStop 48.74 s |
| **total** | **5/7** | **6/7** | **6/7** |

**Against the established baselines** (`c2nav39_tour_report.py report`):

| leg | C2-NAV.5 (CSF 65) | C2-NAV.37 (unfixed default) | **C2-NAV.39** | C2-NAV.39 median s | true clear m | PolygonStop s (legs) |
|---|---|---|---|---|---|---|
| `open_space` | 3/3 | 15/15 | **3/3** | 15.17 | 0.5169 | 0 |
| `wall_adjacent` | 3/3 | 8/15 | **3/3** | 25.99 | 0.3946 | 0 |
| `wall_parallel` | 3/3 | 15/15 | **3/3** | 19.58 | 0.4509 | 0 |
| `obstacle_corner` | 3/3 | 15/15 | **3/3** | 18.56 | 0.4771 | 0 |
| `corridor_gate` | 3/3 | 15/15 | **3/3** | 21.31 | 0.4112 | 0 |
| `enclosure_entry` | 2/3 | 0/15 | **2/3** | 55.12 | 0.2832 | 0 |
| `enclosure_exit` | 1/3 | 15/15 | **0/3** | 72.60 | 0.2423 | 176.38 (3) |
| **total** | **18/21** | **83/105** | **17/21** | | | |

**Reading, strictly from these runs.**

- The five ordinary legs are 15/15, with 0 PolygonStop, as in C2-NAV.5.
- `enclosure_entry` is 2/3 like C2-NAV.5, with median 55.12 s against
  74.91 s. r01's timeout reached the 0.25 m xy tolerance at 64.32 s and ended
  0.199 m from the goal. Its heading was still 3.05 rad from the goal yaw
  when the cap hit, which is the known terminal-yaw behaviour (C2-NAV.1/.22).
- `enclosure_exit` failed 3/3. In every run the collision monitor held
  PolygonStop for most of the leg and the robot moved 0.25–0.30 m. The
  minimum world-file clearance, 0.2423 m, is inside the 0.25 m stop circle
  and outside the 0.2051 m circumscribed radius. This is the C2-NAV.5/.6
  mechanism: at CSF 65 the entry parks the robot where PolygonStop correctly
  holds it. It was 1/3 in C2-NAV.5. Across N=3 against N=3, 0/3 versus 1/3 is
  not distinguishable, and it is reported as a failure, not explained away.
- Against the unfixed default C2-NAV.37 actually ran: `wall_adjacent` went
  8/15 → 3/3 (median 56.58 → 25.99 s), and `enclosure_entry` went 0/15 → 2/3.
  `enclosure_exit` went 15/15 → 0/3, because C2-NAV.37's robot never entered
  the pocket, so its exit was not a control.
- The new `final_yaw_err_rad` field shows every SUCCEEDED leg ending 0.24–0.46
  rad (median per leg) from yaw 0 in ground truth. The goal checker's
  `yaw_goal_tolerance` is 0.25, and it judges AMCL's estimate. **Observation,
  not investigated:** AMCL's reported heading at goal acceptance differs from
  ground truth by roughly that excess.

N=3 fresh simulators is reproducibility, not a rate.

### 6b. Diagnostic-enabled run

**Protocol.** `nav_tour_run.sh gazebo_models/config/experiments/baseline_amcl_diag.yaml`,
with the same navigation inputs as `baseline`, plus two additions:

- the live component swap to `coco_nav_diag::AmclNode`, capture enabled;
- the GT sidecar.

**`baseline_amcl_diag_r01`** (`0b91330`, clean tree). Provenance is in
`docs/data/c2nav39/baseline_amcl_diag_r01_*`.

**Component swap** (`amcl_diag_swap.log`):

- `unloaded /amcl (uid 1); 13 other components intact`;
- `coco_nav_diag::AmclNode active with 45 parameters from nav2_params.yaml + 4 overrides`;
- read-back OK: `diag_enabled` True, `diag_output_path`, `resample_interval`
  1, `robot_model_type` `nav2_amcl::DifferentialMotionModel`;
- instrumented `/amcl` active 3 s after load;
- at teardown: `unloaded /amcl (uid 15); 13 other components intact`.

**Tour.** 7/7 SUCCEEDED. `enclosure_entry` took 57.57 s with true clearance
0.2927 m. `enclosure_exit` took 30.69 s with true clearance 0.3111 m and 0 s
of PolygonStop. This is the one tour this session in which the robot left
the pocket; one run, not a rate.

**Capture integrity, first live run on schema 2.**

- Header `{"type":"diag_session_start","node_name":"/amcl",…}` is the first line.
- Footer: `{"type":"diag_session_end","recorded":4438,"dropped":0,"written":4438,"write_failures":0,"last_update_index":2219}`.
- GT meta: 11859 rows written, 0 dropped, 0 stamp regressions, 0 duplicate
  stamps, one frame pair `world → base_footprint`, `clean_shutdown: true`.

**Strict join:** `status: OK`, schema 2, **213 candidates, 213 correlated, 0
unobservable.** Residuals, odometry input − GT per update:

| axis | residual |
|---|---|
| along-track | +0.000487 m |
| lateral | +0.001812 m |
| yaw | −0.011855 rad |

The along-track mean has the same sign and order of magnitude as C2-NAV.37's
+0.000747. Lateral and yaw have the opposite sign to C2-NAV.37's pooled
−0.002104 / +0.010160. That comparison is one run on the accepted config
against four runs on the unfixed default. **Observation only, not
interpreted.**

**Failure found and fixed.** The runner process died *after* the tour and
the join had both completed, while its teardown sweep was running (Bash exit
144).

- **Cause, measured:** the `'amcl_dia[g]'` pattern this session added to
  `ros_clean.sh` matches any command line *containing* `amcl_diag`, including
  `nav_tour_run.sh …/baseline_amcl_diag.yaml` and its parent shell. The log
  reads `ros_clean: 2 matched`, after the runner had already killed its own
  process groups.
- **Demonstrated before the fix:** `ros_clean.sh --list baseline_amcl_diag.yaml`
  listed its own process.
- **Fixed in `f144852`:** the patterns are anchored on `lib/coco_nav_diag/amcl_diag`
  and `amcl_diag_swap.py`, and the runner now refuses to start if
  `ros_clean.sh --list` already matches anything.
- **Verified after the fix:** the same `--list` matches nothing, and a
  `nav2_` argument still self-matches, so the detection works.
- **Impact on r01:** no leftover processes, and the data files are complete.
  `manifest.json` lacks `finished_utc`/`exit_code` because the runner was
  killed before writing them. That field is left missing, not back-filled.

**`baseline_amcl_diag_r02`** (`f144852`, clean tree) — re-run of the
failing configuration. Provenance: `docs/data/c2nav39/baseline_amcl_diag_r02_*`.

- **Validates the `f144852` fix:** the runner passed its new pre-start
  self-check and survived its own teardown sweep. `ros_clean: 0 matched`,
  and the manifest has `finished_utc` and `exit_code` written.
- **Found a second, independent defect** (runner exit 6, before any leg
  ran):
  - The unload was clean: `unloaded /amcl (uid 3); 13 other components intact`.
  - The load succeeded.
  - Then `ros2 lifecycle set /amcl configure` failed: "Unknown transition
    requested, available ones are: deactivate [4], shutdown [7]". nav2's
    localization lifecycle manager had already configured and activated the
    reloaded node. r01 had won that race; r02 lost it.
- **The runner handled the failure correctly:** it unloaded the instrumented
  node (`uid 15; 13 other components intact`), tore down, and left no
  leftover processes.
- **Fixed in `fbb4dc5`:** the swap reads `ros2 lifecycle get /amcl` and issues
  only the transition that state needs, re-querying after a lost race. It
  logs which transitions it issued, and the live parameter read-back remains
  the proof.
- **Tests:** `test_amcl_diag_swap` 33/0; `ament_flake8` / `ament_pep257` clean.

**`baseline_amcl_diag_r03`** (`fbb4dc5` plus one uncommitted documentation
file, `docs/agents/HANDOFF.md`; both fixes, fresh simulator). The runner
exited 0, `ros_clean: 0 matched`, and no processes were left over. Provenance: `docs/data/c2nav39/baseline_amcl_diag_r03_*`.

**Swap** (state-aware path, live):

- `unloaded /amcl (uid 1); 13 other components intact` → load → `lifecycle get`
  unconfigured → `configure` → `get` inactive → `activate` → `get` active;
  "transitions issued here: configure, activate".
- Read-back OK; instrumented `/amcl` active 3 s after load.
- At teardown: `unloaded /amcl (uid 15); 13 other components intact`.
- **In this run the tool won the race.** The branch where the lifecycle
  manager activates the node first, which is what failed r02, is covered by
  the state parser's unit tests. **It was not exercised live.**

**Tour:** 6/7.

- `enclosure_exit` SUCCEEDED in 32.07 s.
- `enclosure_entry` TIMEOUT at 57.67 s: inside the xy tolerance at 47.57 s and
  ending 0.083 m from the goal, but −2.94 rad from goal yaw at the cap. That
  is the terminal-yaw pattern again, the same as baseline r01's entry
  timeout.

**Capture integrity:**

- Footer `recorded 4752, dropped 0, written 4752, write_failures 0, last_update_index 2376`.
- GT meta: 12849 rows, 0 dropped, 0 regressions, 0 duplicate stamps,
  `clean_shutdown: true`.
- **Strict join: `status: OK`, 207 candidates, 207 correlated, 0 unobservable.**
- Residuals per update: along-track +0.000467 m, lateral +0.000822 m,
  yaw −0.006242 rad.

**Both completed diagnostic tours** (r01 + r03, `c2nav39_tour_report.py`):
**13/14**.

- Ordinary legs 10/10.
- `enclosure_entry` 1/2.
- `enclosure_exit` 2/2, 0 s PolygonStop; minimum world-file clearance 0.2949 m.
- The residual means agree in sign across the two capture runs: along-track
  +0.000487 / +0.000467 m, lateral +0.001812 / +0.000822 m, yaw
  −0.011855 / −0.006242 rad.

The navigation inputs are identical to the three `baseline` runs, where
`enclosure_exit` went 0/3, so the 2/2 is not attributed to the diagnostic.
**N = 2 against N = 3 separates nothing.**

Across all five completed fresh-sim tours of the shipped config (3 baseline
+ 2 diagnostic):

| | Result |
|---|---|
| Total | **30/35** |
| Ordinary legs | 25/25 |
| `enclosure_entry` | 3/5 (both failures are terminal-yaw timeouts inside the xy tolerance) |
| `enclosure_exit` | 2/5 (all three failures are PolygonStop holds) |

## 7. Established baselines used for comparison

Per-leg, from `c2nav39_tour_report.py report` (measured this session from
committed/on-disk data, not re-run):

- **C2-NAV.5**: CSF 65, BaseObstacle 8.0, committed TOUR, 1 repeat per fresh
  sim, 75 s. **18/21.**
- **C2-NAV.37**: the unfixed default (BaseObstacle 2.0, CSF 5.0), 3 repeats
  per fresh sim, 75 s. **83/105.**

| leg | C2-NAV.5 succeeded | C2-NAV.5 median s | C2-NAV.37 succeeded | C2-NAV.37 median s |
|---|---|---|---|---|
| `open_space` | 3/3 | 14.89 | 15/15 | 14.76 |
| `wall_adjacent` | 3/3 | 22.19 | 8/15 | 56.58 |
| `wall_parallel` | 3/3 | 18.97 | 15/15 | 16.17 |
| `obstacle_corner` | 3/3 | 17.80 | 15/15 | 17.95 |
| `corridor_gate` | 3/3 | 25.78 | 15/15 | 21.30 |
| `enclosure_entry` | 2/3 | 74.91 | 0/15 | 77.22 |
| `enclosure_exit` | 1/3 (142.86 s PolygonStop over 2 legs) | 77.14 | 15/15 | 22.44 |

C2-NAV.37's clean `enclosure_exit` is not a control. Its entry never
succeeded, so the robot never parked inside the pocket (C2-NAV.5's own
caveat).

## 8. Decision

**Improved** (measured, against what actually shipped before this session):

- **Shipped config.** The default no longer carries a rejected value, and a
  plain `nav.launch.py` / `mission.launch.py` now gets the validated
  configuration.
- **Navigation.** Ordinary legs 25/25 across five fresh tours.
  `wall_adjacent` went from 8/15 (median 56.58 s) to 5/5. `enclosure_entry`
  went from 0/15 to 3/5.
- **Safety.** No gate moved, and all 12 live safety/accepted-parameter checks
  passed on every run.
- **Capture integrity.** Integrity is now judged from the files themselves.
  Every differential-review check exists, is unit-tested, and passed on
  2,273 historical and 420 new real updates.
- **Tooling.** A tour is one command with a manifest. The simulator starts
  from the renamed workspace.

**Regressed or not improved:**

- **`enclosure_exit`: 2/5**, with all three failures PolygonStop holds.
  C2-NAV.5 measured 1/3 on the same configuration. The C2-NAV.37 15/15 is
  not a control.
- **Baseline protocol total: 17/21** against C2-NAV.5's 18/21. N=3 against
  N=3 does not separate these.
- **`enclosure_entry`:** both of its failures are terminal-yaw timeouts
  inside the xy tolerance.
- **Topology B.** Changing the default also changes `mission.launch.py`'s
  navigation, and topology B was not validated in this session.

**Unresolved:**

- The `enclosure_exit` PolygonStop trap (C2-NAV.5/.6 geometry).
- Terminal yaw on `enclosure_entry`.
- Ground-truth heading at goal is 0.24–0.46 rad from goal yaw against an AMCL
  tolerance of 0.25 (observed, not investigated).
- `nav_bench.py`'s shutdown segfault (pre-existing).
- The lost-race branch of the swap is covered by unit tests only.
- The residual sign difference between C2-NAV.37 and C2-NAV.39 captures
  (lateral/yaw).
- C2-NAV.38's scientific question (the place-dependent AMCL bias) is recorded
  and paused, per the brief.

**Single best next implementation action:** attack the largest measured
failure, `enclosure_exit`, without touching PolygonStop.

1. Add a `goals` override to the experiment schema; `nav_bench.py --goal`
   already exists.
2. Commit `experiments/entry_corridor_centre.yaml` with C2-NAV.7's
   `(-3.575, 2.95)`, the only change on record that took the exit to 3/3 with
   0 STOP frames.
3. Run ≥3 fresh sims through `nav_tour_run.sh`.
4. Judge against this session's 17/21 and 2/5 with `c2nav39_tour_report.py`,
   gating explicitly on the C2-NAV.8 SW-corner deadlock (PolygonStop time on
   `enclosure_entry`).

Moving a benchmark goal is the human engineer's decision.
