# C2-NAV.44 — M6 re-measured on the fixed command path

Branch `c2nav43-integration`, HEAD `c8a8206`, clean tree. Date 2026-09-17.
Every number below was produced in this session, on this branch, on a **fresh
simulator per run**, headless, never `--fast`, depth fusion **off**. Run
directories are under `~/coco_nav_runs/c2nav44_m6/`; the per-run readbacks and
summaries are committed under `docs/data/c2nav44_live/`.

**Why this exists.** `PROJECT_STATE.md` KNOWN LIMITATIONS 0 said "M6's 19/20
is **not yet measured** on the fixed path". The C2-NAV.42 fix changed which
command reaches the wheels, so the standing 19/20 — measured with the
`/cmd_vel_nav` loop in place — could not be quoted as evidence for the
shipping architecture. This measures the mission again on the current wiring.

---

## 1. Verdicts

| | |
|---|---|
| **Command path** | **No regression. Clean in every run.** Raw-controller → wheel bypass **0** in all six missions, by C2-NAV.41's unmodified metric. Stale command drops **0**. PolygonStop activations **0**. Exactly one wheel publisher throughout. |
| **M6 fetch, executive-driven** | **3 of 6 complete** (r01 red, r03 blue, r04 yellow). Three aborts, **all green**, all `PRE_RAMP_POSE_OUT_OF_REGION`, all before the climb. |
| **Attribution** | The three aborts are **not** a command-path failure. They are the executive's ground-truth arrival gate set to the same 0.25 m as Nav2's own goal checker, which the repo already documents as a defect class for the **yaw** gate (`mission_states.GOAL_YAW_TOLERANCE`). Measured mechanism in §4. |
| **Comparability with 19/20** | **Not a like-for-like control.** The historical matrix ran `traverse_demo.py`, which has **no** ground-truth arrival gate and therefore cannot produce this abort. Run on this branch, that harness delivers the **green** fetch end to end — `FETCH COMPLETE`, home to 0.04 m (§5). |
| **Depth fusion** | Off in every run, proven per run (§3). Unchanged: KEEP AS CANDIDATE. |

---

## 2. Procedure

The shipping mission, exactly as `HOW_TO_RUN.md` documents it, one fresh
simulator per run, torn down by process name between runs:

```bash
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py rviz:=false target_colour:=<colour>
ros2 service call /mission/start std_srvs/srv/Trigger
```

`gui:=false` because the session is headless; every other argument is at its
shipped default (executive on, `target_source:=target_finder`, web panel on,
localization monitor and recovery on, `depth_cloud` not passed = off).
Colours follow the M6 matrix rotation (red, green, blue, yellow).

Runner: `docs/data/c2nav44_m6_run.sh` (adapted from C2-NAV.42's
`live_mission.sh` with the fault injection removed). Per run it refuses to
start if anything is already running, checks bring-up, records the whole
mission, and tears down. Recorders, all passive:

- `docs/data/c2nav42_cmdpath.py record --until-terminal` — every link of the
  command chain, the collision monitor, the arbiter's active source,
  `/mission/state`, ground truth;
- `docs/data/c2m51_hrec.py` — the localization monitor's own output;
- `ros2 topic echo /mission/state` — the raw state line;
- a 20 s poll of `/diff_drive_controller/cmd_vel`'s publisher list.

Offline report: `docs/data/c2nav44_m6_report.py` (no ROS; counts over the
logs and the trace, command-path metrics taken from the recorder's own
`summary.json`).

**Run count declared before running: 4** (one per colour). After the green
abort, **one** extra green run was declared to test lane-specificity (r05),
and **one** more green run with `/amcl_pose` recording (r06) to separate two
candidate causes. Nothing was re-run to get a better number, and no run was
discarded for its outcome.

---

## 3. Pre-flight and per-run safety readback

Identical in all six runs, and re-checked on each fresh graph (the full
readbacks are in `docs/data/c2nav44_live/<run>/`):

- HEAD `c8a8206`, **0 dirty paths**; `nav2_params.yaml` sha256
  `6f61e499…` — the owner-accepted file, byte-identical.
- Every `coco*` / `custom_teleop` / `gazebo_models` package resolves into this
  worktree's `install/`; both launch files and the shipped policy resolve.
- All ten Nav2 lifecycle nodes `active`; live parameter readback **0
  mismatches** against the shipped file.
- `verify-topology` topology B: **12 OK**, the single MISMATCH being
  `arbiter mode: want nav, got 'idle'`, which is correct before a mission
  starts.

```
/diff_drive_controller/cmd_vel  publisher count 1  ['cmd_vel_arbiter']
/cmd_vel_nav       pub [behavior_server, controller_server]  sub [velocity_smoother]
/cmd_vel_smoothed  pub [velocity_smoother]                   sub [collision_monitor]
/cmd_vel           pub [collision_monitor, docking_server]   sub [cmd_vel_relay]
/cmd_vel_gated     pub [cmd_vel_relay]                       sub [cmd_vel_arbiter]
```

- **Depth fusion off, proven per run:** no `image_proc/resize_node`, no
  `depth_image_proc/point_cloud_xyz_node`, no `depth_cloud.launch.py`; and
  all four costmap layers read `observation_sources: scan`
  (`depth_off.txt`).
- The wheel topic had **exactly one publisher on every poll taken while a
  mission was running**. The polls returning nothing all fall outside the
  mission window (before the sim was up, or after terminal).

### Command path, per mission

Metrics are C2-NAV.41's `monitor_authority`, `bypass_source` and
`stop_breach`, imported unchanged, on the recorder's 10 Hz trace. The
headline row is the subset the Nav2 chain owns — arbiter `active=nav`, 1.0 s
switch guard — exactly as C2-NAV.42 reported a whole mission.

| run | colour | outcome | Nav2-owned rows | wheels above monitor | bypass rows | wheel = raw controller | STOP rows | stale drops |
|---|---|---|---|---|---|---|---|---|
| r01 | red | COMPLETE | 577 | **0** | **0** | **0** | 0 | **0** |
| r02 | green | ABORT | 57 | **0** | **0** | **0** | 0 | **0** |
| r03 | blue | COMPLETE | 552 | **0** | **0** | **0** | 0 | **0** |
| r04 | yellow | COMPLETE | 740 | **0** | **0** | **0** | 0 | **0** |
| r05 | green | ABORT | 56 | **0** | **0** | **0** | 0 | **0** |
| r06 | green | ABORT | 59 | **0** | **0** | **0** | 0 | **0** |

**Raw-controller → wheel bypass = 0 in every run.** PolygonStop never
activated in any run (r01 had 95 SLOWDOWN rows, r04 had 4; no STOP), so no
run tested a STOP hold — this sprint adds **no** new evidence about STOP
beyond C2-NAV.43's controlled test, and does not claim any.

Over **all** rows (not just the Nav2-owned subset) r01/r03/r04 show
343/1507, 344/1467 and 346/1666 rows where the wheels exceeded the monitor,
and 73/74/75 "bypass" rows, every one at `cm_action 0`. These are the RL
climb and the approach servo driving the wheels while Nav2's monitor is
idle — sources that never passed through the monitor, before the fix or
after. In every one of those rows the wheel command matched the raw
controller **0** times. This is the same reading C2-NAV.42 recorded for its
instrumented mission.

**Smoother in the live path** (message-level, per wheel command): r01 955
wheel messages had raw ≠ smoothed, and the wheel matched the smoother 905
times and the raw-only command **4**; r03 436/403/**0**; r04 555/399/**2**;
r05 131/129/**1**; r06 41/39/**0**. The 7 raw-only matches across six runs are
reported, not explained — they are not attributed to a mechanism here, and
they are not counted as bypass rows by C2-NAV.41's metric.

---

## 4. The three aborts: what actually happened

All three green runs end the same way, before the climb:

```
NAVIGATE_TO_RAMP -> RECOVERY [PRE_RAMP_POSE_OUT_OF_REGION]: 0.32 m from (0.5, -0.25), tolerance 0.25 m
RECOVERY -> NAVIGATE_TO_RAMP [PRE_RAMP_POSE_OUT_OF_REGION]: retry 1/2
... retry 2/2 ...
RECOVERY -> ABORT: NAVIGATE_TO_RAMP exhausted its retries
MISSION ABORT: result=aborted reason=PRE_RAMP_POSE_OUT_OF_REGION attempts={'NAVIGATE_TO_RAMP': 2}
```

**Measured, per run** (ground truth from `/model/coco/odometry`, the same
source the executive gates on):

| | r02 | r05 | r06 |
|---|---|---|---|
| where the wheels stopped (world) | (0.193, −0.212) | (0.191, −0.209) | (0.197, −0.223) |
| true distance to the goal (0.5, −0.25) | 0.3097 m | 0.3115 m | 0.3047 m |
| closest the robot ever came | 0.3097 m | 0.3115 m | 0.3047 m |
| Nav2's own verdict | `Reached the goal!`, `Goal succeeded` ×3 | same | same |
| localization health | CONSISTENT, `degraded=0` | same | same |

Four facts, each measured:

1. **Nav2 was satisfied.** `controller_server` logged `Reached the goal!` and
   `bt_navigator` `Goal succeeded` on all three attempts of every run. The
   goal checker is `SimpleGoalChecker`, `stateful: true`,
   `xy_goal_tolerance: 0.25`, judged against the pose Nav2 is steering by.
2. **The executive checks a different pose.** `_check_nav_leg` re-tests the
   leg against **ground truth** with `GOAL_XY_TOLERANCE = 0.25` — deliberately
   "calibrated to the same number the planner was told to achieve".
   Two tolerances of 0.25 m, measured from two different poses, leave **zero**
   margin: any localization or odometry offset in the adverse direction puts
   the true error above the gate.
3. **The retries could not have worked.** After the first stop the wheels
   never turned again: `v_wheel` is 0.000 m/s for the rest of the run in all
   three. Each retry re-sent the same goal to a controller that already
   believed it had arrived, and Nav2 re-reported success without moving. The
   repo already says this of the yaw gate — *"the same goal through the same
   goal checker cannot produce a tighter yaw than the checker's own tolerance,
   so the retry is structurally futile"* — and this is the same statement on
   the position axis, now measured live.
4. **AMCL was not diverged.** With `/amcl_pose` recorded (r06,
   `docs/data/c2nav44_live/r06_green_posed/poses.csv`), the estimate is a
   staircase: it holds while the robot drives and jumps every ~0.25–0.30 m
   (AMCL's `update_min_d`). Read immediately after each correction its error
   against ground truth is **0.004–0.049 m**. The health monitor scored
   `CONSISTENT` with `degraded=0` throughout.

**Why green and not the others.** The goal checker stops the robot when its
*estimate* says 0.25 m. Where the estimate sits relative to truth decides the
outcome, and it is consistent per lane across runs:

| lane | colour | where the wheels first stopped | true distance at the stop | leg verdict |
|---|---|---|---|---|
| −0.75 | red | x 0.381 | 0.147 m | pass |
| −0.25 | green | x 0.193 / 0.191 / 0.197 | **0.310 / 0.311 / 0.305 m** | **fail ×3** |
| +0.25 | blue | x 0.414 | 0.089 m | pass |
| +0.75 | yellow | x 0.486 | 0.056 m | pass |

On the three passing lanes the estimate lags, so the robot drives closer than
the tolerance and the ground-truth error lands at 0.056–0.147 m. On green it
leads, so the robot stops at the tolerance and the ground-truth error lands at
0.305–0.311 m — 0.055–0.061 m outside a 0.25 m gate. The green stop point
repeats to within 6 mm across three fresh simulators.

**What is NOT established.** Why the offset's sign is lane-dependent is not
diagnosed here, and this sprint deliberately did not investigate AMCL, tune
DWB or touch a goal, a parameter or a tolerance. Three runs per lane is not a
rate.

---

## 5. Comparison with the historical 19/20

**Historical pre-fix reference: 19/20** — 20 runs, five per colour, measured
with the `/cmd_vel_nav` loop in place **and with a different sequencer**.

`traverse_demo.py` is the harness that matrix was measured with, and its
`nav_to()` returns `True` on Nav2's `SUCCEEDED` and nothing else. It has **no
ground-truth region check**, so the failure that ends r02/r05/r06 **cannot
occur in it**. The executive's gate arrived later, with C2-M3. The two figures
therefore measure different things, and 3/6 must not be subtracted from 19/20.

### The harness comparison, run (`t02_green_traverse`)

So the harness was run, on **this** branch, on the **same fixed command
path**, on a fresh simulator: `mission.launch.py rviz:=false
executive:=false` + `ros2 run gazebo_models traverse_demo.py --colour green`
— the colour that aborts 3 times out of 3 under the executive.

```
--- 1. nav to the pre-ramp pose ---   nav to (0.50, -0.25): SUCCEEDED in 16.2s
--- 2. RL climb ---                   outcome=goal, 64 steps, progress 4.73, lateral +0.10
--- 2b. confirm green is in front --- sel=green found=1 lane=-0.250 seen=red,green,blue,yellow
--- 2c. stow the arm ---              outcome=done
--- 3. approach the target ---        outcome=arrived, travel 1.153 m, stop 0.154
--- 4. pick it up ---                 outcome=held, lifted=1, x=0.1545
--- 5. scripted descent ---           outcome=goal, 586 steps, progress 6.65
--- 6. nav home ---                   nav to (-2.00, 0.00): SUCCEEDED in 190.7s
--- 7. put it down at home ---        outcome=placed
end (world): (-1.97, 0.02) — home to within 0.04 m
FETCH COMPLETE — green delivered          (rc 0, 413 s wall)
```

**The green fetch completes end to end on the fixed command path.** The same
first leg that the executive rejected three times — Nav2 reporting SUCCEEDED
with the robot ~0.31 m short — is accepted by `traverse_demo.nav_to()`, and
the mission proceeds to a physically verified grasp (`lifted=1`, base-x
0.1545, inside the 5.5 mm window) and a delivery home.

This is the direct answer to "did the command-path fix regress M6": **no.**
The fetch the 19/20 harness measures still completes on the current wiring;
what changed the outcome in r02/r05/r06 is the executive's arrival gate,
which did not exist when 19/20 was measured.

Command path on this run: Nav2-owned rows **947**, **bypass rows 0**,
raw-controller matches **0**, STOP rows 0, stale drops **0**, one wheel
publisher. Wheels above the monitor **3 / 947 (0.32 %)** — inside the
0.088–2.92 % short-streak residual C2-NAV.43 recorded and did not attribute,
and the only run in this sprint to show it at all.

**One observation, offered as an observation.** The home leg took **190.7 s**
against the historical single-run figure of 103.6 s. N = 1 against N = 1, a
different colour, and this sprint changed nothing that would be tested by
comparing them — no causal claim is made, and none should be read in.

**Caveat.** One run. It shows the fetch *can* complete on this lane and this
wiring; it is not a rate, and it does not re-measure 19/20.

---

## 6. Runs

| run | colour | result | reason | wall s to terminal | ramp-leg end error | grasp lift | notes |
|---|---|---|---|---|---|---|---|
| r01 | red | **COMPLETE** `result=fetch` | — | 295 | 0.091 m | 35.3 mm | no retry, no recovery |
| r02 | green | ABORT | `PRE_RAMP_POSE_OUT_OF_REGION` | 24 | 0.315 m | — | aborted before the climb |
| r03 | blue | **COMPLETE** `result=fetch` | — | 311 | 0.081 m | 34.6 mm | pre-climb heading +0.292 rad; climb drift +0.02 m |
| r04 | yellow | **COMPLETE** `result=fetch` | — | 323 | 0.068 m | 35.6 mm | one Nav2 recovery + costmap clear on the ramp leg, then succeeded |
| r05 | green | ABORT | `PRE_RAMP_POSE_OUT_OF_REGION` | 24 | 0.313 m | — | reproduces r02 |
| r06 | green | ABORT | `PRE_RAMP_POSE_OUT_OF_REGION` | 23 | 0.305 m | — | `/amcl_pose` recorded |
| t02 | green | **FETCH COMPLETE** | — | 413 | leg accepted by the harness | held, base-x 0.1545 | historical harness, `executive:=false` (§5) |

The three completions traversed all 16 nominal states with `attempt=1`,
`reason=--` and `attempts={}` — no retry, no RECOVERY, no RELOCALIZE. Mission
duration, first active state to terminal: **149.3 / 145.4 / 165.0 s** sim.

**Grasp and approach, against the M6 matrix bands** (the matrix measured
33.9–35.9 mm lift and a 5.5 mm approach window centred at 0.1537):

| run | base-x reported | in the [0.1510, 0.1565] window | lift |
|---|---|---|---|
| r01 red | 0.1540 | yes | 35.3 mm |
| r03 blue | 0.1547 | yes | 34.6 mm |
| r04 yellow | 0.1546 | yes | 35.6 mm |

3 of 3, inside the historical band, on the fixed command path. Each lift is
`grasp_server` reading the target's height from Gazebo, not an action result.

### VOID runs

| run | why |
|---|---|
| `t01_green_traverse` | **VOID — infrastructure.** `traverse_demo.py` could not create a ROS node: *"Failed to find a free participant index for domain 0"*. A second, unrelated Gazebo (`eyantra_kepler_colony`, from `~/ros2_ws`) was running on this machine, started **after** this run's pre-flight check found the machine idle. Two full stacks plus this session's recorders exhausted the CycloneDDS participants on domain 0. No mission was driven. Replaced by `t02_green_traverse`. |

No run was voided for its outcome.

### Shutdown

Every run tore down cleanly: `ros_clean.sh --list` empty afterwards, and the
runner's "no launched process died" check passed **during** every mission.
After the teardown SIGINT, several Python nodes exit 1 with
`ExternalShutdownException` and `nav2_container` / `move_group` exit on a
signal; these lines always follow the terminal state in the log and are
teardown noise, not mission failures.

---

## 7. Traps paid for this session

- **`ros_clean.sh` sweeps every Gazebo on the machine, not just this
  session's.** Its `g[z] sim` pattern matched an unrelated simulator from
  `~/ros2_ws` that started mid-run, and the teardown killed it. The runner's
  refusal check only proves the machine was idle *at the start*.
- **A polling watcher can exhaust the DDS domain.** A 20 s poll that shells
  out to `ros2 topic info` and `ros2 topic echo` creates participants; with a
  second stack up, the next process to start fails with "Failed to find a free
  participant index for domain 0" — and the failure lands on the innocent
  process, not the cause.
- **`/amcl_pose` is not a pose feed.** It publishes on resample updates only,
  so between updates it is stale by up to a whole `update_min_d` of travel.
  Anything comparing it to ground truth at an arbitrary instant measures the
  staleness, not the localization error. Read it right after a jump.

---

## 8. Decision

**The command-path fix is regression-tested for M6 and stands.** Six fresh
missions, zero raw-controller bypass, zero stale drops, one wheel publisher,
the smoother and the monitor demonstrably in the path, and three complete
fetches whose grasp and approach numbers sit inside the historical bands.

**M6 on the shipping executive is 3/6, and the three failures are one
defect with one measured mechanism**, independent of the command path: a
ground-truth arrival gate set to the same 0.25 m as the estimate-based goal
checker it is checking, with a retry that cannot move the robot. The repo
already reached this conclusion for the yaw gate and turned that gate off,
recording that *"the mission it aborted is the mission that completes
19/20"*. The harness that measured 19/20 delivers the green fetch on this
branch (§5), which is what separates "the fix broke the mission" from "the
gate rejects a leg the mission can finish from".

**Integration recommendation: keep `c2nav43-integration` ready for `main` on
the command-path evidence, and treat the pre-ramp gate as a separate,
owner's decision.** Merging is not mine to do. The gate is mission logic, it
is reachable by a one-line change, and three candidate directions exist
(widen the ground-truth gate above Nav2's own tolerance, tighten Nav2's goal
checker for that leg, or report-not-gate as the yaw gate already does) — each
needs a measured threshold, and none was chosen here.

**Not done here, deliberately:** nothing was changed to make the number
better. The gate, the tolerances, the goals, the parameters and the depth
default are all untouched.
