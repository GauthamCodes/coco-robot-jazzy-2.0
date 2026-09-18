# C2-NAV.47 — the C2-NAV.43–46 work prepared as the new main baseline

Branch `c2nav43-integration`, audited and re-measured at `aa5b968`, clean
tree. Date 2026-09-19. **This sprint is an integration, not an
investigation.** Every number below was produced in this session, on this
branch, on a fresh simulator, headless, never `--fast`, depth fusion
**off**.

Run directories `~/coco_nav_runs/c2nav47_m6/` and
`~/coco_nav_runs/c2nav47_cmdpath/`; readbacks committed under
`docs/data/c2nav47_live/`.

**No runtime code was changed by this sprint.** The only files it adds are
this document, the `docs/data/README.md` index rows for evidence that was
already committed without them, the session-log checkpoint, and its own
regression readbacks.

It also fixes **one defect the merge itself would introduce**:
`c2nav45_m6_sweep.sh` and `c2nav46_matrix_sweep.sh` hardcoded the worktree
path `.claude/worktrees/c2nav43-integration`, which stops existing the
moment the branch merges. Both now use the self-locating idiom
`c2nav44_m6_run.sh` already used — verified to resolve to the repo root and
to stay overridable by `COCO_WT`. The `MV=` moveit-prefix hardcode in
`c2nav44_m6_run.sh` and `c2nav44_m6_run_traverse.sh` points at the *real*
workspace, survives the merge, and is **reported rather than changed**.

---

## 1. Verdicts

| | |
|---|---|
| **Ancestry** | `main` is at `ea66155` and **has not moved**. `git merge-base main c2nav43-integration` **equals `main`**, so the branch is a strict descendant and a **fast-forward is available**. Measured, not assumed. |
| **Clean build** | **9 of 9 packages, exit 0**, from a wiped `build/ install/ log/`. |
| **Tests** | **997 passed, 0 failed, 0 skipped**, per package, cwd inside each package, on a clean ROS graph. |
| **M6 green regression** | **COMPLETE, `result=fetch`**, one fresh executive-driven mission. |
| **The arrival gate** | **Both bands exercised in the one run.** Pre-ramp 0.290 m → WARN, accepted, **not retried**. Return 0.108 m → INFO, clean. |
| **Command path** | **Raw-controller → wheel bypass 0** in both the mission and the controlled experiment. The arbiter is the **sole** publisher to the wheels (`Publisher count: 1`, read off the live graph). |
| **Collision monitor** | **The STOP reaches the wheels: 69 STOP rows, 0 with the wheels driven**, while the probe held a raw 0.30 m/s. |
| **Depth fusion** | **Off**, read back live: no depth-cloud process, and all four costmap observation sources are `scan`. |

---

## 2. What was audited, and what it resolved to

23 commits, `main..c2nav43-integration`, 297 changed paths. The production
surface is small; the bulk is the evidence record.

The four SHAs the sprint brief named are the **originals on
`worktree-c2nav0-diagnosis`**, not commits on this branch. They are
already here, re-applied:

| brief SHA | subject | on this branch | patch |
|---|---|---|---|
| `d707327` | C2-NAV.42 cmd_vel wiring | `6503cd5` | **byte-identical** |
| `8bf1fe4` | C2-NAV.42 wiring tests | `2a66959` | **byte-identical** |
| `57f75d8` | C2-NAV.39 world-path quoting | `f3bfa0a` | **byte-identical** |
| `ad2b8b8` | C2-NAV.39 nav2 defaults | `d260749` | **equivalent** — see below |

`ad2b8b8` also reverted the **rejected** `BaseObstacle.scale: 2.0`
experiment back to 8.0. `main` never carried that value, so `d260749`
had no such hunk. Both commits land
`gazebo_models/config/nav2_params.yaml` on **the same blob, `ac5debd`** —
which is the check that matters, and it is an identity, not a judgement.

### Production runtime (A)

`custom_teleop/cmd_vel_arbiter.py`, `custom_teleop/cmd_vel_relay.py`,
`coco_mission/scripts/mission_states.py`,
`coco_mission/scripts/mission_executive.py`.

### Production configuration (B) — two lines, and that is all

```
bt_navigator.default_nav_through_poses_bt_xml
    navigate_to_pose_w_replanning_and_recovery.xml
 -> navigate_through_poses_w_replanning_and_recovery.xml
local_costmap.inflation_layer.cost_scaling_factor   5.0 -> 65.0
```

`BaseObstacle.scale` is **8.0 on both sides** — it was already the shipped
value. The **global** costmap's `cost_scaling_factor` is **untouched at
5.0**: the accepted change is local-only, and that scope was verified, not
assumed. The collision-monitor block (`PolygonStop`, `PolygonSlow`,
`PolygonLimit`, `FootprintApproach`) is outside the two-line diff and is
therefore **byte-identical to `main`**.

### Tests (C), launch and tooling (D), documentation (E), evidence (F)

7 test files; 6 launch/build files (two of them docstring-only) plus
`ros_clean.sh` and the bench tooling; 8 documents; and the
`docs/data/c2nav4*` evidence.

**The evidence bundles are kept, deliberately.** `docs/data/` is already
the repository's evidence record — `main`'s own
`docs/data/README.md` opens *"Backing data for the numbers in RESULTS.md,
committed so the claims are checkable rather than asserted"* — and the
four `C2-NAV.4x_RESULTS.md` documents reference `docs/data` 31 times. The
four new bundles total **1.4 MB** against an existing 7.1 MB `docs/data/`.
They are the repository's stated convention, at its established scale, not
a dump.

What they did **not** have was an index row. This sprint adds those to
`docs/data/README.md`, and records one property measured here: **the
committed run directories are slimmed, and re-running a report against
them does not reproduce the committed report** — the gate report prints
`no arrival recorded`. The report *outputs* (`gate_report.json`,
`matrix.json`, `matrix_report.txt`) are the committed artefact. Said
plainly in the index so no one mistakes a slimmed directory for a
reproduction failure.

---

## 3. No experimental default was promoted

Each verified against the shipped configuration, not against a launch
argument's description:

- **Depth fusion off.** `nav.launch.py`'s `depth_cloud` argument defaults
  to `'false'` behind an `IfCondition`. In the shipped `nav2_params.yaml`
  every `observation_sources` is `scan` with `data_type: "LaserScan"`;
  there is no `/camera` topic, no `PointCloud2` source. Read back **live**
  during the M6 run: *"no depth-cloud process, every observation source is
  scan"*, all four.
- **The experiment arms cannot activate themselves.**
  `gazebo_models/config/experiments/*.yaml` — including
  `depth_fusion.yaml` and `baseline_topology_b.yaml` — are referenced only
  by `nav_tour_run.sh`, `nav_params_overlay.py` and tests. **No launch
  file and no runtime node reads that directory.**
- **Goals unchanged.** `docs/data/nav2_goals.json` and `coco_config` are
  not in the diff at all; no goal constant moved in `mission_states.py`.
- **Safety unchanged.** See the collision-monitor note in §2.

---

## 4. Build and tests

Clean build: `build/ install/ log/` wiped, then all nine packages, **exit
0**. The only stderr is a pre-existing `pytest-repeat` setuptools warning
from the environment.

| package | tests |
|---|---|
| `coco_config` | 70 |
| `custom_teleop` | 75 |
| `coco_rl` | 164 |
| `coco_perception` | 139 |
| `gazebo_models` | 171 |
| `coco_moveit_config` | 12 |
| `coco_sim` | 55 |
| `coco_mission` | 311 |
| **total** | **997** |

**0 failed, 0 skipped.** `coco_moveit_config`'s 12 include the 7
`test_pick_poses` that *skip* without the user-space MoveIt prefix, so
their presence is itself the check that the prefix was on the path.

`coco_web` has no `test/` directory and reports *"no tests ran"*. It exits
**5** (pytest's "no tests collected"), not the **4** `CLAUDE.md` records.
Measured; the exit code is the only thing wrong, and it is a pre-existing
documentation nit, left alone by this sprint rather than edited silently.

---

## 5. The M6 regression

One fresh green mission, `full_world_robo.launch.py traverse:=true
gui:=false` then `mission.launch.py rviz:=false target_colour:=green`,
every other argument at its shipped default. `meta.txt` records
`head=aa5b968`, `dirty_paths=0`.

**`state=COMPLETE prev=VERIFY_PLACEMENT result=fetch`.** Duration 21:27:27
→ 21:33:28 UTC.

The C2-NAV.45 gate, both bands, in one run:

```
[WARN] NAVIGATE_TO_RAMP arrived: ground truth 0.290 m from the goal —
       Nav2 reported SUCCESS but ground truth is 0.290 m out, past the
       0.25 m tolerance. Accepted: within the 0.50 m consistency band,
       and the robot has stopped, so re-issuing the goal cannot close it
[INFO] RETURN_HOME arrived: ground truth 0.108 m from the goal
       (tolerance 0.25 m)
```

The outer band was **not** reached and **no retry was issued**, which is
the point of the fix: C2-NAV.44 measured the retry commanding 0.000 m/s.

**One number to record honestly.** 0.290 m is a **fourth** green sample
and it is *below* the 0.315–0.348 m range C2-NAV.45 and C2-NAV.46
measured. The observed green pre-ramp discrepancy is therefore
**0.290–0.348 m over four runs**, not 0.315–0.348 over three. Four runs
is still not a rate.

Command path over the whole mission, 1,527 trace rows, 2,282 wheel
messages:

| metric | value |
|---|---|
| `wheel_matches_raw_controller` | **0** (of 78 bypass rows; **0** of 498 nav-owned rows) |
| `stop_breach.stop_rows` | 0 — the mission never entered a monitor STOP |
| smoother | 90 of 109 rows where raw ≠ smoothed followed **smoothed**; 1 raw-only |
| monitor actions | SLOWDOWN 18 rows, LIMIT 5 rows, and the wheels obeyed |
| `gated_zero_moving` | 3 of 498 nav rows (0.60 %), worst wheel 0.0158 m/s |

That last row is the **known, already-documented** short-streak residual
(`CLAUDE.md`: 0.088–2.92 % of trace samples per tour, not attributed).
0.60 % sits inside it. It is not new, and this sprint does not attribute
it either.

---

## 6. The command-path regression, and the gap the mission left

The nominal mission is **not sufficient** for one of the six checks: it
never entered a collision-monitor STOP (`stop_rows` 0), so "the monitor's
STOP reaches the wheels" was untested by it. Saying the mission proved it
would be the "we saw nothing" failure `CLAUDE.md` warns about. So
`c2nav42_cmdpath.py stop` was run once, on a fresh topology-B graph.

The probe turns to face the west wall, then publishes a **constant raw
0.30 m/s** on `/cmd_vel_nav` and keeps publishing it. Nothing but the
safety chain can stop the robot.

| check | measured |
|---|---|
| arbiter is the only wheel publisher | `Publisher count: 1`, `cmd_vel_arbiter`; one subscriber, `diff_drive_controller` |
| raw Nav2 does not reach the wheels | `wheel_matches_raw_controller` **0** |
| smoother output is respected | **9 of 9** rows where raw ≠ smoothed followed smoothed; `wheel_eq_raw_only` **0** |
| collision-monitor output reaches the wheels | `stop_held: true`; **69 STOP rows, 0 with wheels driven** |
| monitor authority | 252 samples, **0 exceeded**; angular 252 samples, **0 exceeded** |
| relay restamping | 506 gated → 624 wheel messages; the wheels were never starved as stale |

The robot stopped with `min_scan_m` **0.342 m**, at x −3.65 against a wall
face at −3.90, while the raw command still said 0.30 m/s. That is the
safety chain holding the wheels against a controller that wants to drive.

---

## 7. Known remaining navigation issue — recorded, not solved

C2-NAV.46's `r2_blue` **`RETURN_FAILED`**: a PolygonStop hold of **595.5
s** on the *return* leg, after a clean pre-ramp gate (0.066 m) and a
successful pick, with AMCL CONSISTENT and bypass 0, depth fusion off. The
same blue lane completed twice.

It is **classified, not diagnosed**, and this sprint deliberately does not
touch it. It is already written up in `PROJECT_STATE.md` and `CLAUDE.md`.
Per the sprint brief: do **not** weaken PolygonStop, alter safety
thresholds, change the blue goal, tune DWB, or open another AMCL
investigation. A future sprint owns it.

Depth fusion also remains a **candidate, not a default**: its stale
obstacle marks are ~3.2× higher and ramp driving is unvalidated
(C2-NAV.43). This sprint re-benchmarked none of it, by instruction, and
verified only that it builds and stays off.
