# C2-NAV.45 — the pre-ramp arrival gate, fixed and re-measured

Branch `c2nav43-integration`, fix committed at `56c324b`, clean tree. Date
2026-09-18. Every number below was produced in this session, on this branch,
on a **fresh simulator per run**, headless, never `--fast`, depth fusion
**off**, with no Nav2 parameter, goal, planner, controller or
collision-monitor change. Run directories are under
`~/coco_nav_runs/c2nav45_m6/`; the per-run readbacks are committed under
`docs/data/c2nav45_live/`.

**Scope.** This sprint fixes **one** thing: a mission-level arrival-gate
conflict. It does not touch AMCL, LiDAR, depth fusion, DWB, PolygonStop,
planner tuning, goal placement or the command path. See §8 for what is
explicitly *not* claimed.

---

## 1. Verdicts

| | |
|---|---|
| **The defect** | The executive's ground-truth arrival check was set to Nav2's own `xy_goal_tolerance` (0.25 m). Two 0.25 m windows measured from two different poses leave **zero margin**, so a localisation offset in the adverse direction failed a leg Nav2 had already finished — and the retry re-sent the goal to a stopped controller that believed it had arrived. |
| **The fix** | The check keeps both jobs, **split apart and named**: clean arrival, accepted-and-reported discrepancy, and hard failure. The band is **derived** (2× Nav2's own tolerance), not invented. Local to the executive's nav-leg check. |
| **M6, executive-driven, green** | **3 of 3 complete.** Green is the lane that aborted **3 of 3** in C2-NAV.44. All 16 nominal states, `attempts={}`, `reason=--`, **zero** RECOVERY entries in all three. |
| **Original failure reproduced** | **Yes, 3 of 3.** Ground-truth error at the pre-ramp **0.348 / 0.315 / 0.316 m** — outside the 0.25 m tolerance in every run, and bracketing C2-NAV.44's measured 0.3047–0.3115 m. The fix **handles** the scenario; it does not avoid it. |
| **Command path** | **No regression.** Raw-controller → wheel bypass **0**, wheels above the monitor **0**, stale command drops **0**, PolygonStop activations **0**, in all three runs. |
| **Tests** | **997 passed, 0 failed, 0 skipped**, per package, cwd inside each package, on a clean ROS graph. The branch's 975 plus **22** new. Run before and after the live sweep; identical. |

---

## 2. Exact old gate behaviour

`mission_states._check_nav_leg`, before `56c324b`:

```python
error = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
if error > self.plan.xy_tolerance:          # 0.25 m
    return (FAILURE, region_reason, ...)
return SUCCESS
```

One threshold, one verdict. `xy_tolerance` was deliberately set to Nav2's
`xy_goal_tolerance` so as not to invent a second number — which is what
created the zero-margin gate.

**Measured consequence** (C2-NAV.44, and re-derived in this session by running
`docs/data/c2nav45_gate_report.py` over those saved runs):

| | r02 | r05 | r06 |
|---|---|---|---|
| true error at the first stop | **0.3097 m** | **0.3115 m** | **0.3047 m** |
| Nav2's verdict | `Reached the goal!` ×3, `Goal succeeded` ×3 | same | same |
| region failures logged | 7 | 7 | 7 |
| RECOVERY entries | 3 | 3 | 3 |
| `NAVIGATE_TO_RAMP` retries | 2 | 2 | 2 |
| **max wheel speed after the first stop** | **0.000 m/s** | **0.000 m/s** | **0.000 m/s** |
| outcome | ABORT | ABORT | ABORT |

The last row is the one that matters: **the retries could not move the robot.**
Both were spent re-sending an identical goal to a controller already holding
station. That is the same "structurally futile" retry the repo already
documents for the **yaw** axis at `GOAL_YAW_TOLERANCE`, now measured on the
position axis.

---

## 3. Exact new gate behaviour

Three bands, explicit in code and in tests:

```
error <= xy_tolerance    (0.25 m)  -> SUCCESS. Clean arrival. Error logged at INFO.
error <= xy_consistency  (0.50 m)  -> SUCCESS. Recorded in arrival_discrepancy,
                                      logged at WARN naming the disagreement,
                                      and the mission continues.
error >  xy_consistency  (0.50 m)  -> FAILURE, region_reason. Unchanged.
```

**The bound introduces no new number.** `GOAL_XY_CONSISTENCY = 2.0 *
GOAL_XY_TOLERANCE`. Nav2 halts with its *estimate* inside 0.25 m, so a true
error beyond 0.50 m requires the pose Nav2 steered by to be wrong by more than
the entire arrival window — a localisation failure, owned by the C2-M5 health
monitor already checked at the top of `_check_nav_leg`, not a tolerance
disagreement. M6 run 15's 3.4 m divergence sits far outside it; the measured
0.305–0.348 m sits inside.

Setting `xy_consistency == xy_tolerance` restores the old single hard gate
exactly, and a test asserts that.

**Why it is applied in the shared helper, not to the pre-ramp leg alone.** The
mechanism on `RETURN_HOME` is identical — same two 0.25 m windows, same stopped
controller — so restricting the fix to `NAVIGATE_TO_RAMP` would knowingly leave
a futile retry behind on the other leg. Both legs are covered by the one
change; `HOME_POSE_OUT_OF_REGION` still fires past the band.

**Precedent followed.** `GOAL_YAW_TOLERANCE`'s comment block already says:
*report the number, do not assert a threshold nobody measured.* The position
axis now does the same, with the one difference that an unbounded position
error is a real failure mode, so the outer band stays.

---

## 4. Files changed

| file | change |
|---|---|
| `coco_mission/scripts/mission_states.py` | `GOAL_XY_CONSISTENCY` + its derivation comment; `MissionPlan.xy_consistency` (clamped never below `xy_tolerance`); `MissionMachine.arrival_error` / `.arrival_discrepancy`; the three-band logic in `_check_nav_leg`, with the per-tick clear that keeps a recorded arrival meaning *this* attempt |
| `coco_mission/scripts/mission_executive.py` | `_log_event` reports the ground-truth error on every nav leg, and **warns explicitly** when Nav2 SUCCESS and ground truth disagree |
| `coco_mission/test/test_mission_states.py` | `TestArrivalConsistency`, 22 tests |
| `docs/data/c2nav45_gate_report.py` | offline per-run gate report (no ROS); reproduces C2-NAV.44's numbers from its saved runs |
| `docs/data/c2nav45_m6_sweep.sh` | the three-run green sweep driver |
| `docs/data/c2nav45_live/` | per-run readbacks, executive logs, command-path summaries |

**Not touched:** Nav2 parameters (`nav2_params.yaml` byte-identical), goals,
planner, controller, velocity smoother, collision monitor, `cmd_vel_arbiter`,
the command path, the yaw gate, the lane gate, the on-the-flat gate, the climb
or descent, perception, the arm.

---

## 5. Tests

`TestArrivalConsistency`, 22 tests, covering every case the policy has:

- **the bound itself** — derived as 2× Nav2's tolerance; never tighter than
  `xy_tolerance`; the old single hard gate still reachable by configuration;
- **A, inside tolerance** — advances, no discrepancy recorded, error still
  recorded;
- **B, the measured discrepancy** — parametrised over r02/r05/r06's **recorded
  ground-truth stop points**, asserting each is outside 0.25 m and inside
  0.50 m, accepted, recorded, and that `RECOVERY` is never entered; plus that
  the mission reaches `CLIMB` from there;
- **C, past the band** — still `PRE_RAMP_POSE_OUT_OF_REGION`; the run-15-class
  3.4 m divergence still caught; the band edge inclusive on the accepting side;
- **D, Nav2 did not succeed** — an ABORTED or REJECTED leg with ground truth
  40 mm from the goal is still a failure, and records **no** arrival; a
  degraded leg records none either;
- **E, no futile retries** — exactly **one** pre-ramp goal is issued after an
  accepted arrival, `attempts` stays 0, and a full fetch completes from a
  discrepant arrival;
- **F, the yaw gate unchanged** — still off by default, still reported at
  +0.28 rad, still gates when switched on; and the **lane** gate still rejects
  an off-lane arrival that the distance band accepted.

Nothing depends on simulator timing: the machine is pure and the harness is a
scripted world.

**Full sweep**, per package, cwd inside each package, clean ROS graph,
`gazebo_models` with `--ignore=test_integration`, MoveIt prefix on the path:

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
| **total** | **997 / 0 failed / 0 skipped** |

975 (the branch's C2-NAV.43 baseline) + 22 new = 997. Measured twice, before
and after the live sweep, identical both times.

---

## 6. Three fresh M6 runs

Shipping procedure, unchanged from C2-NAV.44, one fresh simulator per run,
torn down by process name between runs:

```bash
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
ros2 launch coco_mission mission.launch.py rviz:=false target_colour:=green
ros2 service call /mission/start std_srvs/srv/Trigger
```

Runner: `docs/data/c2nav44_m6_run.sh`, **unmodified**, so the two sprints are
directly comparable. Driver: `docs/data/c2nav45_m6_sweep.sh`. All three runs
recorded `head=56c324b`, `dirty_paths=0`.

**Green in all three**, because green is the lane C2-NAV.44 measured aborting
3 of 3. Run count declared before running: 3. Nothing re-run, nothing discarded.

| | r01 | r02 | r03 |
|---|---|---|---|
| **mission result** | **COMPLETE, `fetch`** | **COMPLETE, `fetch`** | **COMPLETE, `fetch`** |
| `reason` / `attempts` | `--` / `{}` | `--` / `{}` | `--` / `{}` |
| state progression | all 16 nominal | all 16 nominal | all 16 nominal |
| RECOVERY entries | **0** | **0** | **0** |
| relocalizations | **0** | **0** | **0** |
| **pre-ramp GT error** | **0.348 m** | **0.315 m** | **0.316 m** |
| pre-ramp gate decision | accepted + WARN | accepted + WARN | accepted + WARN |
| `PRE_RAMP_POSE_OUT_OF_REGION` | **0** | **0** | **0** |
| pre-climb heading | −0.160 rad | −0.209 rad | −0.132 rad |
| climb | reached `VERIFY_CLIMB` | same | same |
| grasp | `pick finished: held` | same | same |
| **lift (ground truth)** | **36.0 mm** | **35.4 mm** | **34.9 mm** |
| home GT error | 0.070 m | 0.038 m | 0.022 m |
| home gate decision | clean (INFO) | clean (INFO) | clean (INFO) |
| place | `place finished: placed` | same | same |
| wall s, start → terminal | 825 | 450 | 424 |
| runner checks | 22 PASS / 0 FAIL | 22 / 0 | 22 / 0 |

Lifts 34.9–36.0 mm sit inside C2-NAV.44's completion band (34.6–35.6 mm) and
adjacent to it; the grasp was physically verified from Gazebo ground truth in
every run (`Target lifted … — the grasp is real`).

### Localization health

| | r01 | r02 | r03 |
|---|---|---|---|
| samples | 3206 | 1571 | 1540 |
| `degraded=1` | **0** | **0** | **0** |
| CONSISTENT / UNKNOWN | 2610 / 594 | 914 / 655 | 940 / 600 |

UNKNOWN is the off-map platform phase, where the scan metric is gated by
design. **No run degraded, and no run relocalized** — confirming the aborts
this sprint fixed were never a localization failure.

### Command path and safety

C2-NAV.41's `monitor_authority`, `bypass_source` and `stop_breach`, imported
unchanged, on the recorder's 10 Hz trace; the headline row is the Nav2-owned
subset (arbiter `active=nav`, 1.0 s switch guard).

| | r01 | r02 | r03 |
|---|---|---|---|
| Nav2-owned rows | 2209 | 642 | 612 |
| **wheels above the monitor** | **0 / 2209** | **0 / 642** | **0 / 612** |
| **raw-controller → wheel bypass rows** | **0** | **0** | **0** |
| wheel == raw controller | **0** | **0** | **0** |
| **PolygonStop STOP rows** | **0** | **0** | **0** |
| SLOWDOWN rows | 1047 | 156 | 87 |
| **stale command drops** | **0** | **0** | **0** |
| topology B links | OK (only the pre-start `arbiter mode` mismatch) | same | same |
| depth fusion | off, proven | off, proven | off, proven |
| live nav2 param mismatches | 0 | 0 | 0 |

No run tested a STOP hold (PolygonStop never activated), so this sprint adds
**no** new evidence about STOP beyond C2-NAV.43's controlled test, and claims
none.

---

## 7. Retry behaviour, before and after

| | before (C2-NAV.44, green) | after (C2-NAV.45, green) |
|---|---|---|
| region failures per run | 7 | **0** |
| RECOVERY entries per run | 3 | **0** |
| `NAVIGATE_TO_RAMP` retries | 2 | **0** |
| goals re-sent to a stopped controller | 2 | **0** |
| wheel speed after the first stop | 0.000 m/s | n/a — no retry issued |
| outcome | ABORT ×3 | **COMPLETE ×3** |

The "before" column was **re-derived in this session** by running
`docs/data/c2nav45_gate_report.py` over C2-NAV.44's committed run directories,
not copied from its report. It reproduces that sprint's published numbers
exactly, which is also the tool's validation.

---

## 8. What this does NOT claim

Stated plainly, because the temptation runs the other way:

- **AMCL is not fixed.** Nothing in AMCL changed. Its estimate still leads on
  the green lane and lags on the other three; why the sign is lane-dependent
  is still not diagnosed.
- **Localization is not perfect.** The C2-M5 limitation stands: severe
  confident divergence is detected but not reliably recovered to a
  Nav2-plannable pose.
- **The enclosure problem is not fixed.** Untouched.
- **Depth fusion is not validated.** Off in every run here, still a candidate.
- **The collision monitor's short-streak residual is not attributed.** These
  three runs happened to show 0 rows above the monitor; that is three runs, not
  a refutation of the 0.088–2.92 % residual C2-NAV.43 recorded.
- **Three runs is not a rate.** One colour, three fresh simulators. It is
  enough to say the demonstrated abort no longer occurs and the mission
  completes; it is not enough to quote an M6 success percentage.
- **19/20 is still not a like-for-like comparison.** That matrix ran
  `traverse_demo.py`, which has no ground-truth arrival gate at all.

What **is** claimed, exactly: **a mission-level arrival-gate conflict was
found, fixed locally, and the three green missions it used to abort now
complete.**

---

## 9. Reproducing

```bash
# the three-run green sweep (fresh simulator each, ~15 min per run)
bash docs/data/c2nav45_m6_sweep.sh ~/coco_nav_runs/c2nav45_m6

# what the gate decided, per run, offline and without ROS
python3 -P docs/data/c2nav45_gate_report.py ~/coco_nav_runs/c2nav45_m6/r0*_green

# the same tool over C2-NAV.44's runs, showing the old behaviour
python3 -P docs/data/c2nav45_gate_report.py ~/coco_nav_runs/c2nav44_m6/r0{2,5}_green

# the tests
cd coco_mission && python3 -m pytest test/test_mission_states.py -k ArrivalConsistency
```
