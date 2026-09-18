# C2-NAV.46 — the M6 fetch colour matrix on the fixed arrival gate

Branch `c2nav43-integration`, sweep run at `ff98171`/`e73494d`, clean tree.
Date 2026-09-18. Every number below was produced in this session, on this
branch, on a **fresh simulator per run**, headless, never `--fast`, depth
fusion **off**, with no Nav2 parameter, goal, planner, controller or
collision-monitor change.

Run directories `~/coco_nav_runs/c2nav46_m6/`; per-run readbacks committed
under `docs/data/c2nav46_live/`, with the composed matrix in
`docs/data/c2nav46_live/matrix_report.txt` and `matrix.json`.

**Scope.** C2-NAV.45 fixed the pre-ramp arrival gate and validated it on the
**green** lane, 3 of 3. Three runs of one colour are not a matrix. This
sprint collects three fresh executive-driven missions for each of **red**,
**blue** and **yellow** and reports the four-colour matrix. **It changes no
runtime code.** The C2-NAV.45 green runs are carried over unchanged as the
green sample.

---

## 1. Verdicts

| | |
|---|---|
| **Matrix** | **11 of 12 fetches**, 12 valid runs, **0 void**. red 3/3, green 3/3, blue **2/3**, yellow 3/3. |
| **The gate fix generalises** | Yes. All 12 runs passed the pre-ramp gate; **no run in any lane hit the outer band**, and **zero futile retries** were issued anywhere. |
| **The green discrepancy is lane-specific** | **Measured, and it is not universal.** Green uses the consistency band in all 3 runs (0.315–0.348 m). Red (0.108–0.131 m), blue (0.066–0.085 m) and yellow (0.030–0.047 m) are **clean inside the original 0.25 m tolerance** in all 9 runs. |
| **Command path** | **No regression.** Raw-controller → wheel bypass **0** and stale command drops **0** in **all 12 runs**. |
| **The one failure** | `r2_blue`, **ABORT `RETURN_FAILED`**. A **navigation** failure on the return leg, *after* a successful pick. Not the gate, not the command path, not localisation. Classified in §5. |
| **Tests** | **997 passed, 0 failed, 0 skipped**, measured in this session after the sweep, per package, cwd inside each package, on a clean ROS graph. Unchanged from C2-NAV.45, as expected for a sprint that changes no runtime code. |

---

## 2. The matrix

```
             Run 1   Run 2   Run 3     Result
red          FETCH   FETCH   FETCH     3/3
green        FETCH   FETCH   FETCH     3/3
blue         FETCH   ABORT   FETCH     2/3
yellow       FETCH   FETCH   FETCH     3/3
```

| colour | fetch success | valid | void |
|---|---|---|---|
| red | **3/3** | 3 | 0 |
| green | **3/3** (C2-NAV.45) | 3 | 0 |
| blue | **2/3** | 3 | 0 |
| yellow | **3/3** | 3 | 0 |

| total | |
|---|---|
| fetches completed | **11** |
| valid runs | **12** |
| void runs | **0** |
| recovery entries | **3** (all in `r2_blue`) |
| relocalization count | **0** |
| command-path bypass | **0** |
| wheels above the monitor | **4** of 9744 nav-active samples = **0.0411 %** |
| stale command drops | **0** |
| PolygonStop activations | **1** (`r2_blue`) |

**Green is not re-run here.** Its three runs are C2-NAV.45's, at `56c324b`,
carried over unchanged so the two sprints' numbers stay the same numbers.

---

## 3. Per run

```
run         colour  result    pre-ramp  gate         home  nav2   rec  byp >cm stale poly   lift   base-x  sim s
r1_red      red     COMPLETE     0.124  clean       0.083  2/0      0    0   1     0    0   34.7   0.1538    147
r2_red      red     COMPLETE     0.108  clean       0.104  2/0      0    0   0     0    0   35.6   0.1547    146
r3_red      red     COMPLETE     0.131  clean       0.040  2/0      0    0   1     0    0   34.5   0.1534    147
r01_green   green   COMPLETE     0.348  ACCEPTED    0.070  2/0      0    0   0     0    0   36.0   0.1538    317
r02_green   green   COMPLETE     0.315  ACCEPTED    0.038  2/0      0    0   0     0    0   35.4   0.1547    154
r03_green   green   COMPLETE     0.316  ACCEPTED    0.022  2/0      0    0   0     0    0   34.9   0.1545    151
r1_blue     blue    COMPLETE     0.085  clean       0.087  2/0      0    0   0     0    0   35.2   0.1543    145
r2_blue     blue    ABORT        0.066  clean          --  1/2      3    0   0     0    1   36.2   0.1541    741
r3_blue     blue    COMPLETE     0.069  clean       0.075  2/0      0    0   0     0    0   36.4   0.1542    231
r1_yellow   yellow  COMPLETE     0.047  clean       0.128  2/0      0    0   1     0    0   36.0   0.1544    162
r2_yellow   yellow  COMPLETE     0.041  clean       0.128  2/0      0    0   0     0    0   34.9   0.1536    153
r3_yellow   yellow  COMPLETE     0.030  clean       0.119  2/0      0    0   1     0    0   35.8   0.1542    152
```

Every COMPLETE run walked the **16 nominal states exactly once**,
`attempts={}`, `reason=--`, **zero** RECOVERY entries, and shut the graph down
clean. All 22 runner checks passed in all 12.

**Grasp, across all 12 runs.** Lift **34.5–36.4 mm**; grasp `base-x`
**0.1534–0.1547 m**, every one inside the measured
`[0.1510, 0.1565]` window — **12 of 12**, across four different cylinder
radii (red 20 mm, green 24 mm, blue 28 mm, yellow 32 mm). The pick is not
colour-sensitive.

---

## 4. What the gate did, per colour

The question this sprint exists to answer is whether C2-NAV.45's three bands
behave the same way in lanes it was never tested in. They do, and the
**discrepancy it was built to absorb turns out to be a property of the green
lane, not of the robot**:

| colour | lane y | pre-ramp GT error, n=3 | band used | futile retries |
|---|---|---|---|---|
| red | −0.75 | 0.108–0.131 | clean (≤ 0.25 m) | 0 |
| green | −0.25 | **0.315–0.348** | **consistency (≤ 0.50 m)** | 0 |
| blue | +0.25 | 0.066–0.085 | clean | 0 |
| yellow | +0.75 | 0.030–0.047 | clean | 0 |

Read against the five checks the task set:

1. **Does Nav2 report SUCCESS?** Yes — `2/0` goals succeeded/failed in all
   11 completing runs.
2. **Does the ground-truth gate agree?** In 9 of 12 runs, yes, cleanly. In
   the 3 green runs it disagrees by 0.315–0.348 m.
3. **Is a slight disagreement classified WARN/continue?** Yes, 3 of 3 green
   runs: `DISCREPANCY ACCEPTED (inside the consistency band)`, logged at WARN,
   mission continued.
4. **Are futile retries still zero?** **Yes, 12 of 12.** No run re-sent a
   goal to a stopped controller.
5. **Does the fetch complete?** 11 of 12; the exception is §5.

**The outer band was never reached.** The largest pre-ramp error anywhere is
green's 0.348 m, against a 0.50 m bound. The band is doing its job with
0.15 m of headroom, and no lane needed it widened.

**Note for the record.** Red, blue and yellow would all have passed the *old*
single-threshold gate. Only green would not. The C2-NAV.44 failure was
therefore a green-lane failure that happened to be measured on green, and
C2-NAV.45 fixed a real defect on the lane that exposes it — not a defect
uniformly present in all four.

---

## 5. The one failure: `r2_blue`, ABORT `RETURN_FAILED`

This is a **valid run that failed**, not a VOID: 22 runner checks passed,
no process died, the graph shut down clean. It is reported, not discarded.

**What happened**, from the executive's own log:

```
NAVIGATE_TO_RAMP arrived: ground truth 0.066 m from the goal (tolerance 0.25 m)
...
RETURN_HOME -> RECOVERY [RETURN_TIMEOUT]: no completion in 240s
RECOVERY -> RETURN_HOME [RETURN_TIMEOUT]: retry 1/2 after RETURN_TIMEOUT
RETURN_HOME -> RECOVERY [RETURN_FAILED]: action aborted
RECOVERY -> RETURN_HOME [RETURN_FAILED]: retry 2/2 after RETURN_FAILED
RETURN_HOME -> RECOVERY [RETURN_FAILED]: action aborted
RECOVERY -> ABORT [RETURN_FAILED]: RETURN_HOME exhausted its retries
MISSION ABORT: result=aborted reason=RETURN_FAILED attempts={'RETURN_HOME': 2}
```

**The fetch itself succeeded.** It navigated, climbed, found and grasped the
cylinder (**lift 36.2 mm, base-x 0.1541 m** — both nominal), verified the
grasp and descended. The mission failed **on the way home, carrying the
object**.

### Classification: **navigation**

Ruled out by measurement, not by assertion:

| candidate | evidence against |
|---|---|
| **mission logic** (the gate, a futile retry) | Pre-ramp gate was **clean at 0.066 m**. And **max wheel speed 0.394 m/s** — these retries *drove the robot*, unlike C2-NAV.44's, which commanded **0.000 m/s**. The executive re-drove a leg that genuinely had not finished. Correct behaviour. |
| **localization** | AMCL health **CONSISTENT for 6873 of 7486 samples, 0 degraded**; final AMCL-vs-truth gap **0.113 m**. Not a divergence. |
| **command path** | bypass **0**, wheels above the monitor **0**, stale drops **0**. The robot obeyed the monitor throughout. |
| **object / lane geometry** | The pick succeeded, in the same lane that completed cleanly in `r1_blue` and `r3_blue`. |
| **infrastructure** | 22/22 checks, no process died, clean shutdown. |

What remains, and what the trace shows directly:

- **PolygonStop was active for 595.5 s**, 5920 of those 10 Hz rows inside
  `RETURN_HOME`. The collision monitor held the robot stopped for ~10
  minutes of a 741 s mission.
- `min_scan_m` **0.15 m** — it was hard against something.
- Nav2 worked the problem and could not free it: **40** `controller_failed_progress`,
  **38** costmap clears, `spin` ×9, `wait` ×9, `backup` ×6, and **2** aborted goals.
- `dwb_no_valid_trajectories` **0** — this is *not* the DWB degeneracy of C2-NAV.21.
- It finished at world **(0.17, 0.35)**, just past the ramp base, never
  reaching home at (−2.0, 0.0).

So: **the robot descended and then got itself into a pose where the collision
monitor's stop polygon was continuously violated, and Nav2's recovery
behaviours could not extract it.** That is the documented
`enclosure_entry`/PolygonStop deadlock class from C2-NAV.8–11, on the return
leg. It is **intermittent** — the same lane completed cleanly in the two
other blue runs, with PolygonStop **0** in both, and in the other nine runs.

**This is a pre-existing navigation limitation, not a C2-NAV.45 regression.**
Nothing in this sprint's changes touches it, and the safety chain behaved
exactly as designed throughout: the monitor stopped the robot, and the wheels
obeyed the monitor.

---

## 6. Command-path safety, all 12 runs

| | |
|---|---|
| raw-controller → wheel bypass | **0** in every run |
| stale command drops | **0** in every run |
| PolygonStop activations | **0** in 11 runs; **1** in `r2_blue` (§5) |
| wheels above the monitor | **4 of 9744** nav-active samples = **0.0411 %**, worst gap **0.0316 m/s**, in 4 separate runs (`r1_red`, `r3_red`, `r1_yellow`, `r3_yellow`), 1 sample each |

The last row is the **known, live, unattributed short-streak residual** that
`CLAUDE.md` records at 0.088–2.92 % of trace samples per tour. Measured here
at **0.0411 %** over twelve missions. **It is not claimed fixed and it is not
attributed** — this sprint changed nothing that would explain it. It is
recorded because it was measured.

---

## 7. What this does and does not establish

**Established.**

- The C2-NAV.45 arrival gate behaves correctly in all four lanes, with the
  outer band never reached and zero futile retries in 12 runs.
- The pre-ramp discrepancy is **specific to the green lane** and reproduces
  there to within 33 mm across three runs.
- The command path shows no regression in any colour: bypass 0, stale 0.
- The grasp is colour-independent: 12/12 inside the measured window across
  four cylinder radii.

**Not established, and not claimed.**

- **Three runs per colour is a small sample.** blue 2/3 is *one* failure; it
  does not establish a blue-specific failure *rate*, and the 11/12 aggregate
  should not be read as a 92 % reliability figure.
- The `r2_blue` PolygonStop deadlock is **classified, not diagnosed**. Its
  root cause — why that pose, why unrecoverable — is not established here,
  and no fix is proposed.
- The wheels-above-monitor residual remains **unattributed**.
- Depth fusion stayed **off** and is not mixed into this matrix. Its open
  issue (stale costmap marks) and its ramp-driving effect remain unmeasured.

---

## 7a. Tests, as measured after the sweep

Run after the twelve missions, on a clean ROS graph, per package with cwd
**inside** each package, MoveIt user prefix on the path, `gazebo_models` with
`--ignore=test_integration`:

| package | passed | failed | skipped |
|---|---|---|---|
| `coco_config` | 70 | 0 | 0 |
| `custom_teleop` | 75 | 0 | 0 |
| `coco_rl` | 164 | 0 | 0 |
| `coco_perception` | 139 | 0 | 0 |
| `gazebo_models` | 171 | 0 | 0 |
| `coco_moveit_config` | 12 | 0 | 0 |
| `coco_sim` | 55 | 0 | 0 |
| `coco_mission` | 311 | 0 | 0 |
| **total** | **997** | **0** | **0** |

The **total matches** `CLAUDE.md`'s branch figure of 997 exactly. The
**per-package split does not match** the table in `CLAUDE.md`, which is the
**release-tree** baseline totalling 829 — a different tree. The difference is
concentrated in `gazebo_models` (171 here vs 41 there) and `coco_mission`
(311 vs 281). This was **not** investigated in this sprint; it is recorded
because it was measured, and it is flagged as the one number here whose
per-package provenance is not fully accounted for.

`coco_moveit_config` reporting 12 (not 5) confirms the MoveIt prefix was on
the path and the 7 `test_pick_poses` tests ran rather than skipping.

---

## 8. Files

| file | what |
|---|---|
| `docs/data/c2nav46_matrix_sweep.sh` | the nine-run sweep driver, colours interleaved by round |
| `docs/data/c2nav46_matrix_report.py` | the matrix composer over the two existing per-run reports |
| `docs/data/c2nav46_live/` | per-run readbacks, executive logs, command-path summaries |
| `docs/data/c2nav46_live/matrix_report.txt` | the composed matrix, as printed |
| `docs/data/c2nav46_live/matrix.json` | the same, machine-readable |

**No runtime source file was changed in this sprint.** `nav2_params.yaml` is
byte-identical across all twelve runs (sha256 `6f61e499…`), and every run
recorded `dirty_paths=0`.

---

## 9. Verdict

**The M6 executive path is REGRESSION-PASSED under the fixed command
architecture**, with one recorded navigation failure that is not attributable
to the architecture.

All four colours completed their required fresh runs. No systematic mission
regression appears in any lane: the gate generalises, futile retries are zero
everywhere, the command path is clean in all twelve runs, and the single
failure is an intermittent, pre-existing navigation deadlock on the return
leg whose cause lies outside anything C2-NAV.43/44/45 changed.
