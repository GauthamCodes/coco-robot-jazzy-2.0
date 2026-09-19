# C2-NAV.48 — the blue return-leg PolygonStop deadlock, diagnosed and fixed

Branch `c2nav48-return-deadlock`, off `main` at `1425e6c`. Date 2026-09-19.
Every number below was produced in this session unless it is explicitly
attributed to an earlier sprint's measurement. Fresh simulator per run,
headless, never `--fast`, depth fusion **off**.

C2-NAV.46 left one classified-but-undiagnosed failure: `r2_blue`,
`RETURN_FAILED`, a PolygonStop hold of **595.5 s** on the return leg *after* a
successful pick, with AMCL CONSISTENT, a clean pre-ramp gate at 0.066 m, and
command-path bypass 0. This sprint diagnoses it to a named obstacle and a
specific parameter disagreement, and fixes it without touching the collision
monitor.

---

## 1. Verdicts

| | |
|---|---|
| **Root cause** | **FOUND and measured.** The local costmap and the collision monitor disagreed about which poses are navigable. `cylinder_obstacle`'s surface sat **0.2486 m** from `base_footprint` — 1.4 mm *inside* PolygonStop's 0.25 m circle and 43 mm *outside* the costmap's real inscribed radius of 0.2060 m. The planner scored the pose 15.8 of 254; the monitor treated it as a stop. |
| **Why nothing escaped** | **Measured, not inferred.** PolygonStop is `type: circle`, `action_type: stop` — direction-agnostic. During the hold Nav2 commanded motion in **5423 of 5955** rows, including **905** rows of `spin` (`w=+1.0`) and **603** rows of `backup` (`v=−0.15`). The wheels moved in **0**. |
| **First divergence** | **23.8 mm.** `r3_blue` passed the same obstacle at 0.2724 m and went home; `r2_blue` passed at 0.2486 m and held for 595.5 s. |
| **Fix** | **One parameter.** `local_costmap.robot_radius` 0.20 → 0.25, raising the inflation layer's inscribed radius from 0.205965 m to 0.255004 m — 5.0 mm past the stop circle. `PolygonStop` is untouched, and the readback proves it. |
| **Reproduction on unmodified runtime** | **NOT obtained.** See §7. This is the sprint's one evidential limitation, and the root cause does not rest on it. |
| **Validation** | **3 valid fresh blue runs, 3/3 `result=fetch`.** Plus red, green and yellow smoke, **3/3 `result=fetch`**. PolygonStop rows **0** in all six. |
| **Regression** | Nav2 works strictly *less* hard: `controller_failed_progress` **0** in all six, against 40 in `r2_blue`. Goals aborted 0. RECOVERY entries 0. |
| **Tests** | **1001 passed, 0 failed, 0 skipped** (997 baseline + 4 new). |
| **Clean build** | 9 of 9 packages, exit 0. |
| **Orphans** | 0, and all seven run directories record `ros_clean: 0 matched, 0 still running`. |

---

## 2. The obstacle, identified to 0.9 mm

Deadlock ground truth: `x=0.1698, y=0.3460, yaw=−2.3208 rad (−132.97°)`, frozen
bit-for-bit for all 5955 held rows — the robot was genuinely at rest, not a
stalled trace.

`cylinder_obstacle` is a model in the frozen `coco_world.world`: centre
**(−0.2, 0.6)**, radius **0.2 m**, height **0.6 m**. The LiDAR plane sits at
z = 0.20 m, so the cylinder spans it. A real obstacle, correctly sensed.

`LIDAR_MOUNT_XYZ = (-0.09, 0.10, 0.20)` — the sensor is 9 cm behind and 10 cm
left of `base_footprint`, an offset of **0.1345 m**. That offset is why the
trace's `scan_min` reads *larger* than the stop radius while the stop is
legitimately firing, and it is the check that identifies the obstacle:

| quantity | value |
|---|---|
| laser world position at the deadlock | (0.3043, 0.3437) |
| laser → cylinder surface, **predicted** | **0.3657 m** |
| `scan_min` in the trace, **observed** | **0.3666 m** |
| **agreement** | **0.9 mm** |
| `base_footprint` → cylinder surface | **0.2486 m** |
| PolygonStop circle | 0.25 m → **inside by 1.4 mm** |
| true chassis clearance (hull at 0.20 m) | **49 mm — never in contact** |

Answering TASK B and TASK C directly:

1. **Which obstacle?** `cylinder_obstacle`, identified to 0.9 mm.
2. **Present in true world geometry?** Yes — a static model in the frozen world.
3. **Was the scan representation correct?** Yes. Predicted range matched
   observed to 0.9 mm.
4. **Was the local costmap correct?** Yes, *as configured*. It faithfully
   reported a pose 43 mm outside its own inscribed radius. The configuration
   was wrong, not the representation.
5. **Could the robot physically move safely?** Yes — 49 mm of hull clearance.
6. **Was the controller trying to move into the region?** It was trying to move
   *at all*: forward, reverse and rotation were all commanded and all vetoed.
7. **Did the stop persist because the robot could not change its geometry?**
   Yes. A circular stop has no escape direction, so the state is absorbing.
8. **Stale sensor or costmap data?** No. `scan_min` was live and correct
   throughout; 38 costmap clears did not help because the obstacle was real.

---

## 3. Why the planner drove there

The costmap's real threshold is **not** `robot_radius`. `Costmap2DROS` builds a
16-gon of circumradius `robot_radius`, pads it by `footprint_padding`
(default 0.01), and `LayeredCostmap` takes the apothem. **C2-NAV.0 measured**
that as **0.205879 m** for `robot_radius` 0.20 — only that value reproduces all
34 distinct inflated costs in its captured grid, and it warned that "anything
reasoning about this costmap from `robot_radius` is a few per cent wrong in
every cell."

This sprint's model, `(robot_radius + padding) · cos(π/16)`, gives 0.205965 m —
**0.086 mm** from C2-NAV.0's measurement. A test now guards that agreement so
the invariant cannot come to rest on arithmetic that stopped matching.

So the band **0.2059 … 0.25 m**, **44 mm wide**, was legal to the planner and
terminal to the monitor. And the local costmap's `cost_scaling_factor` of
**65.0** collapses the gradient so fast that the band is not merely legal but
*cheap*:

| clearance | global cost (csf 5.0) | local cost (csf 65.0) |
|---|---|---|
| **0.2486 m** — the deadlock pose | **203.6** of 254 | **15.8** of 254 |
| 0.2724 m — `r3_blue` | 180.8 | 3.4 |
| 0.30 m | 157.5 | 0.6 |

The **global** planner already pays 203.6 to sit where `r2_blue` sat and avoids
the band unaided. The **local** controller prices the identical pose at 15.8 —
indistinguishable from open floor. **The defect is local, so the fix is local,
and the global costmap is deliberately left at 0.20.** That asymmetry is
measured, not assumed, and a test records it.

**Source-verified, not assumed.** `dwb_critics::BaseObstacleCritic::isValidCost`
returns false for `INSCRIBED_INFLATED_OBSTACLE` (253), and `scorePose` then
throws `IllegalTrajectoryException(name_, "Trajectory Hits Obstacle")` — so such
a trajectory is *rejected*, not merely penalised. `BaseObstacleCritic` scores
the **centre** cell, which is the same `base_footprint` origin PolygonStop
measures its circle from. The two mechanisms share a reference point exactly,
which is what makes the invariant in §4 well posed.

---

## 4. The fix

```
local_costmap.robot_radius   0.20 -> 0.25
```

Inscribed radius 0.205965 → **0.255004 m**, **5.0 mm past** the 0.25 m stop
circle. `r2_blue`'s 0.2486 m pose becomes cost 253, so DWB throws it out
instead of scoring it 15.8.

**The invariant, stated once:** no pose the planner will accept may lie inside
the collision monitor's stop circle.

**What this deliberately does not do.** It does not move `PolygonStop.radius`,
`min_points`, or any collision-monitor semantic; it does not touch the command
path, the goals, planner/controller tuning, Nav2's goal tolerances, or the
depth-fusion default. C2-NAV.6 ruled for this exact failure class that
"**neither `PolygonStop` knob should move**", and C2-NAV.7 fixed the
`enclosure_entry` instance by a goal stand-off leaving "nearest geometry past
0.25 m from the base origin". This is that same criterion, moved from the *goal*
poses a stand-off can reach to the *transit* poses Nav2 chooses for itself,
which a stand-off cannot reach at all.

**Known residual, stated plainly.** Costmap resolution is 0.05 m, so the
rasterised boundary is about one cell accurate. This is a large reduction in
exposure, **not a proof** that the band can never be entered.

### Live readback

`robot_radius` was absent from `nav_params_overlay.py`'s `LIVE_CHECKS`, which is
why this sprint's first three runs could only establish the live value from the
install symlink and the source mtime. Both costmaps are now in the list, and
every run proves what it loaded:

```
OK  /local_costmap/local_costmap   robot_radius            file=0.25 live=0.25
OK  /global_costmap/global_costmap robot_radius            file=0.2  live=0.2
OK  /collision_monitor             PolygonStop.radius      file=0.25 live=0.25
OK  /collision_monitor             PolygonStop.min_points  file=4    live=4
```

That one block simultaneously proves the fix is live, the global costmap is
untouched, and `PolygonStop` is unchanged.

---

## 5. Validation

Six valid missions on `robot_radius` 0.25, fresh simulator each. Clearance is
the closest approach of `base_footprint` to any obstacle surface during
`RETURN_HOME`.

| run | colour | outcome | dur | PolygonStop rows | closest approach | to `cylinder_obstacle` |
|---|---|---|---|---|---|---|
| `c2nav48_repro/r1_blue` | blue | **COMPLETE `fetch`** | 178.8 s | **0** | 0.2905 m | 0.2905 m |
| `c2nav48_repro/r2_blue` | blue | **COMPLETE `fetch`** | 160.8 s | **0** | 0.4949 m | 0.5249 m |
| `c2nav48_valid/r1_blue` | blue | **COMPLETE `fetch`** | 162.8 s | **0** | 0.4577 m | 1.0334 m |
| `c2nav48_valid/r2_red` | red | **COMPLETE `fetch`** | 154.9 s | **0** | 0.4817 m | 1.7614 m |
| `c2nav48_valid/r3_green` | green | **COMPLETE `fetch`** | 159.1 s | **0** | 0.5063 m | 1.7267 m |
| `c2nav48_valid/r4_yellow` | yellow | **COMPLETE `fetch`** | 166.4 s | **0** | 0.4700 m | 1.0649 m |

For comparison, the same measurement on the three C2-NAV.46 blue runs:

| run | outcome | dur | PolygonStop rows | to `cylinder_obstacle` |
|---|---|---|---|---|
| `r1_blue` | COMPLETE | 148.7 s | 0 | 0.9223 m |
| `r2_blue` | **ABORT `RETURN_FAILED`** | 750.7 s | **5955** | **0.2486 m — inside** |
| `r3_blue` | COMPLETE | 236.3 s | 0 | 0.2724 m |

The closest any post-fix run came to that obstacle is **0.2905 m**, which is
35.5 mm outside the new inscribed radius and 40.5 mm outside the stop circle.

### Command path and safety

| metric | C2-NAV.46 `r2`/`r3` | the six post-fix runs |
|---|---|---|
| bypass, wheel == raw controller | 0 / 0 | **0 in all six** |
| bypass rows, nav-owned | 0 / 0 | **0 in all six** |
| PolygonStop rows | 5955 / 0 | **0 in all six** |
| PolygonStop rows with wheels **driven** | 0 / 0 | **0 in all six** |
| `gated_zero_moving`, nav-owned | 38 / 15 | **0 – 34** |
| worst wheel while gated to zero | 0.0189 / 0.0199 m/s | **≤ 0.0199 m/s** |
| monitor exceeded, nav-owned | 0 / 0 | **1 row in two runs, 0 in four** |
| smoother raw-only rows | 0 / 1 | **0 – 6** |

`gated_zero_moving` stays inside the documented, still-unattributed
0.088–2.92 % residual band and is **not** attributed here either. The single
`monitor exceeded` row in two runs and the 0–6 smoother raw-only rows are
reported rather than rounded to zero; they are single-digit counts in traces of
~1600–1800 rows, of the same order as the historical figures, and no
mechanism is claimed for them.

### Mission-level regression

| | C2-NAV.46 `r1`/`r2`/`r3` | the six post-fix runs |
|---|---|---|
| `controller_failed_progress` | 0 / **40** / 5 | **0 in all six** |
| `spin` | 12 / **64** / 11 | 6 – 12 |
| `backup` | 4 / **22** / 4 | 3 – 4 |
| goals aborted | 0 / **2** / 0 | **0 in all six** |
| RECOVERY entries | 0 / **3** / 0 | **0 in all six** |
| no-valid-trajectory | 0 / 0 / 0 | **0 in all six** |

The arrival gate and the grasp were also unaffected: pre-ramp arrivals
0.094 m and 0.108 m, return arrivals 0.062 m and 0.014 m — all clean inside the
original 0.25 m tolerance, **0** WARN-band entries, so C2-NAV.45's outer
0.50 m band was never reached. Grasp base-x 0.1539 and 0.1545, both inside the
measured [0.1510, 0.1565] window.

---

## 6. Decision: **KEEP**

The fix is one parameter, in the sanctioned footprint-representation class,
derived rather than tuned, guarded by four tests, proven loaded from the live
graph, and it leaves the collision monitor and the command path untouched.
Six fresh missions across all four colours complete, PolygonStop never fires,
and Nav2's recovery workload falls.

**What KEEP rests on, honestly.** The deadlock was not reproduced on unmodified
runtime in this session, so this is *not* an A/B against a reproduced failure.
It rests on three things instead: the root cause measured from C2-NAV.46's own
trace (0.9 mm geometric agreement, escape commands counted), the mechanism
verified in Nav2's source, and six post-fix missions with no regression. Six
runs is **not a rate**.

---

## 7. What was NOT achieved

**No reproduction on unmodified runtime.** Two reasons, both recorded rather
than worked around:

1. The base rate is **1 in 12** (C2-NAV.46). Three runs was never likely to
   reproduce it, and the brief said not to manufacture a new experiment if it
   did not.
2. The three runs that executed were **already on the fixed value**. The
   overlay is `--symlink-install`, so the installed `nav2_params.yaml` is a
   symlink to source; source was edited at 22:30:50 UTC and Nav2 loaded
   parameters at ~22:31:36 UTC. `params_live.txt` did not yet check
   `robot_radius`, so this was established from the symlink and the mtime.

The consequence is stated in §6 rather than smoothed over. The root cause
itself does not depend on the new runs.

**One VOID run.** `c2nav48_repro/r3_blue` aborted
`LOCALIZATION_RECOVERY_UNAVAILABLE` because its `nav2_container` died during
bring-up — a SIGSEGV reported by ImageMagick's signal handler
(`Magick: abort due to signal 11`), after which `all Nav2 lifecycle nodes
active` timed out at 300 s. The mission executive correctly detected the dead
stack (`TRANSFORM_STALE` → `LOCALIZATION_DEGRADED`). **Infrastructure, not a
mission failure**, and replaced by `c2nav48_valid/r1_blue` rather than counted.

Its cause was this sprint's own harness: running the runner under `env -i`
produced that abort in **3 of 3** runs where the C2-NAV.46 worktree runs had
**0 of 3**, and it also slowed Nav2 bring-up from 19 s to 2 min 42 s. Twice the
abort landed at teardown, after the mission had completed; once during
bring-up, which voided the run. Isolating only the ROS/colcon variables instead
of stripping the whole environment removed the slow bring-up and left 1 abort
in 4 runs, at teardown, harmless. **Do not run these harnesses under `env -i`.**

---

## 8. The two harness defects this sprint had to fix first

Neither is a product defect; both blocked every live run on `main`.

1. **The run scripts conflated the repo root with the colcon workspace root.**
   `c2nav44_m6_run.sh`, `c2nav44_m6_run_traverse.sh` and `nav_tour_run.sh`
   sourced `$WT/install` and guarded on `"$WT"/install/*`. Correct in the
   worktree they were written in, which carried its own overlay; on `main` the
   overlay is `<ws>/install`, and all three refused to start:

   ```
   m6_run: REFUSING: coco_mission does not resolve into
           <repo>/install
   ```

   The guard was right and the paths were wrong. Worse, a **stale
   `<repo>/install` dated 2026-07-27 does exist and holds exactly one package,
   `coco_rl`**, so the source succeeded and layered a seven-week-old `coco_rl`
   over the fresh build. That refusal is the only reason it did not become a
   silent wrong-overlay run. `WS` is now derived as `setup_env.sh` derives it
   and is overridable by `COCO_WS`, mirroring `COCO_WT`. C2-NAV.47 made the two
   sweep drivers self-locating but left this assumption in the runners beneath
   them.

2. **`<ws>/install` cannot launch Gazebo at all.** It holds half-installed
   `turtlebot3_*` packages whose ament index entries do not resolve, and
   `ros_gz_sim`'s `gz_sim.launch.py` calls `GazeboRosPaths.get_paths()`, which
   enumerates **every** package in the index. One bad entry raises
   `PackageNotFoundError` and the whole launch dies:

   ```
   [ERROR] [launch]: package 'turtlebot3_teleop' not found
   .../ros_gz_sim/launch/gz_sim.launch.py:54 in get_paths
   ```

   Pre-existing and unrelated to this repo — `CLAUDE.md` already records the
   turtlebot3 breakage as such. Measured both ways: `<ws>/install` yields **1**
   unresolvable entry; an isolated overlay of the nine coco packages yields
   **0 of 490**. Runs therefore used `$HOME/c2nav48_overlay`, which is what
   every prior sprint did from its worktree.

---

## 9. Remaining work

Concrete and unresolved:

1. **`<ws>/install` cannot launch a simulator.** Until the half-installed
   `turtlebot3_*` prefixes are repaired or removed, every gz launch from the
   user's own overlay dies in `GazeboRosPaths.get_paths()`. Runs need an
   isolated overlay. This is the user's environment, so it is reported, not
   changed.
2. **The stale `<repo>/install` and `<repo>/build` from 2026-07-27 should go.**
   They hold one stale `coco_rl` and exist only to be sourced by mistake.
3. **`PROJECT_STATE.md` and `CLAUDE.md` still record this deadlock as
   "classified, not diagnosed".** Both need updating. Deliberately **not**
   edited here: the working tree carries an unrelated local `CLAUDE.md`
   modification, and mixing the two would obscure both.
4. **The rasterisation residual.** At 0.05 m resolution the inscribed boundary
   is about one cell accurate. If a pose inside the stop circle is ever
   observed again, the derived next value is `robot_radius` 0.30 (inscribed
   0.3040 m, one full cell past the circle), not a change to `PolygonStop`.
5. **`gated_zero_moving` remains unattributed**, as it has since C2-NAV.42.
   Not touched here.

## Next engineering action

Re-run the full 12-run colour matrix (`c2nav46_matrix_sweep.sh`, three runs per
colour) on `robot_radius` 0.25 against the isolated overlay, to replace
C2-NAV.46's 11-of-12 with a figure measured on the fixed configuration:

```bash
COCO_WS=$HOME/c2nav48_overlay \
  bash docs/data/c2nav46_matrix_sweep.sh ~/coco_nav_runs/c2nav48_matrix
```
