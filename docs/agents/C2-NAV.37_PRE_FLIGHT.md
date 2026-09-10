# C2-NAV.37 — pre-flight checklist for the live odometry-input capture

**Agent:** independent pre-flight audit of C2-NAV.36 (`coco_nav_diag`,
commit `ef5f50c`). No AMCL/costmap/controller parameter, `PolygonStop`/
`PolygonSlow`, goal tolerance, map, benchmark threshold, or navigation
launch file was changed by this task. One correctness blocker was found
and fixed (see SS1) — everything else in this document is a checklist,
not a behavior change.

This label ("C2-NAV.37") does not appear anywhere else in the repo
before this file; it names the live capture that C2-NAV.35/.36 already
called "the next, separately authorized live experiment."

## 0. Objective

Answer, for the first time with real data: does the `odom ->
base_footprint` transform AMCL's motion model actually consumes carry a
repeatable, signed along-track bias, of a sign and magnitude compatible
with the already-measured ~0.09-0.13 m southward `/amcl_pose`-vs-ground-truth
bias (C2-NAV.28-34)? This does not by itself prove AMCL's overall
localization bias is *caused* by odometry — see SS6's evidence rules.

## 1. Blocker found and fixed during this audit

`coco_nav_diag/src/main.cpp` included the upstream
`nav2_amcl/amcl_node.hpp` and instantiated `nav2_amcl::AmclNode` (the
plain, uninstrumented class) instead of this package's own
`coco_nav_diag::AmclNode` fork. It built and linked without error
because `nav2_amcl` (a listed dependency) exports the real
`AmclNode` from its own installed library. `diag_enabled` /
`diag_output_path` / `diag_max_events` are declared only by
`coco_nav_diag::AmclNode`'s constructor — `nav2_amcl::AmclNode` has none
of them — so running `amcl_diag` with `-p diag_enabled:=true` would have
silently produced ordinary, uninstrumented AMCL and never written
`diag.jsonl`. No existing test could have caught this:
`test_diag_recorder.cpp` deliberately never constructs `AmclNode`
(tests `DiagRecorder` in isolation), and no live run had ever been
performed.

**Fix applied** (two lines, `coco_nav_diag/src/main.cpp`):
`#include "nav2_amcl/amcl_node.hpp"` -> `#include
"coco_nav_diag/amcl_node.hpp"`; `nav2_amcl::AmclNode` ->
`coco_nav_diag::AmclNode`.

**Regression coverage added:** `coco_nav_diag/test/test_amcl_diag_node.cpp`
(new gtest, wired into `CMakeLists.txt`) constructs the real
`coco_nav_diag::AmclNode` and asserts `has_parameter("diag_enabled")` /
`"diag_output_path"` / `"diag_max_events"` are present with their
documented defaults (`false`, `""`, `200000`). Needs no map, TF,
simulator, or lifecycle activation, since these parameters are declared
unconditionally in the constructor.

**Verified this session** (`colcon build`/`colcon test`, standalone
against `/opt/ros/jazzy` only, matching how C2-NAV.36's own tests were
run):
- `colcon build --packages-select coco_nav_diag`: succeeds.
- `colcon test --packages-select coco_nav_diag` -> `ctest`:
  **6/6 test executables pass** (`test_diag_recorder`,
  `test_amcl_diag_node` [new], `cppcheck`, `lint_cmake`, `uncrustify`,
  `xmllint` — the same aggregation C2-NAV.36's own "32/32" used;
  `colcon test` reports per-executable, not per-assertion, counts here).
- `python3 -P docs/data/c2nav36_diag.py selftest`: still **19/19** —
  untouched by this fix.
- **End-to-end smoke check, actually run this session:** the built
  `amcl_diag` binary was started standalone (no map, no TF, no
  simulator) with `-p diag_enabled:=false`; `ros2 node list` showed
  `/amcl`, and `ros2 param list /amcl` listed `diag_enabled`,
  `diag_max_events`, `diag_output_path` alongside AMCL's normal
  parameter set. The process was then stopped and verified gone
  (`pgrep -af 'coco_nav_diag/lib/coco_nav_diag/amcl_diag'` empty). This
  confirms the fix end-to-end, not just at the class-construction level.

**Not fixed, deliberately:** the `ros_clean.sh` coverage gap for
`amcl_diag` (SS4), and the `nav2_lifecycle_manager` bond-monitoring
UNKNOWN (SS3) — both are handled procedurally below, not by code
changes, per this task's "otherwise leave the implementation untouched"
instruction.

## 2. Exact executable / configuration

| | |
|---|---|
| Diagnostic executable | `coco_nav_diag`'s `amcl_diag` (built into the **main workspace** for the real run: `cd ~/ros2_ws && colcon build --symlink-install --packages-select coco_nav_diag`, then `source setup_env.sh` as normal — do not use a scratch standalone build for the live run, that was only for this audit's isolated verification) |
| Params file | `gazebo_models/config/nav2_params.yaml` — **unmodified**, same file the normal `amcl` node uses |
| Map / world | `gazebo_models/maps/coco_world.yaml`, `gazebo_models/worlds/coco_world.world` — the existing frozen map, unchanged |
| Benchmark route | `gazebo_models/scripts/nav_bench.py`'s existing `TOUR` (the "21-leg" route), run with **no `--goal` override** so it stays byte-identical to prior C2-NAV runs. Includes `wall_adjacent` (goal `(-2.00,-3.00)`) and `open_space` as legs within the same continuous run. |
| Ground truth | `/model/coco/odometry` (gz `OdometryPublisher`, `world -> base_footprint`, 50 Hz, no TF broadcast) via `docs/data/c2nav36_gt_sidecar.py --out <path>/gt.csv` |
| Do **not** use | `.navbench/` (untracked, predates `coco_nav_diag`, an earlier/separate scratch track — not the frozen configuration) |

## 3. Exact launch sequence

1. `cd ~/ros2_ws && colcon build --symlink-install --packages-select coco_nav_diag` (after this fix is merged/present).
2. Bring up the simulator and full mission stack exactly as normal for a
   benchmark run (fresh Gazebo — mandatory per repo hygiene rule: never
   reuse a simulator across runs). This starts `map_server`, `amcl`,
   controllers, everything else, completely unchanged.
3. Identify the stock `amcl` process precisely, by its **installed
   path**, not a loose name match (a loose `amcl` pattern also matches
   `amcl_diag` and, in some shells, the checking command's own argument
   text — hit and confirmed during this audit's own cleanup check):
   ```bash
   pgrep -af 'lib/nav2_amcl/amcl' | grep -v pgrep
   ros2 node list | grep -x /amcl
   ```
4. Stop only that process (prefer `kill <pid>`, i.e. SIGTERM, over
   `pkill` with a loose pattern).
5. **STOP CONDITION:** run `ros2 node list` again immediately. If
   `map_server` is also gone, or a new `map_server`/`amcl` appeared
   (i.e. `lifecycle_manager_localization` reset/restarted the group),
   **abort** — `nav2_params.yaml` does not override
   `lifecycle_manager_localization`'s `bond_timeout`, and whether its
   bond-based liveliness monitor treats a deliberate `amcl` kill as a
   node failure and resets the whole managed group (including
   `map_server`) is an **UNKNOWN**, not verified from source on this
   machine. Do not proceed past this point if it fires; the
   substitution has diverged from a clean single-node swap and any
   capture taken after would not be comparable to prior C2-NAV runs.
6. Start the diagnostic node in place of the killed one:
   ```bash
   ros2 run coco_nav_diag amcl_diag --ros-args \
     --params-file <path-to-repo>/gazebo_models/config/nav2_params.yaml \
     -p diag_enabled:=<true|false> \
     -p diag_output_path:=<run_dir>/diag.jsonl \
     -p use_sim_time:=true
   ```
7. Replay the lifecycle transitions the bringup manager already did for
   the original process:
   ```bash
   ros2 lifecycle set /amcl configure
   ros2 lifecycle set /amcl activate
   ```
8. **Pre-run check, mandatory:** confirm the recorder is actually armed
   before driving any goal:
   ```bash
   ros2 param get /amcl diag_enabled        # must read True
   ros2 param get /amcl diag_output_path    # must read the intended path, non-empty
   ```
   (An empty `diag_output_path` is a documented, deliberate no-op —
   capture stays disabled even with `diag_enabled:=true`. Confirming
   both, not just one, is the point.)
9. In a second terminal, start the ground-truth sidecar:
   ```bash
   python3 -P docs/data/c2nav36_gt_sidecar.py --out <run_dir>/gt.csv
   ```
10. Drive `nav_bench.py`'s existing `TOUR` to completion, unmodified.
11. Stop `amcl_diag` and the GT sidecar (`Ctrl-C`, allow the
    `DiagRecorder` background writer thread to flush on destruction —
    watch for the `C2-NAV.36 diagnostic capture dropped N events`
    warning at shutdown; `N` should be `0`).
12. Tear down with `ros_clean.sh`, **then** an explicit manual check
    `ros_clean.sh` does not cover:
    ```bash
    pgrep -af 'coco_nav_diag/lib/coco_nav_diag/amcl_diag' | grep -v pgrep
    ```
    If anything remains, kill it by PID before starting the next run —
    an orphaned `amcl_diag` colliding with the next run's freshly
    launched stock `amcl` is the same double-publisher hazard this
    repo has already paid for once (`mission_hud`).

## 4. Output locations

Per run, under a per-run directory (e.g. `<run_dir> =
~/coco_nav_runs/c2nav37_run<N>/`):
- `diag.jsonl` — `amcl_diag`'s odom-input capture.
- `gt.csv` — `c2nav36_gt_sidecar.py`'s ground truth.
- `joined.csv` — from SS7's offline join.
- Console logs from `amcl_diag`, the GT sidecar, and `nav_bench.py`
  (redirect, don't rely on scrollback).

## 5. The bounded capture: 5 live runs, not invented from scratch

`HANDOFF.md §9` (C2-NAV.35) already specifies the intended follow-up:
"re-run the same 21-leg/4-run route with the hook enabled" —
independently agreed by both reviewers (`CODEX_REVIEW.md §35.5`). This
plan adopts that number rather than picking a new one, plus one cheap
run in front of it to de-risk the process-substitution procedure itself
(SS3 step 5's UNKNOWN) before spending any of the four real captures on
discovering a problem with it.

| Run | `diag_enabled` | Purpose |
|---|---|---|
| 0 (control) | `false` | Confirms the substitution procedure itself works, the lifecycle-manager/bond UNKNOWN doesn't fire, and `nav_bench.py`'s TOUR completes with outcomes/timing comparable to this repo's existing recorded baseline for the same route (SS8). If this diverges, stop and fix the *procedure*, not the instrumentation, before Run 1. |
| 1-4 (measurement) | `true` | Same TOUR, same config, fresh simulator each, no tuning between runs. Each run's 21 legs give many `motion_delta` events for a within-run distribution; 4 independent runs give the cross-run sign/magnitude consistency check that separates a repeatable bias from run-to-run noise. Matches the scale C2-NAV.34's own proxy analysis used (n~253) and the scale both prior independent reviews already agreed on. |

Do not add more runs "to be safe" and do not cut to fewer — both
directions were considered and this is the minimum that can
distinguish repeatability from noise while matching the
already-agreed-on follow-up scope.

## 6. Pre-registered analysis (defined now, before any run)

`c2nav36_diag.py join diag.jsonl gt.csv --out joined.csv` gives
per-update rows with `residual_along/lat/yaw = odom_input_delta -
GT_delta` (body-frame). **As written today, `c2nav36_diag.py`'s
`summarize()` computes only the pooled mean** — no median, percentile,
or CI. That tool is already tested (19/19) and is not modified by this
task; instead, run this identical post-processing over every run's
`joined.csv` (same recipe every time, no cherry-picking):

```python
import pandas as pd
df = pd.read_csv("joined.csv")
for col in ("residual_along", "residual_lat", "residual_yaw"):
    s = df[col]
    print(col, "mean", s.mean(), "median", s.median(),
          "p2.5", s.quantile(0.025), "p97.5", s.quantile(0.975),
          "n", len(s), "sign_frac_positive", (s > 0).mean())
```

Also compute, across the 4 measurement runs:
- per-run `residual_along` sign and the fraction of runs agreeing in sign;
- per-leg accumulated `residual_along` (grouped by leg, using
  `nav_bench.py`'s own leg boundaries/timestamps) to see whether any
  bias concentrates on `wall_adjacent`-like legs vs. `open_space`, or is
  uniform;
- direct comparison of magnitude and sign against the existing
  `/amcl_pose`-vs-GT proxy bias already on record (~0.09-0.13 m
  southward) — not just "differs from zero."

**Evidence rules (reusing, not replacing, C2-NAV.35 §10's own
pre-registered falsifier):**
- **SUPPORTS** the odometry-bias hypothesis: `residual_along`'s
  interval excludes zero, is positive, sign-consistent across all 4
  measurement runs, and its magnitude is compatible with the existing
  proxy bias's order of magnitude and sign.
- **WEAKENS** it: small or sign-inconsistent residual across runs, or a
  magnitude far too small to account for the proxy bias alone.
- **REJECTS** odometry as *sufficient* explanation: the residual's
  interval straddles zero while the existing `/amcl_pose` proxy bias
  remains significant.
- "Residual differs from zero" alone, without the magnitude/sign
  compatibility check against the existing proxy bias, is **never**
  sufficient to claim causality.
- If any run's `join` reports `status: UNOBSERVABLE`, or produces fewer
  than ~50 correlated per-update events, exclude that run from the
  pooled statistic and report it as a gap — never silently treat a
  thin/empty join as a clean zero result. `c2nav36_diag.py`'s existing
  blindness guard already enforces the *detection*; this rule is about
  what to *do* when it fires (report and exclude, don't re-run until it
  goes away).

## 7. Offline analysis commands (after each run, and pooled at the end)

```bash
python3 -P docs/data/c2nav36_diag.py schema <run_dir>/diag.jsonl
python3 -P docs/data/c2nav36_diag.py join <run_dir>/diag.jsonl <run_dir>/gt.csv --out <run_dir>/joined.csv
# then the pandas recipe in SS6 over <run_dir>/joined.csv, and again over
# the concatenation of all 4 measurement runs' joined.csv files.
```

## 8. Timing-neutrality sanity check (not a proof, a sanity gate)

C2-NAV.36 documentation is explicit that live timing-neutrality with
`diag_enabled=true` is unmeasured — this task does not claim to prove
it. Minimum sanity check, no new tuning:
- **Disabled baseline:** reuse this repo's existing recorded
  leg-timing/outcome data for the same `nav_bench.py` TOUR (already on
  record from prior C2-NAV benchmark runs) rather than re-measuring a
  fresh disabled baseline from scratch — "where practical," per the
  task.
- **Enabled sanity:** compare Run 0's (control, `diag_enabled:=false`
  but running through `amcl_diag`/the substitution procedure) and Runs
  1-4's (`diag_enabled:=true`) overall mission time and per-leg outcome
  against that baseline. Record whether they stay comparable (same
  order of magnitude, same leg pass/fail pattern) — do not tune
  anything to force agreement, and report a material divergence as a
  finding, not a discrepancy to explain away.
- This is a sanity gate, not a timing study. A material divergence is
  itself a STOP condition (SS9) — investigate before trusting the
  residual measurements from that run.

## 9. STOP conditions

Abort (do not proceed, do not improvise a workaround) if any of:
- Wrong branch/worktree, or `main` is not untouched.
- Any protected file is dirty: AMCL/costmap/controller params,
  `PolygonStop`/`PolygonSlow`, goal tolerances, maps, benchmark
  thresholds, navigation launch files.
- `amcl_diag` executable missing, fails to build, or the pre-run check
  (SS3 step 8) does not show `diag_enabled:=True` and a non-empty
  `diag_output_path`.
- `/model/coco/odometry` is not publishing (check with `ros2 topic hz
  /model/coco/odometry` before driving any goal).
- Navigation configuration differs from `gazebo_models/config/nav2_params.yaml`
  / the existing map/world in any way.
- `diag.jsonl` or `gt.csv` is malformed (`c2nav36_diag.py schema`
  fails) or empty.
- `join` reports non-overlapping timestamps or `status: UNOBSERVABLE`
  for a measurement run.
- SS3 step 5's lifecycle-manager/bond divergence fires.
- Any unexpected behavior change versus normal bringup (robot doesn't
  move as expected, AMCL never activates, TF looks wrong).
- Any safety issue (unexpected motion, collision risk).

## 10. Verdict

**READY_FOR_LIVE_CAPTURE** — conditional on the SS1 fix, which is
included in this same change. Not claimed: that the odometry-bias
hypothesis is true or false (SS6 stays open until Runs 1-4 are
analyzed), and not claimed: that live timing-neutrality is proven (SS8
is a sanity gate, not a proof). No live experiment was run to produce
this document.
