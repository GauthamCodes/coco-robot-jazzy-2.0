# C2-NAV.39 — implementation mode: accepted config shipped, capture integrity, reproducible tours

**Agent:** implementation, integration and simulation validation. The
investigation loop is paused by the human engineer. Full report:
`docs/agents/C2-NAV.39_RESULTS.md`.

## Objective
Turn C2-NAV.37/.38 and the accepted navigation changes into shipped,
tested configuration and tooling, validated in the real simulation, without
weakening any safety gate.

## Current state (FACT, this session)
- **Shipped config.** `gazebo_models/config/nav2_params.yaml` is now
  byte-identical to the validated `docs/data/c2nav11_ntp_params.yaml`
  (sha256 `6f61e499…`): local CSF 65, the multi-pose BT, BaseObstacle 8.0.
  Before this session the default still carried the REJECTED BaseObstacle
  2.0, CSF 5.0 and the single-pose BT, and C2-NAV.37 ran on that.
  `test_nav2_params_guard.py` now pins accepted values, gates and
  tolerances.
- **Capture integrity.** The recorder writes a session header/footer
  (schema 2) and the GT sidecar writes a meta file. `c2nav36_diag.py join`
  validates footer/drops, duplicates, clock/frame discontinuities, TF/GT
  pairing, anchor/timestamp consistency, a 0.1 s GT bracket, model-to-base
  frame correspondence and the motion decomposition before reporting. The
  decomposition is gtest-checked against the installed
  `DifferentialMotionModel`. `amcl_diag_swap.py` makes the C2-NAV.37
  component swap reproducible.
- **Tour tooling.** `nav_tour_run.sh <experiment.yaml>` runs one fresh sim
  and one bounded tour. It guards against running over another sim, uses a
  deterministic run dir and manifest, reads parameters back live, and tears
  down its own process groups. `nav_params_overlay.py` handles
  difference-only experiment overrides, refusing unknown keys and
  unauthorised safety changes.
- **Build and tests.** 4 packages build. Tests: `coco_nav_diag` 79/0 (9
  cppcheck self-skips), `gazebo_models` 68/0, `coco_config` 70/0,
  `custom_teleop` 67/0, `coco_mission` 281/0, analyser selftest 51/0,
  report selftest 13/0.

## Evidence (measured, real simulation)
- **Baseline ×3 fresh sims:** **17/21**, against C2-NAV.5's 18/21 (same
  config, same protocol). Ordinary legs 15/15, `enclosure_entry` 2/3 (median
  55.12 s vs 74.91 s), `enclosure_exit` **0/3**. Every exit was held by
  PolygonStop (176.38 s over 3 legs); minimum world-file clearance 0.2423 m,
  inside the 0.25 m circle and outside the 0.2051 m circumscribed radius.
- **Against the unfixed default C2-NAV.37 ran:** `wall_adjacent` 8/15 → 3/3,
  `enclosure_entry` 0/15 → 2/3.
- **Diagnostic run r01:** 7/7 tour. Footer shows 4438 recorded/written, 0
  dropped. GT meta is clean. Strict join OK: 213/213 correlated, along
  +0.000487 m, lateral +0.001812 m, yaw −0.011855 rad per update.
- **Regression:** re-analysing the four C2-NAV.37 captures with every new
  check reproduces 2273 correlated updates and +0.000747 m/update exactly
  (LEGACY_UNVERIFIED, no footer).
- **Two defects found in simulation and fixed.**
  - `f144852`: the new `ros_clean.sh` pattern `'amcl_dia[g]'` killed the tour
    runner by its experiment name during r01's teardown. Verified fixed by
    r02, which survived its sweep with `0 matched`.
  - `fbb4dc5`: nav2's lifecycle manager activated the reloaded `/amcl` before
    the swap tool's explicit `configure`, which failed r02's swap. The tool
    is now state-aware.
- **Diagnostic r03, with both fixes:** tour 6/7. Footer 4752/4752, 0 dropped.
  Strict join OK, 207/207. Across all five completed tours of the shipped
  config: **30/35** — ordinary legs 25/25, `enclosure_entry` 3/5 (both
  misses terminal-yaw timeouts), `enclosure_exit` 2/5 (all misses PolygonStop
  holds).

## Hypothesis (not tested here)
`enclosure_exit`'s failures are the C2-NAV.5/.6 geometry: the CSF 65 entry
parks the robot inside PolygonStop's circle, and the monitor correctly
refuses to move it. Nothing measured this session contradicts that.

## Requested action (next implementation step)
Give `enclosure_exit` a way out without touching PolygonStop.

The one change on record that removed the exit trap is C2-NAV.7's
corridor-centre entry goal `(-3.575, 2.95)`: `enclosure_exit` 3/3 with 0
STOP frames. It is not on the rejected list, but it is **not clean**. In
the full tour (C2-NAV.8) it scored 18/21, and one tour in three ended in a
269.5 s PolygonStop deadlock at `box_obstacle_1`'s SW corner, which is a
safety-relevant failure.

Concrete step:
1. Add a `goals` override to the experiment schema (`nav_params_overlay.py`
   `bench` block → `nav_bench.py --goal`, which already exists).
2. Commit `experiments/entry_corridor_centre.yaml`.
3. Run ≥3 fresh sims through `nav_tour_run.sh`.
4. Judge it with `c2nav39_tour_report.py` against this session's 17/21,
   gating on SW-corner PolygonStop time as well as success.

Whether a benchmark goal may move at all is the human engineer's call.

## Verification
`python3 -P docs/data/c2nav39_tour_report.py report --c2nav5 docs/data/c2nav5_bench.json --c2nav39 <run dirs>`;
every run's `params_live.txt` all OK; `ros_clean.log` `0 matched`.

## Risks / unresolved
- The default-config change also reaches `mission.launch.py` (topology B),
  which was not validated here.
- `nav_bench.py` intermittently segfaults at shutdown after writing its
  JSON (9/80 historical runs, 1/5 here). This is recorded, not fixed.
- **Observed:** every SUCCEEDED leg ends 0.24–0.46 rad from goal yaw in
  ground truth, while the checker's tolerance of 0.25 judges AMCL's
  estimate. That is consistent with an AMCL heading offset. Not
  investigated.
- The lateral and yaw odometry-input residuals from the diagnostic run have
  the opposite sign to C2-NAV.37's. N=1 on a different config; not
  interpreted.

---

# C2-NAV.38 — AMCL sensor-correction / posterior-bias investigation: RESULTS

**Agent:** offline investigation. No Gazebo, no ROS navigation, no
`nav_bench`, no live experiment, no parameter/map/controller/costmap
change. Full report: `docs/agents/C2-NAV.38_RESULTS.md`.

**FACT, traced directly from `coco_nav_diag/src/amcl_node.cpp` and
`libpf_lib.so` disassembly (not memory):** `/amcl_pose` and the map→odom
TF come from the mean of the **highest-total-weight KD-tree cluster**
(0.5m × 0.5m × 10° grid, hardcoded, confirmed via disassembly + `.rodata`
decode), extracted from the **post-resampling** particle set — under this
repo's `resample_interval: 1`, resampling runs every single scan cycle,
before pose extraction, not periodically.

**FACT, offline analysis of C2-NAV.37's own already-recorded traces (105
leg-instances, all 5 runs, all 7 legs, all 3 reps):** the AMCL-vs-GT
residual is **strongly leg/place-dependent, not accumulation-dependent**
— sign flips by location (`open_space` −0.112 m, `obstacle_corner`
**+0.037 m**), correlation with position in the 21-leg chained trajectory
is **+0.089** (indistinguishable from zero). A second test — whether
resampling's duplicate-particle mechanism concentrates support
southward — also came back against that hypothesis (duplicates skew
slightly *north*, CI [−0.0330, −0.0212], wrong direction).

**Combined with C2-NAV.29-.37's own record**, every process-level
mechanism actually tested now explains at most a minority of the bias or
nothing: odometry input (C2-NAV.37, real, ~8x too small, non-accumulating
per this session); motion-model noise (C2-NAV.32, ~0, CI straddles zero);
sensor/importance weighting (C2-NAV.33, ~3% of the bias, CI straddles
zero); resampling duplication (this session, wrong-signed); pose
extraction/cluster-selection (C2-NAV.30 **and** this session,
independently, twice: ≤0.0003 m, negligible); map/scan geometry
(C2-NAV.29/.31, real but only 29-38% at `wall_adjacent`).

**Leading hypothesis, not proven:** the unexplained majority is most
likely some other **place-dependent, geometry/discretization-linked**
effect this session's leg-type pattern points toward but does not
identify. **One minimal next decisive step proposed** (offline, no new
live experiment): project C2-NAV.37's already-measured odometry residual
— both along-track *and* the previously-uncompared lateral component —
into world frame via each leg's actual heading, and check it against this
session's per-leg-type residual table. Full reasoning, evidence table,
and FACT/OBSERVATION/HYPOTHESIS/UNKNOWN breakdown in
`docs/agents/C2-NAV.38_RESULTS.md`.

**UNRESOLVED, carried forward:** `pf_cluster_stats`/`pf_kdtree_cluster`'s
actual implementation has never been disassembled by any C2-NAV session,
including this one — this investigation relied on recorded-data
comparisons (published pose vs. recorded particle-cloud centroid) as a
strong proxy, not on reading that function directly. Nothing in this
session's findings should be read as license to tune `resample_interval`
or any other AMCL/costmap/controller parameter.

---

# C2-NAV.37 — live odometry-input capture: RESULTS

**Agent:** measurement. Live capture, not tuning: no AMCL/costmap/
controller parameter, map, launch file, or benchmark threshold changed.
One additive fix was required and made: `coco_nav_diag`'s `AmclNode` class
already carried the upstream `RCLCPP_COMPONENTS_REGISTER_NODE` macro, but
`CMakeLists.txt` never called the CMake-side registration macro that writes
the ament index entry `ros2 component load` reads — added as a single
line (`rclcpp_components_register_nodes`), no AMCL/recorder logic touched.
Full report: `docs/agents/C2-NAV.37_RESULTS.md`.

**Blocking discovery, resolved this session:** the pre-flight checklist's
substitution procedure (kill a standalone `/amcl` process) doesn't apply —
`nav.launch.py` composes all 14 Nav2 nodes into one shared
`component_container_isolated` process; there is no standalone `amcl`
process to kill. Fixed by registering `coco_nav_diag::AmclNode` as a
loadable component (see above) and using `ros2 component unload`/`load`
for a genuine clean single-node swap, verified across 5 independent runs
to cause zero disruption to the other 13 nodes or their lifecycle bonds.

**FACT:** Run 0 (control, `diag_enabled:=false`) plus Runs 1-4
(measurement, `diag_enabled:=true`) all completed, fresh simulator each,
zero orphan processes, zero dropped diagnostic events, zero dropped GT
rows, 2273 correlated motion updates pooled across the 4 measurement runs.
The exact odometry input AMCL consumes carries a real, sign-consistent,
statistically-nonzero along-track residual: **+0.000747 m/update pooled**
(range +0.000693 to +0.000810 across the 4 runs) — but this is **~8x
smaller** than C2-NAV.34's own output-side proxy figure (+0.00616
m/update) that was already shown to account for the ~0.09-0.13 m
`/amcl_pose`-vs-GT position bias.

**Classification: B — ODOMETRY CONTRIBUTES BUT IS INSUFFICIENT.** The
input-side discrepancy is real but not large enough, alone, to explain the
observed position bias. This does not resolve what does — C2-NAV.35's own
alternative hypothesis (that the bias is substantially an artifact of
AMCL's own correction/estimation step, not its odometry input) remains
open and is now the better-supported next line of inquiry. Full residual
statistics, the pre-registered evidence-rule evaluation, per-run
breakdowns, and an honest accounting of what's measured vs. inferred vs.
unknown are in `docs/agents/C2-NAV.37_RESULTS.md`.

**UNRESOLVED, carried forward:** what primarily explains the position
bias, if not odometry input. Next scientifically-justified step:
instrument AMCL's particle-cloud mean immediately before vs. after each
measurement-update correction step, compared against ground truth, to
isolate the correction step's own contribution — a new, separate
experiment, not scoped or run here.

---

# C2-NAV.36 — instrument the exact AMCL odometry input, default-disabled

**Agent:** implementation. No AMCL behaviour changed, no navigation
parameter changed, no simulator started, no live experiment run, `main`
untouched. Work is entirely additive: one new package
(`coco_nav_diag`) plus two new standalone analysis scripts under
`docs/data/`. Nothing existing was edited except this file. This section
is the answer to C2-NAV.35 SS8-SS9's own "minimum required instrumentation"
and to `CODEX_REVIEW.md SS35.5`'s independently-converged proposal, both
reproduced below the fold in this file.

Tags below follow the task's own four-way scheme, distinct from (but
compatible with) the FACT/DERIVED/PROXY/HYPOTHESIS/UNKNOWN vocabulary used
in the rest of this file: **FACT** (verified this session, reproducible),
**OBSERVATION** (something measured/run this session, e.g. a test result),
**HYPOTHESIS** (a claim not yet tested against real data), **UNRESOLVED**
(known open question, explicitly not closed by this task).

## 1. Summary

The exact `odom -> base_footprint` transform AMCL consumes, and the exact
delta its motion model integrates, were established in C2-NAV.35 to be
**unrecorded anywhere in this repository's history** (HANDOFF SS5, "This is
UNKNOWN, and no amount of re-analysis of existing bundles changes that").
This task closes that gap structurally: it does **not** collect the
missing measurement (that is a separately-authorized live experiment, not
performed here), it builds and validates the **instrument** that would
collect it, per the exact minimum both C2-NAV.35 (SS8) and
`CODEX_REVIEW.md` (SS35.5) independently specified: a default-disabled,
in-process hook at AMCL's own scan-stamped TF lookup and motion-delta
computation, paired with an out-of-process, acquisition-stamped
ground-truth recorder joined offline.

**FACT:** the instrumentation builds cleanly, its unit tests pass (13/13
gtest + 6/6 ament linters = 32/32 via `colcon test`), and its offline
join/selftest passes (19/19), including a synthetic-injection recovery
check and an explicit blindness-guard check — see SS6.

**UNRESOLVED:** no live capture has been taken. `diag.jsonl` and `gt.csv`
do not exist yet. The odometry-bias HYPOTHESIS from C2-NAV.35 remains
exactly as open as it was before this task — this task changes what is
*possible* to measure next, not what has been measured.

## 2. Exact files changed

All new; nothing pre-existing was edited except this file.

```
coco_nav_diag/package.xml
coco_nav_diag/CMakeLists.txt
coco_nav_diag/include/coco_nav_diag/diag_recorder.hpp
coco_nav_diag/src/diag_recorder.cpp
coco_nav_diag/include/coco_nav_diag/amcl_node.hpp      (forked, see SS3)
coco_nav_diag/src/amcl_node.cpp                         (forked, see SS3)
coco_nav_diag/src/main.cpp                              (forked, unmodified body)
coco_nav_diag/test/test_diag_recorder.cpp
docs/data/c2nav36_gt_sidecar.py
docs/data/c2nav36_diag.py
docs/agents/HANDOFF.md                                  (this section)
```

## 3. Exact instrumentation point

**FACT.** `coco_nav_diag/{include,src}/amcl_node.{hpp,cpp}` is a
version-matched fork of `nav2_amcl` 1.3.11's `amcl_node.hpp`/`.cpp`,
fetched this session from the tagged upstream source
(`https://github.com/ros-navigation/navigation2/blob/1.3.11/nav2_amcl/`).
The header was diffed byte-for-byte against the header actually installed
at `/opt/ros/jazzy/include/nav2_amcl/amcl_node.hpp` (`ros-jazzy-nav2-amcl
1.3.11-1noble.20260412.054619`) and is **identical** before any edit —
`diff` reports zero difference. **UNRESOLVED:** the `.cpp` could not be
verified the same way — no `.cpp` is installed on this machine, only
headers and compiled `.so` files (the same constraint C2-NAV.34/.35 hit
disassembling `libamcl_core.so`). The fetched tagged `.cpp` is assumed to
match the compiled binary's logic; this is standard ROS release practice
(the `.deb` is built from the tagged release tarball with no source
patches applied for this package, as far as could be checked), not
independently confirmed byte-for-byte the way the header was.

The class was renamed `nav2_amcl::AmclNode` -> `coco_nav_diag::AmclNode`
(namespace only; every reference to genuinely external types --
`nav2_amcl::MotionModel`, `nav2_amcl::Laser`, `nav2_amcl::LaserData`,
`nav2_amcl::angleutils`, the `"nav2_amcl::DifferentialMotionModel"`
pluginlib string -- is left pointing at the real, unmodified, installed
`nav2_amcl` package) so the fork cannot collide with the installed
`nav2_amcl::AmclNode` symbol. It builds as a separate executable
`amcl_diag` (library `amcl_diag_core`), linking against the installed
`nav2_amcl` package's headers and its `pf_lib`/`map_lib`/`sensors_lib`/
`motions_lib` shared libraries unchanged -- **only `amcl_node.cpp` is
recompiled**; the particle filter, map, sensor and motion-model code is
not rebuilt or touched.

`diff -u` against the fetched upstream files (reproduced in full in the
commit) shows the change is **purely additive except six one-line
refactors**, each extracting an existing boolean expression into a named
local so it can be read twice (once for its original purpose, once for
the diagnostic event) without changing control flow:
`shouldUpdateFilter(pose, delta)` and `lasers_update_[laser_index]` in
`laserReceived()`. No upstream line was deleted, reordered, or had its
logic changed.

**Instrumentation call site 1 -- the TF lookup**, `AmclNode::getOdomPose()`
(`coco_nav_diag/src/amcl_node.cpp:493`): a `coco_nav_diag::OdomTfLookupEvent`
is recorded in the `catch (tf2::TransformException&)` block (failure) and
immediately before `return true` (success), using the **same**
`tf_buffer_->transform()` result the function already computed for its own
return value -- no second/alternate lookup.

**Instrumentation call site 2 -- the motion delta**,
`AmclNode::laserReceived()` (`coco_nav_diag/src/amcl_node.cpp:746`): a
`coco_nav_diag::MotionDeltaEvent` is recorded once on the `!pf_init_`
(anchor-initialization) branch, and once in the `else` branch, immediately
around the (possibly-invoked) `motion_model_->odometryUpdate(pf_, pose,
delta)` call, using `pf_odom_pose_` read **before** `updateFilter()` can
advance it later in the same call (proven correct: `pf_odom_pose_` is
mutated only at that one later point and at the anchor-init assignment,
confirmed by grep across the whole file). `delta_rot1`/`delta_trans`/
`delta_rot2` are computed with `nav2_amcl::angleutils::angle_diff` -- the
**identical inline header function** `nav2_amcl::DifferentialMotionModel::
odometryUpdate()` uses internally (fetched and read this session:
`https://github.com/ros-navigation/navigation2/blob/1.3.11/nav2_amcl/src/motion_model/differential_motion_model.cpp`)
-- applied to `pf_odom_pose_.v[2]`, which is proven algebraically equal to
that function's own `old_pose.v[2]` (`old_pose = pose - delta =
pf_odom_pose_`). This is not a re-derivation with a hand-copied formula
that could silently diverge; it is the same compiled inline code applied
to values proven identical to what the real, unmodified `motions_lib.so`
receives. **Guard:** these three fields are only populated (and
`motion_model_formula_applicable` only set true) when
`robot_model_type_ == "nav2_amcl::DifferentialMotionModel"` (this repo's
configured value, `nav2_params.yaml:32` per C2-NAV.34 SS5) -- if a future
session switches motion models, the recorded fields correctly go blank
rather than silently mislabelling an omni-model delta as a differential
one.

Neither call site performs a TF lookup to any ground-truth frame, waits on
any topic, or introduces any new blocking call on the enabled path beyond
appending a pre-formatted string to an in-memory bounded queue (see SS4) --
matching `CODEX_REVIEW.md SS35.5`'s explicit constraint: "Do not inject GT
into localization or wait for GT inside AMCL."

## 4. Schema

Two structured event types (plus a one-line `diag_session_start`), one
JSON object per line (JSONL), schema-versioned (`schema_version: 1`).
Full authoritative field list: `coco_nav_diag/include/coco_nav_diag/
diag_recorder.hpp` (`OdomTfLookupEvent`, `MotionDeltaEvent`). Cross-checked
this session, programmatically, against the actual `DiagRecorder::
serialize()` C++ source (not just eyeballed) -- exact match, zero
divergence.

`odom_tf_lookup` (one per `laserReceived()` cycle that reaches the lookup):
`update_index`, `scan_stamp_sec`/`scan_stamp_nanosec` (the scan header
stamp AMCL queried at), `base_frame_id`, `odom_frame_id`,
`lookup_success`; on success: `odom_x`/`odom_y`/`odom_yaw` +
`odom_qx/qy/qz/qw` + `odom_pose_stamp_sec`/`odom_pose_stamp_nanosec` (the
transform actually consumed); on failure: `error_message`,
`consecutive_failures`; always: `node_now_sec`/`node_now_nanosec`.

`motion_delta` (one per `laserReceived()` cycle that got a pose):
`update_index`, `scan_stamp_sec`/`scan_stamp_nanosec`, `is_anchor_init`,
`anchor_valid`, `anchor_update_index` (the update_index that produced the
anchor being used -- lets an offline join look up the anchor's own exact
timestamp instead of reconstructing it by matching pose values),
`anchor_x`/`anchor_y`/`anchor_yaw`, `pose_x`/`pose_y`/`pose_yaw`,
`delta_x`/`delta_y`/`delta_yaw` (raw odom-frame delta, exactly what is/
would-be passed to the motion model), `motion_model_formula_applicable`,
`delta_rot1`/`delta_trans`/`delta_rot2`, `motion_model_type`,
`should_update_filter`, `motion_update_invoked` (these two can and do
differ -- see the code comment at the call site and `CODEX_REVIEW.md
SS35.2`'s "successful sensor updates cannot serve as a substitute count
for actual motion-model invocations"), `node_now_sec`/`node_now_nanosec`.

Ground-truth CSV (`c2nav36_gt_sidecar.py`, independent of the above):
`stamp_sec`/`stamp_nanosec` (message **acquisition** stamp, i.e.
`header.stamp` -- not `recv_wall_sec`/`recv_wall_nanosec`, which is receipt
time and is recorded separately for reference only), `frame_id`,
`child_frame_id`, `x`/`y`/`z`/`yaw`/`qx`/`qy`/`qz`/`qw`.

## 5. How to enable it

Three new AMCL parameters, all default-off:

```yaml
amcl:
  ros__parameters:
    diag_enabled: true              # default: false
    diag_output_path: "/path/to/diag.jsonl"   # default: "" (empty = stays disabled, fail-safe)
    diag_max_events: 200000         # default: 200000
```

`diag_enabled: true` with an empty `diag_output_path` is a deliberate
no-op (logged, not an error) -- enabling capture with no destination never
crashes or silently opens something unexpected.

**This is not wired into any existing launch file.** `amcl_diag` is a
separate executable from the stock `amcl`; nothing currently running
switches to it. Reaching it requires either:

- **(a, recommended, zero launch-file edits)** Build `coco_nav_diag` into
  the workspace, stop the normally-launched `amcl` lifecycle node's
  process (or don't start bringup's AMCL at all), then run
  `ros2 run coco_nav_diag amcl_diag --ros-args --params-file
  <the same nav2_params.yaml> -p diag_enabled:=true -p
  diag_output_path:=<path>` and drive its lifecycle by hand
  (`ros2 lifecycle set /amcl configure`, then `activate`) in place of the
  bringup lifecycle manager. No launch file changes at all.
- **(b)** A dedicated copy of the relevant bringup launch file with
  `package='nav2_amcl', executable='amcl'` changed to
  `package='coco_nav_diag', executable='amcl_diag'` for that one node. Not
  created this session (would be a launch-file change, out of this task's
  scope) -- CLAUDE.md's `ros_clean.sh` pattern-coverage rule would then
  apply the day this executable is added to any launch file.

Ground truth, in parallel, from a second terminal:
`python3 -P docs/data/c2nav36_gt_sidecar.py --out /path/to/gt.csv`.

## 6. How to run it later

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select coco_nav_diag
# ... start the world, bring up everything EXCEPT amcl as usual, then SS5(a) ...
# after the run:
python3 -P docs/data/c2nav36_diag.py schema /path/to/diag.jsonl
python3 -P docs/data/c2nav36_diag.py join /path/to/diag.jsonl /path/to/gt.csv --out joined.csv
```

`join` prints `status: OK` with the pooled along-track/lateral/yaw residual
(consumed odometry input **minus** ground truth, body-frame, matching
`docs/data/c2nav34_odom.py`'s convention exactly so the two are directly
comparable), or `status: UNOBSERVABLE` with a reason breakdown if nothing
could be correlated -- it never silently reports a clean/zero-bias result
when the join actually saw nothing (CLAUDE.md's blindness-guard rule,
`c2nav36_diag.py`'s own `summarize()`, tested in SS6 below).

## 7. Tests performed

**OBSERVATION**, this session, on this machine:

- `colcon build --packages-select coco_nav_diag` (built into a scratch
  build/install dir outside `~/ros2_ws/build|install`, sourcing only
  `/opt/ros/jazzy` -- no dependency on this repo's own packages or on the
  shared workspace's existing build tree): **succeeds**, 0 warnings-as-
  errors, after one fix (restoring upstream's `HAVE_DRAND48`
  `check_symbol_exists` guard, dropped in the first CMakeLists draft and
  caught immediately by a real compile error, not a review).
- `colcon test --packages-select coco_nav_diag`: **32/32, 0 failures, 0
  errors** -- `test_diag_recorder` (13 gtest cases: disabled mode x3,
  enabled logging path x2 including a real async-thread drain-on-stop
  test, TF-failure handling x2, deterministic serialization/schema x4
  including one exact full-string equality assertion, bounded-queue drop
  accounting x1, double-configure misuse x1), `cppcheck`, `lint_cmake`,
  `uncrustify` (2 long-line fixes applied after the first run), `xmllint`.
- `python3 -P docs/data/c2nav36_diag.py selftest`: **19/19 passed** --
  GT-interpolation bracketing and no-extrapolation, body-frame rotation
  convention parity with `c2nav34_odom.py`, a synthetic +0.05 m
  along-track discrepancy injected and recovered to `1e-9` (the same
  injection-recovery pattern `c2nav34_odom.py`'s own selftest uses),
  TF-lookup-failure cycles correctly excluded from the candidate set, the
  blindness guard correctly reporting `UNOBSERVABLE` (not a false-clean
  `OK`) both for an empty capture and for a capture whose timestamps never
  overlap the GT window, and a three-update chained-anchor case confirming
  `anchor_update_index` resolves transitively.
- `c2nav36_diag.py schema` field list cross-checked programmatically
  (`docs/agents/HANDOFF.md`-external one-off check, not part of the
  committed selftest) against a regex-extracted field list read directly
  out of `diag_recorder.cpp`'s two `serialize()` bodies: **exact match,
  zero divergence**, both event types.
- `flake8 --max-line-length=120 --ignore=E203,W503` (this repo's actual
  `.pre-commit-config.yaml` invocation) on both new Python files: **zero
  real findings** after one `W504` fix; the large batch of `D2xx`/`Q0xx`
  warnings a bare local `flake8` also reports come from docstring/quote
  plugins **not** in this repo's pre-commit config (confirmed by running
  the identical command against the pre-existing, already-accepted
  `c2nav34_odom.py`, which trips the same plugin warnings).
- Trailing whitespace / EOF-newline (the two other pre-commit-hooks this
  repo runs that are easy to check without installing pre-commit itself):
  **clean** on every new file.

**Not performed, by instruction:** no `amcl_diag` node was ever started;
no simulator was started; no live capture exists; no real `diag.jsonl` or
`gt.csv` was produced or joined.

## 8. What remains unknown

1. **UNRESOLVED -- the `.cpp`'s byte-identity to the installed binary is
   unverified** (SS3). Only the header was diffable.
2. **UNRESOLVED -- live timing neutrality when `diag_enabled=true` is not
   measured.** The disabled path is proven to be a single relaxed-atomic
   bool load per call site (zero allocation, zero lock) by code inspection
   and by the `DiagRecorderDisabled` test group, matching
   `CODEX_REVIEW.md SS35.5`'s explicit requirement: "Document residual
   scheduling overhead; offline equality alone does not prove live timing
   neutrality." The enabled path's effect on real scan-callback latency
   under load has not been measured against a running AMCL -- that
   requires the live experiment this task does not perform.
3. **UNRESOLVED -- the odometry-bias HYPOTHESIS itself is exactly where
   C2-NAV.35 left it.** This task adds no evidence either way about
   whether AMCL's actual consumed odometry carries the along-track bias;
   it only makes that evidence possible to collect.
4. **UNRESOLVED -- `DiagRecorder::configure()` is a one-shot.** A
   `cleanup` -> `configure` lifecycle cycle after the first `configure()`
   call does not re-arm capture (it warns and stays in its first state).
   Documented, not fixed -- fixing it is scope beyond "minimal
   instrumentation" for a single-shot diagnostic run.
5. **UNRESOLVED -- multi-laser `anchor_update_index` behaviour is reasoned
   through, not empirically tested.** This repo's configuration uses one
   laser (per every prior C2-NAV session's parameter table); the
   single-laser case's "anchor advances exactly when
   `motion_update_invoked`" property was verified by reading the control
   flow (SS3), not by running a multi-laser AMCL.
6. **UNRESOLVED -- no live capture, so the C2-NAV.35 falsifier (SS10) is
   still unevaluated.** Both verdict branches (odometry-bias SURVIVES vs.
   FALSIFIED) remain open.

## 9. Why this instrumentation is sufficient to settle the odometry-input question

Checked against the converged minimum both C2-NAV.35 SS8 and
`CODEX_REVIEW.md SS35.5` specified independently, item by item:

| required | met by |
|---|---|
| scan header stamp/frame + a monotonic event ID | `update_index`, `scan_stamp_sec/nanosec`, `base_frame_id` (**FACT**) |
| TF lookup success/failure, distinguished from MessageFilter drops | `lookup_success` + `error_message`; a MessageFilter drop never reaches `getOdomPose()` at all and so never has an `update_index` allocated for it either -- the gap in `update_index` sequence at the offline-analysis stage is itself the record of a drop (**FACT**, not yet exercised against a real drop) |
| the **actual** successful lookup result, not a second lookup | Recorded from the same `tf_buffer_->transform()` output the function already used for its return value (**FACT**, SS3) |
| the computed delta and the prior anchor pose/event | `delta_x/y/yaw`, `anchor_x/y/yaw`, `anchor_update_index` (**FACT**) |
| lookup failures and init/reset events recorded separately | `is_anchor_init` distinguishes a reset-triggered first-scan from a genuine consumed delta; TF failures are a distinct event type entirely (**FACT**) |
| synchronized ground truth at anchor and current time, bracketing samples, interpolation method, gap recorded, never nearest-receipt substitution | `c2nav36_gt_sidecar.py` (acquisition-stamped) + `c2nav36_diag.py`'s `interpolate_gt()` (linear, bracket-only, returns `None`/UNOBSERVABLE outside the bracket, records `gap`) (**FACT**, code; **OBSERVATION**, selftest-verified; **UNRESOLVED**, never run against a real capture) |
| bounded, nonblocking, drop-accounted | `DiagRecorder`: fixed-capacity queue, background writer thread, atomic drop counter, single relaxed-load fast path when disabled (**FACT**, code + tests) |
| offline validation before any live run: rigid motion, zero-error control, injected error, yaw wrap, duplicate/failed lookups, queue overflow | `c2nav36_diag.py selftest` (interpolation edge cases, injected-error recovery, TF-failure exclusion, blindness guard, chained anchors) + `test_diag_recorder` (disabled/enabled/failure/schema/bounded/misuse) (**OBSERVATION**, 32/32 + 19/19 this session) |

What is **not** claimed: that the odometry is or is not biased (SS8
above), or that this instrumentation has been proven timing-neutral live
(item 2 above). Those are exactly the two things the next, separately
authorized live experiment is for.

---

# C2-NAV.35 — reconcile the odometry hypothesis with the exact AMCL input

**Agent:** investigation/review only. No behaviour changed, no parameter
changed, no simulator started, no live experiment run. `main` untouched at
`ea66155`. This entry supersedes the *interpretation* of C2-NAV.34 below
(the historical entry is retained under "Historical C2-NAV.34" for the
record, including its passing self-tests — nothing in it was re-run today).

**Before writing this, the worktree was fast-forwarded** from `87131a0` to
`eff6dd5` (2 commits: `ea80496` "C2-NAV.34 independently review AMCL
odometry input", `eff6dd5` "C2-NAV.35 resolve odometry evidence and minimum
capture requirements"). That fast-forward brought in `docs/agents/CODEX_REVIEW.md`,
which was **not present** when the historical C2-NAV.34 entry below was
written (its own §12 limitation 9 says so). `CODEX_REVIEW.md`'s own `## 35.*`
sections already contain an independent Codex-side C2-NAV.35 resolution,
produced separately from this one and reaching the same verdict by the same
method (installed-source inspection, not re-running any experiment). This
entry is the Claude-side counterpart, written after reading both.

Tags below: **FACT** (read this session from an installed artefact, or
already a FACT in a source document and re-verified), **DERIVED** (forced
by something measured, not directly read), **PROXY** (a related observable
without proven equivalence to the thing it's being used to argue about),
**HYPOTHESIS** (a candidate mechanism, not tested), **UNKNOWN** (evidence
does not exist).

---

## 1. C2-NAV.34 claims reviewed

The historical entry below made five load-bearing claims. Restated exactly
as written there:

1. F1–F4: AMCL performs a TF lookup `odom → base_footprint` at the scan
   timestamp with a zero local `getOdomPose` timeout, and "AMCL never waits
   for TF."
2. F8 "cancellation identity": `wheel_separation_multiplier` is applied
   symmetrically to command and odometry paths, so it "cannot by itself
   bias the reported yaw rate" — the briefed mechanism is "rejected as
   stated."
3. E1: signed residual `(AMCL delta) − (GT delta)`, pooled over 523 updates
   / 21 legs / 4 runs = along-track `+0.00616 m/update` (CI excludes zero),
   yaw `+0.00271 rad/update` (CI straddles zero).
4. F8 linear channel + E5: `v_odom ≡ v_cmd` "exactly — no separation, no
   multiplier", therefore (combined with a measured command-vs-GT slope of
   0.9712) "odometry over-reports forward distance by **≥ 2.96 %**",
   labelled `(FACT)`.
5. §8: the ≥2.96 % bound plus the residual mean can reproduce the named
   ~0.09 m southward bias "with room to spare", and "the command channel
   *alone*... already supplies 70 % of it."

## 2. Claude/Codex disagreements

Codex's review (`CODEX_REVIEW.md`, historical `## 1`–`## 12`, retained below
its own `## 35.*` addendum) disputed claims 2 and 4 directly, and softened
claim 1's framing:

- **On claim 1:** "'AMCL never waits for TF' is too broad: the preceding
  MessageFilter waits for transform availability with configured buffer
  timeout" — a correction to scope, not to the scan-stamp/direction/failure
  findings.
- **On claim 2 (F8):** "Both paths use the multiplier, but commanded wheel
  velocity is not measured encoder displacement. Cancellation requires
  perfect tracking and matched timing. Even then odom-minus-GT yaw may be
  nonzero." Classified **UNSUPPORTED as an unconditional identity**.
- **On claim 4 (`v_odom ≡ v_cmd`, the ≥2.96 % bound):** classified
  **UNSUPPORTED**, with a decisive counterexample: command 1 m/s, wheels
  physically turn at 0.9712 m/s with **no contact slip** → encoder travel
  0.9712 m → odometry reports 0.9712 m/s → **zero odometry error**, despite
  the exact same 0.9712 command/GT slope used to derive the "lower bound."
  Root cause stated as: "controller integrates measured encoder positions,"
  not commanded velocity, so equating a command-tracking shortfall with an
  odometry error is invalid.
- **On claim 5:** "the 4.2 % odometry gain is unmeasured... the claimed
  command channel supplies 70 % is unsupported."
- **On claim 3:** not disputed as a *statistic* — Codex reproduced the same
  numbers — but disputed as *evidence about odometry specifically*: "These
  are intervals between recorded pose samples, not proven individual filter
  updates or input errors."

## 3. Exact resolution of each disagreement

**Resolved independently in this session**, from the installed
`diff_drive_controller` configuration and parameter semantics — not by
adjudicating rhetoric between the two prior write-ups:

- `gazebo_models/urdf/coco_controllers.yaml:52` sets `open_loop: false`.
  `position_feedback` is left at its documented default `true`
  (`diff_drive_controller_parameters.hpp:87-88`, not overridden). Per the
  installed parameter's own description, `open_loop: false` means odometry
  is calculated **from feedback, not from the commanded values**. Confirms
  the historical entry's own F6 ("Odometry integrates MEASURED wheel
  positions").
- Given F6 is true, `v_odom` (what the controller's real odometry reports)
  is a function of **actual measured wheel rotation**, not of `v_cmd`. The
  claim `v_odom ≡ v_cmd` (F8 linear channel, E5's premise) is therefore
  **false as an unconditional identity** — it holds only in the special
  case where the wheels achieve exactly the commanded rotation, which is
  precisely what is in question.
- **Codex's disagreement on claim 4 is correct.** This is now confirmed from
  the controller's own documented parameter semantics, independently of
  Codex's specific probe/counterexample (which reaches the identical
  conclusion by direct construction).
- The historical entry's own script already contains the seed of this
  contradiction and did not resolve it: `docs/data/c2nav34_odom.py:401-404`
  states, immediately after asserting `v_odom == v_cmd` as its premise,
  "with `position_feedback=true` the odometry integrates MEASURED wheel
  rotation, so any wheel that spins faster than the ground moves adds
  over-report on top, and that component is invisible to every artefact in
  this repository." `HANDOFF.md` E5 then labelled the derived ≥2.96 % number
  `(FACT)` anyway. **Resolution: that label was wrong. Reclassified below as
  UNSUPPORTED/INVALID, matching `CODEX_REVIEW.md`'s independent
  conclusion.**
- **On claim 2 (F8 yaw cancellation):** the command-path application of the
  multiplier (`u_r − u_l = w_cmd · b_eff`) could **not** be verified from
  installed source in this session either — `diff_drive_controller.cpp` is
  not present anywhere on this machine, only headers and the compiled
  `.so`. `CODEX_REVIEW.md`'s own `## 35.3` reports it *did* obtain and read
  the version-tagged upstream `diff_drive_controller.cpp`/`odometry.cpp`
  source (`ros-controls/ros2_controllers` tag `4.39.0`) and confirms both
  paths do use the multiplier — but still classifies the resulting
  cancellation as **conditional**, not unconditional, for the same reason as
  claim 4: it requires measured wheel velocities to equal the setpoints,
  which is not established. **Resolution: F8's yaw-channel claim is DERIVED
  and directionally correct (the multiplier is genuinely self-cancelling in
  the odometry-vs-command comparison when tracking is perfect), but "rejected
  as stated" (§9, E9) overstates it as unconditional.** Reclassify as
  conditional.
- **On claim 1:** Codex's correction stands (confirmed independently — see
  §4 below). The scan-stamp, direction, and zero-`getOdomPose`-timeout
  findings are unaffected.
- **On claim 3 (the residual statistic itself):** not in dispute as
  arithmetic — both sides reproduce `+0.00616 m/update` and `+0.00271
  rad/update` identically. The dispute is entirely about what the statistic
  is evidence *of* (see §5/§6).

## 4. Exact AMCL odometry input

**FACT, cross-validated by two independent disassembly efforts (this
session's predecessors) plus, per `CODEX_REVIEW.md ## 35.2`, against
downloaded version-tagged upstream `nav2_amcl` 1.3.11 source:**

- `laserReceived` calls `getOdomPose(latest_odom_pose_, x, y, yaw,
  Time(scan->header.stamp, RCL_ROS_TIME), base_frame_id_)` — direction is
  **T_odom_base**: target frame `odom`, source frame `base_footprint`, at
  the **scan's header stamp**, not "latest available."
- The direct `getOdomPose`/`tf2_ros::BufferInterface::transform` call has a
  **zero-second lookup timeout** (`pxor %xmm0,%xmm0` immediately before
  `tf2::durationFromSec`).
- **Correction to the historical entry's "AMCL never waits for TF":** too
  broad. The `/scan` subscription sits behind a `tf2_ros::MessageFilter`
  targeting `odom_frame_id_` which *does* wait for transform availability,
  using `transform_tolerance_` (0.5 s here) as its buffer timeout, before
  the zero-timeout `getOdomPose` call runs. Net effect on the finding:
  unchanged — still scan-stamped, still drops the scan entirely on
  lookup failure/extrapolation (no motion update, no sensor update, no
  cloud) — only the "never waits" phrasing needed correcting.
- Lookup happens once per `/scan` message that reaches the filter (lidar
  rate), independent of `update_min_d`/`update_min_a` (which gate the
  motion/sensor *update*, not the TF lookup).

## 5. What historical data can and cannot prove

**Cannot prove:** the exact `odom → base_footprint` transform AMCL consumed
at any specific scan timestamp in any of the runs behind C2-NAV.28's bundle.
Confirmed by exhaustive search: no `tf2_ros.Buffer`/`TransformListener` for
this transform exists anywhere in this repository — not on `main`, not in
this worktree, not in the Codex worktree/branch. `nav_bench.py` subscribes
only to `/model/coco/odometry` (ground truth) and `/amcl_pose` (AMCL's
*output*, post motion-update *and* post laser-correction); it creates no TF
listener at all. The two TF consumers that do exist elsewhere in the repo
(`docs/data/c2m5_locrec.py`, `coco_mission/scripts/localization_monitor.py`)
query `map → odom` or `map → <scan frame>` at *latest available* time, by
explicit design, never stamp-matched, and never `odom → base_footprint`. No
rosbag of `/tf` exists anywhere. **This is UNKNOWN, and no amount of
re-analysis of existing bundles changes that** — it requires new
instrumentation (§8) and a new run, neither of which this task performs.

**Can prove (and does):** what `/amcl_pose` did relative to ground truth,
per filter update, signed and body-frame-resolved, pooled over 523 updates.
That is a real, correctly-computed, twice-independently-reproduced FACT. It
is just not a fact about AMCL's *input*.

## 6. Valid evidence for an along-track bias

- **FACT** (both sides, reproduced independently): the signed along-track
  residual `(AMCL delta) − (GT delta)` is `+0.00616 m/update`, CI
  `[+0.00342, +0.00890]`, excluding zero, over 523 updates/21 legs/4 runs.
- **FACT**: this residual is placed correctly by direction and location —
  positive (southward-consistent) along a due-south leg (`open_space`), and
  the world-frame error is created there and inherited by the next leg
  (historical E3/E4). This is real, descriptive evidence that *something*
  in AMCL's pipeline advances along-track faster than the robot physically
  does, on that specific leg.
- **HYPOTHESIS, structurally supported**: wheel slip (genuine contact-patch
  slip, not a controller tracking shortfall) could cause the underlying
  odometry to over-report forward distance, because F6 (measured-position
  integration, confirmed FACT) has no slip-compensation term — any wheel
  that physically rotates further than the ground moves adds directly to
  reported distance, with no correction. This mechanism is unquantified with
  current data (no wheel-encoder-vs-GT signal is recorded anywhere in this
  repo; `nav_bench.py`'s `v_wheel` is the **commanded** twist to
  `/diff_drive_controller/cmd_vel`, not an encoder reading).
- **FACT, corroborating but from a different experiment**: skid-steer
  yaw-tracking deficits are real and large in this simulator —
  `docs/RESULTS.md:1635-1648` (on `main`) measured a commanded 2.5 rad
  rotation achieving only 1.833 rad (73 %) in Gazebo, with
  `wheel_separation_multiplier: 1.10` explicitly measured to explain only a
  small fraction of that gap ("it is not the explanation, and the remaining
  ~2.6× is unexplained"). This supports skid-steer slip being a real,
  physically-plausible phenomenon on this chassis as a HYPOTHESIS-supporting
  data point for the along-track case — it is a separate pure-rotation
  experiment, not a direct measurement of the C2-NAV.34/.28 nav-leg dataset.

## 7. Invalid/unsupported claims

- **`v_odom ≡ v_cmd` as an unconditional identity — INVALID.** Refuted by
  the controller's own `open_loop: false` / `position_feedback: true`
  configuration (odometry integrates measured encoder feedback), confirmed
  independently of Codex's counterexample, which reaches the same
  conclusion by direct construction.
- **"≥2.96 % odometry over-report" lower bound (E5) — INVALID, labelled
  `(FACT)` in error.** Built on the identity above. `v_wheel` in
  `nav_bench.py` is a command (`/diff_drive_controller/cmd_vel`), not an
  encoder reading, so the `v_act/v_wheel = 0.9712` slope measures a
  controller-tracking shortfall, which — under measured-feedback odometry —
  produces **zero** odometry error by itself. Only genuine wheel slip would
  produce odometry error, and that component is separately unmeasured.
- **"The command channel alone supplies 70 % of the 0.09 m bias" (§8) —
  UNSUPPORTED.** Depends entirely on the invalid ≥2.96 % bound above.
- **`wheel_separation_multiplier` as a mechanism for the *along-track* bias
  — INVALID, and this was never actually in dispute.** Confirmed
  independently from the installed odometry equations
  (`linear = (u_r + u_l)/2`, no `wheel_separation` term at all): the
  multiplier has zero mathematical role in the linear channel. Do not use
  it as a causal candidate for the +0.09 m along-track bias, from either
  side of this disagreement.
- **F8/E9 "the multiplier cannot bias reported yaw... rejected as stated" —
  OVERSTATED, not invalid.** The cancellation argument is directionally
  correct and well-derived, but is conditional on perfect wheel-tracking
  (unverified) rather than unconditional. Reclassify from "rejected" to
  "conditionally supported; not falsified because the yaw residual is
  separately, independently consistent with zero."

## 8. Minimum required instrumentation

Both this session and `CODEX_REVIEW.md ## 35.5` converge on the same
answer, arrived at independently:

- **An external, opt-in TF sidecar** (a `tf2_ros.Buffer` + listener node
  performing the identical scan-stamped `lookup_transform('odom',
  'base_footprint', scan.header.stamp)` query AMCL performs internally,
  recorded alongside acquisition-stamped GT) would very likely reproduce
  the *value* AMCL's internal buffer resolves, since tf2 buffer state is a
  deterministic function of published `/tf` messages and query time. It is
  cheap and is a reasonable first cross-check. **It cannot, by itself,
  prove which scans AMCL actually accepted** (vs. dropped by the
  MessageFilter, or thrown out by an extrapolation exception), nor
  reconstruct the exact anchor state used for a given filter update.
- **The default-disabled in-process diagnostic hook is the actual minimum**
  for a causal claim: placed immediately before the existing
  `odometryUpdate`/prediction call inside a version-matched, isolated
  `nav2_amcl` source overlay, capturing (per `CODEX_REVIEW.md ## 35.5`,
  items 1–4): the scan header stamp/frame and a monotonic event ID; the
  successful `getOdomPose` result actually used (not a second, separate
  lookup relabelled as the consumed one); the computed delta and the prior
  anchor pose/event; lookup and MessageFilter failures, and
  initialization/reset events, recorded separately from each other; and
  synchronized ground truth at both the anchor and current scan times, with
  bracketing samples and interpolation method recorded, never a nearest-
  receipt-time substitution.

**Assessment: yes, this is the minimum required for the causal question**
(does AMCL's *actual consumed* input carry a systematic along-track error),
and no smaller instrumentation closes it — an external listener alone
cannot establish consumption identity, only value existence. **Not
implemented in this task, per instruction.**

## 9. Proposed C2-NAV.36 experiment (proposal only)

1. Implement the default-disabled hook above in an isolated `nav2_amcl`
   source overlay (version-matched to the installed 1.3.11), with a bounded
   nonblocking queue and explicit drop accounting; validate offline first
   against known rigid-frame motion, a zero-error GT control, injected
   signed translation/yaw errors, yaw wrap, duplicate/out-of-order stamps,
   failed lookups, and anchor resets — confirming identical filter
   outputs/RNG progression with capture off vs. on before any live run.
2. Add the external TF sidecar in parallel as a cheap corroborating
   cross-check (no change to existing topics, parameters, RNG, or update
   order).
3. Re-run the same route/leg structure as C2-NAV.28/34 (21 legs, 4 runs)
   with the hook enabled, and compute, for the first time, `(actual
   consumed odometry delta) − (GT delta)` directly — instead of `(AMCL
   output delta) − (GT delta)`.

## 10. Falsifier

If the hook-captured odometry-input residual's 95 % CI **straddles zero**
at `open_space` (no significant along-track odometry-input bias) while the
existing `/amcl_pose`-vs-GT proxy residual **remains significant and
positive**, that falsifies the odometry-bias hypothesis and points to the
laser/scan-matching correction term as the actual source of the ~0.09 m
bias. Conversely, a hook-captured residual whose CI **excludes zero, is
positive, and is of a magnitude consistent with −0.09 to −0.13 m southward**
when projected over `open_space`'s 2.13 m course, supports the odometry-bias
hypothesis directly, for the first time, rather than through a proxy. A
predeclared equivalence interval around zero (sized from the instrumentation's
own error budget, decided *before* collecting data) is required to call
either verdict; missing events/anchors/GT, or an uncertainty comparable to
the proposed effect, must be reported as **UNOBSERVABLE**, not folded into
either verdict — the same trap C2-NAV.33 already paid for once.

## 11. Confidence

| claim | confidence | basis |
|---|---|---|
| AMCL consumes `odom → base_footprint` via TF, at the scan timestamp, with a zero local `getOdomPose` timeout | **High** | Disassembly cross-validated by independent offset checks in two prior sessions, and against downloaded version-tagged upstream source (`CODEX_REVIEW.md ## 35.2`) |
| The exact historical transform AMCL consumed is unrecorded anywhere in this repository | **High** | Exhaustive search, both worktrees and `main` |
| `v_odom ≡ v_cmd` / the ≥2.96 % lower bound is invalid as an unconditional claim | **High** | Confirmed independently from the installed controller's own parameter documentation (`open_loop`, `position_feedback`), not solely from Codex's counterexample |
| `wheel_separation_multiplier` is not a candidate mechanism for the along-track bias | **High** | Confirmed from the actual odometry equations — no `wheel_separation` term in the linear channel |
| The yaw-cancellation argument (F8) is directionally correct but conditional, not unconditional | **Moderate–High** | Command-path symmetry corroborated against upstream source by `CODEX_REVIEW.md`, but perfect-tracking assumption remains unverified against this repo's actual runtime behaviour |
| Odometry (vs. the laser correction) is the actual source of the ~0.09 m bias | **Low — HYPOTHESIS** | Unchanged from the historical entry's own §13; now on firmer footing because the invalid intermediate "FACT" claims (E5, the 70 % attribution) have been corrected rather than propagated |

---

## Answer to the stop condition

**Do we actually have evidence that the exact odometry transform consumed
by AMCL is biased along-track, or was C2-NAV.34 relying on a proxy?**

**C2-NAV.34 was relying on a proxy.** The `+0.00616 m/update` along-track
number is a real, correctly-computed, twice-independently-reproduced
statistic — but it differences AMCL's *published output* pose (`/amcl_pose`,
after both the motion prediction and the laser-scan correction have already
been applied) against ground truth, not the `odom → base_footprint`
transform AMCL actually consumes at the scan timestamp, which is recorded
nowhere in this repository's history, on `main` or in either worktree. No
historical evidence exists, in either direction, about whether AMCL's actual
odometry input is biased along-track. Compounding this, the specific
quantitative argument C2-NAV.34 used to claim a large odometry contribution
(the `v_odom ≡ v_cmd`-derived "≥2.96 % lower bound") is independently
confirmed invalid: the controller integrates measured encoder feedback, not
commanded velocity, so a controller-tracking shortfall alone produces zero
odometry error, not a lower bound on it. The along-track odometry-bias
mechanism remains a live, structurally plausible **HYPOTHESIS** — via
genuine wheel slip, which measured-feedback integration would not
compensate for — not an established **FACT**, pending the instrumentation
proposed in §8–§9. This conclusion was reached independently in this
session and converges with the separately-produced `CODEX_REVIEW.md ## 35.*`
resolution.

---

# Historical C2-NAV.34 — investigation handoff: the odometry input to AMCL

**Agent:** investigation only. No behaviour changed, no parameter changed, no
simulator started, no live experiment run, `main` untouched at `ea66155`.
C2-NAV.33's handoff is not lost — it is this file's parent at `8a7b090`.

Claims are marked **FACT** (read this session from an installed artefact or
produced by a tool run this session), **DERIVED** (forced by something
measured, but not directly read), **HYPOTHESIS** (not tested), **UNKNOWN**.

Everything reproducible with:

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 -P docs/data/c2nav34_odom.py selftest   # 18 passed, 0 FAILED
python3 -P docs/data/c2nav34_odom.py geom
python3 -P docs/data/c2nav34_odom.py incr       # the headline
python3 -P docs/data/c2nav34_odom.py heading    # place dependence
python3 -P docs/data/c2nav34_odom.py elim
python3 -P docs/data/c2nav34_odom.py cmd
python3 -P docs/data/c2nav34_odom.py verdict
```

---

## 1. QUESTION

> Does the `odom → base_footprint` motion actually consumed by AMCL contain a
> systematic translational or angular error that could plausibly generate the
> observed wall-adjacent localization bias?

**Answer: (A), with the axis changed.** AMCL's motion input over-reports
**forward travel**, not yaw. The briefed `wheel_separation_multiplier`
mechanism is **rejected as stated** — it cannot bias the reported yaw rate at
all — and the yaw residual measures **zero**. The along-track residual does
not: `+0.00616 m` per filter update, 95 % CI `[+0.00342, +0.00890]`, which
**excludes zero**.

**But this is not proof of an odometry bias, and I am not claiming it is.**
What is measured is `(AMCL delta) − (GT delta)`, which equals
`(odometry error) + (laser correction)`, and **nothing in this repository
separates those two terms.** The separating measurement is section 9.

> **[C2-NAV.35 note]** This section's headline claims are reconciled above.
> The along-track/yaw axis finding (E1) stands as a statistic. The causal
> "over-reports forward travel" framing, and the "rejected as stated" framing
> for the multiplier, do not — see the C2-NAV.35 entry at the top of this
> file.

---

## 2. ACTUAL AMCL MOTION INPUT

Read from the installed `ros-jazzy-nav2-amcl 1.3.11-1noble.20260412.054619`.
There is no `.cpp` on this machine — headers and shared objects only — so
every statement below is disassembly or an installed header.

**F1. AMCL does NOT consume `/model/coco/odometry`. It performs a TF
lookup. (FACT)**
`laserReceived` (`libamcl_core.so 0xe37b0`) has exactly one call to
`getOdomPose`, at `0xe3be8`. Argument set-up immediately before it:

```
e3ba6:  mov    $0x1,%edx              ; rcl_clock_type_e = 1 = RCL_ROS_TIME
e3bab:  mov    %r11,%rsi              ; &scan->header.stamp
e3bb1:  lea    0x9d8(%rbx),%r13       ; base_frame_id_
e3bb8:  call   rclcpp::Time::Time(builtin_interfaces::msg::Time const&, rcl_clock_type_e)
e3bc1:  mov    %r12,%r9               ; arg5  = that Time
e3bc4:  mov    %rbx,%rdi              ; this
e3bc7:  push   %r13                   ; arg6  = base_frame_id_
e3bd0:  lea    -0x948(%rbp),%rcx      ; arg3  = double& y
e3bd7:  lea    0x5e0(%rbx),%rsi       ; arg1  = latest_odom_pose_
e3bde:  lea    -0x940(%rbp),%r8       ; arg4  = double& yaw
e3be5:  mov    %r14,%rdx              ; arg2  = double& x
e3be8:  call   getOdomPose@plt
```

i.e. `getOdomPose(latest_odom_pose_, x, y, yaw, Time(scan->header.stamp,
RCL_ROS_TIME), base_frame_id_)`.

`r11` is the raw `LaserScan*`. `std_msgs/Header` lays out `stamp` (8 bytes)
then `frame_id`, so `r11+0` is `&header.stamp` — cross-checked by
`e3938: lea 0x8(%rax),%rsi` handing `header.frame_id` to
`strip_leading_slash` (FACT).

**F2. Source frame `base_footprint`, target frame `odom`, and the offsets are
forced by the header's declaration order. (FACT + DERIVED)**
In `getOdomPose` (`0xd6a40`): `lea 0xa88(%r12),%rcx` supplies the target
frame. Walking `amcl_node.hpp:357-386` from `base_frame_id_` at `0x9d8`
(32-byte `std::string`, 8-byte doubles, `bool` padded to alignment):

| member | offset |
|---|---|
| `base_frame_id_` | `0x9d8` |
| `global_frame_id_` | `0xa18` |
| `sensor_model_type_` | `0xa58` |
| **`odom_frame_id_`** | **`0xa88`** ✓ matches the `lea` |
| `resample_interval_` | `0xac8` ✓ independently reproduces C2-NAV.33 F3/F8 |

The arithmetic landing on **both** `0xa88` and C2-NAV.33's separately-derived
`0xac8` is the cross-check; I did not inherit either.

**F3. The lookup timeout is ZERO. (FACT)**

```
d6ba9:  pxor   %xmm0,%xmm0            ; xmm0 = 0.0
d6bad:  mov    0x480(%r12),%r15
d6bb5:  movaps %xmm4,-0x860(%rbp)
d6bbc:  movaps %xmm5,-0x850(%rbp)
d6bc3:  call   tf2::durationFromSec@plt
```

Nothing writes `xmm0` between the `pxor` and the call, so the duration is
`0.0` — the `tf2_ros::BufferInterface::transform` default. **AMCL never waits
for TF.** `transform_tolerance_` (`0xb08`) is *not* passed here; it governs
the outgoing `map → odom` broadcast, not this lookup.

> **[C2-NAV.35 note]** "AMCL never waits for TF" is corrected above: an
> upstream `tf2_ros::MessageFilter` on `/scan` does wait, bounded by
> `transform_tolerance_`, before this zero-timeout call ever runs. The
> zero-timeout finding for this specific call is unaffected.

**F4. Time semantics. (FACT/DERIVED)**
`0xd6c10: imul $0x3b9aca00,%rax,%rax` (×1e9) then `add %rdx,%rax` converts
`sec`/`nanosec` to a tf2 `TimePoint` — the lookup is at the **scan
timestamp**, not "latest". tf2 interpolates between the two bracketing
buffered transforms; with a zero timeout an unavailable or extrapolated time
throws, `getOdomPose` returns false, and the scan is **dropped entirely**
(the `e3bfa: test %r13b,%r13b` / `jne` branch) — no motion update, no
sensor update, no cloud (DERIVED from the branch structure).

**Lookup frequency:** once per `/scan` message that passes the branch, i.e.
at the lidar rate, not at `update_min_d`/`update_min_a` — those gate the
*sensor* update further down (DERIVED).

`this+0x480` is used as `tf_buffer_` via a virtual dispatch
(`mov (%r15),%rax; mov (%rax),%rsi`), which is **consistent with**
`std::shared_ptr<tf2_ros::Buffer>` at `amcl_node.hpp:166` but is not proven
by offset arithmetic the way `0xa88` is. Treat as DERIVED.

---

## 3. ACTUAL ODOMETRY PRODUCER

**F5. `diff_drive_controller`, and it is the sole publisher of that TF.
(FACT)**
`gazebo_models/urdf/coco_controllers.yaml`: `odom_frame_id: odom`,
`base_frame_id: base_footprint`, `enable_odom_tf: true`,
`publish_rate: 50.0`, `open_loop: false`.

The gz `OdometryPublisher` in `coco_robo2.xacro` is **not** a competitor:
its `<odom_frame>` is `world`, it publishes to `/model/coco/odometry`, and
the xacro's own comment states it "Publishes no ROS TF, so it cannot fight
the diff-drive controller's `odom->base_footprint` transform" (FACT).
`robot_state_publisher` publishes only the URDF's fixed/joint transforms,
which do not include `odom` (DERIVED).

**F6. Odometry integrates MEASURED wheel positions, so wheel slip is absorbed
in full. (FACT)**
`diff_drive_controller_parameters.hpp:88` — `position_feedback = true`, the
default, **not overridden** in `coco_controllers.yaml`. With
`open_loop: false` the controller calls
`Odometry::update(left_pos, right_pos, time)`
(`odometry.hpp:41`; both it and `updateOpenLoop` appear as call sites at
`libdiff_drive_controller.so 0x6eaeb` / `0x6eb31`). The integration is

```
u_l = (left_pos  - left_pos_old ) * left_wheel_radius      # ground-arc, m
u_r = (right_pos - right_pos_old) * right_wheel_radius
linear  = (u_r + u_l) / 2
angular = (u_r - u_l) / wheel_separation_
integrateExact / integrateRungeKutta2   (odometry.hpp:63-64)
```

so a wheel that rotates further than the ground moves adds **directly** to
reported distance. There is no slip term and no lateral state.

> **[C2-NAV.35 note]** This F6 finding is exactly what invalidates E5/F8's
> linear-channel claim below: since odometry integrates *measured* rotation
> rather than the command, `v_odom ≡ v_cmd` does not hold in general. See
> the C2-NAV.35 entry at the top of this file.

---

## 4. WHEEL / MODEL GEOMETRY

**F7. The nominal `wheel_separation` IS the true physical track. (FACT)**
Derived from `coco_robo2.xacro` and checked by assertion in
`c2nav34_odom.py geom` / `selftest`. `chassis_joint` carries
`rpy=(π/2,0,0)`, `xyz=(0.12,−0.08,0)`; `Rx(π/2)` maps `(x,y,z)→(x,−z,y)`:

| joint | base_link |
|---|---|
| `base_Revolute-1` front-right | `(+0.090, −0.137, +0.045)` |
| `base_Revolute-2` rear-right | `(−0.090, −0.137, +0.045)` |
| `base_Revolute-3` front-left | `(+0.090, +0.137, +0.045)` |
| `base_Revolute-4` rear-left | `(−0.090, +0.137, +0.045)` |

Physical lateral track `0.274000 m`; parameter `0.274000 m`; difference
**`0.00e+00`**. (This also reproduces the xacro's own comment at line 57.)

So `wheel_separation_multiplier: 1.10` is **not** correcting a nominal/actual
mismatch, which is what the upstream parameter is documented for
("Correction factor when the actual wheel separation differs from the nominal
value", `diff_drive_controller_parameters.hpp:612`). It is used off-label as a
skid-steer yaw compensation, exactly as the repo's own comment says.
Effective `b_eff = 0.301400 m`, `+10.0 %`.

> **[C2-NAV.35 note]** `CODEX_REVIEW.md` flags that this joint-origin track
> (0.274 m) is not necessarily the true skid-steer *contact* geometry — a
> collision-center spacing of 0.243 m is also present in the model. Effective
> skid-steer track is therefore UNKNOWN, not settled at `0.00e+00` difference
> as stated here.

**F8. THE CANCELLATION IDENTITY — this is what rejects the briefed
hypothesis. (DERIVED)**
The multiplier is applied to **both** the command path and the odometry path:

```
command   :  u_r − u_l = w_cmd · b_eff
odometry  :  w_odom    = (u_r − u_l) / b_eff  ==  w_cmd
```

**`b_eff` cancels.** The multiplier cannot by itself bias the *reported* yaw
rate — odometry hands back the commanded yaw rate whatever the multiplier is.
What it changes is the *physical* yaw the wheels are asked to produce:

```
w_true = η · w_cmd · b_eff / b_true = η · 1.10 · w_cmd
w_odom / w_true = 1 / (1.10 · η) ,  unbiased iff η = 1/1.10 = 0.9091
```

η, the skid-steer yaw efficiency, is condition-dependent and is **measured
nowhere in this repository (UNKNOWN)**. So the multiplier remains a *possible*
yaw-error source through η — but the measurement below says the yaw channel
is not where the error is.

**Linear channel (DERIVED):** `v_odom = (u_r + u_l)/2 = v_cmd` exactly — no
separation, no multiplier. Nothing cancels a longitudinal slip.

> **[C2-NAV.35 note]** The command-path symmetry above (`u_r − u_l = w_cmd ·
> b_eff`) was not verified from installed source here or in C2-NAV.35 — only
> from parameter-struct structure. `CODEX_REVIEW.md ## 35.3` reports
> obtaining and reading version-tagged upstream source that confirms both
> paths use the multiplier, but still treats the cancellation as conditional
> on measured wheels matching the commanded setpoints, not unconditional.
> **The linear-channel claim `v_odom = v_cmd` "exactly" is false in general**
> — see F6 above and the C2-NAV.35 entry.

---

## 5. KNOWN PARAMETERS

| parameter | value | source |
|---|---|---|
| `wheel_separation` | 0.274 m | `coco_controllers.yaml` |
| `wheel_radius` | 0.0585 m | ditto |
| `wheel_separation_multiplier` | 1.10 | ditto |
| `left/right_wheel_radius_multiplier` | 1.0 / 1.0 | ditto |
| `open_loop` | false | ditto |
| `position_feedback` | true (default, unset) | `..._parameters.hpp:88` |
| `enable_odom_tf` | true | `coco_controllers.yaml` |
| `publish_rate` | 50.0 Hz | ditto |
| wheel `mu1`/`mu2` | 0.7 / 0.7, isotropic | `coco_robo2.xacro` |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | `nav2_params.yaml:32` |
| `alpha1..5` | 0.2 each | `nav2_params.yaml:7-11` |
| `update_min_d` / `update_min_a` | 0.25 m / 0.2 rad | `nav2_params.yaml:37-38` |
| `odom_frame_id` / `base_frame_id` | `odom` / `base_footprint` | `nav2_params.yaml` |
| `transform_tolerance` | 0.5 s (**not** the lookup timeout, F3) | `nav2_params.yaml:36` |
| `resample_interval` | 1 | `nav2_params.yaml:32` |

---

## 6. EVIDENCE FOR / AGAINST ODOMETRY BIAS

Data: `docs/data/c2nav28_amcl.json` (committed; one row per AMCL update,
carrying GT and the AMCL estimate) and `docs/data/c2nav34_odom.json` (built
this session from the **uncommitted** 10 Hz CSVs under `.navbench/`).

The statistic is the **signed** body-frame residual per filter update,
`r = (AMCL body delta) − (GT body delta)`. Signed, so cloud noise cannot
inflate it; a constant world→map offset — including the open 56 mm `x`
discrepancy — cancels exactly, which `selftest` asserts for both a
translation and a rotation.

> **[C2-NAV.35 note]** This residual differences AMCL's *published output*
> pose against GT — it is a PROXY for odometry error, not a measurement of
> it, because `(AMCL delta) − (GT delta) = (odometry error) + (laser
> correction)`, unseparated. The "FOR"/"AGAINST" items below are correct as
> descriptive statistics; treat any causal reading of them as HYPOTHESIS,
> per the C2-NAV.35 entry.

### FOR

**E1. The along-track residual excludes zero; the yaw residual does not.
(FACT)** Pooled over 523 updates, 21 legs, 4 runs:

| component | mean / update | 95 % CI | verdict |
|---|---|---|---|
| **along-track** | **+0.00616 m** | **[+0.00342, +0.00890]** | **excludes zero** |
| yaw | +0.00271 rad | [−0.00647, +0.01190] | straddles zero |

**This inverts the briefed hypothesis.** The error is translational.

**E2. It is positive in 18 of 21 legs and reproducible per place. (FACT)**
Along-track excess as a fraction of GT travel: `open_space`
**10.1 / 10.0 / 11.1 / 8.2 %** (4 of 4 runs), `corridor_gate` 5.8 / 6.9 / 3.6 %,
`obstacle_corner` 5.9 / 5.7 / 3.7 %. Pooled mean **+7.68 %**.
The one consistent negative is `wall_parallel` (−3.6 / −0.9 / −4.1 %), which
is also the only leg that travels **east** rather than north or south.

**E3. The world-frame error points ALONG the course, and that is what makes
it place-linked. (FACT)** Forward projection positive in **16 of 21** legs,
mean **+0.0377 m**, CI `[+0.0011, +0.0743]`; lateral projection **−0.0189 m**,
CI `[−0.0594, +0.0215]` — straddles zero.

`open_space` is the clean case, 4 of 4 runs: course **−89.5 / −89.2 / −90.9 /
−89.6°** (due south), error almost purely along-track —
`proj_fwd` **+0.117 / +0.104 / +0.118 / +0.099 m** against `proj_lat`
−0.004 / −0.010 / +0.044 / −0.003 m. And `enclosure_entry`, which heads
**north** (+110 / +106 / +108°), carries a **northward** error
(+0.197 / +0.182 / +0.055 m).

**E4. It is created where the bias is created. (FACT)** At `open_space` the
world-frame y error starts at **+0.004 / +0.001 / +0.016 / +0.013 m** and ends
at **−0.113 / −0.103 / −0.103 / −0.086 m** — 4 of 4 runs. `wall_adjacent`
then *inherits* it (starts −0.113 / −0.071 / −0.107 / −0.100). This matches
C2-NAV.30's focus run, whose first `open_space` cloud was effectively a point
mass at dy +0.0119 m.

**E5. The command channel alone already forces a lower bound. (FACT)**
Steady windows (`v_wheel` held ≥ 0.5 s, |`v_wheel`| ≥ 0.1 m/s, robot moving,
collision monitor not gating): slope `v_act / v_wheel` = **0.9712**, n = 253.
Since `v_odom ≡ v_cmd` (F8), odometry over-reports forward distance by
**≥ 2.96 %** — and that captures *only* the controller's velocity-tracking
shortfall. Wheel slip (F6) adds on top and is invisible to every artefact
here.

> **[C2-NAV.35 note] This claim is INVALID, not FACT.** `v_wheel` here is
> `/diff_drive_controller/cmd_vel` — a **command**, not an encoder reading.
> Under F6 (measured-feedback odometry), a controller-tracking shortfall
> (wheels physically turn slower than commanded, with no ground slip) is
> read correctly by the encoders and produces **zero** odometry error — see
> the decisive counterexample in the C2-NAV.35 entry above. The "≥2.96 %"
> figure and the "70 % of the bias" claim in §8 below do not stand.

**E6. The surviving direction is the unobservable one. (FACT, corroborating)**
C2-NAV.31 measured the 99 % likelihood plateau at `wall_adjacent` as 0.110 m
wide spanning **[−0.085, +0.060] m in y** — and `wall_adjacent` runs due
south, so **y is the along-track axis there**. An along-track error is
precisely the one the observation model cannot correct (the corridor aperture
problem), which is why it survives while C2-NAV.29–.33 found every
cross-track mechanism restoring.

### AGAINST / ELIMINATED

**E7. The heading-error integral does NOT generate the position error.
(FACT)** Integrating `ė = e_ψ · (−v_y, +v_x)` against GT velocity and the
*observed* heading error reproduces the observed y error within a factor of 2
and correct sign in only **2 of 21** legs. At `open_space` it predicts
**−0.006 / +0.013 / +0.006 / +0.044 m** against an observed **≈ −0.10 m**.
The bias is not heading-propagated.

**E8. Lateral-skid blindness is real but points the WRONG WAY. (FACT)**
A differential-drive integrator has no lateral state, and the robot genuinely
slides: total |lateral| **4.434 m over 52.697 m of path = 8.4 %**, rising to
**14–33 %** at `wall_adjacent`. But the world-frame term it must accumulate,
`−∫ v_lat n̂ dt`, agrees in sign with the observation in only **2 of 21**
legs and is ~5× too small (`open_space`: +0.022 / +0.016 / +0.023 / +0.020 m
against −0.10). Eliminated as the principal mechanism.

**E9. The `wheel_separation_multiplier` cannot bias reported yaw. (DERIVED,
F8)** And the measured yaw residual straddles zero (E1). The briefed
mechanism is **rejected as stated**. It survives only indirectly, through η.

> **[C2-NAV.35 note]** "Rejected as stated" overstates F8 (see the F8 note
> above) — reclassify as conditionally supported, not falsified. The
> empirical conclusion (yaw residual consistent with zero) is unaffected.

### THE HONEST GAP

`r = (odometry error) + (laser correction)`. E1–E6 do **not** prove the
odometry is biased; they prove **AMCL's estimate advances along-track faster
than the robot does**. Attributing that to odometry requires the assumption
that the laser correction is restoring rather than driving — supported by
C2-NAV.33's `corr(shift, dy_u) = −0.71` and by C2-NAV.29's 90/90 GT-scores-
higher, but it is an assumption, and it is why section 9 exists.

---

## 7. QUANTITATIVE BIAS ESTIMATE

| quantity | value | tag |
|---|---|---|
| along-track residual, per update | +0.00616 m, CI [+0.00342, +0.00890] | FACT |
| along-track residual, per leg | +7.68 % of GT travel (mean, 21 legs) | FACT |
| at `open_space` specifically | +9.87 % (10.1/10.0/11.1/8.2) | FACT |
| yaw residual, per update | +0.00271 rad, CI straddles zero | FACT |
| net world along-track error, `open_space` | +0.099 … +0.118 m over 2.13 m travelled = **4.6–5.5 %** | FACT |
| command-channel lower bound on odometry over-report | ~~≥ 2.96 %~~ — **INVALID, see C2-NAV.35** | ~~FACT~~ |
| unexplained span between the two | ≈ 4.7 percentage points | DERIVED |
| skid-steer yaw efficiency η | — | UNKNOWN |
| true `odom → base_footprint` error | — | **UNKNOWN, unrecorded** |

---

## 8. CAN THAT BIAS EXPLAIN ~0.09 m?

**Yes, on the arithmetic, and with room to spare. (DERIVED)**

`open_space` runs **due south for 2.13 m**. A pure along-track gain error
`ε` lands as a southward y error of `2.13 · ε`:

| ε | y error at end of `open_space` |
|---|---|
| 2.96 % (command-channel bound, E5) | **−0.063 m** |
| 4.2 % | −0.090 m ← the named bias |
| 5.2 % (the observed net, E3/E4) | **−0.111 m** |
| 7.68 % (the residual mean, E2) | −0.164 m |

The named bias is **−0.09 to −0.13 m southward**. The measured net along-track
error at `open_space` is **+0.099 to +0.118 m forward on a −90° course**,
i.e. **−0.099 to −0.118 m in y** — the bias, in magnitude, in sign, and in
axis, without fitting anything. And the command channel *alone*, before any
wheel slip, already supplies **70 %** of it.

`wall_adjacent` then inherits the offset rather than creating it (E4), which
is why C2-NAV.28 saw the bias appear *before* terminal yaw and why C2-NAV.30's
cloud was already displaced in its first observable update.

**Separating absolute odometry bias from AMCL's response (as briefed):** the
absolute odometry bias is **UNKNOWN** — unrecorded. AMCL's response to it is
partly measured: the filter carries ~5.2 % of the ~7.7 % residual through to
its published pose at `open_space`, i.e. the laser removes roughly a third
and cannot remove the rest because the residual lies along the plateau axis
(E6).

> **[C2-NAV.35 note]** The 2.96 %-based rows above ("command-channel bound"
> and "the command channel alone... supplies 70 % of it") are INVALID — see
> §7 and the C2-NAV.35 entry. The 4.2 %/5.2 %/7.68 % scale arithmetic itself
> is still valid *conditional on* an odometry gain of that size actually
> existing, which remains unmeasured (`CODEX_REVIEW.md`: "SUPPORTED as
> conditional scale arithmetic... the 4.2 % odometry gain is unmeasured").

---

## 9. REQUIRED INSTRUMENTATION

**Diagnostic proposal — NOT implemented. No file under `gazebo_models/` was
modified this session.**

The minimum is one `tf2_ros::TransformListener` inside `nav_bench.py`,
recording the transform AMCL itself queries.

| appended column | source |
|---|---|
| `odo_x`, `odo_y`, `odo_yaw` | `lookup_transform('odom', 'base_footprint', t)` |

That alone is sufficient to prove or reject, because `ω_odom ≡ ω_cmd` and
`v_odom ≡ v_cmd` (F8) make the TF the complete statement of what AMCL was
told. Follow C2-NAV.28's and C2-NAV.30's appending rule **exactly**: columns
appended at the end so every existing index is unchanged; last sample in the
half-open bucket `(t−0.1, t]`; **blank when the bucket is empty, never a
forward fill**.

**The blindness guard is mandatory and must be POST-run.** CLAUDE.md: any
check whose success condition is "we saw nothing" must first prove it can see
something. A TF listener that never matches yields blank columns that read
exactly like "the odometry did not drift". C2-NAV.30 already paid for the
pre-run version of this. Assert ≥ 1 non-blank `odo_*` row per leg *after* the
run and report UNOBSERVABLE otherwise.

**Optional second group, only if attribution is wanted** — `/joint_states`
wheel positions, which would split the 2.96 % controller shortfall from the
wheel-slip remainder (E5). Not required to answer the question; do not let it
delay the run.

`ros_clean.sh` needs **no** new pattern: this is a subscription inside an
existing node, not a new process. Stated because CLAUDE.md requires the
check, not because it fires.

> **[C2-NAV.35 note]** This three-column proposal is a reasonable cheap
> cross-check (labelled "Layer 1" in the C2-NAV.35 entry above) but is now
> assessed as **insufficient alone** to prove exact consumption — it cannot
> establish which scans AMCL actually accepted or the literal anchor state
> used. See §8 of the C2-NAV.35 entry for the fuller minimum (an in-process
> hook), which both this session and `CODEX_REVIEW.md ## 35.5` converge on
> independently.

---

## 10. PROPOSED LIVE TEST

One tour, **fresh simulator**, topology A, `open_space` → `wall_adjacent`,
same route and 75 s cap as the C2-NAV.28/.30/.33 focus runs, under
`docs/data/c2nav25_slow_params.yaml`, so it is directly comparable to
`c2n30_focus_r1` and `c2n33_focus_r1`.

**Exact parameter change: NONE. Not one leaf.** A `paramdiff` over all 323
leaves must report added 0, removed 0, changed 0, shown *before* the
simulator starts. `resample_interval: 2` is **not** carried forward — it
halves the `/amcl_pose` rate (C2-NAV.33) and is a diagnostic window, not a
configuration.

**Analysis.** Per AMCL update, along-track:

```
odo_along_cum = along-track travel integrated from odo_* since leg start
gt_along_cum  = along-track travel integrated from GT      since leg start
excess_odo    = odo_along_cum / gt_along_cum − 1
excess_amcl   = (already measured this session: +9.87 % at open_space)
```

`open_space` is the discriminating leg: it heads due south, it is where the
displacement is *created* rather than inherited (E4), and its first cloud in
`c2n30_focus_r1` was effectively a point mass — so the replay must reproduce
the creation, not carry an inherited offset.

> **[C2-NAV.35 note]** Superseded by the C2-NAV.36 proposal in the C2-NAV.35
> entry above, which adds the in-process hook rather than relying solely on
> the three appended `odo_*` columns.

---

## 11. FALSIFIER

Fixed in advance. The discriminator is a **comparison**, not a threshold on
one number.

- **FALSIFIED** if, over `open_space`, `|excess_odo| < 2 %` while
  `excess_amcl` again reaches ~10 %. The odometry then cannot be supplying
  the along-track advance and joins the eliminated list — and the bias
  becomes genuinely unexplained by any term yet examined, which is itself the
  finding.
- **SURVIVES** (not confirmed — one run) if `excess_odo ≥ 4 %` **and the sign
  is positive**. Sign agreement is required: C2-NAV.32's motion-model replay
  was rejected partly for pointing north, and the same standard applies.
- **Between 2 % and 4 %:** report as INDETERMINATE at n = 1. Do not round
  into either bucket.
- **UNOBSERVABLE, a distinct outcome** — reported as such, never as "the
  odometry is clean" — if the post-run guard finds no non-blank `odo_*` row,
  or TF lookups fail on more than 10 % of buckets. C2-NAV.33 hit exactly this
  trap (its verdict printed `(C) UNOBSERVABLE` on a missing artefact); the
  same must not be silently folded into a finding here. `load34()` in
  `c2nav34_odom.py` already `sys.exit`s with a distinct message rather than
  returning an empty result, for the same reason.

**Null control, required.** Replay the analysis with `odo_*` set equal to
ground truth; `excess_odo` must be 0 at machine epsilon. This session's
equivalent already passes: `selftest` reports worst |along| **0.000e+00 m**
over 21 legs, and a synthetic +8.000000 % injection is recovered as
**+8.000000 %**.

---

## 12. LIMITATIONS

1. **The odometry is still not measured.** Everything in section 6 is
   `odometry error + laser correction`, unseparated. This handoff narrows
   *which axis* to look at; it does not close the question.
2. **n = 4 runs, one topology, one route set.** `open_space` is 4 of 4 and
   `enclosure_exit` is n = 1. Legs are not independent — a tour carries error
   forward across leg boundaries, so per-leg values are not 21 independent
   samples and the pooled CI is optimistic.
3. **`docs/data/c2nav34_odom.json` is derived from uncommitted CSVs** under
   `.navbench/results/`, which are not in the repository. The bundle is
   committed so the numbers survive; `build` will not re-run without those
   CSVs. Modes `incr` and `verdict` need only the committed C2-NAV.28 bundle.
4. **Three rows carry a ground-truth twist glitch** — `w_act ≈ 314.10 rad/s`
   (= 100π) at `wall_adjacent` t = 5.0–5.1 s in one run. `v_act` is clean
   (0 rows > 2 m/s). No number reported here uses `w_act`: the position
   analyses differentiate `x`,`y`,`yaw`, and `cmd` uses `v_act`. Flagged
   because it is a real defect in `/model/coco/odometry`.
5. **`this+0x480` as `tf_buffer_` is DERIVED**, not proven by offset
   arithmetic the way `odom_frame_id_` at `0xa88` is.
6. **`open_space`'s point-mass first cloud may not recur** — it was a
   property of `c2n30_focus_r1` beginning just after AMCL initialised, not of
   the route. If the new run's first cloud is already displaced the leg loses
   discriminating power. Record it and say so; do not re-run until it comes
   out convenient.
7. **The `WORLD_TO_MAP` 56 mm x discrepancy is open for a sixth session.** It
   cannot affect anything here — every quantity is a difference of two poses
   in one frame, and `selftest` asserts a constant offset cancels — but it
   still needs a decision rather than a patch.
8. **C2-NAV.30's selftest allow-list still fails** on files from
   C2-NAV.31/.32/.33 and will now fail on C2-NAV.34's too. Pre-existing,
   recorded twice, not caused here. It needs widening or retiring.
9. ~~**`docs/agents/CODEX_REVIEW.md` does not exist** in this worktree.~~
   **[C2-NAV.35 note]** It now does, as of the fast-forward to `eff6dd5`
   described at the top of this file. Read and incorporated above.

---

## 13. CONFIDENCE

| claim | confidence | basis |
|---|---|---|
| AMCL's motion input is a zero-timeout TF lookup of `odom → base_footprint` at the scan stamp | **High** | Disassembly of the deployed `.so`; `odom_frame_id_` offset independently reproduces C2-NAV.33's `resample_interval_` |
| `b_eff` cancels, so the multiplier cannot bias reported yaw | ~~High~~ **Moderate — see C2-NAV.35** | Algebra over the two documented paths, command-path symmetry now corroborated against upstream source, but conditional on unverified perfect wheel-tracking |
| The nominal wheel separation equals the true physical track | ~~High~~ **Moderate — see C2-NAV.35** | Joint-origin spacing matches; collision-center spacing (0.243 m) does not, per `CODEX_REVIEW.md` |
| The residual is along-track, and the yaw channel measures zero | **High** for the sign and axis, **Moderate** for the magnitude | Signed statistic, null control at exactly 0, synthetic injection recovered exactly; but 4 runs and non-independent legs |
| Heading propagation and lateral-skid blindness are eliminated | **Moderate–High** | Heading integral matches in 2 of 21 legs; skid blindness agrees in sign in 2 of 21 and is ~5× too small |
| An along-track odometry over-report of the measured scale would produce −0.09…−0.13 m at `open_space` | **High** (arithmetic, conditional on the gain existing) | 2.13 m due south × 4.6–5.5 %; the gain itself is unmeasured |
| **That the odometry is in fact the source** | **Low — this is a HYPOTHESIS** | The measured residual folds in the laser correction, and the odometry is recorded nowhere. This is the whole point of section 9 — and of C2-NAV.35 |
| The briefed `wheel_separation_multiplier` mechanism as stated | ~~Rejected~~ **Conditionally supported — see C2-NAV.35** | F8 plus a yaw CI straddling zero, but F8 itself is conditional, not unconditional |

---

## 14. EXACTLY ONE NEXT ACTION

~~**Add the three `odo_x`/`odo_y`/`odo_yaw` columns of section 9 to
`nav_bench.py`**~~ — **superseded, see C2-NAV.35 §8–§9**: implement and
validate offline the default-disabled AMCL-internal consumption hook (plus a
separate synchronized GT sidecar) before any live experiment. The three
appended columns remain useful as a cheap corroborating cross-check, but are
not sufficient alone.

---

## FOR CODEX — the claims most worth attacking

Stated so they can be checked rather than accepted:

1. **The cancellation identity (F8).** If `wheel_separation_multiplier` does
   *not* scale the command path as well as the odometry path in
   `diff_drive_controller` 4.39.0, the whole rejection of the briefed
   hypothesis is wrong. I read this from the parameter semantics and the
   `Odometry::update` equations, **not** from `diff_drive_controller.cpp`,
   which is not installed. Exact files:
   `/opt/ros/jazzy/include/diff_drive_controller/diff_drive_controller/odometry.hpp:41-52`,
   `.../diff_drive_controller_parameters.hpp:78-88,612`,
   `/opt/ros/jazzy/lib/libdiff_drive_controller.so` (`0x6eaeb`, `0x6eb31`).
   **[C2-NAV.35: addressed — `CODEX_REVIEW.md ## 35.3` obtained the
   version-tagged upstream source and confirms both paths use the
   multiplier; cancellation is conditional on measured-vs-commanded
   equality, which is separately not established.]**
2. **The zero timeout (F3).** `pxor %xmm0,%xmm0` at `libamcl_core.so 0xd6ba9`,
   call at `0xd6bc3`. If any instruction between them writes `xmm0`, F3 is
   wrong. **[C2-NAV.35: not disputed by Codex; stands.]**
3. **The offset arithmetic** from `base_frame_id_ 0x9d8` to
   `odom_frame_id_ 0xa88`, `amcl_node.hpp:357-386`. It assumes 32-byte
   `std::string` and natural alignment. If that is wrong, so is the frame
   identification — though the independent landing on `0xac8` argues it is not.
   **[C2-NAV.35: not disputed by Codex; corroborated against downloaded
   upstream source.]**
4. **That one row of `docs/data/c2nav28_amcl.json` is one AMCL update.** I
   inferred this from `write_trace`'s documented `bucket_last` + blank rule
   (`nav_bench.py:1428-1450, 1575-1580`) plus the observation that every row
   has a distinct non-blank AMCL pose and `dt` is a variable multiple of
   0.1 s. If the bundle instead forward-fills, the per-update statistics are
   wrong (the per-leg sums and the world-frame endpoints are not).
   **[C2-NAV.35: `CODEX_REVIEW.md` flags this is a proxy for prediction
   calls, not a proven one-row-per-update identity — see §5/§6 above.]**
5. **That the laser correction is restoring**, which is what licenses reading
   the residual as a lower bound on odometry error. Inherited from C2-NAV.33
   (`corr = −0.71`) and C2-NAV.29 (90/90), and measured on the *cross-track*
   centroid — **not** on the along-track axis this session is about. This is
   the weakest link in the chain and I have not strengthened it.
   **[C2-NAV.35: still the weakest link; unresolved.]**
6. **`.navbench/results/` is uncommitted**, so `build` is not reproducible
   from a fresh clone. Check `docs/data/c2nav34_odom.json` against the CSVs
   if they still exist on this machine. **[C2-NAV.35: not re-checked this
   session; still open.]**
