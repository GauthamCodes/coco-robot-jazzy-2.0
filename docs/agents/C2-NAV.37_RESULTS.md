# C2-NAV.37 — live odometry-input capture: RESULTS

Live measurement run executed per `docs/agents/C2-NAV.37_PRE_FLIGHT.md`, on
top of the fix in commit `efde9a0`. This document reports what was
measured. It does not tune anything and does not modify navigation
behavior; see `PROJECT_STATE.md` §KNOWN LIMITATIONS for the pre-existing,
unrelated localization-recovery gap.

## Configuration used (verbatim, unmodified)

- Executable: `coco_nav_diag`'s `amcl_diag`, built from commit `efde9a0`
  inside this worktree (`install/coco_nav_diag/lib/coco_nav_diag/amcl_diag`),
  **plus one additive change made during this session** (see "Deviation
  from the pre-flight checklist" below): `coco_nav_diag::AmclNode` is now
  also registered as a loadable `rclcpp_components` plugin
  (`rclcpp_components_register_nodes(amcl_diag_core "coco_nav_diag::AmclNode")`
  added to `coco_nav_diag/CMakeLists.txt`). No AMCL/motion-model/recorder
  logic changed; `coco_nav_diag/src/amcl_node.cpp` and `diag_recorder.*`
  are byte-identical to `efde9a0`.
- Params: `gazebo_models/config/nav2_params.yaml`, unmodified. AMCL was
  loaded with its full 42-parameter `amcl:` block read directly from that
  file (script-generated, not hand-transcribed — see below), plus
  `use_sim_time:=true` and the three `diag_*` overrides.
- Map/world: `gazebo_models/maps/coco_world.yaml` /
  `gazebo_models/worlds/coco_world.world`, unmodified.
- Bringup: `ros2 launch gazebo_models full_world_robo.launch.py gui:=false`
  then `ros2 launch gazebo_models nav.launch.py arbiter:=false
  params_file:=gazebo_models/config/nav2_params.yaml` (topology A, matching
  every historical nav_bench baseline).
- Benchmark: `gazebo_models/scripts/nav_bench.py --tag c2nav37_run<N>
  --repeats 3 --timeout 75 --out ~/coco_nav_runs/c2nav37_run<N>/`, the
  committed 7-leg `TOUR`, no `--goal`/`--only`/`--leg-timeout` overrides.
  `--repeats 3` gives the pre-flight doc's own specified "21 legs/run".
- Ground truth: `docs/data/c2nav36_gt_sidecar.py`, subscribing
  `/model/coco/odometry` (gz `OdometryPublisher`, `world -> base_footprint`,
  ~48-50 Hz).
- Build/run happened entirely inside this worktree (self-contained
  overlay), never touching `~/ros2_ws`'s main install.

## Deviation from the pre-flight checklist (and why)

The checklist's substitution procedure (`pgrep -af 'lib/nav2_amcl/amcl'`,
SIGTERM the stock `amcl` process, start the `amcl_diag` **executable** in
its place, replay lifecycle transitions) assumes AMCL runs as a standalone
OS process. **It does not, under this repo's own `nav.launch.py`.**
`bringup_launch.py`'s default composition loads all 14 Nav2 nodes —
`map_server`, `amcl`, `controller_server`, `planner_server`,
`bt_navigator`, both lifecycle managers, etc. — into one shared
`component_container_isolated` process (`/nav2_container`). Confirmed
live: `pgrep -af amcl` found nothing; `ros2 component list` showed `/amcl`
as component #3 of 14 in that one container. This is not specific to this
run — `nav.launch.py` never forwards a `use_composition` argument to
`bringup_launch.py`, so every historical nav_bench baseline through this
launch file used the same composed topology.

Killing the container would have taken all 14 nodes down at once (not a
clean single-node swap, and would have immediately tripped the checklist's
own "map_server also vanished" STOP condition). The class the checklist
needed was already present but not wired up:
`RCLCPP_COMPONENTS_REGISTER_NODE(coco_nav_diag::AmclNode)` was already in
`amcl_node.cpp` (carried over from the `nav2_amcl` fork), but
`CMakeLists.txt` never called the CMake-side
`rclcpp_components_register_node(s)` macro that writes the ament
resource-index entry `ros2 component load` actually reads. **The fix
added is one line**: `rclcpp_components_register_nodes(amcl_diag_core
"coco_nav_diag::AmclNode")`, using the plural (index-only) form so it adds
no second executable — `main.cpp`'s existing standalone `amcl_diag` is
untouched. Rebuilt and reconfirmed: 36/36 tests still pass (including the
`efde9a0` regression test), and a throwaway component-container smoke
test round-tripped load → param readback → unload cleanly before this was
trusted for the live runs.

The live substitution actually used, every run: `ros2 component unload
/nav2_container <amcl's id>` → immediate STOP-check (`ros2 component
list`, confirm the other 13 unaffected, confirm `/map_server` still
`active`) → `ros2 component load /nav2_container coco_nav_diag
coco_nav_diag::AmclNode` with all 42 `amcl:` parameters from
`nav2_params.yaml` (read and forwarded by a small script,
`amcl_component_load.py`, precisely to avoid hand-transcription drift —
verified against a throwaway container first: every parameter type,
including the nested `initial_pose.{x,y,z,yaw}`, came back correctly typed
on readback) plus `use_sim_time:=true` and the three `diag_*` overrides →
`ros2 lifecycle set /amcl configure` → `activate` → mandatory
`diag_enabled`/`diag_output_path` readback. This is a genuine clean
single-node swap: at no point did any node other than `/amcl` change
identity, and `ros2 component unload`/`load` produced no bond-manager
disruption (unlike the SIGTERM the checklist anticipated, which was never
actually exercised, since there was no separate process to send it to).

One other environment note, unrelated to navigation: `custom_teleop`'s
`cmd_vel_relay` initially crashed on startup
(`PackageNotFoundError: No package metadata was found for custom-teleop`)
because this worktree's editable-install `.egg-info` lives under
`build/custom_teleop/`, not on the interpreter's default path in this
session's execution environment. Fixed by adding that directory (and
`build/coco_config`) to `PYTHONPATH` before any run; confirmed via
`importlib.metadata.distribution('custom-teleop')` resolving correctly,
then re-verified `cmd_vel_relay` actually running in every subsequent
bringup. This is a session/environment artifact, not a repository change.

## Run IDs, status, artifacts

| Run | `diag_enabled` | Purpose | Legs (SUCCEEDED/TIMEOUT/other) | Artifacts |
|---|---|---|---|---|
| 0 | false | control / lifecycle de-risk | 16/5/0 | `~/coco_nav_runs/c2nav37_run0/` (`gt.csv`, `console_navbench.log`; no `diag.jsonl` — recorder disabled) |
| 1 | true | measurement | 17/4/0 | `~/coco_nav_runs/c2nav37_run1/` (`diag.jsonl`, `gt.csv`, `joined.csv`) |
| 2 | true | measurement | 16/5/0 | `~/coco_nav_runs/c2nav37_run2/` (same set) |
| 3 | true | measurement | 17/4/0 | `~/coco_nav_runs/c2nav37_run3/` (same set) |
| 4 | true | measurement | 17/4/0 | `~/coco_nav_runs/c2nav37_run4/` (same set) |

All 5 runs: fresh Gazebo per run, exit code 0, zero orphan processes after
teardown (verified via `pgrep` for `gz sim`, `component_container`,
`parameter_bridge`, `cmd_vel_relay`, `amcl_diag`, `c2nav36_gt_sidecar`
after every run), zero dropped diagnostic events (the
`DiagRecorder`'s "dropped N events" warning never appeared in any run's
console log — absence is the pass condition), zero dropped GT rows
(`c2nav36_gt_sidecar` reported `dropped 0` in all 5 runs). No leg ever
returned `FAILED` or `ABORTED` in any run — outcomes were exclusively
`SUCCEEDED` or `TIMEOUT`.

**TIMEOUTs were concentrated on exactly two legs, in every run, and never
occurred on any other leg**: `wall_adjacent` (2,1,2,1,1 = 7/15 attempts
across the 5 runs) and `enclosure_entry` (3,3,3,3,3 = **15/15 attempts,
every single rep of every single run**). `open_space`, `wall_parallel`,
`obstacle_corner`, `corridor_gate`, and `enclosure_exit` never timed out.
This is not a new finding — `wall_adjacent` ("0.35 m from the south wall:
inside inflation, outside inscribed") and `enclosure_entry` ("0.63 m NW
pinch, 0.30 m free band") are the two scenarios the `TOUR`'s own comments
already flag as marginal — but the *consistency* (`enclosure_entry`
timing out in literally every rep) is itself new information about the
current frozen `nav2_params.yaml` under a 75 s cap; it is reported here as
an observation, not investigated further, per this task's scope.

## Diagnostic completeness

| Run | motion-update candidates | correlated | unobservable | reason |
|---|---|---|---|---|
| 1 | 533 | 532 | 1 | GT capture doesn't bracket that one update in time |
| 2 | 599 | 598 | 1 | same |
| 3 | 582 | 581 | 1 | same |
| 4 | 563 | 562 | 1 | same |

Every run: `status: OK` (not `UNOBSERVABLE`), well above the ~50-row
minimum the pre-flight doc sets for trusting a run. The single
`unobservable` row per run is a boundary effect (GT sidecar started/ended
a fraction of a second after/before AMCL's first/last update of the
run) — expected, not a capture defect.

## Residual statistics (pre-registered recipe)

**`pandas` is not installed on this machine** and could not be added
without a system-level change (`pip install --user` is blocked by
PEP 668's externally-managed-environment guard on this Python; no
`python3-venv` for 3.12 is installed; installing either was outside this
task's scope to do unilaterally). The recipe was reimplemented with only
the Python standard library, reproducing `mean()`/`median()`/`quantile()`
(linear interpolation, pandas/numpy's default) and `sign_frac_positive`
exactly — verified by construction, not by re-deriving a different
statistic. The recipe's logic is otherwise unaltered.

Per run (n = correlated rows):

| Run | residual_along mean | median | p2.5 | p97.5 | sign_frac_pos |
|---|---|---|---|---|---|
| 1 (n=532) | +0.000810 | +0.000461 | -0.003038 | +0.005428 | 0.650 |
| 2 (n=598) | +0.000720 | +0.000480 | -0.002926 | +0.005338 | 0.620 |
| 3 (n=581) | +0.000693 | +0.000417 | -0.003062 | +0.005678 | 0.606 |
| 4 (n=562) | +0.000773 | +0.000442 | -0.003049 | +0.005287 | 0.607 |
| **Pooled (n=2273)** | **+0.000747** | +0.000445 | -0.003022 | +0.005389 | 0.620 |

residual_lat pooled: mean -0.002104, median -0.001432, p2.5/p97.5
[-0.019745, +0.017836], sign_frac_pos 0.380.
residual_yaw pooled: mean +0.010160, median +0.005090, p2.5/p97.5
[-0.076979, +0.075201], sign_frac_pos 0.612.

**Sign consistency across all 4 measurement runs**: residual_along is
positive in all 4 runs (range +0.000693 to +0.000810 — a narrow, stable
band); residual_lat is negative in all 4; residual_yaw is positive in all
4. Full agreement on sign in every run, every dimension.

**A statistical note on "CI excludes zero."** The recipe's own `p2.5`/
`p97.5` columns are quantiles of the *raw per-update residual
distribution* (individual-update noise), not a confidence interval on the
*mean* — and by that raw-distribution reading, the pooled interval
[-0.003022, +0.005389] straddles zero. C2-NAV.34's own comparison figure
(+0.00616 m/update, CI [+0.00342, +0.00890]) is visibly a CI on the mean
(far tighter than its own per-update spread would be), so a literal
"pre-flight-recipe-p2.5/p97.5 vs. C2-NAV.34-CI" comparison would be
comparing two different statistical objects. To resolve this
honestly rather than pick whichever reading is convenient, a standard
normal-approximation 95% CI on the mean was computed as a **supplementary,
clearly-separate calculation** (not a substitution for the pre-registered
recipe, which is reported unaltered above):

| | mean | 95% CI of the mean | excludes zero |
|---|---|---|---|
| Run 1 | +0.000810 | [+0.000618, +0.001003] | yes |
| Run 2 | +0.000720 | [+0.000542, +0.000898] | yes |
| Run 3 | +0.000693 | [+0.000510, +0.000877] | yes |
| Run 4 | +0.000773 | [+0.000585, +0.000960] | yes |
| Pooled (n=2273) | +0.000747 | [+0.000655, +0.000840] | yes |

By this measure, the mean is cleanly distinguishable from zero, in every
run and pooled — consistent with the sign-consistency already observed.
**Both readings are reported; the magnitude test below does not depend on
resolving between them.**

## Magnitude comparison against the existing proxy bias — the decisive test

The pre-registered rule requires more than "differs from zero": magnitude
and sign must be quantitatively compatible with the existing ~0.09-0.13 m
southward `/amcl_pose`-vs-GT bias, via C2-NAV.34's own per-update **output**
proxy figure of **+0.00616 m/update** (which that analysis already
connected to the 0.09-0.13 m position bias).

**Measured here: the exact odometry input AMCL consumes carries a
per-update along-track residual of +0.000747 m/update (pooled) — about
12% of C2-NAV.34's own output-proxy figure, and about an order of
magnitude too small to be the sole or primary explanation for the
observed 0.09-0.13 m position bias.**

If odometry-input bias were the dominant driver of AMCL's output bias, the
input-side residual measured directly here would need to be comparable in
magnitude to (or larger than, before AMCL's own correction step partially
cancels it) the output-side residual C2-NAV.34 measured. Instead it is
roughly 8x smaller. This does not mean odometry contributes nothing — the
residual is real, signed, and consistent across 4 independent runs — but
it is not big enough, on its own, to account for the bias whose
explanation this whole investigation chain has been chasing since
C2-NAV.27.

## GT / AMCL-output comparison

This capture measured AMCL's *input*, not its *output* — no `/amcl_pose`
was recorded in these runs (that comparison is what produced the existing
proxy figure in C2-NAV.34, from data already on record; re-deriving it was
out of scope here). The two are connected only via the magnitude
comparison above, not a direct joint measurement in this session.

## Timing-sanity observations (sanity gate, not a proof)

Historical per-leg timing/outcome data usable as a byte-identical
baseline was not actually available: tracing `docs/data/c2nav28_matrix.sh`
(the script that produced the closest prior "21-leg/4-run"-scale dataset)
showed it ran against `docs/data/c2nav25_slow_params.yaml` — a **different**
params file than the current `gazebo_models/config/nav2_params.yaml`, and
one its own header comment says was explicitly "rejected as a FINAL
configuration" by C2-NAV.26. No byte-identical historical run against the
*current* frozen `nav2_params.yaml` was found to compare against
numerically. This is reported as a gap, not glossed over: **the timing
comparison below is internal (across this session's own 5 runs), not
against an external historical baseline.**

Internally: all 5 runs (the diag-disabled control and the four
diag-enabled measurement runs) show closely matching leg-timing
distributions and an outcome pattern so consistent it is arguably the
main timing finding — TIMEOUT rate 76.2-81.0% success per run (16-17
SUCCEEDED of 21), concentrated on the same two legs in every run, never on
any other leg. No material divergence between the control run (0) and the
measurement runs (1-4) was observed in either mission time or leg outcome
pattern. This supports (does not prove) that `diag_enabled` does not
change AMCL's timing behavior in a way visible at this resolution — no
stronger claim than that is made.

## Pre-registered criterion outcome

1. Residual excludes zero — **met** (SEM-based CI on the mean, all 4 runs
   individually and pooled; the recipe's own raw-distribution p2.5/p97.5
   does not exclude zero, see note above).
2. Sign-consistent across all 4 measurement runs — **met**, unambiguously,
   for all three residual dimensions.
3. Magnitude/sign quantitatively compatible with the ~0.09-0.13 m proxy
   bias — **not met**. The measured input residual is ~8x smaller than
   C2-NAV.34's own output-side per-update figure that was already shown to
   account for that bias.

## Final classification

**B — ODOMETRY CONTRIBUTES BUT IS INSUFFICIENT.**

A real, signed, sign-consistent, statistically-distinguishable-from-zero
discrepancy exists between what AMCL's motion model actually consumes as
odometry input and ground truth. It is not large enough, by itself, to
account for the ~0.09-0.13 m southward `/amcl_pose`-vs-GT bias this
investigation chain has been chasing. This is neither A (the magnitude
test fails) nor C (the discrepancy is real and repeatable, not absent or
noise-level) nor D (both diagnostic capture and GT capture were
unambiguously `OK` in all 4 runs, well above the minimum-evidence bar).

## What was directly measured / what was inferred / what remains unknown

**Directly measured:**
- The exact scan-stamped `odom -> base_footprint` transform and
  motion-model deltas (`delta_rot1`, `delta_trans`, `delta_rot2`) AMCL's
  own motion model consumed, for 2273 correlated updates across 4 fresh
  simulator runs.
- Ground truth pose (`world -> base_footprint`) at ~48-50 Hz across the
  same runs, via a topic subscription independent of AMCL/TF.
- The resulting body-frame along-track/lateral/yaw residual between that
  input and ground truth: sign-consistent, small, real.
- That the substitution procedure (component unload/load, not the
  SIGTERM-and-restart the checklist assumed) produces no detectable
  disruption to the other 13 Nav2 nodes or their lifecycle bonds, across
  5 independent trials.
- That no diagnostic event was ever dropped and no GT row was ever
  dropped, in any of the 5 runs.

**Inferred:**
- That this measured input residual is too small to be the primary
  explanation for the previously-observed ~0.09-0.13 m position bias —
  inferred from comparing its magnitude to C2-NAV.34's own output-side
  proxy figure, not from a first-principles derivation of how an
  input-side residual propagates through AMCL's correction step to a
  position bias.
- That `diag_enabled` does not materially change AMCL's timing behavior —
  inferred from the absence of a visible difference between the one
  disabled (control) and four enabled runs in this session, not from a
  controlled A/B measurement isolating that one variable.

**Remains unknown:**
- What *does* primarily explain the ~0.09-0.13 m position bias, if not
  odometry input bias. This was never this task's question to answer —
  C2-NAV.35 already showed the bias could be an artifact of AMCL's own
  correction/estimation process rather than its input, and this result is
  consistent with that possibility without confirming it directly.
- Whether the measured input residual would look different under a
  longer-duration or larger-displacement capture (this session's 4 runs,
  105 legs total, are a snapshot, not an exhaustive characterization).
- Whether the current `nav2_params.yaml`'s consistent `enclosure_entry`
  timeout (15/15 attempts, every run) reflects a real navigation
  difficulty or a timeout margin that happens to sit just under this
  leg's typical completion time — out of scope here, reported as an
  observation only.

## Follow-up scientifically justified by this result

Given odometry input is now measured, not merely hypothesized, and shown
insufficient alone: the next natural step is characterizing AMCL's own
correction/estimation dynamics directly (what C2-NAV.35 already flagged as
the alternative explanation) rather than further odometry-side
instrumentation — e.g., comparing AMCL's particle-cloud mean before vs.
after each measurement update against ground truth, isolating the
correction step's own contribution to the output-side residual. This is a
new, separate experiment, not scoped or run here.
