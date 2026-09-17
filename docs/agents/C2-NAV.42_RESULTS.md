# C2-NAV.42 — the `/cmd_vel_nav` loop removed: raw Nav2 commands no longer reach the wheels

**Agent:** implementation → tests → live validation. 2026-09-17 UTC.
Everything below was measured in this session unless it says otherwise.
The owner gave explicit approval to change the `cmd_vel_arbiter` command-path
wiring (CLAUDE.md rule 4). The specification was C2-NAV.41 §12.

**No nav2 parameter, safety gate, goal, costmap or planner/controller value
changed.** Every tour loaded params sha256 `6f61e499…` (live readback OK).

## 1. The command path

### Old (traced from the code at `4d2e8f7`)

`nav2_bringup/navigation_launch.py` remaps `cmd_vel` → `cmd_vel_nav` for
`controller_server`, `behavior_server` and `velocity_smoother` (the smoother's
INPUT), in both the process and the composable branches.

```
controller_server ─┐
behavior_server  ──┼─> /cmd_vel_nav ─> velocity_smoother ─> /cmd_vel_smoothed ─> collision_monitor ─> /cmd_vel ─> cmd_vel_relay ─┐
                   │        ^  │                                                                                                   │
                   │        └──┼───────────────────────────────────────────────────────────────────────────────────────────────────┘  (nav.launch.py arbiter:=true: relay output_topic = /cmd_vel_nav)
                   │           └─> cmd_vel_arbiter (nav_topic default /cmd_vel_nav) ─> /diff_drive_controller/cmd_vel
```

- `gazebo_models/launch/nav.launch.py`: `relay_output` = `/cmd_vel_nav` when `arbiter:=true`.
- `custom_teleop/cmd_vel_arbiter.py`: `nav_topic` default `/cmd_vel_nav`; not
  overridden by `arbiter.launch.py`, `mission.launch.py` or `web.launch.py`.
- Two defects in one: the monitor's output looped into the smoother's input,
  and the arbiter received the raw controller command and the gated one
  interleaved on one topic.

### New (`d707327`)

```
controller_server, behavior_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed
  -> collision_monitor -> /cmd_vel -> cmd_vel_relay -> /cmd_vel_gated -> cmd_vel_arbiter
  -> /diff_drive_controller/cmd_vel
```

| file | change |
|---|---|
| `custom_teleop/custom_teleop/cmd_vel_relay.py` | `GATED_TOPIC = '/cmd_vel_gated'`; every forwarded message re-stamped with the node clock; frame_id and twist untouched |
| `gazebo_models/launch/nav.launch.py` | `arbiter:=true` → relay `output_topic` `/cmd_vel_gated` (was `/cmd_vel_nav`); `arbiter:=false` unchanged (`/diff_drive_controller/cmd_vel`) |
| `custom_teleop/custom_teleop/cmd_vel_arbiter.py` | `nav_topic` default `GATED_TOPIC` |
| `custom_teleop/launch/arbiter.launch.py` | sets `nav_topic: /cmd_vel_gated` explicitly (`web.launch.py` and `mission.launch.py` include this file) |
| `coco_web/launch/web.launch.py`, `coco_mission/launch/mission.launch.py` | docstrings only |
| `coco_mission/scripts/mission_executive.py`, `mission_states.py` | the C2-M5.1 spin comments said the arbiter reads `/cmd_vel_nav`; corrected |

No new node, so `ros_clean.sh` needs no new pattern. `/cmd_vel_gated`
contains no `nav2_`.

## 2. Commits (`worktree-c2nav0-diagnosis`)

| commit | what |
|---|---|
| `d707327` | the wiring fix (above) |
| `8bf1fe4` | wiring tests: `custom_teleop` (8), `gazebo_models` (25), `coco_mission` (8) |
| `e7fc9a6` | `verify-topology` checks every command-chain link live; 8 tests |
| `98c7d68` | `docs/data/c2nav42_cmdpath.py`: live spin and held-raw stop experiments |
| `2418420` | `c2nav42_cmdpath.py record` for a whole mission |
| `9412719` | ARCHITECTURE.md / README.md: arbiter input is `/cmd_vel_gated` |
| (this commit) | results, data bundle, `c2nav42_residual.py`, PROJECT_STATE / HOW_TO_RUN / README / SESSION_LOG |

## 3. Tests

### Added

- **`custom_teleop/test/test_cmd_vel_relay.py` (8)** — real nodes, never spun,
  publisher replaced by a list: a zero or old stamp is replaced by the node
  clock; frame_id and twist forwarded exactly; one message out per message in;
  `GATED_TOPIC` ≠ `/cmd_vel_nav` and has no `nav2_`; relay defaults still
  drive the controller (topology A); the constructed arbiter subscribes
  `GATED_TOPIC` and not `/cmd_vel_nav`.
- **`gazebo_models/test/test_cmd_vel_wiring.py` (25)**, three layers:
  1. launch files **resolved** in a `LaunchContext`: arbiter mode puts the relay
     on the arbiter's `nav_topic`; neither is `/cmd_vel_nav` or the wheel topic;
     `arbiter:=false` still drives the wheels;
  2. the Nav2 chain read from the installed `nav2_bringup` (AST) and the shipped
     `nav2_params.yaml`: every `cmd_vel` remap targets `/cmd_vel_nav`, the
     controller's output and the smoother's input; the monitor reads
     `cmd_vel_smoothed`; the relay reads the monitor;
  3. **a real graph on a private DDS domain**, built from the resolved
     parameters: exactly one wheel publisher (the arbiter); a stand-in raw
     controller publishing 0.777 on `/cmd_vel_nav` never reaches the wheels; a
     monitor STOP reaches the wheels and holds while raw keeps publishing; the
     relay output is re-stamped. **Positive control:** the pre-fix wiring is run
     through the same scenario and must leak (it does).
- **`coco_mission/test/test_mission_cmd_vel_path.py` (8)** — the `arbiter`
  value `mission.launch.py` actually passes resolves the relay onto the
  arbiter's `nav_topic`; exactly one arbiter; the panel's arbiter off; no
  launch file on the mission path names `cmd_vel_nav` in code.
- **`gazebo_models/test/test_nav_params_overlay.py` (+8)** — the live chain
  checker: both topologies accepted, the loop caught, the positive control, an
  unknown `/cmd_vel` publisher named, a smoother bypass rejected.

Wait conditions in the graph tests poll under a 15 s deadline and every
"never" is judged only after the probe itself received the raw commands.
Publishing is on a 20 Hz timer: publishing once per `spin_once` flooded the
executor (measured: probe 8755 raw, wheels 0) and was replaced.
`test_cmd_vel_wiring.py` passed 6 consecutive runs before its fixture was made
exception-safe and 3 consecutive runs after, besides the suite runs.

### Against the pre-fix code

The four wiring files at `4d2e8f7` checked out, tests run, files restored:
`test_cmd_vel_wiring.py` **12 failed** on assertions (raw 0.777 on the wheels,
the STOP tail `[0.777, 0.0, …]`, the arbiter subscribed to `/cmd_vel_nav`),
13 passed (the positive-control class and the checks the loop does not change);
`test_cmd_vel_relay.py` fails to import `GATED_TOPIC`;
`test_mission_cmd_vel_path.py` **3 failed**.

### Suites (cwd = package directory, no simulator running)

Build: `colcon build --symlink-install --packages-select custom_teleop gazebo_models coco_web coco_mission` → 4 packages, rc 0.

| suite | result |
|---|---|
| `coco_config` | 70 / 0 |
| `custom_teleop` | **75 / 0** (was 67; +8) |
| `gazebo_models` (`--ignore=test_integration`) | **136 / 0** (was 103 at C2-NAV.41; +25 wiring, +8 chain checker) |
| `coco_mission` | **289 / 0** (was 281; +8) |
| `c2nav41_topology.py selftest` | 18 / 18 |
| `c2nav39_tour_report.py selftest` | 27 / 0 |

`custom_teleop`'s 12 warnings are pre-existing: 67 passed with the same 12
warnings with the new test file excluded. `coco_web` has no tests.

## 4. Live topology-B graph

`~/coco_nav_runs/c2nav42/live_b_r01` (sim + `arbiter.launch.py initial_mode:=nav` +
`nav.launch.py arbiter:=true`, git `98c7d68`, 0 dirty paths). Committed at
`docs/data/c2nav42_live/fixed/`.

- all 10 Nav2 lifecycle nodes `active [3]`; params readback matches the shipped file;
- `verify-topology B`: **13 of 13 checks OK** (3 wheel owner / arbiter mode, 10 chain links):

| topic | publishers | subscribers |
|---|---|---|
| `/cmd_vel_nav` | `behavior_server`, `controller_server` | `velocity_smoother` |
| `/cmd_vel_smoothed` | `velocity_smoother` | `collision_monitor` |
| `/cmd_vel` | `collision_monitor`, `docking_server` | `cmd_vel_relay` |
| `/cmd_vel_gated` | `cmd_vel_relay` | `cmd_vel_arbiter` |
| `/diff_drive_controller/cmd_vel` | `cmd_vel_arbiter` (exactly 1) | |

- no launched process died.

**The same session on the pre-fix wiring** (`live_b_prefix_r01`, the four
files checked out from `4d2e8f7`, `docs/data/c2nav42_live/prefix/`):
`verify-topology` **FAIL, 4 MISMATCH** — `/cmd_vel_nav` publishers include
`cmd_vel_relay`, subscribers include `cmd_vel_arbiter`, `/cmd_vel_gated` has no
publisher and no subscriber. The live checker sees the loop.

**Residual publisher, not part of this defect.** `docking_server` holds a
publisher on `/cmd_vel` (the relay's input) in both topologies: `nav2_bringup`
does not remap `opennav_docking`. By Nav2's design it publishes only while
executing a dock/undock action, and nothing in this project sends one — **not
separately measured** (no per-publisher message attribution was recorded). It
is named in `verify-topology` as an inert peer; any other extra publisher is a
mismatch. Not changed.

## 5. Controlled command-path experiments (`c2nav42_cmdpath.py`)

Metrics are `c2nav41_topology`'s `monitor_authority`, `bypass_source` and
`stop_breach`, unchanged, on a 10 Hz zero-order-hold resample. Message-level
counts are from every wheel message and the latest message on each upstream
link. One run per arm.

**stop** — the probe stands in for the controller: it turns the robot to face
the west wall through the chain, then holds **raw 0.30 m/s** on `/cmd_vel_nav`
until PolygonStop has held the wheels ≤ 0.01 m/s for 5 s (40 s budget).

**spin** — the C2-M5.1 relocalization Spin goal exactly as `mission_executive`
sends it: `target_yaw = 2π`, nothing else.

| | pre-fix | C2-NAV.42 |
|---|---|---|
| stop: STOP held the wheels for 5 s | **no** (budget exhausted, 38.94 s) | **yes**, after 13.98 s |
| stop: wheels exceeded monitor | **327 / 527 (62.0 %)**, worst 0.30 m/s at monitor 0 | **0 / 277** |
| stop: `bypass_source` rows | **319**, all STOP; wheel = raw 319 | **0** |
| stop: STOP rows with wheels driven | **319 / 346**, worst 0.30 m/s | **0 / 70** |
| stop: collision-monitor actions (rows) | STOP 346, SLOWDOWN 9, APPROACH 19 | STOP 70, SLOWDOWN 2, APPROACH 52 |
| stop: final centre x (wall face −3.90) | −3.7297 (0.170 m) | −3.6514 (0.249 m) |
| stop: min scan range | 0.2615 m | 0.3418 m |
| stop: messages where raw ≠ smoother — wheel = smoother / raw | 2 / 130 of 141 | **9 / 0 of 9** |
| stop: wheel messages = latest monitor output | — | **491 / 491** |
| spin: result | SUCCEEDED, **17.75 s** | SUCCEEDED, **6.70 s** |
| spin: ground-truth rotation | 5.3193 rad | 5.3210 rad |
| spin: peak wheel ω | 1.0 rad/s | 1.0 rad/s |
| spin: messages where raw ≠ smoother — wheel = smoother / raw | 5 / 176 of 181 | **13 / 0 of 13** |
| spin: angular rows wheel > monitor + 0.05 | 48 / 196, worst 1.0 rad/s | 1 / 86 (0.16 rad/s) |
| spin: wheel messages = latest monitor output | — | **172 / 172** |

- **STOP reaches the wheels.** With the raw command held at 0.30 m/s the robot
  stopped 0.249 m from the wall, the PolygonStop radius (0.25 m).
- **The smoother is in the path.** Every wheel message where the raw and
  smoothed commands differed carried the smoother's value.
- The one angular spin row above the monitor is the resample catching a
  message in transit (monitor 0.6098, gated still 0.7698); at message level
  172 / 172 wheel messages equal the latest monitor output.
- **The spin under-rotates by the same amount before and after** (5.32 rad of
  6.28 by ground truth; Spin closes its loop on odometry). Pre-existing, not
  caused by this change, not investigated.
- **The spin is 2.6× faster** after the fix (6.70 s against 17.75 s).

## 6. Three fresh topology-B tours (`baseline_topology_b.yaml`, unchanged)

Runs `r05`–`r07`, git `9412719`, 0 dirty paths, every live readback OK (13 checks).
Per-tour data: `docs/data/c2nav42_per_tour.json`.

| run | legs | ordinary | entry | exit | tour sim s | min true clear m | PolygonStop n / s | deadlocks | goal err med m | \|yaw err\| med rad | longest crawl s | exceeded | bypass rows | = raw | stale drops |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| r05 | 6/7 | 5/5 | 0/1 | 1/1 | 207.71 | 0.2548 | 0 / 0.0 | — | 0.095 | 0.433 | 4.04 | 40/2075 | 3 | 0 | 0 |
| r06 | 6/7 | 5/5 | 0/1 | 1/1 | 216.10 | 0.2873 | 0 / 0.0 | — | 0.076 | 0.419 | 3.62 | 104/2154 | 3 | 0 | 0 |
| r07 | 6/7 | 5/5 | 0/1 | 1/1 | 207.61 | 0.2676 | 0 / 0.0 | — | 0.109 | 0.462 | 44.42 | 11/2075 | 0 | 0 | 0 |
| **arm** | **18/21** | **15/15** | **0/3** | **3/3** | median 207.71 | **0.2548** | 0 / 0.0 | — | 0.092 | 0.431 | | **155/6304** | **6** | **0** | 0 |

Terminal errors are medians over SUCCEEDED legs. "exceeded" is
`monitor_authority` at 0.02 m/s. "longest crawl" is nav_bench's
`worst_crawl.crawl_len_s`, maximum over the tour's legs.

`enclosure_entry` failed all three: r05 TIMEOUT 0.099 m from the goal with
yaw error −2.855 rad; r06 TIMEOUT 0.094 m, −2.27 rad; r07 TIMEOUT 1.338 m from
the goal, never in its terminal phase, a 44.42 s crawl.

### Against C2-NAV.41 (pre-fix B) and C2-NAV.39 (A)

| arm | legs | ordinary | entry | exit | exceeded (frac) | bypass rows | = raw controller | = smoother | STOP rows driven |
|---|---|---|---|---|---|---|---|---|---|
| A, C2-NAV.39 r01–r03 | 17/21 | 15/15 | 2/3 | 0/3 | 0/6900 (0.00 %) | 0 | 0 | 0 | 20/1763 |
| B pre-fix, C2-NAV.41 r01, r02, r04 | 11/21 | 11/15 | 0/3 | 0/3 | **1498/7029 (21.31 %)** | **643** | **349** | 0 | 23/52 |
| **B C2-NAV.42 r05–r07** | **18/21** | **15/15** | 0/3 | **3/3** | **155/6304 (2.46 %)** | **6** | **0** | 0 | 0/0 |

The pre-fix and A rows were recomputed this session from the run directories
with the same code and reproduce C2-NAV.41's numbers exactly.

- **The C2-NAV.41 criterion — wheel commands following the raw controller —
  is 0** (was 349 of 643 rows).
- **Legs 18/21**, from 11/21. Ordinary legs 15/15. `enclosure_exit` 3/3 (A
  0/3, each a PolygonStop hold). `enclosure_entry` 0/3 (A 2/3).
- **No PolygonStop activation in any of the three tours**, so these tours do
  not exercise STOP on the wheels. §5 does.
- **Median speed while moving:** A 0.0934 m/s, B-fixed 0.1109 m/s
  (`c2nav41_topology` definition).

## 7. The residual: 155 trace rows where the wheels exceeded the monitor

C2-NAV.41 §12's own pass bar was "0 `bypass_source` rows **and**
wheels-exceeded-monitor at topology A's control level". On nav_bench's trace
the fixed arm is 6 rows and 2.46 % against A's 0 %. **By that bar the tours
alone do not pass.** What the rows are (`docs/data/c2nav42_residual.py`,
outputs `c2nav42_residual_*.json`):

**Trace classification** — does the wheel value appear in the monitor's own
output within ±0.5 s?

| arm | exceeded | in monitor output ±0.5 s | only later | only earlier | in neither | in neither and = raw |
|---|---|---|---|---|---|---|
| A C2-NAV.39 | 0 | 0 | 0 | 0 | 0 | 0 |
| A C2-NAV.42 r04 | 0 | 0 | 0 | 0 | 0 | 0 |
| B pre-fix | 1498 | 406 | 258 | 73 | **1092** | **593** |
| B fixed r05–r07 | 155 | 134 | 65 | 16 | 21 | 2 |
| B fixed r08 (instrumented) | 24 | 21 | 14 | 3 | 3 | 0 |

**Two explanations tested and REJECTED:**

- *nav_bench recorder stalls.* Rows repeating the previous row's ground truth
  exactly are 63.6 % (A), 66.4 % (B pre-fix) and 66.4 % (B fixed) of moving
  rows; exceeded rows are 65.3 % frozen in the fixed arm — the base rate. No
  association.
- *Different topic rates.* The per-leg wheel/`/cmd_vel` rate ratio nav_bench
  recorded has median 1.0 in all three arms.

**Message-level attribution** (instrumented tour `r08`, a fourth fresh
topology-B tour with `c2nav42_cmdpath.py record` alongside nav_bench; reported
separately from the three pre-registered tours):

| r08 | 6/7 legs (ordinary 5/5, entry 1/1, exit 0/1, exit PolygonStop deadlock 54.9 s) |
|---|---|
| nav_bench trace | exceeded 24 / 2420, bypass rows 0, STOP rows driven 10 / 549 |
| recorder, 10 Hz resample | exceeded **4 / 2449**, worst gap 0.125 m/s, bypass rows **0**, STOP rows driven 1 / 569 (0.0805 m/s) |
| wheel messages | **4629** |
| = latest monitor output (v and ω) | 4578 |
| = latest relay output, monitor message in transit | 6 |
| = an EARLIER relay output | **45**, median age 0.060 s, max **0.078 s** |
| unattributed | **0** |
| = raw controller where the monitor differed | **0** |

**Reading (measured on r08; inferred for r05–r07, same code):** every wheel
command was a collision-monitor output delivered by the relay. The residual is
the arbiter re-sending a gated command up to 78 ms old — its 20 Hz watchdog
fires before the newer `/cmd_vel_gated` message is processed. Topology A has
no such hop and no re-publication, which is consistent with its 0. This is
latency on an extra hop, not a path around the monitor. It is also not zero,
and it is reported as such.

## 8. C2-M5.1 relocalization spin

### The Spin goal, directly (§5)

The exact goal `mission_executive._send_spin` sends, through the fixed chain:
**SUCCEEDED in 6.70 s** (pre-fix 17.75 s), wheels peak 1.0 rad/s, every wheel
message the monitor's output, the smoother between raw and wheels on all 13
differing messages, ground-truth rotation 5.321 rad (pre-fix 5.319).
Ownership: the arbiter, `active=nav`, sole wheel publisher.

### The executive's own RELOCALIZE — not reached

`~/coco_nav_runs/c2nav42/mission_r01`, committed at
`docs/data/c2nav42_live/mission/`: fresh sim (`traverse:=true`),
`mission.launch.py rviz:=false target_colour:=blue` (web panel on, as
shipped), git `9412719`, 0 dirty paths.

**Bring-up (Task 5), all PASS:** 10 Nav2 lifecycle nodes active; params
readback matches; `mission_executive`, `localization_monitor`, `mission_hud`,
`ramp_driver`, `approach_server`, `grasp_server`, `move_group`,
`bt_navigator`, `collision_monitor`, `cmd_vel_relay`, `cmd_vel_arbiter`,
`velocity_smoother` present; exactly one arbiter; wheel topic exactly 1
publisher, `cmd_vel_arbiter`; all 10 chain links OK; arbiter `mode=idle`
(correct before a mission starts — the one `verify-topology` mismatch, and
expected); no process died during bring-up or the mission.

**Mission, with `c2m51_inject.py` (diverged2: AMCL moved 3 m south on
RETURN_HOME):**

| state entered (s after recorder start) | |
|---|---|
| NAVIGATE_TO_RAMP 3.1 → ALIGN_FOR_CLIMB 16.2 → CLIMB 16.4 | Nav2 leg, then the RL climb |
| SEARCH_TARGET 30.3 → APPROACH_TARGET 33.1 → GRASP 45.3 → DESCEND 70.1 | |
| RETURN_HOME 86.1 | injected: `(8.83, 0.02) -> (8.83, -2.98)` |
| RECOVERY 86.7 → RETURN_HOME 86.8 → RECOVERY 133.3 → RETURN_HOME 133.5 → RECOVERY 133.6 → **ABORT 133.7** | `RETURN_FAILED`, retries exhausted |

- **RELOCALIZE was never entered.** The health monitor scored 26
  INCONSISTENT samples on mapped ground during the second RETURN_HOME, but
  **0 latched-degraded samples and 0 recovery triggers** (`c2m51_hrec.py
  --summarise`). Nav2's aborts spent RETURN_HOME's retries first.
  C2-M5.1 recorded detection latency of 3.33 s to 82.9 s; this run is on that
  distribution's slow side. Not a wiring effect: the command path carries no
  health signal.
- **Not retried.** The brief allows the smallest meaningful regression and
  forbids rebuilding infrastructure; the executive's spin is the same
  `nav2_msgs/Spin` goal measured directly above.
- **Command path over the whole mission** (`c2nav42_cmdpath.py record`):
  rows owned by the Nav2 chain (arbiter `active=nav`, 1.0 s switch guard, 47
  rows excluded): **exceeded 0 / 553, bypass rows 0, STOP rows 0**. All rows:
  344 / 1319 exceeded and 75 bypass rows (cm_action 0), which are the RL climb
  and the approach servo driving the wheels while Nav2's monitor is idle —
  those sources never went through the monitor, before or after.

## 9. Topology-A regression (`baseline.yaml` × 1, `baseline_r04`)

- live readback **12 of 12 OK**: the relay owns the wheels, no arbiter,
  `/cmd_vel_gated` unused;
- **6/7**: ordinary 5/5, `enclosure_entry` 1/1 (67.18 s), `enclosure_exit` 0/1
  (PolygonStop hold 68.76 s, 3.148 m from the goal after 0.252 m) — the same
  exit failure C2-NAV.39 measured 3 of 3 times;
- exceeded **0 / 2479**; bypass rows 0; STOP rows driven 10 / 688 (worst
  0.0711 m/s);
- **re-stamp effect: `n_stale_cmd_drops` 0 over 7 legs**, against **216 over
  21 legs** in C2-NAV.39's A arm with the old relay. Both B arms: 0.

**Comparability.** Topology A's harness is not byte-identical to C2-NAV.0 …
.41: the relay now re-stamps, so the controller no longer drops stale commands.
One tour cannot say whether that moves A's leg count.

## 10. Decision

**The command-path fix is ACCEPTED.**

| criterion (brief, Task 10) | result | evidence |
|---|---|---|
| 1. topology-B wheel ownership correct | **met** | exactly 1 wheel publisher, `cmd_vel_arbiter`, in every live readback (tour-style ×5 sessions, `mission.launch.py` ×1) |
| 2. raw controller → wheel bypass count zero | **met** | C2-NAV.41 criterion (bypass rows matching the raw controller): 0 in r05–r07 (was 349), 0 in r08, 0 in the controlled stop (pre-fix 319); message level r08 0 of 4629 |
| 3. collision-monitor intervention reaches the wheels | **met** | controlled stop: raw 0.30 m/s held, STOP held the wheels, 0 / 70 STOP rows driven (pre-fix 319 / 346); r08 recorder 1 / 569 at 0.0805 m/s |
| 4. smoother / safety processing no longer bypassed | **met** | live graph: the arbiter reads only `/cmd_vel_gated`; wheel = smoother on every raw≠smoothed message (13 / 13, 9 / 9); r08: every wheel message a monitor output, 45 of them up to 0.078 s old |
| 5. all relevant tests pass | **met** | 70 / 75 / 136 / 289, 0 failing |
| 6. topology-B bring-up clean | **met** | lifecycle, params, chain, no process died — tour-style and `mission.launch.py` |
| 7. three fresh tours without infrastructure regression | **met** | r05–r07 rc 0, 7 legs each, readback OK |

**Not met, and stated:** C2-NAV.41 §12's stricter trace bar,
wheels-exceeded-monitor at topology A's level. nav_bench's trace reads 155 /
6304 (2.46 %) against A's 0. §7 attributes it on r08 to the arbiter re-sending
gated commands up to 78 ms old, not to any path around the monitor.

**Navigation, after the safety correction:** topology B 18/21 (from 11/21;
A 17/21). Ordinary 15/15. `enclosure_exit` 3/3, never held by PolygonStop in
B. **`enclosure_entry` 0/3** — two terminal-heading timeouts within 0.10 m of
the goal (−2.86, −2.27 rad) and one 44.42 s crawl 1.34 m out. Recorded, not
investigated.

**Should it be integrated?** Yes. It removes a measured safety defect on the
shipping path, adds a live check that stops a tour if the loop returns, and
does not reduce legs completed. Merging is the owner's. **Comparability owed:**
M6's 19/20 fetch matrix was measured with the loop and has not been re-run.

## 10a. Exactly one next implementation action

**Put the fix on `main`:** branch from `main`, cherry-pick `d707327` (wiring)
and `8bf1fe4` (wiring tests), run the four package suites there, and open a
draft PR for the owner. `main` ships `mission.launch.py` with the loop today.
Whether the two commits apply to `main` without conflict is **not yet
checked**. `e7fc9a6` depends on C2-NAV.41's tour tooling, which is not on
`main`.

## 11. Limitations

- One controlled stop and one controlled spin per arm; three pre-registered
  tours plus one instrumented tour; one mission.
- The residual (§7) is attributed at message level on one tour only.
- The tours never entered PolygonStop; STOP on the wheels is shown by the
  controlled experiment and r08, not by r05–r07.
- M6's standing 19/20 was measured with the loop in place. Not re-measured.
- `docking_server`'s unremapped `/cmd_vel` publisher is expected inert (not
  measured) and was not removed.
- The executive-driven RELOCALIZE spin was not reached (§8); only the Spin
  goal itself was exercised on the fixed path.
