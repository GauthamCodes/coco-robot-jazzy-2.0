# COCO 2.0 STATUS: ROBOT FROZEN / PLATFORM IN PROGRESS

**The ROBOT is final. There is no next robotics milestone.** Everything
below describes it and still holds.

**A productization track opened after that freeze** (`docs/ROADMAP.md`
Track 4). P0.1 wraps the frozen robot in a browser-accessible platform:
a versioned WebSocket API, a real control interface, and a Docker
appliance. It adds no robotics capability and is not permitted to change
the robot — see **THE PLATFORM (P0.1)** below.

| | |
|---|---|
| **Canonical branch** | `main` — the only branch. A fresh clone of it is sufficient |
| **Final commit** | the tip of `main`; `git log -1 --oneline` is the authority |
| **Remote** | `https://github.com/GauthamCodes/coco-robot-jazzy-2.0` |
| **Verified test count** | **829 passing, 0 failing, 0 skipped**, across eight packages with test suites (nine packages total). On `c2nav43-integration`: **997 passing, 0 failing, 0 skipped** (C2-NAV.45; was 975 at C2-NAV.43) |
| **Final nominal mission** | **COMPLETE.** All 16 nominal states, `attempt=1` throughout, `reason=--`, 186.7 s. Grasp physically verified from Gazebo ground truth: target lifted **35.1 mm** |
| **Localization health** | **0 triggers** over 5,784 samples on that mission (`degraded=0` on every one) |
| **Localization recovery** | **Detection works; severe recovery does not.** See KNOWN LIMITATIONS 1 |
| **Command-path safety** | **Fixed on `c2nav43-integration`, pending the owner's merge.** Cut from `main`, with the C2-NAV.42 fix integrated. Raw-controller → wheel bypass rows **0** in every live test and every tour, and a held raw 0.30 m/s was stopped by PolygonStop. On `main` before the merge the gating does not reach the wheels. See KNOWN LIMITATIONS 0 and `docs/agents/C2-NAV.43_RESULTS.md` |
| **Depth perception** | **Optional candidate, OFF by default.** `nav.launch.py depth_cloud:=true` plus a `perception` experiment block. See KNOWN LIMITATIONS 0 (C2-NAV.43) |
| **Platform (P0.2, second pass)** | Branch `p02-browser-experience` (from `921f6d0`; `main` does not contain P0.1). **1564 passing, 0 failing, 0 skipped**; clean build 9/9. Two missions **COMPLETE** started from the page in a real (headless Firefox) browser. Docker runtime **NOT VERIFIED**. See *P0.2, second pass* below |

Evidence for the mission row is committed at
`docs/data/release_nominal_mission.txt`.

---

## THE PLATFORM (P0.1)

Branch `worktree-p01-platform`, cut from `main` at `d317d85`. Design in
`docs/PRODUCT_ARCHITECTURE.md`; protocol in `docs/WEB_API.md`; runtime in
`docs/DOCKER.md`.

| | |
|---|---|
| **What it is** | `coco_web` is now a real package: `platform_server`, one ROS node serving the UI on **:8080** and the versioned `coco.v1` protocol on **/ws**. It replaces rosbridge, which let any browser tab publish any topic |
| **Command safety** | The browser can reach **only** `/cmd_vel_teleop` — an arbiter INPUT. Enforced three ways: the schema cannot express a topic; the publish allowlist is checked against the wheel topics at node construction, including ROS-parameter overrides; velocity is clamped at the boundary |
| **Verified live** | Drive reached `/cmd_vel_teleop`; **nothing** ever appeared on `/diff_drive_controller/cmd_vel` (which did not exist on the graph); `stop` published an explicit zero; the last client disconnecting stopped the robot. The "we saw nothing" control was honoured — the wheel subscriber was the same type and QoS as the teleop one, which did receive |
| **Readiness** | `/healthz` answers **503 until every required component is up**, so Docker's HEALTHCHECK going green and the robot being drivable are one statement |
| **Tests** | **1139 passing, 0 failing, 0 skipped** (was 1004). `coco_web` 0 → 116, `coco_mission` 311 → 315, `coco_rl` 164 → 179 |
| **Clean build** | 9/9 |
| **Docker** | **AUTHORED, NEVER BUILT.** Docker is not installed on the development machine. Do not report the image as working |
| **M6 regression** | **NOT RUN at P0.1.** An unrelated project's Gazebo was running on the machine throughout, and this repo allows one simulator at a time. Killing someone else's live simulator was not an acceptable way to free it. **Run at P0.2 — see below** |

---

## P0.2 — the platform becomes usable. DONE, validated live

Same branch. Protocol reference: `docs/WEB_API.md`.

| | |
|---|---|
| **Mission state** | Real, from the executive. `/mission/state` carries **twelve** fields; P0.1 read two and looked for three it has never carried, so the mission colour on the wire was permanently null. `mission_view.py` reads all twelve and translates them into a ten-word product vocabulary while keeping the executive's own state name beside it |
| **Progress** | **Step N of 16, from the executive's own `NOMINAL_NEXT` chain.** No percentage and no ETA — `mission_executive.py:603` sends its Nav2 goal with no `feedback_callback`, so progress *within* a leg is not published anywhere. P0.1's browser interpolated one from a hard-coded list; a test asserts it stays deleted |
| **Subscriptions** | `subscribe` / `unsubscribe` / `set_stream` honoured per client. The default set is P0.1's, which is what let the protocol stay `coco.v1`; camera and depth are opt-in and their ROS subscriptions exist only while someone is watching |
| **Binary transport** | Self-describing binary frames for LiDAR, camera and depth. No ROS message on the wire, no topic in any header. Nine malformed shapes are tested |
| **Backpressure** | One frame in flight per stream per client, plus a 1 MiB socket-buffer bound. Control frames — `ack`, `error`, `pong`, telemetry — are never dropped |
| **Verified live (P0.2)** | A **complete green fetch driven entirely through the browser protocol**: all 16 states in order, `result=fetch`, **170.4 s**. 1 714 telemetry frames, **0 dropped**, peak socket buffer **0 B**. Mission-state latency **18.4–82.7 ms**. LiDAR frame **668.8 B at 10.0 Hz**; camera **3 467 B** mean JPEG at **6.17 fps** under a 10 fps cap; depth **19 frames in 5 s** with `depth_topic` set and **0** without it. Platform CPU **67–75 % of one core**. `/diff_drive_controller/cmd_vel` publisher count **1** (`cmd_vel_arbiter`), the platform publishing only `/cmd_vel_teleop`; pointing it at the wheel topic still refuses to start |
| **Drive path, live** | Browser `drive` moved the wheels (40 commands, max 0.15 m/s); `stop` zeroed them; a second client's `drive` was refused `not_in_control` while its **STOP was honoured and reached the wheels**; disconnecting the last client ended stopped. The "we saw nothing" control was honoured: the recorder saw 97 wheel commands |
| **Tests** | **1334 passing, 0 failing, 0 skipped** (was 1139). `coco_web` 116 → 297, `coco_rl` 179 → 190, `gazebo_models` 178 → 181 |
| **Docker** | **STILL NEVER BUILT.** Unchanged by P0.2, which added no port and no dependency. `docs/DOCKER.md` carries the exact verification procedure for a machine that has Docker |

**Two limitations, stated rather than smoothed over.**

1. **The browser was never driven.** The Chrome extension was not
   connected on this machine, so the page is covered by static asset
   tests (73 element ids used, 73 present; every frame type it sends is
   in the server's schema; no ROS topic string anywhere in it) and by a
   WebSocket client exercising the same server paths. Rendering,
   layout and interaction are **unverified**.
2. **One of two mission attempts aborted.** The first reached
   `RETURN_HOME` — 13 of 16 states, including a verified grasp — and then
   failed `RETURN_FAILED` with `planner_server: GridBased plugin failed
   to plan from (7.97, 1.15) to (0.00, 0.00): "Start occupied"`. That is
   a localisation outcome upstream of the web layer, which publishes no
   TF and no goals during a mission. **Two runs is not a rate**, and
   nothing here re-measures M6.

**One defect found by the live run, in P0.2's own code.** `wants()`
gated binary delivery on `BINARY_STREAMS` (camera, depth) while
`push_sensors` also framed **lidar** as binary — so a client declaring
`binary: false` was sent binary lidar frames anyway, which is precisely
the compatibility guarantee the design claims. It surfaced as a
`UnicodeDecodeError` in a probe calling `json.loads` on bytes, which is
exactly how a P0.1 client would have met it. Fixed by splitting
`BINARY_CAPABLE` from `BINARY_STREAMS`; six tests pin it, and a live
probe now confirms a text client gets 51 JSON scans and 0 binary frames
while a binary client gets 50 binary frames and 0 duplicate JSON.

**One P0.1 gap closed.** `ros_clean.sh` had **no pattern for
`platform_server`** — P0.1 added the node to a launch file and not to the
sweep, the same rule `mission_hud` already broke. An orphan holding
:8080 was observed during this work and the next launch died with
`Address already in use`. Three tests pin the fix.

The M6 command, for whoever has a free machine:

```bash
# T1 — fresh simulator, every run
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
# T2 — stack + the new web platform on :8080
ros2 launch coco_mission mission.launch.py platform:=true
# T3
ros2 run gazebo_models traverse_demo.py --colour blue
```

Expected, unchanged from C2-NAV.49: bypass 0, stale drops 0, PolygonStop
0, and the colour matrix at 12/12.

---

## P0.2, second pass — the browser experience, driven in a browser

Branch **`p02-browser-experience`**, cut from `921f6d0` (which descends
from `main` `d317d85`). `main` itself does **not** contain P0.1 — it has
only the old rosbridge panel; P0.1 and both P0.2 passes live on this line.
Protocol still **`coco.v1`**. Codex's hardening from `codex/p02-hardening`
is integrated (ten commits cherry-picked with `-x`).

**The first pass never drove a browser.** This one does:
`scripts/browser_check/` runs the shipped page in headless Firefox over
WebDriver BiDi (tornado as the client — no Selenium, no driver, no
extension), with real clicks and key presses only. What that found:

| Found by the browser | Consequence | Fixed |
|---|---|---|
| The "COCO is starting" curtain was **always drawn** — `.waiting {display:flex}` outranks `[hidden]` | it covered the whole page **including STOP**; a mouse click on STOP hit the curtain | global `[hidden]{display:none!important}`; STOP stacks above the curtain in every state; test |
| The robot was drawn at the **odometry** pose on a map-frame map | after a climb, a robot verified home was drawn at (0.61, 3.71), outside the arena, LiDAR scattered off the walls | `MapPoseTracker`: AMCL-fixed map→odom correction at odometry rate; drawn at (0.03, 0.09) in run 2 |
| Health read **DEGRADED on an idle, healthy stack** | `/plan` and `/amcl_pose` are silent when stationary | local costmap (2 Hz) as Nav2's heartbeat; Healthy at idle and at the end in run 2 |
| STOP with W still held drove off again | the 10 Hz loop re-read the held key | STOP/Space clear held keys; **0** moving wheel commands after STOP in both runs |
| A frozen server left a live-looking page ("Healthy", a pose) | no client heartbeat | 4 s silence watchdog; measured disconnected + reconnecting at 5.8 s, same session on thaw |
| Joystick drawn over the sticky header; STOP below the fold on a phone; heading unwrapped | cosmetic to serious | fixed; STOP pinned full-width on phones |

**Added:** the health axis `HEALTHY/DEGRADED/UNHEALTHY`, separate from the
lifecycle (`/healthz` 200 ⇔ not UNHEALTHY, all 256 component combinations
tested); simulator readiness requires COCO model odometry **arriving**;
no ROS topic name on the wire (component details and a refusal carried
them); depth image on by default for display, fusion still off; LiDAR
arrivals metered; mission phase on the world view; a Session & health and
a Navigation & sensors card; the executive's result in words.

**From Codex, integrated:** duplicate-key/huge-number/deep-nesting
rejection, exact binary payload lengths and metadata validation, bounded
buffers, padded image rows and big-endian depth (now wired to
`msg.step`/`msg.is_bigendian`), mission-number validation, a lifecycle
edge validator (given a job: a test walks every derived lifecycle change),
real-socket transport tests, and **a Docker entrypoint bug** —
`SIM_PID=$(launch_bg …)` lost the PID to a subshell, so the container tore
its own stack down after starting it. Not taken: Codex's standalone JS
transport (unused; replacing the browser-validated one would void that
evidence) and its evidence logs (they stay on its branch).

### Measured live — two fresh simulators, one browser each (NOT a rate)

| | Run 1 (green) | Run 2 (blue) |
|---|---|---|
| Mission, started by clicking Start | **COMPLETE**, `result=fetch` | **COMPLETE**, `result=fetch` |
| Executive states | 16/16 | 16/16 |
| States rendered on the page | 15/16 by 250 ms polling (missed the brief LOCALIZE) | **16/16**, in-page MutationObserver |
| ROS → page lag per transition | ≤ 700 ms incl. polling | **62.7–74.9 ms**, all 15 transitions |
| Wall time IDLE→COMPLETE | 417.3 s | 362.2 s |
| Real-time factor (sim/wall) | ≈ 0.4 | ≈ 0.4 |
| Page ready after load | 0.56 s | 0.55 s |
| Camera first frame after subscribe | 0.48 s | 0.34 s |
| Depth first frame after subscribe | 0.68 s | 0.48 s |
| W key → first moving wheel command | 93.6 ms | 45.0 ms |
| STOP with W still held → wheel zero | ≤ 2 ms after the click returned; **0** moving after | 15.3 ms; **0** moving after |
| Browser SIGKILLed mid-drive → wheel zero | 69.6 ms; **0** moving after | 71.7 ms; **0** moving after |
| Wheel-topic publishers seen | `/cmd_vel_arbiter` only | `/cmd_vel_arbiter` only |
| Teleop publishers seen | `/coco_web_platform` only | `/coco_web_platform` only |
| Hostile socket frames refused | 8/8 | 8/8 |
| JS errors | 0 | 0 |
| Platform CPU (one core) | 49.8–80.2 %, mean 63.6 | 54.7–78.6 %, mean 64.1 |
| Mission latency (server) | 7.0–51.6 ms | 46.4–76.6 ms |
| Peak socket buffer / drops | 0 B / 0 | 0 B / 0 |

Frame sizes (run 1): LiDAR **669.5 B**, camera JPEG **3 572 B**, telemetry
JSON **4.1 kB** at 10 Hz; ≤ **66 kB/s** to one browser.

Without a simulator, in the same browser: all **seven** connection states
rendered (CONNECTING, CONNECTED, SIMULATOR_STARTING, SIMULATOR_READY,
MISSION_RUNNING, DISCONNECTED, ERROR) with STOP reachable in every one; a
server restart detected by its new session id and the view reset; a
**silent-but-open client reaped by the server after 25.2 s**
(`stale_client_probe.py`).

**Docker: NOT VERIFIED at runtime** — no Docker on this machine. Statically:
compose parses and agrees with the Dockerfile's health contract, every
COPY source exists, every `coco_web` runtime dependency is apt-installed
by the image, and all 19 apt packages resolve in the Noble + Jazzy index.

**Limitations, stated.** Two missions is not a rate. The page was driven
by headless Firefox, not Chrome, and not on a phone (a 390 px viewport was
rendered, not touched). The joystick (nipplejs) was not dragged — driving
was by keyboard. Camera/depth were checked to arrive and render, not for
image correctness.

---

## HOW TO REPRODUCE

Everything is in **`HOW_TO_RUN.md`**, and every command in it was checked
against this tree. The short path:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
    coco_config coco_sim coco_rl coco_perception \
    coco_moveit_config custom_teleop gazebo_models coco_mission coco_web
source src/coco-robot-jazzy-2.0/setup_env.sh
```

Then the three-terminal mission in `HOW_TO_RUN.md`, "A normal autonomous
mission". Tests: per package, cwd inside the package, on a clean ROS
graph — `HOW_TO_RUN.md`, "Test suite".

---

## KNOWN LIMITATIONS

These are the honest end of the project. They are reproducible, they are
measured, and none of them is rounded up.

**0 — C2-NAV.43 update (2026-09-17), on `c2nav43-integration`.** The
branch was cut from `main` and the C2-NAV.42 fix was cherry-picked after
inspection: `d707327`, `8bf1fe4`, the gz world-path quoting, the
owner-accepted nav2 defaults (sha256 `6f61e499…`) and the tour tooling.
`cmd_vel_relay` publishes `/cmd_vel_gated` and re-stamps, and
`cmd_vel_arbiter` reads `/cmd_vel_gated`. Measured on this branch:

- live graph: `verify-topology` **13 / 13 links OK**, exactly one wheel
  publisher;
- a stand-in controller held raw 0.30 m/s toward a wall: wheels above the
  monitor **0 / 278**, 71 STOP rows with the wheels driven **0**, stopped
  0.249 m from the wall face;
- tours: topology A 7/7 (monitor exceeded 0 / 2072, stale drops 0);
  topology B 6/7 (exceeded 9 / 2722, bypass rows 0, stale drops 0); six
  perception-comparison tours in topology B, **raw-controller bypass 0** in
  all six;
- **residual:** the wheels still exceed the monitor on short streaks in every
  B tour, 0.088–2.92 % of trace samples per run, worst 0.3 m/s at a leg
  start with controller, smoother and monitor all at 0. Worst wheel during
  PolygonStop was 0.09 m/s. Not attributed.

**M6 re-measured on the fixed path (C2-NAV.44, 2026-09-17).** Six fresh
missions through `mission.launch.py`, one simulator each, depth fusion off:
**3 complete** (red, blue, yellow), **3 aborted, all green**, all
`PRE_RAMP_POSE_OUT_OF_REGION` before the climb. Command path clean in all
six — **raw-controller → wheel bypass 0**, stale drops 0, PolygonStop
activations 0, exactly one wheel publisher. The three aborts are **not** a
command-path failure: `NAVIGATE_TO_RAMP` ended 0.305–0.311 m from the
pre-ramp goal by ground truth while Nav2's own `SimpleGoalChecker`
(`xy_goal_tolerance: 0.25`, judged on the estimated pose) reported
`Reached the goal!`, and the executive's ground-truth gate is set to the
same 0.25 m, leaving zero margin. Both retries issued **0.000 m/s** — the
same "structurally futile" retry the repo already records for the yaw gate.
AMCL was not diverged (0.004–0.049 m at each correction, `degraded=0`).
The three completions kept the historical bands: lift 34.6/35.3/35.6 mm,
approach base-x 0.1540/0.1546/0.1547, all inside the 5.5 mm window.
**19/20 is not a control for this**: it was measured with `traverse_demo.py`,
whose `nav_to()` returns on Nav2's `SUCCEEDED` alone and has no ground-truth
arrival gate, so that abort cannot occur in it. **Run on this branch, on the
same fixed command path, that harness delivers the green fetch end to end**
(`mission.launch.py executive:=false` + `traverse_demo.py --colour green`):
all seven steps, `outcome=held` with base-x 0.1545, home to within 0.04 m,
`FETCH COMPLETE`, 413 s — bypass rows 0, stale drops 0, wheels above the
monitor 3/947 (inside C2-NAV.43's unattributed residual). So the command-path
fix does not break the fetch; the pre-ramp gate rejects a leg the mission can
finish from. One run is not a rate. Full report:
`docs/agents/C2-NAV.44_RESULTS.md`.

**The pre-ramp gate is FIXED (C2-NAV.45, 2026-09-18), and the three green
missions it aborted now complete.** The defect was a **mission-level
arrival-gate conflict**, nothing else: the executive's ground-truth check was
set to Nav2's own `xy_goal_tolerance`, so two 0.25 m windows measured from two
different poses left **zero margin**, and the retry re-sent the goal to a
controller that had stopped and believed it had arrived (measured: 0.000 m/s
for the rest of every aborted run). `_check_nav_leg` now has three named bands
— clean inside `xy_tolerance` 0.25 m; **accepted, recorded and logged at WARN**
inside `xy_consistency`; hard failure beyond it — and the band is **derived,
not invented**: `GOAL_XY_CONSISTENCY = 2 x GOAL_XY_TOLERANCE = 0.50 m`, past
which the pose Nav2 steered by must be wrong by more than the whole arrival
window, which is a localisation failure the C2-M5 monitor already owns.
Setting `xy_consistency == xy_tolerance` restores the old gate exactly.

Measured, three fresh green missions, one simulator each, depth fusion off,
HEAD `56c324b` with 0 dirty paths: **3 of 3 COMPLETE**, `result=fetch`,
`attempts={}`, all 16 nominal states, **0 RECOVERY entries and 0 retries** in
every run. **The original failure still occurs and is now handled**: pre-ramp
ground-truth error **0.348 / 0.315 / 0.316 m**, outside the 0.25 m tolerance in
all three and bracketing C2-NAV.44's 0.3047-0.3115 m, each accepted with an
explicit warning naming the disagreement. Lift 36.0 / 35.4 / 34.9 mm, home to
0.070 / 0.038 / 0.022 m. Command path unchanged: **bypass 0, wheels above the
monitor 0, stale drops 0, PolygonStop 0** in all three. Localization **0
degraded samples and 0 relocalizations** across 6,317 samples. Tests **997 / 0
/ 0**. The same policy covers `RETURN_HOME`, whose mechanism is identical.
**Not claimed:** AMCL, the lane-dependent sign of the offset, localization
recovery, the enclosure problem, depth fusion, or an M6 success rate - three
runs of one colour is not a rate. Full report:
`docs/agents/C2-NAV.45_RESULTS.md`.

**The M6 fetch colour matrix is COMPLETE (C2-NAV.46, 2026-09-18):
11 of 12 fetches, 12 valid runs, 0 void.** red **3/3**, green **3/3**
(C2-NAV.45's, carried over unchanged), blue **2/3**, yellow **3/3**. Three
fresh executive-driven missions each for red, blue and yellow, fresh
simulator per run, headless, never `--fast`, depth fusion off, no Nav2,
goal, planner, controller or safety change, `dirty_paths=0` in all nine.
**No runtime code was changed in this sprint.**

**The gate generalises, and the green discrepancy turns out to be
lane-specific.** All 12 runs passed the pre-ramp gate, the outer 0.50 m band
was **never reached**, and futile retries were **0 in every run**. But only
green uses the consistency band: green **0.315-0.348 m**, against red
**0.108-0.131 m**, blue **0.066-0.085 m** and yellow **0.030-0.047 m**, all
**clean inside the original 0.25 m tolerance** in all nine new runs. Red,
blue and yellow would all have passed the *old* single-threshold gate; only
green would not. C2-NAV.44's abort was a green-lane failure measured on
green, not a defect uniformly present in all four lanes.

Command path: **bypass 0 and stale drops 0 in all 12**. Wheels above the
monitor **4 of 9,744 nav-active samples = 0.0411 %**, worst gap 0.0316 m/s —
the known unattributed short-streak residual, **not** claimed fixed. Grasp
**12 of 12** inside `[0.1510, 0.1565]` (0.1534-0.1547 m), lift 34.5-36.4 mm,
across four cylinder radii: the pick is **not colour-sensitive**.

**The one failure is navigation, not the architecture.** `r2_blue` ABORTed
`RETURN_FAILED` — a **valid** run (22/22 checks, clean shutdown), reported not
discarded. Its pre-ramp gate was **clean at 0.066 m** and its **pick
succeeded** (lift 36.2 mm); it died carrying the object on the way home.
PolygonStop held the robot **595.5 s**, 5,920 of those rows inside
`RETURN_HOME`; `min_scan_m` 0.15 m; 40 `controller_failed_progress`, 38
costmap clears, spin x9 / wait x9 / backup x6; it ended at world (0.17, 0.35),
never reaching home. Ruled out by measurement: **localisation** (AMCL
CONSISTENT 6,873 of 7,486 samples, 0 degraded, final gap 0.113 m), **the
command path** (bypass 0, wheels above the monitor 0), **mission logic** (max
wheel speed **0.394 m/s** — these retries *drove the robot*, unlike
C2-NAV.44's 0.000 m/s) and **infrastructure**. What remains is the documented
`enclosure_entry`/PolygonStop deadlock class, on the return leg, and it is
**intermittent**: the same lane completed cleanly in the two other blue runs
with PolygonStop 0.

**Verdict: the M6 executive path is REGRESSION-PASSED under the fixed command
architecture**, with one recorded navigation failure not attributable to it.
**Not claimed:** three runs per colour is **not a rate** — 11/12 is not a 92 %
reliability figure, and blue 2/3 is one failure, not a blue failure rate.
Full report: `docs/agents/C2-NAV.46_RESULTS.md`.

(This section previously ended "The `r2_blue` deadlock is **classified, not
diagnosed**: no root cause, no fix." That is now false — it was diagnosed in
C2-NAV.48 and fixed. The 11/12 figure stands as history but is **no longer
the current baseline**: it was measured on `local_costmap.robot_radius` 0.20,
the value that caused its own single failure.)

**The `r2_blue` deadlock is DIAGNOSED and FIXED (C2-NAV.48, 2026-09-19).**
The local costmap and the collision monitor disagreed about which poses are
navigable. `cylinder_obstacle` — static at (−0.2, 0.6), r 0.2, h 0.6, tall
enough to span the LiDAR plane — had its surface **0.2486 m** from
`base_footprint`: **1.4 mm inside** PolygonStop's 0.25 m circle and **43 mm
outside** the costmap's real inscribed radius of **0.2060 m**. The planner
scored that pose 15.8 of 254; the monitor held the wheels. Nothing escaped
because PolygonStop is a **circle** with `action_type "stop"` — it is
direction-agnostic. Measured over the 5,955 held rows: Nav2 commanded motion
in **5,423**, including **905** rows of spin and **603** of backup, and the
wheels moved in **0**, with 49 mm of true chassis clearance. The first
divergence is **23.8 mm**: `r3_blue` passed the same obstacle at 0.2724 m and
went home.

The fix is one parameter — `local_costmap.robot_radius` **0.20 → 0.25**. The
costmap's threshold is not `robot_radius`: `Costmap2DROS` builds a 16-gon of
circumradius `robot_radius`, pads it by `footprint_padding`, and
`LayeredCostmap` takes the apothem, which C2-NAV.0 measured as 0.205879 m for
0.20. 0.25 gives **0.255004 m**, 5.0 mm past the stop circle, so the 44 mm
band closes. **PolygonStop is untouched** (C2-NAV.6 ruled neither of its knobs
should move) and the **global costmap deliberately stays at 0.20** — at
`cost_scaling_factor` 5.0 it already prices that pose at 203.6 of 254 and
avoids the band unaided, while the local costmap's 65.0 prices it at 15.8.
The defect is local, so the fix is local. `docs/agents/C2-NAV.48_RESULTS.md`.

**The colour matrix is RE-MEASURED on the corrected costmap (C2-NAV.49,
2026-09-19): 12 of 12 fetches, 12 valid runs, 0 void — red 3/3, green 3/3,
blue 3/3, yellow 3/3.** Three fresh executive-driven missions per colour, all
four colours re-run (no lane carried over — a changed costmap parameter
invalidates every lane), fresh simulator each, headless, never `--fast`,
depth fusion off, `dirty_paths=0`, 22 runner checks passed and 0 failed in all
twelve, 16 nominal states, 0 re-entries, Nav2 goals **2 succeeded / 0 failed
/ 0 aborted**, **0** recoveries, **0** relocalizations. **No runtime code was
changed to produce it.**

**The deadlock did not recur, and the mechanism was never exercised.** Over
all twelve traces: PolygonStop rows **0**, episodes **0**, stop duration
**0.0 s**, stop-active-with-wheels-driven **0**, `controller_failed_progress`
**0** (against 40 in `r2_blue`), costmap clears **0**. Closest approach to
`cylinder_obstacle` in any run **0.2894 m** (`r1_blue`, return leg) —
**39.4 mm outside** the stop circle. Blue is the exposed lane and that is
geometric: blue **0.2894 / 0.4024 / 0.3335 m**, all three on the return leg,
against green 0.4054–0.5343, yellow 0.4532–0.4628 and red never closer than
**0.6214 m**. **But no run entered the 0.2059–0.25 m band at all**, so none of
these twelve would have deadlocked on 0.20 either: this is **not an A/B of the
fix**. It shows no recurrence and no regression on the corrected value.
**Twelve runs is not a rate and 12/12 is not a 100 % figure.** `min_scan_m`
cannot be used here — it saturates at the 0.15 m LiDAR floor in all twelve.

Command path: **bypass 0, stale drops 0, PolygonStop 0 in all 12**. Wheels
above the monitor **8 of 7,781 nav-active samples = 0.1028 %**, worst gap
0.1250 m/s — *higher* than C2-NAV.46's 0.0411 %, still the unattributed
short-streak residual, **not** claimed fixed or improved. Grasp **12 of 12**
inside `[0.1510, 0.1565]` (0.1535–0.1547 m), lift 34.1–36.6 mm.

**Green's pre-ramp discrepancy reproduces on a second independent sweep and
is still green's alone**: green **0.298 / 0.307 / 0.330 m** (C2-NAV.46:
0.315–0.348) against red 0.088–0.131, blue 0.080–0.114 and yellow
0.040–0.058, all clean inside the original 0.25 m. The outer 0.50 m band was
**never reached** and futile retries were **0 in all 12**.

**`robot_radius` 0.25 is KEPT as the local-costmap default; PolygonStop stays
0.25 / 4; the global costmap stays 0.20.**
`docs/agents/C2-NAV.49_RESULTS.md`.

**Depth perception (C2-NAV.43): optional candidate, not default.** No sensor
was added. The gz `/camera/points` cloud was measured to be in the x-forward
link convention under an optical frame_id, and a full-resolution depth cloud
(1.23 MB) was delivered to a best-effort subscriber once in 12 s. So
`depth_cloud.launch.py` builds a half-resolution cloud from the bridged
depth image. It was measured at 14.97 Hz, with floor points within 5.3 mm
of the floor and ramp points within 5.2 mm of the wedge surface.
Measured:

- At six fixed poses, 0 phantom costmap cells in every arm.
- The LiDAR represents the 18° ramp as a one-cell wall at x 1.625, covering
  0.086 of the ramp area the camera can see; fusion covers 0.904.
- Three fresh tours per arm (topology B): legs 16/21 → 18/21,
  `enclosure_entry` 1/3 → 2/3, `enclosure_exit` 0/3 → 1/3, deadlocks 5 → 2,
  ordinary legs 15/15 in both, min true clearance 0.2449 / 0.2434 m.
- Nav2 container CPU 1.196 / 1.195 cores, plus 0.125 cores for the depth
  nodes.
- **Cost:** off-geometry local-costmap marks while driving rose 3.2×
  (15,438 → 50,068 cells), all near real geometry.

N = 3. Nothing made default; `nav2_params.yaml` is byte-identical.

The paragraph below describes `main` before this branch.

**0. The collision monitor cannot stop this robot.** `/cmd_vel_nav` has
**7 publishers and 2 subscribers** on the live graph: `nav2_bringup`
remaps `controller_server` and `velocity_smoother` onto it, and
`cmd_vel_relay`'s arbiter output points at the same topic, so the
collision monitor's output is fed back into the velocity smoother's
input and the arbiter sees raw and gated commands together. Measured
over three runs: the wheels receive **10.15–10.77 Hz more than the
collision monitor publishes**, exactly `controller_frequency: 10.0`.
During an active SLOWDOWN with a gated cap of 0.090 m/s, wheel commands
reached **0.300 m/s** — 84.2 % of one run's slowdown samples. **A safety
defect, characterized and NOT fixed.** The fix is a topic rename (give
the relay its own output and point the arbiter at it); it was
deliberately not done because the wheel path is frozen and the standing
19/20 was measured with the loop in place. Whoever changes it owes a
measured run and a statement about comparability.

**1. Severe confident divergence is detected but not recovered.** The
monitor sees an injected 3 m pose error and the executive safe-stops and
re-seeds AMCL, and the mission still cannot get home. Two measured
causes, neither in the executive: `amcl.recovery_alpha_fast` and
`recovery_alpha_slow` are **0.0**, so AMCL has no random-particle
injection and cannot leave a mode it is confident in; and
`/reinitialize_global_localization` converged to world **(2.60, −0.64)**
— inside the wedge footprint — because the map's 2D slice is highly
self-similar and a 360° scan from a standing robot does not disambiguate
it. The planner then reported "Start occupied" and "no valid path
found". **No live run has produced degradation → recovery → resume →
COMPLETE.** `RESULTS.md` records that path as UNIT-TESTED ONLY.

**2. `RETURN_HOME` fails by at least two distinct mechanisms.** Six
failed and five succeeded across all recorded sessions — still not a
rate; the standing figure is M6's 19/20. Class A (the pose is wrong and
the filter is sure) is detectable from onboard signals in **0.4 s**, but
**not by covariance**. Class B — an uninjected failure with position
error inside the healthy band (0.300 m median) and yaw error reaching
1.31 rad — is **not separable by any signal recorded**, and no threshold
is proposed for it.

**3. Detection latency varies by 25×.** The same class-A injection
measured **3.33 s, 4.52 s and 82.9 s** across three runs. Three is not a
sample, so the figure in `RESULTS.md` is a range and not a number.

**4. `--target` grasp re-targeting does not work** away from the tuned
point — recorded as 0/5 and later 5/14, rather than omitted.

**5. The lateral reachability verdict is a lower bound**, not a limit:
`approach_server`'s align phase nulls the bearing first, so +30 mm
lateral reached the grasp as −3.0 mm and grasped.

**6. Counts are not rates.** 60 placements, 8 grasps, 5 localization
runs and 1 release mission are each small deterministic samples.

**7. `rviz_2d_overlay_plugins` is not installed**, so
`mission_hud._publish_overlay` has never executed. It degrades cleanly to
the String topic.

**8. The Docker images are untested** — no Docker on the development
machine. Provided for reproducibility only.

The full development-era problem list, with the diagnosis for each, is
kept below under KNOWN PROBLEMS.

---

## MILESTONE STATUS — ALL CLOSED

- **C2-M1 (observability): COMPLETE and verified.**
- **C2-M1.5 (runtime integrity gate): COMPLETE.**
- **C2-M1.6 (RViz presentation): COMPLETE.** Map quality classified GOOD.
- **C2-M2.0 (terrain observer): COMPLETE.**
- **C2-M2.1 (benchmark + verdict): COMPLETE. C2-M2 closed.**
- **C2-M3.0 (mission executive): COMPLETE.** A full fetch ran through it.
- **C2-M3.1 (live failure injection): COMPLETE.** Five live missions,
  four deliberately broken; **no defect found and no source changed.
  C2-M3 closed.**
- **C2-M4.0 (perception → 3D pose → TF → reachability): COMPLETE.**
- **C2-M4.1 (four-colour benchmark + grasp integration): COMPLETE.**
  60/60 placements, 720/720 frames, horizontal error 0.7 / 1.4 / 2.4 mm;
  **8/8 live grasps physically verified**, 7/8 placements.
- **C2-M4.2 (integration gate): COMPLETE. C2-M4 closed.**
- **C2-M5.0 (localization characterization): COMPLETE.** Covariance
  measured to be the **wrong** signal.
- **C2-M5.1 (monitor, recovery, resume): COMPLETE**, with the
  severe-divergence limitation above recorded rather than rounded up.
  **C2-M5 closed.**
- **C2-M6 … C2-M9: scoped, not undertaken.** See `docs/ROADMAP.md`.
  They are a record of what was designed, not pending work.

---

## NO NEXT MILESTONE

COCO 2.0 is finished. Any future work is a new version or a new project.

If someone does pick it up, the honest starting point is the one thing
C2-M5.1 could not do — KNOWN LIMITATIONS 1 — and the decision nobody has
taken, KNOWN LIMITATIONS 0. Turning the `recovery_alpha_*` pair on is the
obvious next experiment and is **not obviously right**: the standing
19/20 was measured with them at 0, and changing an AMCL parameter to make
one mission pass is exactly what the evidence discipline in `CLAUDE.md`
exists to prevent. It needs measuring, on both a healthy matrix and the
injection.

---

## MILESTONE NUMBERING — READ THIS OR YOU WILL WORK ON THE WRONG THING

Two schemes exist and **they collide**:

| Scheme | Meaning | Status |
|---|---|---|
| **M0–M6** | v1, the wedge world. The fetch mission. | **CLOSED**, 19/20 measured |
| **M7** | v2, "The Yard" — randomised terrain, MuJoCo, RL baselines | Phases 1–3 done, **Phase 4 gated** |
| **C2-M1 … C2-M9** | The **COCO 2.0** plan. | **C2-M1 … C2-M5 done and merged.** C2-M6 … C2-M9 scoped, not undertaken |

"M2" is ambiguous. **Always write `C2-M2` for the COCO 2.0 plan** and
plain `M2` only for the historical v1 milestone.

---


---

## MEASURED RECORD — C2-M5, the final milestone

**C2-M5.0 is COMPLETE, and the headline is that the obvious design was
wrong.** Five missions, one fresh simulator each, clean graph, sim time,
`rviz:=false`, **never `--fast`**, `target_source:=target_pose`, colour
blue. Recorder `docs/data/c2m5_locrec.py` at 10 Hz; raw CSVs committed.

| run | injection | RETURN_HOME | outcome | true error, median |
|---|---|---|---|---|
| `healthy1` | none | 80.3 s | **COMPLETE**, home to 0.078 m | 0.257 m |
| `healthy2` | **none** | 12.0 s, 3 attempts | **ABORT** | 0.300 m |
| `obstacle1` | a cylinder into the corridor | 50.0 s | **COMPLETE**, home to 0.079 m | 0.190 m |
| `diverged1` | `/initialpose` −3 m in y, tight covariance, + heading error | 131.5 s | **ABORT** `RETURN_FAILED` | 2.824 m |
| `diverged2` | the same, heading preserved | 24.7 s | **ABORT** `RETURN_FAILED` | 3.248 m |

**`healthy2` failed with no injection at all** — the spontaneous
return-home failure KNOWN PROBLEMS 1 describes, caught with
instrumentation running for the first time. Its position error stayed
inside the healthy band (0.300 m median) while its **yaw** error reached
1.31 rad; the plan lengthened 9.73 → 13.93 m, motion stopped for the
final 4 s, and `navigate_to_pose` returned ABORTED.

**Two successes and three failures is not a rate**, and two of the three
failures were induced. The standing mission figure is still M6's
**19/20**.

**Detection latency, measured against `healthy1`'s own envelope:**

| signal | `diverged1` | `diverged2` |
|---|---|---|
| scan-vs-map mean endpoint distance | **+0.4 s** | **+0.4 s** |
| AMCL `sigma_xy` | +24.5 s | +13.9 s |

**Covariance points the wrong way.** On common ground `diverged2`, 3.14 m
wrong, reported `sigma_xy` **0.281** against 0.370 / 0.389 / 0.372 on the
legs that were right. `healthy2`, the uninjected failure, had the lowest
whole-leg median of all five. Part of the dip is imposed by the
injection; the time AMCL took to notice is not.

**Collision-monitor activity is not the discriminator, in either
direction.** `obstacle1` (finished) and `diverged1` (aborted) logged the
**same 36 PolygonLimit entries**; `diverged2` was 3.2 m wrong with the
monitor at `DO_NOTHING` for the whole leg. And
`/collision_monitor_state` is **edge-triggered** — `healthy1` received
**zero messages in 219.7 s**, so silence and "not running" are identical
to a subscriber. **`PolygonStop` never fired in any of the five runs.**

**Not reproduced:** the 2026-08-17 `PolygonStop` stall and the 4.8 Hz
control loop. RTF never fell below 0.818 and `/scan` held 10 Hz, with
RViz off throughout. Consistent with load-induced; not established.

**`localization_health.py` is imported by nothing**, by design, and its
`Thresholds` has **no defaults** — it cannot be constructed without
someone naming every number, and `classify()` returns `UNKNOWN` rather
than guessing. `UNKNOWN` is falsy so `if health:` cannot read it as good
news. 30 unit tests, including one that reads the observation
dataclass's own field names and fails if anything ground-truth-shaped
appears.

---

**C2-M4.2 is COMPLETE, and the headline is that a whole fetch ran on the
measured pose.** One mission, fresh simulator, clean graph, never
`--fast`, `rviz:=false`: **COMPLETE — all 16 states, `retries=0`,
`reason=--` at every sample, 178 s** from LOCALIZE to COMPLETE. Exactly
**one** publisher on `/perception/target` **and** on
`/perception/status`, both `target_pose_node`, verified **before and
after**; `target_finder` never ran.

**The swap needed a second topic, and that was found by reading rather
than by spending a run.** C2-M4.1's `point_topic` feeds
`approach_server` and is all the *manipulation* chain needs. The
*executive* gates `SEARCH_TARGET` on **`/perception/status`** reading
`found=1` — `mission_states._check_search_target` — and that topic was
`target_finder`'s alone. Swap only the point topic and there are **zero
publishers** on the status topic, `SEARCH_TARGET` never leaves RUNNING,
and the mission dies on its 15 s timeout as `TARGET_NOT_FOUND`: a
topic-name problem wearing a perception diagnosis. **First broken
boundary: the subscriber assumption.** Type, QoS and frame were already
compatible.

The fix is `status_compat_topic`, the other half of the same handover,
and **`target_source:=target_pose`** in `perception.launch.py` sets both
together in an `OpaqueFunction` — one node by construction, an unknown
value raises. **The default is still `target_finder`**, so the path
M6's 19/20 was measured on is untouched.

Chain, measured: 62 `found=1` samples and **62 `validity=VALID` samples
— the same number**, which is the check that `found` is exactly
`validity == VALID`. 190 points on `/perception/target`. Approach
`arrived`, travel 1.139 m, bearing nulled to `-0.000`. Grasp
**`x=0.1540`** held then placed — inside the 5.5 mm window, from the
camera. Record: `docs/data/c2m42_mission.log`.

**One run is not a rate.** The standing mission figure is still M6's
**19/20**.

---

**C2-M4.1 is COMPLETE, and its headline is that the robot grasps a
target using a position it measured itself.** Eight live runs, one
fresh simulator each, never `--fast`, `target_finder` deliberately not
running and the publisher count on `/perception/target` verified 1
before every run.

| | |
|---|---|
| perception VALID at the start | **8 of 8** |
| approach `arrived` | **8 of 8** |
| `check_target_pose` accepted the perception-derived fix | **8 of 8** |
| IK + MoveIt planned and executed | **8 of 8** |
| **grasp physically verified** — the object rose, read from gz | **8 of 8** |
| **placement physically verified** — back on its own deck | **7 of 8** |
| every fix inside the 5.5 mm window [0.1510, 0.1565] | **8 of 8**, 0.15341-0.15471 |

**The integration is one parameter, not a rewrite.** `target_pose_node`
gained `point_topic`, **empty by default**; set to `/perception/target`
it stands exactly where `target_finder` stood and the whole downstream
chain runs unmodified. `approach_server`, `grasp_server`, `arm_ik`,
`arm_control` and MoveIt are **byte-identical**.

**Perception: 60 of 60 placements, 720 of 720 frames detected, 0
wrong-colour selections.** Horizontal error **0.7 / 1.4 / 2.4 mm**
(min/median/max), colour-independent to within 0.47 mm of median.
Frame-to-frame spread **0.0000 m in all 60** — bias, not noise, and a
statement about gz's noiseless depth camera rather than about a real
sensor.

**The lateral residual is sub-pixel and geometric.** On-lane, `dy` is
identical across all four colours to within 0.01 mm at every stand-off —
four diameters, four lanes, one number — and equals **0.29 to 0.43
pixels** on the 320x240 sensor. `CameraInfo` reads `cx=160.00` on a
320-wide image, half a pixel off the geometric centre, which is the
right sign and order; the equivalent offset *rises* across the sweep
rather than holding flat, so the mechanism is **not claimed**.

**THE RESULT: the static reachability verdict is a lower bound, not a
forecast.** Both lateral placements were judged `OFF_ARM_PLANE` and
**both grasped successfully**:

| lateral | perception `y` | static verdict | `y` delivered to the grasp | live |
|---|---|---|---|---|
| −0.010 | +10.2 mm | OFF_ARM_PLANE | **+1.68 mm** | **grasped, verified** |
| +0.030 | −29.2 mm | OFF_ARM_PLANE | **−3.0 mm** | **grasped, verified** |

`approach_server`'s `align` phase pivots until the bearing is nulled and
only then takes the fix the creep and the grasp use, so lateral offset
is **absorbed, not carried**. `reachability_after_approach`'s docstring
models the approach as a straight forward creep that "leaves y alone";
the real approach also turns, so the verdict **under-predicts**
feasibility. That is the safe direction for a gate to be wrong in, and
it was **not changed**. Both lateral runs are `n = 1`; 30 mm is the
largest offset tried, not a characterised limit.

**`min_range`: decision B — no change, envelope documented instead.** At
the 0.30 m operating floor C2-M4.0's defect is **already gone**: `qual`
reads **0.9989 or better** against **0.0423-0.0706** at 0.28 m, and `dx`
is the ordinary negative far-field residual (−0.68 to −1.58 mm), not the
positive radius-proportional one. It stays at 0.15 because that matches
`target_finder`, because the defect does not occur inside the envelope,
and — the reason that generalises — because **`qual` announces the
failure without ground truth**. The characterised envelope starts at
**0.30 m of stand-off**.

**TWO UNSTATED PRECONDITIONS FOUND, BOTH RECORDED AND NEITHER FIXED.**

1. **`check_lifted` verifies the object moved up, not that it is
   upright.** The one placement failure (`blue`, 0.30 m) was a cylinder
   **toppled during the pick sequence** — the instrument read it
   standing at 0.72884 right after the approach, `grasp_server`'s own
   pre-grasp read was 0.6638, and `0.64984 + r = 0.66384` is exactly a
   blue cylinder lying on its side. The magnet welded to it, lifted it
   43.7 mm, and `check_lifted` **passed**, correctly by its contract.
   It was then carried and delivered lying down with every step
   reporting success. Which motion toppled it was **not isolated**
   (1 of 1 at 0.30 m, 0 of 4 at 0.45 m, 0 of 1 at 0.70 m).
2. **`check_released` asserts the object stands at `TARGET_HEIGHT/2` —
   the floor AT HOME.** All eight platform placements failed it,
   **including the seven that released perfectly**. Correct in the M6
   mission, where the robot *is* at home; the precondition was simply
   never written down.

**`GRASP_MAX_LATERAL` was NOT retuned**, and neither was `min_range`.

**READ THIS BEFORE SAYING "MANIPULATION IS VALIDATED".** Eight runs is
not a rate — the standing mission figure is still M6's **19/20**. The
robot was **placed** on the platform with `gz set_pose`, so the climb,
the lane hold and the crest transition were not exercised; the approach
was, from `crest` onwards. Placement was verified **on the platform**,
not at home. The executive, Nav2 and AMCL were **not in the loop** —
this is the perception -> approach -> grasp chain in isolation,
deliberately, to keep the Gazebo + RViz + `move_group` confound of
KNOWN PROBLEMS 1 and 3b out of the measurement. And **nothing launches
with `point_topic` set**: `perception.launch.py` still starts
`target_finder`, which is deliberate until a full mission has run on the
new path.

---

### C2-M4.0 (previous milestone, still standing)

Target localisation measured at **1.1-2.1 mm horizontal** over 16
placements at 0.35-0.90 m, four colours, **240 of 240 frames**. The
estimate tracks a moving target (70.1 mm measured against 70 mm
commanded). The `min_range` near-face defect was found there at 0.28 m
(`dx` +4.1 to +8.3 mm, proportional to radius; the `min_range:=0.11`
control gave −1.0 to −1.4 mm) and left for C2-M4.1, which decided it
above. `target_finder.py` remains **byte-identical**.

### C2-M3.1 (previous milestone, still standing)

**C2-M3.1 is COMPLETE, and its result is that nothing needed fixing.**
Five live missions, four deliberately broken, fresh simulator each,
never `--fast`. Every run followed its contract exactly — retry counts,
escalation targets and terminal states all matched `mission_states.py`
and the pure-harness tests. **`mission_states.py` and
`mission_executive.py` are byte-identical to C2-M3.0**, verified with
`git diff`; the C2-M3.1 commit is documentation and the measurements
behind it.

| Scenario | Trigger | Retries | Final |
|---|---|---|---|
| Operator abort during `CLIMB` | `/mission/abort` on a moving robot | **0** | `ABORT` `OPERATOR_ABORT` (x3) |
| Navigation failure | `--lane 5.0`, goal off the map | **2** | `ABORT` `NAVIGATION_FAILED` |
| Perception failure | `target_blue` removed from the sim | **2** | `ABORT` `TARGET_NOT_FOUND` |
| Manipulation failure | cylinder removed at `GRASP` entry | **2** | `ABORT` `GRASP_FAILED` |

All four routes into `RECOVERY` — operator request, navigation action
status, state timeout, worker terminal outcome — now have a live run.
Both escalations, `ESCALATE_ABORT` and `ESCALATE_SKIP_GRASP`, were
reached. Retry counts are exact, read from the executive's own
`attempts={...}` line, each equal to that state's `max_retries`.

**The abort, measured three times.** Fired on an observation, never a
timer: only once `/mission/state` read `CLIMB` **and** three consecutive
odometry samples exceeded 0.05 m/s. Last nonzero controller command at
**+20 / +30 ms**, then **10 explicit zero commands over 0.88 s**
(`ZERO_HOLD_SECONDS = 1.0`) — the wheels are commanded to stop, not left
to age out. Travel after the abort **13.1 / 15.3 / 23.6 mm**. No stale
command resumed motion: `max |vx| = 0.0` across 50, 264 and 482 later
samples. **0 states entered after `ABORT` in 5 of 5 runs.**

**`cmd_vel` invariant held throughout: 1,134 publisher-count samples
across five runs, every one of them 1.**

**READ THIS BEFORE SAYING "RECOVERY IS VALIDATED".** Four
*representative* branches ran live. **These did not** and remain
unit-tested only: `CLOCK_STALLED`, `--no-grasp` through the executive,
`NAVIGATION_REJECTED`, `NAVIGATION_UNAVAILABLE`, `SERVICE_UNAVAILABLE`,
`SERVICE_REFUSED`, `RECOVERY_TIMEOUT`, every `ALIGN_*`, `CLIMB_TIPPED`,
every `DESCENT_*`, `RETURN_*`, `STOW_*`, `APPROACH_*`, `PLACE_*`,
`VERIFY_PLACEMENT`. The **no-stale-completion** invariant was never
provoked either — no late worker reply arrived after a cancel — so it is
still argued from the code, not measured. The correct sentence is *"live
validation completed for operator abort, navigation failure, perception
failure and grasp retry."*

**And one clean run is still not a rate.** These five runs say the
failure paths behave; they say nothing about how often the mission
fails. The standing figure remains M6's **19/20**.

**One instrumentation trap, paid for and recorded.**
`/diff_drive_controller/cmd_vel` carries **both** `Twist` and
`TwistStamped`; the arbiter publishes `TwistStamped`. A `Twist`
subscriber matches nothing, captures nothing, and raises nothing, while
`ros2 topic info` still reads healthy — and an empty capture looks
exactly like "no stale command was issued", which is the conclusion the
test existed to reach. In `CLAUDE.md`'s trap table now: **any check
whose success condition is "we saw nothing" must first prove it can see
something.**

**One C2-M3.0 open item confirmed live, not fixed.** `grasp_server`
writes `outcome=failed at magnet attach` into a space-separated
`key=value` line, so `parse_kv` keeps `failed` and drops the diagnosis.
Classification is still correct; `grasp_server` is out of scope.


---

## COMPLETED (C2-M4.2) — the mission runs on the measured pose

| Item | Outcome |
|---|---|
| **The headline** | **A full fetch completed through the real mission executive with `target_pose_node` in `target_finder`'s place.** All 16 states, **`retries=0`**, `reason=--` at every sample, **178 s** LOCALIZE→COMPLETE |
| **The defect, found statically** | `point_topic` alone is not enough. `mission_states._check_search_target` gates `SEARCH_TARGET` on **`/perception/status`** reading `found=1`; `target_pose_node` publishes `/perception/target_pose/status`, a different topic with no `found` key. Swap the point topic only and the executive sees **zero publishers**, `SEARCH_TARGET` stays RUNNING, and the run dies on its 15 s timeout as `TARGET_NOT_FOUND` |
| First broken boundary | **the subscriber assumption.** Message type (`PointStamped`), QoS (depth 10, RELIABLE) and frame (`base_footprint`) were all already compatible |
| The fix | `status_compat_topic`, empty by default, `found=1` **iff** `validity == VALID`, on the existing 5 Hz timer. The line is rendered by **`target_finder.format_status`**, so the format keeps exactly one definition |
| The selector | **`target_source`** in `perception.launch.py`, dispatched in an **`OpaqueFunction`** — one node by construction, an unknown value **raises**, and **both** handover parameters are set together. `mission.launch.py` declares and forwards it |
| Publisher invariant | **1** on `/perception/target` and **1** on `/perception/status`, both `target_pose_node`, verified **before and after** the run. `target_finder` never ran; one executive; `/amcl` `active [3]` |
| Both legacy consumers | `mission_executive` **and** `mission_hud` took the compat line unchanged |
| The chain, measured | **62 `found=1` = 62 `validity=VALID`** (the same number, which is the check). **190** points published. Approach `arrived`, travel **1.139 m**, bearing nulled to `-0.000`. Grasp **`x=0.1540`** held, then placed — inside [0.1510, 0.1565] |
| `RETURN_HOME` | **succeeded in 59.9 s** — KNOWN PROBLEMS 1's leg, second consecutive success under light load with RViz off. **Three of six recorded legs have failed; six is not a rate** and it stays open |
| Default unchanged | **`target_finder` is still the default.** `approach_server`, `grasp_server`, `arm_ik`, `arm_control`, `mission_states`, `mission_executive` **byte-identical** |
| Tests after the work | **684 passing / 0 failing**, up from 662, on a clean graph. All +22 in `coco_perception` (117 → 139) |
| Record | `docs/data/c2m42_mission.log` |
| Checkpoint committed and pushed | `8c3660c` on `jazzy2/coco2-m1-observability` |

**This is ONE run.** The standing mission figure is still M6's
**19/20**. It is an existence proof that the swap survives the
executive — not a rate, not a comparison against `target_finder` on the
same course, and **no claim that the new path is better**. It is
measured to work, not to win.

**`VERIFY_PLACEMENT` passed, and that is a precondition holding, not a
fix.** `check_released` asserts the floor height **at home**, and this
mission places at home. C2-M4.1's finding that it fails every correct
*platform* placement stands; the platform figure stays **7 of 8**.
`check_lifted` still verifies the object moved **up**, not that it is
**upright**. Neither was changed — the gate did not require it.

---


---

## COMPLETED (C2-M4.1) — the grasp runs on the measured pose

| Item | Outcome |
|---|---|
| **The headline** | **The robot picks the target up using a position it measured itself.** 8 live runs, fresh simulator each, **grasp physically verified 8 of 8** (the object's own height read from gz, not an action result), placement verified **7 of 8** |
| The benchmark | **60 of 60 placements, 720 of 720 frames detected, 0 wrong-colour selections.** Horizontal error **0.7 / 1.4 / 2.4 mm** (min/median/max), colour-independent to within 0.47 mm of median |
| The integration | **one parameter.** `target_pose_node` gained `point_topic`, empty by default; set to `/perception/target` it stands where `target_finder` stood. `approach_server`, `grasp_server`, `arm_ik`, `arm_control` and MoveIt are **byte-identical** |
| Every fix in the window | **8 of 8**, spanning 0.15341–0.15471 inside [0.1510, 0.1565] — 1.3 mm of spread in a 5.5 mm window, across four colours and three stand-offs |
| **The result** | **the static reachability verdict is a LOWER BOUND, not a forecast.** Both lateral placements read `OFF_ARM_PLANE` and **both grasped**: −0.010 arrived at the grasp as **+1.68 mm**, +0.030 as **−3.0 mm**. `approach_server`'s `align` nulls the bearing before the fix is taken, so offset is **absorbed, not carried** |
| The model gap, recorded | `reachability_after_approach` credits the approach with translation and not rotation, so it under-predicts feasibility. Safe direction; **not changed** |
| Lateral bias, diagnosed | **sub-pixel and geometric.** On-lane `dy` is identical across all four colours to within **0.01 mm** at every stand-off, and equals **0.29–0.43 px** on 320×240. `cx=160.00` on a 320-wide image is half a pixel off centre — right sign and order, but the offset *rises* across the sweep, so the mechanism is **not claimed** |
| `min_range` | **decision B: no change.** At the 0.30 m floor `qual` reads **0.9989+** against **0.0423–0.0706** at 0.28 m and `dx` is the ordinary far-field residual. Envelope documented instead; **`qual` is the runtime tell and needs no ground truth** |
| **Defect 1, not fixed** | **`check_lifted` verifies the object moved up, not that it is upright.** The one placement failure was a cylinder **toppled during the pick** (standing centre 0.72884, observed 0.66384 = deck + radius). The magnet lifted it 43.7 mm, `check_lifted` **passed**, and it was delivered lying down with every step reporting success |
| **Defect 2, not fixed** | **`check_released` asserts the floor height AT HOME** (`TARGET_HEIGHT/2`). All 8 platform placements failed it, **including the 7 that released perfectly**. Correct in the M6 mission; the precondition was never written down |
| Instruments | `docs/data/c2m4_analysis.py` (post-processing, no simulator), `docs/data/c2m4_grasp.py` (one grasp, one fresh sim). Data: `c2m4_benchmark.csv`, `c2m4_grasp.csv`, `c2m4_scatter.png` |
| Tests after the work | **662 passing / 0 failing**, up from 656, on a clean graph |
| Checkpoint committed and pushed | `33028ed` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** `GRASP_MAX_LATERAL` and `min_range` were
**not retuned**; `target_finder.py`, `approach_server.py`,
`grasp_server.py`, `arm_ik.py`, `arm_control.py`, MoveIt, the arbiter,
Nav2, AMCL, the map, the robot model, the world, the action space and
the shipped policy are untouched. `point_topic` is **opt-in and nothing
launches with it** — `perception.launch.py` still starts
`target_finder`.

**Was "not done", now closed by C2-M4.2:** the full mission through the
executive on the new path, including the climb and the delivery at home.
8 grasp runs is still not a rate, and the standing mission figure is
still M6's **19/20**.

---


---

## COMPLETED (C2-M4.0) — the target's position, measured

| Item | Outcome |
|---|---|
| **The headline** | **Target localisation error 1.1 / 1.6 / 2.1 mm horizontal** (min/median/max) over 16 placements at 0.35–0.90 m stand-off, four colours. **240 of 240 frames detected**, 20 of 20 placements measured |
| Colour independence | **within 0.8 mm** across red/green/blue/yellow — the evidence that one configured pipeline serves all four, with no per-colour branch anywhere |
| Corroboration | independently confirms the `~2.0 mm` perception residual `GRASP_MAX_LATERAL`'s comment has carried since M5 as a *budget line*. It is now a measurement |
| Bias, not noise | frame-to-frame spread **0.0000 m in all 20 placements**. Also a statement about gz: the simulated depth camera is noiseless |
| Tracks a moving target | robot parked, **target** displaced: **70.1 mm measured against 70 mm commanded** in x, **100.9 vs 100 mm** in y, and "home" repeated to the last digit after an excursion |
| **The defect** | **`min_range` gates an extended object by its near face**, a radius closer than its axis. At 0.28 m stand-off `dx` = **+4.1 / +5.5 / +6.9 / +8.3 mm** for d = 20/24/28/32 mm — proportional to radius, which is the signature |
| The control | the identical placements at `min_range:=0.11` gave **−1.0 / −1.0 / −1.3 / −1.4 mm**. One parameter changed, nothing else. **Not fixed** — C2-M4.1's call |
| Self-announcing | `hypothesis.score` (usable-depth fraction) read **1.0000 from 0.35 m out** and **0.0423–0.0706 at 0.28 m**. The failure is detectable **without ground truth** |
| A second, separate effect | `dz` at 0.28 m was −4.3 to −5.4 mm and **did not move** when the gate was lowered. That is the framing effect `target_finder`'s docstring predicted — the cylinder's top has left the frame. Costs the grasp nothing: `grasp_point.z` is `TARGET_GRASP_Z` |
| The far-field bias, explained | `SURFACE_TO_AXIS = 0.8` under-shoots a cylinder's true median offset of `0.866r` by `0.066r` = −0.7 to −1.1 mm. **Recorded, not tuned** |
| Frame semantics | `frame_id = base_footprint` 20 of 20; **`tf_age = 0.0000 s` 20 of 20** — the transform resolves at the image's own stamp, asked for there rather than at "latest" |
| Identity | `id` and `class_id` matched the requested target 20 of 20 |
| Reachability | `reach = OUT_OF_WORKSPACE` 20 of 20 — **correct**, the arm reaches base-x 0.157 and perception sees the target from 0.28 m — and `reach_appr = REACHABLE` 20 of 20, evaluated at `approach_stop_x` with the *measured* lateral offset |
| Multiple targets | exercised, not assumed: at 0.9 m the neighbouring lanes enter frame and `seen` reported two or three colours while `cand` stayed 1 and `id` stayed correct |
| Validity, live | target present → `VALID` with a full pose; `gz remove` of the model → `NOT_DETECTED`, every field `--`, and a **zero-length `detections` array still publishing** |
| Arbiter invariant | publisher count **1** on `/perception/target_pose` and **1** on `/perception/grasp_point` |
| Tests after the work | **656 passing / 0 failing**, up from 589. `coco_perception` 44 → 111 |
| Checkpoint committed and pushed | `16e952f` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** `target_finder.py` is **byte-identical**,
and so is everything in `coco_moveit_config`. `/perception/target` still
carries `PointStamped` in `base_footprint` and `approach_server`'s servo
mode still consumes it — the path M6's 20/20 approach was measured
through. C2-M4.0 added a path *beside* the measured one. Also untouched:
`cmd_vel_arbiter`, `grasp_server`, MoveIt, Nav2, AMCL, SLAM, the map,
the robot model, the world, the action space, the shipped policy.

**Not done:** no grasp, no driven approach (the robot was **placed**
with `gz set_pose`), on-lane only. Lateral error is exactly what decides
post-approach feasibility and C2-M4.0 did not sweep it.

---


---

## COMPLETED (C2-M3.1) — the failure paths on the robot

| Item | Outcome |
|---|---|
| **The headline** | **No defect found.** Five live missions, four deliberately broken; every one followed its contract exactly. `mission_states.py` and `mission_executive.py` are **byte-identical to C2-M3.0** |
| Operator abort mid-climb, **x3** | Fired on an observation (state `CLIMB` **and** three odometry samples > 0.05 m/s), never a timer. `RECOVERY` at +36/+44/+104 ms, arbiter `active=none` at +44/+152/+158 ms, `ABORT` at +180/+204/+304 ms. **Travel after the abort 13.1 / 15.3 / 23.6 mm** |
| The stop is **commanded** | Last nonzero controller command +20/+30 ms after the call, then **10 explicit zeros over 0.88 s** — `ZERO_HOLD_SECONDS = 1.0`. Not a watchdog coast |
| No stale command | `max |vx| = 0.0` and `max |wz| = 0.0` across 50, 264 and 482 samples after the last motion, in the three abort runs |
| Navigation failure | `--lane 5.0` puts the pre-ramp goal off the map — **measured** from `coco_world.pgm` (free cells map-y `[-4.585, 3.565]`, array ends `3.840`), not guessed. Nav2 aborted all three goals: `"Goal Coordinates of(2.500000, 5.000000) was outside bounds"`. `IDLE → ABORT` in **1.2 s** |
| Perception failure | `target_blue` removed with `gz service .../remove`; **`coco_perception` untouched**. `SEARCH_TARGET` timed out **15.09 / 15.00 / 15.09 s** against a 15.0 s contract |
| Manipulation failure | Cylinder removed at the instant `GRASP` was entered, after a **successful** approach (12.54 s). `grasp_server` ran its whole unmodified sequence and reported `outcome=failed at magnet attach`. Three attempts at **13.99 / 15.60 / 15.39 s** against a 180 s timeout — a genuine **worker outcome**, not a timeout |
| Retry counts | **Exact**, from the executive's own `attempts={...}`: `NAVIGATE_TO_RAMP` 2, `SEARCH_TARGET` 2, `GRASP` 2 |
| Both escalations reached | `ESCALATE_ABORT` (run 2) and `ESCALATE_SKIP_GRASP` (runs 3 and 4) |
| **No accidental COMPLETE** | Runs 3 and 4 descended and drove home (**120 mm**, **63 mm** from home) and still ended `ABORT` carrying the original reason |
| Arbiter invariant | **1,134 publisher-count samples across five runs, every one of them 1** |
| No state after `ABORT` | **0**, in 5 of 5 runs |
| Instrumentation trap found | `/diff_drive_controller/cmd_vel` carries **both** `Twist` and `TwistStamped`; the arbiter publishes the second. A `Twist` witness captured **zero** commands, which reads exactly like the result being sought. Cost one run; now in `CLAUDE.md`'s trap table |
| Tests after the work | **589 passing / 0 failing, unchanged.** Run on a **clean** ROS graph: with a live stack up, `coco_mission` gives 134/2 |
| Checkpoint committed and pushed | `9a7368c` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** everything. No source file was edited —
the failure injections are the executive's own documented `--lane`
parameter and simulator-side entity removal. `cmd_vel_arbiter`,
`ramp_driver`, `approach_server`, `grasp_server`, `target_finder`,
MoveIt, Nav2, AMCL, SLAM, the map, the robot model, the world, the
action space, the reward, the shipped policy and every C2-M2 artefact
are untouched.

**Still unverified live** (unit-tested only): `CLOCK_STALLED`,
`--no-grasp` through the executive, `NAVIGATION_REJECTED`,
`NAVIGATION_UNAVAILABLE`, `SERVICE_UNAVAILABLE`, `SERVICE_REFUSED`,
`RECOVERY_TIMEOUT`, every `ALIGN_*`, `CLIMB_TIPPED`, every `DESCENT_*`,
`RETURN_*`, `STOW_*`, `APPROACH_*`, `PLACE_*`, `VERIFY_PLACEMENT`, and
the **no-stale-completion** invariant.

---


---

## COMPLETED (C2-M3.0) — the executive, and what the live runs cost

| Item | Outcome |
|---|---|
| **The machine** | `mission_states.py`, **pure Python, no `rclpy`**: an `Observation` in, a `Directive` out. 18 states, a `StateContract` per state (mode, owner, timeout, max retries, retry target, escalation), ~40 structured failure reasons, one uniform failure path through `RECOVERY` |
| **The adapter** | `mission_executive.py`. Subscriptions → `Observation`; one idempotent request out. Offers `/mission/start` and `/mission/abort`. **Publishes no velocity** |
| **Live fetch** | **`COMPLETE`, `result=fetch`, 0 recoveries, 0 retries, 175.8 s, home to 7 mm.** All 15 nominal transitions in order |
| **Arbiter invariant** | `/diff_drive_controller/cmd_vel` publisher count **1**, measured before the mission and again after. Three tests assert it, one by asserting `Twist` appears nowhere in the package |
| **Stronger success conditions, all exercised** | Nav legs verified against the **ground-truth** world pose, not only the action's SUCCEEDED; the climb verified against summit x **and** cross-track; the grasp against `lifted` re-read after the action returned; the place against `lifted=0` with `outcome=placed` |
| **Defect 1, `autostart`** | A launch configuration is inherited by every include and **shadows** the included file's own default. `mission.launch.py`'s `autostart` became `nav2_bringup`'s. Every lifecycle node `unconfigured`; `/amcl_pose` **0 publishers**; `/clock` healthy at 378 Hz; `ros2 param get /lifecycle_manager_localization autostart` → `False` against a params file that never mentions it. **Nothing in any log said the word.** Fixed twice: `mission_autostart`, and `nav.launch.py` pins Nav2's `autostart` |
| **Defect 2, the heading gate** | Gated on nav2_params' 0.25 rad `yaw_goal_tolerance`. Measured **+0.28** and, re-driven, **+0.26** rad; run 4 measured **+0.281**. All inside Nav2's checker, all outside a ground-truth gate, because Nav2 judges yaw against AMCL. Re-driving is structurally futile — the same goal checker cannot beat its own tolerance. **Gate off by default; the number is measured, logged and exposed** |
| **Bug caught by the unit tests first** | A `RECOVERY` that timed out was routed through the ordinary failure path, which re-entered `RECOVERY` and reset its clock — the mission would have sat there for ever. It escalates to `ABORT` now |
| **KNOWN PROBLEMS 3b did not reproduce** | Descent `outcome=goal` in **16.5 s** against 90.1 s twice in C2-M1.6 — under light load with RViz off, **which is exactly the confound 3b named**. Not closed |
| **KNOWN PROBLEMS 1 did not reproduce** | Nav home succeeded first time, against 2 failures in 4 recorded legs. Not closed |
| Tests after the work | **490 → 589**, 0 failing (+99, of which **35 construct the real node**) |
| Checkpoint committed and pushed | `fb2ed09` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** `traverse_demo.py` is **byte-identical** —
it is the harness the M4/M5/M6 numbers were measured with, and
`executive:=false` selects it. Nor were `cmd_vel_arbiter`, `ramp_driver`,
`approach_server`, `grasp_server`, `target_finder`, Nav2's planner /
controller / costmaps / behaviour tree, AMCL, SLAM, the map, the robot
model, the world, the action space, the reward, the shipped policy, or
any C2-M2 artefact. The only change outside `coco_mission` is the pinned
`autostart` in `nav.launch.py` and one pattern in `ros_clean.sh`.

---


---

## COMPLETED (C2-M2.1) — the benchmark, and what it actually shows

| Item | Outcome |
|---|---|
| **Live Gazebo observer validation** (C2-M2.0 never ran the node) | **PASSED, after three defects it exposed.** Every one invisible to the pure-core tests because nothing had ever *constructed* the node |
| Defect 1 | `is_best_effort()` called with **no argument** — it takes the topic. `TypeError` in `__init__`: **the node could not start at all** |
| Defect 2, the substantive one | The estimator was advanced from the **10 Hz publish timer**, so samples arrived exactly `MAX_AGE` apart and the observer withdrew itself on **431 of 431** samples of a full climb. Estimation now runs in the IMU callback at **50 Hz** — the rate C2-M2.0 fixed — and publication stays at 10 Hz |
| Defect 3 | `on_declared_flat` never passed, so the flat reference could never be learned. Now an explicit operator parameter |
| Live rates / integrity | `/imu` **49.1 Hz** (declared 50), `/terrain/state` **10.02 Hz**, **422/422** estimates finite, stamps monotonic sim-time |
| Live grade | **0.0000°** on the flat at confidence **1.000**; **0.0035°** off the built 18.000 at the settled tail |
| Arbiter invariant | `/diff_drive_controller/cmd_vel` publisher count **1**, measured before and after the observer started. The observer publishes `/terrain/state` and nothing else |
| B3 engage + fallback, live | Route B: bound established at t=3.10 s, **B3 engaged 167/200**, and on deliberate withdrawal fell to throttle **0.5** / lateral **3.0** — B1's shipped gains exactly |
| **Cross-engine bonus result** | τ settles at **0.3248** vs tan(18°)=0.3249 and **0.4865** vs tan(26°)=0.4877. C2-M2.0's pinning result was MuJoCo-only; **it now holds in Gazebo** |
| **The benchmark** | **1,440 intended, 1,440 completed, 0 runner errors.** Nothing dropped, retried or re-seeded |
| Estimator, by route | grade MAE **0.057 / 0.253 / 2.681°**, convergence **0.94 / 2.73 / 10.10 s**. Traction bound held on **100.0 %** of single-plane samples on all three routes |
| **The decision rule** | Applied **unchanged**. Gaps **+0.0 / +1.7 / +7.5 pp**. **RL justified on 0 of 3 routes** |
| **The caveat that matters** | On Route A B3 is **B1** (fallback 1.000, 120/120 identical), and B2 completes **97.5 %** against B3's **0.0 %**. The ascent gap is 0.0 pp because the task does not discriminate, **not** because estimation succeeded |
| Where the observer **hurts** | **Route C**: B3 ascends **58.3 %** against B1's **84.2 %** — 25.9 points worse than the baseline it falls back to — losing ascent on 32 seeds, engaging on only 13 % of steps, against a grade MAE of 2.681° |
| Route C tips, the C2-M2.0 open question | **Not smaller.** B1 106, B3 116 under the surface-relative terminator vs Phase 3's 101 under the absolute one. What changed is that it now fires at a **genuine rear-over** instead of 34° short of one. No improvement is claimed |
| Terminology corrected **before** the run | `mu_mae`→`sched_mu_gap_mae`, `mu_hat` on the wire→`mu_sched_input`, new `tau_minus_tangrade_*`. **No friction MAE is reported and none exists to report** |
| Tests after the work | **478 → 490**, 0 failing (+12, all constructing the real node) |
| Checkpoint committed and pushed | `7796d04` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** `baselines.py`, `yard_env.py`,
`terrain_observer.py` and `sensor_model.py` are **byte-identical to
C2-M2.0**, verified with `git diff` before the benchmark ran. The tuned
schedule, the routes, the seeds, the decision task and the
10-percentage-point margin did not move. Nor did Nav2, SLAM, AMCL, the
map, perception, the robot model, the terrain, the action space,
`cmd_vel_arbiter`, the reward, the shipped policy, or the v1 tip
terminator in its three non-Yard homes.

---


---

## COMPLETED (C2-M1.6)

| Item | Outcome |
|---|---|
| **Raw `/map` inspected separately from the costmaps** | **GOOD.** Five world landmarks → one rigid offset, **worst residual 25 mm = 0.50 cell**. No ghost walls; 156 of 186 occupied components are ≤ 2 cells |
| Map quality classified explicitly | **GOOD, no SLAM defect.** Nothing in SLAM changed |
| The one caveat, recorded not fixed | North/south walls have **0.55 m and 0.85 m** gaps in the far east corners the mapping drive never entered. They beat the robot's 0.297 m footprint but open onto **unknown** cells, and `track_unknown_space: true` + `allow_unknown: false` mean no plan can route through them |
| `mission.rviz` | Rewritten clean. Map α 1.0, global costmap **off**, local costmap α 0.22, TF and particles off, camera pane removed, near-top-down framing |
| `mission_debug.rviz` | **New**, and it is the C2-M1.5 view **byte-identical below its header** — verified by diff |
| Framing, measured not guessed | D 13 / pitch 1.45 / yaw 3π/2 → map **949 × 652 px**, margins 135/136/90/64. **36% larger linearly** than the preserved camera's 700 px, both fitting whole |
| Live visual verification | **Performed.** One fresh sim, 7 screenshots, both configs on the same run. Two defects found only by looking (robot lost to costmap blooms; laser invisible on white) and one config comment disproved (the camera pane costs **0** render width, 304 px of display tree) |
| Tests after the work | **435 passing / 0 failing** (+21) |
| Checkpoint committed and pushed | `a7bfc23` on `jazzy2/coco2-m1-observability` |

**Not changed, deliberately:** SLAM, slam_toolbox, map generation, Nav2
planner, controller, AMCL, costmap runtime behaviour, robot model,
terrain, PPO, mission sequencing, perception, controller gains, action
spaces. The only runtime addition is `rviz_config:=mission|mission_debug`
on `mission.launch.py`.

---


---

## COMPLETED (C2-M1.5)

| Item | Outcome |
|---|---|
| Fresh-session bootstrap, branch/state reconstruction | done; state files recovered from the trunk |
| Test baseline re-verified before any change | **404 passing / 0 failing** — matches the record exactly, workspace not stale |
| **Investigation B — `ROBOT PITCH = -0.314 rad`** | **DIAGNOSED and FIXED.** Stale ramp-driver state |
| **Investigation A — the failed fetch** | **First divergence identified**; `found=0` proved a consequence; nav home proved independent |
| **Investigation C — `/approach/target`** | **Semantics established. VOLATILE is correct. No change made** |
| RViz visual inspection | **Performed** — 5 screenshots of the rendered window. One objective defect found and fixed |
| Tests after the work | **414 passing / 0 failing** (+10) |
| Checkpoint committed and pushed | `1c99415` on `jazzy2/coco2-m1-observability` |

**The pitch answer, in one paragraph.** `ramp_driver` writes `self.pitch`
only inside its climb and descend loops; between segments nothing assigns
it while the 5 Hz status timer keeps publishing it. The message is never
late — the number in it is minutes old, which no subscriber-side
staleness check can see. `-0.314 rad` is a *genuine* 18° nose-up sample
taken on the ramp (`RAMP_ANGLE_DEG = 18`, 18° = 0.31416 rad), held
through the platform approach and the whole pick while `/imu` had already
returned to 0.000. Because the climb ends `GOAL_MARGIN = 0.3` m short of
the crest, that sample is *always* taken on the uniform 18° face, so the
stale value is always almost exactly the terrain grade — **a grade
estimator built on it would have matched ground truth on every metre of
ramp and then reported 18° on flat ground forever. It would have
passed.** Fixed at both ends: `/ramp/status` now publishes `pitch=--`
while idle, and `mission_hud` reads `ROBOT PITCH` from `/imu`.


---

## NOT COMPLETED (deliberately out of scope, as recorded at C2-M1.6)

- No C2-M2 implementation. No grade estimator, no friction estimator, no
  observer-driven controller.
- No PPO retraining, no reward change, no `lateral_hold` gain change, no
  Nav2 tuning, no geometry change. None was justified by a diagnosis.
- `coco2-m1-observability` not merged into the trunk *at the time this
  entry was written*. It is merged now — everything is on `main`.
- The three M7 Phase 4 decisions: untouched.

---


---

## KNOWN PROBLEMS — the development-era list, kept for its diagnoses

*Superseded as a summary by KNOWN LIMITATIONS above; kept because each entry carries the evidence and the reasoning that produced it.*

0. **NEW, C2-M5.0: the collision monitor's gating does not reach the
   wheels.** `/cmd_vel_nav` has **7 publishers and 2 subscribers** on the
   live graph — `nav2_bringup` remaps `controller_server` and
   `velocity_smoother` to it, and `nav.launch.py arbiter:=true` points
   `cmd_vel_relay`'s **output** at the same topic, so the collision
   monitor's output is fed back into the velocity smoother's input and
   the arbiter sees the raw and gated commands together. Measured over
   three runs: the wheels receive **10.15–10.77 Hz more than the
   collision monitor publishes**, exactly `controller_frequency: 10.0`.
   During an active SLOWDOWN, gated cap `max_vel_x × slowdown_ratio` =
   0.090 m/s, wheel commands reached **0.300 m/s** — 84.2% of
   `obstacle1`'s slowdown samples, 40.0% of `diverged1`'s. **A safety
   defect. NOT a localization problem. NOT fixed** — the wheel path is
   frozen (`CLAUDE.md` §4) and C2-M5.0's mandate was to characterize.
   The fix is a topic rename; it needs a decision and a measured run.
   **C2-M5.1 must not assume the collision monitor can stop the robot.**

1. **Nav home fails, by at least two distinct mechanisms, and it is not
   downstream of the climb or of vision.** **C2-M5.0 instrumented it and
   separated two classes** — see `RESULTS.md`, "C2-M5.0 localization
   health". Class A (the pose is wrong and the filter is sure) is
   detectable from onboard signals in **0.4 s**, but **not by
   covariance**. Class B (`healthy2`, uninjected: position error 0.300 m,
   inside the healthy band, yaw error to 1.31 rad, plan lengthening,
   `navigate_to_pose` ABORTED) is **not separable by any signal
   recorded**, and no threshold is proposed. The recorded tally over
   RETURN_HOME legs is now **six failed, five succeeded across all
   sessions** — still not a rate; the standing figure is M6's 19/20.
   The 2026-08-17 `PolygonStop` stall was **not reproduced** in five
   runs and stays open. The original entry follows. Four recorded legs: FAILED,
   SUCCEEDED, FAILED, SUCCEEDED. Run 1 was AMCL divergence (**≈3.2 m** in
   y at the leg start — the M6 run-15 family). The 2026-08-17 run had
   AMCL within **0.45 m**, a clean climb, confirmed vision and a
   successful pick, and *still* stalled **2.59 m short of home** behind
   repeated `collision_monitor: PolygonStop`, 11× `Failed to make
   progress` and two timed-out Spin recoveries, ending on the client's
   240 s timeout. **Confound not isolated:** that run logged `Control
   loop missed its desired rate of 10.0000 Hz. Current loop rate is
   4.8077 Hz` with Gazebo, RViz, move_group and the probe all running.
   **Four runs are not a success rate**; the standing figure is M6's
   19/20. **This belongs to C2-M5**, which already names M6 run 15 as its
   benchmark. **C2-M3.0 (2026-08-20) drove home successfully on the
   first attempt**, under light load with RViz off. Five runs are still
   not a success rate and the problem stays open.

2. **Why *that* climb drifted 0.51 m is still unknown.** `lateral_hold`
   was on and reached its clamp (peak 0.800 = `LATERAL_CLAMP`), so "not
   engaging" is **refuted** — but the clamp is **not** established as the
   binding constraint, because a good run also peaked at 0.800 and
   finished at cross-track +0.036 m.

3. **Which gate in `target_finder._locate` rejected blue is not
   determined** — the 0.15–2.00 m range gate or the `plausible_blob`
   width check. The status line records `found=0` and not the reason.

3b. **The scripted descent timed out in both C2-M1.6 traverse runs.**
   `--no-grasp`, fresh simulator each. Both navigated out, climbed
   cleanly (`outcome=goal`, cross-track −0.01 m, disp +0.03 m) and
   confirmed blue at 1.159 m; both then **timed out at 90.1 s** in step 5
   with the robot at world **(4.50, 0.24)**, the far edge of the
   platform. **No diagnosis was attempted** — C2-M1.6 was
   presentation-only and nothing it changed can reach the controller.
   **Confound not isolated:** run 1 had two RViz instances alive (a
   harness fault, since fixed), and problem 1 above already records a
   control loop degraded to 4.8 Hz against a 10 Hz target under
   Gazebo + RViz + move_group. **Two runs are not a rate**; the standing
   figure is M6's 19/20. Whoever next runs the mission for its own sake
   should check whether this reproduces under light load before treating
   it as a defect. **C2-M3.0 did exactly that (2026-08-20): descent
   `outcome=goal` in 16.5 s, RViz off, load average under 4.0.** That is
   consistent with 3b being load-induced and does not establish it —
   three runs, and the two that failed shared a confound this one
   removed.

4. **The user's `~/ros2_ws` may still have a stale `coco_sim` build.**
   Signature: 29 `coco_rl` tests failing with `FileNotFoundError` on
   `build/coco_sim/worlds/yard_params.yaml`. Fix, **measured**:
   ```bash
   cd ~/ros2_ws && colcon build --packages-select coco_sim
   ```
   **Confirmed still true 2026-08-19**: `~/ros2_ws/install/coco_sim`
   exists but has **no `worlds/yard_params.yaml`** at all. The feature
   work is unaffected because the branch builds its own overlay (below),
   but a fresh session running from `~/ros2_ws` will see the 29 failures.

4b. **The feature branch uses a worktree-local overlay, not
   `~/ros2_ws/install`.** This is how the branch's tests and nodes
   actually resolve, and it is not obvious from anywhere else:
   ```bash
   cd <worktree> && colcon build --symlink-install \
       --build-base .build_wt --install-base .install_wt \
       --packages-select coco_rl coco_sim gazebo_models
   . <worktree>/.install_wt/setup.bash
   ```
   Both `.build_wt/` and `.install_wt/` are gitignored. **It goes stale
   like any other build**: C2-M2.1 found the overlay predated C2-M2.0's
   `terrain_observer` entry point, so `ros2 run coco_rl terrain_observer`
   did not exist until it was rebuilt. Rebuild before trusting a live run.

5. **Run pytest from inside each package directory** — from the repo root
   the `coco_rl/` *directory* shadows the installed module — **and pass
   `--ignore=test_integration`**, or `gazebo_models` dies during
   collection on `test_sim_bringup.launch.py` and silently reports zero
   tests for the package. Also source the user-space MoveIt prefix
   (`setup_env.sh` does), or `coco_moveit_config` reports 5 passed and 7
   **skipped** rather than 12. Neither is a regression; both move the
   headline total.

6. `rviz_2d_overlay_plugins` is not installed, so
   `mission_hud._publish_overlay` has **still never executed**. It
   degrades cleanly to the String topic.

7. `docs/RSE_ASSIGNMENT_PLAN_V2.md` is **untracked and belongs to a
   different project** (an AMR fleet assignment). Not part of COCO. Not
   touched.

**Resolved this session, removed from this list:** the `-0.314 rad` pitch
(#4 previously), and `/approach/target`'s one-shot VOLATILE publication
(#5 previously — investigated, and it is correct as it stands).

---


---

## EXPERIMENTS RUN

### C2-M3.0 (2026-08-20)

Fresh simulator each, `ros_clean.sh` between, `gui:=false`, RViz off,
never `--fast`. One Gazebo at a time.

| # | What | Result |
|---|---|---|
| 1 | Full fetch through the executive | **ABORT / `NO_LOCALIZATION`.** Every Nav2 lifecycle node `unconfigured`, `/amcl_pose` **0 publishers** — the `autostart` leak |
| 2 | Repeat after the fix | **Contaminated and discarded.** An orphaned stack from a killed foreground launch: wheel-topic publisher count **2**, two `mission_executive` processes. It is why runs 3 and 4 gate on publisher count = 1 before starting |
| 3 | Repeat, clean stack | **ABORT / `ALIGN_HEADING`** at +0.28 rad, re-driven +0.26. `NAVIGATE_TO_RAMP`'s ground-truth region check **passed** |
| 4 | Repeat, heading gate off | **COMPLETE, `result=fetch`.** 15/15 transitions, 0 recoveries, 0 retries, 175.8 s, home to **7 mm**, arbiter publisher count 1 before and after |

### C2-M1.5 / C2-M1.6 (2026-08-17)

Fresh simulator each, `ros_clean.sh` between, `gui:=false`, never
`--fast`. One Gazebo at a time.

| # | What | Runs | Result |
|---|---|---|---|
| 1 | Full fetch, blue, with `pitch_probe` at 10 Hz | 1 | 1,900 samples. Pitch defect quantified: **0.314 rad** peak error off-segment, field changed 21× vs `/imu`'s 144. Mission **failed at nav home** with a clean climb and a successful pick |
| 2 | Traverse only (`--no-grasp`) after the fix | 1 | 2,200 samples. `pitch=--` off-segment throughout; on-segment tracks `/imu` to one sample. **TRAVERSE COMPLETE, home to 0.10 m** |
| 3 | RViz framing check, robot at spawn | 1 | First re-framing attempt was **worse**; caught by screenshot |
| 4 | RViz `Distance` sweep, one stack, viewer restarted per value | 3 viewers | **14 overflows, 18 fits with margin, 22 too far.** Shipped 18 |
| — | Archive re-read of the two 2026-08-16 runs in `~/.ros/log` | 0 new | Supplied the whole failed-fetch answer without re-running anything |

Also measured live: `ros2 topic info -v /approach/target` — publisher 1,
subscriptions 2, QoS compatible, VOLATILE both ends.

### C2-M1.6 (same day, later)

| # | What | Runs | Result |
|---|---|---|---|
| 5 | Offline map audit against `coco_world.world` | 0 sim | **Map GOOD.** 5 landmarks → one rigid offset, worst residual **25 mm = 0.50 cell**. Reproduce: `python3 docs/data/map_audit.py` |
| 6 | RViz framing sweep, map_server + rviz2 only, viewer restarted per value | 5 viewers | D12 bottom margin 24 px; **D13/pitch 1.45 → 949 × 652 px, margins 135/136/90/64, shipped**; D14 and D16 wasteful |
| 7 | Live traverse, `--no-grasp`, both configs on one sim | 1 | 7 screenshots. Nav OK, climb `outcome=goal`, blue CONFIRMED, **descent timed out at 90.1 s** (see KNOWN PROBLEMS 3b) |
| 8 | Second live traverse (first attempt, 2 viewers alive) | 1 | Same outcome; the two-viewer harness fault was found here and fixed |

**Three harness traps worth not re-paying.** `x11grab` captures a screen
*region*, so another session's window landed in one shot — use
`xwd -id <win>`, which asks the X server for the window's own pixels.
Parking the pointer in a screen corner hits a desktop hot corner and
silently orbits the camera away from the config under test; the tell is
the status bar reading "Left-Click: Rotate" instead of "RViz is ready".
And kill the previous viewer, or `xdotool search --name RViz` returns the
wrong window. RViz does **not** write the `-d` config back on exit —
checked.

---


---

## FILES ADDED BY COCO 2.0 — what each one is for

Nothing is mid-edit. Changed by C2-M1.5:

Changed by C2-M1.5:

- `coco_rl/coco_rl/ramp_driver.py` — `pitch=--` while idle, `live_pitch()`
- `coco_mission/scripts/mission_hud.py` — `ROBOT PITCH` from `/imu`
- `gazebo_models/scripts/pitch_probe.py` — **new**, the instrument
- `gazebo_models/CMakeLists.txt`, `gazebo_models/scripts/ros_clean.sh`
- `coco_mission/package.xml`, and the two test files

Changed by C2-M1.6 (`a7bfc23`):

- `gazebo_models/rviz/mission.rviz` — rewritten as the clean view
- `gazebo_models/rviz/mission_debug.rviz` — **new**; the C2-M1.5 view,
  byte-identical below its comment header
- `coco_mission/launch/mission.launch.py` — `rviz_config:` argument
- `gazebo_models/test/test_rviz_configs.py` — **new**, 21 tests
- `docs/data/map_audit.py` — **new**, the map instrument. Read-only, no
  ROS, deliberately not installed by any `CMakeLists.txt`
- `docs/RESULTS.md`, `docs/RUNNING.md`, `docs/SESSION_LOG.md`,
  `docs/data/README.md`, and three figures under `docs/images/`

Changed by C2-M2.0 (`1aa6670`):

- `coco_rl/coco_rl/terrain_observer.py` — **new**, the estimator. Pure
  Python, no `rclpy`
- `coco_rl/coco_rl/sensor_model.py` — **new**, the information boundary as
  a type: `DeployableSignals` vs `GroundTruth`, sharing no field name
- `coco_rl/coco_rl/terrain_observer_node.py` — **new**, the ROS face
- `coco_rl/coco_rl/terrain_benchmark.py` — **new**, the frozen benchmark
- `coco_rl/coco_rl/baselines.py` — `B3` and `schedule_gains()`
- `coco_rl/coco_rl/yard_env.py` — the surface-relative tip terminator
- `docs/data/c2m2_sanity.py` — **new**, the five implementation checks

Changed by C2-M2.1 (`7796d04`):

- `coco_rl/coco_rl/terrain_observer_node.py` — **the three live defects**:
  per-topic QoS from `is_best_effort(topic)`, estimation moved into the
  IMU callback at 50 Hz, `declare_flat` wired through
- `coco_rl/coco_rl/baseline_eval.py` — metric terminology, and `tau` /
  `tan_grade_true` recorded
- `coco_rl/coco_rl/terrain_benchmark.py` — reporting terminology only
- `coco_rl/test/test_terrain_observer_node.py` — **new**, 12 tests that
  construct the real node
- `docs/data/c2m2_benchmark.json` — **the 1,440 episodes, raw**
- `docs/data/c2m2_analysis.py`, `c2m2_plots.py`, `c2m2_live_gate.py` —
  **new** instruments; the two live-gate CSVs beside them
- `docs/RESULTS.md`, `docs/SESSION_LOG.md`, `docs/DESIGN_DECISIONS.md`,
  `docs/data/README.md`, four figures under `docs/images/`

Changed by C2-M3.0 (`1c36499`, `fb2ed09`):

- `coco_mission/scripts/mission_states.py` — **new**, the state machine.
  Pure Python, no `rclpy`. Read this one first
- `coco_mission/scripts/mission_executive.py` — **new**, the ROS adapter
- `coco_mission/scripts/mission_hud.py` — renders the new
  `/mission/state` line; the `RECOVERY` row has a source now
- `coco_mission/launch/mission.launch.py` — `executive:` and
  `mission_autostart:` arguments
- `coco_mission/CMakeLists.txt`, `coco_mission/package.xml`
- `coco_mission/test/test_mission_states.py` — **new**, 62 tests
- `coco_mission/test/test_mission_executive.py` — **new**, 35 tests that
  construct the real node
- `gazebo_models/launch/nav.launch.py` — pins `autostart: 'true'` on the
  nav2_bringup include. **The one change outside `coco_mission`, and it
  is an interface bug fix** — see RESULTS "the `autostart` leak"
- `gazebo_models/scripts/ros_clean.sh` — `mission_executiv[e]`
- `docs/ARCHITECTURE.md`, `docs/DESIGN_DECISIONS.md`, `docs/RESULTS.md`,
  `docs/RUNNING.md`, `docs/SESSION_LOG.md`

**`gazebo_models/scripts/traverse_demo.py` is byte-identical.** It is the
harness the M4/M5/M6 numbers were measured with. `executive:=false`
selects it, and running both at once is an operator error nothing
enforces against — two publishers on `/mission/mode`, which the arbiter
latches.

**C2-M3.1 touched none of these.** It ran the failure paths on the robot
and the machine held, so `mission_states.py` and `mission_executive.py`
are still byte-identical to C2-M3.0 and `RECOVERY` still stops and
nothing more — which was measured to be sufficient in every branch that
ran.

Changed by C2-M4.0 (`16e952f`):

- `coco_perception/coco_perception/target_pose.py` — **new**, the
  geometry and the policy. Pure Python, no `rclpy`, no `tf2`, no message
  types. **Read this one first**, and its module docstring before its
  code: it defines what the published point *means*
- `coco_perception/coco_perception/target_pose_node.py` — **new**, the
  ROS face. Holds no geometry
- `coco_perception/test/test_target_pose.py` — **new**, 67 tests
- `docs/data/c2m4_localisation.py` — **new**, the instrument **and the
  C2-M4.1 benchmark runner**. Deliberately not installed by any
  `CMakeLists.txt`
- `docs/data/c2m4_sanity_sweep.csv` — the 20 placements, raw
- `docs/data/c2m4_minrange_probe.csv` — the 8-placement control
- `coco_perception/setup.py`, `package.xml` — the entry point, and
  `vision_msgs` / `tf2_ros` / `coco_moveit_config`
- `gazebo_models/scripts/ros_clean.sh` — `target_pose_nod[e]`
- `CLAUDE.md` — the `sys.path[0]` shadowing trap
- `docs/ARCHITECTURE.md`, `docs/DESIGN_DECISIONS.md`, `docs/RESULTS.md`,
  `docs/SESSION_LOG.md`, `docs/data/README.md`

**`coco_perception/coco_perception/target_finder.py` is byte-identical**,
and so is everything in `coco_moveit_config`. C2-M4.0 added a path
beside the measured one rather than changing it.

**What C2-M4.1 actually did to them:**

- `docs/data/c2m4_localisation.py` — **run, not written.** `--benchmark`
  produced all 60 placements unmodified. The experiment was not
  redesigned
- `coco_perception/coco_perception/target_pose_node.py` — **the only
  source file changed.** Gained `point_topic`, empty by default;
  `min_range` was decided and **left alone**
- `coco_moveit_config/scripts/grasp_server.py` — **not touched.** It
  never held a hard-coded grasp coordinate: it takes a fresh
  `/approach/target` fix and falls back to `approach_stop_x(colour)`
  = 0.1537, a value derived from the window geometry. C2-M4.1 changed
  **where the fix comes from**, upstream, and left the grasp path that
  M6 measured exactly as it was
- `custom_teleop/custom_teleop/approach_server.py` — **not touched**,
  and it turned out to be the component that makes the lateral result
  come out well: its `align` phase nulls the bearing before the fix is
  taken
- `custom_teleop/custom_teleop/cmd_vel_arbiter.py` — **read, not
  changed.** Sole publisher to the controller topic; verified 1 before
  every C2-M4.1 grasp run

---


---

## UNRESOLVED QUESTIONS

**Opened by C2-M5.1, and it is the honest end of the milestone:**

0a. **How should a confidently-wrong AMCL be recovered on this map?**
    C2-M5.1 detects the divergence, stops safely, and re-seeds the
    filter, and the mission still cannot get home. Two measured causes,
    neither in the executive: `amcl.recovery_alpha_fast` and
    `recovery_alpha_slow` are **0.0**, so AMCL has no random-particle
    injection and cannot leave a mode it is confident in; and
    `/reinitialize_global_localization` converged to world
    **(2.60, −0.64)** — inside the wedge footprint — because the map's
    2D slice is highly self-similar and a 360° scan from a standing
    robot does not disambiguate it. The planner then reported "Start
    occupied". **Turning the `recovery_alpha_*` pair on is the obvious
    next experiment and is NOT obviously right**: the standing 19/20 was
    measured with them at 0, and changing an AMCL parameter to make one
    mission pass is what NEXT EXACT ACTION has forbidden since C2-M5.0.
    It needs measuring, on both a healthy matrix and the injection.

0b. **Why does detection latency vary by 25×?** The same class-A
    injection measured **3.33 s, 4.52 s and 82.9 s** across three runs.
    Three is not a sample, and until it is larger the latency figure in
    RESULTS.md is a range and not a number.

**Opened by C2-M5.0, and it is a decision somebody has to take:**

00. **Should `cmd_vel_relay`'s arbiter output move off `/cmd_vel_nav`?**
    It is the same topic `nav2_bringup` remaps `controller_server` and
    `velocity_smoother` onto, so the relay feeds the collision monitor's
    output back into the smoother's input and the arbiter receives the
    raw and gated commands together. Measured: the wheels see
    **10.15–10.77 Hz more than the collision monitor publishes**
    (= `controller_frequency: 10.0`), and during an active SLOWDOWN with
    a 0.090 m/s gated cap the wheels were commanded **0.300 m/s** on
    84.2% of `obstacle1`'s slowdown samples.

    **The fix is a topic rename** — give the relay its own output, e.g.
    `/cmd_vel_gated`, and point the arbiter's `nav_topic` at it. It was
    **deliberately not done in C2-M5.0**: the wheel path is frozen
    (`CLAUDE.md` §4), the change alters what the robot is commanded on
    every leg, and the standing **19/20** was measured with the loop in
    place. Whoever changes it owes a measured run and a statement about
    comparability to that baseline. Until then, **C2-M5.1 must not
    assume the collision monitor can stop the robot.**

01. **What is the healthy spread of the scan-vs-map signal?** C2-M5.0
    has two legs that finished and cannot place a threshold in the
    0.054 m gap between the worst of them and the best failure. The
    missing evidence is **more healthy legs**, not more failures. Until
    it exists, `localization_health.Thresholds` stays without defaults.

02. **What caused `healthy2`?** An uninjected RETURN_HOME failure with
    position error inside the healthy band (0.300 m median) and yaw
    error reaching 1.31 rad — measured **not** to be AMCL staleness
    (an upper-bound lag estimate explains 0 of the 55 samples above
    0.5 rad). The plan lengthened 9.73 → 13.93 m and
    `navigate_to_pose` returned ABORTED, with the two retries aborting
    in 0.1 s each. **One run. Not diagnosed.**

**Opened by C2-M2.1, and the most important thing on this list:**

0. **Was `ascent` the right decision task?** The benchmark says it does
   not discriminate where the privileged advantage is largest: on Route A
   every controller including open-loop reaches the deck 92–99 %, while
   B2 **completes** 97.5 % against B3's 0.0 %. C2-M2.0 chose ascent
   because Phase 3 made completion look like a score on deck geometry —
   but **B2 crosses that bridge 117 times in 120 on terrain-aware
   throttle alone**, and a pure geometry problem does not yield to
   terrain information. **Deliberately NOT resolved in C2-M2.1**:
   changing the task after seeing the result is the failure the freeze
   existed to prevent. It belongs to whoever sets the next rule. Evidence
   in `RESULTS.md` "C2-M2.1" and `DESIGN_DECISIONS.md` "The decision rule
   was not moved after the result arrived".

**Two of M7 Phase 4's three gates remain** (the third is now closed):

1. **Deck convergence geometry** — 1.95 m lateral shift in 1.80 m of
   travel before a 0.65 m bridge, against a 0.40 m turn radius. **Now
   partly contradicted:** B1 and B3 still fall off 93 times in 120, but
   **B2 falls off zero times**. The geometry is not the whole story.
2. **Route B viability** — **39.3% of episodes have mu < tan(grade) and
   are physically unclimbable**. Four options costed in `RESULTS.md`.
   **None chosen.** C2-M2.1 flagged rather than dropped them: the
   `ascent|climbable` column reads 51–54 % against a raw 32–34 %.
3. ~~Route C's tip terminator~~ — **CLOSED by C2-M2.0.** Made
   surface-relative, 0.6 rad kept exactly, with a 54.5° absolute
   backstop; the other three `TIP_LIMIT` homes are untouched at 0.6 rad
   absolute and a test asserts the split. **Do not "unify" them.**
   C2-M2.1 measured the consequence: the tip population did **not**
   shrink (B1 106, B3 116 vs Phase 3's 101). What changed is that it now
   fires at a genuine rear-over instead of 34° short of one.

**Opened by C2-M4.0, and it is one parameter:**

- **Should `min_range` drop from 0.15 to 0.11?** The gate rejects an
  extended target's near face, which sits a full radius in front of its
  axis, so below a ~0.30 m stand-off the reported range is long by an
  amount proportional to the target's radius: **+4.1 to +8.3 mm**
  measured, collapsing to **−1.0 to −1.4 mm** at 0.11. The argument for
  leaving it: it matches `target_finder`, and the approach's last leg is
  blind below `min_range` by construction, so perception's operating
  envelope starts near 0.30 m regardless. The argument for changing it:
  it is a bias with a known mechanism sitting inside a pipeline whose
  whole job is a millimetre-scale number. **Deliberately not decided in
  C2-M4.0** — the control run is in `RESULTS.md` and the decision
  belongs with whoever runs the benchmark. A radius-aware gate is a
  third option and nobody has costed it.
- **Is the vertical estimate worth reporting at all?** `point.z` is
  framing-dependent by construction and no consumer uses it —
  `grasp_point.z` comes from `TARGET_GRASP_Z`. It is currently published
  so the framing effect stays visible. If C2-M4.1 finds nothing reads
  it, that is a case for dropping it rather than for defending it.

**Opened by C2-M3.0:**

- **Can `ALIGN_FOR_CLIMB`'s heading gate be calibrated at all?** It needs
  either a tighter goal checker for that one leg (nav2_params already
  defines a `precise_goal_checker` at 0.05 m) or an aligner **behind the
  arbiter**, plus a threshold measured against climbs that actually
  failed rather than against Nav2's tolerance. Nothing measures the
  relationship between start yaw and climb drift today.
- **Every failure path is unverified live.** One clean run entered no
  recovery at all. Until one does, the recovery architecture is a
  well-tested description of behaviour nobody has watched.
- Whether two publishers on `/mission/mode` (executive plus
  `traverse_demo.py`) should be prevented at runtime rather than by
  documentation. The arbiter latches the last value it saw.

**Also open, from C2-M2.1:**

- Whether Route C's tips are avoidable by control at all.
- Whether B3's poor Route C behaviour (58.3 % ascent vs B1's 84.2 %)
  improves with a better grade channel on rubble. The correlation with
  grade MAE 2.681° / 10.10 s convergence / 13 % engagement is
  **suggestive and not a demonstrated cause.**
- The simulated IMU is still **noiseless**
  (`imu_noise_sigma: not_yet_measured`). Nothing in the observer
  integrates, and this still bounds what any of it claims about a real
  robot.

---


---

## FROZEN — DO NOT CHANGE WITHOUT AN EXPLICIT INSTRUCTION

From `CLAUDE.md` §4, plus additions:

- The action space `(linear, angular)`, normalised `[-1, 1]`
- `cmd_vel_arbiter`, and its position as the **sole** publisher to the controller
- Camera RPY `(0,0,0)` — two tests assert this; a −0.6 rad pitch was
  proposed and is wrong in **both sign and magnitude**
- `GRASP_SELF_COLLISION_X = 0.150` — measured by probing
  `/check_state_validity` at 1 mm steps
- The target bay geometry. **`coco_perception/target_finder.py` is
  frozen** — `/perception/target` feeds `approach_server`'s servo
  mode and is the path M6's 20/20 approach was measured through, so
  C2-M4.0 added `target_pose.py` / `target_pose_node.py` beside it
  rather than editing it. A test
  (`test_optical_to_base_matches_the_urdf_chain`) pins its hard-coded
  extrinsics against the xacro so the two frame paths cannot drift
  while both exist. New files in the package are fine; changing that
  one needs an explicit instruction
- The v1 wedge world, frozen as `world_v1`
- `GOAL_SUMMIT` / `GOAL_MARGIN`
- **`gazebo_models/rviz/coco_robot.rviz`** — the TF-only view. Its
  `base_footprint` fixed frame is correct *there*. A test asserts it.
- **`gazebo_models/rviz/mission_debug.rviz` is the C2-M1.5 view,
  preserved byte-identically below its comment header.** Its oblique
  Distance-18 camera is deliberate: TF triads, the arm's pose and the
  costmaps' z-stacking are readable at 35° and degenerate near straight
  down. If it needs re-framing, re-measure it — do not copy the clean
  view's numbers, which were measured for a different camera.
- **The occupancy map `maps/coco_world.pgm`.** C2-M1.6 classified it
  **GOOD** by registration against the world (worst residual 25 mm, half
  a cell). There is no SLAM problem to fix, and a rebuild would have to
  beat that.
- The training environment (`coco_rl/coco_rl/mujoco_env.py` and
  everything it touches) **must never import `rclpy`**.
- **`/approach/target` stays VOLATILE.** Investigated in C2-M1.5 and the
  current semantics are correct; see `docs/DESIGN_DECISIONS.md`.
- **The C2-M2 benchmark configuration.** Controllers B0/B1/B2/B3, routes
  A/B/C, seeds 0–119, `TUNED_SCHEDULE`, decision task `ascent`, margin
  **10 percentage points**. It was frozen before any result existed and
  the result is now recorded against it. **Re-scoring the same run on a
  different task would be choosing the metric that gives the answer.** If
  a future milestone wants a different task, it states so in advance and
  re-runs — see UNRESOLVED QUESTIONS 0.
- **The four `TIP_LIMIT` homes and their deliberate split.**
  `yard_env.py` is **surface-relative**; `reward.py`, `mujoco_env.py` and
  `ramp_driver.py` are **0.6 rad absolute** and carry the v1 curriculum,
  the shipped policy and the mission's runtime check. A test asserts it.
  **Do not "unify" them.**
- **Friction is not identifiable on this robot** from an IMU and wheel
  encoders. Measured in MuJoCo (τ spans 0.0003 over a μ span of 0.35;
  τ − tan(grade) ≈ 0 over 1,440 episodes) and confirmed live in Gazebo
  (0.3248 vs tan 18° = 0.3249). **Nothing may report a friction
  estimate**; τ is a traction-demand ratio and `mu_lower` a proven bound.

- **`gazebo_models/scripts/traverse_demo.py`.** Byte-identical through
  C2-M3.0 and it stays that way: it is the harness the M4/M5/M6 numbers
  were measured with. The executive supersedes it; it does not replace
  it in the record.
- **`ALIGN_FOR_CLIMB`'s heading gate is OFF**, and the threshold is not
  to be re-asserted without a measurement. Nav2 judges yaw against the
  AMCL pose it is steering by; this check reads ground truth; the two
  differ by the localisation error, and the leg arrives at +0.26 to
  +0.28 rad every time. Turning it on aborts missions that complete.
- **`nav.launch.py` pins `autostart: 'true'`** on its nav2_bringup
  include, and no launch file above it may declare a bare `autostart`.
  A launch configuration is inherited by every include and shadows the
  included file's own default; the symptom is every Nav2 node
  `unconfigured` with nothing in any log naming the cause.

**Baseline:** the immutable v1 result is M6's **19/20** fetch matrix on
the frozen `world_v1`. Any experiment changing the world, reward, robot
model, action space, controller, map or perception assumptions **must
state explicitly whether it remains comparable to that baseline.**

---


---

## FUTURE IDEAS (recorded, NOT to be started)

- Install `rviz_2d_overlay_plugins` and record the demo video (C2-M9).
- The web panel (`coco_web`) could render `/mission/hud` directly.
- Give `/approach/target` a home in an **action result** when C2-M3
  replaces the `Trigger` services — for the interface mismatch, not for
  durability. **Making it TRANSIENT_LOCAL was removed from this list**:
  C2-M1.5 established it would be a defect, because the payload is in
  `base_footprint` and latching a robot-relative point hands a late
  subscriber a coordinate in a frame that has since moved.


---
