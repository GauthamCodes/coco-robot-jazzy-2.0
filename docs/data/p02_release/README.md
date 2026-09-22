# P0.2 release pass — live evidence, 2026-09-22

Branch `p02-release-candidate`. Overlay `~/coco_ws_build`, rebuilt from
this branch by `scripts/build_overlay.sh` (9/9 packages). Every run: a
**fresh simulator**, never `--fast`, depth fusion off, Nav2 untouched,
driven entirely by `scripts/browser_check/live_run.sh` — headless Firefox
156 over WebDriver BiDi, real pointer clicks and key presses on the
shipped page, a ROS-side wheel recorder, then a hostile-socket probe and
teardown of the run's own process groups only (`live_run.sh` refuses to
start if any Gazebo is already running; none was).

| dir | colour | Gazebo | outcome |
|---|---|---|---|
| `run1_red/` | red | headless | **COMPLETE, `result=fetch`** |
| `run2_blue/` | blue | headless | **COMPLETE, `result=fetch`** |
| `run3_yellow/` | yellow | headless | **COMPLETE, `result=fetch`** |
| `run4_gui_green/` | green | **`gui:=true`** | **COMPLETE, `result=fetch`** — see *GUI vs headless* |
| `run5_green_control/` | green | headless, **ROS domain 61**, session-sweep teardown | **COMPLETE, `result=fetch`** — the harness fixes, validated; the same-colour control for run 4 |

Run 1 used the pre-change `platform.launch.py`, whose `web_video_server`
listened on `0.0.0.0:8081`; runs 2 and 3 used the loopback bind
(`video_listen.txt`: `127.0.0.1:8081` only). Nothing else differed.

## What was measured

| | run 1 red | run 2 blue | run 3 yellow |
|---|---|---|---|
| controllers active after sim launch | 10 s | 7 s | 9 s |
| `/healthz` 200 after mission launch | 8 s | 8 s | 9 s |
| STOP topmost at its centre: at page open / ready / mid-mission | yes / yes / yes | yes / yes / yes | yes / yes / yes |
| annotated MJPEG via `/video/annotated` | loaded 320×240 | loaded 320×240 | loaded 320×240 |
| first camera frame after the subscribe click | 0.57 s | 0.66 s | 0.36 s |
| first depth frame after the subscribe click | 0.39 s | 0.26 s | 0.37 s |
| W held: key → first moving wheel command | 37.4 ms | 41.8 ms | 89.3 ms |
| **joystick** dragged forward: → first moving wheel command | 147.3 ms | 103.6 ms | 161.2 ms |
| joystick max wheel command, forward / back | +0.346 / −0.346 m/s | +0.346 / −0.346 | +0.346 / −0.346 |
| moving commands > 600 ms after either joystick release | 0 / 0 | 0 / 0 | 0 / 0 |
| STOP clicked with W still held: first zero | 12.6 ms | 79.0 ms (no moving command after the click at all) | 2.7 ms |
| browser SIGKILLed mid-drive: first zero | 91.5 ms | 84.3 ms | 76.0 ms |
| moving commands > 600 ms after STOP or kill | 0 | 0 | 0 |
| publishers on `/diff_drive_controller/cmd_vel` | 1 (`cmd_vel_arbiter`) | 1 | 1 |
| publishers on `/cmd_vel_teleop` | 1 (`coco_web_platform`) | 1 | 1 |
| hostile socket frames refused | 8 / 8 | 8 / 8 | 8 / 8 |
| every executive state rendered on the page | 16 / 16 | 16 / 16 | 16 / 16 |
| state change → page DOM (15 transitions, in-page observer) | 22.7–68.4 ms, median 27.3 | 76.7–101.7, median 84.3 | 46.5–92.5, median 54.5 |
| target lifted (grasp_server, ground truth) | 35.9 mm | 36.0 mm | 36.0 mm |
| place | `placed` | `placed` | `placed` |
| retries (`attempts=`) | `{}` | `{}` | `{}` |
| mission wall time, IDLE → COMPLETE | 318.0 s | 348.6 s | 325.4 s |
| real-time factor (Σ state `elapsed` / Σ wall, 8 states ≥ 2 s) | 0.455 | 0.456 | 0.474 |
| final pose drawn on the page (map frame; home is 0, 0) | (0.00, 0.07) | (−0.06, 0.14) | (−0.05, −0.03) |
| session after COMPLETE | READY / HEALTHY | READY / HEALTHY | READY / HEALTHY |
| dropped frames, any stream | 0 | 0 | 0 |
| platform CPU, one core, mean (min–max) | 67.3 % (56.7–75.9) | 65.7 % (53.7–72.5) | 69.6 % (55.7–77.0) |
| mission-state latency, `/mission/state` callback → frame out | 5.8–36.6 ms | 59.2–88.0 ms | 37.0–70.5 ms |
| JS errors | 0 | 0 | 0 |
| orphans after teardown | 0 | 0 | 0 |

`analysis.json` in each directory is `analyse_live.py`'s full output; the
table is read from it and from `mission_excerpt.txt` (the grasp server's
and executive's own log lines).

## GUI vs headless (run 4, `COCO_LIVE_GUI=true`, green)

The previous pass (`docs/data/clean_runtime/`) had GUI 0 / 2 home —
`RETURN_FAILED`, `planner_server` "Start occupied" at the foot of the
ramp — against headless 1 / 1. On this release candidate the same
scenario with the GUI **completed**: lifted 35.7 mm, `placed`,
`attempts={}`, **0** "Start occupied", all 16 states on the page, the 15
transitions 9.6–35.3 ms to the DOM (median 17.8). Every safety and
rendering row of the table above held (STOP-with-W first zero 18.9 ms,
SIGKILL 133.4 ms, 0 moving after either; 1 wheel publisher; 8/8 refused;
0 drops; 0 JS errors). **One GUI run is not a rate**, and GUI 1/3 overall
across both passes is not a diagnosis.

Compared on the three things the brief named:

- **Simulator timing: no divergence.** RTF 0.457 against 0.455–0.474
  headless; RETURN_HOME 121.8 s wall against 84.6–127.6 s headless.
- **Mission state: no divergence.** The same 16 states in the same order,
  no retries, as every headless run.
- **Command path: one divergence, and it is the first concrete one.**
  94 ms after the first joystick release, `/mission/mode` switched
  `idle → nav → idle → nav` within 110 ms and `bt_navigator` began
  navigating to **(2.50, 2.00)**; the arbiter forwarded two Nav2 commands
  (≤ 0.012 m/s, **0.5 rad/s**) at +608 and +658 ms until the mode fell back
  to idle at +676 ms (`external_goal_*.txt`). **Nothing in the run sent
  it**: the browser sends `set_mode auto` only from the Auto button (never
  clicked) and every `/cmd_vel_teleop` message after the release was zero;
  the platform had exactly one client (`clients_during_run.txt`); no
  process either launch started publishes `/goal_pose`; the Gazebo GUI
  loaded only stock plugins; `mission_hud` publishes `/mission/goal`.
  **Unattributed.** The run was on the shared default ROS domain 0, so
  anything else using domain 0 on this machine or LAN at that moment could
  have sent it; domain 0 was empty when checked afterwards.
  `live_run.sh` now defaults to a dedicated domain (61).
- **Teardown: GUI mode can orphan the simulator.** `gz sim server` and
  `gz sim gui` run in process groups of their own, without the world path
  on their command lines (measured with a standalone probe). After run 4,
  `gz sim server` outlived `live_run.sh`'s process-group teardown and was
  stopped by PID (`orphan_note.txt`). `live_run.sh` now also sweeps each
  session it created. `ros_clean.sh` cannot recognise such an orphan (its
  pattern needs the world path) — recorded, not changed.

## Run 5 — the harness fixes, run once

Headless green on `live_run.sh` as changed after run 4: dedicated ROS
domain 61 (a domain-61 `ros2-daemon` was spawned by the run's own
`ros2 topic info` calls) and a session sweep in the teardown. **COMPLETE,
`result=fetch`**: lifted 35.6 mm, `placed`, `attempts={}`, all 16 states
on the page, 15 transitions 34.0–61.0 ms to the DOM (median 48.5), RTF
0.506, joystick forward/back with 0 moving commands after either
release, STOP-with-W first zero 39.3 ms, SIGKILL 77.7 ms, 0 moving after
either, 1 wheel publisher, 8/8 refused, `/video/annotated` loaded,
`127.0.0.1:8081`, 0 drops, 0 JS errors, **0 orphans**, and no foreign
goal (2 `Begin navigating`, both the mission's own).

**Totals, this pass: 5 / 5 browser-driven fetches COMPLETE** — 4 headless
(red, blue, yellow, green), 1 GUI (green). Five runs, four colours: not a
rate.

## Notes on reading it

- **The page lag excludes `IDLE`.** `analyse_live.py`'s own
  `page_lag_ms_min_max` includes it and reads 605–848 ms at the top; that
  `IDLE` is the state already current before Start, and its "lag" is when
  the page's MutationObserver was installed, not a transition.
- **Joystick release windows end at the next action.** The first analysis
  of run 1 reported 7 moving commands "after release"; they were the
  second drag, which starts 1.5 s later. Checked against the recorder: 0.
- **`peak_buffer_bytes` read 0 in all three runs, and this time it
  measured something.** The probe that produced P0.2's "0 B" returned 0
  on every call; it now reads tornado's real buffer, and a test proves it
  sees a peer that stopped reading. On loopback, with one fast browser,
  the buffer was empty at every 5 s sample.
- **Three runs is not a rate.** They are three colours, one each.

## What is NOT established here

- Docker. Not installed on this machine; nothing here ran in a container.
- A touch-screen joystick: the drag used a mouse pointer. nipplejs
  handles both, but only the mouse path was exercised.
- A browser other than Firefox.
