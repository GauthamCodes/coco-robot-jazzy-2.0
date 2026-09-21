# browser_check — drive the shipped page in a real browser

The COCO page is verified here by a real browser engine — **headless
Firefox over WebDriver BiDi**, with tornado as the client — not by a
WebSocket client standing in for one. No Selenium, Playwright, driver
binary or browser extension. Every interaction is a real pointer click
or key press on the shipped page.

Why it exists: P0.2's first pass could not drive a browser and shipped on
static asset tests. The first real render found the "COCO is starting"
curtain drawn permanently over the page, **including over STOP**.

Requirements: Firefox ≥ 129 on `PATH` (the Ubuntu snap works; the
profile is kept under `$HOME` because the snap cannot read `/tmp` or dot
directories), a built overlay, and — for the live run — a machine with
**no other Gazebo running**. `live_run.sh` refuses to start otherwise and
never kills a simulator it did not start.

## Without a simulator

```bash
# the REAL server code, fed by a fake node with moving synthetic data
scripts/browser_check/envrun.sh <repo> <overlay> \
    python3 scripts/browser_check/fakestack.py <repo> 8090 &
python3 scripts/browser_check/render.py http://127.0.0.1:8090/ out/ 1400 1000
python3 scripts/browser_check/render.py http://127.0.0.1:8090/ out/phone 390 844
# connection lifecycle: sim not up, server gone, restart, FROZEN, thawed
python3 scripts/browser_check/lifecycle.py <repo> <overlay> out/life
```

`render.py` prints a JSON report per step: every readout on the page,
whether STOP is the element under its own centre (`stopHit`), and any JS
error or warning.

## With a simulator

```bash
scripts/browser_check/live_run.sh <repo> <overlay> out/live green
python3 scripts/browser_check/analyse_live.py out/live
```

`live_run.sh` starts a fresh simulator and `mission.launch.py
platform:=true`, a ROS recorder (`wheel_recorder.py`) and a metrics
sampler, then `live.py` drives the page: telemetry and LiDAR, camera and
depth after their subscribe clicks, W/S driving, STOP clicked while W is
still held, SIGKILL of the browser mid-drive, then a new browser that
picks a colour and presses Start and follows the mission to the end.
Afterwards `safety_probe.py` tries eight hostile frames on the socket.
Teardown kills only the process groups it started.

`analyse_live.py` joins the browser's wall-clock action log with what
the recorder saw reach `/diff_drive_controller/cmd_vel`: key-to-wheel
latency, time to zero after STOP and after the kill, moving commands
after either (must be 0), every executive state vs every state the page
rendered (an in-page MutationObserver, so none is missed), the wheel
topic's publishers, CPU, rates and drops.

None of this is in the colcon suite — Firefox is not a package
dependency. The invariants it found are pinned by static tests that are.
