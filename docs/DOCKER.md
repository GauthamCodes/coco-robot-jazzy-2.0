# DOCKER — the containerised COCO runtime

```bash
git clone <this repo> && cd coco-robot-ros2
docker compose build                    # the image (cold: ~13-45 min, network-bound)
docker compose up                       # the appliance -> http://localhost:8080
docker compose run --rm coco test       # every package's test suite, in the image
docker compose run --rm coco info       # what the image was built from
```

For an image whose content is a function of a commit and nothing else,
and for the evidence to go with it:

```bash
scripts/container/build.sh --ref HEAD                    # clean room: git archive of the commit
scripts/container/validate.sh --image coco-platform:<sha12> --test-reps 3 --sim --web --nav
```

`scripts/run_platform.sh` (build if needed, then `up`) and
`scripts/run_platform.sh --down` still work unchanged.

---

## Status (2026-09-25): built, tested and run — measured

Everything below was produced on the development machine (Ubuntu 24.04,
12 CPUs, 15 GiB RAM, RTX 4050 **not** used: no NVIDIA container runtime is
installed) with Docker Engine 29.8.1, Compose v5.5.1, Buildx v0.37.1, the
containerd image store. Machine-readable evidence, and the index that
regenerates it from the raw runs: `docs/data/container_validation/`.

| Claim | Measured |
|---|---|
| The ORIGINAL (P0.1/P0.2) Dockerfile builds | yes, first attempt: 2711.2 s from an empty Docker (network-bound; the link measured 1.7 MB/s) |
| ...and its image is reproducible | **no** — torch resolved 2.14.0+cpu, not the 2.12.1 the shipped policy was saved with; mujoco absent, so `coco_rl` collected **0 of 218** tests and `coco_sim` stopped at a collection error; `rosdep check` unsatisfied (exec: depth-image-proc, image-proc, xterm; test: nodejs) |
| ...and the appliance boots | yes, 1 boot: healthy in 33.2 s, `/healthz` HEALTHY, one `cmd_vel` publisher |
| This image builds clean-room from a commit | yes (`git archive`, no working tree, no `.git`) |
| The environment is the image's own | yes: decoy host `ROS_DOMAIN_ID`/`AMENT_PREFIX_PATH`/`ROS_DISTRO`/`PYTHONPATH`/`GZ_*` never appear inside; package path = `/opt/ros/jazzy` + the 9 COCO prefixes |
| Every declared dependency is installed | yes: `rosdep check` (exec + test) satisfied; pip layer == `docker/pip-constraints.txt`, enforced at build |
| The full test suite passes in the container | **1740/1740, 0 failed, 0 skipped**, x3 serial (71.6/70.9/71.3 s) and x3 at `--jobs 9` (21.5/21.4/21.3 s), as uid 1000, `--network none`, no capabilities |
| ...and it is the SAME suite as the host | host 1740/1740 x6; the same 1740 test ids, 0 outcome mismatches (repo root normalised) |
| Gazebo + ROS 2 + COCO run in the container | yes, headless, software rendering, as uid 1000 with no capabilities: 10 compose boots healthy in 26.4-33.2 s (1 more came up UNHEALTHY — the activation race, fixed); RTF 0.31-0.43; 6.6-8.1 cores, 1.35-1.59 GiB |
| The browser protocol works from the host | coco.v1 handshake 7-20 ms, ~10 Hz telemetry, 0 binary frames to a text client, 0 topic-name leaks against 163 live topic names (3 runs) |
| Nav2 in the container | 3/3 goals SUCCEEDED end to end with `executive:=false` (0.70-0.71 m, 151-160 commands at the wheels); with the executive on, the arbiter keeps an outside Nav2 goal off the wheels by design (0 wheel commands, Nav2 recoveries 8-10) |
| The four-colour fetch in the container | a regression matrix, not a rate: one valid run per colour — **green COMPLETE**, red/blue/yellow **ABORT (GRASP_FAILED)**, all after a real grasp, all at the same step: the carry move exceeding a 40 s *wall-clock* MoveIt wait at the container's lower real-time factor (host archive: 34.35 s; container green: 39.80 s). 3 further runs VOID (host suspended twice; disk filled by host core dumps) |

Two defects were found by running it, and are fixed (see below): the
wheel controller's **activation race** under software rendering, and a
**readiness gate that passed for a controller that had failed**. A third
finding is a documentation bug: the `bash -lc 'ros2 …'` debugging idiom
this file used to recommend found no `ros2` in either image.

---

## The image

### Why one image, not runtime/test/dev stages

Measured, not assumed. What the test suite and headless RL training need
beyond the appliance is MuJoCo (with PyOpenGL, glfw, absl, etils: 73.3 MB)
and Node (~51 MB): **~124 MB of an 8.36 GB image, 1.5 %**. A separate
runtime stage would save that and cost a second image identity; one image
means the digest CI tests is the digest that runs. A builder stage would
save nothing: the colcon output is 4.3-4.8 MB and the compilers are in the
base layers, which no later stage can remove.

### Layers (hardened image, `docker history`)

| Layer | Size | Why it is there |
|---|---|---|
| `osrf/ros:jazzy-desktop` (digest-pinned) | 3.35 GB desktop + 467 MB ros-core + 318 MB build tools + 97 MB ros-base + 88 MB Ubuntu | ROS 2 Jazzy, rviz2, rqt, OpenCV, scipy, PIL; the known-good base the native install mirrors |
| apt: gz, Nav2, MoveIt, ros2_control, web_video_server, rosbridge, depth/image_proc, nodejs, xvfb, ... | 1.14 GB | the simulator, the navigation and manipulation stacks, and what `rosdep check` demands |
| pip: the locked set | 1.01 GB | torch 2.12.1+cpu alone is 747 MB (the ramp policy runs in inference); mujoco 55 MB |
| source + colcon build | 29 MB + 4.3 MB | the nine packages, `--symlink-install` |
| everything else | < 1 MB | user, manifests, build-info, entrypoint |

The layer sum is 8.36 GB; the merged filesystem is **5.96 GB** (`du -sx /`):
files the base rewrites layer over layer are counted once per layer.
Compressed (what a registry would hold): 1.79 GB for the original image.

A slimmer `ros:jazzy-ros-base` was **not** attempted: MoveIt and Nav2 pull
most of the desktop's Qt/rviz closure back in, and trading the known-good
base for an untested one is a separate change with its own validation.

### Pins: what is hard, what is flexible

- **Base image: pinned by digest** (`osrf/ros:jazzy-desktop@sha256:2f520187…c45`).
  Refresh deliberately: change the line, rebuild, re-run `validate.sh`.
- **pip: the whole layer is locked** (`docker/pip-constraints.txt`, 17
  distributions) and `docker/check_pip_lock.py` fails the build on any
  extra, missing or moved distribution. HARD pins, each for a measured
  reason: torch 2.12.1+cpu / stable-baselines3 2.9.0 / gymnasium 1.3.0 /
  cloudpickle 3.1.2 (what the shipped policy was saved with — its own
  `system_info.txt`), mujoco 3.11.0 (the parity and calibration numbers),
  tornado 6.5.7 (what coco_web is written to and was measured on; apt has
  6.4.0), setuptools 78.1.0 (pulled in by torch; it shadows apt's for
  colcon's `--symlink-install`). The rest are transitive and refreshed with
  `docker/refresh_pip_constraints.sh`.
- **apt: NOT version-pinned, deliberately.** packages.ros.org keeps only
  the latest sync, so an exact pin stops resolving the day ROS syncs again.
  What was installed is recorded in the image instead
  (`/opt/coco/manifest/dpkg.txt`, 1,8xx packages), so drift between two
  builds is a `diff`.
- **Downloads are version-pinned, not hash-pinned.** `--require-hashes`
  needs a hash for every transitive wheel; that is a tooling decision
  (pip-compile/uv) for later, not done here.

### Identity: reproducibility metadata vs build metadata

`coco-entrypoint info` prints both.

- **Reproducibility metadata** — what the image was built FROM — is a
  file in a layer, `/opt/coco/build-info.json`: git sha (and, for an
  overlaid build, the infra sha), dirty flag, the commit's timestamp,
  base digest, ROS distro, RMW, simulator mode, packages. Same commit +
  same base = the same bytes.
- **Build metadata** — branch, and when/where the build ran — never
  enters a layer: branch is image-config env/label only, and wall-clock
  time, duration and per-step timings are written by `build.sh` OUTSIDE
  the image (`build-meta.json`). The commit time is passed as
  `SOURCE_DATE_EPOCH`.
- A plain `docker compose build` records `unknown` for the git fields,
  which is true; `scripts/container/build.sh` fills them in.

### Non-root

The image runs as **`coco`, uid 1000**; the workspace stays root-owned,
so the running stack cannot modify its own code. Verified: the full test
suite (1740/1740) and every platform boot ran as uid 1000 with **every
Linux capability dropped** and `no-new-privileges`. Nothing needed root,
a capability, or a writable source tree. Noble's base ships a user
`ubuntu` at uid 1000; it is removed first.

uid 1000 is chosen so a bind-mounted output directory is writable by the
usual first user of a Linux host. The cost, recorded under *Ports and
collisions*: a container process at uid 1000 is killable by that host
user's own sweeps.

---

## Build performance (measured)

| Build | Wall | Notes |
|---|---|---|
| original Dockerfile, empty Docker | **2711.2 s** | base pull 783 s, apt 1672 s, pip 154 s, colcon 17.9 s, export 80 s; link measured 1.73 MB/s |
| this Dockerfile, base cached, cache mounts cold | **760.4 s** | apt 443.7 s, pip 196.9 s, colcon 18.3 s (a faster network hour than the one above) |
| + two apt packages, cache mounts warm | **229.7 s** | apt 66.3 s (8 new debs, 1.29 MB fetched), pip 49.4 s |
| clean-room, source-only change | **20.8-22.8 s** | only COPY + colcon + identity re-run |
| `main`'s tree with this branch's container files | **22.0-22.4 s** | shares every heavy layer |
| `--no-cache` rebuild of the same commit | **870.3 s** | apt 595 s, pip 171 s: `--no-cache` also bypassed the warm cache mounts |

Cold build time is dominated by the network and varies with it; compare
the per-step timings, not the totals. The apt and pip downloads live in
BuildKit cache mounts, never in the image.

Resources during the cold build (sampled every 2 s, 1,333 samples): host
memory in use rose from 6.29 to at most 7.08 GiB (+0.79 GiB); dockerd +
containerd peaked at 210 MB RSS; load average peaked at 5.1. Free disk
fell from 23.4 to 13.1 GB — **disk, not CPU or RAM, is the constraint
that bites**; see *Mounts and generated data*.

---

## Tests in the container

```bash
docker compose run --rm -v "$PWD/out:/out" -e COCO_TEST_OUT=/out coco test --jobs 9
```

`coco-entrypoint test` runs `scripts/ci/run_package_tests.py`: one pytest
per package, cwd = the package directory (CLAUDE.md), `gazebo_models`
with `--ignore=test_integration`, each package on its own
`ROS_DOMAIN_ID`, JUnit + a JSON summary out. It exits non-zero on any
failure or error.

| Run | Result |
|---|---|
| original image | 1393 passed, 1 error, 2 skipped **of 1396 collected** (host: 1740). `coco_rl`: one module-level `importorskip('mujoco')`, under the launch_testing pytest plugin every ROS environment loads, skips the WHOLE directory — 218 tests become "0 collected / 1 skipped", rc 5. `coco_sim`: collection error. `coco_web`: the Node decoder test skipped (no node). |
| this image, serial, x3 | 1740/1740 each |
| this image, `--jobs 9`, x3 | 1740/1740 each — parallel packages are safe |
| host, serial x3 and `--jobs 9` x3 | 1740/1740 each (78.2/77.6/77.1 s; 22.7/23.2/22.2 s) |
| 4 containers at once, `--jobs 9` each (load1 peak 17.5) | 3 of 4 containers: 1 failure each, always `gazebo_models test_cmd_vel_wiring.py::TestLiveGraph::test_the_relay_output_is_restamped_and_unaltered` ("assert 9 >= 10") |
| that test alone, x20, quiet | 20/20 |
| final image (61bef2a), `--jobs 9`, while another user's native ROS run held host load1 at 23.8 | 1739/1740 — the same relay test |
| final image (61bef2a), `--jobs 9`, x2, minutes later on a quieter host | 1740/1740 each (20.8 / 20.5 s) |
| host, final branch state, `--jobs 9` | 1740/1740 (22.3 s) |
| `main`'s tree (b15d445) in the same infrastructure | 1011/1011 |

**Classification.** Every test PASSES in the container on a quiet host.
The relay test is ENVIRONMENT-LIMITED (CPU starvation; CLAUDE.md
already records it failing 8/9 at load 43 on the host) — not a container
regression, and not modified. The browser harness
(`scripts/browser_check/`) is NOT-APPLICABLE in the image (it starts its
own native simulator and needs Firefox, which Ubuntu ships only as a
snap); the protocol is exercised from the host instead (below).

---

## Running the appliance

`scripts/container/platform_smoke.sh` executes the verification procedure
this file used to list as "never run", through the real
`docker-compose.yml`, and measures it. Every boot below is a fresh
container; none shared a machine with another simulator.

| Boot | Image, settings | Healthy after | Result |
|---|---|---|---|
| 1 | original image, original compose (root, default caps, `0.0.0.0:8080`) | 33.2 s | `/healthz` 200 HEALTHY, READY; RTF 0.409; 8081 refused from host; 1 `cmd_vel` publisher |
| 1 | hardened (89a4e45), non-root, no caps | never | **never healthy** — `/healthz` UNHEALTHY with `robot` missing; captured live, then stopped by hand: the activation race (below) |
| 3 | hardened (c61ab25), + host WebSocket probe | 26.7 / 33.0 / 33.0 s | all HEALTHY; RTF 0.33 / 0.33 / 0.31 |
| 3 | hardened (c61ab25), + Nav2 probe, executive on | 26.5 / 26.6 / 26.5 s | all HEALTHY; RTF 0.36 / 0.37 / 0.38 |
| 3 | hardened (5ed23e7), + Nav2 probe, `executive:=false` | 26.5 / 26.4 / 26.4 s | all HEALTHY; RTF 0.41 / 0.42 / 0.43 |
| 10 + 10 | boot probe: original vs hardened alternated, then the fixed image | — | see *The controller activation race* |

In every healthy boot: all four ros2_control controllers **active**;
`/diff_drive_controller/cmd_vel` has **exactly one** publisher,
`cmd_vel_arbiter`; `web_video_server` listens on `127.0.0.1:8081` only
and the host's `curl localhost:8081` is refused; 48-49 nodes, 205
topics; every TF pair checked resolves (`odom→base_footprint`,
`map→odom`, `base_footprint→base_link`, `base_link→lidar_link`,
`base_link→camera_link`); topic rates scale with the RTF as they should
(at RTF 0.33: `/clock` 160 Hz, odometry 16.3 Hz, `/scan` 3.3 Hz, camera
4.9 Hz); `docker compose stop` 2.5-2.6 s, exit 0, nothing left behind.

**Resources, steady state after healthy:** 6.6-8.1 CPU cores (the gz
server, rendering camera + depth + gpu_lidar in software, is the largest
single consumer), 1.35-1.59 GiB RAM, 422 PIDs (original image). The host
has 12 cores; that is why *one simulator per machine* is not a courtesy.

**The browser protocol, from the host** (`ws_probe.py`, 3 boots): the
coco.v1 WebSocket handshake took 7-20 ms; `welcome` names `coco.v1`;
telemetry arrived at ~10 Hz (100-101 frames in 10 s) plus the `map`
frame; `ping` got its `pong`; **0 binary frames** reached a client that
declared it cannot take them; **0 error frames**; and **0 occurrences of
any of the 163 live ROS topic names** (harvested from the container's own
`ros2 topic list`) in any frame. `/healthz`, `/api/session`,
`/api/metrics` and `/` all answered 200.

### The controller activation race (found and fixed)

The first hardened boot came up UNHEALTHY with `robot` missing:
`diff_drive_controller` was **inactive**. The controller manager logged
`Switch controller timed out after 5 seconds!` and the spawner exited 1.

The spawner gives an activation **5 s of wall time** to happen inside the
controller manager's update loop, which runs on **sim time**. Under
software rendering, Gazebo's Ogre2 sensor initialisation stalls stepping
for seconds. A boot probe (10 boots, original vs hardened image
alternated) showed the mechanism: the 9 boots in which render
initialisation came BEFORE the diff_drive activation request all came up;
the 1 in which it came AFTER failed — and so had the smoke that started
this. The measured stalls sit right on the boundary: failures truncated
at exactly 5.00 s; stalled-but-successful activations took 4.12 and 4.57 s.

Fixed in `full_world_robo.launch.py`: the spawners pass
`--switch-timeout 60`, the spawner's own remedy for a simulation that
cannot switch immediately. It changes how long the spawner waits for an
activation, nothing after it. After the fix: 10/10 boots with every
controller active, 0 timeouts — including 2 with render-after-activation.
**Honest limit:** those two stalls (4.12 s, 4.57 s) were under the old
5 s too, so the post-fix sample did not contain a >5 s stall; the fix
removes the boundary rather than being proven against a stall that
crossed it.

The entrypoint's gate had the matching hole: it waited for an odometry
**publisher**, which `diff_drive_controller` creates when *configured* —
so it passed for a controller that then failed to activate, and started
the stack with wheels that could not move. It now waits for an odometry
**message**, which only an active controller produces, and on failure
dies naming the controller states. The health check was right all along
(`robot` missing, UNHEALTHY): the appliance never reported a dead robot as
drivable.

---

## Nav2 in the container

`scripts/container/nav_smoke.py`, inside a live appliance: lifecycle
state of the ten Nav2 nodes, then a goal 1 m ahead of the robot's map
pose (taken from TF), counting every link of the command chain, then an
unreachable goal 40 m beyond the map.

**With the shipped appliance (executive on), 3 boots:** every Nav2
lifecycle node `active`; the planner published `/plan` (172-188 messages,
21 poses); the controller commanded — `/cmd_vel_nav` ~2,260 non-zero,
`/cmd_vel_smoothed` ~4,500, `/cmd_vel` (after the collision monitor)
~3,900-4,200, `/cmd_vel_gated` (after the relay) the same — and
**`/diff_drive_controller/cmd_vel` received 0**; the robot moved 0.000 m;
Nav2's own feedback counted **8, 8 and 10 recoveries** before the 240 s
probe timeout. That is the design working: the mission executive
re-asserts `/mission/mode` at 2 Hz (`idle` while no mission runs), and the
arbiter forwards Nav2 only in `nav`. A Nav2 goal from outside the mission
is kept off the wheels. It is also why a probe that expects otherwise must
say so: `executive:=false` is the launch file's documented way to drive
the mode from outside, exposed here as `COCO_MISSION_ARGS`.

**With `COCO_MISSION_ARGS=executive:=false`, 3 boots:**

| | rep 1 | rep 2 | rep 3 |
|---|---|---|---|
| reachable goal | SUCCEEDED, 6.7 s wall | SUCCEEDED, 6.7 s | SUCCEEDED, 6.5 s |
| recoveries | 0 | 0 | 0 |
| odometry displacement | 0.698 m | 0.701 m | 0.714 m |
| `/plan` messages | 7 | 7 | 7 |
| non-zero commands: nav / smoothed / collision monitor / relay / **wheels** | 66 / 132 / 131 / 131 / **160** | 66 / 132 / 127 / 127 / **157** | 64 / 128 / 124 / 124 / **151** |
| unreachable goal | ABORTED, error 204 (goal outside map), 0 recoveries | same | same |

The map frame's origin is the spawn pose on this world: rep 1's TF pose
went from (0.000, 0.000) to (0.733, −0.008). Recovery behaviour was
exercised in the executive-on runs (above) — the progress checker saw a
robot that could not move and Nav2 recovered, repeatedly, as designed.

---

## The four-colour regression

**What was reproduced.** The four-colour fetch on `main`'s 24 × 18 m
arena (runtime `474601a`, tree `b15d445`; the host archive,
`main:docs/data/navigation_world_final/results.json`, has all four
COMPLETE, green and blue with one recovery each). Image:
`build.sh --ref main --infra-ref <this branch>` — main's code, this
branch's container files. Runner: `main:docs/data/navigation_world_run.sh`,
**unchanged**, mounted read-only from the same commit; one fresh container
per run; `COCO_TEST_GUI=false`; RViz (which the runner forces on) on
`xvfb-run`; `--network none`; non-root; no capabilities. Results are read
from the executive's and grasp server's own logs by
`extract_mission_result.py`, which reproduces the archived results.json
exactly from the archived logs (all four colours, every field).

**This is a regression matrix, not a success rate.**

| Colour | Run | Result | Wall | Recoveries | Home error | Notes |
|---|---|---|---|---|---|---|
| green | 1 | **COMPLETE/fetch** | 568.9 s | 1 (RELOCALIZE) | 0.115 m | every runner check PASS (host: 495.8 s, 1, 0.060 m) |
| blue | 1 | **ABORT — GRASP_FAILED** | 585.2 s | 1 (RELOCALIZE) | 0.046 m (home, empty-handed) | pick 1: the carry move got no MoveIt result within `arm_control`'s 40 s **wall-clock** wait (code −999); picks 2-3: "could not stow the arm"; `move_group` segfaulted after the third |
| red | 1 | **VOID** | — | — | — | the host SUSPENDED (lid closed) 21 min into the mission with the robot home; the runner's wall-clock budget ran out across the sleep |
| yellow | 1 | **VOID** | — | — | — | the disk was full (below); the stack could not start |
| red | 2 | **ABORT — GRASP_FAILED** | 616.7 s | 0 | 0.052 m (home, empty-handed) | same signature as blue |
| yellow | 2 | **VOID** | — | — | — | the host suspended 23,848 s during the run (detected, not counted) |
| yellow | 3 | **ABORT — GRASP_FAILED** | 594.6 s | 0 | 0.157 m (home, empty-handed) | same signature as blue |

**Valid runs: 4 of 4 colours, 1 COMPLETE (green), 3 ABORT — GRASP_FAILED.**
In all four the robot navigated to its lane, climbed, found and approached
the target and **really grasped it** (lift 35.2-36.9 mm, "the grasp is
real"); every failure is the same next step.

**The mechanism, measured.** After the lift the grasp server's carry move
(`Arm -> 'up'`) waits for MoveIt's result through
`arm_control.await_future(…, 40.0)` — **40 s of WALL time** — while the arm
moves in **sim time**:

| Run | carry move, wall | result |
|---|---|---|
| host archive, green (`main:docs/data/navigation_world_final/green_grasp.txt`) | **34.35 s** | done |
| container, green run 1 | **39.80 s** | done — 0.2 s inside the limit |
| container, blue 1 / red 2 / yellow 3 | cut off at **40.0 s** (code −999) | FAILED; the retries then fail "could not stow" (code −4); `move_group` segfaults afterwards |

The real-time factor during the grasp (from the runner's own `hrec.csv`,
sim vs wall time) orders the outcomes: **green, the one that completed,
0.279-0.280; the three failures 0.226-0.267** (red 0.23, yellow 0.23-0.25,
blue 0.26-0.27) — software rendering, RViz on Xvfb, ~8-10 cores busy. The
host already had only 5.6 s of margin; the container has none. This is a reproducibility
hazard **in the mission code** — a wall-clock timeout on sim-time motion —
that the container exposes. It is not a container defect, and it was
**not changed** here (mission behaviour is out of scope). The fix belongs
to the owner: a sim-time (or RTF-scaled) wait in `arm_control`, or a GPU
in the container to raise the RTF (the NVIDIA Container Toolkit is not
installed on this machine).

**Two infrastructure failures the regression found, both now handled:**

1. **Core dumps filled the host disk.** At every run's teardown, RViz and
   Nav2's component container crashed (exit −11). A crash inside a
   container is handled by the HOST's `core_pattern` — apport on Ubuntu —
   and Docker's default core limit is unlimited: **12 GB** of cores (three
   rviz2 of 2.5-5.0 GB, two Nav2 containers of 0.5 GB) landed in
   `/var/lib/apport/coredump`, root-owned, and filled the disk; the next
   run died at bring-up. Every container now runs with `core=0`
   (compose `ulimits`, and every harness `docker run`); verified: a
   SIGSEGV under `core=0` leaves only apport's 34 KB `.crash` report.
2. **The laptop suspended mid-run, twice** (lid closed: 21 min, then
   3.5 h). A suspend freezes the simulator while every wall-clock budget
   keeps counting. `mission_regression.sh` now measures time spent
   suspended during each run (`CLOCK_BOOTTIME − CLOCK_MONOTONIC`) and
   marks any run that spans a suspend **VOID**. It also runs each
   container detached and named, removes it on any exit, and refuses to
   start on a disk under 2 GB free: when a foreground `docker run` client
   died writing to the full disk, its container outlived it.

---

## Episodes

The episode specification (`coco_sim.episode`) is simulator-independent
and not yet wired into any launch file: no argument consumes a manifest,
and the frozen P0.2 mission still takes its lane from `lane_for_colour()`.
What can honestly be executed today is level `fixed`, whose layout IS the
frozen world. `scripts/container/episode_run.sh` does that:

1. generates the manifest **in the image** from the seed, and on the host
   from the source tree — they must be byte-identical;
2. hands the robot **only** `task_view()` — `{episode_id,
   requested_colour}` — as `COCO_TARGET_COLOUR`; the privileged manifest
   (target poses, obstacles, seed) never enters the container;
3. runs the frozen mission in a fresh, hardened, network-less appliance,
   waiting for LOCALISATION (a `map→odom` transform), not merely
   `/healthz`;
4. records the run as an `EpisodeResult` and regenerates the seed to
   check the record still describes it (`check_reproducible`).

Seed 7, level `fixed` (`ep-2370db8aac74`, requested colour blue): the
host and image manifests were **byte-identical** (sha256 `0692e99c…`);
the robot's input was the two task-view fields; healthy after 25 s,
localised after 37 s; the frozen mission ran **COMPLETE/fetch** in 451.5 s
wall, 0 recoveries, home error 0.130 m, lift 35.5 mm, magnet detached;
the `EpisodeResult` (`outcome: complete`, software commit `61bef2a`)
regenerated from its seed: **`reproducible_from_seed: true`**. One run —
not a rate. (Its carry move took 29.7 s wall on this smaller world, against
39.8-40+ s in the 24 × 18 m arena; see the regression.)

Levels `colours` and `positions` are NOT-APPLICABLE until a world can be
spawned from a manifest (stage C of `docs/EPISODE_ARCHITECTURE.md`);
`episode_run.sh` refuses them rather than run the wrong world.

---

## Reproducibility and determinism

`scripts/container/determinism.sh` rebuilt commit 61bef2a with
`--no-cache` and compared it with the cached build of the same commit.

- **Layers:** the 11 base layers are identical (digest pin). **All 12 of
  COCO's layers differ** — including a `COPY` of two byte-identical
  files from the same `git archive`, because the directory the COPY
  creates is stamped with the build's wall clock. The image is **not
  bit-for-bit reproducible.**
- **Contents, ignoring timestamps:** of 27,701 files hashed under the
  trees COCO's layers write, 8,254 differ — **every one of them is Python
  bytecode (8,251 `.pyc`, whose headers embed source mtimes) or a CMake
  configure log (3)**; 3 CMake API reply files have a timestamp in their
  NAME. Zero other files differ: `/var/lib/dpkg/status`, both manifests,
  `build-info.json` and `/etc/passwd` hash identically. The software is
  **content-reproducible, modulo bytecode and build logs**, on the same day.
- **Across days it will not be:** apt is unpinned by design, so a rebuild
  after a ROS sync installs newer debs. `/opt/coco/manifest/dpkg.txt`
  makes that a one-line `diff`.
- **`--no-cache` also bypassed the BuildKit cache mounts**: that rebuild
  re-downloaded everything (apt 595 s, pip 171 s, 870 s total) where a
  warm rebuild of the same layers took 64 s and 52 s.
- Not done, and what bit-reproducibility would take: `SOURCE_DATE_EPOCH`
  with BuildKit's `rewrite-timestamp` exporter option, bytecode compiled
  with `--invalidation-mode unchecked-hash` (or not at all), and the CMake
  logs removed at build time.

**Intentional nondeterminism:** physics and sensor noise (the simulator is
not deterministic and nothing here tries to make it so), `requested_colour`
when an episode is drawn without one (seeded, so reproducible from the
seed), and the robot's path. **Accidental nondeterminism found:** file
timestamps and bytecode in every layer; three parametrised test ids that
embed the absolute checkout path (`coco_mission`'s
`test_no_launch_file_on_the_path_names_the_raw_topic_in_code[...]`), so
test-result ids differ between a host and a container checkout until the
path is normalised; host load, which alone decides whether the
`test_cmd_vel_wiring` relay test passes; and, before the fix, the order of
Gazebo's render initialisation against a controller activation.
**Deliberately kept out of the image:** wall-clock time, host, branch
(config-only), ROS domain (unset; `validate.sh` proves a host value
never leaks in).

---

## Networking and the ROS graph

- **Default (compose's bridge network):** the whole DDS graph is inside
  the container, on its own loopback (`CYCLONEDDS_URI` pins Cyclone to
  `lo`; `GZ_IP=127.0.0.1` does the same for gz-transport). Measured with
  the appliance healthy: the host's `ros2 topic list` (domain 0, same RMW
  settings) showed only its own `/parameter_events` and `/rosout` — none
  of the container's ~205 topics.
- **The container never needs the network at run time.** The test suite
  and the mission regression run under `--network none`.
- **`--network host` is a debugging opt-in, not a default.** It shares the
  host's loopback, so Cyclone would join the host's domain-0 graph —
  where other work on this machine (another project's Gazebo, another
  agent's simulator) publishes `/clock`. If you must, set a distinct
  `ROS_DOMAIN_ID` in both places.
- **Published ports:** 8080 only, on the **host's loopback** by default.
  The platform has no authentication yet (P1); a bare `8080:8080` binds
  every host interface — measured, the original compose did exactly that
  (`0.0.0.0:8080`). `COCO_BIND=0.0.0.0` is the deliberate opt-in. Inside
  the container `web_video_server` listens on `127.0.0.1:8081` only.
- **Future multi-session (P1.1, not solved here):** one container per
  session on its own bridge network; the web tier speaks HTTP/WebSocket
  to each; DDS never crosses a container boundary.

---

## Security baseline

| Item | Before | Now |
|---|---|---|
| user | root | `coco` (uid 1000) |
| capabilities | Docker default set | **all dropped** (`cap_drop: ALL`), `no-new-privileges` |
| host publish | `0.0.0.0:8080` | `127.0.0.1:8080` (opt-in `COCO_BIND`) |
| 8081 (web_video_server) | not published, loopback inside | unchanged |
| privileged / docker socket / host mounts | none | none |
| secrets in layers | none found | none (no credentials are used by the build) |
| DDS exposure | inside the container | inside the container; `--network none` for tests and missions |
| core dumps | unlimited (Docker default), written by the HOST's apport | `core: 0` — measured: 12 GB of host cores from three teardowns before; none after |

Not addressed here, on purpose: authentication and TLS (P1), image
signing, a read-only root filesystem.

---

## Mounts and generated data

`docker-compose.yml` mounts **nothing** from the host.

| Artifact | Where | Why |
|---|---|---|
| build/, install/ | in the image | `--symlink-install` needs build/ next to the source |
| colcon log/ | removed at build | timestamped; neither needed nor deterministic |
| apt debs, pip wheels | BuildKit cache mounts | fast rebuilds; never in the image |
| ROS logs, Gazebo caches | container home (ephemeral) | gone with the container; a run that must keep them sets `ROS_LOG_DIR` into a mounted directory (the regression does) |
| test results, evidence | a bind mount the caller chooses (`-v out:/out`) | the only thing a run writes to the host |
| episode manifests / results | the caller's output directory | privileged; never mounted into the robot's view |
| browser profiles | outside Docker (`scripts/browser_check/`) | native harness |

**Disk, measured on this machine:** the image is 8.36 GB of layers
(1.83 GB compressed). The original cold build took free space from 23.4
to 13.1 GB; the first build of this Dockerfile (new apt and pip layers,
cache mounts filling) took another ~5 GB. Two traps:
`docker rmi` frees nothing while BuildKit cache records still reference
the layers, and `docker builder prune` followed by a rebuild DUPLICATES
the heavy layers if another image still holds the old ones (4.7 GB
consumed by one 236 s rebuild). Rebuild every image you keep on the new
layers first, then prune: `docker builder prune --filter type=regular`
keeps the apt/pip cache mounts.

---

## Failure capture

A stage of `validate.sh` that fails leaves `OUT/failures/<stage>/`:

| File | Contents |
|---|---|
| `context.json` | stage, exit code, the exact command, UTC time, image tag, image id and repo digest, git sha and infra sha (from the image), base image, ROS distro, RMW, ROS domain, host kernel/CPUs/memory/load, Docker server version |
| `env.txt` | the container's ROS/RMW/DDS/GZ/COCO/path variables; any variable named like a secret is redacted |
| `dpkg.txt`, `pip.txt` | the image's own manifests, or — for an image built without them — `dpkg-query`/`pip list` run in it |
| `build-info.json` | the image's reproducibility metadata |
| `container.log`, `coco_sim.log`, `coco_stack.log`, `ros_log/` | for a stage that ran a live container |
| the stage's own output | JUnit and summaries for a failed test run |

Demonstrated on a genuinely failing image — the original one — not on a
staged failure: four bundles (identity: no `info`; environment: runs as
root; dependencies: rosdep unsatisfied, no manifests; tests: the original
entrypoint has no `test` command), each with 1,887 dpkg and 110 pip
versions — among them the drifted `torch==2.14.0+cpu`. The activation-race
boot was captured live the same way (controller states, `/healthz`, sim
and stack logs, ROS logs, the container's inspect record).

---

## CI

`.github/workflows/container.yml` builds clean-room from the commit and
runs `validate.sh` (identity, environment, dependencies, full test suite
— which already contains every package's ament flake8, pep257 and
copyright tests). The Gazebo platform smoke is a manual opt-in. It is
**not triggered by a branch push** and **has not yet run on GitHub**.

What a GitHub-hosted runner (4 vCPU, 16 GB, no GPU, ~14 GB free disk
before cleanup) can and cannot do:

| Job | Needs | Hosted runner |
|---|---|---|
| lint + unit (the test stage) | 8.4 GB image, ~2 min | yes, after freeing disk |
| container build | network, ~15-20 GB disk at peak | yes, after freeing disk |
| Gazebo smoke | ~7 CPU cores at RTF ~0.3 here, software rendering | expect a lower RTF; untested |
| Nav2 / mission regression | 15-20 min per colour here, a virtual display | not a hosted-runner job; self-hosted |
| Isaac Sim | NVIDIA GPU + container toolkit | no |

---

## Why one container

`TASK 8` sketched a `coco` / `web` split, and then said the split should
follow the existing architecture and not separate ROS nodes without
reason. It does not, here.

`platform_server` **is a ROS 2 node**. It subscribes to odometry, the
scan, the plan, the map and three status topics, and publishes to an
arbiter input. Putting it in a second container means running a DDS graph
across containers: a shared network namespace, a matching
`CYCLONEDDS_URI`, and discovery that fails in ways which present to the
user as *"the robot will not move"*. For a single-user appliance whose
first goal is reliability, that is cost with no benefit.

The split becomes necessary at **P1.1**, when the web tier fronts many
simulation containers rather than sharing one graph with a single
simulator. `PRODUCT_ARCHITECTURE.md` records that.

---

## What starts, in what order

`docker/entrypoint.sh`:

1. Source ROS and the workspace overlay. If the overlay is missing, die
   loudly — the image did not build.
2. `gazebo_models full_world_robo.launch.py gui:=false traverse:=true`.
3. **Wait for `/model/coco/odometry`** (the gz plugin: the simulator is
   stepping), then **wait for a message on `/diff_drive_controller/odom`**
   (the wheel controller is ACTIVE — a publisher alone is not enough; see
   *The controller activation race*).
4. `coco_mission mission.launch.py platform:=true web:=true rviz:=false`,
   plus `COCO_MISSION_ARGS` if set.
5. Poll `/healthz` and **log** what is missing.

Step 5 deliberately does not gate anything. `HEALTHCHECK` is the single
source of truth for readiness; the loop exists so the log says *"not
ready yet: missing simulator, robot"* instead of going quiet.

The ordering in 2–4 is load-bearing. Starting the mission stack before
the simulator publishes a clock leaves every `use_sim_time` node waiting
on a `/clock` that is not there, and Nav2's lifecycle then times out in a
way that reads like a Nav2 bug.

`tini` is the init. Without one, killing the container leaves `gz sim`
and the component containers as zombies inside it. Measured: `docker
compose stop` takes 2.5-2.6 s, exit code 0, and leaves nothing behind.

---

## Health

```bash
docker compose ps                     # starting -> healthy
curl -s localhost:8080/healthz | head -20
```

`/healthz` returns **200 exactly when health is not `UNHEALTHY`** —
every required component up (`ros`, `simulator`, `robot`, `arbiter`) and
the session neither FAILED nor shutting down — and 503 with the missing
ones named otherwise. So `docker ps` saying *healthy* and *the robot can be
driven* are the same statement. Measured both ways: the activation-race
boot stayed `starting` → `unhealthy` with `missing: ["robot"]`; every good
boot went healthy in 26.5-33.2 s, well inside the 180 s `start_period`.

`mission`, `perception` and `navigation` are reported but **not**
required: the appliance is useful for manual driving without them.

---

## Configuration

| Variable | Default | Meaning |
|---|---|---|
| `COCO_HTTP_PORT` | `8080` | UI, `/ws`, `/healthz`, MJPEG at `/video/<alias>` (host side) |
| `COCO_BIND` | `127.0.0.1` | host address the port is published on |
| `COCO_IMAGE` | `coco-platform:jazzy` | which image compose runs (`build.sh` tags `coco-platform:<sha12>`) |
| `COCO_VIDEO_PORT` | `8081` | `web_video_server`, **inside the container, loopback only**; not published |
| `COCO_TARGET_COLOUR` | `blue` | the colour the mission preselects |
| `COCO_MISSION_ARGS` | empty | extra `mission.launch.py` arguments, e.g. `executive:=false` |
| `COCO_GUI` / `COCO_RVIZ` | `false` | Gazebo GUI / RViz (need a display; `xvfb-run` is in the image) |
| `COCO_GIT_SHA`, `COCO_GIT_BRANCH`, `COCO_GIT_DIRTY`, `COCO_SOURCE_DATE_EPOCH` | `unknown`/`0` | build args; `build.sh` sets them |

**No host path may appear in the image.** The workspace is `/opt/coco_ws`.
The machine this was developed on has its workspace at
`~/ros2_ws(personal)` — parentheses in the path, which have already
broken git and CMake quoting in this project once — and a test asserts no
such path reaches the Dockerfile, the compose file or the entrypoint.

---

## Ports and collisions

Only `8080` is published. The ROS graph stays inside the container: it
cannot collide with a simulator already running natively on the host.

**But the host can see the container's processes.** Containers share the
host's PID namespace view: a host `pgrep -f 'g[z] sim'` lists the
container's Gazebo, so a native runner that refuses to start while "a gz
sim is running" will refuse while a COCO container runs, and
`ros_clean.sh`'s `g[z] sim.*gazebo_models/worlds` pattern MATCHES the
container's server (`/opt/coco_ws/install/gazebo_models/share/gazebo_models/worlds/...`).
At uid 1000 — the usual host user's uid — that host user's `pkill` can
kill it. One Gazebo per machine applies across the container boundary.

---

## Debugging

```bash
docker compose logs -f coco
docker compose exec coco coco-entrypoint shell           # overlay sourced
docker compose exec coco coco-entrypoint ros2 node list  # any command, overlay sourced
docker compose exec coco coco-entrypoint ros2 topic info /diff_drive_controller/cmd_vel -v
docker compose exec coco coco-entrypoint ros2 control list_controllers
docker compose exec coco cat /tmp/coco_sim.log
docker compose exec coco cat /tmp/coco_stack.log
docker compose run --rm coco info
```

`bash -lc 'ros2 …'` works too now (`/etc/profile.d/coco-ros.sh`); before
this pass it found no `ros2` in either image — measured.

The safety check worth knowing: `/diff_drive_controller/cmd_vel` must
have **exactly one** publisher, `cmd_vel_arbiter` — measured 1 in every
boot here. If it has two, the robot tracks the average of two decisions.

---

## Running natively instead

```bash
./scripts/run_platform.sh --native
```

It needs ROS 2 Jazzy, Gazebo Harmonic and this workspace built. It
refuses to start if another Gazebo is running. See `RUNNING.md`.
