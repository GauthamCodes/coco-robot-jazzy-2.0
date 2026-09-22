# DOCKER — the local platform runtime

```bash
git clone <this repo> && cd coco-robot-ros2
docker compose build          # first build is long: ROS desktop + CPU torch
docker compose up
# then open http://localhost:8080
```

or, equivalently:

```bash
./scripts/run_platform.sh     # builds if needed, then up
./scripts/run_platform.sh --down
```

---

## Status: authored, not runtime-tested

**`docker build` has never been executed against this Dockerfile.** Docker
is not installed on the machine this was written on — no binary, no
daemon, no podman — so the image has not been built, started, or
health-checked.

This is a careful translation of a stack that **is** verified natively
(a clean 9/9 build, the full test suite, and complete fetches driven
first through the platform's WebSocket and then, at P0.2's second pass,
through the shipped page in a real browser), but a translation is not a
run. Treat the first `docker compose build` as a bring-up, not a
regression, and expect to fix things.

**The P0.2 release pass changed the image's ports, from inspection.**
The platform now serves the retained MJPEG view itself at
`:8080/video/<alias>` and reaches `web_video_server` on loopback, so
`web_video_server` binds `127.0.0.1` (`platform.launch.py`), compose no
longer publishes `8081`, and the Dockerfile `EXPOSE`s `8080` only. A
published 8081 would have handed any browser on the LAN any image topic
on the graph — its URLs take the topic in the query string. `web/frame.js`
is a new page asset and a test confirms `.dockerignore` lets it through.
Nothing else the image depends on changed: no apt package, no Python
import. Still **NOT VERIFIED** at runtime.

**P0.2's second pass changed nothing the image depends on.** No port, no
apt package, no Python import was added: the health axis, the depth
default and the UI fixes are all inside `coco_web`, and the new
`scripts/browser_check/` harness is a development tool that is not part
of the appliance (Firefox is not in the image). One new *default*
matters at runtime: `depth_topic` now points at the depth image, so a
browser can show depth without a parameter — still display only, still
subscribed only while someone watches.

**P0.2 changed two things here, both from inspection rather than a run.**
The image still installs `web-video-server`, which remains correct
because MJPEG is kept for one more release; binary sensor frames ride the
existing 8080, so no port was added. (P0.2's first pass also kept 8081
published; the release pass withdrew it — above.)

What did change: `coco_web` now encodes JPEG, so it imports `cv2` and
`numpy` directly. `osrf/ros:jazzy-desktop` does carry OpenCV, but relying
on that is relying on the base *variant* — so `python3-opencv` and
`python3-numpy` are now named explicitly in both the Dockerfile's apt set
and `coco_web/package.xml`. It also does **not** import `psutil`, which
is not in the image; CPU comes from `os.times()`, and a test asserts
that.

### Verification procedure, for a machine that has Docker

Nothing below has been run. Run it in this order, on a machine with **no
other Gazebo running**, and record what happens — every step names what
counts as a pass. Do not install Docker onto the development machine to
do this unasked; it is not there, and nothing in the repo assumes it.

```bash
cd <repo>
docker --version && docker compose version    # record both
docker compose build 2>&1 | tee build.log     # first build is slow; expect friction
docker image ls coco-platform:jazzy           # record the size

docker compose up -d
# "healthy" and "drivable" are the same statement here, so wait for it:
watch -n5 docker compose ps                   # STATUS: (health: starting) -> (healthy)
curl -fsS http://localhost:8080/healthz | head -30
#   PASS: HTTP 200, "health": "HEALTHY" or "DEGRADED", "lifecycle": "READY"
curl -fsS http://localhost:8080/api/session | head -30
curl -fsS http://localhost:8080/api/metrics | head

# the ports, from the HOST:
curl -s -o /dev/null -w '%{http_code}\n' http://localhost:8080/video/annotated
#   PASS: 200 and a multipart stream (Ctrl-C it), or 503 if video:=false
curl -s -m 3 http://localhost:8081/ ; echo "exit=$?"
#   PASS: connection refused -- 8081 must NOT be reachable from the host
docker compose exec coco bash -lc 'ss -ltn "( sport = :8081 )"'
#   PASS: 127.0.0.1:8081 only

# the command path, inside the container:
docker compose exec coco bash -lc 'ros2 topic info /diff_drive_controller/cmd_vel -v'
#   PASS: Publisher count: 1, node cmd_vel_arbiter
docker compose exec coco bash -lc 'ros2 node info /coco_web_platform | sed -n "/Publishers/,/Service/p"'
#   PASS: the only velocity publisher is /cmd_vel_teleop

# then open http://localhost:8080 and confirm, in Play mode:
#   the world view draws the ramp, platform and four target lanes
#   the robot is drawn at home, and "finding its position" clears
#   the joystick and W/A/S/D move the robot; STOP halts it, including
#     with W still held
#   ticking "show camera" starts frames; unticking stops them; depth too
#   picking a colour and pressing Start runs the fetch to COMPLETED
#   closing the tab mid-drive stops the robot (arbiter status: idle)

# scripts/browser_check/ is a NATIVE harness: live_run.sh starts its own
# simulator and a ROS-side wheel recorder, which cannot see inside the
# container's graph. Against Docker, do the page checks above by hand.

docker compose logs -f coco          # if the health check never goes green
docker compose down                  # shutdown; then `docker ps -a` is empty
```

Record, at minimum: build success and time, image size, seconds from
`up` to `(healthy)`, the real-time factor (executive `elapsed` against
wall time), one fetch's outcome, and the two command-path checks. Until
all of that exists, the status line above stays **NOT VERIFIED**.

`HEALTHCHECK` has a 180 s `start_period` because the simulator spawn is
the slow part and slower still under software rendering. Until it passes
the container reports *starting*, not *unhealthy* — which is the honest
state while Gazebo boots.

What *is* verified, without Docker:

| Checked | How |
|---|---|
| every package the mission needs is in the build | `coco_rl/test/test_docker_context.py` |
| the shipped PPO policy survives `.dockerignore` | same |
| worktrees and build trees are excluded | same |
| no host path is baked into the image **or the entrypoint** | same, comments excluded (entrypoint added at P0.2) |
| `docker-compose.yml` parses, is ONE service, publishes **8080 only** (not 8081) | same, via PyYAML (release pass; P0.2 published both) |
| the Dockerfile `EXPOSE`s 8080 only | same (release pass) |
| `platform.launch.py` binds `web_video_server` to `127.0.0.1` | `coco_web/test/test_web_assets.py`, reading the real launch description; and measured natively with `ss -ltn` during the release pass's live runs |
| compose and Dockerfile agree on `/healthz` and a 180 s start period | same (P0.2) |
| every `COPY` source exists and is not dockerignored | same (P0.2) |
| every `coco_web` runtime `exec_depend` is apt-installed by the image | same (P0.2), checked both ways |
| all 19 apt packages the Dockerfile names **exist** in the Ubuntu Noble + ROS Jazzy apt index | `apt-cache policy` on the dev host, 2026-09-22 — every one has a candidate (e.g. `ros-jazzy-web-video-server 3.1.0`, `ros-jazzy-nav2-bringup 1.3.13`, `tini 0.19.0`). Same distro as `osrf/ros:jazzy-desktop` |
| all nine packages build from scratch | a clean `colcon build` into an empty overlay, 9/9 in 18.4 s (natively, not in the image) |
| both shell scripts parse | `bash -n` |
| `/healthz` answers 503 until ready, 200 when ready | `coco_web/test/test_session.py`, `test_platform_server.py`, plus live runs |

What is **not** verified — **Docker runtime: NOT VERIFIED**: that the
image builds (the apt names resolve, but nothing proves the install
succeeds inside that base), that `colcon build` succeeds in the image,
that Gazebo Harmonic runs headless under software rendering at a useful
rate, or that the health check goes green. The native live run measured a
real-time factor of about 0.4 *with a GPU*; under `LIBGL_ALWAYS_SOFTWARE`
expect worse, and see *Likely first-build friction*.

### Likely first-build friction

- **Software rendering.** `LIBGL_ALWAYS_SOFTWARE=1` is set because no GPU
  is assumed. Gazebo's *sensor* rendering (the camera the fetch mission
  depends on) will be slow. If the sim runs far below real time, mission
  timeouts will fire before anything is wrong with the mission.
- **Image size.** `osrf/ros:jazzy-desktop` plus MoveIt, Nav2 and CPU
  torch is several gigabytes. A slimmer `ros-base` image is a reasonable
  follow-up; it was not attempted here because it trades a known-good
  base for an untested one in a change that is already untested.
- **`start_period: 180s`.** Chosen to cover a Gazebo spawn on software
  rendering. If the health check flaps, raise it rather than lowering the
  bar for "ready".

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
3. **Wait for `/model/coco/odometry`**, then **wait for
   `/diff_drive_controller/odom`**. Both, for the reason `verify_all.sh`
   gives: the gz plugin's odometry appears well before `ros2_control`
   finishes activating, so waiting only for it races the controllers.
4. `coco_mission mission.launch.py platform:=true web:=true rviz:=false`.
5. Poll `/healthz` and **log** what is missing.

Step 5 deliberately does not gate anything. `HEALTHCHECK` is the single
source of truth for readiness; the loop exists so the log says *"not
ready yet: missing simulator, robot"* instead of going quiet for three
minutes.

The ordering in 2–4 is load-bearing. Starting the mission stack before
the simulator publishes a clock leaves every `use_sim_time` node waiting
on a `/clock` that is not there, and Nav2's lifecycle then times out in a
way that reads like a Nav2 bug.

`tini` is the init. Without one, killing the container leaves `gz sim`
and the component containers as zombies inside it, and the next
`docker compose up` starts a second simulator on top of the first.

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
driven* are the same statement — which is the whole point of `TASK 10`. A
health check that goes green while Gazebo is still spawning is worse than
none, because orchestration then routes traffic at a simulator that
cannot answer.

`mission`, `perception` and `navigation` are reported but **not**
required: the appliance is useful for manual driving without them.

---

## Configuration

Environment variables, all with container defaults:

| Variable | Default | Meaning |
|---|---|---|
| `COCO_HTTP_PORT` | `8080` | UI, `/ws`, `/healthz`, MJPEG at `/video/<alias>` |
| `COCO_VIDEO_PORT` | `8081` | `web_video_server`, **inside the container, loopback only**; not published |
| `COCO_TARGET_COLOUR` | `blue` | the colour the mission preselects |
| `COCO_GUI` | `false` | Gazebo GUI (needs X passthrough) |
| `COCO_RVIZ` | `false` | RViz (same) |

Host overrides:

```bash
COCO_HTTP_PORT=9090 docker compose up
```

**No host path may appear in the image.** The workspace is `/opt/coco_ws`.
The machine this was developed on has its workspace at
`~/ros2_ws(personal)` — parentheses in the path, which have already
broken git and CMake quoting in this project once — and a test asserts no
such path reaches the Dockerfile or the compose file.

---

## Ports and collisions

Only `8080` is published (release pass; P0.2 also published `8081`,
withdrawn because `web_video_server` would serve any topic named in its
URL). The ROS graph stays inside the container, which is deliberate: it
cannot collide with a simulator already running natively on the host.

That is not hypothetical. An unrelated project's Gazebo was running on
the development machine throughout this work, publishing `/clock` onto
the same graph — which is also why `ros_clean.sh` scopes its sweep to
this repo's own worlds rather than any `gz sim`, and why
`run_platform.sh --native` **refuses to start** when it finds another
simulator instead of sweeping it away. Cleaning up after someone else's
simulator is not a script's business.

---

## Debugging

```bash
docker compose logs -f coco
docker compose exec coco coco-entrypoint shell   # overlay sourced
docker compose exec coco bash -lc 'ros2 node list'
docker compose exec coco bash -lc 'ros2 topic info /diff_drive_controller/cmd_vel'
docker compose exec coco cat /tmp/coco_sim.log
docker compose exec coco cat /tmp/coco_stack.log
```

The safety check worth knowing: `/diff_drive_controller/cmd_vel` must
have **exactly one** publisher, `cmd_vel_arbiter`. If it has two, the
robot tracks the average of two decisions — see
`PRODUCT_ARCHITECTURE.md`.

---

## Running natively instead

The repo is developed natively and that path is fully verified:

```bash
./scripts/run_platform.sh --native
```

It needs ROS 2 Jazzy, Gazebo Harmonic and this workspace built. It
refuses to start if another Gazebo is running. See `RUNNING.md`.
