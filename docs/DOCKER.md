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
(1139 tests, clean 9/9 build, live `platform_server` smoke test), but a
translation is not a run. Treat the first `docker compose build` as a
bring-up, not a regression, and expect to fix things.

What *is* verified, without Docker:

| Checked | How |
|---|---|
| every package the mission needs is in the build | `coco_rl/test/test_docker_context.py` |
| the shipped PPO policy survives `.dockerignore` | same |
| worktrees and build trees are excluded | same |
| no host path is baked into the image | same, comments excluded |
| both shell scripts parse | `bash -n` |
| `/healthz` answers 503 until ready, 200 when ready | `coco_web/test/test_session.py`, plus a live run |

What is **not** verified: that the apt package set resolves, that
`colcon build` succeeds in the image, that Gazebo Harmonic runs headless
under software rendering at a useful rate, or that the health check goes
green.

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

`/healthz` returns **200 only when every required component is up**
(`ros`, `simulator`, `robot`, `arbiter`), and 503 with the missing ones
named otherwise. So `docker ps` saying *healthy* and *the robot can be
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
| `COCO_HTTP_PORT` | `8080` | UI, `/ws`, `/healthz` |
| `COCO_VIDEO_PORT` | `8081` | MJPEG |
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

Only `8080` and `8081` are published. The ROS graph stays inside the
container, which is deliberate: it cannot collide with a simulator
already running natively on the host.

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
