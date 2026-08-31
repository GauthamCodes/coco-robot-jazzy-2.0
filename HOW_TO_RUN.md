# How to Run COCO 2.0

COCO is a simulated autonomous mobile manipulator. You open a control
panel in a browser — on your phone, or on the machine running it — pick
one of four coloured objects, and press start. COCO drives across the
arena under Nav2, climbs a ramp under a learned policy, finds *your*
colour with its own camera, picks it up, carries it home and puts it
down, checking at every step whether the step actually worked.

There is one demonstration, and it is the whole robot.

---

## Requirements

| | |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Simulator | Gazebo Harmonic (`gz sim`) |
| Python | 3.12 |

---

## 1. Clone

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/GauthamCodes/coco-robot-jazzy-2.0.git
cd coco-robot-jazzy-2.0
```

Everything is on `main`. There is no other branch, and nothing to
download separately — the trained ramp policy ships in the repository.

## 2. Install dependencies

```bash
sudo apt install \
    ros-jazzy-desktop ros-jazzy-ros-gz \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-smac-planner \
    ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-slam-toolbox ros-jazzy-cv-bridge ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rosbridge-suite ros-jazzy-web-video-server \
    python3-numpy python3-scipy

pip install --user --break-system-packages stable-baselines3 gymnasium
```

All of these are needed to *run* COCO, not just to develop it:
`rosbridge-suite` and `web-video-server` are what the browser talks to,
`numpy`/`scipy` build the localization monitor's distance field, and
`stable-baselines3`/`gymnasium` load the ramp policy.

`--break-system-packages` is not optional on Ubuntu 24.04: it ships a
[PEP 668](https://peps.python.org/pep-0668/) Python, and plain `pip
install --user` refuses with *externally-managed-environment*. A
virtualenv works too, as long as ROS 2 can import from it.

## 3. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source src/coco-robot-jazzy-2.0/setup_env.sh
```

**Expect:** `Summary: 9 packages finished`, **0 errors**. `setuptools`
deprecation warnings on stderr are normal. If your workspace holds other
packages too, add `--packages-select coco_config coco_sim coco_rl
coco_perception coco_moveit_config custom_teleop gazebo_models
coco_mission coco_web` to build only these nine.

Source `setup_env.sh` **in every terminal, before anything else**. It
finds the workspace from its own path — so the clone can live anywhere
under `<workspace>/src/` — and sets up ROS, this workspace's overlay,
DDS, Gazebo and the render fallback. If it prints `note: no overlay at
<ws>/install`, build first and source second.

---

## 4. Run COCO ⭐

![COCO fetching a cylinder](docs/images/demo_fetch.gif)

*COCO navigates, climbs, identifies the selected target, grasps it,
returns home and places it. Four moments from one continuous run.*

### Start COCO

Two terminals, each with `setup_env.sh` sourced.

```bash
# T1 — the simulator
ros2 launch gazebo_models full_world_robo.launch.py traverse:=true
```

```bash
# T2 — the robot, and the control panel
ros2 launch coco_mission mission.launch.py rviz:=false
```

That is all of it. No environment variables, no policy path, nothing to
configure. Bringing the stack up never moves the robot — nothing happens
until you press start in the browser.

`rviz:=false` leaves the GPU to the simulator, so you watch the robot in
the Gazebo window. Drop it for the RViz mission view — map, both plans
and the perception markers.

### Open the control panel

T2 serves the panel itself. On the machine running COCO:

```text
http://localhost:8000
```

From your phone, on the same network as that machine, use its address on
the network instead of `localhost` — the phone's own `localhost` is the
phone:

```bash
hostname -I | awk '{print $1}'     # e.g. 192.168.1.42
```

```text
http://192.168.1.42:8000
```

The dot beside the title turns **green** when the browser has reached
the robot. If it stays red, the page loaded but rosbridge did not — see
*Troubleshooting*.

### Choose a target

In the **Fetch** card, press one of

```text
RED (20 mm)   GREEN (24 mm)   BLUE (28 mm)   YELLOW (32 mm)
```

Your choice is what the robot's camera will hunt for. The four
cylinders differ in size as well as colour, and the button shows both.

### Start the mission

Press **Start mission**. It stays greyed out until a colour is picked,
because the robot refuses to set off without a target.

The panel then shows the mission's own state line, live, and offers
**Abort** for the rest of the run.

### What happens

```text
navigate  →  climb  →  detect  →  grasp  →  return  →  place
```

Nav2 drives the flat. The ramp cannot be planned onto at all — the lidar
plane cuts an 18° slope, so it scans as a solid wall — so a learned PPO
policy takes the wheels for the climb and hands them back at the top. On
the platform the camera measures where your cylinder actually is, a
visual servo drives onto that estimate, and MoveIt picks it up. Then
back down, home, and set down.

### Success

The panel's state line ends on:

```text
state=COMPLETE ... result=fetch
```

That is the win. The mission otherwise ends in `ABORT`, which always
carries a `reason=`.

**Between runs, tear the simulator down and start a fresh one:**

```bash
bash "$(ros2 pkg prefix gazebo_models)/lib/gazebo_models/ros_clean.sh"
```

Gazebo's `DetachableJoint` binds its child on first spawn, so a second
mission in the same simulator picks nothing up and reports success
anyway.

---

## Tests

Run them **per package, from inside that package's directory, on a clean
ROS graph**. A live stack changes what the `coco_mission` fixtures see,
and one pytest run across the whole workspace dies on duplicate test
module names before it runs anything.

```bash
bash "$(ros2 pkg prefix gazebo_models)/lib/gazebo_models/ros_clean.sh"
cd coco_mission && python3 -m pytest -q && cd ..
```

Repeat for `coco_config`, `custom_teleop`, `coco_rl`, `coco_perception`,
`coco_moveit_config` and `coco_sim`; `gazebo_models` needs one extra
flag, `python3 -m pytest -q --ignore=test_integration`.

**Expect 829 passing, 0 failing, 0 skipped** across the eight packages.
`coco_web` has no tests; pytest exits 4 there, and that is not a failure.

---

## Troubleshooting

| What you see | What to do |
|---|---|
| The panel loads but its dot stays **red** | rosbridge is not running or is unreachable. Check T2 for `Rosbridge WebSocket server started on port 9090`, and install `ros-jazzy-rosbridge-suite` if it is missing. From a phone, confirm you used the machine's network address, not `localhost`. |
| **Start mission** never enables | No colour is picked. Press one of the four buttons first. |
| The state line reads `mission: offline` | The executive is not up. It is started by T2; check that terminal for errors. |
| `package 'coco_mission' not found` | The overlay was not sourced, or was sourced before the build. Build from the workspace root, **then** `source setup_env.sh` in each terminal. |
| Every Nav2 goal rejected, *"Action server is inactive"*; the map missing | Orphaned processes from a previous run are holding a stale `/clock`. Run `ros_clean.sh` and start again. The tell is that each run is worse than the last. |
| The mission stalls at `CLIMB` | No policy was found. The shipped one is at `coco_rl/policies/phase5_24deg_s0p0.zip` and is installed by the build — rebuild `coco_rl` and source again. |
| The robot picks nothing up, but the run reports success | The simulator was reused. Every mission needs a fresh one. |

Two behaviours are **known, and not faults in your setup**: the
localization monitor reads `UNKNOWN` on the ramp and the platform,
because they are outside the map and there is nothing to score a scan
against; and `/cmd_vel_nav` has several publishers, a documented and
unfixed topic loop that means the collision monitor's gating does not
reach the wheels.

### Limitations

COCO is a simulation result, not a product, and has never run on physical
hardware. Severe localization divergence is detected but **not** reliably
recovered. Grasping is calibrated to these four cylinders, not to objects
in general. Missions do sometimes abort — the reason is always on the
state line. The measured evidence, including what did not work, is in
[`docs/RESULTS.md`](docs/RESULTS.md).

---

## Technical documentation

- [`README.md`](README.md) — what COCO is, what was measured, and what
  it still cannot do
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — the nine packages, the
  four control paradigms, and who owns the wheels in each mission state
- [`docs/RESULTS.md`](docs/RESULTS.md) — every measured number and the
  run that produced it; `docs/data/` holds the raw CSVs
- [`docs/DESIGN_DECISIONS.md`](docs/DESIGN_DECISIONS.md) — problem,
  diagnosis, fix, evidence, including the things that turned out wrong

Also here: [`docs/RUNNING.md`](docs/RUNNING.md) runs the individual
subsystems on their own — teleop, mapping, standalone Nav2, MoveIt
pick-and-place — and [`CLAUDE.md`](CLAUDE.md) carries the engineering
constraints, if you intend to change the code rather than run it.

For evaluation without a browser, the mission also takes its colour from
the command line: add `target_colour:=green` to T2 and call
`ros2 service call /mission/start std_srvs/srv/Trigger`. The panel is the
product; this is the developer's door into the same machinery.
