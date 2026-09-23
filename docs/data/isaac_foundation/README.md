# Isaac Sim on the development machine — 2026-09-24

Everything here was measured on 2026-09-24 on the development machine.
The scripts are the ones that produced the numbers, **as run**. They write
to a job scratch directory (`TMP=` at their top), so change that path before
re-running them. `run*_milestones.txt` are grep extracts of each run's log;
the full logs (≈470–740 lines of Kit start-up) were not committed.

## Hardware (measured)

| | |
|---|---|
| OS | Ubuntu 24.04.4 LTS, kernel 6.8.0-139-generic, x86_64 |
| CPU | 13th Gen Intel Core i5-13420H, 8 cores / 12 threads |
| RAM | 15696 MiB (+ 4 GiB swap) |
| GPU | NVIDIA GeForce RTX 4050 Laptop, **6141 MiB**, compute 8.9; plus Intel UHD (hybrid) — Kit skips the Intel device and uses GPU 0 |
| Driver | 580.173.02 (CUDA 13.0) |
| Vulkan | `vulkaninfo` not installed; Kit reports "Graphics API: Vulkan", RTX 4050 active |
| Disk | 102 G, 38 G free at start |

## Requirement check

NVIDIA's current requirements page
(<https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html>,
updated 2026-09-18, the 6.x docs) sets the **minimum** at RTX 4080,
**16 GB VRAM, 32 GB RAM**, 50 GB SSD, Linux driver **595.58.03**.
This machine fails VRAM (6.1 of 16), RAM (15.3 of 32), the GPU class, and
the driver version. **Isaac Sim 6.1 was therefore not downloaded.** A
lighter configuration of 6.1 cannot pass the driver requirement without a
driver upgrade, and cannot pass the VRAM or RAM requirement at all.

## The existing install

`~/isaac-sim/venv` — a pip install, **Isaac Sim 4.5.0.0** (`isaacsim-*`
4.5.0.0, `omniverse-kit` 106.5.0.162521, Python 3.10.20), 5.9 G, plus
7.3 G of downloaded Kit extensions in `~/.local/share/ov/data/exts`.
The venv carries `torch 2.11.0`. Its ROS 2 bridge (`isaacsim.ros2.bridge`
4.1.15) ships **Humble** libraries only
(`exts."isaacsim.ros2.bridge".ros_distro = "humble"`: *"to fallback onto if
none were sourced"*). There is no `jazzy/` directory.

## Runs

All runs used a clean environment (`env -i HOME=… PATH=/usr/bin:/bin`), so
nothing from `~/.bashrc`, the COCO overlay or `/opt/ros` reached Isaac
unless named.

| # | Configuration | Result |
|---|---|---|
| 1 | default `SimulationApp({'headless': True})` + cube drop | no output in **900 s**; killed by `timeout` |
| 2 | same, unbuffered log | Kit "app ready" at 13.3 s; `SimulationApp()` had not returned after ≥ 20 min observed (log ends at 17.5 s) |
| 3 | same + `faulthandler` at 360 s | stack: `simulation_app.py:520 _wait_for_viewport` ← `__init__:270`. The loop is `while viewport_api.frame_info.get("viewport_handle") is None: app.update()` — **unbounded**. The renderer asked for a 7508933632 B TLAS buffer and fell back to 4287216384 B |
| 4 | `create_new_stage=False` + own `new_stage()`; physics; bridge; OmniGraph `/clock`; rclpy ping/pong; domain 77 | **startup complete 12.4 s**; **physics OK** (cube 1.000 → 0.100 m, 120 steps, 0.25 s); bundled Humble `rclpy` loaded; 10 `app.update()` took ≈401 s; `ROS2PublishClock`: *"Unable to create ROS2 node"*; Jazzy **discovered** `/coco_ping`, `/isaac_pong`; no `/clock`, no pong in 30 s; **`LLVM ERROR: out of memory`**, SIGABRT |
| 5 | as 4, render-free (`world.step(render=False)`, no `app.update()` after the bridge), rclpy publishing `/isaac/cube_z` | Jazzy **discovered** `/isaac/cube_z`, `/coco_ping`, `/isaac_pong`; **no message received** in 30 s; `LLVM ERROR: out of memory`, SIGABRT in the first loop iteration (main thread in `time.sleep`) |
| 6 | as 5 but **no bridge, no ROS environment** | **clean**: 60 s, 5114 steps, cube 3.000 → 0.100 m and resting; **peak RSS 4769124 kB**; exit 0 |

The two runs with the bridge loaded both aborted; the one without it did
not. So memory exhaustion is not the explanation — the host had ≈9 GB
free at the start of run 5 and run 6 peaked at 4.8 GB. The mechanism is
**unknown**. Runs 4–5 also set `ROS_DISTRO`, `RMW_IMPLEMENTATION` and
`LD_LIBRARY_PATH=<bridge>/humble/lib`, which run 6 did not, so the bridge
extension and its bundled libraries are **not separated** by these runs.

## Verdict

| | |
|---|---|
| Isaac Sim 6.1 installed | **no** — hardware below minimum (see above) |
| Existing 4.5.0 | **kept**, pending an owner decision (13.2 G) |
| starts | only with the viewport wait bypassed (`create_new_stage=False`) |
| renderer | initialises, never delivers a viewport frame; forced rendering ends in `LLVM ERROR: out of memory` |
| physics | **verified** headless, render-free |
| Python launcher | verified (`~/isaac-sim/venv/bin/python`) |
| ROS 2 bridge enabled | verified (bundled Humble rclpy, clean env) |
| Isaac ↔ Jazzy | **discovery verified**, **message exchange NOT verified** |
| usable as COCO's second backend here | **no** — COCO's contract needs RTX cameras and LiDAR, and those need the renderer |

## Environment invocation (the clean case)

Isaac side — nothing inherited except what is named:

```bash
BRIDGE=$HOME/isaac-sim/venv/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge
env -i HOME=$HOME PATH=/usr/bin:/bin \
    ROS_DISTRO=humble RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_DOMAIN_ID=77 LD_LIBRARY_PATH="$BRIDGE/humble/lib" \
    $HOME/isaac-sim/venv/bin/python -u your_script.py
```

```python
app = SimulationApp({'headless': True, 'create_new_stage': False})
omni.usd.get_context().new_stage()      # the wait this skips never ends here
```

Jazzy side — `/opt/ros/jazzy` only, never the COCO overlay, same private
domain:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic list --no-daemon
```

Never source `/opt/ros/jazzy` or a COCO overlay into Isaac's process: its
Python is 3.10 and Jazzy's is 3.12, and the bridge would then try the
sourced distro instead of its bundled one. Never run it on domain 0, where
the live COCO stack and an unrelated project's Gazebo publish.

## Docker

Not installed (measured: `docker: command not found`, no NVIDIA container
toolkit). Not installed by this pass: Isaac's container needs the same
16 GB VRAM, so it would not change the verdict above. Docker is worth
installing for its own reason — `docker build` of COCO's image has **never
run** (P0.2: "Docker runtime NOT VERIFIED") — as a separate task with the
owner's approval (it needs `sudo` and adds a system daemon).

## Isaac Lab

Not present (measured): no `isaaclab*` / `isaac_lab*` directory within five
levels of `~`, and no `isaaclab` package in the Isaac venv.
