# Isaac Sim version compatibility on the development machine — 2026-09-24

Question asked: *can an older Isaac Sim release run the subset of Isaac Sim
COCO needs on this RTX 4050 Laptop (6 GB VRAM, ~15 GB RAM)?*

Everything below marked (measured) was produced in this session on the
development machine; (derived) is arithmetic on measured values; (docs) is
quoted from NVIDIA's archived documentation. This configuration is **below
NVIDIA's documented minimum for every release examined** — nothing here
claims it is supported. The question was empirical.

**Headline: the previous verdict (`../isaac_foundation/README.md`) that
Isaac Sim 4.5 cannot render and cannot exchange ROS 2 messages here was
wrong.** Every blocker it recorded is reproducible and has a measured cause
that is configuration or first-run behaviour, not the GPU, not RAM, and not
the release. With those causes addressed, the *existing* 4.5 install passes
the whole COCO subset at COCO's own sensor settings. See §3.

## 1. Official requirements (docs)

Sources: archived NVIDIA requirements pages (the live site now serves only
5.1.0+). 4.5.0 and 4.2.0 from their versioned URLs via the Wayback Machine
(snapshots 2026-06-08 and 2026-02-22); 2023.1.1, 4.0.0 and 4.1.0 from
`docs.omniverse.nvidia.com/isaacsim/latest/…` snapshots of 2024-02-24,
2024-06-13 and 2024-07-29, each matched to its release by the release-notes
snapshot of the same period (2024-02-25 lists 2023.1.1 newest, 2024-06-22
4.0.0, 2024-08-04 4.1.0).

| Version | RAM min | VRAM min | GPU min | Ubuntu | ROS 2 bundled | Driver (Linux, recommended) | Install |
|---|---|---|---|---|---|---|---|
| 4.5.0 | 32 GB | 8 GB | RTX 3070 | 20.04 / 22.04 | Humble only | 535.129.03 | pip (4.5.0.0), zip |
| 4.2.0 | 32 GB | 8 GB | RTX 3070 | 20.04 / 22.04 | Humble + Foxy | 535.129.03 | pip (4.2.0.1, 4.2.0.2), Launcher |
| 4.1.0 | 32 GB | 8 GB | RTX 3070 | 20.04 / 22.04 | Humble + Foxy | 535.129.03 | pip (4.1.0.0), Launcher |
| 4.0.0 | 32 GB | 8 GB | RTX 3070 | 20.04 / 22.04 | Humble + Foxy | 535.129.03 | pip (4.0.0.0), Launcher |
| 2023.1.1 | 32 GB | 8 GB | RTX 2070 | 20.04 / 22.04 | Humble + Foxy | 525.85 | Launcher, NGC container — **no pip wheel** |
| *this machine* | 15.3 GiB | 6 GB | RTX 4050 Laptop | **24.04** | COCO: **Jazzy** | 580.173.02 | |

- 4.0–4.5 have *identical* documented minimums; the older releases are not
  lighter on paper. 2023.1.1 lowers only the GPU class.
- 4.5's page adds: *"8GB VRAM is insufficient to run a complex scene
  rendering more than 16MP per frame."*
- **No release examined supports Jazzy or Ubuntu 24.04.** 4.5's ROS page:
  *"For the ROS 2 bridge, Isaac Sim is compatible with ROS 2 Humble."*
  Everything below that talks to Jazzy is therefore an **unsupported,
  experimental** configuration (Humble bridge libraries ↔ Jazzy peer).
- pip availability (measured from `https://pypi.nvidia.com/isaacsim/`):
  4.0.0.0, 4.1.0.0, 4.2.0.2, 4.5.0.0 (and 5.x/6.x). No 2023.1.1.
- 2023.1.1 is therefore **not installable here** without the retired
  Omniverse Launcher or Docker + NVIDIA container toolkit (not installed;
  needs `sudo` and a system daemon). Not attempted.

## 2. Candidate selection

Isaac Sim **4.2.0 (pip 4.2.0.2)**. The documented criteria do not separate
4.0 / 4.1 / 4.2 (same minimums, same Humble + Foxy bridge, all pip), so
none is "lighter". 4.2.0.2 is the last, patched release of the pre-4.5
line (`omni.isaac.*` API, Humble + Foxy bridge), so a result on it is the
best single stand-in for that whole line. Installed isolated at
`~/isaacsim-4.2.0-test/` (own venv, own pip cache and temp, run with
`HOME=~/isaacsim-4.2.0-test/home` so Kit's extension/shader caches never
touch 4.5's `~/.local/share/ov`). Results: §4.

## 3. Isaac Sim 4.5.0 (existing `~/isaac-sim`, unmodified) — measured

Every run: `env -i HOME PATH` plus only the variables named in that run's
`runs/<label>/env.txt`; ROS on private domain 77; `/usr/bin/time -v` for
peak RSS; `nvidia-smi` sampled at 1 Hz for VRAM (idle ≈ 255–268 MiB,
included in the peaks). Nothing was installed into or changed in
`~/isaac-sim`. Runs also wrote Kit/driver caches (`~/.local/share/ov`,
`~/isaac-sim/venv/.../omni/cache/nv_shadercache`) as any launch does.

### 3.1 The four blockers, and what they actually were

| Previous verdict | Measured cause | Evidence |
|---|---|---|
| Default start hangs forever in `_wait_for_viewport` | **First launch in a context compiles ~300 MB of RTX shaders** into the driver cache (`omni/cache/nv_shadercache/GLCache/<hash>/…bin`, 308 446 786 B written during the previous session, 308 484 010 B during this one). Cold GUI start: **385.7 s** to first viewport frame; warm: **20.7 s**. The previous session's runs were killed (900 s timeout, ≥20 min observation) before a cache persisted, so each started cold. A headless default start today returned in **21.7 s**. | `v45_gui` (killed at 420 s), `v45_gui2` (385.7 s), `v45_gui3` (20.7 s), `v45_vp_mgpu` |
| "Rendering → `LLVM ERROR: out of memory`" | Not rendering. 0 crashes in 10 render/GUI runs (one of them killed by my own 420 s timeout while compiling shaders). | `v45_nvicd*`, `v45_default`, `v45_res_*`, `v45_gui*`, `v45_vp_*` |
| Bridge → `LLVM ERROR: out of memory` | **Humble ↔ Jazzy discovery type mismatch.** gdb backtrace (`v45_gdb2`): Isaac's bundled Humble `rmw_fastrtps_cpp` takes a Jazzy participant's `ros_discovery_info` sample, `cdr_deserialize(ParticipantEntitiesInfo)` → `std::vector<rmw_dds_common::msg::Gid>::resize` with a garbage length → `operator new` fails → the process-wide new-handler installed by `omni.warp.core`'s embedded LLVM (`warp.so`) prints *"LLVM ERROR: out of memory"* and aborts. RSS was 4.8 GB with ≈9 GB free — not memory. | `v45_gdb2/milestones.txt` |
| Bridge segfault / "no message delivered" | **Enabling the bridge extension before `new_stage()`** segfaults on a TBB worker in `omni.graph.image.core` → `omni.graph.core` at the first update (gdb, `v45_gdb1`). In that order: 3 segfaults in 4 runs (the 4th, `v45_ros1b`, got past it and hit the Fast-DDS abort). Stage first: 0 segfaults in 8 bridge runs. | `v45_ros1`, `v45_ros_alone`, `v45_gdb1`, `v45_ros1b` |

Not causes (measured, both directions): the Mesa Vulkan ICDs
(`VK_ICD_FILENAMES` NVIDIA-only vs default rendered identically,
`v45_nvicd2` vs `v45_default`), and `multi_gpu` (default start returned
with it on, 21.7 s, and off, 22.5 s).

**The Gid mismatch is a property of the bundled Humble libraries, and every
release in §1 bundles Humble.** CycloneDDS on the Isaac side is not affected
in any run here. **COCO already uses CycloneDDS** (`setup_env.sh:85`,
`Dockerfile:73`), so the working configuration *is* COCO's configuration;
the previous session tested Fast-DDS.

### 3.2 Results (measured)

Sensor/ROS runs: stage 2 of `scripts/ros_probe.py` — a 5 kg rigid box
driven by Twist, a camera (RGB + depth via `ROS2CameraHelper`) and a PhysX
2-D LiDAR (`IsaacReadLidarBeams` → `ROS2PublishLaserScan`), all on the box;
Jazzy side `scripts/jazzy_side.py` (`/opt/ros/jazzy` only) publishes Twist
0.5 m/s for 8 s then 0.5 rad/s for 6 s and counts what arrives.

| Run | Config | Result |
|---|---|---|
| `v45_default` | headless render product 160×120 | first RGB after 8.4 s; 60.7 fps; RGB (120,160,4) 24 184 non-zero; depth 8 034 finite, 1.051–5.956 m; VRAM 1 795 MiB; RSS 5 102 484 kB |
| `v45_res_640x480` | 640×480 | first RGB 9.6 s; 57.7 fps; VRAM 1 923 MiB; RSS 5 109 968 kB |
| `v45_res_1280x720` | 1280×720 | first RGB 7.5 s; 49.9 fps; VRAM 2 111 MiB; RSS 5 211 932 kB |
| `v45_gui3` | GUI, 1280×720 viewport, warm cache | start 20.7 s; box 3.000 → 0.250 m; 64.4 fps; screenshot `runs/v45_gui3/gui_viewport.png`; VRAM 1 940 MiB; RSS 5 518 788 kB |
| `v45_gui2` | GUI, cold cache | start 385.7 s; **RSS 9 774 716 kB**; VRAM 1 872 MiB |
| `v45_coco_cyc_rel` | **COCO scale**: camera 320×240, LiDAR 480 beams 0.15–12 m; Cyclone ↔ Cyclone; image subs RELIABLE | Jazzy received in ≈42 s: /clock 1693, /isaac/odom 1693, /tf 1693, /isaac/rgb **1689** (320×240 rgb8), /isaac/depth **1688** (32FC1), /isaac/scan **1693** (480 beams); Twist → x 3.967 m, yaw 171.2°; median update 23.8 ms, p95 27.4 ms; VRAM 1 823 MiB; exit 0 |
| `v45_coco_mixed` | as above, Isaac Cyclone ↔ **Jazzy Fast-DDS** | all six streams (rgb 1470, depth 1455, scan 1518, clock 1480, odom 1513, tf 1516); x 4.008 m, yaw 165.9°; exit 0 |
| `v45_coco_cyc3` | as `_rel` but image subs BEST_EFFORT | depth stalled after 2 frames while rgb ran (1190); see 3.3 |
| `v45_ros1b`, `v45_gdb2` | Isaac **Fast-DDS** ↔ Jazzy Fast-DDS | 0 messages received; SIGABRT ≈2–4 s after the Jazzy node starts (the Gid mismatch) |

Twist accuracy (derived): commanded 0.5 m/s × 8 s = 4.0 m and
0.5 rad/s × 6 s = 171.9°; measured 3.967–4.008 m and 165.9–172.6° once the
contact was frictionless.

### 3.3 Things that looked like Isaac faults and were not

- **Yaw would not follow the command** (3.2–3.3°). Physics: friction
  combines by average, so an icy body on a default ground still has μ 0.25
  and cancels 0.5 rad/s within a step. `yaw_probe.py`, no ROS: default
  ground 2.21°, both icy 85.83° after 180 updates (0.5 rad/s × 3 s =
  85.9°, derived), in the air 73.91°. A probe artefact of using a sliding
  box as a "robot"; a real port uses an articulation + differential
  controller.
- **One large image stream stalls under BEST_EFFORT.** This machine's
  `net.core.rmem_max` is **212 992 B**, smaller than one 320×240 depth frame
  (307 200 B), so fragments are dropped; which of RGB/depth starved varied
  run to run. RELIABLE subscriptions delivered both (Isaac's camera
  publisher is reliable). This is a host-socket property that applies to
  any DDS stack on this machine, not an Isaac property; the sysctl was
  **not** changed.
- **My first depth subscriber** walked 76 800 floats in pure Python per
  frame and starved its own executor; replaced with numpy.

## 4. Isaac Sim 4.2.0 (pip 4.2.0.2, `~/isaacsim-4.2.0-test`)

Not yet measured.

## 5. Reproduce

```bash
# render (headless)
bash scripts/run_probe.sh <label> ~/isaac-sim/venv/bin/python scripts/render_probe.py 600 \
     OMNI_KIT_ACCEPT_EULA=YES PROBE_RES=320x240 PROBE_FRAMES=100
# ROS 2, COCO scale, COCO's RMW
IMG_RELIABLE=1 ISAAC_RMW=rmw_cyclonedds_cpp JAZZY_RMW=rmw_cyclonedds_cpp \
  bash scripts/ros_run.sh <label> ~/isaac-sim/venv/bin/python \
  ~/isaac-sim/venv/lib/python3.10/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib \
  2 60 PROBE_RES=320x240 PROBE_LIDAR_RES=0.75
```

The scripts write under a `BASE=` job scratch path at their top; change it
before re-running. Isaac side sees only `HOME PATH OMNI_KIT_ACCEPT_EULA
ROS_DISTRO=humble RMW_IMPLEMENTATION ROS_DOMAIN_ID=77
LD_LIBRARY_PATH=<bridge>/humble/lib` and the `PROBE_*` knobs; `AMENT_PREFIX_PATH`,
`COLCON_PREFIX_PATH`, `PYTHONPATH` are unset (see each `env.txt`). The
Jazzy side sources `/opt/ros/jazzy/setup.bash` only (see
`jazzy_env_filtered.txt`) — never a COCO overlay.

Launch-order rules that the working runs depend on:
1. `SimulationApp({'headless': True, 'create_new_stage': False})`, then
   `new_stage()`, **then** enable `isaacsim.ros2.bridge`.
2. `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` on the Isaac side.
3. Expect the first launch in a new context (headless/GUI, new install) to
   spend minutes compiling shaders; do not kill it before the first frame.
