# Future work / known limitations

Honest list of what's not done or not perfect, in rough priority order.
(All five roadmap layers are implemented and verified — see RUNNING.md.)

## Hardware / environment

1. **NVIDIA driver is currently not loaded** (SecureBoot + post-Windows
   state). Everything runs on the Intel iGPU at RTF ≈ 1.0, but re-verify
   sensor rates and record demo videos on the dGPU after
   `sudo modprobe nvidia` / a reboot.
2. **Install the real debs when sudo is available** —
   `ros-jazzy-moveit`, `ros-jazzy-rosbridge-suite`,
   `ros-jazzy-web-video-server` — then delete `~/ros2_ws/moveit_prefix/`
   and the prefix block in `setup_env.sh` stops mattering.

## Manipulation

3. **Fingertip lips are a sim-side fix.** The end-stop lips that make the
   grasp survive the lift are collision geometry only (the visual mesh is
   unchanged). On hardware the equivalent is a printed fingertip with a
   raised tip edge or a compliant pad. A gz `detachable_joint` "vacuum
   gripper" remains an alternative demo mode.
4. **Grasp depth is chassis-limited.** The m_link2 collision split now
   matches the real link shape, but deeper grasps stay out of reach for a
   geometric reason: poses that extend further curl the pinch point back
   toward the chassis. Extra reach needs a longer wrist link or a
   different arm mount, not better collision boxes.
5. **Gripper force control**: the fingers are position-controlled; a
   grasp is "whatever the trajectory controller holds". An effort
   interface + grasp force controller would be more realistic.

## Navigation / perception

6. **Depth camera is unused** by the nav stack: could feed
   `/camera/points` into a voxel/STVL costmap layer for 3-D obstacles.
7. **The corridor behind the ramp (x > 5.5) maps poorly** — two long
   parallel walls leave the scan matcher unconstrained along the corridor
   axis (a classic lidar-SLAM degeneracy). The mapping route deliberately
   skips it; Nav2 goals there are rejected. Fixes: add visual features to
   the east wall, or fuse wheel/GT odometry more tightly.
8. **AMCL initial pose is hardcoded to the spawn pose** — fine for the
   demo, but relocalisation from an arbitrary start isn't exercised.

## RL

9. **Train longer + domain randomization.** The PPO baseline trains and
   the learning curve is saved, but a policy that reliably climbs the
   ramp from arbitrary approach angles needs more steps plus
   randomized spawn yaw/friction. All the plumbing (fast physics,
   ground-truth rewards, Monitor CSV, checkpoints) is in place.
10. **Vision-free observations**: the policy sees pose/velocity/tilt
    only. Adding the depth camera or lidar would let it generalize to
    unseen ramp placements.

## Housekeeping

11. **CI and Dockerfile have not been executed yet** (no Docker/runner
    on this machine). Every referenced `ros-jazzy-*` apt package name
    has been verified against the live Jazzy/noble package index, and
    the workflow YAML parses, so the remaining risk is runner-side
    (setup-ros behavior, build tooling), not package naming.
12. **Launch-file integration tests** (`launch_testing`) would catch
    regressions the unit tests can't (controller activation, topic
    wiring).
13. **Demo video**: screenshots are current (docs/images), but a 30 s
    screen capture of the pick-and-place + a Nav2 run would present
    better on LinkedIn than stills.
