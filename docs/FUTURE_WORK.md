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

3. **Grasp retention**: the rigid 7 mm CAD fingers pinch and drag the
   cylinder but drop it partway through the lift arc. Options:
   (a) redesign fingertips with a concave/compliant pad (best),
   (b) gz `detachable_joint` "vacuum grasp" hack for demo purposes,
   (c) slower, straighter lift trajectory (already partially done).
4. **Grasp depth is chassis-limited** at shoulder/elbow ≈ [0.30, 0.58]
   because the m_link2 collision box is its mesh *bounding box* (50 mm
   tall for a mostly 15 mm link). Splitting it into 2 tighter boxes
   (slim arm + motor block) would unlock several cm of extra reach.
5. **Pose-goal IK**: planning is joint-space (2 positional DOF).
   A tiny analytic 2-link IK helper would let `pick_place.py` take
   Cartesian targets instead of joint tuples.

## Navigation / perception

6. **Map coverage**: the shipped map has unobserved wedges (240° lidar +
   short mapping run). A longer teleop mapping session would let Nav2
   accept goals anywhere in the arena, including near the ramp.
7. **Ramp is not in the map/costmap** — mapping ran before the ramp area
   was explored. Map the east half if you want Nav2 goals near the ramp.
8. **Depth camera is unused** by the nav stack: could feed
   `/camera/points` into a voxel/STVL costmap layer for 3-D obstacles.

## RL (Layer 5 is a verified scaffold, not a trained policy)

9. **Train the policy**: `python3 -m coco_rl.train_ppo --steps 200000`
   overnight, then plot the Monitor CSV learning curve and record
   before/after rollouts. Consider domain randomization (spawn yaw,
   friction) once the baseline learns.
10. **Ground-truth pose for reward**: the env uses wheel odometry, which
    under-reads on the ramp slope. Bridging Gazebo's pose topic (or an
    odometry-publisher plugin) would give cleaner rewards.
11. **Faster-than-realtime training**: run gz-sim with a higher physics
    RTF cap (`<real_time_factor>0</real_time_factor>` unlocks it) and
    drive the env off sim time to cut wall-clock training time.

## Housekeeping

12. **CI and Dockerfile are updated to Jazzy but untested** — first push
    will tell; expect minor apt-name fixes.
13. **Demo videos/screenshots**: `docs/images/*` still show the old
    Gazebo Classic build — re-capture on the new stack (SLAM map growth,
    web panel on a phone, MoveIt pick sequence with the GUI).
14. **Unit tests** exist only for teleop; the pick/place waypoints and
    the RL env reward math are good candidates for pytest.
