# Future work / known limitations

Honest list of what's not done or not perfect, in rough priority order.
All five roadmap layers are implemented; layers 1–4 are verified
end-to-end (see RUNNING.md), and layer 5's *infrastructure* is verified.
Layer 5's RL is now **solved**: after the original 0/10 was traced to an
unclimbable mesh, an unreachable goal and a real-time-factor flag that was
corrupting the control loop, the trained policy scores **10/10 at both 18° and
24°** — see item 9.

## Hardware / environment

1. ~~**NVIDIA driver is not loaded**~~ — **fixed 2026-07-20.** The DKMS
   module was built and signed all along; SecureBoot was rejecting it
   because only Canonical's CA was enrolled, not the machine-owner key,
   so `modprobe nvidia` failed with `Operation not permitted`. Fixed with
   `sudo mokutil --import /var/lib/shim-signed/mok/MOK.der` + enrolling
   at the MOK Manager screen on reboot. If it recurs after a kernel or
   driver update, re-enroll rather than rebuilding DKMS; diagnose with
   `mokutil --sb-state`, `mokutil --list-enrolled`, and
   `modinfo <module> | grep signer`. The full stack (sim, verify_sim,
   pick-and-place) has been re-verified on the dGPU.
2. **Install the real debs when sudo is available** —
   `ros-jazzy-moveit`, `ros-jazzy-rosbridge-suite`,
   `ros-jazzy-web-video-server` — then delete `~/ros2_ws/moveit_prefix/`
   and the prefix block in `setup_env.sh` stops mattering.

## Manipulation

3. **The grasp is now a `detachable_joint` magnet — DONE, and it is what
   ships.** The friction pinch was measured not to generalise (item 5b),
   so `pick_place.py` welds the target to the palm on command instead.
   The fingers still close, and their stall position is still logged as
   corroborating evidence, but the weld is the grasp. Numbers in
   `docs/RESULTS.md`.

   Two behaviours of the gz plugin are worth remembering before reusing
   it, both measured rather than read: it attaches its child **the
   instant the model spawns**, not when commanded, and it binds to that
   entity **once** — remove and re-spawn the model and a later attach
   still answers `"attached"` while welding nothing. So the demo detaches
   right after spawning, wants a fresh simulator per run, and confirms
   the grasp by reading the target's height out of Gazebo rather than by
   believing the plugin's state topic.

   The fingertip end-stop lips remain in the collision geometry. They are
   no longer load-bearing for the demo, but they are what a hardware
   version would need — a printed fingertip with a raised tip edge or a
   compliant pad — if the magnet is ever swapped back out for real
   friction.
4. **Grasp depth is chassis-limited.** The m_link2 collision split now
   matches the real link shape, but deeper grasps stay out of reach for a
   geometric reason: poses that extend further curl the pinch point back
   toward the chassis. Extra reach needs a longer wrist link or a
   different arm mount, not better collision boxes.
5. **Gripper force control**: the fingers are position-controlled; a
   grasp is "whatever the trajectory controller holds". An effort
   interface + grasp force controller would be more realistic.
5b. **`--target` re-targeting: the grasp is fixed, the pedestal is what
   is left.** This was 0/5 with the friction pinch, every failure being
   fingers closing on nothing. With the magnet (item 3) that failure mode
   is gone entirely and the score is 5/14 across the four
   `docs/RESULTS.md` points and a ±9 mm box, with every completed run
   lifting the cylinder a measured 32–40 mm.

   The remaining 7 failures are **motion planning, not grasping**, and
   they split cleanly on x: every point at x ≥ 0.1505 completes, every
   point at x ≤ 0.1468 is rejected with MoveItErrorCode 99999. move_group
   says why — the goal satisfies both joint constraints but the palm
   (`m_link3`) is in contact with the `pedestal` collision object, so
   RRTConnect cannot sample a valid goal state. The arm has no wrist, so
   a grasp point closer to the robot is only reachable by curling the
   forearm back over the 50 mm pedestal box. This is item 4 wearing a
   different hat. Pedestal height is not the driver; x is.

   **Resolved in M5, and not the way this said.** The claim above — that
   the fetch mission has no pedestal, so the obstruction should not apply
   — was true and irrelevant. The obstruction did not apply because the
   grasp could not happen at all. Measured against `arm_ik.ik()`: a 60 mm
   cylinder standing on the platform grasps at base-z 0.030, where the
   arm reaches only to base-x 0.1299, while the chassis collision box
   ends at 0.120. The approach window `[0.120 + radius, 0.1299]` came out
   at +3.9 mm for the 12 mm target, +0.9 mm for the 18 mm one, and
   **negative for the 24 mm and 30 mm ones** — two of the mission's four
   objects were geometrically impossible to pick up, and nothing in the
   world, the camera or the planner would have said so before the last
   step of the mission.

   The demo only ever worked because its 98 mm pedestal lifted the target
   to z=0.128, where reach is 0.1608. The targets are now 158 mm-tall
   cylinders standing directly on the platform, so the grasp band lands
   at that same verified height with no pedestal in the scene: every
   window is ~27 mm, and the palm-vs-pedestal collision above simply has
   no object to collide with. A plinth would have reinstated it — and
   would have parked a static obstacle in the lane the robot drives
   through on the up-over-down descent, where a cylinder leaves with the
   robot. `coco_config/test/test_reach.py` pins all of it.

   Still open for the *demo*'s own window (it keeps its pedestal): a
   shorter, wider plinth, or deriving the approach direction from the
   target instead of reusing a fixed hover-then-descend.

   Note the original 0/5 was only visible once `move_gripper` started
   confirming the grasp against `/joint_states`; the timed version before
   it reported all five as successes. The same lesson applied again this
   round, which is why the magnet is not trusted to self-report either.

## Reinforcement learning / mission

8b. **The RL policy has no closed-loop lateral control, and it is now the
   mission's blocking defect.** Measured twice: +0.61 m of drift over the
   climb in M4, and +0.59 m in the M5 end-to-end run. The four target
   lanes are 0.5 m apart, so a robot sent to lane +0.25 arrives at +0.84
   — in the *next* lane. In the M5 run it did more than miss: it drove
   into `target_yellow` and knocked it flat.

   `target_finder` now reports this rather than leaving it as a mystery
   (`sel=blue found=0 seen=yellow`), but reporting is not fixing. M6
   (grasp and carry) cannot be attempted until the policy holds a lane.

   The cause is understood: the policy trained from a *fixed* spawn where
   a near-constant action solves the task, so it never had to learn to
   steer. `ramp_env` already carries y and sin/cos yaw in the
   observation, so the fix needs **no new code** — a `--randomize`
   retraining run (spawn ±0.5 m, ±0.4 rad), which is hours of unattended
   compute.

   A cheaper candidate worth evaluating first: a lateral-hold outer loop
   around the policy, correcting yaw toward the lane centreline the way
   `ramp_driver.descend_cmd` already does for heading on the down-slope.
   No training, unit-testable, and the descent controller is the evidence
   that a heading-hold works on a grade. The risk is that it perturbs the
   action distribution the policy was trained on.

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

9. ~~**Train the rebuilt curriculum**~~ — **SOLVED 2026-07-27. 10/10.**
   A five-stage curriculum (start line back, then grade up: 12° from +2.5 m,
   +1.0 m, 0 m, then 18°, then 24°) scores **10/10 deterministic at both 18°
   and 24°** on the full 5.2 m task — 126–127 steps, returns 69.5–69.9.
   Numbers in [RESULTS.md](RESULTS.md#reinforcement-learning).

   Three bugs stood between 0/10 and 10/10, each hiding the next:
   an unclimbable mesh; a goal sitting 1.6 cm *beyond* the tip-over terminator
   (a completed climb at x=5.4838 logged as a fall against a 5.5 m goal — hence
   1 goal in 1,399 episodes); and **`--fast`**, which unlocks the real-time
   factor so sim time outruns wall-clock ROS delivery, making the
   `diff_drive_controller`'s 0.5 s watchdog repeatedly halt the wheels. With
   `--fast`: 531/533 tipped, eval 0/10. Without: 0/533 tipped, eval 10/10 — and
   *faster* (8.7 vs 8.2 steps/s), because physics was never the bottleneck.

   **Remaining RL work, in order of value:**
   (a) **`--randomize`** — the solved task has a fixed spawn, so a constant
   action also solves it. Randomising spawn offset (±0.5 m) and yaw (±0.4 rad)
   forces the policy to actually use `y` and `sin/cos yaw` to steer onto the
   ramp, which no open-loop sequence can do. Zero new code; the observation
   already carries those terms. This is what turns "learned a fixed motion" into
   "learned a closed-loop controller".
   (b) The 12° full-distance stage still evaluates 0/10 on its own — a greedy
   stall at 4.34 m, reproducible to within 0.02 of return. Later stages fixed it,
   but the stall itself is unexplained. `ramp_env.py:110` records 0.10 m/s timing
   out at x=4.38 and 0.17 m/s finishing 2/2, so `MIN_LIN = 0.15` sitting between
   them is the leading suspect.
   (c) Vectorise across headless gz instances if longer runs are ever needed —
   ~8.6 env-steps/s is the ceiling today.

10. **Vision-free observations**: the policy sees pose/velocity/tilt
    only. Adding the depth camera or lidar would let it generalize to
    unseen ramp placements.

## Housekeeping

11. **CI and Dockerfile have not been executed yet** (no Docker/runner
    on this machine). Every referenced `ros-jazzy-*` apt package name
    has been verified against the live Jazzy/noble package index, the
    workflow YAML parses, and the test-count guard was validated against
    real result files locally — so the remaining risk is runner-side
    (setup-ros behavior, build tooling), not package naming. **Expect
    the first CI run to need a fix or two**: the coco_rl tests have
    genuinely never executed in CI before (they were skipping silently
    on a missing gymnasium), and red_ball_nav's workflow builds
    TurtleBot3 from source, which is the most likely thing to break.
12. ~~**Launch-file integration tests**~~ — **added.**
    `gazebo_models/test_integration/test_sim_bringup.launch.py` starts the
    headless sim and asserts all four controllers reach `active`, every
    load-bearing topic publishes at its sim-time rate, and TF resolves
    `base_footprint -> base_link`. It reuses `verify_sim.run_checks()` so
    the test and the operator tool can't drift. Off by default (it needs
    Gazebo and takes ~20 s); enable with
    `--cmake-args -DBUILD_SIM_INTEGRATION_TESTS=ON`. **Still only one
    scenario** — a Nav2 goal and a full pick-and-place would be the
    natural next ones, but both are minutes long and better suited to a
    nightly job than to per-push CI.
13. **Demo video**: screenshots are current (docs/images) and the
    pick-and-place has an animated GIF, but a 30 s screen capture of a
    Nav2 run would present better on LinkedIn than stills. Now worth
    re-recording on the dGPU (item 1).
14. **`red_ball_nav`'s demo video lives in git history.** The 52 MB
    `media/demo.webm` was removed from the working tree and moved to a
    release asset, so fresh clones no longer pay for it twice — but the
    blob is still reachable in history, and only a `git-filter-repo`
    rewrite would remove it. Not worth rewriting a published repo for.
