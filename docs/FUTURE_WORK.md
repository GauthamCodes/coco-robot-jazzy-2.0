# Future work / known limitations

Honest list of what's not done or not perfect, in rough priority order.
All five roadmap layers are implemented; layers 1–4 are verified
end-to-end (see RUNNING.md), and layer 5's *infrastructure* is verified.
Layer 5's RL ramp was rebuilt after the original 0/10 was traced to an
unclimbable ramp mesh; the robot now climbs, and what remains is training the
curriculum — item 9.

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
5b. **`--target` re-targeting does not generalise — measured 0/5.** The
   analytic IK re-solves correctly for nearby grasp points and the
   pedestal is re-sized to match, but the demo then fails: the fingers
   close on nothing, and in three of four cases the approach knocked the
   cylinder off the pedestal first. Only the tuned point (0.152, 0.128)
   works, and it works 4/4. Reachable IK is necessary but not
   sufficient — the approach path, the re-placed pedestal height and the
   fingertip lip geometry all have to agree. Fixing it properly means
   deriving the approach direction from the target rather than reusing a
   fixed hover-then-descend, and re-checking the lip cage against the
   cylinder at each height. Numbers in `docs/RESULTS.md`. Note this was
   only visible once `move_gripper` started confirming the grasp against
   `/joint_states`; the previous timed version reported all five as
   successes.

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

9. **Train the rebuilt curriculum — this is the biggest open item.**
   The original policy scored **0/10**, and that was traced to the
   environment, not the training: the shipped ramp mesh had a ~66°
   near-vertical face (unclimbable by anything on wheels) and the goal only
   reached the ramp foot — see
   [DESIGN_DECISIONS.md](DESIGN_DECISIONS.md#diagnosing-and-replacing-the-unclimbable-ramp).
   That is now rebuilt and **verified climbable**: `climb_check.py` drives the
   robot to the summit at a measured 18.1° pitch on the 18° wedge, reproducibly.
   The 2048-step PPO smoke runs only prove the pipeline runs against the new
   environment — their returns still sit in the old band and vary between
   identically-seeded runs (Gazebo is not bit-reproducible), so they are *not*
   evidence of learning. What remains is **running the full 12→18→24°
   curriculum** (phases transfer via `--resume`) and reporting a real success
   rate. All the plumbing is done
   and verified (fast physics, ground-truth rewards, Monitor CSV, checkpoints,
   `--resume`, `--ramp-angle`, `--randomize`, `evaluate.py`, `plot_curve.py`).
   Compute is still the ceiling: the sim runs ~1–8 env steps/s, so a full
   curriculum is hours-to-days of wall clock. Realistic paths:
   (a) vectorize across several headless gz instances, (b) rent a
   GPU/many-core box for one long run, (c) shape the reward more
   densely (current one is progress − tilt; a heading term and a
   ramp-contact bonus would likely help), or (d) shorten the episode
   horizon so credit assignment is easier. Long runs need `nohup` + a
   checkpoint interval well under the run length (checkpoints every 25k saved
   the usable model previously; two early runs were lost to session
   interruptions before saving).
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
