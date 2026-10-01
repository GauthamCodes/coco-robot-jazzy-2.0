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

8b. ~~**The RL policy has no closed-loop lateral control**~~ — **fixed in
   M6, and the diagnosis in this item was wrong.** The +0.61 m (M4) and
   +0.59 m (M5) were real, but they were not the policy steering badly.
   Teleported to the pre-ramp pose at exactly yaw 0 the bare policy climbs
   2.5 m with **+0.03 m** of drift in every lane. It holds a line; what it
   cannot do is *correct* one — and `nav2_params.yaml` allows a Nav2 leg
   to finish 0.25 rad off heading, which over 2.5 m of climb **is** 0.64 m
   of lateral.

   That made it a safety defect rather than an accuracy one: open loop
   from a Nav2-legal heading error, both outer-lane adverse cases finished
   within 70 mm of a platform edge at ±1.25 m, and neither summited.

   `ramp_driver.lateral_hold` (a clamped cross-track + heading correction
   on the policy's yaw action) takes the worst case to **0.053 m** with no
   retraining and 8/8 summits. The gains were swept rather than guessed,
   and the clamp sweep is what proved the limit was bandwidth and not
   authority — numbers in [RESULTS.md](RESULTS.md#the-lane-hold-and-why-the-gains-are-what-they-are).

   **`--randomize` is still worth doing, but for its own reasons** (item
   9a), and it is *not* the free change this item used to claim: `reward.py`
   has no lateral or heading term at all, so a spawn-randomised run would
   give PPO no gradient toward the lane. It would need a reward change
   too, which invalidates the published 10/10 curve.

## Navigation / perception

6. **Depth camera is unused** by the nav stack: could feed
   `/camera/points` into a voxel/STVL costmap layer for 3-D obstacles.
7. **The corridor behind the ramp (x > 5.5) maps poorly** — two long
   parallel walls leave the scan matcher unconstrained along the corridor
   axis (a classic lidar-SLAM degeneracy). The mapping route deliberately
   skips it; Nav2 goals there are rejected. Fixes: add visual features to
   the east wall, or fuse wheel/GT odometry more tightly.
7b. **That unmapped corridor now has a measured cost, and it is a mission
   failure.** The fetch mission's descent ends at world x ≈ 6.65, inside
   it. With nothing to scan-match against, AMCL dead-reckons on skid-steer
   wheel odometry. Over the 20-run matrix the AMCL-vs-ground-truth gap at
   the end of the descent was 0.119–1.183 m (mean 0.378). Every run at
   ≤ 0.470066 m drove home — though that is the largest gap among the
   successes, so the threshold is only bracketed to (0.470, 1.183) m with
   nothing sampled between. The one run at 1.183 m could not — Nav2 planned
   from an estimate 3.4 m from the truth, no part of the global plan fell
   in the lidar-built local costmap, DWB scored 0 of 819 trajectories and
   `bt_navigator` aborted in 1.7 s. That is **1/20 of the whole mission
   lost to item 7**, after the object had been successfully picked. The
   cheapest fix is probably to map the corridor rather than to tune
   AMCL. Numbers in [RESULTS.md](RESULTS.md#the-one-failure-run-15-and-it-is-a-localisation-failure).

8. **AMCL initial pose is hardcoded to the spawn pose** — fine for the
   demo, but relocalisation from an arbitrary start isn't exercised.

8b. **Nav2 finishes legs outside its own `yaw_goal_tolerance`, and nobody
   knows why yet.** The pre-ramp leg is sent with `orientation.w = 1.0`
   and `SimpleGoalChecker` is configured `yaw_goal_tolerance: 0.25`, but
   over 20 runs the robot came to rest at |yaw| = 0.104–0.472 rad, outside
   tolerance in **14 of 20**. It is a settled pose, not a transient (yaw
   unchanged over the next second in all 20; stationary for the prior 2 s
   in 12 of the 14). It is **not** AMCL error — estimate and ground truth
   agree to 0.076 rad mean and disagree about compliance in 1 run of 20.
   `stateful: true` does not explain it either: that latches only the xy
   check. Candidates not yet tested: the BT terminating on a condition
   other than the controller's goal checker, or the progress checker
   ending the final rotation. This matters because the RL climb inherits
   that heading — see the lane-hold envelope in
   [RESULTS.md](RESULTS.md#the-lane-holds-envelope-is-wider-than-0053-m-and-here-is-why).

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
   (a) **`--randomize`, plus a reward that pays for it.** The solved task has a
   fixed spawn, so a constant action also solves it. Randomising spawn offset
   (±0.5 m) and yaw (±0.4 rad) would force the policy to use `y` and
   `sin/cos yaw` to steer, turning "learned a fixed motion" into "learned a
   closed-loop controller". It is NOT zero new code, which this said until M6
   measured it: `reward.py` has no lateral or heading term, so randomising the
   spawn alone gives PPO nothing to descend toward. Adding one changes the
   reward function the published 10/10 curve was produced under. The mission
   no longer needs this — `ramp_driver.lateral_hold` holds the lane to 0.053 m
   without it (item 8b) — so it is now a research question about the policy
   rather than a blocker.

   **Confirmed by measurement, and it is the policy.** The 20-run matrix
   drifted +y in all 20 runs with only r² = 0.32 explained by entry
   heading — a constant offset, not a correction failure. Two experiments
   on the same flat lane separate the candidates. Open loop (constant
   `linear.x`, `angular.z` = 0, no policy) over **10.05 m × 3 trials:
   lateral +0.0000 m, yaw change 0.00000 rad** — the machine drives
   straight, so it is not track width, wheel radius or the controller. The
   bare policy over the same lane: **+0.3115 / +0.3107 m over 6.13 m**
   (≈ +50.8 mm/m). Teleported to the pre-ramp pose at exactly yaw 0, the
   climb drifts **+0.0452 / +0.0452 / +0.0438 / +0.0438 m** in lanes
   +0.75 / +0.25 / −0.25 / −0.75 — same sign, same magnitude, both sides
   of the centreline. **The bias follows the robot, not the lane**, and it
   is not kinematic. That is precisely what "a constant action also solves
   it" predicts, and it is the concrete argument for M7's reward carrying
   cross-track and heading terms.

   **It is not, however, the explanation for the mission drift, and an
   earlier version of this note implied it was.** The confirmed constant
   bias is **+0.045 m**; the worst mission cross-track is **+0.301 m**,
   roughly **6.7× larger**. So the policy bias accounts for about **15 %**
   of it. Entry heading covers some further part (r² = 0.32 against
   drift), and **the majority remains unexplained** — the arrival offset
   inherited from the Nav2 leg (up to +0.158 m at the ramp foot) is the
   next candidate and has not been separated out. Numbers in
   [RESULTS.md](RESULTS.md#there-is-a-real-constant-y-policy-bias--and-it-explains-15--of-the-drift).
   Also unexplained: the bias rate is ~2.5× larger on the flat than on the
   grade.
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

10b. **`gazebo_models/scripts/` and `coco_moveit_config/scripts/` have no
    linters.** Every other package runs `ament_flake8` + `ament_pep257` in
    CI; these two do not, because they are `ament_cmake` packages whose
    Python lives under `scripts/` rather than in a package directory. The
    genuine defects M6 found there are fixed (two unused imports, an
    E305, four over-length lines, a nested-quote escape and two missing
    class newlines), but ~118 docstring and import-order findings remain.
    Adding the two tests is a mechanical sweep across nine files and is
    better done on its own than folded into a feature branch — it would
    bury a real change in 118 lines of reflow.

    Worth doing, because the one defect that mattered was invisible for
    exactly this reason: `colcon test --packages-select coco_moveit_config`
    had been failing since before M5 (a module-level `importorskip`
    aborting the whole collection, so five tests never ran and pytest's
    exit 5 came back as a failed package) and nothing was watching.

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
