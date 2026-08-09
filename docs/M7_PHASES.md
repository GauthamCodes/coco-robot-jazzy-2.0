# M7 phases — paste one block per session

Standing context lives in `CLAUDE.md` at the repo root and is read
automatically. **Do not paste it.** Do not paste this whole file either —
copy exactly one fenced block below, one session at a time.

Order matters. Phase 3 before Phase 4 is not negotiable; the reason is in the
block.

Spec: `docs/M7_DESIGN.md`.

---

## Phase 0 — close M6 first

```
Do not begin M7. First close M6.

1. Run one full fetch for blue on the existing v1 wedge world, fresh
   simulator, with the corrected approach window [0.1510, 0.1565].

     T1: ros2 launch gazebo_models full_world_robo.launch.py traverse:=true gui:=false
     T2: ros2 launch coco_mission mission.launch.py policy:=<shipped policy>
     T4: ros2 run gazebo_models traverse_demo.py --colour blue

2. Report the base-x actually achieved by the creep phase, whether
   /grasp/pick planned, and the furthest phase reached.

3. If it fails, diagnose from /approach/status and /grasp/status. Do NOT
   change the window without showing me the numbers first — the last change
   to it was made without a run behind it and that is what left M6 open.

4. Once it passes, write the result into docs/RESULTS.md marked (measured),
   and push all outstanding commits to origin.

Report back before doing anything else.
```

---

## Phase 0.5 — consolidate M6

```
M6 completed once (blue, 1/1). Turn that into a measured result, push the
work, and clear three loose ends. No new features. No M7 work.

1. Push. I have run `gh auth login` on this machine. Commit the untracked
   CLAUDE.md, docs/M7_DESIGN.md and docs/M7_PHASES.md, then:
     git push -u origin jazzy-harmonic-port
   New branch, main untouched, no force. Do NOT commit
   README_BANNER_snippet.md — it is paste-and-delete, add it to .gitignore
   or delete it. Confirm the branch is visible on origin before continuing.

2. Fix the two reporting defects in the M6 write-up:
   (a) The "+4.8 mm above GRASP_SELF_COLLISION_X" figure uses the ground
       truth 0.1548 while the surrounding text uses the reported 0.1544,
       which gives +4.4 mm. State both, each labelled with its source.
   (b) Identify the arbiter warning at the end of the mode trace. Quote it
       verbatim. If two sources published to the controller simultaneously
       that is a safety defect and I want to know now — report it, do not
       fix it in this session.

3. Run the fetch matrix: 5 runs per colour, all four colours, 20 total.
   Fresh simulator every run — the DetachableJoint binds once and a reused
   sim welds nothing while reporting success. Headless, never --fast.
   Script it so it is unattended, and log per run:
     colour, base-x reported, base-x ground truth, in-window (y/n),
     grasp outcome, lift height, place outcome, lateral drift at summit,
     total duration, phase reached
   Report the matrix and every failure with its phase and cause. Write it
   into docs/RESULTS.md marked (measured), replacing the 1/1 note.

4. Investigate the lateral drift. This run summited at +0.09 m, but
   RESULTS.md documents the lane hold at a 0.053 m worst case over 8/8.
   Using the 20 runs from step 3, report the drift distribution: min, max,
   mean, and whether it correlates with colour or lane. My hypothesis is
   that entering the ramp from a real Nav2 leg gives a wider initial
   heading error than the teleported sweep did. Test it, do not assume it.
   If the envelope is genuinely wider, amend the RESULTS.md figure and say
   why — do not touch the gains.

5. Record a demo video: one continuous uncut fetch, split-screen sim and
   RViz, 60-90 s. Do not commit the file to git — red_ball_nav's 52 MB
   demo.webm is still reachable in history and that mistake is not being
   repeated. Release asset only.

Deliverables: branch on origin, the 20-run matrix in RESULTS.md, the drift
distribution, the arbiter warning identified, video as a release asset, a
SESSION_LOG.md checkpoint. Then M7 Phase 1.
```

---

## Phase 1 — MuJoCo throughput baseline (M7.0)

```
Goal: establish whether MuJoCo is actually faster here, with a number.

1. Create package coco_sim (ament_python). It generates an MJCF model of the
   robot from coco_config constants. No hand-written geometry parameters —
   every dimension traces to coco_config.

2. Write coco_rl/coco_rl/mujoco_env.py: a Gymnasium env wrapping that model
   on a flat plane. Same observation and action shapes as the existing
   ramp_env for now. Zero rclpy imports. Add a test asserting the module
   imports cleanly with rclpy absent from sys.modules.

3. Measure raw throughput: steps/s with 1, 4, 8 and 12 SubprocVecEnv workers.
   Report the table. Do not assume a speedup — measure it. The Gazebo figure
   to beat is 8.7 steps/s. If MuJoCo is not meaningfully faster, STOP and
   tell me, because the rest of M7 depends on this.

4. Fidelity check: drive an identical open-loop command sequence in MuJoCo
   and in Gazebo from the same initial pose. Report trajectory divergence
   over 10 s. State the divergence; do not tune it yet.

Deliverables: throughput table, divergence number, tests passing. Both into
docs/RESULTS.md marked (measured).
```

---

## Phase 2 — The Yard, both simulators (M7.1)

```
Goal: one terrain, two simulators, generated from one parameter file.

Build the world in docs/M7_DESIGN.md §2: flat apron, three ascent routes onto
a platform, obstacle deck, loaded descent.

1. Write coco_sim/worlds/yard_params.yaml holding every geometric and
   physical parameter: per-route grade, run, width, camber, friction range,
   heightfield roughness, curb height, washboard amplitude and wavelength,
   bridge width, platform height. One file, both simulators read it.

2. Generate the MJCF world from it — hfield for the rubble and washboard,
   geoms for ramps and bridge.

3. Generate the Gazebo SDF from the same file. Leave the v1 wedge world
   untouched as world_v1. This is an addition, not a replacement.

4. Parity test: sample terrain height at ~200 points across both worlds and
   assert agreement. Report the tolerance you ACHIEVED, rather than picking
   one that passes.

5. Implement per-episode randomisation from §2.5 in the MuJoCo env, including
   initial yaw sampled across ±0.25 rad — the full Nav2 yaw_goal_tolerance.
   Seed it; make it reproducible.

Do not train anything in this phase.
```

---

## Phase 3 — Classical baselines (M7.2) — BEFORE any policy

```
Goal: build the controllers that could prove the policy unnecessary.

Build these BEFORE training anything. Build them after and you will
unconsciously tune them to lose. That is the only reason this phase is
ordered where it is.

Implement three baselines against the Yard, in coco_rl/baselines/:

  B0  open-loop constant throttle
  B1  the current lateral_hold PD with its existing global gains
      (LATERAL_GAIN 3.0, HEADING_GAIN 2.5, LATERAL_CLAMP 0.8), unchanged
  B2  gain-scheduled PD, retuned per route, GIVEN PRIVILEGED ACCESS to the
      true grade and true friction of the current episode

B2 is the honest strong baseline and it gets information the policy will
never get. Tune it properly — spend real effort. A weak B2 makes the entire
M8 result worthless. The existing gain sweep in RESULTS.md is your starting
point; note that past 3.0/2.5 the error changes sign rather than shrinking.

Run each baseline over >=100 randomised episodes per route. Produce a matrix:
success rate, mean traverse time, mean cross-track error, and failure mode
breakdown (tipped / slid back / high-centred / fell off / timed out).

Write it into docs/RESULTS.md marked (measured). Then tell me plainly which
of the five claims in M7_DESIGN.md §3 the baselines already refute. If B2
handles the camber fine, say so — that changes what M8 should be.
```

---

## Phase 4 — Policy training (M8.0, M8.1)

```
Goal: a policy that beats B2, or an honest report that it does not.

1. Rewrite the reward per M7_DESIGN.md §4.3. Log every term separately to
   TensorBoard as its own scalar. v1 demonstrated that three bugs can mask
   each other when only the total is watched.

2. Extend the observation to the 132-dim space in §4.1: proprioception,
   10-frame history, 11x7 height scan, goal. Test the shape, and test that
   the history buffer resets on episode reset.

3. Train per-route policies first (M8.0), one route at a time. For each,
   report the same matrix the baselines produced, on the same episode count,
   same seeds where possible.

4. Only once all three beat their B2, train the unified policy (M8.1) with a
   curriculum. Write the curriculum stages down BEFORE running: what each
   stage varies, and the promotion criterion.

5. After every run report: total steps, wall-clock time, final success rate,
   per-term reward breakdown at convergence.

If a policy does not beat B2, do NOT retune the baseline to make it lose.
Report it.
```

---

## Phase 5 — Sim-to-sim transfer (M8.2)

```
Goal: the headline number.

1. Take the trained policy unchanged — same SB3 .zip — and load it in the
   existing ramp_driver against the Gazebo Yard. Do not retrain, do not
   fine-tune, do not edit the policy file.

2. Run >=30 episodes per route in Gazebo. Fill in the transfer table in
   M7_DESIGN.md §5.3: MuJoCo success, Gazebo success, gap.

3. Report the gap whatever it is. A large gap is a finding, not a failure.
   Analyse where it comes from: which route, which failure mode, whether it
   correlates with sampled friction or roughness.

4. ONLY after reporting: attempt contact calibration per §5.3 step 3 —
   fit MuJoCo solref/solimp/friction to minimise divergence against a
   measured Gazebo rollout. Re-run the transfer table. Report BOTH the
   before and after numbers.
```

---

## Phase 6 — Mission integration (M9.0)

> **Four additions specced in M7_DESIGN §2.7 (spec only, nothing built).**
> In priority order: **(1) EKF sensor fusion** (`robot_localization`,
> wheel odometry + IMU) — motivated by run 15 of the Phase 0.5 matrix,
> which lost the mission after a successful pick when AMCL drifted 3.4 m
> and DWB scored 0 of 819 trajectories; judged on the descent-end AMCL
> gap, currently 0.119–1.183 m. **(2) A VLM task interface** above the
> sequencer — open-vocabulary target selection, and grasp verification
> from the camera frame to replace `check_lifted`'s read of gz ground
> truth. **(3) MPPI vs DWB** with a comparison table. **(4) Residual RL**
> for the B2 / B2+residual / policy-alone ablation.
>
> Explicitly excluded, so they are not revisited by default: a **VLA on
> Coco** (wrong embodiment — 2-DOF planar arm, no wrist, magnet grasp),
> and **SmolVLA on the ST3215 arm** (worth doing, but as a separate
> project in its own repo).
>
> These are additive to an already large M7 and may be cut. If cut, keep
> 1 and 2.


```
Goal: the full fetch, on the Yard, with route selection.

1. Add route selection to the Nav2 layer: three candidate approach poses,
   one per route foot, costs reflecting route difficulty. Nav2 picks; the
   policy executes. The policy does not choose the route.

2. Extend traverse_demo.py: nav to chosen route foot -> rl traverse ->
   vision gate -> stow -> approach -> pick -> loaded descent -> nav home ->
   place. Keep the existing rule that a failed gate skips 3, 4 and 7 but
   still runs 5 and 6 — a robot that comes home empty is recoverable, one
   parked on the platform is not.

3. Verify arbiter mode transitions are unchanged and teleop still preempts
   in every mode including idle.

4. Run the full mission 10 times per route. Report the success matrix and
   every failure with its phase and cause.

5. Record a continuous, uncut video of one successful run per route.
```

---

## Phase 7 — Write-up (M10)

```
Rewrite README.md for v2. Keep the voice of the current one exactly: dense
tables, explicit reasoning, (measured) and (derived) markers, failure
analysis kept rather than sanitised.

Must include:
  - the layer-split argument (M7_DESIGN.md §1.2): why each of the four
    paradigms is present and what breaks without it
  - the baseline-vs-policy ablation matrix
  - the sim-to-sim transfer table
  - the v1 -> v2 progression, with frozen v1 results retained
  - a section on what is still unverified

Do not delete the v1 results. The progression is the story.

Also update docs/FUTURE_WORK.md: items 9(a), 9(c) and 10 are what M7
executes. Mark them resolved with a pointer, rather than deleting them —
they are the record that this was planned, not improvised.
```
