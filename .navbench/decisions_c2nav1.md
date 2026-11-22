
---

## A goal with no heading, and the two things that went with it

*(C2-NAV.1, measured 2026-09-01. Numbers in `docs/RESULTS.md`.)*

`NavigateToPose` takes a `PoseStamped`. A pose has an orientation. There
is no field for "I do not care", so every caller in this repo fills in
`orientation.w = 1.0` and Nav2 reads that identity quaternion as a
*requirement to finish facing map-x*.

That is a claim nobody in this project ever made. The planner says so in
its own config — `use_final_approach_orientation: false`, commented "the
goal has no meaningful heading here". The mission's heading gate was
turned off precisely because the arrival heading was not meaningful. And
yet C2-NAV.0 measured a median **35 % of every navigation leg** being
spent turning on the spot to satisfy it, with `RotateToGoal` throwing
**50.7 %** of all trajectory rejections and the progress checker aborting
`follow_path` every 10 s because a rotating robot never translates 0.1 m.

**The fix is a plugin, not a tolerance.** Nav2 ships
`nav2_controller::PositionGoalChecker` — "Goal checker that only checks
XY position and ignores orientation". The alternative was to raise
`yaw_goal_tolerance` toward π, which produces the same behaviour and
leaves a knob in the file that invites tuning. Removing the parameter is
better than widening it: there is nothing left to adjust, and the config
now states the intent rather than approximating it.

It worked, and by a wide margin: **RotateToGoal rejections 465 063 → 0**,
median leg **−37 %** with median *transit* time flat, `wall_adjacent`
1/3 → 3/3 and 77.34 → 4.22 s. Watched on a fresh sim, the baseline's
terminal spin runs at **1.037 rad/s** — `max_vel_theta` is 1.0, so it is
spinning as fast as the robot can — and afterwards peaks at 0.231 rad/s.

### Two things came off with it, and only one was expected

**The heading, obviously.** Median |final heading| 0.449 → 1.583 rad.
That was the point. But it is not free here, and the reason is three
sections up in this file: 2.5 m of climb at 0.25 rad **is** 0.64 m of
lateral, and the RL policy holds a line to three centimetres but cannot
*correct* one. The pre-ramp leg was already arriving up to 0.472 rad off
with `SimpleGoalChecker` bounding it at 0.25. Remove the bound and there
is nothing left holding that number down. **This is why C2-NAV.1 is
measured but unadopted.**

**The late position correction, which was not expected.** Ground-truth
arrival error went **0.118 → 0.263 m**, and only 7 of 21 legs got within
0.25 m of the goal by ground truth against 18 of 21. The terminal phase
was never *purely* rotation: while `RotateToGoal` pinned the robot,
`GoalDist` kept pulling in the last ~0.15 m of position error, and the
goal checker was judging against AMCL while the tape measure is ground
truth. Deleting the phase deleted the correction that happened to be
riding along inside it.

That is the general lesson worth keeping. **A phase named for one
behaviour was doing two.** "The terminal phase is the robot settling on
a yaw it does not need" was true, and acting on it still cost something
that had nothing to do with yaw. The before/after was wide enough to
show it — a success-rate-only comparison would have read 16/21 → 18/21
and shipped.

### A methodological note: one metric did not survive the change

`xtrack_med_m` moved 0.571 → 1.227 m and means nothing. It is the median
distance from *every time-uniform sample* of the driven track to the
*last* global plan, and the last plan is a stub at the goal. A leg that
parks at the goal for a third of its duration donates a third of its
samples at distance ≈ 0. Measured: the baseline parks **32.8 %** of its
samples within 0.25 m of the goal, C2-NAV.1 parks **0.0 %**. The metric
was rewarding the stall it was supposed to be neutral about.

Worth stating because the number is *directionally plausible* — a robot
that no longer settles could genuinely track worse — and it would have
been easy to write "cross-track doubled" into a results table and have
it read as a real cost for the rest of the project's life. A metric
whose definition depends on the shape of a leg cannot be used to compare
two different leg shapes.
