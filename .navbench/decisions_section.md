
---

## C2-NAV.0 — the wall stall is three mechanisms, and only one is about walls (2026-09-01)

**Decision: diagnose and rank, change nothing.** The brief was explicit
that no Nav2 value should be tuned because it looks suspicious, and after
measuring, two of the three values that look most suspicious turn out to
be either correct or the wrong direction. Recording that here so the next
session does not re-derive it.

### Why the benchmark splits every leg at the goal

Every leg is reported as **transit** (start → first entry into the goal
checker's 0.25 m `xy_goal_tolerance`) and **terminal** (everything
after). Without the split, `open_space` reports a mean velocity of
0.10 m/s and looks like a robot that crawls; with it, the robot drives
2.2 m at a clean 0.28–0.30 m/s and then stands still for 10 s. Those are
opposite diagnoses and a single mean cannot tell them apart. **The
terminal phase is a median 35 % of every leg**, so this is not a corner
case.

### Why `BaseObstacle.scale: 8.0` is not simply "too high"

The comment in `nav2_params.yaml` that raised it from 0.02 argued that
obstacle avoidance carried 1/1600th the weight of path-following, and
that the ratio was indefensible. That was correct. What was not checked
is that the critics are scaled **in different units**:
`MapGridCritic::getScale()` returns `resolution * 0.5 * scale` — a weight
per grid *cell* — while `BaseObstacle` inherits the base
`getScale() = scale` and applies it to a *cost* running 0–252
(`dwb_critics/map_grid.hpp:69`, `dwb_core/trajectory_critic.hpp:177`).

At 8.0 the ratio has inverted rather than balanced: measured over 21
legs, **BaseObstacle is 55 % of the score of the trajectory DWB actually
picks and PathDist + PathAlign together are 2.4 %**. Lowering the scale
is therefore a candidate, but so is fixing the *shape* of the field it
multiplies, and the evidence does not yet say which. That is C2-NAV.1.

### Why the inflation radius is the more likely culprit than the scale

`inflation_radius: 0.50` with `robot_radius: 0.20` means the cost only
reaches zero 0.50 m from an obstacle, so a corridor needs **1.00 m of
clearance either side of the robot centre** before any cell in it is
free. Measured, this arena does not have that anywhere it matters:

| passage | gap | non-inscribed band | zero-cost band |
|---|---|---|---|
| Zone A gate | 1.30 m | 0.90 m | 0.30 m |
| north channel past `box_obstacle_1` | 0.75 m | 0.35 m | **0.00 m** |
| NW pinch | 0.63 m | 0.30 m | **0.00 m** |

Where the zero-cost band is empty there is no line the robot can take
that escapes the gradient, so BaseObstacle reads a large absolute number
everywhere and — because `sum_scores: false` scores only the trajectory's
final pose — the cheapest command is always the slowest one. **The stall
is a property of the cost field, not of any one trajectory being
blocked**: at the measured stall DWB had 777 of 819 trajectories valid
and chose zero anyway.

### Why `robot_radius` will not be reduced

Measured from live TF and the URDF collision geometry, the robot's
circumscribed radius is **0.2051 m** and `robot_radius` is 0.2000 —
**5.1 mm too small**, not too large. The intuition that a 0.20 m disc
around a 0.283 m-wide robot is over-cautious is wrong because the robot
is 0.3195 m *long* and rotates in place at every goal, so the swept disc
is the correct model. Reducing it, or replacing it with a polygon, would
make Nav2 plan through gaps the robot cannot turn in.

This also means the first measurement attempt — which reported 0.2199 m
by treating each wheel cylinder as its bounding box — was wrong by 15 mm
in the direction that flatters the conclusion. Cylinders are now sampled
at both rims. The chassis cross-checks exactly against `CHASSIS_SIZE`.

### Why the collision monitor is recorded as an aggravator, not a cause

It gates a great deal — 86 % of the `wall_adjacent` leg — and its zones
reach further than they read: `PolygonSlow` and `PolygonLimit` are
axis-aligned **squares**, so their diagonal reach is 0.566 m and 0.778 m
against a nominal 0.40 m and 0.55 m. Against a flat wall, which supplies
`min_points: 4` trivially, they engage most of the time. And
`slowdown_ratio: 0.3` scales **angular** velocity too, throttling exactly
the rotation that mechanism 1 makes every leg depend on.

But at the enclosure stall the monitor was reading `SLOWDOWN` against a
command that was **already zero at `/cmd_vel_nav`**, upstream of it, and
across all of topology A it reduced the command on only 10.5 % of
samples. It makes a slow robot slower; it is not why the robot stops.

### The `/cmd_vel_nav` ownership loop, measured against a control

KNOWN LIMITATIONS 0 has been characterised before, but never against the
same tour run without the loop. Both were run here:

* topology A (`nav.launch.py` alone): wheels exceeded the collision
  monitor's output on **4 of 6 956 samples (0.06 %)**, worst gap
  0.016 m/s — i.e. resampling jitter. **The monitor is authoritative.**
* topology B (`arbiter:=true`, the mission): **1 371 of 9 793 (14.0 %)**,
  worst case the monitor commanding **0.0 m/s while the wheels received
  0.300**.

The loop also costs movement quality: 14/21 legs against 16/21 and a
median transit speed of 0.155 m/s against 0.208 (−25 %). **It is still
not the stall** — `enclosure_entry` fails 0/3 in both — so the fix for it
is a safety fix and is scoped separately from the movement work.

One incidental finding falls out of the comparison: topology A dropped
**233** wheel commands as stale and topology B dropped **0**, because
`cmd_vel_relay` republishes with the original `header.stamp` while
`cmd_vel_arbiter` re-stamps. Whoever renames the relay's output topic
should re-stamp at the same time.

### What was deliberately not done

No parameter was changed, no test was edited, and no fix was applied. The
ranked proposals live in `docs/ROADMAP.md` under C2-NAV.1 with the
validation each one needs, because the whole point of measuring first is
that the two most obvious candidates — a smaller footprint and the
collision-monitor wiring — are now ruled out as explanations for the
symptom that prompted the session.
