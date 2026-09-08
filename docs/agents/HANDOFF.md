# C2-NAV.34 — investigation handoff: the odometry input to AMCL

**Agent:** investigation only. No behaviour changed, no parameter changed, no
simulator started, no live experiment run, `main` untouched at `ea66155`.
C2-NAV.33's handoff is not lost — it is this file's parent at `8a7b090`.

Claims are marked **FACT** (read this session from an installed artefact or
produced by a tool run this session), **DERIVED** (forced by something
measured, but not directly read), **HYPOTHESIS** (not tested), **UNKNOWN**.

Everything reproducible with:

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis
python3 -P docs/data/c2nav34_odom.py selftest   # 18 passed, 0 FAILED
python3 -P docs/data/c2nav34_odom.py geom
python3 -P docs/data/c2nav34_odom.py incr       # the headline
python3 -P docs/data/c2nav34_odom.py heading    # place dependence
python3 -P docs/data/c2nav34_odom.py elim
python3 -P docs/data/c2nav34_odom.py cmd
python3 -P docs/data/c2nav34_odom.py verdict
```

---

## 1. QUESTION

> Does the `odom → base_footprint` motion actually consumed by AMCL contain a
> systematic translational or angular error that could plausibly generate the
> observed wall-adjacent localization bias?

**Answer: (A), with the axis changed.** AMCL's motion input over-reports
**forward travel**, not yaw. The briefed `wheel_separation_multiplier`
mechanism is **rejected as stated** — it cannot bias the reported yaw rate at
all — and the yaw residual measures **zero**. The along-track residual does
not: `+0.00616 m` per filter update, 95 % CI `[+0.00342, +0.00890]`, which
**excludes zero**.

**But this is not proof of an odometry bias, and I am not claiming it is.**
What is measured is `(AMCL delta) − (GT delta)`, which equals
`(odometry error) + (laser correction)`, and **nothing in this repository
separates those two terms.** The separating measurement is section 9.

---

## 2. ACTUAL AMCL MOTION INPUT

Read from the installed `ros-jazzy-nav2-amcl 1.3.11-1noble.20260412.054619`.
There is no `.cpp` on this machine — headers and shared objects only — so
every statement below is disassembly or an installed header.

**F1. AMCL does NOT consume `/model/coco/odometry`. It performs a TF
lookup. (FACT)**
`laserReceived` (`libamcl_core.so 0xe37b0`) has exactly one call to
`getOdomPose`, at `0xe3be8`. Argument set-up immediately before it:

```
e3ba6:  mov    $0x1,%edx              ; rcl_clock_type_e = 1 = RCL_ROS_TIME
e3bab:  mov    %r11,%rsi              ; &scan->header.stamp
e3bb1:  lea    0x9d8(%rbx),%r13       ; base_frame_id_
e3bb8:  call   rclcpp::Time::Time(builtin_interfaces::msg::Time const&, rcl_clock_type_e)
e3bc1:  mov    %r12,%r9               ; arg5  = that Time
e3bc4:  mov    %rbx,%rdi              ; this
e3bc7:  push   %r13                   ; arg6  = base_frame_id_
e3bd0:  lea    -0x948(%rbp),%rcx      ; arg3  = double& y
e3bd7:  lea    0x5e0(%rbx),%rsi       ; arg1  = latest_odom_pose_
e3bde:  lea    -0x940(%rbp),%r8       ; arg4  = double& yaw
e3be5:  mov    %r14,%rdx              ; arg2  = double& x
e3be8:  call   getOdomPose@plt
```

i.e. `getOdomPose(latest_odom_pose_, x, y, yaw, Time(scan->header.stamp,
RCL_ROS_TIME), base_frame_id_)`.

`r11` is the raw `LaserScan*`. `std_msgs/Header` lays out `stamp` (8 bytes)
then `frame_id`, so `r11+0` is `&header.stamp` — cross-checked by
`e3938: lea 0x8(%rax),%rsi` handing `header.frame_id` to
`strip_leading_slash` (FACT).

**F2. Source frame `base_footprint`, target frame `odom`, and the offsets are
forced by the header's declaration order. (FACT + DERIVED)**
In `getOdomPose` (`0xd6a40`): `lea 0xa88(%r12),%rcx` supplies the target
frame. Walking `amcl_node.hpp:357-386` from `base_frame_id_` at `0x9d8`
(32-byte `std::string`, 8-byte doubles, `bool` padded to alignment):

| member | offset |
|---|---|
| `base_frame_id_` | `0x9d8` |
| `global_frame_id_` | `0xa18` |
| `sensor_model_type_` | `0xa58` |
| **`odom_frame_id_`** | **`0xa88`** ✓ matches the `lea` |
| `resample_interval_` | `0xac8` ✓ independently reproduces C2-NAV.33 F3/F8 |

The arithmetic landing on **both** `0xa88` and C2-NAV.33's separately-derived
`0xac8` is the cross-check; I did not inherit either.

**F3. The lookup timeout is ZERO. (FACT)**

```
d6ba9:  pxor   %xmm0,%xmm0            ; xmm0 = 0.0
d6bad:  mov    0x480(%r12),%r15
d6bb5:  movaps %xmm4,-0x860(%rbp)
d6bbc:  movaps %xmm5,-0x850(%rbp)
d6bc3:  call   tf2::durationFromSec@plt
```

Nothing writes `xmm0` between the `pxor` and the call, so the duration is
`0.0` — the `tf2_ros::BufferInterface::transform` default. **AMCL never waits
for TF.** `transform_tolerance_` (`0xb08`) is *not* passed here; it governs
the outgoing `map → odom` broadcast, not this lookup.

**F4. Time semantics. (FACT/DERIVED)**
`0xd6c10: imul $0x3b9aca00,%rax,%rax` (×1e9) then `add %rdx,%rax` converts
`sec`/`nanosec` to a tf2 `TimePoint` — the lookup is at the **scan
timestamp**, not "latest". tf2 interpolates between the two bracketing
buffered transforms; with a zero timeout an unavailable or extrapolated time
throws, `getOdomPose` returns false, and the scan is **dropped entirely**
(the `e3bfa: test %r13b,%r13b` / `jne` branch) — no motion update, no
sensor update, no cloud (DERIVED from the branch structure).

**Lookup frequency:** once per `/scan` message that passes the branch, i.e.
at the lidar rate, not at `update_min_d`/`update_min_a` — those gate the
*sensor* update further down (DERIVED).

`this+0x480` is used as `tf_buffer_` via a virtual dispatch
(`mov (%r15),%rax; mov (%rax),%rsi`), which is **consistent with**
`std::shared_ptr<tf2_ros::Buffer>` at `amcl_node.hpp:166` but is not proven
by offset arithmetic the way `0xa88` is. Treat as DERIVED.

---

## 3. ACTUAL ODOMETRY PRODUCER

**F5. `diff_drive_controller`, and it is the sole publisher of that TF.
(FACT)**
`gazebo_models/urdf/coco_controllers.yaml`: `odom_frame_id: odom`,
`base_frame_id: base_footprint`, `enable_odom_tf: true`,
`publish_rate: 50.0`, `open_loop: false`.

The gz `OdometryPublisher` in `coco_robo2.xacro` is **not** a competitor:
its `<odom_frame>` is `world`, it publishes to `/model/coco/odometry`, and
the xacro's own comment states it "Publishes no ROS TF, so it cannot fight
the diff-drive controller's `odom->base_footprint` transform" (FACT).
`robot_state_publisher` publishes only the URDF's fixed/joint transforms,
which do not include `odom` (DERIVED).

**F6. Odometry integrates MEASURED wheel positions, so wheel slip is absorbed
in full. (FACT)**
`diff_drive_controller_parameters.hpp:88` — `position_feedback = true`, the
default, **not overridden** in `coco_controllers.yaml`. With
`open_loop: false` the controller calls
`Odometry::update(left_pos, right_pos, time)`
(`odometry.hpp:41`; both it and `updateOpenLoop` appear as call sites at
`libdiff_drive_controller.so 0x6eaeb` / `0x6eb31`). The integration is

```
u_l = (left_pos  - left_pos_old ) * left_wheel_radius      # ground-arc, m
u_r = (right_pos - right_pos_old) * right_wheel_radius
linear  = (u_r + u_l) / 2
angular = (u_r - u_l) / wheel_separation_
integrateExact / integrateRungeKutta2   (odometry.hpp:63-64)
```

so a wheel that rotates further than the ground moves adds **directly** to
reported distance. There is no slip term and no lateral state.

---

## 4. WHEEL / MODEL GEOMETRY

**F7. The nominal `wheel_separation` IS the true physical track. (FACT)**
Derived from `coco_robo2.xacro` and checked by assertion in
`c2nav34_odom.py geom` / `selftest`. `chassis_joint` carries
`rpy=(π/2,0,0)`, `xyz=(0.12,−0.08,0)`; `Rx(π/2)` maps `(x,y,z)→(x,−z,y)`:

| joint | base_link |
|---|---|
| `base_Revolute-1` front-right | `(+0.090, −0.137, +0.045)` |
| `base_Revolute-2` rear-right | `(−0.090, −0.137, +0.045)` |
| `base_Revolute-3` front-left | `(+0.090, +0.137, +0.045)` |
| `base_Revolute-4` rear-left | `(−0.090, +0.137, +0.045)` |

Physical lateral track `0.274000 m`; parameter `0.274000 m`; difference
**`0.00e+00`**. (This also reproduces the xacro's own comment at line 57.)

So `wheel_separation_multiplier: 1.10` is **not** correcting a nominal/actual
mismatch, which is what the upstream parameter is documented for
("Correction factor when the actual wheel separation differs from the nominal
value", `diff_drive_controller_parameters.hpp:612`). It is used off-label as a
skid-steer yaw compensation, exactly as the repo's own comment says.
Effective `b_eff = 0.301400 m`, `+10.0 %`.

**F8. THE CANCELLATION IDENTITY — this is what rejects the briefed
hypothesis. (DERIVED)**
The multiplier is applied to **both** the command path and the odometry path:

```
command   :  u_r − u_l = w_cmd · b_eff
odometry  :  w_odom    = (u_r − u_l) / b_eff  ==  w_cmd
```

**`b_eff` cancels.** The multiplier cannot by itself bias the *reported* yaw
rate — odometry hands back the commanded yaw rate whatever the multiplier is.
What it changes is the *physical* yaw the wheels are asked to produce:

```
w_true = η · w_cmd · b_eff / b_true = η · 1.10 · w_cmd
w_odom / w_true = 1 / (1.10 · η) ,  unbiased iff η = 1/1.10 = 0.9091
```

η, the skid-steer yaw efficiency, is condition-dependent and is **measured
nowhere in this repository (UNKNOWN)**. So the multiplier remains a *possible*
yaw-error source through η — but the measurement below says the yaw channel
is not where the error is.

**Linear channel (DERIVED):** `v_odom = (u_r + u_l)/2 = v_cmd` exactly — no
separation, no multiplier. Nothing cancels a longitudinal slip.

---

## 5. KNOWN PARAMETERS

| parameter | value | source |
|---|---|---|
| `wheel_separation` | 0.274 m | `coco_controllers.yaml` |
| `wheel_radius` | 0.0585 m | ditto |
| `wheel_separation_multiplier` | 1.10 | ditto |
| `left/right_wheel_radius_multiplier` | 1.0 / 1.0 | ditto |
| `open_loop` | false | ditto |
| `position_feedback` | true (default, unset) | `..._parameters.hpp:88` |
| `enable_odom_tf` | true | `coco_controllers.yaml` |
| `publish_rate` | 50.0 Hz | ditto |
| wheel `mu1`/`mu2` | 0.7 / 0.7, isotropic | `coco_robo2.xacro` |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | `nav2_params.yaml:32` |
| `alpha1..5` | 0.2 each | `nav2_params.yaml:7-11` |
| `update_min_d` / `update_min_a` | 0.25 m / 0.2 rad | `nav2_params.yaml:37-38` |
| `odom_frame_id` / `base_frame_id` | `odom` / `base_footprint` | `nav2_params.yaml` |
| `transform_tolerance` | 0.5 s (**not** the lookup timeout, F3) | `nav2_params.yaml:36` |
| `resample_interval` | 1 | `nav2_params.yaml:32` |

---

## 6. EVIDENCE FOR / AGAINST ODOMETRY BIAS

Data: `docs/data/c2nav28_amcl.json` (committed; one row per AMCL update,
carrying GT and the AMCL estimate) and `docs/data/c2nav34_odom.json` (built
this session from the **uncommitted** 10 Hz CSVs under `.navbench/`).

The statistic is the **signed** body-frame residual per filter update,
`r = (AMCL body delta) − (GT body delta)`. Signed, so cloud noise cannot
inflate it; a constant world→map offset — including the open 56 mm `x`
discrepancy — cancels exactly, which `selftest` asserts for both a
translation and a rotation.

### FOR

**E1. The along-track residual excludes zero; the yaw residual does not.
(FACT)** Pooled over 523 updates, 21 legs, 4 runs:

| component | mean / update | 95 % CI | verdict |
|---|---|---|---|
| **along-track** | **+0.00616 m** | **[+0.00342, +0.00890]** | **excludes zero** |
| yaw | +0.00271 rad | [−0.00647, +0.01190] | straddles zero |

**This inverts the briefed hypothesis.** The error is translational.

**E2. It is positive in 18 of 21 legs and reproducible per place. (FACT)**
Along-track excess as a fraction of GT travel: `open_space`
**10.1 / 10.0 / 11.1 / 8.2 %** (4 of 4 runs), `corridor_gate` 5.8 / 6.9 / 3.6 %,
`obstacle_corner` 5.9 / 5.7 / 3.7 %. Pooled mean **+7.68 %**.
The one consistent negative is `wall_parallel` (−3.6 / −0.9 / −4.1 %), which
is also the only leg that travels **east** rather than north or south.

**E3. The world-frame error points ALONG the course, and that is what makes
it place-linked. (FACT)** Forward projection positive in **16 of 21** legs,
mean **+0.0377 m**, CI `[+0.0011, +0.0743]`; lateral projection **−0.0189 m**,
CI `[−0.0594, +0.0215]` — straddles zero.

`open_space` is the clean case, 4 of 4 runs: course **−89.5 / −89.2 / −90.9 /
−89.6°** (due south), error almost purely along-track —
`proj_fwd` **+0.117 / +0.104 / +0.118 / +0.099 m** against `proj_lat`
−0.004 / −0.010 / +0.044 / −0.003 m. And `enclosure_entry`, which heads
**north** (+110 / +106 / +108°), carries a **northward** error
(+0.197 / +0.182 / +0.055 m).

**E4. It is created where the bias is created. (FACT)** At `open_space` the
world-frame y error starts at **+0.004 / +0.001 / +0.016 / +0.013 m** and ends
at **−0.113 / −0.103 / −0.103 / −0.086 m** — 4 of 4 runs. `wall_adjacent`
then *inherits* it (starts −0.113 / −0.071 / −0.107 / −0.100). This matches
C2-NAV.30's focus run, whose first `open_space` cloud was effectively a point
mass at dy +0.0119 m.

**E5. The command channel alone already forces a lower bound. (FACT)**
Steady windows (`v_wheel` held ≥ 0.5 s, |`v_wheel`| ≥ 0.1 m/s, robot moving,
collision monitor not gating): slope `v_act / v_wheel` = **0.9712**, n = 253.
Since `v_odom ≡ v_cmd` (F8), odometry over-reports forward distance by
**≥ 2.96 %** — and that captures *only* the controller's velocity-tracking
shortfall. Wheel slip (F6) adds on top and is invisible to every artefact
here.

**E6. The surviving direction is the unobservable one. (FACT, corroborating)**
C2-NAV.31 measured the 99 % likelihood plateau at `wall_adjacent` as 0.110 m
wide spanning **[−0.085, +0.060] m in y** — and `wall_adjacent` runs due
south, so **y is the along-track axis there**. An along-track error is
precisely the one the observation model cannot correct (the corridor aperture
problem), which is why it survives while C2-NAV.29–.33 found every
cross-track mechanism restoring.

### AGAINST / ELIMINATED

**E7. The heading-error integral does NOT generate the position error.
(FACT)** Integrating `ė = e_ψ · (−v_y, +v_x)` against GT velocity and the
*observed* heading error reproduces the observed y error within a factor of 2
and correct sign in only **2 of 21** legs. At `open_space` it predicts
**−0.006 / +0.013 / +0.006 / +0.044 m** against an observed **≈ −0.10 m**.
The bias is not heading-propagated.

**E8. Lateral-skid blindness is real but points the WRONG WAY. (FACT)**
A differential-drive integrator has no lateral state, and the robot genuinely
slides: total |lateral| **4.434 m over 52.697 m of path = 8.4 %**, rising to
**14–33 %** at `wall_adjacent`. But the world-frame term it must accumulate,
`−∫ v_lat n̂ dt`, agrees in sign with the observation in only **2 of 21**
legs and is ~5× too small (`open_space`: +0.022 / +0.016 / +0.023 / +0.020 m
against −0.10). Eliminated as the principal mechanism.

**E9. The `wheel_separation_multiplier` cannot bias reported yaw. (DERIVED,
F8)** And the measured yaw residual straddles zero (E1). The briefed
mechanism is **rejected as stated**. It survives only indirectly, through η.

### THE HONEST GAP

`r = (odometry error) + (laser correction)`. E1–E6 do **not** prove the
odometry is biased; they prove **AMCL's estimate advances along-track faster
than the robot does**. Attributing that to odometry requires the assumption
that the laser correction is restoring rather than driving — supported by
C2-NAV.33's `corr(shift, dy_u) = −0.71` and by C2-NAV.29's 90/90 GT-scores-
higher, but it is an assumption, and it is why section 9 exists.

---

## 7. QUANTITATIVE BIAS ESTIMATE

| quantity | value | tag |
|---|---|---|
| along-track residual, per update | +0.00616 m, CI [+0.00342, +0.00890] | FACT |
| along-track residual, per leg | +7.68 % of GT travel (mean, 21 legs) | FACT |
| at `open_space` specifically | +9.87 % (10.1/10.0/11.1/8.2) | FACT |
| yaw residual, per update | +0.00271 rad, CI straddles zero | FACT |
| net world along-track error, `open_space` | +0.099 … +0.118 m over 2.13 m travelled = **4.6–5.5 %** | FACT |
| command-channel lower bound on odometry over-report | **≥ 2.96 %** | FACT |
| unexplained span between the two | ≈ 4.7 percentage points | DERIVED |
| skid-steer yaw efficiency η | — | UNKNOWN |
| true `odom → base_footprint` error | — | **UNKNOWN, unrecorded** |

---

## 8. CAN THAT BIAS EXPLAIN ~0.09 m?

**Yes, on the arithmetic, and with room to spare. (DERIVED)**

`open_space` runs **due south for 2.13 m**. A pure along-track gain error
`ε` lands as a southward y error of `2.13 · ε`:

| ε | y error at end of `open_space` |
|---|---|
| 2.96 % (command-channel bound, E5) | **−0.063 m** |
| 4.2 % | −0.090 m ← the named bias |
| 5.2 % (the observed net, E3/E4) | **−0.111 m** |
| 7.68 % (the residual mean, E2) | −0.164 m |

The named bias is **−0.09 to −0.13 m southward**. The measured net along-track
error at `open_space` is **+0.099 to +0.118 m forward on a −90° course**,
i.e. **−0.099 to −0.118 m in y** — the bias, in magnitude, in sign, and in
axis, without fitting anything. And the command channel *alone*, before any
wheel slip, already supplies **70 %** of it.

`wall_adjacent` then inherits the offset rather than creating it (E4), which
is why C2-NAV.28 saw the bias appear *before* terminal yaw and why C2-NAV.30's
cloud was already displaced in its first observable update.

**Separating absolute odometry bias from AMCL's response (as briefed):** the
absolute odometry bias is **UNKNOWN** — unrecorded. AMCL's response to it is
partly measured: the filter carries ~5.2 % of the ~7.7 % residual through to
its published pose at `open_space`, i.e. the laser removes roughly a third
and cannot remove the rest because the residual lies along the plateau axis
(E6).

---

## 9. REQUIRED INSTRUMENTATION

**Diagnostic proposal — NOT implemented. No file under `gazebo_models/` was
modified this session.**

The minimum is one `tf2_ros::TransformListener` inside `nav_bench.py`,
recording the transform AMCL itself queries.

| appended column | source |
|---|---|
| `odo_x`, `odo_y`, `odo_yaw` | `lookup_transform('odom', 'base_footprint', t)` |

That alone is sufficient to prove or reject, because `ω_odom ≡ ω_cmd` and
`v_odom ≡ v_cmd` (F8) make the TF the complete statement of what AMCL was
told. Follow C2-NAV.28's and C2-NAV.30's appending rule **exactly**: columns
appended at the end so every existing index is unchanged; last sample in the
half-open bucket `(t−0.1, t]`; **blank when the bucket is empty, never a
forward fill**.

**The blindness guard is mandatory and must be POST-run.** CLAUDE.md: any
check whose success condition is "we saw nothing" must first prove it can see
something. A TF listener that never matches yields blank columns that read
exactly like "the odometry did not drift". C2-NAV.30 already paid for the
pre-run version of this. Assert ≥ 1 non-blank `odo_*` row per leg *after* the
run and report UNOBSERVABLE otherwise.

**Optional second group, only if attribution is wanted** — `/joint_states`
wheel positions, which would split the 2.96 % controller shortfall from the
wheel-slip remainder (E5). Not required to answer the question; do not let it
delay the run.

`ros_clean.sh` needs **no** new pattern: this is a subscription inside an
existing node, not a new process. Stated because CLAUDE.md requires the
check, not because it fires.

---

## 10. PROPOSED LIVE TEST

One tour, **fresh simulator**, topology A, `open_space` → `wall_adjacent`,
same route and 75 s cap as the C2-NAV.28/.30/.33 focus runs, under
`docs/data/c2nav25_slow_params.yaml`, so it is directly comparable to
`c2n30_focus_r1` and `c2n33_focus_r1`.

**Exact parameter change: NONE. Not one leaf.** A `paramdiff` over all 323
leaves must report added 0, removed 0, changed 0, shown *before* the
simulator starts. `resample_interval: 2` is **not** carried forward — it
halves the `/amcl_pose` rate (C2-NAV.33) and is a diagnostic window, not a
configuration.

**Analysis.** Per AMCL update, along-track:

```
odo_along_cum = along-track travel integrated from odo_* since leg start
gt_along_cum  = along-track travel integrated from GT      since leg start
excess_odo    = odo_along_cum / gt_along_cum − 1
excess_amcl   = (already measured this session: +9.87 % at open_space)
```

`open_space` is the discriminating leg: it heads due south, it is where the
displacement is *created* rather than inherited (E4), and its first cloud in
`c2n30_focus_r1` was effectively a point mass — so the replay must reproduce
the creation, not carry an inherited offset.

---

## 11. FALSIFIER

Fixed in advance. The discriminator is a **comparison**, not a threshold on
one number.

- **FALSIFIED** if, over `open_space`, `|excess_odo| < 2 %` while
  `excess_amcl` again reaches ~10 %. The odometry then cannot be supplying
  the along-track advance and joins the eliminated list — and the bias
  becomes genuinely unexplained by any term yet examined, which is itself the
  finding.
- **SURVIVES** (not confirmed — one run) if `excess_odo ≥ 4 %` **and the sign
  is positive**. Sign agreement is required: C2-NAV.32's motion-model replay
  was rejected partly for pointing north, and the same standard applies.
- **Between 2 % and 4 %:** report as INDETERMINATE at n = 1. Do not round
  into either bucket.
- **UNOBSERVABLE, a distinct outcome** — reported as such, never as "the
  odometry is clean" — if the post-run guard finds no non-blank `odo_*` row,
  or TF lookups fail on more than 10 % of buckets. C2-NAV.33 hit exactly this
  trap (its verdict printed `(C) UNOBSERVABLE` on a missing artefact); the
  same must not be silently folded into a finding here. `load34()` in
  `c2nav34_odom.py` already `sys.exit`s with a distinct message rather than
  returning an empty result, for the same reason.

**Null control, required.** Replay the analysis with `odo_*` set equal to
ground truth; `excess_odo` must be 0 at machine epsilon. This session's
equivalent already passes: `selftest` reports worst |along| **0.000e+00 m**
over 21 legs, and a synthetic +8.000000 % injection is recovered as
**+8.000000 %**.

---

## 12. LIMITATIONS

1. **The odometry is still not measured.** Everything in section 6 is
   `odometry error + laser correction`, unseparated. This handoff narrows
   *which axis* to look at; it does not close the question.
2. **n = 4 runs, one topology, one route set.** `open_space` is 4 of 4 and
   `enclosure_exit` is n = 1. Legs are not independent — a tour carries error
   forward across leg boundaries, so per-leg values are not 21 independent
   samples and the pooled CI is optimistic.
3. **`docs/data/c2nav34_odom.json` is derived from uncommitted CSVs** under
   `.navbench/results/`, which are not in the repository. The bundle is
   committed so the numbers survive; `build` will not re-run without those
   CSVs. Modes `incr` and `verdict` need only the committed C2-NAV.28 bundle.
4. **Three rows carry a ground-truth twist glitch** — `w_act ≈ 314.10 rad/s`
   (= 100π) at `wall_adjacent` t = 5.0–5.1 s in one run. `v_act` is clean
   (0 rows > 2 m/s). No number reported here uses `w_act`: the position
   analyses differentiate `x`,`y`,`yaw`, and `cmd` uses `v_act`. Flagged
   because it is a real defect in `/model/coco/odometry`.
5. **`this+0x480` as `tf_buffer_` is DERIVED**, not proven by offset
   arithmetic the way `odom_frame_id_` at `0xa88` is.
6. **`open_space`'s point-mass first cloud may not recur** — it was a
   property of `c2n30_focus_r1` beginning just after AMCL initialised, not of
   the route. If the new run's first cloud is already displaced the leg loses
   discriminating power. Record it and say so; do not re-run until it comes
   out convenient.
7. **The `WORLD_TO_MAP` 56 mm x discrepancy is open for a sixth session.** It
   cannot affect anything here — every quantity is a difference of two poses
   in one frame, and `selftest` asserts a constant offset cancels — but it
   still needs a decision rather than a patch.
8. **C2-NAV.30's selftest allow-list still fails** on files from
   C2-NAV.31/.32/.33 and will now fail on C2-NAV.34's too. Pre-existing,
   recorded twice, not caused here. It needs widening or retiring.
9. **`docs/agents/CODEX_REVIEW.md` does not exist** in this worktree. The
   brief asked me to read it; only `HANDOFF.md` was present. If a Codex
   review was produced it did not land here.

---

## 13. CONFIDENCE

| claim | confidence | basis |
|---|---|---|
| AMCL's motion input is a zero-timeout TF lookup of `odom → base_footprint` at the scan stamp | **High** | Disassembly of the deployed `.so`; `odom_frame_id_` offset independently reproduces C2-NAV.33's `resample_interval_` |
| `b_eff` cancels, so the multiplier cannot bias reported yaw | **High** | Algebra over the two documented paths; asserted in `selftest` |
| The nominal wheel separation equals the true physical track | **High** | Derived from the xacro and asserted (`0.00e+00` difference) |
| The residual is along-track, and the yaw channel measures zero | **High** for the sign and axis, **Moderate** for the magnitude | Signed statistic, null control at exactly 0, synthetic injection recovered exactly; but 4 runs and non-independent legs |
| Heading propagation and lateral-skid blindness are eliminated | **Moderate–High** | Heading integral matches in 2 of 21 legs; skid blindness agrees in sign in 2 of 21 and is ~5× too small |
| An along-track odometry over-report of the measured scale would produce −0.09…−0.13 m at `open_space` | **High** (arithmetic) | 2.13 m due south × 4.6–5.5 % |
| **That the odometry is in fact the source** | **Low — this is a HYPOTHESIS** | The measured residual folds in the laser correction, and the odometry is recorded nowhere. This is the whole point of section 9 |
| The briefed `wheel_separation_multiplier` mechanism as stated | **Rejected** | F8 plus a yaw CI straddling zero |

---

## 14. EXACTLY ONE NEXT ACTION

**Add the three `odo_x`/`odo_y`/`odo_yaw` columns of section 9 to
`nav_bench.py` — instrumentation only, no parameter leaf moved — and get that
diff reviewed before any simulator starts.**

Not the run. The column-append rule, the half-open bucket, the never-forward-
fill rule and the post-run blindness guard are exactly the details that have
cost this investigation runs before, and they are cheaper to review than to
re-run.

---

## FOR CODEX — the claims most worth attacking

Stated so they can be checked rather than accepted:

1. **The cancellation identity (F8).** If `wheel_separation_multiplier` does
   *not* scale the command path as well as the odometry path in
   `diff_drive_controller` 4.39.0, the whole rejection of the briefed
   hypothesis is wrong. I read this from the parameter semantics and the
   `Odometry::update` equations, **not** from `diff_drive_controller.cpp`,
   which is not installed. Exact files:
   `/opt/ros/jazzy/include/diff_drive_controller/diff_drive_controller/odometry.hpp:41-52`,
   `.../diff_drive_controller_parameters.hpp:78-88,612`,
   `/opt/ros/jazzy/lib/libdiff_drive_controller.so` (`0x6eaeb`, `0x6eb31`).
2. **The zero timeout (F3).** `pxor %xmm0,%xmm0` at `libamcl_core.so 0xd6ba9`,
   call at `0xd6bc3`. If any instruction between them writes `xmm0`, F3 is
   wrong.
3. **The offset arithmetic** from `base_frame_id_ 0x9d8` to
   `odom_frame_id_ 0xa88`, `amcl_node.hpp:357-386`. It assumes 32-byte
   `std::string` and natural alignment. If that is wrong, so is the frame
   identification — though the independent landing on `0xac8` argues it is not.
4. **That one row of `docs/data/c2nav28_amcl.json` is one AMCL update.** I
   inferred this from `write_trace`'s documented `bucket_last` + blank rule
   (`nav_bench.py:1428-1450, 1575-1580`) plus the observation that every row
   has a distinct non-blank AMCL pose and `dt` is a variable multiple of
   0.1 s. If the bundle instead forward-fills, the per-update statistics are
   wrong (the per-leg sums and the world-frame endpoints are not).
5. **That the laser correction is restoring**, which is what licenses reading
   the residual as a lower bound on odometry error. Inherited from C2-NAV.33
   (`corr = −0.71`) and C2-NAV.29 (90/90), and measured on the *cross-track*
   centroid — **not** on the along-track axis this session is about. This is
   the weakest link in the chain and I have not strengthened it.
6. **`.navbench/results/` is uncommitted**, so `build` is not reproducible
   from a fresh clone. Check `docs/data/c2nav34_odom.json` against the CSVs
   if they still exist on this machine.
