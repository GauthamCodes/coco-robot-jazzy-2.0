# C2-NAV.33 — investigation handoff: AMCL weighting and resampling

**Agent:** investigation only. No behaviour changed, no parameter changed,
no simulator started, no live experiment run, `main` untouched. Everything
below was either read out of the **installed** `ros-jazzy-nav2-amcl`
`1.3.11-1noble.20260412.054619` or reproduced offline from the committed
bundles in `docs/data/`.

Claims are marked **(measured)** — produced from a run or a tool executed
in this session; **(derived)** — forced by something measured but not
directly read; **(hypothesis)** — not yet tested.

---

## QUESTION

As briefed:

> Determine exactly how `nav2_amcl` calculates particle weights and
> resampling, and establish the minimum evidence-backed way to observe
> pre-resampling importance weights for the wall-adjacent AMCL bias
> problem.

**The first finding is that this question is already closed in this
worktree, and the answer is not a hypothesis.** C2-NAV.33 pre-registered
the experiment at `5bc214c` and ran it at `d44041a` (2026-09-07). The
branch `worktree-c2nav0-diagnosis` is level with
`jazzy2/worktree-c2nav0-diagnosis`, 0 ahead / 0 behind **(measured)**.

So this handoff does three things instead of re-proposing a completed
experiment:

1. **Verifies the AMCL path independently** from the deployed binary,
   rather than inheriting C2-NAV.33's reading of it.
2. **Records one correction** to a disassembly citation in
   `c2nav33_extra.py` that changes no measurement.
3. **Defines the next experiment** — the one C2-NAV.33's result actually
   points at — with its falsifier fixed in advance.

The standing question that remains open is therefore **not** "do the
weights favour south". It is:

> The particle cloud is already ~0.09 m south in the **first observable
> update** of the `wall_adjacent` leg, and every per-update mechanism
> inside AMCL now measures negligible. What supplies the displacement?

---

## FACTS

Facts about the filter, each verified in this session against the
installed artefacts.

**F1. Only headers and shared objects are installed. There is no
`nav2_amcl` source on this machine. (measured)**
`dpkg -L ros-jazzy-nav2-amcl` lists `amcl_node.hpp`, `pf.hpp`, `map.hpp`,
the motion-model and laser headers, and `libamcl_core.so`,
`libmap_lib.so`, `libmotions_lib.so`, `libpf_lib.so`, `libsensors_lib.so`.
No `.cpp`. Every statement about `laserReceived` below is disassembly or
a header, never recalled upstream source.

**F2. `pf_update_sensor` writes live, non-uniform, normalised weights
into `sets[current_set]` and does not flip the set. (measured)**
Disassembly of `pf_update_sensor` at `libpf_lib.so+0x1aa0`:
`movslq 0x18(%rbx),%rax` (`current_set`), `lea (%rax,%rax,8)` +
`shl $0x4` (×144 = `sizeof(pf_sample_set_t)`),
`lea 0x20(%rbx,%rax,1),%r12` (`sets` at +0x20), then `call *%rcx` — the
sensor model — and a loop striding `shl $0x5` (32 = `sizeof(pf_sample_t)`)
writing `movsd %xmm0,0x18(%rax)`, which is the `weight` field
(`pf_vector_t pose` is 24 = 0x18 bytes, `pf.hpp:61-68`).

**F3. The resample is guarded by `resample_count_ % resample_interval_`.
(measured)**
`libamcl_core.so`, inside `laserReceived` (`0xe37b0`):

```
e404a:  mov    %eax,0x8e0(%rbx)      ; resample_count_
e4050:  idivl  0xac8(%rbx)           ; ... % resample_interval_
e4056:  test   %edx,%edx
e4058:  je     e438a                 ; remainder 0 -> resample
e405e:  xor    %r13d,%r13d           ; else resampled = 0
```

and at the target:

```
e438a:  mov    0x410(%rbx),%rsi
e4391:  call   pf_update_resample@plt
e4396:  mov    0x8b8(%rbx),%rdi      ; pf_
e439d:  jmp    e4061                 ; rejoins the common path
```

**F4. The pointer handed to `publishParticleCloud` is
`&pf_->sets[pf_->current_set]`, computed *after* the conditional
resample, on both paths. (measured)**

```
e4061:  movslq 0x18(%rdi),%rax        ; current_set   (pf.hpp: +0x18)
e4065:  lea    (%rax,%rax,8),%rax
e4069:  shl    $0x4,%rax              ; x144
e406d:  lea    0x20(%rdi,%rax,1),%rax ; sets          (pf.hpp: +0x20)
e4072:  mov    %rax,-0xa58(%rbp)      ; -> publishParticleCloud arg
```

The skip path falls through at `e405e`; the resample path jumps back to
this same `e4061`. **This is the whole mechanism.** The struct offsets
+0x18 / +0x20 / stride 144 are forced by `pf.hpp`'s declaration order
(`min_samples`, `max_samples`, `pop_err`, `pop_z`, `current_set`,
`sets[2]`) **(derived)**.

**F5. The cloud publish is gated on `force_update_` only — never on
whether the resample ran. (measured)**

```
e4106:  movzbl 0x5c8(%rbx),%eax      ; force_update_ (amcl_node.hpp:236)
e410d:  test   %al,%al
e410f:  je     e43a2                 ; if (!force_update_) publish
...
e43ac:  call   publishParticleCloud@plt
```

**F6. The weight is copied into the message verbatim — no scaling, no
renormalisation. (measured)**
The copy loop in `publishParticleCloud` strides 0x20 (source
`pf_sample_t`) against 0x40 (destination `nav2_msgs/Particle`: 7 doubles
of pose + 1 weight) and ends:

```
e2a58:  movsd  -0x8(%r15),%xmm0      ; sample.weight
e2a5e:  movsd  %xmm0,-0x8(%r14)      ; particle.weight
```

**F7. There is exactly ONE call site each for `pf_update_resample`,
`publishParticleCloud` and `publishAmclPose` in the entire library, and
all three are inside `laserReceived`. (measured)**
`objdump -dC` over all 182,654 lines, filtered to `call` instructions:
`e4391`, `e43ac`, `e4130`. `laserReceived` spans `0xe37b0` to
`initParameters` at `0xe49e0`.

**F8. `initParameters` contains a hard-coded fallback
`resample_interval_ = 1` behind a WARN-severity logging block.
(measured)**

```
e5531:  mov    $0x1e,%esi            ; 30 = RCUTILS_LOG_SEVERITY_WARN
e5536:  call   rcutils_logging_logger_is_enabled_for@plt
e5545:  test   %bl,%bl
e5547:  jne    e5708                 ; emit the warning
e554d:  movl   $0x1,0xac8(%r12)      ; resample_interval_ = 1
```

This is why a live readback off the running `/amcl` is mandatory and a
config-file check is not sufficient. C2-NAV.33's
`c2nav33_liveparam.sh` exists for exactly this.

**F9. The pose publish is gated on the `resampled` accumulator; the cloud
publish is not. (measured)**
`e4115: or %r13d,%r15d` accumulates `resampled` into `r15d`; `e3dbb: test
%r15b,%r15b` then gates the region that reaches `getMaxWeightHyp` and
`publishAmclPose` at `e411d`/`e4130`.

**F10. `nav_bench.py` records no wheel odometry and installs no TF
listener. (measured)**
Its only `Odometry` subscription is `/model/coco/odometry`
(`nav_bench.py:570-571`), which the xacro documents as the gz
`OdometryPublisher` **ground-truth world pose**. There is no
`TransformListener`, no `Buffer`, no `lookup_transform` anywhere in the
file. The `odom → base_footprint` transform AMCL actually integrates is
recorded nowhere, in any artifact, in any session.

**F11. The wheels are integrated with a tuned skid-steer correction, and
AMCL is told the robot is a differential drive. (measured)**
`gazebo_models/urdf/coco_controllers.yaml`:

```
wheel_separation: 0.274        wheel_radius: 0.0585
wheel_separation_multiplier:   1.10
publish_rate: 50.0             open_loop: false
odom_frame_id: odom            base_frame_id: base_footprint
enable_odom_tf: true
```

`gazebo_models/config/nav2_params.yaml:33`:
`robot_model_type: "nav2_amcl::DifferentialMotionModel"`.

**F12. The repo already documents skid-steer rotation error as real, and
tuned SLAM around it — at thresholds 2–2.5× finer than AMCL's.
(measured)**
`gazebo_models/config/slam_params.yaml:22-25, 30-32`:

> "Fine thresholds: process scans often during turns so the matcher can
> correct skid-steer odometry drift before it accumulates."
> `minimum_travel_distance: 0.1`, `minimum_travel_heading: 0.1`
> "Wider correlation search so the matcher can pull the pose back when
> skid-steer odometry under/over-rotates during in-place turns."

AMCL runs `update_min_d: 0.25` and `update_min_a: 0.2` — **2.5× and 2×
coarser** than the thresholds chosen to keep this exact error bounded.

**F13. Nothing is left in a diagnostic state. (measured)**
`gazebo_models/config/nav2_params.yaml:32` and
`docs/data/c2nav25_slow_params.yaml:32` both read `resample_interval: 1`.
Only `docs/data/c2nav33_ri2_params.yaml:32` reads `2`, and nothing that
runs a mission or a normal benchmark references it.

---

## EVIDENCE

Reproduced offline in this session, clean ROS graph, no simulator:

| tool | result |
|---|---|
| `c2nav33_weights.py selftest` | **62 passed, 0 FAILED** (measured) |
| `c2nav33_weights.py gate` | **PASSED**, including part B which *executes* `libpf_lib.so` (measured) |
| `c2nav33_weights.py phase` | `open_space` 21 msgs / 10 pre / 21-of-21 alternation; `wall_adjacent` 18 / 9 / 18-of-18 (measured) |
| `c2nav33_weights.py weights` | table below (measured) |
| `c2nav33_weights.py verdict` | **(B)** (measured) |

The gate's executed-filter half is worth stating separately, because it
turns F2–F6 from a reading into a demonstration **(measured)**:

- after `pf_update_sensor`: 500 distinct weights of 500, summing to
  1.000000000000, **ESS/n = 0.7777**
- after `pf_update_resample`: **1** distinct bit-identical value at
  exactly 1/n, ESS/n = 1.000000000000001, and `current_set` flips 0 → 1

That second line is C2-NAV.30's independent measurement (ESS/n = 1.0000
on 40 of 40 published clouds) arrived at from the other direction.

**The headline, `wall_adjacent`, 9 pre-resample clouds (measured):**

| quantity | median | mean | 95 % CI of mean |
|---|---|---|---|
| unweighted dy | −0.0884 m | −0.1002 m | — |
| weighted dy | −0.0906 m | −0.1027 m | — |
| **shift** (w − u) | **−0.00266 m** | −0.00250 m | **[−0.00638, +0.00139]** |
| mass_shift | +0.0174 | +0.0160 | [+0.01167, +0.02042] |
| ESS/n | 0.9377 | 0.9361 | — |
| weight entropy | 0.9948 | — | — |
| max normalised weight | 0.0029 (1.6× uniform on 569 particles) | — | — |

Null control on the post-resample clouds of the same legs, where the
weights are flat by construction: `shift` max |·| = **1.332e-15**,
`mass_shift` max |·| = **5.551e-16** **(measured)**. The pipeline cannot
manufacture a shift.

`corr(shift, dy_u)` at `wall_adjacent` = **−0.7113** **(measured)** — the
weighting is a *restoring* signal, strongest north exactly when the cloud
is furthest south.

**Standing eliminations, all by measurement:**

| session | mechanism | verdict |
|---|---|---|
| C2-NAV.29 | scan-to-map registration, sensor extrinsics | eliminated; GT scores higher in 90/90 at `wall_adjacent` |
| C2-NAV.30 | particle depletion, collapse, multimodality | eliminated; GT inside the cloud 19/19, 36.4 % of particles north of it |
| C2-NAV.31 | likelihood field + map geometry | minority: 29 % by the mean, and the genuine dilation contributes 0.4 % |
| C2-NAV.32 | `DifferentialMotionModel` sampling | negligible; 95 % CI straddles zero, mean points **north** |
| C2-NAV.33 | importance weighting before resampling | **(B)**; 3.0 % of the bias, CI straddles zero, correlation restoring |

---

## AMCL weighting/resampling path

The path, as executed, with each step's evidence tag:

```
laserReceived(scan)                                    [libamcl_core.so 0xe37b0]
│
├─ motion update: DifferentialMotionModel::odometryUpdate  (F11; measured
│    by C2-NAV.32 against libmotions_lib.so, alphas all 0.2)
│    consumes  odom -> base_footprint  from TF          <-- NOT RECORDED (F10)
│
├─ if (moved past update_min_d 0.25 m OR update_min_a 0.2 rad)
│  │
│  ├─ sensorUpdate -> pf_update_sensor(pf, LikelihoodFieldModel, ldata)
│  │     writes normalised, NON-UNIFORM weights into
│  │     pf->sets[pf->current_set].samples[i].weight        (F2, measured)
│  │     current_set is NOT flipped                          (F2, measured)
│  │
│  ├─ ++resample_count_;                                     (F3, measured)
│  │  if (resample_count_ % resample_interval_ == 0) {
│  │      pf_update_resample(pf, random_pose_data);           (F3, measured)
│  │      // levels weights to exactly 1/n, flips current_set (gate B, measured)
│  │      resampled = 1;
│  │  } else {
│  │      resampled = 0;                                      (F3, measured)
│  │  }
│  │
│  ├─ set = &pf->sets[pf->current_set];   // AFTER the conditional  (F4, measured)
│  │
│  ├─ if (!force_update_) publishParticleCloud(set);           (F5, measured)
│  │      copies sample.weight -> particle.weight VERBATIM     (F6, measured)
│  │
│  └─ if (resampled || ...) { getMaxWeightHyp -> publishAmclPose } (F9, measured)
```

**Why `resample_interval: 2` genuinely exposes pre-resampling weights, and
why this is a mechanism rather than a hope.** F4 and F5 together: the
published pointer is re-derived from `current_set` **after** the
conditional resample, and the publish is not gated on `resampled`. So on
every update where the modulo is non-zero, `publishParticleCloud` receives
the set `pf_update_sensor` just weighted, and F6 copies those weights out
untouched.

That is not an inference from behaviour — it is the instruction sequence.
And it was then **confirmed live**: the published clouds alternate
`RpRpRpRpRpRpRpRpRpRpR` / `pRpRpRpRpRpRpRpRpR` — alternation runs
**21 of 21** and **18 of 18** messages, i.e. every adjacent pair differs
in both legs — with phase classified from a **bit-level all-equal test on
the weights themselves** and never from the config **(measured)**. The two legs continue **one** sequence — `open_space` ends
`R`, `wall_adjacent` begins `p` — which is what a member counter reset only
at filter (re)initialisation must do.

**Answer to brief item 3: YES, and it is now measured rather than
predicted.** The caveat that survives: interval 2 exposes the weights of
the **skipped-resample** updates only. The alternate updates' weights are
still destroyed before publication, and no setting of `resample_interval`
exposes *every* update's weights.

### One correction, changing no measurement

`docs/data/c2nav33_extra.py:85-88` prints that the `publishAmclPose` block
"is entered by a conditional jump on it (`e3fa5 jne e411d`)". The `or
%r13d,%r15d` at `e4115` is correct, and the conclusion is correct, but
`e3fa5`'s `jne` tests `%al` returned by `getMaxWeightHyp` at `e3f9e` —
the **inner** guard. The test on the `resampled` accumulator is
`e3dbb: test %r15b,%r15b` **(measured, this session)**.

This is a prose line in a `print`, not a checked assertion, and the
load-bearing evidence for the halved pose rate is the measured
**0 of 19** pre-resample clouds carrying a fresh pose against **12 of 20**
post-resample ones — which is unaffected. The exact boolean that ORs into
`r15d` alongside `resampled` (flags at `+0x8b0`, `+0x491`, `+0xb00`) is
**not** established here and should not be quoted.

---

## What is currently unobservable

1. **The odometry AMCL actually integrates.** `odom → base_footprint`,
   published by `diff_drive_controller` at 50 Hz with `enable_odom_tf:
   true` (F11). Recorded in no artifact of any session (F10). C2-NAV.32
   named this its largest limitation and substituted ground truth; its
   `noise` term was constructed to be invariant to the substitution, but
   its `geom` term — the *mean* increment — was computed **from ground
   truth**, so by construction it cannot show an odometry bias. **This is
   now the only unmeasured input to the filter.**

2. **The weights of resample updates.** Structural (see above). No
   `resample_interval` exposes them.

3. **Pre-resample clouds while stationary.** 0 of 19 were captured, and
   that is structural too: `update_min_d`/`update_min_a` gate the update,
   and no update means no cloud. C2-NAV.33 correctly reports option (F) as
   **UNTESTED**, not confirmed.

4. **`convertMap`'s half-cell term** (C2-NAV.31) remains derived — the
   header is installed, `amcl_node.cpp` is not.

5. **Any rate.** Every cloud result rests on n = 1 run.

---

## Proposed measurement

**Measure the wheel odometry AMCL consumes, against ground truth, across
the `open_space` → `wall_adjacent` transition.**

This is the last unmeasured input, and the C2-NAV.29→.33 eliminations
force it. The reasoning chain, stated so it can be attacked:

- The published weights are **nearly flat** — ESS/n 0.9377, entropy
  0.9948, max weight 1.6× uniform **(measured, C2-NAV.33)**. So the
  observation model has almost no authority over where the cloud sits.
- The motion model's **noise** term is zero-mean and slightly northward
  **(measured, C2-NAV.32)**.
- With `recovery_alpha_slow/fast = 0.0` no random particles are injected
  **(measured, C2-NAV.32)**.
- Therefore, between the weak likelihood pull and the zero-mean noise,
  the thing that actually *moves* the cloud is the **mean** odometry
  increment — and that is the one quantity substituted by a proxy in
  every analysis so far.

**Hypothesis (labelled as such, not a finding).** The bias is
skid-steer heading error in the integrated wheel odometry, arrested by
the weak likelihood at the edge of the region the observation model
cannot resolve. Three things make it worth testing before anything else:

- The repo already documents this error as real on this robot and tuned
  SLAM's thresholds *specifically* to bound it, at 0.1 m / 0.1 rad — while
  AMCL runs 0.25 m / 0.2 rad (F12).
- C2-NAV.32 found `|dyaw|` clustering at 0.15–0.30 rad around the 0.2
  threshold, with **8 of 18** updates triggered by `update_min_a` and
  **0 of 18** by `update_min_d` — it is the *rotation* threshold that
  fires on these legs **(measured, C2-NAV.32)**.
- C2-NAV.31 measured the 99 % likelihood plateau at `wall_adjacent` as
  **0.110 m wide, spanning [−0.085, +0.060] m**. The observed cloud
  centroid is **−0.0884 to −0.0921 m** — i.e. sitting essentially **at the
  southern edge of the plateau**. Consistent with an arrest, and it
  predicts place-dependence through plateau shape.

I am flagging the arrest half explicitly as **(hypothesis)**. The
numerical coincidence between the plateau edge and the bias is suggestive
and is not evidence.

---

## Exact proposed experiment

**Part A — instrumentation, offline, gated before any simulator runs.**

Append to `nav_bench.py`'s 10 Hz per-leg trace, following C2-NAV.28's and
C2-NAV.30's appending rule **exactly** (columns appended at the end, so
every existing column index is unchanged; last sample in the half-open
bucket `(t−0.1, t]`; **blanks when the bucket is empty, never a forward
fill**):

| column group | source |
|---|---|
| `odo_x`, `odo_y`, `odo_yaw` | `odom → base_footprint` via a `tf2_ros.TransformListener` — the transform AMCL itself integrates |
| `wodo_x`, `wodo_y`, `wodo_yaw`, `wodo_vx`, `wodo_vyaw` | the `nav_msgs/Odometry` published by `diff_drive_controller` |

Both, not one: the TF is what AMCL reads, the topic carries the twist and
the covariance, and recording both makes them cross-checkable.

Then extend `ros_clean.sh` if — and only if — the change adds a process.
It does not; this is a subscription inside an existing node. Stated
because CLAUDE.md requires the check, not because it fires.

**The blindness guard, and it is mandatory.** CLAUDE.md: *any check whose
success condition is "we saw nothing" must first prove it can see
something.* A TF listener that never matches yields blank columns that
read exactly like "the odometry did not drift". So the run must assert
**post-run** — not pre-run, which is the false-negative C2-NAV.30 already
paid for when a pre-run guard printed `particle cloud: NOT SEEN` before 40
messages arrived — that at least one `odo_*` row is non-blank per leg, and
abort the analysis otherwise.

**Part B — the run.** One tour, fresh simulator, topology A,
`open_space` → `wall_adjacent`, the same route and 75 s cap C2-NAV.28's,
.30's and .33's focus runs used, under
`docs/data/c2nav25_slow_params.yaml` (sha256 `4c15893e…`), so the run is
directly comparable to `c2n30_focus_r1` and `c2n33_focus_r1`.

**Part C — the analysis.** For each filter update, compare:

```
odo_dy_cum   = (odom-integrated y travel since leg start)
             - (ground-truth y travel since leg start)
cloud_dy     = particle-cloud centroid y  -  ground-truth y
```

and find the update at which `cloud_dy` first crosses −0.05 m, then test
whether `odo_dy_cum` has crossed a comparable magnitude by that update.
`open_space` is the discriminating leg, exactly as it was for C2-NAV.32:
its first cloud in the C2-NAV.30 run was effectively a point mass — sd
0.0002 m, dy **+0.0119 m** — so the displacement is *created* during that
leg and the replay has to reproduce its creation, not merely carry an
inherited offset along.

---

## Exact parameter change, if any

**NONE.** Not one leaf.

The experiment is a pure instrumentation addition. It runs under the
existing `c2nav25_slow_params.yaml`, whose amcl block is byte-identical to
the shipped `gazebo_models/config/nav2_params.yaml`. A `paramdiff` over
all 323 leaves must report **added 0, removed 0, changed 0**, and that
should be run and shown before the simulator starts, exactly as C2-NAV.33
showed its one leaf.

`resample_interval: 2` is **not** carried forward. It is a diagnostic
window that halves the `/amcl_pose` rate **(measured, C2-NAV.33: 0 of 19
pre-resample clouds carry a fresh pose against 12 of 20 post-resample)**,
and it is not a candidate configuration. Nothing needs reverting today
(F13).

---

## Expected result

Under the hypothesis: `odo_dy_cum` grows through `open_space` and reaches
a magnitude comparable to the observed cloud displacement at, or before,
the update where `cloud_dy` crosses −0.05 m; and the growth is dominated
by accumulated heading error rather than translation error, so
`odo_dyaw_cum` should be the leading term.

Under the alternative: `odo_dy_cum` stays small — say under 0.02 m —
while `cloud_dy` reaches −0.09 m, and the odometry is eliminated with the
other four.

**I am not predicting which.** Both outcomes are informative and the
falsifier below is written to make the second one publishable rather than
a dead end. Every AMCL-internal mechanism has now measured negligible, so
a small odometry error would leave the bias genuinely unexplained by any
term yet examined — which is itself the finding, and would point next at
the leg-boundary/initial-condition question C2-NAV.33 raised.

---

## Falsifier

Fixed before the run, and the discriminator is a *comparison*, not a
threshold on one number:

- **The hypothesis is FALSIFIED** if, at the update where the cloud
  centroid first reaches −0.05 m, the cumulative odometry y-error
  `|odo_dy_cum|` is **less than one third** of `|cloud_dy|`. Odometry
  then cannot be supplying the displacement, and joins the eliminated
  list.
- **The hypothesis SURVIVES** (it is not confirmed by one run) if
  `|odo_dy_cum| ≥ |cloud_dy|/3` at that update **and** the sign agrees.
  Sign agreement is required: C2-NAV.32's motion-model replay was rejected
  partly for pointing north, and the same standard applies here.
- **The experiment FAILED, and the answer is neither** — reported as
  UNOBSERVABLE, not as "the odometry is clean" — if the post-run guard
  finds no non-blank `odo_*` row, or if TF lookups fail on more than 10 %
  of buckets. This is the C2-NAV.33 (C)-vs-(B) distinction and it must be
  a distinct outcome in the tool, not a silent fall-through. C2-NAV.33 hit
  exactly this bug — with `.navbench` absent its verdict printed
  **(C) UNOBSERVABLE**, reporting a missing artefact as a finding — and
  fixed it; the same trap is live here.

**A null control is required**, matching C2-NAV.33's, which returned
1.332e-15: replay the analysis with `odo_*` set equal to ground truth. It
must return `odo_dy_cum ≡ 0` at machine epsilon. An arithmetic that cannot
return zero on a zero input cannot be trusted to return a real number on a
real one.

---

## Risks

1. **`open_space`'s point-mass first cloud may not recur.** It happened in
   `c2n30_focus_r1` because the leg began just after AMCL initialised.
   That is a property of the run, not the route. If the new run's first
   cloud is already displaced, the leg loses its discriminating power and
   the experiment answers a weaker question. *Mitigation:* record it and
   say so; do not re-run until it comes out convenient.

2. **A TF listener is a new failure surface in `nav_bench.py`**, an
   instrument that has produced thirty-three commits of results.
   `lookup_transform` raises on extrapolation, and a broad `except` would
   silently blank the columns. *Mitigation:* the columns are appended, no
   existing column moves, and the selftest asserts the frozen column list
   from the committed `c2nav22_yaw.json` rather than a retyped one — the
   pattern C2-NAV.30 and .33 already use.

3. **n = 1 again.** One run is an observation, not a rate. This has been
   true of every cloud result since C2-NAV.30 and should be stated in the
   entry rather than discovered by a reader.

4. **The `WORLD_TO_MAP` 56 mm x discrepancy is open for a fifth session.**
   It cannot affect this experiment — every quantity is a *difference* of
   two y values in one frame, so a constant offset cancels exactly, the
   same argument C2-NAV.32 made. But it still needs a decision rather than
   a patch, and it should not be quietly fixed inside this work.

5. **A concurrent session is active in this worktree.** A second Claude
   job was observed running the C2-NAV.28–.33 selftests read-only during
   this investigation. Nothing was coordinated with it; anyone committing
   here should check `git status` first.

6. **C2-NAV.30's selftest allow-list still fails** on files from
   C2-NAV.31/.32/.33 — pre-existing, recorded in the log twice, not caused
   here, and it will fail again on this session's files. It needs widening
   or retiring.

---

## Confidence

| claim | confidence | basis |
|---|---|---|
| The weighting/resampling path as drawn above | **High** | Disassembly of the deployed `.so` plus installed headers, every offset cross-checked against `pf.hpp` declaration order; call-site counts exhaustive over the whole library |
| `resample_interval: 2` exposes pre-resampling weights | **High** | Mechanism read from the binary (F4+F5+F6), *and* confirmed live by perfect pre/post alternation (21 of 21, 18 of 18) classified from the weights themselves |
| Importance weighting does not explain the bias — verdict (B) | **High** for the mechanism, **Moderate** for the magnitude | Effect is 3.0 % of the bias with a CI straddling zero, correlation −0.71 restoring, null control at 1e-15; but n = 1 run, 9 pre-resample clouds, and the CI uses the 1.96 normal quantile where t at n=9 would widen it — which only strengthens a CI that already straddles zero |
| Wheel odometry is unrecorded and is the last unmeasured input | **High** | Read directly from `nav_bench.py`; no TF listener, no wheel-odometry subscription, no rosbag anywhere (C2-NAV.29 searched) |
| Skid-steer odometry drift is the remaining cause | **Low — this is a hypothesis** | Motivated by four eliminations, `slam_params.yaml`'s own documentation, the `update_min_a`-dominated trigger pattern, and AMCL's 2–2.5× coarser thresholds. Untested |
| The plateau-edge arrest mechanism | **Very low — speculation** | A numerical coincidence between C2-NAV.31's [−0.085, +0.060] plateau and the −0.088 m centroid. Recorded so it can be tested, not leaned on |

---

## Status

Investigation complete. No behaviour changed, no parameter changed, no
simulator run, `main` untouched. One correction filed against a print
statement in `c2nav33_extra.py` that changes no measurement. The next
experiment is defined with its falsifier and its null control fixed in
advance, and it requires **zero** parameter leaves to move.

### Reproduce this handoff

```bash
cd ~/ros2_ws/src/coco-robot-ros2/.claude/worktrees/c2nav0-diagnosis

# the AMCL path, from the deployed binary
objdump -dC /opt/ros/jazzy/lib/libamcl_core.so > /tmp/amcl.dis
grep -nE '^\s+[0-9a-f]+:.*\bcall\b' /tmp/amcl.dis \
  | grep -E 'pf_update_resample|publishParticleCloud\(|publishAmclPose\('
objdump -dC --start-address=0xe4030 --stop-address=0xe4120 \
  /opt/ros/jazzy/lib/libamcl_core.so     # F3, F4, F5
objdump -d  --start-address=0x1aa0 --stop-address=0x1c10 \
  /opt/ros/jazzy/lib/libpf_lib.so        # F2

# the C2-NAV.33 evidence, offline
python3 -P docs/data/c2nav33_weights.py selftest    # 62 checks
python3 -P docs/data/c2nav33_weights.py gate        # observability proof
python3 -P docs/data/c2nav33_weights.py phase       # READ THIS FIRST
python3 -P docs/data/c2nav33_weights.py weights     # the headline
python3 -P docs/data/c2nav33_weights.py verdict     # (B)
```
