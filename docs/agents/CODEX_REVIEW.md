# C2-NAV.35 — independent resolution of the C2-NAV.34 odometry disagreement

2026-09-11. **Disposition: existing evidence does not prove an error in AMCL's exact odometry input.** The +0.00616 m statistic is an output/GT **PROXY**, not an input measurement. The claimed >=2.96% odometry lower bound is rejected. An along-track odometry contribution remains a **HYPOTHESIS**. Minimum sufficient evidence is a record of actual prediction-call inputs and anchors, paired with acquisition-stamped GT at those same times. No hook was implemented and no ROS runtime or simulator was run.

This addendum supersedes any conflicting wording below; the C2-NAV.34 review is retained as historical evidence, including its historical test results. **FACT** means inspected in this review; **DERIVED** means algebra from stated inputs; **PROXY** means a related observable without proven equivalence; **HYPOTHESIS** means untested explanation; **UNKNOWN** means missing evidence. No historical test is represented as rerun today.

## 35.1 Scope and claim resolution

**FACT:** resumed in the requested existing Codex worktree, branch `codex/c2nav34-odom-final`, HEAD `ea80496a91b192ba6734e1a7096f489f23e275f1`. Main was listed at `ea66155`, Claude at `87131a0`, and the remote diagnosis branch at `ea80496`. The pre-existing untracked `base_footprint` file is untouched. Only this review is changed; the user's review-only scope takes precedence over the generic SESSION_LOG checkpoint instruction.

Read the current HANDOFF, previous review, relevant .28–.34 analysis/recorder paths and installed evidence. The earlier claim-by-claim tables remain applicable with these explicit resolutions:

| Claim | Resolution and classification |
| --- | --- |
| F1–F4: scan-time odometry TF, zero timeout | **FACT:** supported for `getOdomPose`; “AMCL never waits for TF” rejected as a statement about the whole path. MessageFilter precedes this lookup; scanner setup also has a distinct timed transform. |
| F5: controller supplies the edge | **FACT:** configured producer; sole historical runtime authority **UNKNOWN** without captured publisher/configuration evidence. |
| F6: encoders feed odometry | **FACT:** supported. “Slip absorbed” means encoder motion is integrated without ground-slip correction, not that wheel rotation equals ground travel. |
| F7/F8: true track and cancellation | **DERIVED:** 0.274 m is joint-origin spacing; 0.243 m is collision-center spacing in the earlier geometry analysis. Neither establishes effective skid-steer track. Command cancellation is conditional on measured wheels following the limited commands at matched times. |
| E1/E2: +0.00616 m/update | **PROXY:** reproduced numerical statistic, but denominator is 523 recorded-pose intervals, not verified prediction calls. |
| E3/E4: course-aligned excess appears during approach | **PROXY:** descriptive output pattern supports where to instrument, not which input caused it. |
| E5: >=2.96% odometry error | **DERIVED:** invalid inference, explicitly rejected below. |
| E6–E9: observation cannot restore; yaw/skid eliminated | **HYPOTHESIS / UNKNOWN:** neither output yaw nor reconstructed likelihoods identify the missing prediction input. A yaw CI containing zero is not an equivalence test. |
| Laser removes one-third; command supplies 70% | **UNKNOWN:** no measured input/correction decomposition supports either attribution. |
| Three bucketed TF columns prove consumption | **DERIVED:** insufficient. Need actual query result, prediction identity and anchor history. |

**FACT:** `nav_bench.py:571–584` subscribes to model GT odometry, AMCL poses and command topics. `v_wheel` comes from `/diff_drive_controller/cmd_vel` (`TwistStamped`), not wheel encoders. At lines 647–659, both GT and AMCL are stored using `self.now()` callback times rather than their message acquisition stamps. `write_trace` retains the last AMCL sample in each half-open receipt bucket. Blanks prevent forward-fill, but do not prove one publication per bucket, lossless delivery or one output per prediction.

**PROXY:** C2-NAV.30 observes published populations; .32 uses GT-derived increments with the installed motion model, not the absent TF inputs; .33 exposes some pre-resampling weights under interval 2. These constrain their respective experiments. They cannot be combined into exhaustive elimination of sensor, motion or resampling effects in the historical interval-1 trajectory. The previous review's conditional yaw scale example likewise establishes possibility, not occurrence.

## 35.2 Exact installed AMCL path

**FACT:** package versions and both library SHA-256 values in historical section 2 were independently rechecked today and match exactly. Installed disassembly rechecks:

| Address in `libamcl_core.so` | Evidence |
| --- | --- |
| `0xe3ba6–0xe3be8` | Constructs ROS time from the scan header and passes base-frame member `+0x9d8`, current pose outputs and timestamp to `getOdomPose`. |
| `0xd6ba9–0xd6bc3` | Zeroes `xmm0`; intervening moves do not write it; calls `durationFromSec(0)`. |
| `0xd6bd2`, `0xd6c10`, `0xd6c67` | Supplies odom-frame member `+0xa88`, converts seconds to nanoseconds, dispatches buffer lookup. |
| `0xe3bfa–0xe3bfd` | Branches on lookup success before entering prediction logic. |
| `0xe3d1f`, `0xe3d2d` | Initialization stores yaw and x/y into anchor members `+0x8d8`, `+0x8c8`. Initialization is not a motion-model call. |
| `0xdda4b`, `0xdda57`, `0xdda69` | Subtracts anchor x/y and loads anchor yaw for wrapped angular difference. |
| `0xe3e92`, `0xe3f2d–0xe3f46` | Computes update eligibility; scanner flag guards virtual motion-model call. `rdi` receives model at `+0x5d0`, `rsi` PF at `+0x8b8`, `rdx` pose, `rcx` delta; call through vtable `+0x18`. |

**FACT / DERIVED:** the configured query is target `odom`, source `base_footprint`, scan header time: **T_odom_base**, mapping base coordinates into odom. Installed `buffer_interface.hpp:211–217` corroborates the default zero timeout. The identity pose transformed by `getOdomPose` yields current x/y/yaw. Configuration selects `nav2_amcl::DifferentialMotionModel`. The argument delta is `(x-anchor_x, y-anchor_y, angle_diff(yaw,anchor_yaw))` in odom axes, not a subtraction of published AMCL poses.

The tagged [AMCL implementation](https://github.com/ros-navigation/navigation2/blob/1.3.11/nav2_amcl/src/amcl_node.cpp) corroborates this binary path. Its MessageFilter targets odom at scan time with queue 10 and `transform_tolerance_` as buffer timeout (configured 0.5 s). That timeout is distinct from the zero-timeout odometry query and outgoing TF postdating. `addNewScanner` also uses the tolerance for mounting-transform lookup. Thus the earlier review's correction to “never waits” stands.

**DERIVED:** the differential model obtains translation norm and first/second rotations from these actual arguments as detailed in historical section 3. The anchor may span multiple scans. Crucially, prediction precedes `updateFilter`; its laser-angle TF or range validation can return false before anchor advancement. Therefore “successful sensor updates” cannot serve as a substitute count for actual motion-model invocations. A capture must distinguish those events, including a repeated anchor after sensor failure.

## 35.3 Controller and multiplier algebra

**FACT:** current YAML retains separation 0.274 m, radius 0.0585 m, separation multiplier 1.10, both radius multipliers 1, `open_loop=false`. Installed generated default `position_feedback=true` is not overridden. Xacro lines 517–537 expose wheel position/velocity state and velocity command interfaces through `gz_ros2_control/GazeboSimSystem`. These are simulated encoder states, not `/joint_states` subscriptions or model GT poses.

**FACT:** installed gz_ros2_control version is `1.2.17-1noble.20260412.065025`. Version-tagged [hardware source](https://github.com/ros-controls/gz_ros2_control/blob/1.2.17/gz_ros2_control/src/gz_system.cpp) binds the position state interface to `joint_position` and assigns it from Gazebo's `JointPosition` component in `read` (lines 376–382, 656–662). This is source corroboration, not a claim to have proved the exact loaded hardware binary or historical state values.

**FACT:** controller disassembly `0x6eaa6–0x6eaeb` averages feedback by wheels-per-side, tests position-feedback selection and calls `Odometry::update`. The separate `updateOpenLoop` call is at `0x6eb31`. Tagged [controller source](https://github.com/ros-controls/ros2_controllers/blob/4.39.0/diff_drive_controller/src/diff_drive_controller.cpp) corroborates these branches and applies inverse kinematics to limited commands. Tagged [odometry source](https://github.com/ros-controls/ros2_controllers/blob/4.39.0/diff_drive_controller/src/odometry.cpp) scales measured position differences and integrates displacement; rolling averages affect twist.

**DERIVED:** let `B=.274*1.10=.3014`, `R_l=R_r=.0585`, and `q_l,q_r` be side-averaged measured wheel angles. Then `l=R_l*Δq_l`, `r=R_r*Δq_r`, `ds=(l+r)/2`, `dtheta=(r-l)/B`. Commands are `qdot_l*=(v*-B*w*/2)/R_l`, `qdot_r*=(v*+B*w*/2)/R_r`. Only substituting **measured** velocities equal to these setpoints produces `v_odom=v*`, `w_odom=w*`. Neither equality follows from command telemetry. For fixed encoder travel, the multiplier reduces yaw magnitude by 1/1.10; relative to GT its sign depends on contact slip/effective track. Four-wheel lateral scrub and longitudinal slip remain unmeasured, condition-dependent effects.

## 35.4 Numerical claims and their status

**FACT / PROXY:** reran `c2nav34_odom.py incr`: 523 intervals, 21 legs, mean along residual +0.00616 m, nominal interval [+0.00342,+0.00890], yaw +0.00271 rad with interval [−0.00647,+0.01190]. The formula separately computes `T_previous^-1*T_current` for AMCL and GT, then subtracts components. A constant rigid frame transform cancels within each relative transform; different estimated/true headings, sampling, filter corrections and receipt-time mismatch do not. The nominal independent-sample CI does not account for clustered/serial observations. **UNKNOWN:** mean residual of actual consumed odometry, and whether it equals +0.00616 m per prediction.

**FACT:** `cmd` reads a saved statistic, not raw encoder data. Bundle slope is **0.971214**, n=253; `build` computes `sum(v_wheel*v_act)/sum(v_wheel²)` in selected command-hold windows. No raw `.navbench` archive was available in this worktree to rebuild that regression.

**DERIVED:** `100*(1/.971214-1)=2.963919383369684%`, explaining the printed 2.96%. Using rounded 0.9712 gives 2.9654036243822235%. The reciprocal measures command-relative-to-GT scale under the regression assumptions; it is not a measured distance integral or an odometry lower bound.

**DERIVED — decisive counterexample:** command 1 m/s, measured wheels both rolling without longitudinal slip at 0.971214 m/s for 1 s. GT and encoder odometry both advance 0.971214 m, so odometry error is **zero**, while the command/GT slope is exactly the reported value. This disproves the bound. Slip could produce an error, but its sign/magnitude cannot be added to a command-tracking shortfall. The claimed 4.7 percentage-point remainder and 70% attribution consequently fail too.

## 35.5 Minimum instrumentation and C2-NAV.36 proposal

**DERIVED — minimum for the exact-input question:** a default-disabled AMCL event hook plus a separate acquisition-stamped GT record, joined offline. Do not inject GT into localization or wait for GT inside AMCL. Capture:

1. Session/clock epoch, monotonic event ID, scan header stamp/frame and scanner identity; distinguish repeated stamps. Record lookup success/failure and skipped/initializing callbacks, with reason and loss counters.
2. The **existing successful lookup result**, including frame IDs, query and returned timestamps, full pose/quaternion and actual planar pose. Do not perform a second lookup and label it the consumed result. A returned scan-time stamp is not the producer's last sample timestamp; raw TF brackets are optional corroboration.
3. Immediately before each existing `odometryUpdate` call: actual `pose`, `delta`, prior `pf_odom_pose_`, anchor event ID/acquisition time, initialization/force/scanner flags and prediction ID. Log anchor initialization/reset/advancement separately, plus prediction completion and sensor success/failure. Record MessageFilter drops separately from `getOdomPose` failures.
4. Full GT pose, parent/child frame, original acquisition stamp and receipt clocks, with verified model-to-base extrinsics. Retain bracketing GT samples for **both anchor and current scan times**, interpolation method, gap and uncertainty. Never substitute nearest receipt-time GT without bounding timing error.

An external scan-stamped `nav_bench` tf2 listener can independently measure the published odometry trajectory. It cannot prove AMCL's buffer contained the same transforms, which callbacks reached prediction, or which anchor was used. A latest-TF timer adds varying temporal lag and loses scan/anchor identity; three bucketed columns are therefore insufficient for this question. A listener can corroborate an internal record, not replace it without a separate proof of identical consumption.

**DERIVED:** the previous review's full raw TF/scans, particle/output association and counterfactual replay are useful for broader causal attribution, but are **not all minimum requirements to measure an exact-input error**. GT synchronization is mandatory; full particle weights, command and encoder capture are optional for that narrower measurement. To demonstrate that input error *caused the output bias*, rather than merely existed at sufficient scale, add prediction/sensor/resampling population stages and reproducible replay state, scans/map/configuration and random-state handling. Keep these two evidentiary levels distinct.

**C2-NAV.36 — exact proposal, not implemented:** prepare a version-matched isolated AMCL source overlay containing only the disabled event hook, bounded nonblocking queue/drop accounting and separate GT sidecar/schema described above. Validate offline: known rigid-frame motion, zero-error GT control, injected signed translation/yaw error, yaw wrap, duplicate/out-of-order stamps, failed lookup, initialization/reset, unchanged anchor after sensor failure, and queue overflow. Confirm identical filter outputs/RNG progression with capture off/on on identical offline inputs. Document residual scheduling overhead; offline equality alone does not prove live timing neutrality. Preserve existing 58-column trace and all behavioral leaves. Deliver the reviewed instrumentation and offline checks before any separately authorized live capture. Do not tune the controller or carry interval 2 into the baseline.

## 35.6 Falsifier and confidence

**DERIVED — predeclared discriminator:** over the whole approach and wall transition, compare each actual anchor-to-scan relative odometry motion against matched GT in a common base convention, including along, lateral and wrapped yaw components. Preserve overlapping anchor intervals after failures rather than summing them as disjoint physical travel. Report systematic error with synchronization/interpolation uncertainty and run-level dependence; do not reuse the historical output CI as an input acceptance band.

The specific +0.00616 m-per-prediction input hypothesis is rejected if complete matched capture gives an uncertainty interval excluding that value. A claimed positive systematic input error is rejected to the experiment's resolution if a predeclared equivalence interval around zero contains the measured residual and its uncertainty. Choose that resolution from the instrumentation error budget before collecting data. If capture has missing events/anchors/GT or uncertainty comparable to the proposed effect, report **UNKNOWN/UNOBSERVABLE**, not a clean-input verdict.

Small terminal drift, one nonblank sample, or a 2%/4% threshold alone does not falsify every odometry-mediated explanation. For the stronger causal claim, require time-resolved propagation and an offline GT-input counterfactual: failure to reproduce the observed bias, or its persistence after replacing input with GT within quantified uncertainty, rejects that proposed explanation under the replay assumptions. No single new run retrospectively proves the unrecorded historical input.

**Confidence:** high in the exact query/call path and rejection of the unconditional command bound; high that the historical statistic is an output proxy; moderate in version-tagged hardware-source correspondence; unknown historical TF authority, exact input error and causal contribution. Today's checks were read-only binary/header/config inspection, tagged-source checks, offline `incr`/`cmd` and reciprocal arithmetic. Historical library probes below were not rerun. No runtime/build/behavioral test is claimed.

**Stop-condition answer:** No, the existing evidence does not prove an error in AMCL's exact odometry input. The minimum is the actual scan-stamped lookup and prediction arguments with anchor/event history, paired offline with synchronized GT at those same base origins and times, with explicit failure and loss accounting.

---

# Historical C2-NAV.34 — independent odometry / AMCL input review

2026-09-09. **Review only: repository, installed headers/binaries, offline arithmetic and library probes. No ROS node, navigation, AMCL or simulator was started. No behavioral configuration changed.**

**Answer: UNKNOWN whether the actual AMCL-consumed odometry quantitatively explains the observed localization bias. The necessary time-stamped transforms and consumption events are absent from the historical evidence. A separation multiplier can produce a heading error of sufficient scale under explicit assumptions, but neither its real-world sign nor its contribution to the AMCL bias follows from the multiplier alone.**

Labels: **FACT** = directly inspected or tested this session; **DERIVED** = calculation from stated inputs; **HYPOTHESIS** = plausible untested mechanism; **UNKNOWN** = missing evidence. Historical numerical findings are identified as historical, not new live measurements.

## 1. CLAUDE CLAIMS

Reviewed the original handoff at `8a7b090`, C2-NAV.28–33 in SESSION_LOG, their analysis code/bundles, and the prior independent CODEX_REVIEW at `1d17a13`. **During final delivery, Claude published a new C2-NAV.34 handoff and analysis at `87131a0`; those were also read and independently challenged before this review was committed.**

### Latest C2-NAV.34 handoff (`87131a0`)

| Major claim | Classification | Independent assessment |
| --- | --- | --- |
| F1–F4: scan-stamped base-to-odom lookup with zero local timeout | SUPPORTED with one correction | Lookup call agrees with independent evidence. “AMCL never waits for TF” is too broad: the preceding MessageFilter waits for transform availability with configured buffer timeout. |
| F5: controller is sole TF publisher | PARTIALLY SUPPORTED | Sole configured producer in this launch; historical runtime exclusivity is UNKNOWN. |
| F6: measured position feedback feeds odometry | SUPPORTED | Source, installed binary and compiled probe agree. This directly undermines the unconditional command-equality claims below. |
| F7: 0.274 m is the true physical track | PARTIALLY SUPPORTED | It is joint-origin spacing. The script hardcodes those origins and omits cylinder offsets; the actual collision-center spacing is 0.243 m. Effective skid-steer track is UNKNOWN. |
| F8: multiplier cancels, so reported yaw always equals commanded yaw | UNSUPPORTED as unconditional identity or rejection of yaw bias | Both paths use the multiplier, but commanded wheel velocity is not measured encoder displacement. Cancellation requires perfect tracking and matched timing. Even then odom-minus-GT yaw may be nonzero. |
| E1/E2: positive along-track AMCL-minus-GT increment statistic | SUPPORTED as descriptive arithmetic | Reproduced 523 intervals, mean +0.00616 m, nominal CI [+0.00342,+0.00890]. These are intervals between recorded pose samples, not proven individual filter updates or input errors. Serial dependence and timing limit the CI. |
| “Yaw residual measures zero”; therefore error is translational | UNSUPPORTED | Reproduced mean is +0.00271 rad with CI [−0.00647,+0.01190], not zero/equivalence. It is corrected AMCL-output yaw, not the unobserved odometry yaw. |
| E3/E4: southbound output excess and inherited wall-leg offset | PARTIALLY SUPPORTED | Descriptive pose/route pattern motivates along-track capture; it cannot identify the input source or exclude pre-leg heading effects. |
| E5: command slope 0.9712 implies at least 2.96% odometry over-report | UNSUPPORTED | Frozen slope is reproduced as a saved statistic, not independently rebuilt from missing CSVs. The lower-bound inference is false for encoder feedback; see the counterexample in section 6. |
| E6: plateau makes along-track error uncorrectable | HYPOTHESIS / UNKNOWN | A broad offline likelihood plateau does not prove zero restoring authority over a coupled sequence. |
| E7/E8/E9: heading propagation, lateral skid and multiplier eliminated | UNSUPPORTED as causal eliminations | Integrals use corrected/held AMCL heading and GT proxies, not the actual prediction input. Output residuals cannot isolate these mechanisms. |
| 2.13 m southbound × 4.2% ≈ 0.09 m | SUPPORTED as conditional scale arithmetic | 0.08946 m follows; the 4.2% odometry gain is unmeasured. The claimed command channel supplies 70% is unsupported. |
| Filter retains 5.2% of 7.7%, so laser removes roughly one-third | UNSUPPORTED | Ratios of different output summaries do not measure the missing correction term. |
| Residual = odometry error + laser correction | PARTIALLY SUPPORTED as schematic | It also hides motion-model sampling, resampling/hypothesis changes, initial heading and time/frame composition. Separately differenced body-frame components do not give an exact additive two-term attribution. |
| Three appended TF columns suffice; <2% / >=4% gain settles the question | PARTIALLY SUPPORTED for screening; UNSUPPORTED for exact causation | Need scan timestamps, accepted-update events, paired GT, heading and loss coverage. At least one nonblank row is inadequate. Ratios near zero net travel need explicit rejection. |
| Selftest validates geometry/cancellation and one recorded row = one update | UNSUPPORTED beyond tested arithmetic | All 18 checks pass, but geometry is checked against hardcoded values, not collision transforms. No absence of forward fill proves zero message loss or one publication per bucket. |
| Full rebuild reproducible from committed artifacts | UNKNOWN / limited | `incr` reproduces from .28; .34 `build` requires uncommitted CSVs. This worktree has no such raw archive. Claude explicitly acknowledges this limitation. |

The original C2-NAV.33 handoff claims are retained below because they underpin the latest causal eliminations.


| Major claim | Classification | Independent assessment |
| --- | --- | --- |
| F1: only AMCL headers and libraries are installed | SUPPORTED | Debian package has no implementation source; version-tagged upstream source is separately available and was downloaded. “No source on this machine” is no longer true after download. |
| F2–F8: normalization, modulo resampling, current-set selection, cloud weight copy | SUPPORTED, within scope | Installed binary inspection and existing offline tests support these mechanisms; none establishes what odometry was consumed historically. |
| F9: pose publication depends on resampling | PARTIALLY SUPPORTED | Also initialization/forced publication and first-pose handling. Source has `resampled || force_publication || !first_pose_sent_`; an unconditional “only on resampling” is false. |
| F10: recorder lacks wheel odometry / TF | SUPPORTED for inspected recorder and committed evidence | Its sole Odometry subscription is ground truth. Universal claims about every external archive are UNKNOWN. |
| F11: wheel TF feeds AMCL; differential motion model; multiplier 1.10 | SUPPORTED as configured path | Independently verified below; historical loaded parameter values and TF authority still need capture. |
| F12: SLAM uses finer thresholds to handle skid steer | PARTIALLY SUPPORTED | Config values/comments support design intent. They do not measure current odometry error; AMCL's distance test is componentwise, not Euclidean distance. |
| F13: shipped/baseline interval remains 1 | SUPPORTED | Interval 2 remains confined to the diagnostic parameter file. |
| “All per-update mechanisms eliminated”; odometry is the only unmeasured input | UNSUPPORTED | Raw scans, skipped publications, original particle ancestry, alternate pre-resampling weights and exact event alignment remain unobserved. Proxy replay does not eliminate coupled effects. |
| C2-NAV.32 noise result is invariant to replacing odometry with GT | UNSUPPORTED as a general claim | The motion decomposition and noise variances depend on the input increments. Subtracting a zero-noise result does not remove this dependence. |
| Flat weights uniquely identify post-resampling; stationary updates impossible | UNSUPPORTED | Uniform sensor scores also produce flat weights; initialization/no-motion requests are exceptions to ordinary motion gating. Prior review's counterexamples remain applicable. |
| Importance weighting cannot explain bias because CI crosses zero / correlation restores | UNSUPPORTED as causal elimination | A mean CI crossing zero is not equivalence; correlation and sparse alternate updates do not bound accumulated effects. |
| Skid-steer drift supplies the displacement | UNKNOWN | Worth measuring; not forced by previous eliminations. |
| Likelihood plateau arrests the error and explains place dependence | HYPOTHESIS / UNKNOWN | No coupled time-resolved measurement. The handoff's quoted endpoints and width also must not be treated as one representative interval: endpoints −0.085 to +0.060 span 0.145 m, not 0.110 m; aggregates can differ. |
| Append latest TF and wheel-odom columns at 10 Hz to measure exact input | PARTIALLY SUPPORTED | Useful telemetry, insufficient exact scan-time and accepted-update evidence. Wheel-odom topic is optional corroboration, not the minimum input source. |
| Less than one-third cumulative y error falsifies odometry causation | UNSUPPORTED | Arbitrary ratio, missing heading/frame/timing effects, discarded pre-leg history and AMCL corrections. A small terminal y residual can hide large intermediate errors. |
| “10% wheel-separation error” / “can produce ~0.09 m” | PARTIALLY SUPPORTED only as conditional arithmetic | These are hypotheses in the assignment, not established measurements in the handoff. See sections 5–7; 10% denominator inflation gives 9.09% yaw reduction for fixed encoder travel. |

## 2. INDEPENDENT EVIDENCE

**FACT — worktree safety.** Initial configured cwd was `/home/gautham/ros2_ws/src/coco-robot-ros2/.codex/worktrees/c2nav0-implementation`, clean, detached at `1d17a13`. Main was `ea66155330af26b322758d094eb881ebedac843a`. The requested jazzy-named path differs; this review uses the configured worktree. Root AGENTS.md and worktree CLAUDE.md were read. Neither the main directory nor Claude's locked worktree was edited.

**FACT — delivery ancestry.** Remote diagnosis was `8a7b090`, a sibling of the initial Codex review commit. To append without merge, rebase or force-push, this clean worktree was switched to that fetched commit and branch `codex/c2nav34-odom-review` created. The earlier independent review is retained on `codex/c2nav33-review-preserved` at `1d17a13`. Its relevant disagreements are carried into this report. When the remote advanced to `87131a0`, the review/checkpoint were preserved, a fresh branch `codex/c2nav34-odom-final` was created from that commit, and only those documentation additions carried forward. No history was rewritten; Claude's new handoff, analysis and bundle are preserved unchanged.

**FACT — installed packages and SHA-256.**

| Artifact | Version / digest |
| --- | --- |
| ros-jazzy-nav2-amcl | `1.3.11-1noble.20260412.054619` |
| ros-jazzy-diff-drive-controller | `4.39.0-1noble.20260412.063951` |
| `/opt/ros/jazzy/lib/libamcl_core.so` | `5b17902144fd474985ddebf771e2d1dff58c464112c80cc707c3ba7d38e6139f` |
| `/opt/ros/jazzy/lib/libdiff_drive_controller.so` | `5d52a164589ca1d8cfc731be7bbecac68ff1fbe8404407afec2192b9afac05e9` |

Version-tagged primary sources, corroborated against installed code rather than assumed identical Debian build inputs:

- [AMCL node 1.3.11](https://github.com/ros-navigation/navigation2/blob/1.3.11/nav2_amcl/src/amcl_node.cpp), SHA `11c801a2bf290d299046ace3bdc7f6be4317e07da8d43010d6c50f7102fa6d4b`.
- [Differential motion model](https://github.com/ros-navigation/navigation2/blob/1.3.11/nav2_amcl/src/motion_model/differential_motion_model.cpp), SHA `3d543e7115a0c52714af2ec7715ce82a0274499b4c8fb062609e23de737c60b0`.
- [Controller 4.39.0](https://github.com/ros-controls/ros2_controllers/blob/4.39.0/diff_drive_controller/src/diff_drive_controller.cpp), SHA `c67e1e697766e958e3df9e4c0a79e88c3bb8d2f868ce10c65de803237c4c2e6c`.
- [Odometry integration](https://github.com/ros-controls/ros2_controllers/blob/4.39.0/diff_drive_controller/src/odometry.cpp), SHA `42abab164713f1c8559fe8b4e6bd9e78f93ab0ff97a49b2ff772e81482c9dc69`.

**FACT — historical limits.** C2-NAV.28 records receipt-bucket AMCL poses and a place-linked error; .29 reconstructs scans because only minimum ranges survived; .30 records published clouds; .31 scores reconstructed scans on already-published particles; .32 substitutes GT for odometry; .33 changes resampling interval and sees only alternate pre-resampling populations. These support descriptive findings, not an exhaustive causal elimination. This review re-ran their available offline checks; no new wall-bias observation was collected.

## 3. EXACT AMCL ODOM INPUT

**FACT — direction:** `lookupTransform(target="odom", source="base_footprint", time=scan.header.stamp)`, equivalently **T_odom_base**, the pose of the base in odom coordinates. This is the edge commonly written `odom -> base_footprint`; it maps base coordinates into odom, not the inverse. It is not a nav_msgs/Odometry subscription.

**FACT — installed evidence:** `laserReceived` constructs the scan timestamp and passes the base-frame member at `+0x9d8` to `getOdomPose` at call `0xe3be8`. `getOdomPose` starts at `0xd6a40`; it constructs an identity PoseStamped, assigns the supplied stamp at `0xd6b4a–0xd6b59`, supplies odom-frame member `+0xa88` at `0xd6bd2`, and invokes the buffer lookup at `0xd6c67`. The installed `tf2_ros/buffer_interface.hpp:211–217` expands `transform` to lookup with target, input frame, input stamp and default zero timeout. Source `amcl_node.cpp:417–445, 661–694` corroborates the call chain and x/y/yaw extraction. A failed lookup returns before prediction.

**FACT — timing:** SensorDataQoS `/scan` enters a TF MessageFilter targeting `odom`, queue size 10. `initMessageFilters:1524–1543` passes `transform_tolerance_` as the filter's **buffer timeout** (installed `message_filter.hpp` constructor), not a command to sample odometry 0.5 s in the future. Baseline tolerance is 0.5 s. Separately, outgoing `map -> odom` is post-dated by that tolerance. Input uses the scan header stamp, not receipt time, latest TF, scan end time, or each beam's acquisition time. Lidar is configured 10 Hz.

**FACT — motion:** `shouldUpdateFilter` at binary `0xdda20` subtracts previous accepted odometry x/y and computes wrapped yaw difference. Source lines 781–793 specify strict OR tests: `abs(dx)>0.25`, `abs(dy)>0.25`, `abs(dyaw)>0.2`, or force-update. Thus (0.20,0.20) m does **not** pass the translation gate despite 0.283 m norm. The reference is `pf_odom_pose_`, not the preceding scan or preceding published pose. Initialization sets that anchor; `updateFilter` advances it after sensor processing. Scanner flags and forced updates matter; a scan callback is not synonymous with a motion update.

**DERIVED — differential prediction:** `trans=hypot(dx,dy)`; `rot1=angle_diff(atan2(dy,dx),old_yaw)`, except translation below 0.01 m forces rot1=0; `rot2=angle_diff(dyaw,rot1)`. Sampled translation and rotations move each particle in its own heading. Alphas are all 0.2; alpha5 is unused here. Input errors affect both mean propagation and motion-dependent noise. Controller covariance and twist fields are not AMCL motion inputs.

**UNKNOWN — actual past consumption:** static configuration cannot prove historical TF authority, message loss, loaded overlays or dynamic overrides. `/particle_cloud` is stamped with AMCL `now()`, and `nav_bench` uses callback receipt times; nearest-cloud alignment cannot restore exact scan identity.

## 4. EXACT ODOMETRY PRODUCER

**FACT — configured chain:** `full_world_robo.launch.py` loads package-share xacro, starts robot_state_publisher, bridges configured sensors, and spawns `diff_drive_controller`. Xacro lines 516–555 expose position/velocity feedback and velocity commands through `gz_ros2_control/GazeboSimSystem`, whose plugin loads `coco_controllers.yaml`. There is **no active Gazebo DiffDrive system plugin in this model**. The separate Gazebo OdometryPublisher produces `/model/coco/odometry` in world coordinates; bridge.yaml bridges that Odometry message, not its TF.

**FACT — controller:** left joints are -3/-4, right -1/-2. `open_loop=false`; installed generated default `position_feedback=true`. It averages the two encoder positions on each side and integrates them. Controller manager update rate is configured 100 Hz; publication is requested at 50 Hz, with independent realtime try-publish operations for `/diff_drive_controller/odom` and `/tf`. These rates and matching publication are not guaranteed delivery measurements. TF parent/child are `odom` / `base_footprint`; empty namespace/prefix in this launch leaves those names unchanged. Robot_state_publisher supplies the downstream fixed base/laser chain.

**FACT — binary:** configure multiplies dimensions before `setWheelParams` at `0x70b65–0x70b7b`; encoder-position integration is called at `0x6eaeb`. `Odometry::update` at `0xa2580` multiplies positions by radii and differences previous positions. At `0xa23cd–0xa23e4`, right-minus-left is divided by separation and passed to `integrateExact`. The integration operates on displacement, despite intermediate names containing “velocity”. Rolling averages affect published twist, not integrated pose. The library probe below confirms the actual arithmetic.

**FACT — local deployment caveat:** installed controller YAML and xacro symlink into the main checkout and currently match this worktree byte-for-byte. Installed nav2_params differs outside AMCL, but its AMCL block is identical. No installed file was changed. A future capture must retain resolved package paths, library hashes and parameter readback; repository edits alone do not establish loaded configuration.

## 5. WHEEL GEOMETRY

**FACT / DERIVED — xacro geometry independently transformed, including collision offsets:**

| Quantity | Value and meaning |
| --- | --- |
| Joint origins in base_link | front/rear x=±0.090, right/left y=∓0.137, z=0.045 m |
| Joint-origin track | 0.274 m; matches coco_config.WHEEL_SEPARATION and controller nominal separation |
| Cylinder origin in each wheel link | `(0,0,-0.0155)` m; left wheel frames have roll pi |
| Collision centers in base_link | front/rear x=±0.090, right/left y=∓0.1215, z=0.045 m |
| Collision-center track | **0.243 m**, not 0.274 m |
| Cylinder radius / width | 0.0585 / 0.040 m |
| Controller radius and side multipliers | 0.0585 m, 1.0 left, 1.0 right |
| Controller separation multiplier | 1.10 |
| Effective controller separation | **0.3014 m** |
| Controller yaw lever arm | **0.1507 m** per side |
| Longitudinal wheelbase | 0.180 m |

Derivation uses `chassis_joint` rotation Rx(pi/2) and translation `(0.12,-0.08,0)`, each wheel joint's actual origin/rotation, then its collision origin. The root comment describes joint origins, not collision centers. Positive wheel rotation is normalized by opposite joint axes. Wheel friction is isotropic mu1=mu2=0.7 in the xacro.

**UNKNOWN:** physical instantaneous centers of rotation and effective skid-steer track. Four finite-width contact patches scrub during turns; neither geometric track is a measurement of that effective track. The 0.1507 m number is a controller lever arm, not a universal chassis turning radius. For encoder travel increments l/r, the controller's path radius is `R=0.3014*(r+l)/(2*(r-l))`, zero for equal/opposite travel, unbounded for equal travel. Contact-center discrepancy is a finding to retain, not a justification to retune.

## 6. QUANTITATIVE ODOMETRY BIAS

**DERIVED — integration:** `ds=(r+l)/2`, `dtheta=(r-l)/0.3014`. For non-small dtheta, `x += R*(sin(theta+dtheta)-sin(theta))`, `y -= R*(cos(theta+dtheta)-cos(theta))`; below 1e-6 rad it uses midpoint/RK2. Encoder angles are scaled by 0.0585 m. Very small dt below 0.0001 s is rejected. The probe verifies a unit-distance/unit-yaw arc gives `(0.841470984808, 0.459697694132, 1.0)`.

**FACT — fixed wheel travel probe:** encoder travel `l=-0.137`, `r=+0.137` m yields yaw **+0.909090909091 rad**, x=y=0; reversed travel yields −0.909090909091 rad. Those inputs would yield ±1 rad with separation 0.274. Hence 1.10 gives **9.0909% less yaw magnitude**, not 10% more yaw. The error is negative for positive turns and positive for negative turns.

**DERIVED — physical comparison is conditional:** let `B_actual=(r-l)/dtheta_GT` on a turn. Then `dtheta_odom/dtheta_GT=B_actual/0.3014`. If B_actual=0.3014, yaw is correct. If greater, odometry over-reads; if smaller, it under-reads. Equivalently, for slip factor k relative to joint track, `dtheta_GT=k*(r-l)/0.274`, giving ratio `1/(1.10*k)` and equality at k=0.9090909. This sign cannot be selected from the YAML.

The hypothetical no-slip collision-center model instead gives `0.243/0.3014=0.80623756`, a 19.3762% under-read. It is **not** a measured prediction for four-wheel skid steer. Moreover inverse kinematics commands `(v ± omega*0.3014/2)/0.0585`: changing the multiplier also changes wheel differential commands. Fixed-command and fixed-encoder comparisons are different experiments.

**FACT / DERIVED — independent counterexample to Claude's new command bound:** suppose a 1 m/s command is tracked at wheel/ground speed 0.9712 m/s with no longitudinal slip. After one second, encoder travel is l=r=0.9712 m. The installed library reports **x=0.9712 m and linear velocity=0.9712 m/s**, confirmed by the extended probe. GT is also 0.9712 m: odometry error is zero despite the exact command/GT slope Claude uses to claim at least 2.96% over-report. This is an algebraic/library counterexample, not a new robot experiment. Tracking shortfall is already reflected in encoder feedback; it cannot be added as odometry error. Longitudinal slip remains possible but unmeasured.

**DERIVED — 0.09 m scale example, not fitted historical evidence:** a 1 rad positive turn with nominal-track/no-slip assumptions leaves heading residual −0.0909091 rad. A subsequent 1 m straight segment whose true heading is east produces y error `sin(-1/11)=-0.0907839 m` before localization correction. A pure centered turn has zero deterministic translation error. To create 0.09 m perpendicular displacement over straight distance L requires `abs(epsilon)=asin(0.09/L)`:

| L | Heading residual |
| --- | --- |
| 0.5 m | 0.180986 rad / 10.3698 degrees |
| 1.0 m | 0.090122 rad / 5.1636 degrees |
| 2.0 m | 0.045015 rad / 2.5792 degrees |

These are reproducible scale calculations. None supplies the missing observed turn error, distance, orientation or AMCL correction sequence.

## 7. WHETHER IT CAN EXPLAIN THE AMCL BIAS

**HYPOTHESIS:** yes, the order of magnitude is physically possible. **UNKNOWN:** whether it happened, with the required southward sign and timing, in the recorded route. It has not been proved or rejected quantitatively.

Place dependence neither proves nor excludes odometry error. A common heading error yields map-frame increments proportional to `(-sin(theta),cos(theta))*ds*epsilon`; route direction, prior turns, slip and AMCL observation corrections vary by location. C2-NAV.28's different error direction at another location does not eliminate that mechanism. Conversely a fixed wheel multiplier has no built-in preference for the south wall.

Compare synchronized relative SE(2) motion over the **whole approach and transition**, not only y since wall-leg start. Establish the same physical base origin and use one initial rigid alignment `T_world_odom=T_world_base(t0)*inverse(T_odom_base(t0))`; do not refit the alignment every row or leg. Constant world/map translation cancels in increments; yaw misalignment does not. Retain both historical and landmark-derived map conventions without changing old constants.

For actual update intervals compute body-relative odometry and GT increments, heading errors, and propagation through the installed motion model on the observed population. Quantify prediction versus subsequent sensor/resampling corrections. Similar cumulative drift is evidence of sufficiency in scale, not unique causation; small endpoint drift is not a general falsifier. A causal conclusion requires event-complete paired data and a controlled offline counterfactual with stated uncertainty, not the handoff's one-third rule.

## 8. INSTRUMENTATION PLAN

**DERIVED — minimum useful external measurement:** an opt-in sidecar separate from the historical 58-column trace, with a `tf2_ros.Buffer` and `TransformListener` on resolved `/tf` and `/tf_static`, a SensorDataQoS `/scan` subscription, and full header-stamped GT samples. For every scan, request **target odom, source base_footprint, scan stamp**. Do not use `Time(0)` (latest), receipt time or fixed 10 Hz timer samples as the scientific record. Keep all relevant raw TF samples at producer cadence (nominal 50 Hz), static transforms and scan headers; raw scan ranges are also needed for full filter counterfactual replay. `/diff_drive_controller/odom` is optional pose/twist corroboration, not mandatory for AMCL input capture.

Use nonblocking pending lookups retried when TF arrives; never block the single executor waiting for a transform it must itself receive. Preserve integer seconds/nanoseconds, original frame IDs, query stamp, returned stamp, pose/quaternion, x/y/yaw, simulation receipt time and monotonic receipt time. Record session/clock epoch, sequence, timeout/extrapolation/connectivity/lookup errors, queue overflow and final unresolved requests. Keep missing values null with explicit status; never hold, zero-fill, or substitute GT/latest TF. A finite pending deadline and bounded queue must be declared and losses counted. One nonblank sample per leg is insufficient; the analysis interval must have complete paired coverage or explicitly remain inconclusive.

**FACT / DERIVED — interpolation:** tf2 linearly interpolates translation and interpolates rotation between bracketing transforms. The installed BufferCore probe returns x=1 and yaw=0.1 halfway between x=0/yaw=0 and x=2/yaw=0.2, and rejects a future query. This interpolated value is precisely the input semantics to reproduce, even if it differs from continuous chassis motion. Preserve bracketing samples/gaps; avoid Euler interpolation across yaw wrap. For an ideal constant arc at v=1 m/s, omega=2 rad/s and a 0.02 s bracket, chord sagitta is about 0.000100 m. This illustrative value is not a bound under missing TF, acceleration or contact slip. An external buffer may have different delivery/order/history than AMCL's buffer.

**DERIVED — minimum exact-consumption measurement:** the external listener cannot prove which value AMCL actually used or which scans reached prediction. Add a default-disabled diagnostic hook in a version-matched isolated nav2_amcl source overlay: snapshot successful `getOdomPose` result and, immediately before the existing `odometryUpdate` call, its actual `pose`, `delta`, prior anchor, scan stamp/frame, epoch and update sequence. Record initialization/anchor resets, lookup failures and sensor-update completion/early-return status. Associate output cloud/pose with that same update ID rather than guessing from publication times. This is the smallest reliable observation boundary for **actual consumed** input; it does not require all particle weights just to measure odometry.

The hook should copy into a bounded preallocated queue, count drops, and drain outside the filter callback. Do not change parameters, RNG calls, update order or existing topics. Enabled capture still has scheduling overhead: validate capture-off/on equivalence offline. A separate TF listener can corroborate it but is not a substitute for this event record. GT synchronization still needs full acquisition stamps and verified model-to-base extrinsics; the advertised child frame alone does not prove Gazebo model-origin equivalence.

## 9. IMPLEMENTATION

**FACT: none.** Only this review and the required SESSION_LOG checkpoint change. The repo does not vendor nav2_amcl. A latest-TF extension could be small but would fail exact-consumption observability; a reliable hook needs an isolated source overlay and lifecycle/output integration. That is not a clean, justified change to `nav_bench.py` in this delivery. No trace columns, timestamp rules, frame constants, controller values or launch files changed. No behavioral permission is needed or requested to complete this review.

## 10. TESTS

All tests below were offline; no rclcpp/rclpy initialization, node creation, DDS participant, ROS CLI or simulator launch was used.

| Check performed this session | Result |
| --- | --- |
| `python3 -P docs/data/c2nav22_yaw.py selftest` | PASS |
| `python3 -P docs/data/c2nav24_chain.py selftest` | PASS, all gates |
| `python3 -P docs/data/c2nav28_amcl.py selftest` | FAIL: five existing schema/count/scope/byte-identity assertions, before review edits |
| `python3 -P docs/data/c2nav29_scanmap.py --selftest` | PASS 21/21 |
| `python3 -P docs/data/c2nav30_cloud.py selftest` | FAIL: one existing historical changed-file allow-list assertion; schema and bucket tests pass |
| `python3 -P docs/data/c2nav31_lf.py selftest` | PASS 48/48 |
| `python3 -P docs/data/c2nav32_mm.py selftest` | PASS 43/43, including installed motion-library oracle |
| `python3 -P docs/data/c2nav33_weights.py selftest` | PASS 62/62 |
| `python3 -P docs/data/c2nav34_odom.py selftest` | PASS 18/18; does not validate the challenged physical/causal assumptions |
| `python3 -P docs/data/c2nav34_odom.py incr` | Descriptive residual statistics reproduced; actual-input interpretation rejected |
| `python3 -P docs/data/c2nav34_odom.py cmd` | Saved slope 0.9712, n=253 reproduced; claimed odometry lower bound disproved |
| Independent controller/BufferCore C++ probe below | Compiles with `-Wall -Wextra -Wpedantic -Werror`; all assertions PASS |
| XML rotation/translation arithmetic on all four wheel joints/collisions | Coordinates in section 5 reproduced |
| AST schema check | 28 frozen columns + 3 AMCL + 27 cloud = 58, exact order preserved |
| Byte comparison to initial `1d17a13` | Recorder and all 27 tracked C2-NAV.22–32 files identical |
| Python `compile()` of recorder without executing it | PASS |
| `python3 -m flake8 --isolated --select=E9,F63,F7,F82 gazebo_models/scripts/nav_bench.py` | PASS; focused error lint, not full style lint |
| `git diff --check` | PASS |

No instrumentation was implemented, so no colcon/package build or full package test run is claimed. Such tests can initialize ROS and are outside this session. Historical artifacts remain byte-identical/reproducible to their previous extent; the two historical selftest failures are not claimed fixed.

Investigation failures: the first probe compile omitted `rcpputils/version.h`, selecting deprecated header compatibility code; including the installed version header fixed it. First probe execution failed during shared-library load because the inherited Cyclone RMW dependency could not find `libddsc.so.0`; adding `/opt/ros/jazzy/lib/x86_64-linux-gnu` to its process-local library path fixed it without ROS initialization. Initial ad-hoc AST assertions assumed a literal 58-element list, then selected shorter legacy bundle schemas; the actual header expands `PC_FIELDS`, and the frozen full schema is the longest `traces[*].schema` as the existing selftests specify. Corrected assertions pass. Temporary searches used nonexistent header/helper paths before package enumeration corrected them. No repository change resulted from these failures.

### Reproduce the independent library probe

Save the following as `/tmp/c2nav34-probe.cpp`. It calls library math and an in-memory TF buffer only; no init/spin/node calls.

```cpp
#include <cassert>
#include <cmath>
#include <cstdio>
#include "rcpputils/version.h"
#include "diff_drive_controller/odometry.hpp"
#include "tf2/buffer_core.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

int main() {
  constexpr double r = .0585, b = .274 * 1.10;
  for (double sign : {-1., 1.}) {
    diff_drive_controller::Odometry o;
    o.setWheelParams(b, r, r);
    o.init(rclcpp::Time(0, 0, RCL_ROS_TIME));
    assert(o.update(-sign * .137 / r, sign * .137 / r,
                    rclcpp::Time(1, 0, RCL_ROS_TIME)));
    assert(std::abs(o.getHeading() - sign / 1.10) < 1e-12);
    assert(std::abs(o.getX()) < 1e-12 && std::abs(o.getY()) < 1e-12);
    std::printf("pivot sign=%+.0f yaw=%.12f x=%.12f y=%.12f\n",
                sign, o.getHeading(), o.getX(), o.getY());
  }
  diff_drive_controller::Odometry o;
  o.setWheelParams(b, r, r);
  o.init(rclcpp::Time(0, 0, RCL_ROS_TIME));
  assert(o.update((1-b*.5)/r, (1+b*.5)/r,
                  rclcpp::Time(1, 0, RCL_ROS_TIME)));
  assert(std::abs(o.getX()-std::sin(1.)) < 1e-12);
  assert(std::abs(o.getY()-(1-std::cos(1.))) < 1e-12);
  std::printf("arc x=%.12f y=%.12f yaw=%.12f\n",
              o.getX(), o.getY(), o.getHeading());
  diff_drive_controller::Odometry tracking;
  tracking.setWheelParams(b, r, r);
  tracking.init(rclcpp::Time(0, 0, RCL_ROS_TIME));
  assert(tracking.update(.9712/r, .9712/r,
                         rclcpp::Time(1, 0, RCL_ROS_TIME)));
  assert(std::abs(tracking.getX()-.9712)<1e-12);
  assert(std::abs(tracking.getLinear()-.9712)<1e-12);
  puts("tracking counterexample: encoder travel .9712 => odom .9712, not command 1");
  tf2::BufferCore buffer;
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id="odom"; t.child_frame_id="base_footprint";
  t.header.stamp.sec=1; t.transform.rotation.w=1;
  assert(buffer.setTransform(t,"probe",false));
  t.header.stamp.sec=3; t.transform.translation.x=2;
  tf2::Quaternion q; q.setRPY(0,0,.2);
  t.transform.rotation.z=q.z(); t.transform.rotation.w=q.w();
  assert(buffer.setTransform(t,"probe",false));
  const auto v=buffer.lookupTransform("odom","base_footprint",
                                     tf2::timeFromSec(2));
  assert(std::abs(v.transform.translation.x-1)<1e-12);
  assert(std::abs(2*std::atan2(v.transform.rotation.z,
                             v.transform.rotation.w)-.1)<1e-12);
  bool missing=false;
  try {buffer.lookupTransform("odom","base_footprint",tf2::timeFromSec(4));}
  catch (const tf2::TransformException &) {missing=true;}
  assert(missing);
  puts("TF midpoint x=1 yaw=.1; future extrapolation rejected; all assertions PASS");
}
```

Build/run without sourcing or launching ROS:

```python
import os
import subprocess
from pathlib import Path
prefix = '/opt/ros/jazzy'
args = ['g++', '-std=c++17', '-Wall', '-Wextra', '-Wpedantic', '-Werror',
        '/tmp/c2nav34-probe.cpp', '-o', '/tmp/c2nav34-probe']
args += ['-I' + str(p) for p in Path(prefix + '/include').iterdir() if p.is_dir()]
args += ['-L' + prefix + '/lib', '-ldiff_drive_controller', '-lrclcpp',
         '-ltf2', '-lrcutils', '-Wl,-rpath,' + prefix + '/lib']
env = os.environ.copy()
env['LD_LIBRARY_PATH'] = prefix + '/lib:' + prefix + '/lib/x86_64-linux-gnu'
subprocess.run(args, env=env, check=True)
subprocess.run(['/tmp/c2nav34-probe'], env=env, check=True)
```

Binary inspection can be reproduced with `objdump -d --no-show-raw-insn -C` and the address ranges in sections 3–4; `nm -D -C` identifies the Odometry methods. Downloads/disassembly/test logs from this session are in `/tmp/c2nav34-review` (scratch, not required historical artifacts).

## 11. DISAGREEMENTS

**FACT / DERIVED:** the controller multiplier is a calibrated effective dimension, not evidence of a measured 10% chassis yaw error. The collision centers also differ from the nominal joint track. Motion direction and slip determine the sign relative to GT. A clock-time TF sample cannot be labeled the scan-time transform consumed by AMCL, and a scan-time external lookup cannot establish accepted-update identity.

**FACT / DERIVED:** Claude's latest unconditional `v_odom == v_cmd` and `omega_odom == omega_cmd` identities fail without perfect wheel tracking. The 2.96% lower bound is disproved by an encoder-feedback counterexample. A CI crossing zero on corrected AMCL yaw does not eliminate odometry yaw error.

**UNKNOWN:** the cause remains unresolved. Neither previous proxy-based eliminations nor a matching 0.09 m scale calculation closes it. The proposed latest-TF columns and one-third y-error falsifier are inadequate for the causal claim. Exact-input instrumentation should precede any controller tuning or live diagnostic run.

## 12. EXACTLY ONE NEXT ACTION

**Implement and validate offline the default-disabled, scan-stamped AMCL odometry-consumption hook and its separate synchronized GT recorder specified in section 8, with capture-off/on equivalence and complete missing-data accounting, before requesting any live experiment.**
