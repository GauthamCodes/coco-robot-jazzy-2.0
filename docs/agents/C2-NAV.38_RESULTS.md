# C2-NAV.38 — AMCL sensor-correction / posterior-bias investigation: RESULTS

Offline/source/installed-binary investigation only. No Gazebo, no ROS
navigation, no `nav_bench`, no live experiment, no parameter/map/
controller change. Every number below is either (a) read directly from
installed source/headers in this worktree, (b) decoded from disassembly
and raw `.rodata` bytes of the installed `libpf_lib.so`/`libamcl_core.so`,
or (c) computed offline from CSV/`.npz` files C2-NAV.37's already-completed
live runs wrote to `~/coco_nav_runs/c2nav37_run{0..4}/`. No new simulator
run produced any number in this document.

## 1. Executive conclusion

**Every mechanism in the motion→weight→resample→pose-extract→publish
pipeline that has actually been tested — by this session or by
C2-NAV.29–.37 — explains at most a minority of the observed ~0.09–0.13 m
bias, and several explain essentially none.** Odometry input (C2-NAV.37):
real but ~8x too small. Motion-model noise (C2-NAV.32): ~0, CI straddles
zero. Importance weighting (C2-NAV.33, the deployed likelihood field
scored against 9 actually-captured pre-resample clouds): ~3% of the
bias, CI straddles zero. Cluster-selection / pose-extraction (C2-NAV.30,
independently reconfirmed this session from four fresh runs): negligible,
≤0.0003 m against a 0.09–0.13 m target. Map/scan geometry (C2-NAV.29/.31):
real, place-dependent, but only 29–38% of the bias at `wall_adjacent`
specifically.

**One new, decisive structural finding from this session**: the bias is
**strongly and repeatably place-dependent, not time/accumulation-dependent**.
Pooling this session's own AMCL-vs-GT residual across all 7 `TOUR` legs,
all 5 C2-NAV.37 runs, all 3 reps (105 leg-instances, the full chained
21-leg trajectory each run): the residual's sign and magnitude are set by
*which leg* (`open_space` −0.112 m, `obstacle_corner` +0.037 m, `corridor_gate`
−0.015 m — the sign **flips** by location), essentially independent of how
far into the 21-leg chain that leg occurs (correlation between
chain-position and residual = **+0.089**, indistinguishable from zero). A
second test — whether resampling's own duplicate-particle mechanism
concentrates surviving support southward, distinct from whether the
weights themselves are biased — also comes back **against** that
hypothesis (duplicated particles sit slightly *north*, not south, of
their own snapshot's centroid; 95% CI [−0.0330, −0.0212], excludes zero
but in the wrong direction).

**Leading hypothesis, evidence-ranked, not proven**: the majority of the
remaining bias is most likely a **place-dependent effect tied to map/
geometry/discretization** that C2-NAV.31's specific likelihood-field
reconstruction did not fully capture — not a weighting bias, not a
resampling-duplication bias, not motion noise, not pose-extraction, and
not temporal accumulation. This narrows the search space; it does not
identify the exact remaining mechanism.

## 2. Exact AMCL sensor-correction data path

Traced from `coco_nav_diag/src/amcl_node.cpp` (a byte-faithful
class-renamed fork of `nav2_amcl::AmclNode` 1.3.11 — confirmed identical
control flow to the installed `libamcl_core.so` via the disassembly
cross-checks in item **9** below) plus the installed headers under
`/opt/ros/jazzy/include/nav2_amcl/` and disassembly of
`/opt/ros/jazzy/lib/libpf_lib.so`.

`AmclNode::laserReceived()` (`amcl_node.cpp:746`), in call order, one
scan-received callback:

1. **`getOdomPose()`** (`:784`) — the scan-stamped `odom→base_footprint` TF
   lookup. This is the exact quantity C2-NAV.37 instrumented and measured.
2. **Motion update**, gated by `shouldUpdateFilter()` (`:982`, thresholded
   on `d_thresh_`/`a_thresh_` = `update_min_d`/`update_min_a`):
   `motion_model_->odometryUpdate(pf_, pose, delta)` (`:893`) — perturbs
   `pf_->sets[current_set]`'s particles per the odometry delta plus
   `alpha1..5` noise (`nav2_amcl::DifferentialMotionModel`, in
   `libmotions_lib.so`; not re-disassembled here — C2-NAV.32 already did,
   see item 9).
3. **`updateFilter()`** (`:996`) if `lasers_update_[laser_index]` is set
   (`:901`) — builds `LaserData` from the scan, then
   **`lasers_[laser_index]->sensorUpdate(pf_, &ldata)`** (`:1071`). For
   this repo's config (`laser_model_type: "likelihood_field"`,
   `do_beamskip: false`) this dispatches to
   `nav2_amcl::LikelihoodFieldModel::sensorUpdate` (declared
   `sensors/laser/laser.hpp:158`; implementation compiled into
   `libsensors_lib.so`, not re-disassembled this session — C2-NAV.29/.31
   already rebuilt this exact model formula from source configs, see
   item 9). That method calls `pf_update_sensor(pf, sensorFunction, data)`
   (`pf.hpp:158`), which — per C2-NAV.33's own disassembly of
   `libpf_lib.so+0x1aa0`, reproduced and trusted here rather than
   redone — writes **normalized** weights into
   `sets[current_set].samples[i].weight` and **does not flip
   `current_set`**. `pf_odom_pose_ = pose` (`:1073`) advances the anchor
   immediately after.
4. **Resample decision** (`:905`): `if (!(++resample_count_ %
   resample_interval_))`. With this repo's `nav2_params.yaml`
   (`resample_interval: 1`, unmodified), this is **true on every single
   cycle** — `pf_update_resample(pf_, map_)` (`:906`) runs every scan
   update, not periodically. `pf_update_resample` (per C2-NAV.33's
   disassembly, item 9) **does** flip `current_set` and writes back
   **flat, equal weights** (`1/N`) into the new current set — the
   mechanism behind C2-NAV.30's independently-measured
   `ESS/n = 1.0000000` exactly, every sample, both legs.
5. **`publishParticleCloud(set)`** (`:914`) — `set = pf_->sets +
   pf_->current_set` (`:910`), i.e. **the just-resampled set**, gated only
   on `!force_update_`. This is exactly the `/particle_cloud` topic
   `nav_bench.py` records into the `.npz`/`pc_*` trace columns this
   session re-analyzed.
6. **Pose extraction and publication**, gated on `resampled ||
   force_publication || !first_pose_sent_` (`:917`) — under
   `resample_interval: 1` this condition is true on essentially every
   cycle once initialized:
   - **`getMaxWeightHyp()`** (`:1103`) loops `hyp_count` over
     `pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean,
     &pose_cov)` (`:1116`, `pf.hpp:168`) — **one call per cluster in the
     current set's KD-tree clustering**, picks the cluster with the
     **largest total weight** (`:1125-1127`), and returns that cluster's
     **mean** as `max_weight_hyps.pf_pose_mean` (`:1138`).
   - **`publishAmclPose()`** (`:1144`) publishes
     `hyps[max_weight_hyp].pf_pose_mean` verbatim as `/amcl_pose`
     (`:1165-1167`).
   - **`calculateMaptoOdomTransform()`** (`:1202`) uses the **same**
     `hyps[max_weight_hyp].pf_pose_mean` to compute the published
     map→odom TF (`:1211-1214`) — the actual localization signal every
     downstream consumer (costmaps, the controller, `nav_bench`'s own
     `amcl_x/amcl_y` trace columns) reads.

**The published pose is the mean of whichever KD-tree cluster currently
holds the greatest total particle weight — not the filter's global
weighted mean over all particles — extracted from the *post-resampling*
set on essentially every cycle under this repo's actual configuration.**
This is read directly from source, not inferred from topic names or
recalled from general AMCL knowledge.

**Clustering grid**, verified from `libpf_lib.so` disassembly + raw
`.rodata` bytes (not from memory — decoded via `objdump -s` and IEEE-754
unpacking): `pf_kdtree_alloc` (`libpf_lib.so+0x1ca0`) hardcodes the KD-tree
cell size to `size[0]=size[1]=0.5` m, `size[2]=0.17453292519943295` rad
(exactly `10° = 10·π/180`), loaded from `.rodata` offsets `0x5220` and
`0x5230`. `pf_kdtree_insert` (`+0x1dc0`) computes each particle's grid key
as `round(pose.v[i] / size[i])` per axis (confirmed from the `divsd`/
`cvttsd2si` sequence at `1dd4-1eea`). Particles landing in the same or
adjacent grid cells are merged into one cluster by `pf_kdtree_cluster`
(declared `pf_kdtree.hpp:91`; not disassembled this session — see item 9
for what was and wasn't checked).

## 3. Mechanistic breakdown

| Stage | What happens | Where measured this session / prior sessions |
|---|---|---|
| Motion input | Scan-stamped `odom→base_footprint`, integrated by `DifferentialMotionModel::odometryUpdate` with `alpha1..5` noise | C2-NAV.37 (real input, live); C2-NAV.32 (noise variance, offline oracle replay) |
| Sensor weighting | `LikelihoodFieldModel::sensorUpdate` → `pf_update_sensor`, writes normalized per-particle weights, **`current_set` unchanged** | C2-NAV.29/.31 (deployed model rebuilt from config, scored offline); C2-NAV.33 (actual deployed pre-resample weights, captured live via `resample_interval:2`) |
| Normalization | Weights normalized inside `pf_update_sensor` (verified structurally from disassembly, C2-NAV.33) | C2-NAV.33 |
| Resampling | `pf_update_resample`, runs **every cycle** (`resample_interval:1`), flips `current_set`, **flattens weights to `1/N`** | C2-NAV.30 (ESS/n≡1.0 signature); C2-NAV.33 (disassembly + oracle: `current_set` flips 0→1, weight → bit-identical `1/N`) |
| Particle-set statistics | `pf_cluster_stats` / `pf_kdtree_cluster` on the post-resample set; **not disassembled by any C2-NAV session to date** (see item 9) | C2-NAV.30 (own re-implementation of clustering, not AMCL's); this session (comparison of `/amcl_pose` vs the recorded cloud's own weighted/unweighted mean, all four C2-NAV.37 runs) |
| Pose extraction | `getMaxWeightHyp`: mean of the **highest-total-weight cluster**, not the global mean | Source-traced this session (§2); independently cross-checked against recorded data this session (§4, row "pose extraction / cluster selection") |
| Publication | `publishAmclPose` + `calculateMaptoOdomTransform`, both from the same `hyps[max_weight_hyp].pf_pose_mean` | Source-traced this session |

## 4. Evidence table

| Mechanism | Supporting evidence | Contradicting evidence | Status |
|---|---|---|---|
| Odometry input bias (motion prediction) | C2-NAV.37: real, sign-consistent, +0.000747 m/update along-track pooled, CI excludes zero (SEM-based) in all 4 runs | ~8x smaller than C2-NAV.34's own output-side proxy (+0.00616 m/update); C2-NAV.32's GT-proxy replay: 95% CI on wall_adjacent noise term [−0.00447, +0.00683] straddles zero, mean weakly *north* | **INSUFFICIENT ALONE** (FACT: bias exists; HYPOTHESIS: contributes meaningfully; not confirmed as primary) |
| Sensor/likelihood weighting | C2-NAV.31: deployed model doesn't prefer AMCL's pose over GT (median w(AMCL)/w(GT)=0.7774, AMCL never wins, 0/19); C2-NAV.33: `shift` (weighted−unweighted centroid) = −0.00266 m, only 3.0% of the −0.0884 m unweighted displacement | C2-NAV.33's own `shift` 95% CI [−0.00638, +0.00139] straddles zero; `mass_shift` (+0.016 to +0.017) is real and significant but "worth 2.7mm of centroid" | **WEAKENED/ELIMINATED** as primary driver of the centroid shift |
| Resampling (duplication/support concentration) | This session: tested directly — duplicated vs. unique-particle south-fraction in 369 post-resample `wall_adjacent` snapshots | Difference is **−0.0271** (CI [−0.0330,−0.0212], excludes zero) — duplicates skew *north*, opposite the needed direction | **TESTED, WRONG DIRECTION** — does not explain a southward bias |
| Pose extraction / cluster selection | C2-NAV.30: whole-set mean vs `/amcl_pose`, median diff 0.00004 m (n=19, one run); **this session, independently, 4 fresh runs**: published pose vs recorded weighted-centroid mean diff 0.0001–0.0003 m (n=369, magnitude 0.0003 m) | None found; two independent measurements (different sessions, different runs) converge | **RULED OUT** as a meaningful separate contributor |
| Map/scan geometry | C2-NAV.29: raycast-reconstructed scan scores GT higher than AMCL in 98.7% (537/544) samples, 100% (90/90) at `wall_adjacent`; C2-NAV.31: budget table, map+half-cell convention = 29% (mean) / 38% (mode) of `wall_adjacent`'s bias, direction agrees 7/7 scenarios | C2-NAV.31: 62–71% of the `wall_adjacent` bias remains unexplained by this component alone; dilation itself contributes only 0.4% | **REAL, PARTIAL** (FACT: contributes; not sufficient alone) |
| Temporal accumulation over the trajectory | — | **This session**: correlation(chain-position, residual) = +0.089 across 105 leg-instances, 5 runs; residual pattern is a repeating per-leg-type sawtooth, not a monotonic trend; C2-NAV.33 separately found the bias already present in the *first* pre-resample cloud (t=0.64s into `wall_adjacent`) | **REJECTED** — not the mechanism |
| Place/leg-type dependence | **This session**: mean AMCL−GT y-residual varies systematically and consistently by leg (`open_space` −0.112 m, `obstacle_corner` **+0.037 m**, `corridor_gate` −0.015 m — sign flips by location), same pattern across all 5 runs | None found this session | **CONFIRMED, extends C2-NAV.29/.31's map-linkage finding** — leading candidate for where the *unexplained* portion lives |
| Motion-model noise (variance, not the real-odometry mean bias) | C2-NAV.32: 200-seed replay, 95% CI [−0.00447, +0.00683] straddles zero; robustness checks (frame reversal, sub-stepping) all still ~0 or weakly north | — | **ELIMINATED** as primary driver |

## 5. Explicit classifications

**FACT** (measured this session or reproduced from a prior session's own instrumentation/disassembly, not inference):
- The published `/amcl_pose` and map→odom TF come from the highest-total-weight KD-tree cluster's mean, extracted from the post-resampling particle set, on a cycle cadence set by `resample_interval` (source-traced, `amcl_node.cpp:900-931`).
- `pf_update_resample` runs every scan cycle under this repo's `resample_interval: 1` and flattens weights to `1/N` (C2-NAV.33's disassembly+oracle, cross-referenced).
- The KD-tree clustering grid is hardcoded at 0.5 m × 0.5 m × 10° (this session, disassembly + `.rodata` decode, `libpf_lib.so+0x1ca0`).
- Cluster-selection/pose-extraction contributes ≤0.0003 m of shift versus the recorded particle-cloud centroid, confirmed independently in two separate sessions (C2-NAV.30, one run; this session, four runs).
- The bias's sign and magnitude are leg/place-dependent, not chain-position-dependent (this session, 105 leg-instances, corr = +0.089).
- Duplicated (resampled) particles in the recorded post-resample cloud sit slightly *north*, not south, of their own snapshot centroid (this session, 369 snapshots, CI excludes zero).
- The real (not GT-proxy) odometry input carries a small, sign-consistent, non-zero along-track bias, ~8x smaller than the output-side proxy figure (C2-NAV.37).

**OBSERVATION** (measured, but from n=1 runs, synthetic/reconstructed inputs, or otherwise narrower evidence than FACT status warrants):
- C2-NAV.29's raycast-reconstructed scan (not a recorded scan) scoring GT above AMCL in 98.7% of samples.
- C2-NAV.31's rebuilt likelihood-field budget table (29–38% explained at `wall_adjacent`) — the model is rebuilt from config, not extracted from the running binary's actual per-particle computation.
- C2-NAV.33's weighting `shift` result (n=9 pre-resample clouds, one run).
- C2-NAV.32's motion-noise elimination (GT substituted for the real odometry input available at the time; C2-NAV.37 has since measured the real input and confirms it is small, but the *noise variance* claim in C2-NAV.32 was never re-verified against real odometry).

**HYPOTHESIS** (not directly tested, offered as the best-supported remaining candidate):
- The 62–71% of the `wall_adjacent` bias unexplained by C2-NAV.31's specific likelihood-field/map reconstruction is itself some other place-dependent, geometry-or-discretization-linked effect (costmap inflation interacting with the controller's realized path near walls/corners; a sensor raycasting detail not captured by C2-NAV.29's reconstruction; a KD-tree clustering interaction that only matters for certain cloud shapes, not on average — none of these individually tested).

**UNKNOWN**:
- What specifically produces the place-dependent, sign-flipping pattern this session found across all 7 legs.
- Whether `pf_cluster_stats`/`pf_kdtree_cluster`'s actual implementation (never disassembled by any C2-NAV session, including this one) does anything beyond the textbook mean/covariance-per-cluster computation the header comments imply — e.g. how it handles circular yaw statistics, whether cluster covariance computation itself is numerically stable near the observed cloud shapes. This session relied on *recorded-data comparisons* (published pose vs. recorded centroid) rather than disassembling this specific function, and that comparison is a strong proxy but not a substitute for reading the function itself.
- Whether the real (C2-NAV.37-measured) odometry input's *lateral* (cross-track) component (pooled −0.002104 m/update, not examined against the proxy bias by C2-NAV.37, which compared only the along-track component) could, once correctly projected into world-frame via the robot's actual heading during each leg, account for a larger fraction of a place-dependent, heading-dependent bias than the along-track comparison alone suggests. Not computed in this session — flagged as a gap in C2-NAV.37's own magnitude comparison, not resolved here.

## 6. Why the measured +0.000747 m/update odometry residual is insufficient

C2-NAV.37 already established the headline comparison (input residual
~8x smaller than the historical output-side proxy). This session adds
two independent reasons the *magnitude* alone is not the whole story:

1. **It is a per-update mean, not a per-leg constant** — over a leg with
   order 10-30 AMCL updates (C2-NAV.30/.33 measured 9-21 fresh-cloud
   messages per `wall_adjacent` traversal), naive linear accumulation of
   +0.000747 m/update would total roughly 0.007-0.022 m — an order of
   magnitude short of 0.09-0.13 m even before accounting for AMCL's own
   correction step partially canceling accumulated drift each cycle.
2. **The observed bias does not accumulate with elapsed trajectory** (this
   session, §2/§4) — if the odometry residual's small per-update effect
   were the dominant driver, a monotonic growth with elapsed
   updates/distance would be the natural signature. None was found
   (corr = +0.089). This is independent evidence, not just a magnitude
   argument, that odometry input is not the primary mechanism.

## 7. Leading remaining hypothesis

**Place-dependent, geometry/discretization-linked effect, not yet fully
identified.** Ranked by how much of the pipeline has been tested and
eliminated:

1. Motion-model noise: eliminated (C2-NAV.32, this repo does not
   contest it).
2. Sensor weighting: eliminated as primary (C2-NAV.33 + this session's
   corroboration that pose-extraction tracks the recorded centroid almost
   exactly, meaning whatever the weights did upstream is what shows up,
   and C2-NAV.33 already showed the weights barely move the centroid).
3. Resampling-as-duplication: tested this session, wrong direction.
4. Cluster-selection/pose-extraction: ruled out, twice, independently.
5. Odometry input bias (C2-NAV.37, real): present but ~8x too small and
   non-accumulating.
6. Map/scan geometry (C2-NAV.29/.31): real, place-dependent, but only
   29-38% at the one leg it was fully budgeted for.

What is left, by elimination, is **whatever produces items 4-6's
combined but still-incomplete explanation** — and this session's own new
finding (strong, sign-flipping, leg-type dependence; no time dependence)
is structurally consistent with a place/geometry mechanism, not a
process-level one. This is not a proof; it is what remains after
subtracting every mechanism actually tested.

## 8. One minimal next decisive measurement, if needed

**Project C2-NAV.37's own measured odometry residual (both along-track
*and* lateral components) into world/map frame using each leg's actual
heading, and compare the resulting per-leg predicted contribution against
this session's own per-leg-type residual table (§4, "place/leg-type
dependence" row).** This is decisive because: C2-NAV.37 only compared its
along-track figure against C2-NAV.34's along-track proxy; the lateral
component (−0.002104 m/update pooled) was measured but never projected
into the world frame or checked against the *leg-to-leg* pattern this
session found. If the projected odometry contribution tracks the sign
flips seen across `open_space`/`obstacle_corner`/`corridor_gate`/etc.,
odometry regains standing as a place-dependent (heading-dependent)
contributor after all — reframing, not just discarding, C2-NAV.37's
result. If it does not track that pattern, odometry is more firmly ruled
out and the search should move to map/costmap-side, place-dependent
mechanisms specifically. This is computable entirely offline from data
already on disk (`~/coco_nav_runs/c2nav37_run{0..4}/`, the GT/odometry
sidecar CSVs plus the per-leg trace CSVs already used in this report) —
no new live experiment is proposed.

## 9. Things explicitly ruled out

- Scan-to-map registration/geometry as the *primary* cause (C2-NAV.29).
- Particle-cloud collapse or genuine multimodality (C2-NAV.30).
- The deployed likelihood-field sensor model preferring AMCL's pose over
  GT (C2-NAV.31: it does the opposite, 0/19).
- Weighting shifting the particle centroid by anything close to the
  observed bias (C2-NAV.33: 3%, CI straddles zero).
- Resampling's duplication mechanism concentrating support southward
  (this session: tested, wrong-signed effect).
- Cluster-selection/pose-extraction as a meaningful separate contributor
  (C2-NAV.30 and this session, independently, twice).
- Motion-model noise as a systematic (as opposed to per-seed-random)
  contributor (C2-NAV.32).
- Temporal/trajectory accumulation as the driving pattern (this session).

**What was NOT tested and is not "ruled out" by anything above**: the
actual implementation of `pf_cluster_stats`/`pf_kdtree_cluster`
(disassembly never done, any session); whether costmap inflation or the
controller's realized path differs place-dependently in a way that
correlates with AMCL's own bias pattern; the lateral (not along-track)
projection of the real measured odometry residual (§6/§8).

## 10. What NOT to change yet

Per this task's own scope and the standing rules in `CLAUDE.md`: no AMCL
parameter (`alpha1..5`, `resample_interval`, `laser_model_type`,
likelihood-field parameters), no map, no costmap/inflation settings, no
controller settings, no `PolygonStop`/`PolygonSlow`, no goal tolerances.
None of this session's findings constitute a validated root cause —
`resample_interval` in particular must stay at its shipped value of `1`;
C2-NAV.33 already found that raising it to `2` is "not neutral and not
proposed" (halves the AMCL pose publish rate). Nothing here should be
read as a recommendation to tune anything; it narrows *where to look
next*, not what to change.

---

## Git state at completion

```
git status  # (at time of writing)
  modified:   coco_nav_diag/CMakeLists.txt        (C2-NAV.37, pre-existing, uncommitted)
  modified:   docs/SESSION_LOG.md                 (C2-NAV.37, pre-existing, uncommitted)
  modified:   docs/agents/HANDOFF.md               (C2-NAV.37 + this task)
  new file:   docs/agents/C2-NAV.37_RESULTS.md      (pre-existing, uncommitted)
  new file:   docs/agents/C2-NAV.38_RESULTS.md      (this task)
```

No `main` changes, no merges, no history rewrites, no force-push — this
worktree's own branch only, still uncommitted pending the user's earlier
go-ahead for the C2-NAV.37 changes.
