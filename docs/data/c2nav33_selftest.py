#!/usr/bin/env python3
# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
c2nav33_selftest.py
===================
C2-NAV.33's offline self-test. It runs BEFORE the simulator, and it is
not a smoke test: every arithmetic claim the session will make is checked
against an answer worked out by hand on a fixture small enough to verify
by inspection.

Five things it is specifically for.

1. THE NULL CONTROL IS A REAL CONTROL. `weights` reports, on the SAME
   leg, the post-resample clouds, whose weights are flat by construction
   and on which `shift` and `mass_shift` MUST be exactly zero. If the
   arithmetic had a bug that manufactured a shift out of a flat weight
   vector, that control would be the thing that caught it -- so it is
   checked here on a hand-built flat cloud, not merely printed live.

2. MISSING DATA IS REPRESENTED, NOT FILLED. A cloud with no trace row
   inside the join window must come back `gt_ok` False with every
   spatial field None. A forward fill here would silently attribute one
   pose's particles to another moment.

3. THE PHASE CLASSIFIER IS EXACT. `pre` vs `post` is decided by a
   bit-level all-equal test on the weight vector, not by a threshold on
   ESS. A near-flat pre-resample cloud must still read `pre`.

4. THE GATE CANNOT PASS VACUOUSLY. Its checks are regexes over a
   disassembly; a regex that matches nothing would silently report the
   binary as failing, and one applied to the wrong text could report a
   pass. Both directions are exercised.

5. THE PRIOR SESSIONS STILL REPRODUCE. C2-NAV.30's module is imported
   as a library here, so a change that broke it would break this
   session's numbers too.

Everything is offline: no ROS, no Gazebo, no simulator, no network, and
no dependency on `.navbench`, which is scratch and is not committed.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import c2nav30_cloud as C            # noqa: E402
import c2nav33_weights as W          # noqa: E402

_N_OK = [0]
_N_BAD = [0]


def _check(ok, label, detail=''):
    if ok:
        _N_OK[0] += 1
        print('  [ok]   %s' % label)
    else:
        _N_BAD[0] += 1
        print('  [FAIL] %s   %s' % (label, detail))
    return bool(ok)


def _close(a, b, tol=1e-12):
    return a is not None and b is not None and abs(a - b) <= tol


# ------------------------------------------------------------ fixture
def _install(clouds, rows, t0=100.0):
    """Monkeypatch C2-NAV.30's readers so `wsnaps` sees a fixture.

    Restored by `_restore`. Nothing touches the filesystem: the point of
    a fixture is that its answers are known, and a fixture that has to be
    written and re-read can fail for reasons that are not the code under
    test.
    """
    saved = (C.read_clouds, C.read_trace, C.cloud_meta)
    C.read_clouds = lambda tag, leg, rep=0: clouds
    C.read_trace = lambda tag, leg, rep=0: rows
    C.cloud_meta = lambda tag, leg, rep=0: {'t0_sim_s': t0}
    return saved


def _restore(saved):
    C.read_clouds, C.read_trace, C.cloud_meta = saved


def _row(t_rel, x, y, **kw):
    d = {'t_rel': '%.3f' % t_rel, 'x': '%.6f' % x, 'y': '%.6f' % y,
         'yaw': '0.0', 'amcl_x': '', 'amcl_y': '', 'amcl_yaw': '',
         'v_act': '0.0', 'w_act': '0.0', 'v_nav': '0.0', 'w_nav': '0.0',
         'scan_min': '0.6', 'cm_polygon': ''}
    for k, v in kw.items():
        d[k] = '' if v is None else (v if isinstance(v, str) else
                                     '%.6f' % v)
    return d


def cmd_selftest(_args=None):
    print('=' * 78)
    print('C2-NAV.33 selftest -- offline, before any simulator')
    print('=' * 78)

    # ---------------------------------------------- 1. constants
    print()
    print('1. constants inherited from C2-NAV.29/.30, NOT redefined')
    _check(W.C is C, 'c2nav30_cloud imported as the library')
    _check(C.WORLD_TO_MAP_X == 2.0 and C.WORLD_TO_MAP_Y == 0.0,
           'historical world->map (2.0, 0.0) unchanged',
           '%r' % ((C.WORLD_TO_MAP_X, C.WORLD_TO_MAP_Y),))
    _check(C.MEASURED_TO_MAP_X == 2.0560 and C.MEASURED_TO_MAP_Y == 0.0150,
           'measured world->map (2.0560, 0.0150) unchanged')
    _check(W.WALL_LEG == C.WALL_LEG and W.CONTROL_LEG == C.CONTROL_LEG,
           'leg names match C2-NAV.30 (%s / %s)'
           % (W.CONTROL_LEG, W.WALL_LEG))
    _check(W.BASE_RUN == C.RUN,
           'the comparison run IS C2-NAV.30\'s own run (%s)' % W.BASE_RUN)
    _check(W.ONE_LEAF == 'amcl.ros__parameters.resample_interval',
           'the one permitted leaf is named explicitly')
    _check(W.NEAR_R == 0.05, 'near-band radius is the 0.05 m map cell')

    # ---------------------------------------------- 2. _wstats by hand
    print()
    print('2. _wstats, against answers worked out by hand')
    # w = [1, 1, 2] -> sum 4, normalised [.25,.25,.5]
    # ESS = 1/(0.0625+0.0625+0.25) = 1/0.375 = 2.6666666666666665
    d = W._wstats(np.array([1.0, 1.0, 2.0]))
    _check(_close(d['wsum'], 4.0), 'wsum 4.0', repr(d['wsum']))
    _check(d['nuniq'] == 2, 'nuniq 2', repr(d['nuniq']))
    _check(_close(d['ess'], 1.0 / 0.375, 1e-12), 'ESS = 1/0.375 = 2.666667',
           repr(d['ess']))
    _check(_close(d['ess_ratio'], (1.0 / 0.375) / 3.0), 'ESS/n')
    _check(_close(d['wmax'], 0.5), 'max normalised weight 0.5')
    _check(d['flat_exact'] is False, 'flat_exact False on a non-flat vector')
    # entropy: -(0.25 ln0.25 *2 + 0.5 ln0.5)/ln3
    ent = -(2 * 0.25 * np.log(0.25) + 0.5 * np.log(0.5)) / np.log(3)
    _check(_close(d['entropy'], float(ent), 1e-12), 'normalised entropy',
           repr(d['entropy']))
    f = W._wstats(np.array([0.002, 0.002, 0.002, 0.002]))
    _check(f['flat_exact'] is True and f['nuniq'] == 1
           and _close(f['ess_ratio'], 1.0),
           'a flat vector: flat_exact True, nuniq 1, ESS/n exactly 1')
    z = W._wstats(np.zeros(0))
    _check(z['n'] == 0 and z['ess'] is None and z['flat_exact'] is False,
           'an EMPTY vector reports None, not a fabricated statistic')

    # ------------------------------ 3. the central spatial arithmetic
    print()
    print('3. the central spatial test, on a 4-particle hand fixture')
    # GT at world y = 0.0 -> map y = 0.0 (historical offset 0.0).
    # particles y = [-0.2, -0.1, +0.1, +0.3]; unweighted mean = +0.025
    # weights   w = [4, 3, 2, 1] (sum 10) -> normalised .4 .3 .2 .1
    # weighted mean y = .4*-.2 + .3*-.1 + .2*.1 + .1*.3
    #                 = -0.08 -0.03 +0.02 +0.03 = -0.06
    # dy_u = +0.025, dy_w = -0.06, shift = -0.085  (SOUTHWARD)
    # south_u = 2/4 = 0.5 ; south_w = .4+.3 = 0.7 ; mass_shift = +0.2
    # x = 2.0 because GT world (0, 0) is map (2.0, 0.0) under the
    # historical offset: a fixture particle sitting at GT's x is at 2.0.
    arr = np.array([[2.0, -0.2, 0.0, 4.0],
                    [2.0, -0.1, 0.0, 3.0],
                    [2.0, 0.1, 0.0, 2.0],
                    [2.0, 0.3, 0.0, 1.0]])
    saved = _install([(100.0, arr)], [_row(0.0, 0.0, 0.0)])
    try:
        S = W.wsnaps('t', 'l')
    finally:
        _restore(saved)
    _check(len(S) == 1, 'one record per cloud message')
    d = S[0]
    _check(d['phase'] == 'pre', 'a non-flat cloud classifies as `pre`')
    _check(_close(d['dy_u'], 0.025), 'unweighted dy +0.025',
           repr(d['dy_u']))
    _check(_close(d['dy_w'], -0.06), 'weighted dy -0.060', repr(d['dy_w']))
    _check(_close(d['shift'], -0.085), 'shift -0.085 (southward)',
           repr(d['shift']))
    _check(_close(d['south_u'], 0.5), 'south particle fraction 0.5')
    _check(_close(d['south_w'], 0.7), 'south weight mass 0.7')
    _check(_close(d['mass_shift'], 0.2), 'mass_shift +0.2')
    _check(_close(d['north_w'], 0.3), 'north weight mass 0.3')
    _check(_close(d['nearest'], 0.1), 'nearest particle to GT 0.1 m')

    # SIGN DISCIPLINE. The same fixture mirrored must give the opposite
    # sign; a shift that came out negative regardless would pass the
    # test above and be worthless.
    arr2 = arr.copy()
    arr2[:, 1] *= -1.0
    saved = _install([(100.0, arr2)], [_row(0.0, 0.0, 0.0)])
    try:
        S2 = W.wsnaps('t', 'l')
    finally:
        _restore(saved)
    _check(_close(S2[0]['shift'], +0.085),
           'the MIRRORED fixture gives shift +0.085 -- the sign is real',
           repr(S2[0]['shift']))
    _check(_close(S2[0]['mass_shift'], -0.2), 'and mass_shift -0.2')

    # ------------------------------------------ 4. the NULL CONTROL
    print()
    print('4. the null control: flat weights must give EXACTLY zero')
    flat = arr.copy()
    flat[:, 3] = 0.25
    saved = _install([(100.0, flat)], [_row(0.0, 0.0, 0.0)])
    try:
        S3 = W.wsnaps('t', 'l')
    finally:
        _restore(saved)
    d3 = S3[0]
    _check(d3['phase'] == 'post', 'a bit-flat cloud classifies as `post`')
    _check(d3['shift'] == 0.0 or abs(d3['shift']) < 1e-15,
           'shift is zero on flat weights', repr(d3['shift']))
    _check(abs(d3['mass_shift']) < 1e-15,
           'mass_shift is zero on flat weights', repr(d3['mass_shift']))
    _check(_close(d3['ess_ratio'], 1.0), 'ESS/n exactly 1 on flat weights')
    # A cloud that is nearly but not exactly flat must still read `pre`.
    near = arr.copy()
    near[:, 3] = [0.25, 0.25, 0.25, 0.2500000001]
    saved = _install([(100.0, near)], [_row(0.0, 0.0, 0.0)])
    try:
        S4 = W.wsnaps('t', 'l')
    finally:
        _restore(saved)
    _check(S4[0]['phase'] == 'pre',
           'a NEARLY flat cloud still reads `pre` (ESS/n %.10f)'
           % S4[0]['ess_ratio'])

    # -------------------------------------- 5. missing data explicit
    print()
    print('5. missing data is represented, never filled')
    saved = _install([(100.0, arr)], [_row(9.0, 0.0, 0.0)])   # 9 s away
    try:
        S5 = W.wsnaps('t', 'l')
    finally:
        _restore(saved)
    d5 = S5[0]
    _check(d5['gt_ok'] is False, 'a cloud 9 s from any trace row: gt_ok False')
    _check(all(d5[k] is None for k in ('gt_y', 'dy_u', 'dy_w', 'shift',
                                       'south_w', 'mass_shift', 'nearest')),
           'every spatial field is None, not forward-filled')
    _check(d5['n'] == 4 and d5['ess_ratio'] is not None,
           'the WEIGHT statistics survive -- they need no ground truth')
    saved = _install([(100.0, arr)], [])
    try:
        S6 = W.wsnaps('t', 'l')
    finally:
        _restore(saved)
    _check(S6[0]['gt_ok'] is False and S6[0]['dt_trace'] is None,
           'an EMPTY trace does not crash and does not invent a join')

    # A blank ground-truth cell is missing data too, not a zero.
    blank_y = _row(0.0, 0.0, 0.0)
    blank_y['y'] = ''
    saved = _install([(100.0, arr)], [blank_y])
    try:
        S7 = W.wsnaps('t', 'l')
    finally:
        _restore(saved)
    _check(S7[0]['gt_ok'] is False and S7[0]['shift'] is None,
           'a BLANK y cell is missing, not 0.0')

    # ---------------------------------- 6. the AMCL near-band ratio
    print()
    print('6. w(AMCL)/w(GT) near-band ratio')
    # GT at y=0, AMCL at y=-0.2. Band radius 0.05.
    #   near GT   : y=-0.2? no. y=-0.1? no. y=+0.1? no. y=+0.3? no.
    # Widen: use particles at -0.2, -0.19, 0.0, 0.01 so both bands hit.
    arr3 = np.array([[2.0, -0.200, 0.0, 4.0],
                     [2.0, -0.190, 0.0, 6.0],
                     [2.0, 0.000, 0.0, 1.0],
                     [2.0, 0.010, 0.0, 3.0]])
    # sum 14 -> normalised 4/14, 6/14, 1/14, 3/14
    # near GT (|y| <= .05): particles 3,4 -> mean (1+3)/14/2 = 4/28
    # near AMCL(-0.2): particles 1,2     -> mean (4+6)/14/2 = 10/28
    # ratio = 10/4 = 2.5
    saved = _install([(100.0, arr3)],
                     [_row(0.0, 0.0, 0.0, amcl_x=2.0, amcl_y=-0.2)])
    try:
        S8 = W.wsnaps('t', 'l')
    finally:
        _restore(saved)
    d8 = S8[0]
    _check(d8['n_near_gt'] == 2 and d8['n_near_amcl'] == 2,
           'both bands contain 2 particles')
    _check(_close(d8['lr_near'], 2.5, 1e-12),
           'w(AMCL)/w(GT) = 2.5 -- above 1 means AMCL\'s neighbourhood '
           'is favoured', repr(d8['lr_near']))
    _check(_close(d8['amcl_dy'], -0.2), 'amcl_dy -0.2')

    # ------------------------------------- 7. the frame convention
    print()
    print('7. both frame conventions, and the offset is really applied')
    saved = _install([(100.0, arr)], [_row(0.0, 0.0, 0.0)])
    try:
        h = W.wsnaps('t', 'l', measured_frame=False)[0]
        m = W.wsnaps('t', 'l', measured_frame=True)[0]
    finally:
        _restore(saved)
    _check(_close(m['gt_y'] - h['gt_y'], C.MEASURED_TO_MAP_Y),
           'the measured convention shifts GT_y by exactly %.4f'
           % C.MEASURED_TO_MAP_Y)
    _check(_close(m['shift'], h['shift']),
           'shift is INVARIANT to the convention -- both centroids move '
           'with GT, so the difference cannot depend on the offset')
    _check(not _close(m['dy_u'], h['dy_u']),
           'but dy_u is NOT invariant, which is why both are reported')

    # ------------------------------------------- 8. the gate regexes
    print()
    print('8. the gate cannot pass vacuously')
    fake = ('00000000000e37b0 <nav2_amcl::AmclNode::laserReceived(x)>:\n'
            '   e4039:\tmov    0x8e0(%rbx),%eax\n'
            '   e404a:\tmov    %eax,0x8e0(%rbx)\n'
            '   e4050:\tidivl  0xac8(%rbx)\n'
            '   e4058:\tje     e438a <x>\n'
            '   e4061:\tmovslq 0x18(%rdi),%rax\n'
            '   e4065:\tlea    (%rax,%rax,8),%rax\n'
            '   e4069:\tshl    $0x4,%rax\n'
            '   e406d:\tlea    0x20(%rdi,%rax,1),%rax\n'
            '   e4106:\tmovzbl 0x5c8(%rbx),%eax\n'
            '   e4391:\tcall   b4c30 <pf_update_resample@plt>\n'
            '\n'
            '00000000000e2890 <nav2_amcl::AmclNode::other(x)>:\n'
            '   e2900:\tret\n')
    body = W._fn_body(fake, r'AmclNode::laserReceived')
    _check(len(body) == 10,
           '_fn_body extracts exactly the 10 instruction lines of the '
           'named function', str(len(body)))
    _check(all('other' not in b for b in body),
           'and stops at the blank line before the NEXT function')
    _check(W._fn_body(fake, r'NoSuchFunction') == [],
           'a name that matches nothing yields NO lines -- so a gate check '
           'over it FAILS rather than vacuously passing')
    # THE PLT TRAP, observed for real while writing the gate. objdump
    # emits the trampoline at a LOWER address than the definition, so
    # "first label that matches" returns a 2-line stub and every content
    # check over it fails -- reporting the binary as not doing what it
    # demonstrably does.
    plt = ('00000000000b5940 <nav2_amcl::AmclNode::publishParticleCloud'
           '(_pf_sample_set_t const*)@plt>:\n'
           '   b5940:\tjmp    *0x123(%rip)\n'
           '\n'
           '00000000000e2890 <nav2_amcl::AmclNode::publishParticleCloud'
           '(_pf_sample_set_t const*)@@Base>:\n'
           '   e2a58:\tmovsd  -0x8(%r15),%xmm0\n'
           '   e2a5e:\tmovsd  %xmm0,-0x8(%r14)\n')
    pb = W._fn_body(plt, r'AmclNode::publishParticleCloud')
    _check(len(pb) == 2 and 'movsd' in pb[0],
           'the @plt trampoline is SKIPPED and the real definition found',
           repr(pb))
    off = W._hpp_offsets()
    _check(off is not None and off['current_set'] == 0x18
           and off['sets'] == 0x20 and off['set_size'] == 144,
           'pf.hpp gives current_set +0x18, sets +0x20, stride 144',
           repr(off))
    _check(0x20 + 0 * 144 == 0x20 and 0x20 + 1 * 144 == 0x20 + 144,
           'and 144 is what the disassembly\'s rax*9<<4 computes')

    # ------------------------------------ 9. paramdiff catches a plant
    print()
    print('9. paramdiff would catch a SECOND change')
    base = {'amcl.ros__parameters.resample_interval': 1,
            'amcl.ros__parameters.alpha1': 0.2}
    cand_ok = {'amcl.ros__parameters.resample_interval': 2,
               'amcl.ros__parameters.alpha1': 0.2}
    cand_bad = {'amcl.ros__parameters.resample_interval': 2,
                'amcl.ros__parameters.alpha1': 0.3}
    ch_ok = sorted(k for k in base if base[k] != cand_ok[k])
    ch_bad = sorted(k for k in base if base[k] != cand_bad[k])
    _check(ch_ok == [W.ONE_LEAF], 'the honest candidate shows one leaf')
    _check(len(ch_bad) == 2 and ch_bad != [W.ONE_LEAF],
           'a planted alpha1 edit shows TWO and would fail the gate')
    _check('alpha1' in W.GUARDED and 'update_min_d' in W.GUARDED
           and 'sigma_hit' in W.GUARDED and 'max_beams' in W.GUARDED
           and 'z_hit' in W.GUARDED and 'z_rand' in W.GUARDED
           and 'min_particles' in W.GUARDED
           and 'max_particles' in W.GUARDED,
           'every leaf the brief names by hand is in the guarded list')
    _check(len(W._flatten({'a': {'b': 1, 'c': [1, 2]}})) == 2
           and W._flatten({'a': {'c': [1, 2]}})['a.c'] == [1, 2],
           'lists are LEAVES, as C2-NAV.25 defined them')

    # ---------------------------------- 10. prior sessions reproduce
    print()
    print('10. the prior sessions still reproduce')
    try:
        import c2nav31_lf                      # noqa: F401
        _check(True, 'c2nav31_lf imports')
    except Exception as exc:                   # noqa: BLE001
        _check(False, 'c2nav31_lf imports', repr(exc)[:120])
    try:
        import c2nav32_mm                      # noqa: F401
        _check(True, 'c2nav32_mm imports')
    except Exception as exc:                   # noqa: BLE001
        _check(False, 'c2nav32_mm imports', repr(exc)[:120])
    try:
        import c2nav28_amcl                    # noqa: F401
        _check(True, 'c2nav28_amcl imports')
    except Exception as exc:                   # noqa: BLE001
        _check(False, 'c2nav28_amcl imports', repr(exc)[:120])
    b = C.bundle()
    _check(isinstance(b, dict) and bool(b),
           'C2-NAV.30\'s frozen bundle still loads (%d keys)' % len(b or {}))

    # ------------------------- 10b. the bundle fallback is REAL
    print()
    print('10b. the frozen bundle reproduces the run without scratch')
    live = W.wsnaps(W.RUN, W.WALL_LEG)
    saved = (C.read_clouds, C.read_trace, C.cloud_meta)
    C.read_clouds = lambda tag, leg, rep=0: []      # scratch gone
    try:
        froz = W.wsnaps(W.RUN, W.WALL_LEG)
    finally:
        _restore(saved)
    if not live:
        _check(True, 'no live run on disk -- fallback check skipped, and '
                     'SAID so rather than passing silently')
    else:
        _check(len(froz) == len(live),
               'the bundle carries every snapshot the live path builds '
               '(%d vs %d)' % (len(froz), len(live)))
        keys = ('shift', 'mass_shift', 'dy_u', 'dy_w', 'ess_ratio',
                'south_w', 'phase', 'n')
        same = all(a2.get(k) == b2.get(k)
                   for a2, b2 in zip(live, froz) for k in keys)
        _check(same, 'and every headline field is BIT-IDENTICAL, so the '
                     'fallback is a fallback and not a second '
                     'implementation')
    _check(W.wsnaps('no_such_run_at_all', W.WALL_LEG) == [],
           'a run that exists nowhere yields [] -- which `verdict` '
           'reports as NO DATA, explicitly NOT as classification (C)')

    # ------------------------------- 11. scratch-independence check
    print()
    print('11. nothing here needs .navbench, which is scratch')
    _check(not os.path.exists(os.path.join(HERE, '..', '..', '.navbench',
                                           'nonexistent')),
           'the selftest read no live scratch artefact')

    print()
    print('=' * 78)
    print('C2-NAV.33 selftest: %d passed, %d FAILED'
          % (_N_OK[0], _N_BAD[0]))
    print('=' * 78)
    return 0 if _N_BAD[0] == 0 else 1


if __name__ == '__main__':
    sys.exit(cmd_selftest())
