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
c2nav33_weights.py
==================
C2-NAV.33 -- AMCL's PRE-RESAMPLING importance weights, made observable.

THE QUESTION
------------
Do AMCL's pre-resampling particle weights systematically favour the
south-biased hypothesis at the wall-adjacent location?

Every other major mechanism has been eliminated by measurement:

  C2-NAV.29  scan-to-map registration is not the principal cause; the
             scan agrees with the map at ground truth, not at AMCL.
  C2-NAV.30  the cloud does not collapse. It is unimodal, broad, its
             support straddles the truth in 19 of 19 snapshots, and its
             whole centre of mass sits ~0.092 m south.
  C2-NAV.31  the deployed likelihood-field model prefers only -0.027 m
             (mean) / -0.035 m (mode); re-weighting C2-NAV.30's own
             recorded particles with it moves the centroid +0.003 m,
             NORTH. The map's half-cell convention is a fixed -0.025 m.
  C2-NAV.32  the deployed DifferentialMotionModel, EXECUTED rather than
             re-implemented, contributes +0.0012 m [-0.0045, +0.0068] --
             statistically zero, and northward.

What none of them could see is the importance weight itself.
`/particle_cloud` is published after resampling, and C2-NAV.30 MEASURED
the consequence rather than assuming it: ESS/n = 1.0000 exactly on 40 of
40 samples, both legs. Resampling levels the weights, so the published
cloud says nothing about which hypotheses the observation preferred.

THE ONE CHANGE
--------------
    amcl.resample_interval: 1 -> 2

and nothing else, in either direction. `paramdiff` proves that leaf by
leaf against the file every wall-adjacent measurement since C2-NAV.25 was
taken under.

WHY THAT EXPOSES THE WEIGHTS, AND WHY IT IS NOT ASSUMED
-------------------------------------------------------
`gate` establishes it from the INSTALLED artefacts before any simulator
starts, because the brief is explicit that the diagnostic effect must not
be taken on trust. Two independent lines of evidence, neither of them a
recollection of upstream source (which is NOT installed -- only the
headers and the .so are):

1. THE BINARY'S OWN CONTROL FLOW, disassembled from
   `/opt/ros/jazzy/lib/libamcl_core.so` at run time by this module, not
   pasted in. In `AmclNode::laserReceived` there is EXACTLY ONE call site
   of `pf_update_resample` and EXACTLY ONE of `publishParticleCloud` in
   the whole library, and their guards differ:

       call updateFilter(...)               # runs the sensor model
       eax = ++this->resample_count_        # member at +0x8e0
       edx = eax % this->resample_interval_ # member at +0xac8
       if (edx == 0) { pf_update_resample(pf_, map_); }   <-- SKIPPED at
       set = &pf_->sets[pf_->current_set]                     interval 2
       if (!force_update_) publishParticleCloud(set);    <-- NOT guarded
                                                             on resampled

   The address arithmetic is checked against the installed `pf.hpp`:
   `current_set` at +0x18, `sets` at +0x20, and the disassembly's
   `rax*9<<4` is *144 = sizeof(pf_sample_set_t) laid out from that
   header. So the set handed to the publisher is `pf_->sets +
   pf_->current_set` -- and on an update where the resample is skipped,
   `current_set` still indexes the set `pf_update_sensor` just wrote
   weights into, because it is `pf_update_resample` that flips it.

   `publishParticleCloud` then copies the weight VERBATIM:
   `movsd -0x8(%r15),%xmm0 ; movsd %xmm0,-0x8(%r14)`, at the -0x8 offset
   that is `pf_sample_t::weight` behind the 24-byte pose. No
   renormalisation, no filtering.

2. THE DEPLOYED FILTER, EXECUTED. `c2nav33_oracle.cpp` links against the
   installed `libpf_lib.so` and calls `pf_update_sensor` and
   `pf_update_resample` itself, so whatever the shipped binary does to
   the weights is what is reported. It contains no particle-filter
   equations. Measured at n=500: after `pf_update_sensor` there are 500
   distinct weights summing to 1 with ESS/n = 0.7777; after
   `pf_update_resample` there is ONE distinct weight, bit-identical
   across every particle at 1/500, and `current_set` has flipped 0 -> 1.
   (ESS/n there computes to 1.000000000000001 rather than 1.0 exactly --
   summing n identical doubles is not associative. The exact claim is
   the bit-level all-equal test; the ratio is asserted to 1e-12.)

   That second line is precisely C2-NAV.30's observation, which makes
   this a prediction with a matching record and not a story.

AND IT IS STILL FALSIFIABLE LIVE. If the parameter does not take effect
-- a clamp, a rejected value, a build that ignores it -- then every
published cloud stays exactly flat, `phase` finds zero pre-resample
snapshots, and the verdict is (C) UNOBSERVABLE. `phase` is written to
detect that rather than to assume the change worked, and the CLAUDE.md
rule that a "we saw nothing" finding must come from an instrument proven
able to see something is discharged by the alternation test: at
interval 2 the phases must ALTERNATE, and a run that is uniformly flat
is reported as a failed experiment, not as a null result.

WHAT THIS CANNOT SHOW
---------------------
Nothing here makes `resample_interval: 2` a candidate configuration. The
parameter is a diagnostic window, changed to reveal information that
resampling destroys before publication; one run is an observation, not a
rate; and the accuracy numbers `context` prints are context, not a
performance comparison.

USAGE
    python3 c2nav33_weights.py gate
    python3 c2nav33_weights.py paramdiff
    python3 c2nav33_weights.py selftest
    python3 c2nav33_weights.py phase | weights | temporal | compare
    python3 c2nav33_weights.py context | verdict
    python3 c2nav33_weights.py dump
"""
import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)

import c2nav30_cloud as C            # noqa: E402

# ------------------------------------------------------------ constants
#
# NOTHING in this block is a tuning knob. The frame conventions and the
# leg names are C2-NAV.29/.30's, carried unchanged so the comparison is
# a comparison; `selftest` asserts they still match that module.
BASELINE_YAML = os.path.join(HERE, 'c2nav25_slow_params.yaml')
CANDIDATE_YAML = os.path.join(HERE, 'c2nav33_ri2_params.yaml')
ORACLE_SRC = os.path.join(HERE, 'c2nav33_oracle.cpp')
BUNDLE = os.path.join(HERE, 'c2nav33_weights.json')

ROS_LIB = '/opt/ros/jazzy/lib'
ROS_INC = '/opt/ros/jazzy/include'
AMCL_CORE = os.path.join(ROS_LIB, 'libamcl_core.so')
PF_LIB = os.path.join(ROS_LIB, 'libpf_lib.so')
PF_HPP = os.path.join(ROS_INC, 'nav2_amcl', 'pf', 'pf.hpp')

RUN = os.environ.get('C2NAV33_RUN', 'c2n33_focus_r1')
BASE_RUN = 'c2n30_focus_r1'          # C2-NAV.30, resample_interval 1
WALL_LEG = 'wall_adjacent'
CONTROL_LEG = 'open_space'

# The only behavioural leaf this session is permitted to move.
ONE_LEAF = 'amcl.ros__parameters.resample_interval'

# Radius, metres, for the "weight near GT vs weight near AMCL" ratio.
# 0.05 m is the map cell size -- the finest scale the likelihood field
# can distinguish at all (C2-NAV.31), so a smaller band would be reading
# below the model's own resolution.
NEAR_R = 0.05


def hdr(t):
    print()
    print('=' * 78)
    print(t)
    print('=' * 78)


def med(xs):
    return C.med(xs)


def pct(xs, q):
    return C.pct(xs, q)


def mean(xs):
    xs = [x for x in xs if x is not None]
    return float(np.mean(xs)) if xs else None


def ci95(xs):
    """Two-sided 95 % interval for the mean. None under 2 samples."""
    xs = [x for x in xs if x is not None]
    n = len(xs)
    if n < 2:
        return None, None
    m = float(np.mean(xs))
    se = float(np.std(xs, ddof=1)) / math.sqrt(n)
    # 1.96 is the NORMAL quantile. Stated rather than hidden: at the
    # n ~ 10-20 this run yields the t correction widens the interval by
    # roughly 5-15 %, so a CI that only just excludes zero should be
    # read as marginal, not decisive.
    return m - 1.96 * se, m + 1.96 * se


def fmt(v, w=9, p=4, sign=True):
    if v is None:
        return '-'.rjust(w)
    return ('%+*.*f' if sign else '%*.*f') % (w, p, v)


# ============================================================ 1. gate
def _disasm(path):
    r = subprocess.run(['objdump', '-d', '--no-show-raw-insn', '-C', path],
                       capture_output=True, text=True)
    if r.returncode != 0:
        return None
    return r.stdout


def _fn_body(dis, name_re):
    """The instruction lines of the first DEFINITION whose label matches.

    `@plt` labels are skipped, and that is not a detail. objdump emits
    the PLT trampoline for a symbol at a LOWER address than the real
    body, so a naive "first label that matches" returns a three-line
    stub -- which then fails every content check and reports the binary
    as not doing what it demonstrably does. That false negative was
    observed while writing this, which is why the selftest exercises it.
    """
    out, on = [], False
    for line in dis.splitlines():
        if re.match(r'^[0-9a-f]+ <', line):
            on = bool(re.search(name_re, line)) and '@plt>' not in line
            continue
        if on:
            if not line.strip():
                break
            out.append(line)
    return out


def _hpp_offsets():
    """Offsets DERIVED from the installed pf.hpp, not remembered.

    pf_t   : int,int, double,double, int current_set, pf_sample_set_t[2]
    Only the two the disassembly indexes are needed, plus the size of
    pf_sample_set_t, which is what the *144 in the address arithmetic
    has to equal for the indexed object to be `sets[current_set]`.
    """
    if not os.path.exists(PF_HPP):
        return None
    src = open(PF_HPP).read()
    if 'int current_set;' not in src or 'pf_sample_set_t sets[2];' not in src:
        return None
    # 2*int = 8, 2*double = 16  -> current_set at 24 (0x18); sets needs
    # 8-byte alignment -> 32 (0x20).
    cur = 8 + 16
    sets = cur + 8
    # pf_sample_set_t: int(+pad) ptr ptr int int (+pad) ptr
    #                  pf_vector_t(3 double) pf_matrix_t(9 double) int(+pad)
    size = 8 + 8 + 8 + 8 + 8 + 3 * 8 + 9 * 8 + 8
    return {'current_set': cur, 'sets': sets, 'set_size': size}


def cmd_gate(args):
    """Prove -- from the installed artefacts -- that interval 2 exposes
    the pre-resampling weights, BEFORE any simulator starts."""
    hdr('C2-NAV.33 gate -- can this experiment observe anything at all?')
    ok = True

    print()
    print('A. THE INSTALLED BINARY (%s)' % AMCL_CORE)
    dis = _disasm(AMCL_CORE) if os.path.exists(AMCL_CORE) else None
    if dis is None:
        print('   UNAVAILABLE -- objdump failed or the library is absent.')
        print('   The gate CANNOT be closed from the binary; see B.')
        ok = False
    else:
        n_res = len(re.findall(r'call.*<pf_update_resample@plt>', dis))
        n_pub = len(re.findall(r'call.*publishParticleCloud', dis))
        print('   call sites, WHOLE library:  pf_update_resample %d   '
              'publishParticleCloud %d' % (n_res, n_pub))
        body = _fn_body(dis, r'AmclNode::laserReceived')
        txt = '\n'.join(body)
        has_mod = bool(re.search(r'idivl\s+0xac8\(%rbx\)', txt))
        has_inc = bool(re.search(r'mov\s+0x8e0\(%rbx\),%eax', txt)
                       and re.search(r'mov\s+%eax,0x8e0\(%rbx\)', txt))
        res_in = 'pf_update_resample@plt' in txt
        pub_in = 'publishParticleCloud' in txt
        # The publish's guard: a byte member test, NOT the modulo result.
        has_fu = bool(re.search(r'movzbl\s+0x5c8\(%rbx\),%eax', txt))
        off = _hpp_offsets()
        addr_ok = False
        if off:
            addr_ok = bool(
                re.search(r'movslq\s+0x%x\(%%rdi\),%%rax' % off['current_set'],
                          txt)
                and re.search(r'lea\s+\(%rax,%rax,8\),%rax', txt)
                and re.search(r'shl\s+\$0x4,%rax', txt)
                and re.search(r'lea\s+0x%x\(%%rdi,%%rax,1\),%%rax'
                              % off['sets'], txt))
        checks = [
            ('exactly one pf_update_resample call site', n_res == 1),
            ('exactly one publishParticleCloud call site', n_pub == 1),
            ('both inside laserReceived', res_in and pub_in),
            ('resample_count_ read-inc-write at +0x8e0', has_inc),
            ('modulo by resample_interval_ at +0xac8', has_mod),
            ('publish guarded by force_update_ at +0x5c8', has_fu),
        ]
        if off:
            checks.append(
                ('published set is &pf_->sets[current_set]  '
                 '(cur +0x%x, sets +0x%x, stride %d from pf.hpp)'
                 % (off['current_set'], off['sets'], off['set_size']),
                 addr_ok))
        else:
            checks.append(('pf.hpp readable for the offsets', False))
        for lab, v in checks:
            print('   [%s] %s' % ('ok' if v else 'FAIL', lab))
            ok = ok and v
        pub = '\n'.join(_fn_body(dis, r'AmclNode::publishParticleCloud'))
        copy_ok = bool(re.search(r'movsd\s+-0x8\(%r\w+\),%xmm0', pub)
                       and re.search(r'movsd\s+%xmm0,-0x8\(%r\w+\)', pub))
        print('   [%s] weight copied verbatim into the message '
              '(offset -0x8 = pf_sample_t::weight)'
              % ('ok' if copy_ok else 'FAIL'))
        ok = ok and copy_ok
        print()
        print('   CONSEQUENCE. The publish is guarded on force_update_ and')
        print('   NOT on whether the resample ran, so at interval 2 the')
        print('   updates where the modulo is non-zero publish the set')
        print('   pf_update_sensor just weighted.')

    print()
    print('B. THE DEPLOYED FILTER, EXECUTED (%s)' % PF_LIB)
    o = _oracle()
    if o is None:
        print('   UNAVAILABLE -- see the message above. The gate rests on')
        print('   A alone, and the live alternation test in `phase`.')
        ok = False
    else:
        pre, post, cs = o['pre'], o['post'], o['current_set']
        for lab, v in (
                ('pf_update_sensor leaves weights NON-FLAT '
                 '(%d distinct of %d, ESS/n %.4f)'
                 % (pre['nuniq'], pre['n'], pre['ess_ratio']),
                 pre['nuniq'] > 1 and pre['ess_ratio'] < 0.999),
                ('  ... and normalised to sum 1 (sum %.12f)' % pre['wsum'],
                 abs(pre['wsum'] - 1.0) < 1e-9),
                # The EXACT claim is `flat_exact`, a bit-level all-equal
                # test on the weight vector. ESS/n is reported alongside
                # it to a tolerance, because summing n identical doubles
                # and dividing is not associative: the flat vector gives
                # 1.000000000000001, and asserting == 1.0 there would be
                # asserting a property of floating-point addition rather
                # than of the filter.
                ('pf_update_resample leaves them EXACTLY flat '
                 '(%d distinct value, bit-identical)' % post['nuniq'],
                 post['nuniq'] == 1 and post['flat_exact']),
                ('  ... ESS/n = %.15f, within 1e-12 of 1'
                 % post['ess_ratio'],
                 abs(post['ess_ratio'] - 1.0) < 1e-12),
                ('  ... at exactly 1/n', post['flat_exact']),
                ('resample flips current_set (%s -> %s)'
                 % (cs[0], cs[1]), cs[0] != cs[1])):
            print('   [%s] %s' % ('ok' if v else 'FAIL', lab))
            ok = ok and v
        print()
        print("   The post-resample line IS C2-NAV.30's measurement:")
        print('   ESS/n = 1.0000 exactly, 40 of 40 samples, both legs.')

    print()
    print('C. THE LIVE FALSIFIER, which no static check can replace')
    print('   At interval 2 the published clouds must ALTERNATE pre/post.')
    print('   If they do not -- if every cloud is exactly flat -- the')
    print('   parameter did not take effect, the weights stay hidden, and')
    print('   the verdict is (C) UNOBSERVABLE. `phase` tests exactly that')
    print('   and reports a uniformly-flat run as a FAILED EXPERIMENT,')
    print('   never as evidence that the weights are unbiased.')

    print()
    print('GATE: %s' % ('PASSED -- the experiment can observe '
                        'pre-resampling weights' if ok else
                        'NOT CLOSED -- do not interpret weights as '
                        'pre-resampling'))
    return 0 if ok else 1


_ORACLE_CACHE = {}


def _oracle(n=500, seed=7, mu_y=-0.09, sigma_y=0.20):
    """Build and run c2nav33_oracle against the installed libpf_lib.so.

    Returns None (with a printed reason) if it cannot be built or run.
    Nothing downstream is allowed to invent the numbers it would have
    produced.
    """
    key = (n, seed, mu_y, sigma_y)
    if key in _ORACLE_CACHE:
        return _ORACLE_CACHE[key]
    why = None
    if not os.path.exists(ORACLE_SRC):
        why = 'c2nav33_oracle.cpp missing'
    elif not os.path.exists(PF_LIB):
        why = 'libpf_lib.so missing'
    elif shutil.which('g++') is None:
        why = 'g++ not on PATH'
    if why:
        print('   oracle unavailable: %s' % why)
        _ORACLE_CACHE[key] = None
        return None
    d = tempfile.mkdtemp(prefix='c2nav33_')
    try:
        exe = os.path.join(d, 'oracle')
        cmd = ['g++', '-O2', '-o', exe, ORACLE_SRC, '-I' + ROS_INC,
               '-L' + ROS_LIB, '-lpf_lib', '-Wl,-rpath,' + ROS_LIB]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print('   oracle unavailable: g++ exit %d: %s'
                  % (r.returncode, r.stderr.strip()[:200]))
            _ORACLE_CACHE[key] = None
            return None
        r = subprocess.run([exe, str(n), str(seed), str(mu_y), str(sigma_y)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print('   oracle unavailable: exit %d: %s'
                  % (r.returncode, r.stderr.strip()[:200]))
            _ORACLE_CACHE[key] = None
            return None
        pre, post, cs = [], [], [None, None]
        for line in r.stdout.splitlines():
            p = line.split()
            if not p:
                continue
            if p[0] == 'sensor':
                cs[0] = int(p[1])
            elif p[0] == 'resample':
                cs[1] = int(p[1])
            elif p[0] == 's':
                pre.append(float(p[5]))
            elif p[0] == 'r':
                post.append(float(p[5]))
        out = {'current_set': cs,
               'pre': _wstats(np.array(pre)),
               'post': _wstats(np.array(post))}
        _ORACLE_CACHE[key] = out
        return out
    finally:
        shutil.rmtree(d, ignore_errors=True)


def _wstats(w):
    """Weight-vector statistics. `flat_exact` is a BIT-level test."""
    w = np.asarray(w, dtype=np.float64)
    n = int(len(w))
    s = float(w.sum())
    d = {'n': n, 'wsum': s, 'nuniq': int(len(set(w.tolist())))}
    if n > 0 and s > 0:
        wn = w / s
        d['ess'] = float(1.0 / float((wn * wn).sum()))
        d['ess_ratio'] = d['ess'] / n
        d['wmax'] = float(wn.max())
        nz = wn[wn > 0]
        d['entropy'] = (float(-(nz * np.log(nz)).sum() / math.log(n))
                        if n > 1 else None)
    else:
        d['ess'] = d['ess_ratio'] = d['wmax'] = d['entropy'] = None
    d['flat_exact'] = bool(n > 0 and bool((w == w[0]).all()))
    return d


# ======================================================== 2. paramdiff
def _flatten(node, prefix=''):
    """Every leaf of a YAML document as dotted-path -> value.

    C2-NAV.25's function, reused verbatim in behaviour: lists are LEAVES,
    because a Nav2 parameter list is one parameter to the node that loads
    it and exploding it would report a reordering as many changes.
    """
    out = {}
    if isinstance(node, dict):
        for k, v in node.items():
            out.update(_flatten(v, f'{prefix}.{k}' if prefix else str(k)))
    else:
        out[prefix] = node
    return out


def _sha(path):
    import hashlib
    if not os.path.exists(path):
        return None
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


# The AMCL leaves the brief names explicitly. Every one is asserted
# IDENTICAL between the two files, by name, so a silent edit to any of
# them fails loudly instead of riding along with the experiment.
GUARDED = (
    'alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5',
    'update_min_d', 'update_min_a',
    'laser_model_type', 'laser_likelihood_max_dist', 'laser_max_range',
    'laser_min_range', 'max_beams', 'sigma_hit',
    'z_hit', 'z_rand', 'z_max', 'z_short', 'lambda_short',
    'do_beamskip', 'beam_skip_distance', 'beam_skip_error_threshold',
    'beam_skip_threshold',
    'min_particles', 'max_particles', 'pf_err', 'pf_z',
    'recovery_alpha_slow', 'recovery_alpha_fast',
    'robot_model_type', 'transform_tolerance', 'save_pose_rate',
    'tf_broadcast', 'set_initial_pose', 'always_reset_initial_pose',
    'first_map_only', 'base_frame_id', 'odom_frame_id', 'global_frame_id',
)


def cmd_paramdiff(args):
    """Exactly one behavioural leaf changes. Nothing else."""
    import yaml
    hdr('C2-NAV.33 paramdiff -- one leaf, and the guarded list')
    for lab, p in (('baseline ', BASELINE_YAML),
                   ('candidate', CANDIDATE_YAML)):
        print('%s  %-42s sha256 %s'
              % (lab, os.path.relpath(p, WT), (_sha(p) or 'MISSING')[:32]))
    if not (os.path.exists(BASELINE_YAML) and os.path.exists(CANDIDATE_YAML)):
        print('MISSING parameter file -- cannot verify the diff.')
        return 1
    with open(BASELINE_YAML) as f:
        base = _flatten(yaml.safe_load(f))
    with open(CANDIDATE_YAML) as f:
        cand = _flatten(yaml.safe_load(f))

    print()
    print('leaves: baseline %d   candidate %d' % (len(base), len(cand)))
    added = sorted(set(cand) - set(base))
    removed = sorted(set(base) - set(cand))
    changed = sorted(k for k in set(base) & set(cand) if base[k] != cand[k])
    print('added %d   removed %d   changed %d'
          % (len(added), len(removed), len(changed)))
    for k in added:
        print('   ADDED    %s = %r' % (k, cand[k]))
    for k in removed:
        print('   REMOVED  %s = %r' % (k, base[k]))
    for k in changed:
        print('   CHANGED  %s : %r -> %r' % (k, base[k], cand[k]))

    ok = (not added and not removed and changed == [ONE_LEAF]
          and base.get(ONE_LEAF) == 1 and cand.get(ONE_LEAF) == 2)
    print()
    print('[%s] exactly one leaf, and it is %s : 1 -> 2'
          % ('ok' if ok else 'FAIL', ONE_LEAF))

    print()
    print('the guarded AMCL leaves, asserted identical by name:')
    bad = []
    for name in GUARDED:
        k = 'amcl.ros__parameters.' + name
        b, c = base.get(k, '<absent>'), cand.get(k, '<absent>')
        if b != c or b == '<absent>':
            bad.append((name, b, c))
    if bad:
        for name, b, c in bad:
            print('   FAIL %-28s %r -> %r' % (name, b, c))
    else:
        print('   [ok] all %d identical and present' % len(GUARDED))
    ok = ok and not bad

    print()
    print('and the NON-AMCL leaves, which this session may not touch:')
    nonamcl = [k for k in changed if not k.startswith('amcl.')]
    print('   [%s] %d navigation leaves changed'
          % ('ok' if not nonamcl else 'FAIL', len(nonamcl)))
    ok = ok and not nonamcl
    print()
    print('PARAMDIFF: %s' % ('ONE LEAF' if ok else 'NOT ONE LEAF'))
    return 0 if ok else 1


# ==================================================== 3. cloud records
_BUNDLE_CACHE = {}


def _bundle():
    """The frozen derived records, or {} if the bundle is absent."""
    if 'b' not in _BUNDLE_CACHE:
        try:
            with open(BUNDLE) as f:
                _BUNDLE_CACHE['b'] = json.load(f)
        except Exception:                       # noqa: BLE001
            _BUNDLE_CACHE['b'] = {}
    return _BUNDLE_CACHE['b']


def wsnaps(tag, leg, rep=0, measured_frame=False, max_dt=0.5):
    """One record per /particle_cloud MESSAGE in the leg window.

    Not per trace row. C2-NAV.30's `samples()` walks the 10 Hz trace and
    keeps rows whose 0.1 s bucket caught a cloud, which is the right unit
    for a trace-shaped question. The unit HERE is the filter update, and
    the raw particles are the measurement, so the clouds lead and the
    trace is joined onto them by nearest timestamp.

    Missing data is REPRESENTED, never filled: a snapshot with no trace
    row inside `max_dt` keeps `gt_ok` False and every spatial field None,
    and the tables count those rows rather than dropping them silently.
    """
    ox = C.MEASURED_TO_MAP_X if measured_frame else C.WORLD_TO_MAP_X
    oy = C.MEASURED_TO_MAP_Y if measured_frame else C.WORLD_TO_MAP_Y
    # `.navbench` is SCRATCH and is not committed. When the raw particles
    # are not on disk, the DERIVED per-snapshot records frozen by `dump`
    # stand in, so every headline reproduces from the repository alone.
    # They are the same records the live path builds -- `dump` writes
    # exactly what these functions read -- so this is a fallback, not a
    # second implementation. Commands that genuinely need raw particles
    # (c2nav33_extra.py sections 2 and 4) say so instead of printing
    # something that looks like a measurement.
    if not C.read_clouds(tag, leg, rep):
        key = '%s/%s/%s' % (tag, leg, 'measured' if measured_frame
                            else 'hist')
        return list(_bundle().get('snaps', {}).get(key, []))
    meta = C.cloud_meta(tag, leg, rep) or {}
    t0 = meta.get('t0_sim_s')
    rows = []
    for r in C.read_trace(tag, leg, rep):
        t = C.fl(r.get('t_rel'))
        if t is not None:
            rows.append((t, r))
    blank = ('gt_x', 'gt_y', 'gt_yaw', 'amcl_x', 'amcl_y', 'amcl_dy',
             'scan_min', 'v_act', 'w_act', 'v_nav', 'w_nav', 'dy_u',
             'dy_w', 'shift', 'south_w', 'north_w', 'south_u', 'north_u',
             'mass_shift', 'nearest', 'lr_near', 'n_near_gt',
             'n_near_amcl')
    out = []
    for k, (ts, arr) in enumerate(C.read_clouds(tag, leg, rep)):
        arr = np.asarray(arr, dtype=np.float64)
        trel = (ts - t0) if t0 is not None else ts
        w = arr[:, 3] if len(arr) else np.zeros(0)
        d = {'k': k, 'ts_sim': float(ts), 't_rel': float(trel),
             'n': int(len(arr))}
        d.update(_wstats(w))
        # phase, from the weights themselves and nothing else.
        d['phase'] = (None if d['n'] == 0
                      else ('post' if d['flat_exact'] else 'pre'))
        d['ux'] = float(arr[:, 0].mean()) if len(arr) else None
        d['uy'] = float(arr[:, 1].mean()) if len(arr) else None
        wn = None
        if d['n'] and d['wsum'] > 0:
            wn = w / d['wsum']
            d['wx'] = float((wn * arr[:, 0]).sum())
            d['wy'] = float((wn * arr[:, 1]).sum())
        else:
            d['wx'] = d['wy'] = None
        d['ylo'] = float(arr[:, 1].min()) if len(arr) else None
        d['yhi'] = float(arr[:, 1].max()) if len(arr) else None
        # the trace join
        near = min(rows, key=lambda rr: abs(rr[0] - trel)) if rows else None
        d['dt_trace'] = abs(near[0] - trel) if near else None
        d['cm_polygon'] = ''
        if near is None or d['dt_trace'] > max_dt:
            d['gt_ok'] = False
            for key in blank:
                d[key] = None
            out.append(d)
            continue
        r = near[1]
        gx, gy = C.fl(r.get('x')), C.fl(r.get('y'))
        d['gt_ok'] = gx is not None and gy is not None and d['n'] > 0
        d['cm_polygon'] = r.get('cm_polygon') or ''
        for key, col in (('gt_yaw', 'yaw'), ('scan_min', 'scan_min'),
                         ('v_act', 'v_act'), ('w_act', 'w_act'),
                         ('v_nav', 'v_nav'), ('w_nav', 'w_nav')):
            d[key] = C.fl(r.get(col))
        ax, ay = C.fl(r.get('amcl_x')), C.fl(r.get('amcl_y'))
        d['amcl_x'], d['amcl_y'] = ax, ay
        if not d['gt_ok']:
            for key in ('gt_x', 'gt_y', 'amcl_dy', 'dy_u', 'dy_w', 'shift',
                        'south_w', 'north_w', 'south_u', 'north_u',
                        'mass_shift', 'nearest', 'lr_near', 'n_near_gt',
                        'n_near_amcl'):
                d[key] = None
            out.append(d)
            continue
        d['gt_x'], d['gt_y'] = gx + ox, gy + oy
        d['amcl_dy'] = (ay - d['gt_y']) if ay is not None else None
        # ---- THE CENTRAL SPATIAL TEST -------------------------------
        y_rel = arr[:, 1] - d['gt_y']
        d['dy_u'] = d['uy'] - d['gt_y']
        d['dy_w'] = (d['wy'] - d['gt_y']) if d['wy'] is not None else None
        # The one number the question turns on: what the WEIGHTS do to
        # the centroid, with the cloud's own existing displacement --
        # the thing C2-NAV.30 measured, which this session is not
        # re-deriving -- divided out.
        d['shift'] = ((d['dy_w'] - d['dy_u'])
                      if d['dy_w'] is not None else None)
        if wn is not None:
            d['south_w'] = float(wn[y_rel < 0].sum())
            d['north_w'] = float(wn[y_rel > 0].sum())
        else:
            d['south_w'] = d['north_w'] = None
        d['south_u'] = float((y_rel < 0).mean())
        d['north_u'] = float((y_rel > 0).mean())
        # Same comparison in mass terms. Positive => weighting ADDS mass
        # to the south side beyond the share of particles already there.
        d['mass_shift'] = ((d['south_w'] - d['south_u'])
                           if d['south_w'] is not None else None)
        dist = np.hypot(arr[:, 0] - d['gt_x'], arr[:, 1] - d['gt_y'])
        d['nearest'] = float(dist.min()) if len(dist) else None
        # weighted likelihood ratio, GT band vs AMCL band
        d['lr_near'] = None
        d['n_near_gt'] = int((dist <= NEAR_R).sum())
        d['n_near_amcl'] = None
        if wn is not None and ax is not None and ay is not None:
            da = np.hypot(arr[:, 0] - ax, arr[:, 1] - ay)
            d['n_near_amcl'] = int((da <= NEAR_R).sum())
            if d['n_near_gt'] > 0 and d['n_near_amcl'] > 0:
                mg = float(wn[dist <= NEAR_R].mean())
                ma = float(wn[da <= NEAR_R].mean())
                if mg > 0:
                    d['lr_near'] = ma / mg
        out.append(d)
    return out


# ========================================================== 4. phase
def cmd_phase(args):
    """Did the parameter take effect, and which clouds are pre-resample?

    This is the blindness guard. It must be read BEFORE any weight
    number, because every weight number below is meaningless if the
    answer here is "all flat".
    """
    hdr('C2-NAV.33 phase -- is the resample actually being skipped?')
    print('run %s   (C2-NAV.30 control run: %s)' % (RUN, BASE_RUN))
    print()
    print('phase is read from the WEIGHTS THEMSELVES -- a cloud whose')
    print('weights are bit-identical across every particle is `post`,')
    print('anything else is `pre`. Nothing is inferred from the config.')
    print()
    print('%-16s%6s%6s%6s%9s%11s%12s'
          % ('leg', 'msgs', 'pre', 'post', 'alt runs', 'ESS/n pre',
             'ESS/n post'))
    print('-' * 78)
    any_pre = False
    any_msg = False
    rows = []
    for leg in (CONTROL_LEG, WALL_LEG):
        S = wsnaps(RUN, leg)
        if not S:
            print('%-16s%6s   -- NO RECORD: not on disk and not in the '
                  'bundle --' % (leg, '0'))
            continue
        any_msg = True
        ph = [d['phase'] for d in S]
        npre = sum(1 for p in ph if p == 'pre')
        npost = sum(1 for p in ph if p == 'post')
        runs = 1 + sum(1 for a, b in zip(ph, ph[1:]) if a != b)
        e_pre = med([d['ess_ratio'] for d in S if d['phase'] == 'pre'])
        e_post = med([d['ess_ratio'] for d in S if d['phase'] == 'post'])
        print('%-16s%6d%6d%6d%9d%11s%12s'
              % (leg, len(S), npre, npost, runs,
                 fmt(e_pre, 11, 4, False), fmt(e_post, 12, 8, False)))
        any_pre = any_pre or npre > 0
        rows.append((leg, ph))
    print()
    print('the phase sequence, message by message (p = pre, R = post):')
    for leg, ph in rows:
        s = ''.join('p' if x == 'pre' else ('R' if x == 'post' else '?')
                    for x in ph)
        print('   %-16s %s' % (leg, s))
    print()
    print('perfect alternation would be pRpRpR...; the modulo counter is')
    print('a MEMBER that persists across legs and is reset only when the')
    print('filter (re)initialises, so the two legs continue one sequence.')

    print()
    if not any_msg:
        print('RESULT: NO RECORD AT ALL. Not a finding -- the artefacts')
        print('are simply absent. This is NOT evidence that the weights')
        print('are flat, and it is NOT classification (C).')
    elif not any_pre:
        print('RESULT: NO pre-resample cloud was observed.')
        print('The parameter did not produce the intended diagnostic')
        print('effect in this run. Every weight table below would be a')
        print('table of flat weights. Classification is (C) WEIGHT')
        print('INFORMATION REMAINS UNOBSERVABLE, and no statement about')
        print('what the weights prefer may be made from this run.')
    else:
        print('RESULT: pre-resampling weights ARE present in the record.')
        print('The captured `pre` clouds correspond to updates on which')
        print('pf_update_resample was skipped, which is what `gate` A')
        print('showed leaves pf_->current_set indexing the set')
        print('pf_update_sensor weighted.')
    return 0


# ========================================================= 5. weights
def cmd_weights(args):
    """WHERE DOES THE WEIGHT LIVE?"""
    hdr('C2-NAV.33 weights -- where does the weight live?')
    print('Signed lateral coordinate y_rel = particle_y - GT_y.')
    print('NEGATIVE = SOUTH = the wall side = the direction C2-NAV.28')
    print('measured the bias in. Frame convention: historical')
    print('world->map (%.4f, %.4f), as every earlier session used; the'
          % (C.WORLD_TO_MAP_X, C.WORLD_TO_MAP_Y))
    print('measured convention (%.4f, %.4f) is carried in `compare`.'
          % (C.MEASURED_TO_MAP_X, C.MEASURED_TO_MAP_Y))
    print()
    print('THE DISCRIMINATOR IS NOT south_w > north_w. The cloud is')
    print('ALREADY displaced south -- C2-NAV.30 measured 36.4 % of')
    print('particles north of GT -- so a spatially NEUTRAL weighting')
    print('already gives south_w ~ 0.64. The question is whether the')
    print('weights move the centroid FURTHER south than the particles')
    print('alone already are:')
    print()
    print('    shift      = (weighted dy) - (unweighted dy)')
    print('    mass_shift = south_w - south_u')
    print()
    print('NEGATIVE shift / POSITIVE mass_shift => weights favour south.')

    for leg in (CONTROL_LEG, WALL_LEG):
        S = wsnaps(RUN, leg)
        if not S:
            print()
            print('%s: no cloud snapshots on disk.' % leg)
            continue
        P = [d for d in S if d['phase'] == 'pre' and d.get('gt_ok')]
        Q = [d for d in S if d['phase'] == 'post' and d.get('gt_ok')]
        print()
        print('--- %s ---  %d messages, %d pre-resample with GT, '
              '%d post-resample with GT' % (leg, len(S), len(P), len(Q)))
        miss = sum(1 for d in S if not d.get('gt_ok'))
        if miss:
            print('    %d message(s) had no trace row within 0.5 s and are '
                  'EXCLUDED, not filled.' % miss)
        if not P:
            print('    NO pre-resample cloud with ground truth. Nothing '
                  'here can speak to the weights.')
            continue
        print()
        print('    %-22s%10s%10s%10s%10s'
              % ('quantity (pre-resample)', 'median', 'mean', 'p05', 'p95'))
        print('    ' + '-' * 62)
        for lab, key in (('unweighted dy   [m]', 'dy_u'),
                         ('WEIGHTED  dy    [m]', 'dy_w'),
                         ('shift  w - u    [m]', 'shift'),
                         ('south weight    [-]', 'south_w'),
                         ('south particles [-]', 'south_u'),
                         ('mass_shift      [-]', 'mass_shift'),
                         ('ESS / n         [-]', 'ess_ratio'),
                         ('max norm weight [-]', 'wmax'),
                         ('weight entropy  [-]', 'entropy'),
                         ('nearest to GT   [m]', 'nearest'),
                         ('AMCL dy         [m]', 'amcl_dy'),
                         ('w(AMCL)/w(GT)   [-]', 'lr_near')):
            v = [d.get(key) for d in P]
            print('    %-22s%10s%10s%10s%10s'
                  % (lab, fmt(med(v), 10, 4), fmt(mean(v), 10, 4),
                     fmt(pct(v, 5), 10, 4), fmt(pct(v, 95), 10, 4)))
        lo, hi = ci95([d['shift'] for d in P])
        nn = len([d for d in P if d['shift'] is not None])
        print()
        print('    shift, 95%% CI of the mean: [%s, %s] m over n=%d'
              % (fmt(lo, 8, 5), fmt(hi, 8, 5), nn))
        lo2, hi2 = ci95([d['mass_shift'] for d in P])
        print('    mass_shift, 95%% CI:        [%s, %s]'
              % (fmt(lo2, 8, 5), fmt(hi2, 8, 5)))
        nneg = sum(1 for d in P if d['shift'] is not None and d['shift'] < 0)
        print('    shift is southward (negative) in %d of %d snapshots'
              % (nneg, nn))

        if Q:
            print()
            print('    NULL CONTROL, the post-resample clouds of the SAME')
            print('    leg. Their weights are flat by construction, so the')
            print('    arithmetic above MUST return exactly zero on them.')
            sh = [d['shift'] for d in Q if d['shift'] is not None]
            ms = [d['mass_shift'] for d in Q if d['mass_shift'] is not None]
            print('    shift      max |.| = %s over n=%d'
                  % (('%.3e' % max(abs(x) for x in sh)) if sh else '-',
                     len(sh)))
            print('    mass_shift max |.| = %s over n=%d'
                  % (('%.3e' % max(abs(x) for x in ms)) if ms else '-',
                     len(ms)))
    print()
    print('Read `phase` first. If it found no pre-resample cloud, every')
    print('number above is a number about flat weights.')
    return 0


# ======================================================== 6. temporal
def cmd_temporal(args):
    """A-F: when does any southward weight concentration appear?"""
    hdr('C2-NAV.33 temporal -- when does the weighting appear?')
    print("The brief's options:")
    print('  A before entering wall_adjacent   B on entry')
    print('  C after one or more updates       D only after turning')
    print('  E after a resampling event        F stable while stationary')
    print()
    for leg in (CONTROL_LEG, WALL_LEG):
        S = [d for d in wsnaps(RUN, leg)
             if d['phase'] == 'pre' and d.get('gt_ok')]
        if not S:
            print('%s: no pre-resample cloud with GT.' % leg)
            continue
        print('--- %s ---' % leg)
        print('%6s%8s%9s%9s%10s%9s%9s%8s%9s'
              % ('k', 't_rel', 'dy_u', 'dy_w', 'shift', 'south_w',
                 'ESS/n', '|w_act|', 'scan_min'))
        print('-' * 78)
        for d in S:
            wa = abs(d['w_act']) if d['w_act'] is not None else None
            print('%6d%8.2f%9s%9s%10s%9s%9s%8s%9s'
                  % (d['k'], d['t_rel'], fmt(d['dy_u'], 9, 4),
                     fmt(d['dy_w'], 9, 4), fmt(d['shift'], 10, 5),
                     fmt(d['south_w'], 9, 4),
                     fmt(d['ess_ratio'], 9, 4, False),
                     fmt(wa, 8, 3, False), fmt(d['scan_min'], 9, 3, False)))
        print()
        first = S[0]
        print('   first pre-resample cloud of the leg: shift %s, '
              'south_w %s' % (fmt(first['shift'], 8, 5),
                              fmt(first['south_w'], 7, 4)))
        movin = [d['shift'] for d in S
                 if d['shift'] is not None and _moving(d)]
        still = [d['shift'] for d in S
                 if d['shift'] is not None and _moving(d) is False]
        print('   shift while MOVING  median %s over n=%d'
              % (fmt(med(movin), 8, 5), len(movin)))
        print('   shift while STOPPED median %s over n=%d'
              % (fmt(med(still), 8, 5), len(still)))
        print()
    return 0


def _moving(d):
    """True/False if the command record supports a call, None if not."""
    v, w = d.get('v_act'), d.get('w_act')
    if v is None and w is None:
        return None
    return (abs(w or 0.0) > 0.05) or (abs(v or 0.0) > 0.02)


# ========================================================= 7. compare
def cmd_compare(args):
    """C2-NAV.30 (interval 1) against C2-NAV.33 (interval 2)."""
    hdr('C2-NAV.33 compare -- interval 1 (C2-NAV.30) vs interval 2')
    print('%-12s%-14s%6s%6s%11s%9s%9s%8s'
          % ('run', 'leg', 'msgs', 'pre', 'ESS/n', 'unw dy', 'wtd dy',
             'frac N'))
    print('-' * 78)
    for tag, label in ((BASE_RUN, 'C2-NAV.30'), (RUN, 'C2-NAV.33')):
        for leg in (CONTROL_LEG, WALL_LEG):
            S = wsnaps(tag, leg)
            if not S:
                print('%-12s%-14s   -- not on disk --' % (label, leg))
                continue
            npre = sum(1 for d in S if d['phase'] == 'pre')
            G = [d for d in S if d.get('gt_ok')]
            print('%-12s%-14s%6d%6d%11s%9s%9s%8s'
                  % (label, leg, len(S), npre,
                     fmt(med([d['ess_ratio'] for d in S]), 11, 8, False),
                     fmt(med([d['dy_u'] for d in G]), 9, 4),
                     fmt(med([d['dy_w'] for d in G]), 9, 4),
                     fmt(med([d['north_u'] for d in G]), 8, 4, False)))
    print()
    print('The unweighted columns are the ones that must AGREE across the')
    print('two runs: they describe cloud geometry, which the parameter is')
    print('not supposed to change materially. The `pre` column is the')
    print('whole point -- C2-NAV.30 had none, by construction.')

    print()
    print('BOTH FRAME CONVENTIONS, as C2-NAV.29 and .30 both carried, and')
    print('neither constant changed here:')
    print()
    print('%-16s%-12s%9s%9s%10s%9s'
          % ('leg', 'convention', 'unw dy', 'wtd dy', 'shift', 'mass_sh'))
    print('-' * 78)
    for leg in (CONTROL_LEG, WALL_LEG):
        for label, meas in (('historical', False), ('measured', True)):
            S = [d for d in wsnaps(RUN, leg, measured_frame=meas)
                 if d['phase'] == 'pre' and d.get('gt_ok')]
            if not S:
                print('%-16s%-12s   -- none --' % (leg, label))
                continue
            print('%-16s%-12s%9s%9s%10s%9s'
                  % (leg, label,
                     fmt(med([d['dy_u'] for d in S]), 9, 4),
                     fmt(med([d['dy_w'] for d in S]), 9, 4),
                     fmt(med([d['shift'] for d in S]), 10, 5),
                     fmt(med([d['mass_shift'] for d in S]), 9, 4)))
    return 0


# ========================================================= 8. context
def cmd_context(args):
    """Accuracy and safety. CONTEXT ONLY -- not a tuning result."""
    hdr('C2-NAV.33 context -- accuracy and safety (CONTEXT, NOT a result)')
    print('One diagnostic run. These numbers do NOT make interval 2 a')
    print('candidate configuration and are not compared as a tuning arm.')
    print()
    legs = C.legs_of(RUN)
    if not legs:
        print('no results JSON for %s on disk.' % RUN)
        return 0
    print('%-16s%12s%10s%12s%10s'
          % ('leg', 'status', 'dur [s]', 'clearance', 'path [m]'))
    print('-' * 78)
    for name, L in legs.items():
        print('%-16s%12s%10s%12s%10s'
              % (name, str(L.get('status'))[:12],
                 fmt(L.get('duration_sim_s'), 10, 1, False),
                 fmt(L.get('min_clearance_m'), 12, 3, False),
                 fmt(L.get('path_len_m'), 10, 2, False)))
    print()
    for leg in (CONTROL_LEG, WALL_LEG):
        S = wsnaps(RUN, leg)
        G = [d for d in S if d.get('gt_ok') and d['amcl_dy'] is not None]
        pol = sorted({d['cm_polygon'] for d in S if d.get('cm_polygon')})
        print('%-16s final AMCL |dy| %s   collision-monitor polygons seen: %s'
              % (leg,
                 fmt(abs(G[-1]['amcl_dy']) if G else None, 8, 4, False),
                 ', '.join(pol) if pol else '(none)'))
    return 0


# ========================================================= 9. verdict
def cmd_verdict(args):
    """Exactly one of A / B / C."""
    hdr('C2-NAV.33 verdict')
    S = wsnaps(RUN, WALL_LEG)
    P = [d for d in S if d['phase'] == 'pre' and d.get('gt_ok')]
    print('wall_adjacent: %d cloud messages, %d pre-resample with GT'
          % (len(S), len(P)))
    print()
    if not S:
        # NOT a verdict. An absent artefact and an observed flat cloud
        # are different claims, and reporting (C) here would be the
        # exact false negative CLAUDE.md forbids: a "we saw nothing"
        # finding from an instrument that could not see.
        print('NO DATA -- no cloud snapshots for %s, on disk or in the'
              % RUN)
        print('frozen bundle. This is NOT classification (C); it is the')
        print('absence of a record. Re-run c2nav33_matrix.sh, or run')
        print('`dump` against a run that exists.')
        return 2
    if not P:
        print('(C) WEIGHT INFORMATION REMAINS UNOBSERVABLE')
        print('    Every published cloud carried bit-identical weights.')
        print('    resample_interval: 2 did not expose the pre-resampling')
        print('    state in this run. No claim about what the weights')
        print('    prefer may be drawn from it.')
        print()
        print('    Minimum instrumentation that WOULD observe them:')
        print('    a source build of nav2_amcl with a publisher inserted')
        print('    between updateFilter() and the resample branch in')
        print('    AmclNode::laserReceived -- the binary shows those two')
        print('    are adjacent, so the insertion point is one statement')
        print('    and touches no filter mathematics.')
        return 0
    sh = [d['shift'] for d in P if d['shift'] is not None]
    ms = [d['mass_shift'] for d in P if d['mass_shift'] is not None]
    lo, hi = ci95(sh)
    m = mean(sh)
    print('shift (weighted dy - unweighted dy), pre-resample clouds:')
    print('    median %s   mean %s   95%% CI [%s, %s]   n=%d'
          % (fmt(med(sh), 8, 5), fmt(m, 8, 5), fmt(lo, 8, 5),
             fmt(hi, 8, 5), len(sh)))
    print('    southward (negative) in %d of %d'
          % (sum(1 for x in sh if x < 0), len(sh)))
    print('mass_shift (south_w - south_u): median %s' % fmt(med(ms), 8, 5))
    print()
    print('against the standing observation, C2-NAV.28/.30:')
    print('    AMCL/cloud centroid bias at wall_adjacent ~ -0.09 m')
    print()
    # The classification rule, fixed here so it is not chosen after the
    # numbers are seen: (A) requires the 95 % CI of the mean shift to lie
    # WHOLLY south of zero. Anything else is (B).
    south = (m is not None and hi is not None and hi < 0)
    if south:
        print('(A) PRE-RESAMPLING WEIGHTS CLEARLY FAVOUR SOUTHWARD')
        print('    PARTICLES. The 95 % CI of the mean shift lies wholly')
        print('    below zero.')
    else:
        print('(B) WEIGHTS DO NOT FAVOUR SOUTHWARD PARTICLES.')
        print('    The mean shift does not lie wholly south of zero, so')
        print('    the observation weighting does not explain the')
        print('    persistent southward centroid bias.')
    return 0


# ========================================================== 10. dump
def cmd_dump(args):
    """Freeze the DERIVED per-snapshot statistics so the headline
    numbers survive .navbench, which is scratch and is not committed.

    The raw particle sets are far too large to commit; what is frozen is
    exactly what the tables above read, so every number reproduces with
    C2NAV_SCRATCH pointed at a nonexistent path -- and the commands that
    genuinely need raw particles say so instead of printing something
    that looks like a measurement.
    """
    out = {'run': RUN, 'base_run': BASE_RUN,
           'one_leaf': ONE_LEAF,
           'baseline_yaml_sha256': _sha(BASELINE_YAML),
           'candidate_yaml_sha256': _sha(CANDIDATE_YAML),
           'near_r_m': NEAR_R, 'snaps': {}}
    o = _oracle()
    if o:
        out['oracle'] = o
    for tag in (RUN, BASE_RUN):
        for leg in (CONTROL_LEG, WALL_LEG):
            for meas in (False, True):
                S = wsnaps(tag, leg, measured_frame=meas)
                if not S:
                    continue
                key = '%s/%s/%s' % (tag, leg, 'measured' if meas else 'hist')
                out['snaps'][key] = S
    with open(BUNDLE, 'w') as f:
        json.dump(out, f, separators=(',', ':'), sort_keys=True)
    print('wrote %s  (%d snapshot groups, %d bytes)'
          % (os.path.relpath(BUNDLE, WT), len(out['snaps']),
             os.path.getsize(BUNDLE)))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description='C2-NAV.33 pre-resampling '
                                             'AMCL weight diagnosis')
    sub = ap.add_subparsers(dest='cmd')
    for name, fn in (('gate', cmd_gate), ('paramdiff', cmd_paramdiff),
                     ('phase', cmd_phase), ('weights', cmd_weights),
                     ('temporal', cmd_temporal), ('compare', cmd_compare),
                     ('context', cmd_context), ('verdict', cmd_verdict),
                     ('dump', cmd_dump)):
        sub.add_parser(name).set_defaults(fn=fn)

    def _selftest(a):
        import c2nav33_selftest as T
        return T.cmd_selftest(a)
    sub.add_parser('selftest').set_defaults(fn=_selftest)
    a = ap.parse_args(argv)
    if not getattr(a, 'fn', None):
        ap.print_help()
        return 2
    return a.fn(a)


if __name__ == '__main__':
    sys.exit(main())
