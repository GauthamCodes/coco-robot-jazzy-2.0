#!/usr/bin/env python3
"""Run COCO's per-package pytest suites the way CLAUDE.md prescribes.

cwd = the package directory, one pytest invocation per package, gazebo_models
with --ignore=test_integration, each package on its own ROS_DOMAIN_ID so
parallel packages cannot see each other's graph. Writes one JUnit XML per
package and prints a JSON summary (per package and per test) on stdout.

stdlib only: this has to run on a bare host and in any container.
"""
import argparse
import concurrent.futures as cf
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

PACKAGES = [
    'coco_config', 'custom_teleop', 'coco_rl', 'coco_perception',
    'gazebo_models', 'coco_moveit_config', 'coco_sim', 'coco_mission',
    'coco_web',
]
EXTRA = {'gazebo_models': ['--ignore=test_integration']}


def parse_junit(path):
    out = {'tests': 0, 'passed': 0, 'failed': 0, 'errors': 0, 'skipped': 0,
           'cases': []}
    if not os.path.exists(path):
        return out
    root = ET.parse(path).getroot()
    for tc in root.iter('testcase'):
        name = f"{tc.get('classname', '')}::{tc.get('name', '')}"
        outcome, msg = 'passed', ''
        for child in tc:
            if child.tag == 'failure':
                outcome, msg = 'failed', (child.get('message') or '')[:300]
            elif child.tag == 'error':
                outcome, msg = 'error', (child.get('message') or '')[:300]
            elif child.tag == 'skipped':
                outcome, msg = 'skipped', (child.get('message') or '')[:300]
        out['tests'] += 1
        key = {'passed': 'passed', 'failed': 'failed', 'error': 'errors',
               'skipped': 'skipped'}[outcome]
        out[key] += 1
        out['cases'].append({'id': name, 'outcome': outcome,
                             'time': float(tc.get('time') or 0), 'msg': msg})
    return out


def run_one(repo, pkg, junit_dir, domain, extra_args, timeout):
    pkg_dir = os.path.join(repo, pkg)
    xml = os.path.join(junit_dir, f'{pkg}.xml')
    log = os.path.join(junit_dir, f'{pkg}.log')
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(domain)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    cmd = [sys.executable, '-m', 'pytest', '-p', 'no:cacheprovider',
           '-q', '-rfEs', f'--junitxml={xml}'] + EXTRA.get(pkg, []) + extra_args
    t0 = time.monotonic()
    try:
        with open(log, 'w') as fh:
            rc = subprocess.run(cmd, cwd=pkg_dir, env=env, stdout=fh,
                                stderr=subprocess.STDOUT,
                                timeout=timeout).returncode
    except subprocess.TimeoutExpired:
        rc = 'timeout'
    dt = round(time.monotonic() - t0, 1)
    res = parse_junit(xml)
    res.update({'package': pkg, 'rc': rc, 'seconds': dt, 'domain': domain,
                'cmd': ' '.join(cmd), 'cwd': pkg_dir})
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', required=True)
    ap.add_argument('--junit-dir', required=True)
    ap.add_argument('--packages', nargs='*', default=PACKAGES)
    ap.add_argument('--jobs', type=int, default=1)
    ap.add_argument('--domain-base', type=int, default=40)
    ap.add_argument('--timeout', type=int, default=3600)
    ap.add_argument('pytest_args', nargs='*')
    a = ap.parse_args()
    os.makedirs(a.junit_dir, exist_ok=True)
    t0 = time.time()
    results = []
    with cf.ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = {ex.submit(run_one, a.repo, p, a.junit_dir, a.domain_base + i,
                          a.pytest_args, a.timeout): p
                for i, p in enumerate(a.packages)}
        for f in cf.as_completed(futs):
            r = f.result()
            results.append(r)
            print(f"[tests] {r['package']:<20} rc={r['rc']} "
                  f"{r['passed']}p/{r['failed']}f/{r['errors']}e/"
                  f"{r['skipped']}s in {r['seconds']}s", file=sys.stderr)
    results.sort(key=lambda r: PACKAGES.index(r['package'])
                 if r['package'] in PACKAGES else 99)
    tot = {k: sum(r[k] for r in results)
           for k in ('tests', 'passed', 'failed', 'errors', 'skipped')}
    summary = {'wall_seconds': round(time.time() - t0, 1), 'jobs': a.jobs,
               'totals': tot, 'packages': results}
    with open(os.path.join(a.junit_dir, 'summary.json'), 'w') as fh:
        json.dump(summary, fh, indent=1)
    print(json.dumps({'totals': tot, 'wall_seconds': summary['wall_seconds']}))
    return 0 if tot['failed'] == 0 and tot['errors'] == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
