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
watch_training.py
=================
Live progress display for a ``train_curriculum.sh`` run.

  ./watch_training.py                  # newest run under ~/coco_rl_runs/
  ./watch_training.py <run_dir>        # a specific run
  ./watch_training.py --once           # print one frame and exit (for logs)
  ./watch_training.py --interval 2     # refresh faster (default 5s)

Read-only: it only ever reads the run directory and /proc, so it can never
disturb the training. Safe to start, stop and restart at any time; quit with
Ctrl-C. It exits by itself once the run writes its DONE marker.

Everything shown is derived from the run's own artifacts — the Monitor CSVs
(episode returns and lengths), STATUS, curriculum.log and eval_*.log — so the
display can never disagree with what the run actually recorded.
"""

import argparse
import csv
import glob
import os
import re
import subprocess
import sys
import time

C = {
    'r': '\033[0m', 'b': '\033[1m', 'dim': '\033[2m',
    'grn': '\033[32m', 'yel': '\033[33m', 'red': '\033[31m',
    'blu': '\033[36m', 'mag': '\033[35m',
}


def paint(s, *codes):
    if not sys.stdout.isatty():
        return s
    return ''.join(C[c] for c in codes) + s + C['r']


def newest_run(root):
    runs = sorted(glob.glob(os.path.join(root, 'curriculum_*')))
    return runs[-1] if runs else None


def read_text(path, default=''):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


def monitor_parts(prefix):
    """Every Monitor CSV for a phase, oldest first.

    A resumed phase has its earlier episodes parked in .part1, .part2, ...
    because SB3's Monitor truncates its file on open. Reading only the live
    CSV would make a resumed run appear to have lost all its progress.
    """
    parts = sorted(glob.glob(prefix + '.monitor.csv.part*'),
                   key=lambda p: int(p.rsplit('part', 1)[-1] or 0))
    live = prefix + '.monitor.csv'
    return parts + ([live] if os.path.exists(live) else [])


def episodes(prefix):
    """(lengths, returns, times) across every Monitor CSV for a phase.

    Each file opens with a '#{"t_start": ...}' JSON comment line, which is why
    the first line is skipped rather than fed to DictReader as a header. Rows
    are read defensively: the trainer appends while we read, so the final line
    can be half-written. Each part's 't' restarts at 0, so the elapsed column
    is rebased onto a single increasing timeline.
    """
    lengths, returns, times = [], [], []
    offset = 0.0
    for path in monitor_parts(prefix):
        last = 0.0
        try:
            with open(path) as f:
                f.readline()
                for row in csv.DictReader(f):
                    try:
                        length = int(row['l'])
                        ret = float(row['r'])
                        t = float(row['t'])
                    except (TypeError, ValueError, KeyError):
                        continue  # torn last line; complete on the next tick
                    lengths.append(length)
                    returns.append(ret)
                    times.append(offset + t)
                    last = max(last, t)
        except OSError:
            continue
        offset += last
    return lengths, returns, times


def bar(frac, width=34, done=False):
    frac = max(0.0, min(1.0, frac))
    filled = int(round(frac * width))
    colour = 'grn' if done else 'blu'
    return (paint('█' * filled, colour) + paint('░' * (width - filled), 'dim')
            + f' {100 * frac:5.1f}%')


def hms(seconds):
    if seconds is None or seconds < 0 or seconds != seconds:  # NaN-safe
        return '  --  '
    seconds = int(seconds)
    if seconds >= 3600:
        return f'{seconds // 3600}h{(seconds % 3600) // 60:02d}m'
    return f'{seconds // 60}m{seconds % 60:02d}s'


def procs_matching(pattern):
    try:
        out = subprocess.run(['pgrep', '-fc', pattern], capture_output=True,
                             text=True, timeout=5)
        return int((out.stdout or '0').strip() or 0)
    except (OSError, ValueError, subprocess.SubprocessError):
        return 0


def on_ac():
    """True if a Mains supply reports online. None if it cannot be read."""
    for ps in glob.glob('/sys/class/power_supply/*'):
        if read_text(os.path.join(ps, 'type')).strip() == 'Mains':
            online = read_text(os.path.join(ps, 'online')).strip()
            if online:
                return online == '1'
    return None


def battery_pct():
    for ps in glob.glob('/sys/class/power_supply/BAT*/capacity'):
        cap = read_text(ps).strip()
        if cap.isdigit():
            return int(cap)
    return None


def inhibitor_held():
    try:
        out = subprocess.run(['systemd-inhibit', '--list'],
                             capture_output=True, text=True, timeout=5)
        return 'coco curriculum' in out.stdout
    except (OSError, subprocess.SubprocessError):
        return False


def disk_free_gb(path):
    try:
        st = os.statvfs(path)
        return st.f_bavail * st.f_frsize / 1e9
    except OSError:
        return None


class Run:
    """Everything the display needs, re-read from disk on every tick."""

    def __init__(self, run_dir):
        self.dir = run_dir
        self.log = read_text(os.path.join(run_dir, 'curriculum.log'))
        self.status = read_text(os.path.join(run_dir, 'STATUS')).strip()
        self.done = os.path.exists(os.path.join(run_dir, 'DONE'))
        self.steps_per_phase = self._int(r'steps/phase\s*:\s*(\d+)')
        self.total_steps = self._int(r'\(total (\d+)\)')
        self.commit = self._str(r'^([0-9a-f]{7,}) ')
        # Stages are "<deg>:<start_progress>". Older runs logged a bare
        # "grades : 12 18 24 deg" line instead, so accept both — otherwise
        # every run directory from before the staged curriculum stops
        # rendering, and the watcher silently shows an empty run rather than
        # saying it could not parse anything.
        self.stages = self._parse_stages()
        self.phases = [self._phase(i, deg, start)
                       for i, (deg, start) in enumerate(self.stages, 1)]

    def _parse_stages(self):
        m = re.search(r'^stages\s*:\s*(.+?)\s*\(', self.log, re.M)
        if m:
            out = []
            for spec in m.group(1).split():
                deg, _, start = spec.partition(':')
                try:
                    out.append((int(deg), start or '0.0'))
                except ValueError:
                    continue
            return out
        m = re.search(r'^grades\s*:\s*([\d ]+?)\s*deg', self.log, re.M)
        if m:
            return [(int(g), None) for g in m.group(1).split()]
        return []

    def _int(self, pat):
        m = re.search(pat, self.log)
        return int(m.group(1)) if m else 0

    def _str(self, pat):
        m = re.search(pat, self.log, re.M)
        return m.group(1) if m else '?'

    def _phase(self, n, deg, start=None):
        # Staged runs name files phase<N>_<deg>deg_s<start>; pre-stage runs used
        # phase<N>_<deg>deg. Probe both so either renders.
        prefix = os.path.join(self.dir, f'phase{n}_{deg}deg')
        if start is not None:
            # '.' is stripped from the on-disk name (SB3 treats it as a file
            # extension and then skips appending .zip); mirror that here.
            staged = f"{prefix}_s{str(start).replace('.', 'p')}"
            if glob.glob(staged + '.monitor.csv*') or glob.glob(staged + '*.zip'):
                prefix = staged
            elif not glob.glob(prefix + '.monitor.csv*'):
                prefix = staged          # not started yet: still the right name
        lens, rets, times = episodes(prefix)
        ev = read_text(os.path.join(self.dir, f'eval_phase{n}.log'))
        rate = re.search(r'success rate: (\d+/\d+ \(\d+%\))', ev)
        # Retries reuse the same CSV, so trust the log for the attempt count.
        tries = len(re.findall(rf'--- phase {n}: retry', self.log))
        started = f'phase {n}/' in self.log or bool(lens)
        return {
            'n': n, 'deg': deg, 'start': start, 'steps': sum(lens), 'eps': len(lens),
            'lens': lens, 'returns': rets, 'times': times, 'retries': tries,
            'rate': rate.group(1) if rate else None,
            'complete': os.path.exists(prefix + '.zip'),
            'started': started,
            'checkpoints': len(glob.glob(prefix + '_*_steps.zip')),
        }

    @property
    def steps_done(self):
        return sum(p['steps'] for p in self.phases)

    def active(self):
        """The phase currently being worked on, per STATUS.

        STATUS is authoritative: picking "first started but incomplete" instead
        reports phase 1 forever once phase 1 has failed, which then made the
        grade cross-check compare the live sim against the wrong phase.
        """
        m = re.match(r'phase (\d+)/', self.status)
        if m:
            n = int(m.group(1))
            for p in self.phases:
                if p['n'] == n:
                    return p
        for p in self.phases:
            if not p['complete'] and p['started']:
                return p
        return None


def throughput(phase, window=25):
    """Recent env-steps/s, from the Monitor CSV's own elapsed-time column.

    A trailing window rather than the whole phase: the average over the run
    would hide a slowdown, and a slowdown is the thing worth noticing.
    """
    lens, times = phase['lens'], phase['times']
    if len(times) < 3:
        return None
    lo = max(0, len(times) - window)
    span = times[-1] - times[lo]
    if span <= 0:
        return None
    return sum(lens[lo + 1:]) / span


def render(run):
    w = 78
    out = []
    add = out.append

    head = f' Coco RL curriculum — {os.path.basename(run.dir)} '
    add(paint('┌' + head.center(w - 2, '─') + '┐', 'b'))

    # ── overall ──────────────────────────────────────────────────────────────
    total = run.total_steps or 1
    add('  ' + paint('overall  ', 'b') + bar(run.steps_done / total,
                                             done=run.done)
        + f'  {run.steps_done:,}/{total:,} steps')
    if not run.phases:
        add('')
        add('  ' + paint('could not parse any stages from curriculum.log — '
                         'the run may just be starting', 'yel'))

    # ── per phase ────────────────────────────────────────────────────────────
    add('')
    for p in run.phases:
        target = run.steps_per_phase or 1
        if p['complete']:
            state, colour = 'done   ', 'grn'
        elif run.done:
            # The run is over, so an incomplete phase is not "running" — it
            # either failed outright or was carried forward as PARTIAL.
            state, colour = ('partial' if p['checkpoints'] else 'failed '), 'red'
        elif p['started']:
            state, colour = 'running', 'yel'
        else:
            state, colour = 'pending', 'dim'
        label = f"{p['deg']:>2}°"
        if p.get('start') not in (None, '0.0', '0'):
            label += f"+{p['start']}m"
        grade = paint(f'{label:<9}', 'b')
        add(f"  {grade} {paint(state, colour)} "
            f"{bar(p['steps'] / target, width=24, done=p['complete'])}"
            f"  {p['steps']:>6,}/{target:,}")
        detail = f"        {p['eps']:>4} episodes"
        if p['returns']:
            recent = p['returns'][-20:]
            detail += (f"   mean {sum(recent) / len(recent):>7.2f}"
                       f"   best {max(p['returns']):>7.2f}")
        if p['checkpoints']:
            detail += f"   {p['checkpoints']} ckpt"
        if p['retries']:
            detail += paint(f"   {p['retries']} retry", 'red')
        if p['rate']:
            detail += paint(f"   eval {p['rate']}", 'mag')
        add(paint(detail, 'dim') if not p['rate'] else detail)

    # ── rate / eta ───────────────────────────────────────────────────────────
    add('')
    cur = run.active()
    rate = throughput(cur) if cur else None
    remaining = max(0, total - run.steps_done)
    eta = remaining / rate if rate else None
    add(f"  throughput {paint(f'{rate:.1f}' if rate else ' -- ', 'b')} steps/s"
        f"   remaining {remaining:,}"
        f"   ETA {paint(hms(eta), 'b')}")

    # ── health ───────────────────────────────────────────────────────────────
    trainer = procs_matching('coco_rl.train_ppo')
    sim = procs_matching('gz_tools_vendor')
    script = procs_matching('train_curriculum.sh')
    ac, pct, inh = on_ac(), battery_pct(), inhibitor_held()
    free = disk_free_gb(run.dir)

    def ok(flag, yes, no):
        return paint(yes, 'grn') if flag else paint(no, 'red')

    add('')
    add('  ' + '  '.join([
        ok(script, 'runner✓', 'runner✗'),
        ok(trainer, 'trainer✓', 'trainer✗'),
        ok(sim, 'sim✓', 'sim✗'),
        ok(inh, 'no-sleep✓', 'no-sleep✗'),
        (paint('AC✓', 'grn') if ac else paint(f'BATTERY {pct}%', 'red')
         if ac is False else paint('AC?', 'yel')),
        paint(f'{free:.1f}GB free', 'grn' if (free or 0) > 2 else 'red')
        if free is not None else '',
    ]))

    # ── correctness cross-check ──────────────────────────────────────────────
    # The grade lives in the sim, not the trainer, so a mismatch between the
    # launched wedge and the grade the trainer is recording is the one silent
    # way a curriculum can be wrong. Compare the two directly.
    if cur and not run.done:
        launched = re.findall(r'sim up at (\d+)°', run.log)
        recorded = re.findall(r'--ramp-angle (\d+)', run.log)
        if launched and recorded:
            lg, rg = int(launched[-1]), int(recorded[-1])
            add('  ' + (paint(f'grade✓ sim {lg}° = trainer {rg}°', 'grn')
                        if lg == rg else
                        paint(f'GRADE MISMATCH sim {lg}° vs trainer {rg}°',
                              'red')))

    # ── status line ──────────────────────────────────────────────────────────
    add('')
    add('  ' + paint(run.status[:w - 4] or 'no status yet', 'dim'))
    if run.done:
        add('  ' + paint('RUN FINISHED — see SUMMARY.md', 'b', 'grn'))
    add(paint('└' + '─' * (w - 2) + '┘', 'b'))
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir', nargs='?', default=None)
    ap.add_argument('--interval', type=float, default=5.0)
    ap.add_argument('--once', action='store_true')
    ap.add_argument('--root', default=os.path.expanduser('~/coco_rl_runs'))
    args = ap.parse_args()

    run_dir = args.run_dir or newest_run(args.root)
    if not run_dir or not os.path.isdir(run_dir):
        sys.exit(f'no run directory found under {args.root} — is a run going?')

    try:
        while True:
            run = Run(run_dir)
            frame = render(run)
            if args.once:
                print(frame)
                return
            # Home + clear-to-end rather than a full clear: no flicker, and
            # scrollback is not destroyed.
            sys.stdout.write('\033[H\033[J' + frame + '\n')
            sys.stdout.flush()
            if run.done:
                print('\n' + read_text(os.path.join(run_dir, 'SUMMARY.md')))
                return
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print('\n(watcher stopped — training is unaffected)')


if __name__ == '__main__':
    main()
