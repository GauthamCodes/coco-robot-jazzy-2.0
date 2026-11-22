#!/usr/bin/env python3
"""Would C2-NAV.8's tables reproduce in a FRESH CLONE?

The scratch directory `.navbench/` is not committed. Every earlier
experiment in this series folded its per-run JSONs into one committed
artifact and checked the two sources agree -- but that check passes
trivially on THIS machine, where the scratch CSVs still exist and get
picked up by the fallback. It proves nothing about a clone.

So this points the report's scratch fallback at an empty directory and
re-renders every table. What survives is what a fresh clone of the
branch can reproduce from `docs/data` alone.
"""
import io
import os
import sys
import contextlib

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, '..', '..', 'docs', 'data'))
sys.path.insert(0, DATA)
import c2nav8_report as R                                   # noqa: E402

EMPTY = os.path.join(HERE, '_no_scratch')
os.makedirs(EMPTY, exist_ok=True)
R.DEFAULT_DIR = EMPTY                       # the clone has no .navbench

src = os.path.join(DATA, 'c2nav8_bench.json')
failed = []
for mode in ('legs', 'entry', 'exit', 'stop', 'clear', 'compare'):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        R.MODES[mode]([src])
    out = buf.getvalue()
    empty = ('NO CSV FOUND' in out
             or (mode in ('stop', 'clear')
                 and 'enclosure_exit' not in out))
    print(f'{"FAIL" if empty else "ok  "}  {mode}: '
          f'{len(out.splitlines())} lines'
          f'{"  <-- reads nothing without the scratch dir" if empty else ""}')
    if empty:
        failed.append(mode)

# And the numbers themselves, from the committed CSVs only.
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    R.MODES['stop']([src])
tot = [ln for ln in buf.getvalue().splitlines() if 'C2-NAV.8 TOTAL' in ln]
print()
for ln in tot:
    print(' ', ln.strip())
print(f'\n{len(failed)} failed')
sys.exit(1 if failed else 0)
