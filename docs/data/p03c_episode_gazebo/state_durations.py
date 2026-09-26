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
Sim seconds spent in each mission state, from the /mission/state stream.

    python3 state_durations.py RUN_DIR [RUN_DIR ...]

``elapsed`` on /mission/state is the time IN THE CURRENT STATE, on the
executive's ROS (sim) clock -- not mission time (the Docker branch paid
for reading it as such). The largest elapsed seen before a state is left
is that visit's duration, to the stream's ~2 Hz resolution.
"""

import os
import re
import sys

LINE = re.compile(r'^state=(\S+) .*?elapsed=([0-9.]+)')


def durations(path):
    """Return [(state, seconds)] per visit, in order."""
    out = []
    for line in open(path):
        match = LINE.match(line)
        if not match:
            continue
        state, elapsed = match.group(1), float(match.group(2))
        if out and out[-1][0] == state and elapsed >= out[-1][1] - 1e-9:
            out[-1] = (state, elapsed)
        else:
            out.append((state, elapsed))
    return out


def main():
    """Print one line per run: state=seconds for every visit."""
    for run in sys.argv[1:]:
        visits = durations(os.path.join(run, 'state_stream.txt'))
        body = ' '.join(f'{s}={t:.0f}' for s, t in visits
                        if s not in ('IDLE', 'COMPLETE', 'ABORT'))
        print(f'{os.path.basename(run.rstrip("/")):14s} {body}')


if __name__ == '__main__':
    main()
