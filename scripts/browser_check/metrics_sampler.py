# Copyright 2026 Gautham Anil -- Apache-2.0.
"""Poll /api/metrics every 5 s into JSONL: `python3 metrics_sampler.py out`."""

import json
import sys
import time
import urllib.request

with open(sys.argv[1], 'a', buffering=1) as out:
    while True:
        try:
            with urllib.request.urlopen(
                    'http://127.0.0.1:8080/api/metrics', timeout=3) as r:
                out.write(json.dumps({'t': time.time(),
                                      'm': json.loads(r.read())}) + '\n')
        except Exception as exc:   # noqa: BLE001 - platform not up yet
            out.write(json.dumps({'t': time.time(), 'err': repr(exc)}) + '\n')
        time.sleep(5)
