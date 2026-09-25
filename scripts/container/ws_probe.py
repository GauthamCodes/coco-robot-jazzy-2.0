#!/usr/bin/env python3
"""A coco.v1 client, from the host, through the published port.

usage: ws_probe.py URL NEEDLES_FILE [SECONDS]   -> JSON on stdout

Connects to /ws, reads the server's `welcome`, sends `hello` (text-only
client, binary:false) and a `ping`, then reads for SECONDS. Reports the
handshake time, the frame kinds and rates, the pong, any error frames,
whether a binary frame reached a client that declared it cannot take one,
and every occurrence of a live ROS topic name (NEEDLES_FILE: the output of
`ros2 topic list` inside the container) in any frame -- the protocol's
rule is that no topic name ever rides the wire.

Needs tornado (the platform's own dependency).
"""
import asyncio
import json
import re
import sys
import time

from tornado.websocket import websocket_connect


async def probe(url, needles, seconds):
    rx = re.compile('|'.join(re.escape(n) + r'(?![A-Za-z0-9_/])'
                             for n in sorted(needles, key=len, reverse=True)))
    t0 = time.monotonic()
    ws = await websocket_connect(url)
    out = {'url': url, 'handshake_s': round(time.monotonic() - t0, 3),
           'kinds': {}, 'binary_frames': 0, 'errors': [], 'leaks': [],
           'welcome': None, 'pong': None}
    welcome = await asyncio.wait_for(ws.read_message(), 10)
    out['welcome'] = json.loads(welcome)
    ws.write_message(json.dumps({'type': 'hello', 'binary': False,
                                 'client': 'coco-container-probe'}))
    ws.write_message(json.dumps({'type': 'ping', 'id': 1}))
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            msg = await asyncio.wait_for(ws.read_message(),
                                         max(0.1, end - time.monotonic()))
        except asyncio.TimeoutError:
            break
        if msg is None:
            out['closed_early'] = True
            break
        if isinstance(msg, bytes):
            out['binary_frames'] += 1
            continue
        frame = json.loads(msg)
        kind = frame.get('type', '?')
        out['kinds'][kind] = out['kinds'].get(kind, 0) + 1
        if kind == 'pong':
            out['pong'] = frame
        if kind == 'error':
            out['errors'].append(frame)
        for m in rx.finditer(msg):
            out['leaks'].append({'kind': kind, 'needle': m.group(0),
                                 'context': msg[max(0, m.start() - 40):m.end() + 40]})
    ws.close()
    out['seconds'] = seconds
    out['rates_hz'] = {k: round(v / seconds, 2) for k, v in out['kinds'].items()}
    w = out['welcome']
    out['welcome'] = {k: w.get(k) for k in ('type', 'protocol', 'session',
                                            'server', 'commands', 'streams')
                      if k in w}
    out['leaks'] = out['leaks'][:20]
    return out


def main():
    url, needles_file = sys.argv[1], sys.argv[2]
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
    needles = [ln.strip() for ln in open(needles_file)
               if ln.strip().startswith('/') and len(ln.strip()) >= 3]
    out = asyncio.run(probe(url, needles, seconds))
    out['needles_checked'] = len(needles)
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
