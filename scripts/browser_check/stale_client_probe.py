# Copyright 2026 Gautham Anil -- Apache-2.0.
"""
Measure how long the server keeps a client that went silent.

    python3 stale_client_probe.py http://127.0.0.1:8090 [limit_s]

A raw socket completes the WebSocket handshake and then does nothing: it
never reads and never answers a ping -- a frozen tab, or a laptop lid
closed with the TCP connection still nominally open. The server's
ping/pong (10 s interval, 30 s timeout) must reap it, because the LAST
client leaving is what publishes a stop: a ghost client would keep the
platform believing someone is watching. Prints the client count over time
and the moment it dropped.
"""

import base64
import json
import os
import socket
import sys
import time
import urllib.request

BASE = sys.argv[1].rstrip('/')
LIMIT = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
host, port = BASE.split('//')[1].split(':')


def clients():
    with urllib.request.urlopen(BASE + '/api/session', timeout=3) as r:
        return json.loads(r.read())['clients']


before = clients()
sock = socket.create_connection((host, int(port)))
key = base64.b64encode(os.urandom(16)).decode()
sock.sendall((f'GET /ws HTTP/1.1\r\nHost: {host}:{port}\r\n'
              'Upgrade: websocket\r\nConnection: Upgrade\r\n'
              f'Sec-WebSocket-Key: {key}\r\n'
              'Sec-WebSocket-Version: 13\r\n\r\n').encode())
reply = sock.recv(1024).split(b'\r\n', 1)[0]
# From here on: never read, never write. The receive buffer fills and
# stays full; pings arrive and are never answered.
t0 = time.time()
rows = []
reaped = None
while time.time() - t0 < LIMIT:
    count = clients()
    rows.append((round(time.time() - t0, 1), count))
    if count == before and reaped is None and time.time() - t0 > 1:
        reaped = round(time.time() - t0, 1)
        break
    time.sleep(1.0)
sock.close()
print(json.dumps({'handshake': reply.decode(errors='replace'),
                  'clients_before': before,
                  'clients_while_silent': max(c for _, c in rows),
                  'reaped_after_s': reaped, 'samples': rows[-5:]},
                 indent=1))
