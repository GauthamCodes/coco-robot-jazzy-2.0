#!/usr/bin/env python3
"""List listening TCP sockets from /proc (no iproute2 needed)."""
import socket
import struct


def addr(hexaddr, v6):
    ip, port = hexaddr.split(':')
    if v6:
        raw = bytes.fromhex(ip)
        raw = b''.join(raw[i:i + 4][::-1] for i in range(0, 16, 4))
        return f'[{socket.inet_ntop(socket.AF_INET6, raw)}]:{int(port, 16)}'
    return f'{socket.inet_ntoa(struct.pack("<I", int(ip, 16)))}:{int(port, 16)}'


for path, v6 in (('/proc/net/tcp', False), ('/proc/net/tcp6', True)):
    try:
        for line in open(path).readlines()[1:]:
            f = line.split()
            if f[3] == '0A':
                print(addr(f[1], v6))
    except FileNotFoundError:
        pass
