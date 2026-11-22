#!/usr/bin/env python3
"""Nearest world-file collision face to a world pose. Offline arithmetic."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', '..',
                                                 'docs', 'data')))
from c2nav7_geom import CIRCUMSCRIBED, STOP_RADIUS, nearest   # noqa: E402

x, y = float(sys.argv[1]), float(sys.argv[2])
print(f'pose ({x:+.4f}, {y:+.4f})')
for d, who, q in nearest(x, y)[:3]:
    tag = ''
    if d < CIRCUMSCRIBED:
        tag = '  <<< BELOW CIRCUMSCRIBED'
    elif d < STOP_RADIUS:
        tag = f'  inside PolygonStop by {(STOP_RADIUS-d)*1000:.1f} mm'
    print(f'  {d:.4f} m to {who:<16} at ({q[0]:+.4f}, {q[1]:+.4f}){tag}')
