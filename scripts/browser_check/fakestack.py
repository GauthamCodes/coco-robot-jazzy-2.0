"""
Serve the REAL platform_server app (make_app/Platform/ControlSocket) on
:8090 with a fake ROS node producing synthetic, moving data -- so the page
can be rendered and exercised in a real browser without a simulator.

Usage: python3 fakestack.py <repo_worktree> [port]
"""
import base64
import importlib.util
import math
import os
import struct
import sys
import time

REPO = sys.argv[1]
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8090
sys.path.insert(0, os.path.join(REPO, 'coco_web'))

from coco_web import imaging, protocol  # noqa: E402
from coco_web import platform_server as ps  # noqa: E402

spec = importlib.util.spec_from_file_location(
    'tps', os.path.join(REPO, 'coco_web', 'test', 'test_platform_server.py'))
tps = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tps)

import tornado.ioloop  # noqa: E402

STATES = ['IDLE', 'LOCALIZE', 'NAVIGATE_TO_RAMP', 'ALIGN_FOR_CLIMB', 'CLIMB',
          'VERIFY_CLIMB', 'SEARCH_TARGET', 'STOW_ARM', 'APPROACH_TARGET',
          'GRASP', 'VERIFY_GRASP', 'DESCEND', 'RETURN_HOME', 'PLACE',
          'VERIFY_PLACEMENT', 'COMPLETE']


def make_map():
    w, h, res = 243, 175, 0.05
    ox, oy = -2.119, -4.910
    cells = bytearray(w * h)
    for y in range(h):
        for x in range(w):
            wx, wy = ox + x * res, oy + y * res
            wall = x < 3 or y < 3 or x > w - 4 or y > h - 4
            box = (1.0 < wx < 1.6 and 1.2 < wy < 1.8) or \
                  (-1.2 < wx < -0.8 and -2.5 < wy < -1.5)
            cells[y * w + x] = 100 if (wall or box) else 0
    return protocol.map_frame(w, h, res, ox, oy,
                              base64.b64encode(bytes(cells)).decode('ascii'))


def main():
    node = tps.FakeNode(sim=(len(sys.argv) < 4 or sys.argv[3] != 'nosim'))
    node.world_geometry = lambda: {
        'frame': 'map', 'offset_x': 2.0,
        'ramp': {'x0': 3.0, 'x1': 5.0, 'width': 2.5},
        'platform': {'x0': 5.0, 'x1': 6.5, 'width': 2.5},
        'home': {'x': 0.0, 'y': 0.0},
        'targets': [{'colour': c, 'x': 6.05, 'y': y, 'diameter': 0.06}
                    for c, y in (('red', -0.75), ('green', -0.25),
                                 ('blue', 0.25), ('yellow', 0.75))]}
    node.snap['map'] = make_map()
    node.snap['map_seq'] = 1
    node.fresh.update({'mission': True, 'navigation': True,
                       'perception': True, 'colour': True})
    node.snap['colour'] = 'green'
    node.snap['perception'] = 'online=1 found=0 seen=green'
    node.snap['arbiter'] = 'mode=nav active=nav teleop=-- nav=0.05 rl=-- approach=--'
    platform = ps.Platform(node)
    app = ps.make_app(platform, os.path.join(REPO, 'coco_web', 'web'))
    app.listen(PORT, address='127.0.0.1')
    t0 = time.time()
    seq = {'n': 0}

    lose_at = 8.0 if (len(sys.argv) > 3 and sys.argv[3] == 'lose') else None
    if len(sys.argv) > 3 and sys.argv[3] == 'simonly':
        # COCO's simulator is stepping, but the controllers and the
        # arbiter are not up yet: SIMULATOR_READY.
        node.fresh['robot'] = node.fresh['arbiter'] = False

    def step():
        t = time.time() - t0
        if lose_at is not None and t > lose_at:
            # A required component falls over AFTER convergence: ERROR.
            node.fresh['arbiter'] = False
        x, y = 2.0 + 1.5 * math.cos(t / 6), 1.5 * math.sin(t / 6)
        yaw = t / 6 + math.pi / 2
        node.snap['pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}
        node.snap['velocity'] = {'linear': 0.25, 'angular': 0.17}
        node.snap['scan'] = {
            'angle_min': -2.0944, 'angle_step': 4.1888 / 239,
            'ranges': [None if i % 37 == 0 else 1.5 + 0.5 * math.sin(i / 9)
                       for i in range(240)],
            'floor': 0.15}
        node.snap['path'] = [[x + 0.2 * k, y + 0.05 * k * k]
                             for k in range(12)]
        state = STATES[int(t / 4) % len(STATES)]
        node.snap['mission'] = (
            f'state={state} prev=IDLE event=enter elapsed={t % 4:.1f} '
            f'timeout=60.0 attempt=1 retries=2 owner=nav2 mode=nav '
            f'reason=-- result={"fetch" if state == "COMPLETE" else "--"}')
        seq['n'] += 1
        rgb = bytes((i * 7 + seq['n'] * 3) % 256 for i in range(320 * 240 * 3))
        jpeg, w, h = imaging.colour_jpeg(320, 240, 'rgb8', rgb, quality=60)
        node.snap['camera'] = {'seq': seq['n'], 't': time.time(), 'w': w,
                               'h': h, 'jpeg': jpeg, 'extra': None,
                               'quality': 60}
        depth = b''.join(struct.pack('<f', 0.3 + (i % 320) / 40.0)
                         for i in range(320 * 240))
        jd, w, h, lo, hi = imaging.depth_jpeg(320, 240, '32FC1', depth,
                                              quality=60, clip=(0.1, 8.0))
        node.snap['depth'] = {'seq': seq['n'], 't': time.time(), 'w': w,
                              'h': h, 'jpeg': jd,
                              'extra': {'min_m': lo, 'max_m': hi,
                                        'palette': 'grey_near_bright'},
                              'quality': 60}
        platform.tick()

    tornado.ioloop.PeriodicCallback(step, 100).start()
    print(f'fake stack on http://127.0.0.1:{PORT}', flush=True)
    tornado.ioloop.IOLoop.current().start()


main()
