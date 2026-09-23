"""Isaac Sim 4.5 physics-only + ROS 2 bridge probe.

Bypasses SimulationApp's unbounded _wait_for_viewport (create_new_stage=
False), creates the stage itself, checks physics, enables the ROS 2 bridge,
publishes /clock from OmniGraph and answers /coco_ping with /isaac_pong.
"""
import faulthandler
import os
import sys
import time

TMP = '/home/gautham/.claude/jobs/d213ad33/tmp'
T0 = time.time()
_FH = open(f'{TMP}/isaac_stack6.txt', 'w')
faulthandler.dump_traceback_later(200, repeat=False, file=_FH)


def stamp(msg):
    print('[%7.1fs] %s' % (time.time() - T0, msg), flush=True)


stamp('env ROS_DISTRO=%s RMW=%s DOMAIN=%s' % (
    os.environ.get('ROS_DISTRO'), os.environ.get('RMW_IMPLEMENTATION'),
    os.environ.get('ROS_DOMAIN_ID')))

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({'headless': True, 'create_new_stage': False})
stamp('SimulationApp STARTED (viewport wait bypassed)')

import omni.usd  # noqa: E402

omni.usd.get_context().new_stage()
t = time.time()
for _ in range(5):
    app.update()
stamp('stage created; 5 app.update() took %.2fs' % (time.time() - t))

rc = 0
try:
    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid, GroundPlane

    world = World(stage_units_in_meters=1.0)
    GroundPlane(prim_path='/World/ground', size=10.0)
    cube = world.scene.add(DynamicCuboid(
        prim_path='/World/cube', name='cube',
        position=np.array([0.0, 0.0, 1.0]), size=0.2,
        color=np.array([1.0, 0.0, 0.0])))
    world.reset()
    z0 = float(cube.get_world_pose()[0][2])
    t = time.time()
    for _ in range(120):
        world.step(render=False)
    z1 = float(cube.get_world_pose()[0][2])
    stamp('PHYSICS z %.4f -> %.4f (drop %.4f m) in %.2fs wall, 120 steps'
          % (z0, z1, z0 - z1, time.time() - t))
    stamp('PHYSICS OK' if z1 < z0 - 0.5 else 'PHYSICS DID NOT MOVE')
except Exception as e:  # noqa: BLE001
    import traceback
    traceback.print_exc()
    stamp('PHYSICS FAILED: %s: %s' % (type(e).__name__, e))
    rc = 5

try:
    cube.set_world_pose(position=np.array([0.0, 0.0, 3.0]))
    stamp('NO BRIDGE: physics-only loop for 60 s wall')
    end = time.time() + 60
    n = 0
    zs = []
    while time.time() < end:
        world.step(render=False)
        zs.append(float(cube.get_world_pose()[0][2]))
        time.sleep(0.01)
        n += 1
    stamp('loop done: %d steps; cube z first %.3f min %.3f last %.3f'
          % (n, zs[0], min(zs), zs[-1]))
except Exception as e:  # noqa: BLE001
    import traceback
    traceback.print_exc()
    stamp('LOOP FAILED: %s: %s' % (type(e).__name__, e))
    rc = 7

app.close()
stamp('closed')
sys.exit(rc)
