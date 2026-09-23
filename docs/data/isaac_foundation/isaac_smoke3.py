import sys, time, os

def stamp(msg):
    print("[%7.1fs] %s" % (time.time() - T0, msg), flush=True)

T0 = time.time()
import faulthandler
_FH = open("/home/gautham/.claude/jobs/d213ad33/tmp/isaac_stack.txt", "w")
faulthandler.dump_traceback_later(360, repeat=False, file=_FH)
stamp("python %s" % sys.version.split()[0])

try:
    import isaacsim
    stamp("import isaacsim OK")
except Exception as e:
    stamp("IMPORT FAILED: %s: %s" % (type(e).__name__, e))
    sys.exit(2)

try:
    from isaacsim import SimulationApp
    stamp("import SimulationApp OK")
except Exception as e:
    stamp("SimulationApp IMPORT FAILED: %s: %s" % (type(e).__name__, e))
    sys.exit(3)

stamp("constructing SimulationApp(headless=True) ...")
try:
    app = SimulationApp({"headless": True})
except Exception as e:
    import traceback; traceback.print_exc()
    stamp("SimulationApp START FAILED: %s: %s" % (type(e).__name__, e))
    sys.exit(4)
stamp("SimulationApp STARTED")

rc = 0
try:
    stamp("importing core api ...")
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid
    import numpy as np
    stamp("core api OK; building world ...")

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    stamp("ground plane added")
    cube = world.scene.add(
        DynamicCuboid(prim_path="/World/cube", name="cube",
                      position=np.array([0.0, 0.0, 1.0]),
                      size=0.2, color=np.array([1.0, 0.0, 0.0]))
    )
    stamp("cube added; resetting ...")
    world.reset()
    z0 = float(cube.get_world_pose()[0][2])
    stamp("reset done, z0=%.4f; stepping 90 frames ..." % z0)
    for _ in range(90):
        world.step(render=False)
    z1 = float(cube.get_world_pose()[0][2])
    stamp("PHYSICS z %.4f -> %.4f (drop %.4f m)" % (z0, z1, z0 - z1))
    stamp("PHYSICS OK" if z1 < z0 - 0.1 else "PHYSICS DID NOT MOVE")
except Exception as e:
    import traceback; traceback.print_exc()
    stamp("PHYSICS TEST FAILED: %s: %s" % (type(e).__name__, e))
    rc = 5

try:
    app.close()
    stamp("closed cleanly")
except Exception as e:
    stamp("close raised: %s" % e)
sys.exit(rc)
