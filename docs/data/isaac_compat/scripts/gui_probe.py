"""GUI probe: windowed SimulationApp, default stage creation (viewport wait
NOT bypassed), falling box, viewport screenshot, then close."""
import os
import sys
import time

LOG = os.environ.get('PROBE_LOG', '.')
T0 = time.time()


def stamp(msg):
    print('[%7.1fs] PROBE %s' % (time.time() - T0, msg), flush=True)


from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({'headless': False, 'width': 1280, 'height': 720})
stamp('SimulationApp STARTED (GUI, default viewport wait)')

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdPhysics.Scene.Define(stage, '/World/physics')
g = UsdGeom.Cube.Define(stage, '/World/ground')
g.CreateSizeAttr(1.0)
g.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
g.AddScaleOp().Set(Gf.Vec3f(10, 10, 0.1))
UsdPhysics.CollisionAPI.Apply(g.GetPrim())
b = UsdGeom.Cube.Define(stage, '/World/box')
b.CreateSizeAttr(0.5)
b.AddTranslateOp().Set(Gf.Vec3d(0, 0, 3.0))
b.CreateDisplayColorAttr([(0.9, 0.1, 0.1)])
UsdPhysics.CollisionAPI.Apply(b.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
UsdLux.DistantLight.Define(stage, '/World/sun').CreateIntensityAttr(3000.0)
stamp('scene built')


def z():
    return UsdGeom.Xformable(b.GetPrim()).ComputeLocalToWorldTransform(0).ExtractTranslation()[2]


tl = omni.timeline.get_timeline_interface()
tl.play()
z0 = z()
dts = []
for i in range(600):
    t = time.time()
    app.update()
    dts.append(time.time() - t)
    if i % 100 == 0:
        stamp('update %d dt=%.3f box_z=%.3f' % (i, dts[-1], z()))
stamp('box z %.3f -> %.3f; median dt %.4fs (%.1f fps)' % (
    z0, z(), sorted(dts)[len(dts) // 2], 1.0 / sorted(dts)[len(dts) // 2]))
try:
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
    path = os.path.join(LOG, 'gui_viewport.png')
    capture_viewport_to_file(get_active_viewport(), path)
    for _ in range(30):
        app.update()
    stamp('screenshot %s exists=%s bytes=%s' % (
        path, os.path.exists(path), os.path.getsize(path) if os.path.exists(path) else 0))
except Exception as e:  # noqa: BLE001
    stamp('screenshot failed: %s' % e)
tl.stop()
app.close()
stamp('closed')
sys.exit(0)
