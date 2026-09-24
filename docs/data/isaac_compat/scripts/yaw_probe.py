"""Physics-only: does a commanded yaw rate turn a rigid box? No ROS."""
import math
import sys
from isaacsim import SimulationApp

app = SimulationApp({'headless': True, 'create_new_stage': False})
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(3):
    app.update()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdPhysics.Scene.Define(stage, '/World/physics')
g = UsdGeom.Cube.Define(stage, '/World/ground')
g.CreateSizeAttr(1.0)
g.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
g.AddScaleOp().Set(Gf.Vec3f(20, 20, 0.1))
UsdPhysics.CollisionAPI.Apply(g.GetPrim())
mat = UsdShade.Material.Define(stage, '/World/ice')
pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
pm.CreateStaticFrictionAttr(0.0)
pm.CreateDynamicFrictionAttr(0.0)
MODE = sys.argv[1]
if 'groundice' in MODE:
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, 'physics')
base = UsdGeom.Xform.Define(stage, '/World/base')
base.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.11))
base.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))
body = UsdGeom.Cube.Define(stage, '/World/base/body')
body.CreateSizeAttr(1.0)
body.AddScaleOp().Set(Gf.Vec3f(0.4, 0.3, 0.2))
UsdPhysics.CollisionAPI.Apply(body.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr(5.0)
UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, 'physics')
if 'air' in MODE:   # lift clear of the ground: no contact at all
    base.GetPrim().GetAttribute('xformOp:translate').Set(Gf.Vec3d(0, 0, 2.0))
    UsdPhysics.Scene.Get(stage, '/World/physics').CreateGravityMagnitudeAttr(0.0)
tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(2):
    app.update()
from isaacsim.core.prims import SingleRigidPrim  # noqa: E402
rb = SingleRigidPrim('/World/base')
rb.initialize()


def yaw():
    m = UsdGeom.Xformable(base.GetPrim()).ComputeLocalToWorldTransform(0)
    return m.ExtractRotation().Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())[2]


for i in range(180):
    rb.set_angular_velocity(np.array([0.0, 0.0, 0.5]))
    app.update()
    if i % 30 == 0:
        print('MODE %s frame %d yaw=%.2f get_ang=%s' % (MODE, i, yaw(), np.round(rb.get_angular_velocity(), 3)), flush=True)
print('MODE %s RESULT yaw after 180 updates = %.2f deg (sim time %.2f s)'
      % (MODE, yaw(), tl.get_current_time()), flush=True)
tl.stop()
app.close()
