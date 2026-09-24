"""Portable Isaac Sim render + physics probe (4.2 and 4.5).

Pure USD (pxr) scene, timeline-driven physics, Replicator annotators for
RGB and depth. No isaacsim.core / omni.isaac.core API, so the same file
runs on both releases. Parameters from the environment:
  PROBE_RES=160x120  PROBE_FRAMES=60  PROBE_LOG=<dir>
"""
import faulthandler
import os
import sys
import time

LOG = os.environ.get('PROBE_LOG', '.')
W, H = (int(v) for v in os.environ.get('PROBE_RES', '160x120').split('x'))
FRAMES = int(os.environ.get('PROBE_FRAMES', '60'))
T0 = time.time()
_FH = open(os.path.join(LOG, 'stack.txt'), 'w')
faulthandler.dump_traceback_later(int(os.environ.get('PROBE_STACK_AT', '600')),
                                  repeat=False, file=_FH)


def stamp(msg):
    print('[%7.1fs] PROBE %s' % (time.time() - T0, msg), flush=True)


stamp('res=%dx%d frames=%d VK_ICD_FILENAMES=%s' % (
    W, H, FRAMES, os.environ.get('VK_ICD_FILENAMES')))

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({'headless': True, 'create_new_stage': False,
                     'width': W, 'height': H})
stamp('SimulationApp STARTED')

import omni.usd  # noqa: E402
import omni.timeline  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(3):
    app.update()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdPhysics.Scene.Define(stage, '/World/physics')

ground = UsdGeom.Cube.Define(stage, '/World/ground')
ground.CreateSizeAttr(1.0)
ground.AddScaleOp().Set(Gf.Vec3f(10.0, 10.0, 0.1))
ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

box = UsdGeom.Cube.Define(stage, '/World/box')
box.CreateSizeAttr(0.3)
box.AddTranslateOp().Set(Gf.Vec3d(1.5, 0.0, 1.0))
box.CreateDisplayColorAttr([(0.9, 0.1, 0.1)])
UsdPhysics.CollisionAPI.Apply(box.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())

light = UsdLux.DistantLight.Define(stage, '/World/sun')
light.CreateIntensityAttr(3000.0)

cam = UsdGeom.Camera.Define(stage, '/World/cam')
# USD cameras look down -Z; rotate so it looks along +X from (-1, 0, 0.5)
xf = UsdGeom.XformCommonAPI(cam)
xf.SetTranslate(Gf.Vec3d(-1.0, 0.0, 0.5))
xf.SetRotate(Gf.Vec3f(90.0, 0.0, -90.0))
cam.CreateFocalLengthAttr(18.0)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
stamp('scene built')

import omni.replicator.core as rep  # noqa: E402

t = time.time()
rp = rep.create.render_product('/World/cam', (W, H))
rgb = rep.AnnotatorRegistry.get_annotator('rgb')
depth = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
rgb.attach([rp])
depth.attach([rp])
stamp('render product + annotators attached in %.2fs' % (time.time() - t))

timeline = omni.timeline.get_timeline_interface()
timeline.play()
box_prim = box.GetPrim()


def box_z():
    m = UsdGeom.Xformable(box_prim).ComputeLocalToWorldTransform(0)
    return m.ExtractTranslation()[2]


import numpy as np  # noqa: E402

dts = []
first_rgb = None
z0 = box_z()
BUDGET = float(os.environ.get('PROBE_WARMUP_S', '300'))
i = 0
t_loop = time.time()
while True:
    t = time.time()
    app.update()
    dts.append(time.time() - t)
    a = rgb.get_data()
    a_shape = getattr(a, 'shape', None)
    have = bool(a_shape) and len(a_shape) == 3
    if have and first_rgb is None:
        first_rgb = i
        stamp('FIRST RGB at update %d, %.1fs after loop start' % (i, time.time() - t_loop))
    if first_rgb is not None and (i - first_rgb) % 10 == 0:
        d = depth.get_data()
        d_shape = getattr(d, 'shape', None)
        nz = int(np.count_nonzero(a[..., :3]))
        finite = d[np.isfinite(d)] if d_shape and len(d_shape) == 2 else np.array([])
        stamp('frame %d dt=%.3fs rgb=%s nonzero=%d mean=%.1f depth=%s finite=%d range=[%s] box_z=%.3f'
              % (i, dts[-1], a_shape, nz, float(a[..., :3].mean()), d_shape, finite.size,
                 '%.3f..%.3f' % (finite.min(), finite.max()) if finite.size else '-', box_z()))
        np.save(os.path.join(LOG, 'rgb_frame.npy'), a)
        np.save(os.path.join(LOG, 'depth_frame.npy'), d)
    i += 1
    if first_rgb is not None and i - first_rgb >= FRAMES:
        break
    if first_rgb is None and time.time() - t_loop > BUDGET:
        stamp('NO RGB within %.0fs (%d updates)' % (BUDGET, i))
        break
post = dts[first_rgb + 1:] if first_rgb is not None else dts
z1 = box_z()
steady = post[5:] if len(post) > 10 else (post or dts)
stamp('RESULT frames=%d first_dt=%.2fs steady_mean_dt=%.4fs (%.1f fps) '
      'first_nonzero_rgb=%s box_z %.3f -> %.3f'
      % (len(dts), dts[0], sum(steady) / len(steady),
         len(steady) / sum(steady), first_rgb, z0, z1))
timeline.stop()
app.close()
stamp('closed')
sys.exit(0)
