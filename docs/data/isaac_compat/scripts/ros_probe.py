"""Isaac Sim <-> ROS 2 feasibility probe (4.2 and 4.5).

Stage 1 (PROBE_STAGE=1): /clock, Twist in (/isaac/cmd_vel) moves a rigid
body, odometry (/isaac/odom) and TF out.
Stage 2 (PROBE_STAGE=2): stage 1 + RGB, depth (ROS2CameraHelper) and a
PhysX 2D LiDAR (/isaac/scan).
Writes <PROBE_LOG>/ready.marker when publishing starts, runs PROBE_SECONDS.
"""
import faulthandler
import math
import os
import sys
import time

LOG = os.environ.get('PROBE_LOG', '.')
STAGE = int(os.environ.get('PROBE_STAGE', '1'))
SECONDS = float(os.environ.get('PROBE_SECONDS', '60'))
W, H = (int(v) for v in os.environ.get('PROBE_RES', '160x120').split('x'))
T0 = time.time()
_FH = open(os.path.join(LOG, 'stack.txt'), 'w')
faulthandler.dump_traceback_later(int(os.environ.get('PROBE_STACK_AT', '600')),
                                  repeat=False, file=_FH)


def stamp(msg):
    print('[%7.1fs] PROBE %s' % (time.time() - T0, msg), flush=True)


stamp('stage=%d res=%dx%d ROS_DISTRO=%s RMW=%s DOMAIN=%s LD=%s' % (
    STAGE, W, H, os.environ.get('ROS_DISTRO'),
    os.environ.get('RMW_IMPLEMENTATION'), os.environ.get('ROS_DOMAIN_ID'),
    os.environ.get('LD_LIBRARY_PATH')))

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({'headless': True, 'create_new_stage': False,
                     'width': W, 'height': H})
stamp('SimulationApp STARTED')

import omni.kit.app  # noqa: E402

em = omni.kit.app.get_app().get_extension_manager()
if em.get_extension_dict('isaacsim.ros2.bridge') is not None or \
        any(e['name'] == 'isaacsim.ros2.bridge' for e in em.get_extensions()):
    V = '4.5'
    BR, CORE, PX = 'isaacsim.ros2.bridge', 'isaacsim.core.nodes', 'isaacsim.sensors.physx'
else:
    V = '4.2'
    BR, CORE, PX = 'omni.isaac.ros2_bridge', 'omni.isaac.core_nodes', 'omni.isaac.range_sensor'
stamp('API generation %s: bridge=%s core=%s physx_sensors=%s' % (V, BR, CORE, PX))
import omni.graph.core as og  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import usdrt  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

ctx = omni.usd.get_context()
ctx.new_stage()
for _ in range(3):
    app.update()
# enable extensions AFTER the new stage: enabling the bridge first and
# then calling new_stage() segfaulted in omni.graph.image.core (gdb, 4.5)
for ext in ((BR, PX) if STAGE >= 2 else (BR,)):
    ok = em.set_extension_enabled_immediate(ext, True)
    stamp('enable %s -> %s' % (ext, ok))

for _ in range(3):
    app.update()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdPhysics.Scene.Define(stage, '/World/physics')

ground = UsdGeom.Cube.Define(stage, '/World/ground')
ground.CreateSizeAttr(1.0)
ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
ground.AddScaleOp().Set(Gf.Vec3f(20.0, 20.0, 0.1))
UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

obstacle = UsdGeom.Cube.Define(stage, '/World/obstacle')
obstacle.CreateSizeAttr(0.5)
obstacle.AddTranslateOp().Set(Gf.Vec3d(3.0, 1.2, 0.25))  # off the drive line, in camera + lidar view
obstacle.CreateDisplayColorAttr([(0.9, 0.1, 0.1)])
UsdPhysics.CollisionAPI.Apply(obstacle.GetPrim())

UsdLux.DistantLight.Define(stage, '/World/sun').CreateIntensityAttr(3000.0)

# "robot": a rigid box whose velocity we set from the Twist
base = UsdGeom.Xform.Define(stage, '/World/base')
base.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.11))
base.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
body = UsdGeom.Cube.Define(stage, '/World/base/body')
body.CreateSizeAttr(1.0)
body.AddScaleOp().Set(Gf.Vec3f(0.4, 0.3, 0.2))
UsdPhysics.CollisionAPI.Apply(body.GetPrim())
UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr(5.0)
# frictionless contact so the commanded velocity is what the body does
# (a sliding 5 kg box otherwise cancels 0.5 rad/s of yaw rate in one step)
from pxr import UsdShade  # noqa: E402
mat = UsdShade.Material.Define(stage, '/World/ice')
pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
pm.CreateStaticFrictionAttr(0.0)
pm.CreateDynamicFrictionAttr(0.0)
pm.CreateRestitutionAttr(0.0)
UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(
    mat, UsdShade.Tokens.weakerThanDescendants, 'physics')
# friction combines by AVERAGE: an icy body on a default ground still has
# mu 0.25 and cannot yaw (yaw_probe.py: 2.2 deg vs 85.8 deg with both icy)
UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(
    mat, UsdShade.Tokens.weakerThanDescendants, 'physics')
base_prim = base.GetPrim()

keys = og.Controller.Keys
nodes = [
    ('Tick', 'omni.graph.action.OnPlaybackTick'),
    ('SimTime', CORE + '.IsaacReadSimulationTime'),
    ('Clock', BR + '.ROS2PublishClock'),
    ('Twist', BR + '.ROS2SubscribeTwist'),
    ('Odom', CORE + '.IsaacComputeOdometry'),
    ('PubOdom', BR + '.ROS2PublishOdometry'),
    ('PubTF', BR + '.ROS2PublishRawTransformTree'),
]
values = [
    ('Twist.inputs:topicName', '/isaac/cmd_vel'),
    ('Odom.inputs:chassisPrim', [usdrt.Sdf.Path('/World/base')]),
    ('PubOdom.inputs:topicName', '/isaac/odom'),
    ('PubOdom.inputs:odomFrameId', 'odom'),
    ('PubOdom.inputs:chassisFrameId', 'base_link'),
    ('PubTF.inputs:parentFrameId', 'odom'),
    ('PubTF.inputs:childFrameId', 'base_link'),
]
connect = [
    ('Tick.outputs:tick', 'Clock.inputs:execIn'),
    ('SimTime.outputs:simulationTime', 'Clock.inputs:timeStamp'),
    ('Tick.outputs:tick', 'Twist.inputs:execIn'),
    ('Tick.outputs:tick', 'Odom.inputs:execIn'),
    ('Odom.outputs:execOut', 'PubOdom.inputs:execIn'),
    ('Odom.outputs:execOut', 'PubTF.inputs:execIn'),
    ('SimTime.outputs:simulationTime', 'PubOdom.inputs:timeStamp'),
    ('SimTime.outputs:simulationTime', 'PubTF.inputs:timeStamp'),
    ('Odom.outputs:position', 'PubOdom.inputs:position'),
    ('Odom.outputs:orientation', 'PubOdom.inputs:orientation'),
    ('Odom.outputs:linearVelocity', 'PubOdom.inputs:linearVelocity'),
    ('Odom.outputs:angularVelocity', 'PubOdom.inputs:angularVelocity'),
    ('Odom.outputs:position', 'PubTF.inputs:translation'),
    ('Odom.outputs:orientation', 'PubTF.inputs:rotation'),
]

if STAGE >= 2:
    import omni.replicator.core as rep
    cam = UsdGeom.Camera.Define(stage, '/World/base/cam')
    xf = UsdGeom.XformCommonAPI(cam)
    xf.SetTranslate(Gf.Vec3d(0.2, 0.0, 0.15))
    xf.SetRotate(Gf.Vec3f(90.0, 0.0, -90.0))
    cam.CreateFocalLengthAttr(18.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    rp = rep.create.render_product('/World/base/cam', (W, H))
    rp_path = rp.path if hasattr(rp, 'path') else str(rp)
    stamp('render product %s' % rp_path)
    ok, lidar = omni.kit.commands.execute(
        'RangeSensorCreateLidar', path='/lidar', parent='/World/base',
        min_range=0.15, max_range=12.0, draw_points=False, draw_lines=False,
        horizontal_fov=360.0, vertical_fov=1.0,
        horizontal_resolution=float(os.environ.get('PROBE_LIDAR_RES', '1.0')),
        vertical_resolution=1.0, rotation_rate=0.0, high_lod=False,
        yaw_offset=0.0, enable_semantics=False)
    lidar_path = str(lidar.GetPath())
    lidar.GetPrim().GetAttribute('xformOp:translate').Set(Gf.Vec3d(0.0, 0.0, 0.2))
    stamp('lidar %s created=%s' % (lidar_path, ok))
    nodes += [
        ('RGB', BR + '.ROS2CameraHelper'),
        ('Depth', BR + '.ROS2CameraHelper'),
        ('Beams', PX + '.IsaacReadLidarBeams'),
        ('Scan', BR + '.ROS2PublishLaserScan'),
    ]
    values += [
        ('RGB.inputs:renderProductPath', rp_path),
        ('RGB.inputs:topicName', '/isaac/rgb'),
        ('RGB.inputs:type', 'rgb'),
        ('RGB.inputs:frameId', 'camera'),
        ('Depth.inputs:renderProductPath', rp_path),
        ('Depth.inputs:topicName', '/isaac/depth'),
        ('Depth.inputs:type', 'depth'),
        ('Depth.inputs:frameId', 'camera'),
        ('Beams.inputs:lidarPrim', [usdrt.Sdf.Path(lidar_path)]),
        ('Scan.inputs:topicName', '/isaac/scan'),
        ('Scan.inputs:frameId', 'laser'),
    ]
    connect += [
        ('Tick.outputs:tick', 'RGB.inputs:execIn'),
        ('Tick.outputs:tick', 'Depth.inputs:execIn'),
        ('Tick.outputs:tick', 'Beams.inputs:execIn'),
        ('Beams.outputs:execOut', 'Scan.inputs:execIn'),
        ('SimTime.outputs:simulationTime', 'Scan.inputs:timeStamp'),
    ] + [('Beams.outputs:' + a, 'Scan.inputs:' + a) for a in (
        'azimuthRange', 'depthRange', 'horizontalFov', 'horizontalResolution',
        'intensitiesData', 'linearDepthData', 'numCols', 'numRows',
        'rotationRate')]

try:
    og.Controller.edit({'graph_path': '/World/ROSGraph', 'evaluator_name': 'execution'},
                       {keys.CREATE_NODES: nodes, keys.SET_VALUES: values,
                        keys.CONNECT: connect})
    stamp('graph built: %d nodes' % len(nodes))
except Exception as e:  # noqa: BLE001
    import traceback
    traceback.print_exc()
    stamp('GRAPH FAILED: %s' % e)
    app.close()
    sys.exit(3)

twist_lin = og.Controller.attribute('/World/ROSGraph/Twist.outputs:linearVelocity')
twist_ang = og.Controller.attribute('/World/ROSGraph/Twist.outputs:angularVelocity')

timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(2):
    app.update()
import numpy as np  # noqa: E402
if V == '4.5':
    from isaacsim.core.prims import SingleRigidPrim as _RP
else:
    from omni.isaac.core.prims import RigidPrim as _RP
rb = _RP('/World/base')
rb.initialize()
stamp('rigid prim handle %s initialised' % type(rb).__name__)
open(os.path.join(LOG, 'ready.marker'), 'w').write('go')
stamp('PUBLISHING for %.0fs' % SECONDS)


def pose():
    m = UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(0)
    t = m.ExtractTranslation()
    r = m.ExtractRotation().Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
    return t[0], t[1], r[2]


n = 0
cmds = 0
last = None
t_end = time.time() + SECONDS
t_rep = time.time()
dts = []
while time.time() < t_end:
    t = time.time()
    app.update()
    dts.append(time.time() - t)
    n += 1
    lin = twist_lin.get()
    ang = twist_ang.get()
    cmd = (float(lin[0]), float(ang[2]))
    if cmd != (0.0, 0.0):
        cmds += 1
    if cmd != last:
        stamp('twist in: linear.x=%.3f angular.z=%.3f' % cmd)
        last = cmd
    x, y, yaw_deg = pose()
    yaw = math.radians(yaw_deg)
    rb.set_linear_velocity(np.array([cmd[0] * math.cos(yaw), cmd[0] * math.sin(yaw), 0.0]))
    rb.set_angular_velocity(np.array([0.0, 0.0, cmd[1]]))
    if time.time() - t_rep > 5.0:
        stamp('updates=%d (%.1f Hz) pose x=%.3f y=%.3f yaw=%.1fdeg nonzero_cmd_frames=%d'
              % (n, n / (time.time() - T0 - 0), x, y, yaw_deg, cmds))
        t_rep = time.time()

x, y, yaw_deg = pose()
dts.sort()
stamp('RESULT updates=%d median_dt=%.4fs p95_dt=%.4fs final pose x=%.3f y=%.3f yaw=%.1fdeg nonzero_cmd_frames=%d'
      % (n, dts[len(dts) // 2], dts[int(len(dts) * 0.95)], x, y, yaw_deg, cmds))
timeline.stop()
app.close()
stamp('closed')
sys.exit(0)
