# Copyright 2026 Gautham Anil
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Generate the MuJoCo model of the Coco base from coco_config.

**Nothing here is a hand-written dimension.** Every length, mass and
position comes from ``coco_config.robot``; ``test_mjcf_traces_to_config``
asserts that by rebuilding the model with a monkeypatched constant and
checking the output moved. The rule is M7_DESIGN 5.2's, and the reason is
specific: two hand-maintained robot models diverge within a week, and the
divergence does not announce itself as a transcription error — it
announces itself as a mysterious sim-to-sim transfer gap, which is a much
more expensive thing to chase.

What this model IS
------------------
The **base**, as physics sees it: a chassis box with four cylindrical
wheels on hinge joints, on a flat plane. Four velocity actuators, one per
wheel, so a differential command reaches the ground the same way it does
through ``diff_drive_controller``.

What it is NOT
--------------
Not the arm, not the sensors, not the meshes. The v1 observation vector is
``[x, y, sin yaw, cos yaw, v, w, roll, pitch]`` — pure base state — so a
base-only model is sufficient for parity with ``ramp_env`` and for the
throughput question M7 Phase 1 asks. Adding the arm before it is needed
would be geometry nobody has checked against anything.

Two honest caveats, both belonging in the transfer table rather than in a
fudge factor here:

1. **The visual meshes are not reproduced.** Inertia comes from primitive
   shapes with the xacro's masses, not from the CAD tensors. Masses match;
   inertia distribution does not exactly.
2. **Yaw does not transfer exactly, and is not made to.** Phase 1.5
   calibrated the contact here and got the worst sim-to-sim yaw deviation
   from 1.707x to 1.274x; it does not go lower, because friction is a weak
   lever on skid-steer scrub and Gazebo disagrees with its own mirrored
   command by 1.361x past 1 rad. ``mujoco_env`` applies
   ``WHEEL_SEPARATION_MULTIPLIER`` from ``coco_config`` — parity with the
   deployed controller — and randomises a yaw gain per episode over
   ``YAW_GAIN_RANGE`` to cover what is left. Transfer is bought by making
   the policy insensitive to steering authority, not by making the engines
   agree.
"""

import argparse

from coco_config.robot import (
    ARM_CHAIN_MASS, ARM_MOUNT_XYZ, AXLE_Z_IN_BASE_LINK, CAMERA_MASS,
    CAMERA_MOUNT_XYZ, CHASSIS_GROUND_CLEARANCE, CHASSIS_MASS, CHASSIS_SIZE,
    LIDAR_MASS, LIDAR_MOUNT_XYZ, WHEEL_MASS, WHEEL_RADIUS,
    WHEEL_SEPARATION, WHEEL_WIDTH, WHEELBASE)

# Integrator timestep, matching the Gazebo world's <max_step_size>.
#
# This read 0.001 until M7 Phase 2, under a comment asserting "Gazebo runs
# 1 ms physics, so this does too". That was false: coco_world.world line 10
# sets 0.002. So M7_DESIGN 5.3's "match integrator timestep exactly across
# both simulators" had been broken by 2x since Phase 1 -- including
# underneath the Phase 1.5 contact calibration, which is why that fit is
# re-verified after this change rather than assumed to survive it.
TIMESTEP = 0.002

# ramp_env's control period, so an env step means the same thing on both
# sides of the comparison.
CONTROL_DT = 0.1

# ── contact, calibrated against Gazebo (Phase 1.5) ───────────────────────
# These are NOT the xacro's numbers, and the difference is deliberate.
#
# Gazebo sets mu1 = mu2 = 0.7 on the wheels, isotropic, no fdir1 (the
# xacro warns that anisotropic mu in DART is a direction lottery). Copying
# 0.7 straight across gave a MuJoCo base that achieved only ~61% of a
# commanded yaw where Gazebo achieves ~103%, a 1.71x gap at the small
# commands the lane hold actually issues.
#
# What that gap was NOT, each measured and ruled out:
#   - anisotropic friction: both engines are isotropic. Refuted at source.
#   - torsional friction: condim=3 changed achieved yaw by <1%.
#   - actuator tracking: the velocity servos deliver 98.8% of the
#     commanded left-right wheel-speed difference at kv=10.
# It is skid-steer scrub: the wheels turn at the right speed and the body
# under-rotates, and MuJoCo's contact resists that scrub far more than
# DART's does at the same nominal friction.
#
# Sliding friction turned out to be a weak lever on this (0.2 -> 1.5 moved
# yaw efficiency only 59.5% -> 65.2%); contact SOFTNESS is the strong one.
# Fitted across a 7-point yaw sweep, both signs: worst deviation from
# Gazebo 1.707x -> 1.274x, and straight-line agreement IMPROVED from 4.1%
# to 2.8%. Numbers and method in docs/RESULTS.md.
WHEEL_FRICTION = 0.4         # the xacro's value is 0.7; this is fitted
CONTACT_SOLREF = 0.25        # MuJoCo's default is 0.02 — softer contact
CONTACT_SOLIMP_D0 = 0.5      # default 0.9

# RE-FITTED in Phase 2, and the reason is worth keeping.
#
# The Phase 1.5 fit (0.4 / 0.1 / 0.5, worst 1.274x) was made against a
# model with three defects: a 1 ms timestep against Gazebo's 2 ms, a
# chassis 2.1x too high, and 11% too little mass with all the deficit high
# and rearward. Correcting those moved the worst deviation to 1.936x --
# worse than the 1.707x it had started from. A calibration is only ever a
# fit to the model underneath it, and that model had changed.
#
# The re-fit then exposed a second, sharper problem: adding the <pair>
# elements had SILENTLY DISCARDED the softness calibration. MuJoCo pairs do
# not inherit solref/solimp from the geoms -- they fall back to the engine
# defaults (0.02, 0.9) -- so the strong lever had quietly stopped applying
# while still being set on the geoms and still appearing in this file. It
# is only visible in mjModel.pair_solref. Hence the pair below carries the
# contact parameters explicitly rather than relying on inheritance.
#
# Re-fitted worst deviation: 1.170x, better than Phase 1.5's 1.274x, and
# at the principled separation multiplier (controller parity) rather than
# an inflated one.

# EXPLICIT CONTACT PAIRS, and the reason is not cosmetic.
#
# MuJoCo combines the friction of two geoms as the elementwise MAX. With
# the wheels pinned at 0.4 by the calibration above, that makes any terrain
# with mu < 0.4 a no-op: the pair still slides at 0.4. M7_DESIGN 2.5 asks
# for friction randomised over 0.35-1.10, so its entire low end would have
# been silently unreachable in the training simulator -- while Gazebo, which
# combines per-shape coefficients differently, would give ~0.245 for the
# same nominal surface. A randomisation range that cannot be entered is
# worse than none, because the training curve looks fine.
#
# A <pair> sets the wheel-ground coefficient DIRECTLY, bypassing the max
# rule, so `mu` in yard_params.yaml means the effective wheel-ground
# coefficient in both engines rather than a per-surface value that each
# engine then combines its own way.
USE_EXPLICIT_CONTACT_PAIRS = True

# The deployed diff_drive_controller applies wheel_separation_multiplier
# to the physical track before computing wheel speeds. Reproducing that
# here is parity with the deployment target, not a fudge: a policy that
# commands (linear, angular) through cmd_vel gets that conversion in
# Gazebo, so it must get it in training too. coco_config records the
# value; mujoco_env applies it.

WHEEL_SITES = (
    # name,           x sign,  y sign
    ('front_right', +1.0, -1.0),
    ('rear_right',  -1.0, -1.0),
    ('front_left',  +1.0, +1.0),
    ('rear_left',   -1.0, +1.0),
)


def wheel_positions():
    """(name, x, y, z) for the four wheels, in the chassis body frame.

    Derived, not typed: the wheels sit at half the wheelbase fore and aft
    and half the track left and right, with their axles on the chassis
    centre plane.
    """
    return [(name, sx * WHEELBASE / 2.0, sy * WHEEL_SEPARATION / 2.0, 0.0)
            for name, sx, sy in WHEEL_SITES]


def chassis_z_in_body():
    """Chassis box centre in the MJCF body frame (origin = axle height).

    The body roots at the axles, which sit ``AXLE_Z_IN_BASE_LINK`` above
    base_link; the chassis box centre sits ``CHASSIS_GROUND_CLEARANCE +
    half-height`` above the ground. Both are known, so the offset is
    derived rather than typed:

        ground clearance 0.0135 + half-height 0.030 = 0.0435 above ground
        axle             0.0585                      above ground
        -> chassis centre is 0.015 BELOW the body origin

    Before Phase 2 the chassis was pinned at the body origin, which put its
    underside 28.5 mm up instead of 13.5 -- a 2.1x error in the one number
    that decides whether the robot bellies out on an obstacle, and one that
    a terrain-height parity test cannot see.
    """
    return (CHASSIS_GROUND_CLEARANCE + CHASSIS_SIZE[2] / 2.0
            - (AXLE_Z_IN_BASE_LINK + CHASSIS_GROUND_CLEARANCE))


def lumped_masses():
    """(name, xyz in body frame, mass) for everything that is not a wheel.

    Placed where the parts actually are rather than folded into the
    chassis: the arm is rearward, the lidar is on a mast, and this world's
    whole subject is camber-induced tipping, which CoM height decides.
    Positions are the xacro's joint origins in base_link, shifted down by
    the axle height because the MJCF body roots at the axles.
    """
    dz = AXLE_Z_IN_BASE_LINK
    return (
        ('arm_mass', (ARM_MOUNT_XYZ[0], ARM_MOUNT_XYZ[1],
                      ARM_MOUNT_XYZ[2] - dz), ARM_CHAIN_MASS),
        ('lidar_mass', (LIDAR_MOUNT_XYZ[0], LIDAR_MOUNT_XYZ[1],
                        LIDAR_MOUNT_XYZ[2] - dz), LIDAR_MASS),
        ('camera_mass', (CAMERA_MOUNT_XYZ[0], CAMERA_MOUNT_XYZ[1],
                         CAMERA_MOUNT_XYZ[2] - dz), CAMERA_MASS),
    )


def build_mjcf():
    """Return the MJCF XML for the base, as a string."""
    cx, cy, cz = CHASSIS_SIZE
    wheels = []
    for name, x, y, z in wheel_positions():
        wheels.append(f"""
      <body name="{name}" pos="{x:.6f} {y:.6f} {z:.6f}">
        <joint name="{name}_joint" type="hinge" axis="0 1 0"/>
        <geom name="{name}_geom" type="cylinder" size="{WHEEL_RADIUS:.6f} {WHEEL_WIDTH / 2.0:.6f}"
              quat="0.7071068 0.7071068 0 0" mass="{WHEEL_MASS:.6f}"
              friction="{WHEEL_FRICTION} 0.005 0.0001"/>
      </body>""")

    actuators = '\n'.join(
        f'    <velocity name="{name}_motor" joint="{name}_joint" kv="10"/>'
        for name, _, _, _ in wheel_positions())

    # contype/conaffinity 0: these carry mass and inertia and collide with
    # nothing. They stand in for parts that exist on the real robot but
    # whose collision geometry is not modelled here.
    lumped = '\n'.join(
        f'      <geom name="{name}" type="sphere" size="0.01" '
        f'pos="{x:.6f} {y:.6f} {z:.6f}" mass="{m:.6f}" '
        f'contype="0" conaffinity="0"/>'
        for name, (x, y, z), m in lumped_masses())

    pairs = ''
    if USE_EXPLICIT_CONTACT_PAIRS:
        # solref/solimp are set HERE, not left to the geom defaults. A
        # <pair> does not inherit them -- it silently uses MuJoCo's own
        # (0.02, 0.9) -- which discarded the whole softness calibration
        # once without changing a single visible number in this file.
        rows = '\n'.join(
            f'    <pair geom1="ground" geom2="{name}_geom" '
            f'friction="{WHEEL_FRICTION} {WHEEL_FRICTION} 0.005 '
            f'0.0001 0.0001" '
            f'solref="{CONTACT_SOLREF} 1" '
            f'solimp="{CONTACT_SOLIMP_D0} 0.99 0.001"/>'
            for name, _, _, _ in wheel_positions())
        pairs = f'\n  <contact>\n{rows}\n  </contact>\n'

    return f"""<mujoco model="coco_base">
  <!-- GENERATED by coco_sim.mjcf from coco_config.robot. Do not edit by
       hand: regenerate. Every dimension below traces to a constant that
       coco_config/test/test_base_matches_urdf.py pins to the xacro. -->
  <compiler angle="radian" coordinate="local"/>
  <option timestep="{TIMESTEP}" integrator="implicitfast"/>

  <default>
    <geom condim="4" solref="{CONTACT_SOLREF} 1"
          solimp="{CONTACT_SOLIMP_D0} 0.99 0.001"/>
  </default>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="ground" type="plane" size="50 50 0.1"
          friction="{WHEEL_FRICTION} 0.005 0.0001"/>

    <body name="base" pos="0 0 {WHEEL_RADIUS:.6f}">
      <freejoint name="base_free"/>
      <geom name="chassis" type="box"
            pos="0 0 {chassis_z_in_body():.6f}"
            size="{cx / 2.0:.6f} {cy / 2.0:.6f} {cz / 2.0:.6f}"
            mass="{CHASSIS_MASS:.6f}"/>
{lumped}{''.join(wheels)}
    </body>
  </worldbody>
{pairs}
  <actuator>
{actuators}
  </actuator>
</mujoco>
"""


def main():
    """Write the MJCF to a path, for inspection or for mujoco's viewer."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-o', '--output', default='coco_base.xml')
    args = parser.parse_args()
    with open(args.output, 'w') as handle:
        handle.write(build_mjcf())
    print(f'wrote {args.output}')


if __name__ == '__main__':
    main()
