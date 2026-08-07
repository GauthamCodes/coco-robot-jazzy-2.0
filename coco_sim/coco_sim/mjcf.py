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
2. **The controller's ``wheel_separation_multiplier`` is not applied.**
   Gazebo commands yaw against an effective track of
   ``WHEEL_SEPARATION * 1.10``; this model has the physical track. So
   straight-line motion should agree and yaw rate should not, by roughly
   that factor. Recorded in ``coco_config`` and measured rather than
   corrected — correcting it silently would hide a real difference
   between the two simulators.
"""

import argparse

from coco_config.robot import (
    CHASSIS_MASS, CHASSIS_SIZE, WHEEL_MASS, WHEEL_RADIUS, WHEEL_SEPARATION,
    WHEEL_WIDTH, WHEELBASE)

# Integrator timestep. M7_DESIGN 5.3 asks for the integrator timestep and
# control rate to match across both simulators; Gazebo runs 1 ms physics,
# so this does too. Divergence measured under matched timesteps is a
# physics difference; divergence under mismatched ones is arithmetic.
TIMESTEP = 0.001

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
WHEEL_FRICTION = 0.4         # was 0.7 (the xacro's value)
CONTACT_SOLREF = 0.1         # was MuJoCo's default 0.02 — softer contact
CONTACT_SOLIMP_D0 = 0.5      # was 0.9

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
            size="{cx / 2.0:.6f} {cy / 2.0:.6f} {cz / 2.0:.6f}"
            mass="{CHASSIS_MASS:.6f}"/>{''.join(wheels)}
    </body>
  </worldbody>

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
