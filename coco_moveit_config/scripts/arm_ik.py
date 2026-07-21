#!/usr/bin/env python3
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
arm_ik.py
=========
Analytic 2-link planar IK for the Coco arm.

The arm works entirely in the robot's x-z plane (y = 0): the shoulder
(`m_link1_Revolute-6`) and elbow (`m_link2_Revolute-7`) both rotate about
the base y-axis. That reduces IK to the classic planar 2R problem.

Geometry (derived numerically from the URDF kinematic chain
base_footprint -> ... -> m_link3; see the constants' comments):

    elbow  = S + L1 * u(alpha),      alpha = PHI1 - q_shoulder
    pinch  = elbow + L2 * u(beta),   beta  = alpha + GAMMA0 + q_elbow

where u(t) = (cos t, sin t) in the x-z plane and "pinch" is the midpoint
between the closed fingertips — the same reference point used to place
the pick target (verified: fk(0.30, 0.58) == (0.152, 0.128), the
pedestal-top grasp of pick_place.py).

All coordinates are in the `base_footprint` frame (x forward, z up,
origin on the ground under the robot).
"""

import math

# Shoulder pivot in base_footprint (from URDF chain, arm plane y = 0)
S_X = -0.075
S_Z = 0.1485
# Link lengths: shoulder pivot -> elbow pivot, elbow pivot -> fingertip pinch
L1 = 0.150000
L2 = 0.086739
# Zero-pose phase offsets (CAD frames are not aligned with the joint zeros)
PHI1 = 0.007576          # alpha at q_shoulder = 0; alpha = PHI1 - q_shoulder
GAMMA0 = -0.021274       # beta - alpha at q_elbow = 0

# Joint limits from the URDF
SHOULDER_LIMITS = (-3.84, 1.0)
ELBOW_LIMITS = (-1.6, 1.6)


def fk(q_shoulder, q_elbow):
    """Pinch-point (x, z) in base_footprint for the given joint angles."""
    alpha = PHI1 - q_shoulder
    beta = alpha + GAMMA0 + q_elbow
    x = S_X + L1 * math.cos(alpha) + L2 * math.cos(beta)
    z = S_Z + L1 * math.sin(alpha) + L2 * math.sin(beta)
    return x, z


def ik(x, z):
    """All within-limits joint solutions (q_shoulder, q_elbow) that put the
    fingertip pinch point at (x, z) in base_footprint. [] if unreachable.

    The positive-gamma branch (the one the verified pick poses use) is
    listed first when both elbow configurations are feasible.
    """
    rx, rz = x - S_X, z - S_Z
    d2 = rx * rx + rz * rz
    cos_g = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(cos_g) > 1.0:
        return []

    solutions = []
    for sign in (+1.0, -1.0):
        gamma = sign * math.acos(max(-1.0, min(1.0, cos_g)))
        q_elbow = gamma - GAMMA0
        alpha = math.atan2(rz, rx) - math.atan2(
            L2 * math.sin(gamma), L1 + L2 * math.cos(gamma))
        if not ELBOW_LIMITS[0] <= q_elbow <= ELBOW_LIMITS[1]:
            continue
        # principal value in (-pi, pi], but the shoulder limit range
        # (-3.84, 1.0) extends past -pi, so test the 2*pi-equivalent too
        q_p = math.atan2(math.sin(PHI1 - alpha), math.cos(PHI1 - alpha))
        for q_shoulder in (q_p, q_p - 2 * math.pi, q_p + 2 * math.pi):
            if SHOULDER_LIMITS[0] <= q_shoulder <= SHOULDER_LIMITS[1]:
                solutions.append((q_shoulder, q_elbow))
                break
    return solutions


def ik_or_none(x, z):
    """First (preferred) IK solution, or None if (x, z) is unreachable."""
    sols = ik(x, z)
    return sols[0] if sols else None


if __name__ == '__main__':
    import sys
    if len(sys.argv) != 3:
        print(f'usage: {sys.argv[0]} X Z   (metres, base_footprint frame)')
        sys.exit(2)
    tx, tz = float(sys.argv[1]), float(sys.argv[2])
    sols = ik(tx, tz)
    if not sols:
        print(f'({tx:.3f}, {tz:.3f}) is unreachable within joint limits')
        sys.exit(1)
    for q1, q2 in sols:
        fx, fz = fk(q1, q2)
        print(f'shoulder={q1:+.4f} elbow={q2:+.4f}  (fk check: {fx:.4f}, {fz:.4f})')
