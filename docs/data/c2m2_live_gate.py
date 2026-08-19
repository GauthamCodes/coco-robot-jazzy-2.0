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

"""C2-M2.1 live observer gate.

**The gate that found three defects no off-line test could see.** Run it
against a live Gazebo with `terrain_observer` and `cmd_vel_arbiter` up::

    ros2 launch gazebo_models full_world_robo.launch.py gui:=false
    ros2 run coco_rl terrain_observer --ros-args -p use_sim_time:=true \\
        -p declare_flat:=true
    ros2 run custom_teleop cmd_vel_arbiter --ros-args \\
        -p use_sim_time:=true -p initial_mode:=rl
    python3 docs/data/c2m2_live_gate.py /tmp/gate.csv 40 0.35

Add `world:=coco_yard.world` to reach Route B's 26 deg chute, which is the
only surface here steep enough to establish the traction bound: the v1
wedge is 18 deg and tan(18 deg) = 0.325 sits BELOW the 0.35 a-priori
friction floor, so on the wedge the bound can never become informative and
B3 stays in fallback by construction.

Drives the robot up the v1 wedge (foot x=1.0, summit x=3.0, 18 deg)
through the EXISTING cmd_vel_arbiter and records what /terrain/state says
against ground-truth odometry.

Publishes only to /cmd_vel_rl -- an arbiter SOURCE, never the controller
topic -- so cmd_vel_arbiter stays the sole publisher to
/diff_drive_controller/cmd_vel.

It also drives the REAL B3 controller off the live estimate, so the
"observer -> B3 -> gains" half of the chain is exercised by the same
messages the robot produced, and deliberately WITHDRAWS the estimate near
the end to show the fallback engaging.

Measures; tunes nothing.
"""
import csv
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from coco_rl.baselines import B3, TUNED_SCHEDULE
from coco_rl.terrain_observer import TerrainEstimate
from coco_sim.yard import load_params

RAMP_FOOT_X = 1.0
RAMP_SUMMIT_X = 3.0
RAMP_ANGLE_DEG = 18.0


def true_grade_deg(x):
    """Analytic grade of the v1 wedge at world x. Ground truth, eval only."""
    if RAMP_FOOT_X <= x <= RAMP_SUMMIT_X:
        return RAMP_ANGLE_DEG
    return 0.0


class LiveGate(Node):

    def __init__(self, out, seconds, speed):
        super().__init__('c2m21_live_gate')
        self.out = out
        self.seconds = float(seconds)
        self.speed = float(speed)
        self.rows = []
        self.t0 = None
        self.state = None
        self.odom = None

        # The real B3, driven by the live estimate. Route 'a' only supplies
        # a gain block and a reference y; the wedge is not the Yard and no
        # ascent claim is made from it. What is under test is the wiring:
        # does B3 consume a live TerrainEstimate, and does it fall back.
        self.params = load_params()
        self.b3 = B3(schedule=TUNED_SCHEDULE, params=self.params)
        self.b3.reset(_Sample(self.params), 'a')

        self._drive = self.create_publisher(TwistStamped, '/cmd_vel_rl', 10)
        self._mode = self.create_publisher(String, '/mission/mode', 10)
        self.create_subscription(
            DiagnosticArray, '/terrain/state', self._on_state, 10)
        self.create_subscription(
            Odometry, '/model/coco/odometry', self._on_odom,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.1, self._tick)
        self._mode_ticks = 0
        self.withdraw_start = self.seconds - 6.0

    def _on_state(self, msg):
        self.state = msg

    def _on_odom(self, msg):
        self.odom = msg

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        t = self._now()
        if self.t0 is None:
            if self.odom is None or self.state is None:
                return
            self.t0 = t
        el = t - self.t0

        if self._mode_ticks < 30:
            self._mode.publish(String(data='rl'))
            self._mode_ticks += 1

        if el < self.seconds:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist.linear.x = self.speed
            self._drive.publish(m)

        self._record(el)
        if el > self.seconds + 2.0:
            self._finish()

    def _fields(self):
        """Key/values, NAMESPACED BY STATUS.

        Both statuses publish a key called `confidence`. Flattening them
        into one dict silently lets traction overwrite grade, which is
        exactly what the first version of this instrument did -- it
        reported grade confidence 0.000 across a climb whose real grade
        confidence was 1.000. An instrument bug that reads like a result.
        """
        out = {}
        for s in self.state.status:
            ns = 'grade' if 'grade' in s.name else 'traction'
            lvl = (int.from_bytes(s.level, 'big')
                   if isinstance(s.level, bytes) else int(s.level))
            out[f'{ns}_level'] = lvl
            out[f'{ns}_msg'] = s.message
            for kv in s.values:
                out[f'{ns}_{kv.key}'] = kv.value
        return out

    def _estimate_from(self, g, withdrawn):
        """Rebuild a TerrainEstimate from the wire, as a subscriber would.

        `withdrawn` forges exactly what the observer itself emits when an
        input goes stale: every instantaneous channel invalid, the bound
        preserved. That is the fallback trigger, not an invented one.
        """
        def fl(k, d=0.0):
            try:
                return float(g.get(k, d))
            except ValueError:
                return d
        valid = g.get('grade_level') == 0 or g.get('grade_level') == 1
        if withdrawn:
            return TerrainEstimate(
                stamp=fl('grade_stamp'), grade=0.0, grade_valid=False,
                grade_confidence=0.0,
                grade_calibrated=g.get('grade_calibrated') == 'True',
                grade_roughness=0.0, camber=0.0, camber_valid=False,
                tau=0.0, mu_lower=fl('traction_mu_lower_bound'),
                mu_hat=fl('traction_mu_sched_input'),
                traction_valid=False, traction_confidence=0.0,
                mu_established=g.get('traction_established') == 'True',
                saturated=False, deficit=0.0,
                reason='stale input: withdrawn by the gate')
        return TerrainEstimate(
            stamp=fl('grade_stamp'),
            grade=math.radians(fl('grade_grade_deg')),
            grade_valid=bool(valid),
            grade_confidence=fl('grade_confidence'),
            grade_calibrated=g.get('grade_calibrated') == 'True',
            grade_roughness=math.radians(fl('grade_roughness_deg')),
            camber=math.radians(fl('grade_camber_deg')), camber_valid=True,
            tau=fl('traction_tau_traction_demand'),
            mu_lower=fl('traction_mu_lower_bound'),
            mu_hat=fl('traction_mu_sched_input'),
            traction_valid=g.get('traction_level') == 0,
            traction_confidence=fl('traction_confidence'),
            mu_established=g.get('traction_established') == 'True',
            saturated=g.get('traction_saturated') == 'True',
            deficit=fl('traction_deficit_mps2'), reason='')

    def _record(self, el):
        if self.state is None or self.odom is None:
            return
        g = self._fields()
        withdrawn = el >= self.withdraw_start
        est = self._estimate_from(g, withdrawn)

        # Feed the REAL B3 and read the gains it resolves.
        self.b3.last_estimate = est
        gains = self.b3._resolve_gains()

        p = self.odom.pose.pose.position
        x = p.x
        self.rows.append(dict(
            t=round(el, 3), x=round(x, 4), y=round(p.y, 4), z=round(p.z, 4),
            true_grade_deg=true_grade_deg(x),
            grade_deg=g.get('grade_grade_deg', ''),
            camber_deg=g.get('grade_camber_deg', ''),
            grade_conf=g.get('grade_confidence', ''),
            roughness_deg=g.get('grade_roughness_deg', ''),
            calibrated=g.get('grade_calibrated', ''),
            disagreement_deg=g.get('grade_disagreement_deg', ''),
            est_stamp=g.get('grade_stamp', ''),
            tau=g.get('traction_tau_traction_demand', ''),
            mu_lower=g.get('traction_mu_lower_bound', ''),
            mu_sched_input=g.get('traction_mu_sched_input', ''),
            established=g.get('traction_established', ''),
            saturated=g.get('traction_saturated', ''),
            grade_level=g.get('grade_level', ''),
            traction_level=g.get('traction_level', ''),
            grade_msg=g.get('grade_msg', ''),
            traction_msg=g.get('traction_msg', ''),
            withdrawn=int(withdrawn),
            b3_engaged=int(self.b3.engaged),
            b3_throttle=round(gains['throttle'], 4),
            b3_lateral=round(gains['lateral'], 4),
            b3_fallback_rate=round(self.b3.fallback_rate, 4),
        ))

    def _finish(self):
        if not self.rows:
            print('NO ROWS RECORDED', file=sys.stderr)
            rclpy.shutdown()
            return
        with open(self.out, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)
        print(f'wrote {self.out}: {len(self.rows)} rows')
        rclpy.shutdown()


class _Route:
    def __init__(self, y):
        self.y_centre = y


class _Sample:
    """Minimal stand-in for the Yard sample B3.reset reads y_centre from."""

    def __init__(self, params):
        self.routes = {k: _Route(params['routes'][k]['y_centre'])
                       for k in ('a', 'b', 'c')}


def main():
    out = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 40.0
    speed = float(sys.argv[3]) if len(sys.argv) > 3 else 0.35
    rclpy.init()
    n = LiveGate(out, seconds, speed)
    try:
        rclpy.spin(n)
    except Exception:
        pass


if __name__ == '__main__':
    main()
