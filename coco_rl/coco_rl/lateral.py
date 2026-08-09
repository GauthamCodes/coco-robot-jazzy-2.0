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
The lane-hold correction, and the gains it ships with.

**Moved here from ``ramp_driver`` unchanged**, constants and function
body identical, because M7 Phase 3's B1 baseline is defined as "the
current ``lateral_hold`` with its existing global gains, unchanged" — and
``ramp_driver`` imports ``rclpy`` at module scope, which the MuJoCo
training environment must never do. Importing the real function rather
than reimplementing it is the whole point: a baseline that is a *copy* of
the shipped controller stops being the shipped controller the first time
either drifts.

``ramp_driver`` now imports from here, so there is still exactly one
definition. ``test_ramp_driver.py`` reaches it through ``ramp_driver`` and
keeps passing untouched.
"""


# LATERAL_CLAMP is a safety term rather than a tuning one: it bounds how far
# the correction can move the action away from the distribution the policy
# trained on. The clamp was swept too (0.4 / 0.8 / 1.2 / 2.0 moved the
# residual by 6 mm), which is what proved the limit was bandwidth and not
# authority. 0.8 sits just above the 0.625 peak the shipped gains actually
# ask for.
LATERAL_GAIN = 3.0      # action units per metre of drift
HEADING_GAIN = 2.5      # action units per radian of heading error
LATERAL_CLAMP = 0.8     # ceiling on the correction, in action units


def lateral_hold(action, y_err, yaw, gain=LATERAL_GAIN,
                 heading_gain=HEADING_GAIN, clamp=LATERAL_CLAMP):
    """
    Bias a policy action's yaw channel back toward the lane centreline.

    Pure, so the cases that matter — a correction of the right sign, a
    saturated policy action staying inside the action space, the clamp
    holding — are asserted without a simulator or a trained model.

    `action` is the policy's [linear, angular] in [-1, 1]; `y_err` is metres
    of drift from the lane the segment started in (observation index 1,
    positive to the left); `yaw` is heading error in radians, positive to
    the left. The linear channel is never touched: slowing down on a grade
    is how a skid-steer base loses traction, and speed is the policy's
    business.
    """
    correction = -(gain * float(y_err) + heading_gain * float(yaw))
    correction = max(-clamp, min(clamp, correction))
    angular = max(-1.0, min(1.0, float(action[1]) + correction))
    return [float(action[0]), angular]
