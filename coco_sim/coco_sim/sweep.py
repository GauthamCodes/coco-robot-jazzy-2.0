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
Guards for parameter sweeps, so a disconnected knob cannot pass as a
flat response.

Why this module exists
----------------------
M7 Phase 2 added explicit MJCF ``<pair>`` elements so terrain friction
below the wheel value would be reachable. Pairs do **not** inherit
``solref``/``solimp`` from their geoms — they silently fall back to
MuJoCo's defaults — so that change discarded the entire contact-softness
calibration while ``mjcf.py`` still declared the calibrated values and
every number in the file still read as correct. The worst sim-to-sim yaw
deviation went from 1.274x to 1.936x and nothing in the source looked
wrong.

It was caught by luck: a sweep happened to print byte-identical scores for
``solref`` 0.1, 0.25 and 0.35, and somebody noticed. That is not a
detection strategy. **A disconnected parameter is silent by
construction** — the sweep runs, the numbers are plausible, and the knob
simply is not wired to anything.

So every sweep asserts its own connectivity. The check is cheap, it is
impossible to forget once it is in the harness, and it names the parameter
rather than leaving a human to spot a coincidence in a column of numbers.

This is the same argument as ``check_lifted`` in the grasp: do not trust a
system to report its own success when the failure mode produces a
plausible-looking result.
"""


class DisconnectedParameter(AssertionError):
    """A swept parameter produced identical results for distinct inputs."""


def assert_lever_is_connected(name, results, tol=0.0):
    """Fail loudly if sweeping `name` changed nothing at all.

    `results` is an iterable of ``(input_value, measured_output)``. Raises
    :class:`DisconnectedParameter` when every output is identical, which
    in practice means the parameter never reached the model.

    `tol` is the spread below which outputs count as identical. It
    defaults to **exactly zero** on purpose: a genuinely weak lever still
    moves a floating-point result in the last bits, whereas a disconnected
    one returns bit-for-bit the same number. Raising `tol` turns this from
    a wiring check into a sensitivity check, which is a different question
    and should be asked separately.

    Returns the observed spread, so a caller can also report how strong
    the lever is once it is known to exist.
    """
    pairs = list(results)
    if len(pairs) < 2:
        raise ValueError(
            f'{name}: need at least two (input, output) pairs to tell a '
            f'disconnected parameter from a connected one')

    inputs = [k for k, _ in pairs]
    if len(set(map(repr, inputs))) < 2:
        raise ValueError(
            f'{name}: the swept inputs are not distinct ({inputs!r}), so '
            f'identical outputs would prove nothing')

    outputs = [v for _, v in pairs]
    spread = max(outputs) - min(outputs)
    if spread <= tol:
        raise DisconnectedParameter(
            f'{name}: swept {inputs!r} and every result was '
            f'{outputs[0]!r}. A real lever does not return identical '
            f'outputs for distinct inputs — this parameter is almost '
            f'certainly not reaching the model. Check that it is not being '
            f'shadowed by a default that overrides it (MuJoCo <pair> '
            f'elements override geom solref/solimp exactly this way).')
    return spread
