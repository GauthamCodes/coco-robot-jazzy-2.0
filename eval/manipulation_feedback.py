"""Post-run finger feedback, using the controller's existing constants.

AST reading avoids importing ROS/autonomy into the evaluator and prevents a
second set of setpoints/tolerances. Commands and plugin reports are not feedback.
"""
import ast
from pathlib import Path
from oracle import finite


def contract():
    path = Path(__file__).resolve().parents[1] / 'coco_moveit_config/scripts/arm_control.py'
    wanted = {'GRIPPER_JOINTS', 'GRIP_OPEN', 'GRIP_TOLERANCE', 'LIFT_MIN_RISE'}
    values = {}
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in wanted:
                    values[target.id] = ast.literal_eval(node.value)
    if values.keys() != wanted:
        raise ValueError('Existing manipulation feedback contract changed')
    return values


def opening(indexed, release_s, detach_s, max_gap, ref):
    c = contract(); samples = []; rejected = []
    for row in indexed:
        _, event, data = row
        if event['topic'] != '/joint_states':
            continue
        try:
            stamp = data['header']['stamp']
            sec, ns = stamp['sec'], stamp['nanosec']
            if type(sec) is not int or type(ns) is not int or not 0 <= ns < 10**9:
                raise ValueError('invalid source stamp')
            t = sec + ns / 1e9
            names, positions = data['name'], data['position']
            if len(names) != len(positions) or len(set(names)) != len(names):
                raise ValueError('ambiguous joint array')
            joints = dict(zip(names, positions))
            actual = [joints[j] for j in c['GRIPPER_JOINTS']]
            if not all(finite(v) for v in actual):
                raise ValueError('nonfinite finger feedback')
        except (KeyError, TypeError, ValueError):
            rejected.append(dict(evidence=ref(row), reason='Missing/malformed source stamp or both finger positions'))
            continue
        if release_s is None or detach_s is None or not release_s <= t < detach_s:
            continue
        # No nearest/latest extrapolation. Both source and receipt precede detach.
        if not finite(max_gap) or not 0 <= event['sim_s']-t <= max_gap or not 0 <= detach_s-t <= max_gap or event['sim_s'] >= detach_s:
            rejected.append(dict(evidence=ref(row), reason='Uncalibrated gap, stale, future, or post-detach receipt'))
            continue
        error = max(abs(a-b) for a,b in zip(actual,c['GRIP_OPEN']))
        samples.append(dict(sim_s=t, evidence=ref(row), positions_rad=actual,
                            max_open_error_rad=error, open=error <= c['GRIP_TOLERANCE']))
    samples.sort(key=lambda s:s['sim_s'])
    latest = samples[-1] if samples else None
    conflict = bool(latest and any(s['sim_s']==latest['sim_s'] and s['positions_rad']!=latest['positions_rad'] for s in samples))
    return dict(status='UNVERIFIED' if not latest or conflict else 'PASS' if latest['open'] else 'FAIL',
                fingers_open_before_detach=None if not latest or conflict else latest['open'],
                samples=samples, rejected=rejected, contract=c,
                reason='Conflicting feedback at same source stamp' if conflict else
                       'Latest source-stamped feedback before detach' if latest else
                       'No fresh two-finger feedback before detach; commands cannot substitute')
