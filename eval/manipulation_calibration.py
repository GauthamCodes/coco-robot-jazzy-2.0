"""Characterize labeled witness distributions; never approve thresholds.

Input labels must be independent of this verifier. Synthetic and historical
records are diagnostic data, never qualified production calibration controls.
"""
import argparse
import json
from pathlib import Path
from calibration import quantile
from evidence import file_hash, atomic_json
from oracle import finite

KEYS = ('relative_translation_m', 'relative_rotation_rad', 'max_truth_gap_sim_s')


def limits_usable(limits, manifest):
    if not all(finite(limits.get(k)) and limits[k] > 0 for k in KEYS):
        return False
    if manifest.get('evidence_class') == 'SYNTHETIC_TEST':
        return limits.get('status') == 'SYNTHETIC_TEST'
    # New engineering candidates cannot silently become approved evaluation
    # thresholds. Approval must reference committed methodology and controls.
    if limits.get('status') != 'APPROVED' or not limits.get('approval_reference'):
        return False
    sources = limits.get('sources', [])
    if not sources:
        return False
    try:
        return all(file_hash(Path(s['path'])) == s['sha256'] for s in sources)
    except (OSError, KeyError, TypeError):
        return False


def characterize(records):
    fields = ('relative_translation_drift_m', 'relative_rotation_drift_rad',
              'attachment_proximity_m', 'max_sample_gap_sim_s')
    groups = {}
    for r in records:
        if r.get('label') not in ('ATTACHED', 'DETACHED', 'STATIONARY_NOISE') or not r.get('label_evidence'):
            raise ValueError('Independent physical label and evidence reference required')
        if r.get('evidence_class') not in ('SYNTHETIC_TEST', 'HISTORICAL_REPLAY', 'QUALIFIED_CONTROL', 'NON_QUALIFYING'):
            raise ValueError('Explicit evidence class required')
        key = r['evidence_class'] + ':' + r['label']
        group = groups.setdefault(key, dict(records=0, measurements={}))
        group['records'] += 1
        for f in fields:
            v = r.get('measurements', {}).get(f)
            if v is None:
                continue
            if not finite(v) or v < 0:
                raise ValueError('Measurements must be finite and nonnegative')
            group['measurements'].setdefault(f, []).append(v)
    for group in groups.values():
        group['measurements'] = {k:dict(n=len(v), minimum=min(v), p01=quantile(v,.01),
                                      median=quantile(v,.5), p99=quantile(v,.99), maximum=max(v))
                                 for k,v in group['measurements'].items()}
    return dict(schema_version=1, status='CALIBRATION_REQUIRED', groups=groups,
                approved_limits={k:None for k in KEYS}, candidate_limits={k:None for k in KEYS},
                unresolved=['Independent positive and negative physical controls with complete synchronized poses',
                            'Stationary measurement-noise characterization across relevant poses',
                            'Justified margins and sample-gap coverage, then explicit methodology approval',
                            'Production I21 proximity remains governed by eval/calibration.py and qualified oracle-PASS Dev controls'],
                qualification_effect='NONE; summaries are not approval or sufficient sample-size evidence')


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('labels', type=Path); p.add_argument('--output', required=True, type=Path)
    a=p.parse_args()
    if a.output.exists(): raise FileExistsError(a.output)
    result=characterize(json.loads(a.labels.read_text()))
    result['input_sha256']=file_hash(a.labels)
    atomic_json(a.output,result)
