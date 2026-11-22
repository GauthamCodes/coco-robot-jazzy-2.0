"""SYNTHETIC_TEST calibration and capture regressions."""
import json
import pytest
from capture_status import summarize
from manipulation_calibration import characterize, limits_usable


def test_empty_recorder_file_is_not_trace(tmp_path):
    (tmp_path/'logs').mkdir();(tmp_path/'logs/events.jsonl').touch()
    r=summarize(tmp_path,'ABORTED_BOOTSTRAP_SAFETY',['metadata unreadable'])
    assert not r['has_trace'];assert not r['capture_complete']
    assert 'metadata unreadable' in r['incomplete_reasons']


def test_ready_resource_abort_preserves_partial_trace(tmp_path):
    (tmp_path/'logs').mkdir();(tmp_path/'logs/recorder_ready.json').write_text('{}')
    (tmp_path/'logs/events.jsonl').write_text(json.dumps(dict(topic='/clock',data={}))+'\n')
    r=summarize(tmp_path,'ABORTED_BOOTSTRAP_RESOURCE_LIMIT',['RAM reserve'])
    assert r['has_trace'] and r['recorder_ready'] and not r['capture_complete']


def test_labels_never_approve_limits():
    records=[dict(label=k,label_evidence='SYNTHETIC_TEST fixture',evidence_class='SYNTHETIC_TEST',
                  measurements={'relative_translation_drift_m':v}) for k,v in [('ATTACHED',.001),('DETACHED',.2),('STATIONARY_NOISE',.0001)]]
    r=characterize(records)
    assert r['status']=='CALIBRATION_REQUIRED';assert all(v is None for v in r['approved_limits'].values())
    assert len(r['groups'])==3


def test_unlabeled_control_rejected():
    with pytest.raises(ValueError):characterize([{'label':'ATTACHED'}])


def test_string_provenance_cannot_approve_limits():
    assert not limits_usable(dict(relative_translation_m=.1,relative_rotation_rad=.1,
                                  max_truth_gap_sim_s=1,provenance='trust me'),{})
