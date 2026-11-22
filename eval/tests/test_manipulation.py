"""SYNTHETIC_TEST traces, never robot runtime evidence or calibration."""
import copy
import json
import pytest
from manipulation import verify, PHASES, run

LIMITS=dict(relative_translation_m=.005, relative_rotation_rad=.01,
            max_truth_gap_sim_s=1.1, provenance='SYNTHETIC_TEST regression limits',status='SYNTHETIC_TEST')
MANIFEST=dict(seed=5,requested_colour='red',evidence_class='SYNTHETIC_TEST')
ORACLE=dict(oracle_verdict='PASS',mission_verdict='COMPLETE',checks={'target_at_rest':True})

def observation(t):
    stamp=round((t-.01)*1e9)
    return dict(schema='coco.object_height.v1',valid=True,reason='VALID',frame_id='base_footprint',
                source_frame='camera_optical_frame',units='m',colour='red',transform_valid=True,
                track_id='track',reference_id='support',stamp_ns=stamp,depth_stamp_ns=stamp,
                center_height_above_support_m=.08,support_height_m=0.,center_m=[.1,0,.08],
                uncertainty_m=.001,depth_sample_count=100,inlier_count=90,valid_fraction=1.,
                measured_radius_m=.01,measured_axis_extent_m=.16)

def fixture():
    events=[]
    def add(topic,t,data):events.append(dict(topic=topic,sim_s=t,wall_s=t*2,data=data))
    def mark(name,t):add('/diagnostics/manipulation_checkpoints',t,dict(checkpoint=name,stamp_ns=round(t*1e9),observation_contract_valid=True,observation_stamp_ns=round((t-.01)*1e9),phase_started_ns=1,observation=observation(t)))
    mark('observation_acquired',.5);mark('approach_complete',1)
    mark('grasp_request',2);add('/magnet/target_red/attach',2,{})
    add('/magnet/target_red/state',2.1,{'data':'attached'})
    mark('release_request',5.5);add('/magnet/target_red/detach',6,{})
    for t,x,z,gx,gz in [(1,0,.08,0,.1),(3,0,.08,0,.1),(4,0,.12,0,.14),(5,.1,.12,.1,.14),
                         (6,.1,.08,.1,.1),(7,.1,.08,.2,.2),(8,.1,.08,.3,.3)]:
        obj=dict(xyz_m=[x,0,z],quaternion_wxyz=[1,0,0,0])
        grip=dict(xyz_m=[gx,0,gz],quaternion_wxyz=[1,0,0,0],frame='world',source_sim_s=t,provenance='simulator_link_pose')
        add('/eval/ground_truth/state',t,dict(sim_s=t,frame='world',objects={'target_red':obj},gripper=grip))
    return events

def result(events=None,oracle=None,**kw):
    return verify(fixture() if events is None else events,MANIFEST,oracle or ORACLE,limits=LIMITS,**kw)

def statuses(r):return {p['phase']:p['status'] for p in r['phases']}

def truth(events):return [e['data'] for e in events if e['topic']=='/eval/ground_truth/state']

def test_valid_sequence_and_determinism():
    r=result();assert r['phase_sequence_result']=='PASS';assert r['result']=='UNVERIFIED';assert list(statuses(r))==list(PHASES)
    assert json.dumps(r,sort_keys=True)==json.dumps(result(),sort_keys=True)
    assert r['phases'][5]['invariant_results']['I21']['holds'] is None # proximity uncalibrated

@pytest.mark.parametrize('plugin',[False,True])
def test_command_or_report_never_proves_attachment(plugin):
    e=fixture()
    if not plugin:e=[r for r in e if not r['topic'].endswith('/state') or r['topic'].startswith('/eval')]
    for d in truth(e):d['objects']['target_red']['xyz_m']=[0,0,.08]
    s=statuses(result(e));assert s['physical_attachment']=='FAIL'
    assert s['plugin_attachment_report']==('PASS' if plugin else 'UNVERIFIED')

def test_attachment_then_failed_lift():
    e=fixture()
    for d in truth(e):
        if d['sim_s'] in (4,5):
            d['objects']['target_red']['xyz_m']=[.1*(d['sim_s']-3),0,.08]
            d['gripper']['xyz_m']=[.1*(d['sim_s']-3),0,.1]
    s=statuses(result(e));assert s['physical_attachment']=='PASS';assert s['physical_lift']=='FAIL'

def test_lift_without_retention():
    e=[e for e in fixture() if not(e['topic']=='/eval/ground_truth/state' and e['data']['sim_s']==5)]
    s=statuses(result(e));assert s['physical_lift']=='PASS';assert s['retention']=='UNVERIFIED'

@pytest.mark.parametrize('continued_follow',[False,True])
def test_failed_release(continued_follow):
    e=fixture()
    for d in truth(e):
        if d['sim_s']>=6:
            if continued_follow:d['objects']['target_red']['xyz_m']=[d['gripper']['xyz_m'][0],0,d['gripper']['xyz_m'][2]-.02]
            else:d['gripper']['xyz_m']=[.1,0,.1]
    assert statuses(result(e))['physical_release']==('FAIL' if continued_follow else 'UNVERIFIED')

def test_displacement_after_release():
    e=fixture();truth(e)[-1]['objects']['target_red']['xyz_m'][0]+=.02
    assert statuses(result(e))['post_release_stability']=='FAIL'

@pytest.mark.parametrize('field',['objects','gripper'])
def test_missing_physical_signal(field):
    e=fixture()
    for d in truth(e):d.pop(field)
    assert statuses(result(e))['physical_attachment']=='UNVERIFIED'

@pytest.mark.parametrize('change',['stale','frame','nonfinite','conflict'])
def test_invalid_truth_clock_and_frame(change):
    e=fixture();d=truth(e)[2]
    if change=='stale':d['gripper']['source_sim_s']=1000000
    if change=='frame':d['frame']='base_footprint'
    if change=='nonfinite':d['objects']['target_red']['xyz_m'][0]=float('nan')
    if change=='conflict':
        row=copy.deepcopy(next(r for r in e if r.get('data') is d));row['data']['objects']['target_red']['xyz_m'][0]=.2;e.append(row)
    assert result(e)['result']=='UNSCORABLE'

def test_duplicates_and_out_of_order_do_not_manufacture_motion():
    e=fixture();e.reverse();e+=copy.deepcopy(e)
    r=result(e);assert r['phase_sequence_result']=='PASS';assert r['duplicate_events']==len(e)//2

def test_missing_calibration_is_not_pass():
    r=verify(fixture(),MANIFEST,ORACLE)
    assert r['result']=='UNVERIFIED';assert statuses(r)['physical_attachment']=='UNVERIFIED'

def test_mission_oracle_disagreement_and_physical_progress():
    oracle=dict(ORACLE,oracle_verdict='FAIL',mission_oracle_disagreement=True)
    r=result(oracle=oracle);assert r['result']!='PASS';assert r['oracle']==oracle
    oracle=dict(ORACLE,mission_verdict='ABORT',mission_oracle_disagreement=True)
    assert statuses(result(oracle=oracle))['physical_lift']=='PASS'

def test_truncated_sidecar(tmp_path):
    (tmp_path/'logs').mkdir();(tmp_path/'manifest.json').write_text(json.dumps(MANIFEST))
    (tmp_path/'logs/events.jsonl').write_text('{"topic":')
    assert run(tmp_path,tmp_path/'out.json')['result']=='UNSCORABLE'

def test_trace_loss_blocks_all_passes():
    assert result(integrity_errors=['MCAP truncated'])['result']=='UNSCORABLE'

@pytest.mark.parametrize('data',[None,[],{'sim_s':'bad'}, {'sim_s':1,'frame':'world','objects':[]}])
def test_invalid_ground_truth_pipeline_returns_diagnostic(data):
    from evaluation import evaluate_recording
    e=fixture();e.append(dict(topic='/eval/ground_truth/state',sim_s=9,data=data))
    r=evaluate_recording(e,MANIFEST,{})
    assert r['manipulation']['result']=='UNSCORABLE'

def test_initial_detach_is_not_release_command():
    e=fixture()
    for row in e:
        if row['topic'].endswith('/detach'):row['sim_s']=.1
    assert statuses(result(e))['release_request']=='UNVERIFIED'

def test_multiple_attempts_not_silently_combined():
    e=fixture();row=copy.deepcopy(next(r for r in e if isinstance(r['data'],dict) and r['data'].get('checkpoint')=='grasp_request'))
    row['sim_s']=9;row['data']['stamp_ns']=9000000000;e.append(row)
    assert result(e)['result']=='UNSCORABLE'

@pytest.mark.parametrize('fault',['stale','tf','identity','pre_phase','report_only'])
def test_observation_requires_actual_contract(fault):
    e=fixture();d=next(r['data'] for r in e if isinstance(r['data'],dict) and r['data'].get('checkpoint')=='approach_complete')
    if fault=='stale':d['stamp_ns']+=1000000000
    if fault=='tf':d['observation']['transform_valid']=False
    if fault=='identity':d['observation']['track_id']='other'
    if fault=='pre_phase':d['phase_started_ns']=d['observation']['stamp_ns']
    if fault=='report_only':d.pop('observation')
    assert statuses(result(e))['approach_observation']=='UNVERIFIED'

def test_initial_stationary_samples_do_not_hide_later_attachment():
    e=fixture();row=copy.deepcopy(next(r for r in e if r['topic']=='/eval/ground_truth/state' and r['sim_s']==3))
    row['sim_s']=2.5;row['data']['sim_s']=2.5;row['data']['gripper']['source_sim_s']=2.5;e.append(row)
    assert statuses(result(e))['physical_attachment']=='PASS'

def test_rotation_slip_fails_retention():
    e=fixture();d=next(d for d in truth(e) if d['sim_s']==5)
    d['objects']['target_red']['quaternion_wxyz']=[.70710678,0,0,.70710678]
    assert statuses(result(e))['retention']=='FAIL'

def test_timestamp_domains_and_units_remain_separate():
    r=result();p=r['phases'][0]
    assert p['earliest_sim_s'] is None
    assert p['timestamps_by_clock']['source_stamp_ns']['earliest']==500000000
    assert p['timestamps_by_clock']['receive_wall_s']['earliest']==1.
    assert p['measurement_units']['uncertainty_m']=='m'


def test_clock_epoch_reset_is_not_sorted_into_success():
    e=fixture()+[dict(topic='/clock',sim_s=10,wall_s=1,data={'clock':{'sec':10,'nanosec':0}}),
                 dict(topic='/clock',sim_s=1,wall_s=2,data={'clock':{'sec':1,'nanosec':0}})]
    assert result(e)['result']=='UNSCORABLE'

def test_actual_consumer_witness_retains_exact_observation():
    e=fixture();o=observation(2)
    e.append(dict(topic='/diagnostics/manipulation_observation_acceptance',sim_s=2,wall_s=4,
                  data=dict(stamp_ns=2000000000,phase_started_ns=1,phase='approach',observation=o,accepted_by='ArmControl.target_height')))
    acceptance=result(e)['consumer_acceptance'][0]
    assert acceptance['contract_rechecked'];assert acceptance['observation']==o


def test_engineering_boundaries_do_not_manufacture_mission_verdict():
    from evaluation import evaluate_recording
    e=fixture()
    for name,t in [('control_started',.1),('control_finished',9)]:
        e.append(dict(topic='/diagnostics/manipulation_checkpoints',sim_s=t,wall_s=t*2,receive_monotonic_s=100+t*2,data={'checkpoint':name,'stamp_ns':round(t*1e9),'source_monotonic_s':100+t*2}))
    m=dict(MANIFEST,execution_class='NON_QUALIFYING',profile='rgbd_manipulation',home_xy_m=[0,0],targets=[dict(model='target_red',height=.16)])
    r=evaluate_recording(e,m,{})
    assert r['facts']['episode_finished'] is True
    assert r['oracle']['mission_verdict']=='UNOBSERVED'
    assert r['oracle']['oracle_verdict']!='PASS' # no robot/authority evidence
    # Production/Test A does not opt into the engineering boundary adapter.
    r=evaluate_recording(e,dict(m,execution_class='PRODUCTION'),{})
    assert r['facts']['episode_finished'] is False


def test_complete_synthetic_catalog_can_pass_without_relaxing_production_limits():
    e=complete_fixture()
    r=verify(e,MANIFEST,ORACLE,thresholds={'attachment_proximity_m':.03},limits=LIMITS)
    assert r['result']=='PASS';assert r['complete_invariant_verification']
    e[-2]['data']['position']=[.02,-.02]
    assert verify(e,MANIFEST,ORACLE,thresholds={'attachment_proximity_m':.03},limits=LIMITS)['result']=='FAIL'


def complete_fixture():
    e=fixture()
    e.append(dict(topic='/joint_states',sim_s=5.9,data=dict(header={'stamp':{'sec':5,'nanosec':900000000}},
                  name=['m_link3_Revolute-8','m_link3_Revolute-9'],position=[.5,-.5])))
    e.append(dict(topic='/diagnostics/manipulation_observation_acceptance',sim_s=2,
                  data=dict(stamp_ns=2000000000,phase_started_ns=1,phase='approach',
                            observation=observation(2),accepted_by='ArmControl.target_height')))
    return e


@pytest.mark.parametrize('fault',['missing','one_joint','stale','future','late_receipt','duplicate_names'])
def test_real_feedback_required(fault):
    e=complete_fixture();j=e[-2]
    if fault=='missing':e.remove(j)
    if fault=='one_joint':j['data']['name'].pop();j['data']['position'].pop()
    if fault=='stale':j['data']['header']['stamp']['sec']=1
    if fault=='future':j['data']['header']['stamp']['sec']=7
    if fault=='late_receipt':j['sim_s']=6.1
    if fault=='duplicate_names':j['data']['name'][1]=j['data']['name'][0]
    r=result(e,thresholds={'attachment_proximity_m':.03})
    assert r['finger_feedback']['status']=='UNVERIFIED';assert r['result']!='PASS'


def test_provenance_string_does_not_approve_engineering_limits():
    r=verify(complete_fixture(),dict(MANIFEST,evidence_class='HISTORICAL_REPLAY'),ORACLE,limits=LIMITS)
    assert statuses(r)['physical_attachment']=='UNVERIFIED'


def test_missing_consumer_and_disagreement_block_complete_success():
    e=complete_fixture()
    assert result(e[:-1],thresholds={'attachment_proximity_m':.03})['result']=='UNVERIFIED'
    assert result(e,oracle=dict(ORACLE,mission_oracle_disagreement=True),thresholds={'attachment_proximity_m':.03})['result']=='UNVERIFIED'


def test_unobserved_interval_after_release_cannot_verify_detachment():
    e=complete_fixture()
    for row in e:
        if row['topic']=='/eval/ground_truth/state' and row['sim_s']>=6:
            row['sim_s']+=10;row['data']['sim_s']+=10;row['data']['gripper']['source_sim_s']+=10
    assert statuses(result(e))['physical_release']=='UNVERIFIED'
