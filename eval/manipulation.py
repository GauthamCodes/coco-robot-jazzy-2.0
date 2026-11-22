"""Offline phase diagnostics. The existing oracle is the only mission oracle.

No ROS imports or live simulator access. Optional limits must carry provenance;
null defaults deliberately do not invent physical tolerances/calibration.
"""
import argparse
import json
import math
import sys
from pathlib import Path

from evidence import atomic_json, file_hash
from invariants import evaluate
from normalize import payload
from oracle import finite, xyz, quaternion, REST_POSITION_M, REST_WINDOW_SIM_S
from spatial import rotate
from manipulation_calibration import limits_usable
from manipulation_feedback import opening, contract

PHASES = ('baseline_observation', 'approach_observation', 'grasp_request',
          'plugin_attachment_report', 'physical_attachment', 'physical_lift',
          'retention', 'release_request', 'physical_release', 'post_release_stability')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'coco_config'))
from coco_config.object_observation import validated_height

DEFAULT_LIMITS = dict(relative_translation_m=None, relative_rotation_rad=None,
                      max_truth_gap_sim_s=None, provenance=None)


def relative(obj, grip):
    q = grip['quaternion_wxyz']; norm = math.sqrt(sum(x*x for x in q))
    inverse = [q[0]/norm] + [-x/norm for x in q[1:]]
    position = rotate(inverse, [a-b for a, b in zip(obj['xyz_m'], grip['xyz_m'])])
    a,b,c,d = inverse; e,f,g,h = obj['quaternion_wxyz']
    return position, [a*e-b*f-c*g-d*h, a*f+b*e+c*h-d*g,
                      a*g-b*h+c*e+d*f, a*h+b*g-c*f+d*e]


def angle(a, b):
    dot = abs(sum(x*y for x,y in zip(a,b))) / math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
    return 2*math.acos(min(1., dot))


def verify(events, manifest, oracle, trace=(), thresholds=None, limits=None, integrity_errors=()):
    from manipulation_tf import enrich
    malformed = any(not isinstance(e,dict) for e in events)
    events = enrich(events)
    thresholds = thresholds or {}
    limits = dict(DEFAULT_LIMITS, **(limits or {}))
    errors = list(integrity_errors) + (['Malformed event record'] if malformed else []); indexed = []; seen = set(); duplicates = 0
    for sequence, event in enumerate(events):
        if not isinstance(event,dict) or not finite(event.get('sim_s')) or not isinstance(event.get('topic'),str):
            errors.append(f'event {sequence}: missing/invalid ROS receive time'); continue
        data = payload(event)
        key = json.dumps([event.get('topic'), event['sim_s'], data], sort_keys=True)
        if key in seen: duplicates += 1; continue
        seen.add(key)
        indexed.append((sequence, event, data))
    # Source order is used for physical samples; arrival order remains in refs.
    indexed.sort(key=lambda v:(v[1]['sim_s'], v[0]))
    def ref(row):
        i,e,d = row
        return dict(file='logs/events.jsonl', receive_sequence=e.get('receive_sequence',i), decoded_sequence=i, topic=e['topic'],
                    receive_ros_s=e['sim_s'], mcap_receive_ns=e.get('mcap_receive_ns'), receive_wall_s=e.get('wall_s'),
                    receive_monotonic_s=e.get('receive_monotonic_s',e.get('monotonic_s')), receive_wall_epoch_ns=e.get('wall_epoch_ns'),
                    source_stamp_ns=d.get('stamp_ns') if isinstance(d,dict) else None,
                    source_sim_s=d.get('sim_s') if isinstance(d,dict) else None,
                    header=d.get('header') if isinstance(d,dict) else None,
                    gripper_provenance=(d.get('gripper') or {}).get('provenance') if isinstance(d,dict) and isinstance(d.get('gripper'),dict) else None,
                    gripper_tf_chain=(d.get('gripper') or {}).get('tf_chain') if isinstance(d,dict) and isinstance(d.get('gripper'),dict) else None)
    phases = {}
    def phase(name, status='UNVERIFIED', rows=(), measurements=None, required=(), missing=(), reason='', ids=()):
        refs = [ref(r) for r in rows]
        times = [r['source_sim_s'] for r in refs if finite(r['source_sim_s'])]
        clocks={}
        for clock in ('source_sim_s','source_stamp_ns','receive_ros_s','receive_wall_s','receive_monotonic_s','receive_wall_epoch_ns','mcap_receive_ns'):
            values=[r[clock] for r in refs if finite(r[clock])]
            clocks[clock]=dict(earliest=min(values),latest=max(values)) if values else None
        units={k:('m' if k.endswith('_m') else 'rad' if k.endswith('_rad') else 's' if k.endswith('_s') else 'boolean' if isinstance(v,bool) else 'unspecified') for k,v in (measurements or {}).items()}
        inv = {i:evaluate(i, measurements or {}, thresholds) for i in ids}
        phases[name] = dict(phase=name, status=status, earliest_sim_s=min(times) if times else None,
                           latest_sim_s=max(times) if times else None, timestamps_by_clock=clocks, evidence=refs,
                           present_evidence_topics=sorted({r['topic'] for r in refs}), measurement_units=units,
                           required_observations=list(required), missing_observations=list(missing),
                           measurements=measurements or {}, invariant_results=inv, reason=reason or ('Required evidence observed.' if status=='PASS' else 'Observed measurements violate the phase requirement.' if status=='FAIL' else 'Missing: '+', '.join(missing)),
                           uncertainty=dict(status='BOUNDED_BY_APPROVED_LIMITS' if limits_usable(limits,manifest) else 'UNCHARACTERIZED',
                                            translation_m=limits.get('relative_translation_m'), rotation_rad=limits.get('relative_rotation_rad'),
                                            explanation='Sampling and pose-error limits require calibration; no zero uncertainty assumed'),
                           limitations=['Sampled evidence only; gaps cannot establish continuous physical state.'])
        return phases[name]
    clock_rows=[]
    for row in indexed:
        d=row[2]
        if row[1]['topic']=='/clock' and isinstance(d,dict):
            c=d.get('clock',{})
            if isinstance(c,dict) and finite(c.get('sec')) and finite(c.get('nanosec')) and finite(row[1].get('wall_s')):
                clock_rows.append((row[1]['wall_s'],c['sec']+c['nanosec']/1e9))
    clock_rows.sort()
    if any(b[0]>a[0] and b[1]<a[1] for a,b in zip(clock_rows,clock_rows[1:])):
        errors.append('Simulation clock regressed across wall receipt order; epochs cannot be combined')
    checkpoints = [r for r in indexed if r[1]['topic']=='/diagnostics/manipulation_checkpoints' and isinstance(r[2],dict)]
    def marks(name): return [r for r in checkpoints if r[2].get('checkpoint')==name]
    observation_identity=None
    for name, marker in [('baseline_observation','observation_acquired'),('approach_observation','approach_complete')]:
        rows = marks(marker)
        # These are sensor-only driver acceptance witnesses, not ArmControl acceptance.
        valid=[]
        for r in rows:
            d=r[2];now=d.get('stamp_ns');o=d.get('observation');epoch=d.get('phase_started_ns')
            if not isinstance(now,int) or not isinstance(epoch,int) or not isinstance(o,dict):continue
            h=validated_height(o,now_ns=now,colour=manifest.get('requested_colour'))
            identity=(o.get('track_id'),o.get('reference_id'))
            if h is not None and o['stamp_ns']>epoch and d.get('observation_contract_valid') is True and (observation_identity is None or observation_identity==identity):
                valid.append(r);observation_identity=identity
        measurements={k:valid[-1][2]['observation'][k] for k in ('center_height_above_support_m','support_height_m','uncertainty_m')} if valid else {}
        phase(name, 'PASS' if valid else 'UNVERIFIED', rows, measurements,
              required=['fresh phase-consumer observation with source identity'],
              missing=[] if valid else ['fresh phase-consumer observation with source identity'],
              reason='Driver contract independently rechecked; actual ArmControl sample acceptance is not attested.')
    grasp = marks('grasp_request'); release = marks('release_request')
    if len(grasp)>1 or len(release)>1:errors.append('Multiple attempts require explicit per-attempt segmentation')
    # Raw magnet command is a separate assertion, never physical success.
    attaches = [r for r in indexed if r[1]['topic']==f"/magnet/target_{manifest.get('requested_colour')}/attach"]
    detaches = [r for r in indexed if r[1]['topic']==f"/magnet/target_{manifest.get('requested_colour')}/detach"]
    attaches=[r for r in attaches if grasp and r[1]['sim_s']>=grasp[0][1]['sim_s']]
    detaches=[r for r in detaches if release and r[1]['sim_s']>=release[0][1]['sim_s']]
    phase('grasp_request', 'PASS' if grasp and attaches else 'UNVERIFIED', grasp+attaches,
          required=['guarded pick request','magnet attach command'],
          missing=([] if grasp else ['guarded pick request'])+([] if attaches else ['magnet attach command']))
    phase('release_request', 'PASS' if release and detaches else 'UNVERIFIED', release+detaches,
          required=['guarded place request','magnet detach command'],
          missing=([] if release else ['guarded place request'])+([] if detaches else ['magnet detach command']))
    reports = [r for r in indexed if r[1]['topic']==f"/magnet/target_{manifest.get('requested_colour')}/state" and r[2]=='attached']
    phase('plugin_attachment_report','PASS' if reports else 'UNVERIFIED',reports,
          required=['plugin attached report'], missing=[] if reports else ['plugin attached report'],
          reason='Plugin report only; physical attachment evaluated separately.')
    consumer_acceptance=[]
    for row in indexed:
        if row[1]['topic']!='/diagnostics/manipulation_observation_acceptance' or not isinstance(row[2],dict):continue
        d=row[2];o=d.get('observation');now=d.get('stamp_ns');epoch=d.get('phase_started_ns')
        valid=isinstance(now,int) and isinstance(epoch,int) and validated_height(o,now_ns=now,colour=manifest.get('requested_colour')) is not None
        valid=bool(valid and o['stamp_ns']>epoch and d.get('accepted_by')=='ArmControl.target_height')
        consumer_acceptance.append(dict(evidence=ref(row),contract_rechecked=valid,phase=d.get('phase'),observation=o))
    target = 'target_'+manifest.get('requested_colour','')
    truth=[]; invalid=[]
    for row in indexed:
        _,e,d=row
        if e['topic']!='/eval/ground_truth/state': continue
        if not isinstance(d,dict) or not finite(d.get('sim_s')) or d.get('frame')!='world':
            invalid.append(ref(row));continue
        obj=d.get('objects',{}).get(target) if isinstance(d.get('objects',{}),dict) else None; grip=d.get('gripper')
        if obj is None:continue
        if not isinstance(obj,dict) or not xyz(obj.get('xyz_m')) or not quaternion(obj.get('quaternion_wxyz')):
            invalid.append(ref(row));continue
        if not isinstance(grip,dict) or not xyz(grip.get('xyz_m')) or not quaternion(grip.get('quaternion_wxyz')):
            continue
        # Pose pair MUST come from one source packet. No nearest-time TF lookup.
        if grip.get('source_sim_s')!=d['sim_s'] or grip.get('frame')!='world' or grip.get('provenance') not in ('simulator_link_pose','exact_recorded_tf_and_world_base'):
            invalid.append(ref(row));continue
        truth.append(row)
    truth.sort(key=lambda r:(r[2]['sim_s'],r[0]))
    times=[r[2]['sim_s'] for r in truth]
    if len(times)!=len(set(times)):errors.append('Conflicting truth packets at identical source time')
    calibrated=limits_usable(limits, manifest)
    if calibrated:
        if any(abs(r[1]['sim_s']-r[2]['sim_s'])>limits['max_truth_gap_sim_s'] for r in truth):
            errors.append('Truth source/ROS receipt mismatch or stale evidence')
    def stamp(rows):
        value=rows[0][2].get('stamp_ns') if rows else None
        return value/1e9 if isinstance(value,int) and value>0 else None
    start=attaches[0][1]['sim_s'] if attaches else stamp(grasp)
    end=detaches[0][1]['sim_s'] if detaches else stamp(release)
    baseline=[r for r in truth if start is not None and r[2]['sim_s']<=start]
    carry=[r for r in truth if start is not None and r[2]['sim_s']>start and (end is None or r[2]['sim_s']<end)]
    after=[r for r in truth if end is not None and r[2]['sim_s']>=end]
    def covered(rows):
        return calibrated and len(rows)>=2 and all(0<b[2]['sim_s']-a[2]['sim_s']<=limits['max_truth_gap_sim_s'] for a,b in zip(rows,rows[1:]))
    def motion(rows):
        pairs=[relative(r[2]['objects'][target],r[2]['gripper']) for r in rows]
        return dict(sample_count=len(rows), max_sample_gap_sim_s=max(b[2]['sim_s']-a[2]['sim_s'] for a,b in zip(rows,rows[1:])),
                    relative_translation_drift_m=max(math.dist(pairs[0][0],p[0]) for p in pairs),
                    relative_rotation_drift_rad=max(angle(pairs[0][1],p[1]) for p in pairs),
                    gripper_displacement_m=max(math.dist(rows[0][2]['gripper']['xyz_m'],r[2]['gripper']['xyz_m']) for r in rows),
                    target_displacement_m=max(math.dist(rows[0][2]['objects'][target]['xyz_m'],r[2]['objects'][target]['xyz_m']) for r in rows))
    missing=[]
    if not truth:missing.append('synchronized world target and gripper truth')
    if not calibrated:missing.append('approved relative-transform tolerances / sample-gap provenance')
    if baseline and calibrated and start-baseline[-1][2]['sim_s']>limits['max_truth_gap_sim_s']:
        baseline=[]
    if not baseline:missing.append('pre-grasp physical baseline')
    if len(carry)<2:missing.append('post-grasp motion samples')
    if calibrated and len(carry)>=2 and not covered(carry):missing.append('truth sampling gap exceeds configured bound')
    attachment='UNVERIFIED'; m=motion(carry) if len(carry)>=2 else {}
    # First motion pair establishes attachment; later drift belongs to retention.
    attach_rows=[]
    if calibrated and baseline and len(carry)>=2 and carry[0][2]['sim_s']-start<=limits['max_truth_gap_sim_s']:
        attach_rows=carry
        for count in range(2,len(carry)+1):
            if motion(carry[:count])['gripper_displacement_m']>2*limits['relative_translation_m']:
                attach_rows=carry[:count];break
        m=motion(attach_rows)
        if covered(attach_rows) and m['gripper_displacement_m']>2*limits['relative_translation_m']:
            attachment='PASS' if m['relative_translation_drift_m']<=limits['relative_translation_m'] and m['relative_rotation_drift_rad']<=limits['relative_rotation_rad'] else 'FAIL'
        else:missing.append('discriminating gripper movement')
    if baseline:
        b=baseline[-1][2]
        m['attachment_proximity_m']=math.dist(b['objects'][target]['xyz_m'],b['gripper']['xyz_m'])
    phase('physical_attachment',attachment,attach_rows or carry,m,
          required=['pre-grasp baseline','synchronized moving target/palm poses','approved rigidity tolerances'],missing=missing,
          reason='Target-to-palm rigidity tested independently of plugin state; proximity calibration remains separate.',ids=['I21'])
    lift={};lift_status='UNVERIFIED'
    if baseline and carry:
        lift['lift_height_m']=max(r[2]['objects'][target]['xyz_m'][2] for r in carry)-baseline[-1][2]['objects'][target]['xyz_m'][2]
        if attachment=='PASS' and covered(carry):
            lift_status='PASS' if lift['lift_height_m']>=contract()['LIFT_MIN_RISE'] else 'FAIL' if end is not None else 'UNVERIFIED'
    phase('physical_lift',lift_status,baseline[-1:]+carry,lift,required=['physical baseline','world target rise >=0.03m','physical attachment'],
          missing=[] if lift_status!='UNVERIFIED' else ['verified attachment and covered physical lift window'],ids=['I21'])
    retention='UNVERIFIED';rm=motion(carry) if len(carry)>=2 else {}
    if attachment=='PASS' and len(carry)>=3 and end is not None and covered(carry):
        rm=motion(carry)
        if end-carry[-1][2]['sim_s']<=limits['max_truth_gap_sim_s']:
            retention='PASS' if rm['relative_translation_drift_m']<=limits['relative_translation_m'] and rm['relative_rotation_drift_rad']<=limits['relative_rotation_rad'] else 'FAIL'
            rm['target_attached_during_carry']=retention=='PASS'
    phase('retention',retention,carry,rm,required=['covered moving carry until release'],
          missing=[] if retention!='UNVERIFIED' else ['complete independently witnessed carry'],ids=['I22'])
    released='UNVERIFIED';am=motion(after) if len(after)>=2 else {}
    if release and detaches and covered(after) and after[0][2]['sim_s']-end <= limits['max_truth_gap_sim_s']:
        am=motion(after)
        if am['gripper_displacement_m']>2*limits['relative_translation_m']:
            released=('PASS' if am['relative_translation_drift_m']>2*limits['relative_translation_m'] and am['target_displacement_m']<=REST_POSITION_M else 'FAIL' if am['relative_translation_drift_m']<=limits['relative_translation_m'] else 'UNVERIFIED')
            if released!='UNVERIFIED':am['placed_detached']=released=='PASS'
    phase('physical_release',released,after,am,required=['release request','post-release palm retreat and independent target separation'],
          missing=[] if released!='UNVERIFIED' else ['discriminating post-release motion'],ids=['I23'])
    stability='UNVERIFIED';sm={}
    if after:
        sm['placement_rest_window_sim_s']=after[-1][2]['sim_s']-after[0][2]['sim_s']
        sm['placement_rest_motion_m']=max(math.dist(a[2]['objects'][target]['xyz_m'],b[2]['objects'][target]['xyz_m']) for a in after for b in after)
        if covered(after) and sm['placement_rest_window_sim_s']>=REST_WINDOW_SIM_S:
            stability='FAIL' if sm['placement_rest_motion_m']>REST_POSITION_M else 'PASS' if released=='PASS' and oracle.get('checks',{}).get('target_at_rest') is True else 'UNVERIFIED'
    phase('post_release_stability',stability,after,sm,required=['detachment','2s terminal rest','oracle placement and orientation checks'],
          missing=[] if stability!='UNVERIFIED' else ['complete detached terminal rest/placement evidence'],ids=['I23'])
    phases['post_release_stability']['terminal_oracle']=oracle
    if invalid:errors.append('Invalid or mismatched physical truth packets')
    if errors:
        for p in phases.values():
            p['status']='UNSCORABLE';p['limitations']+=errors
    ordered=[phases[name] for name in PHASES]
    phase_sequence_result='UNSCORABLE' if errors else 'FAIL' if any(p['status']=='FAIL' for p in ordered) else 'PASS' if all(p['status']=='PASS' for p in ordered) and oracle.get('oracle_verdict')=='PASS' else 'UNVERIFIED'
    # Resolve the catalog's multi-phase conjunctions together, without inventing
    # a missing finger-opening witness from commands or plugin state.
    catalog_measurements={**m,**(lift if lift_status!='UNVERIFIED' else {}),**rm,**am,**(sm if stability!='UNVERIFIED' else {})}
    finger_feedback=opening(indexed,stamp(release),end,limits['max_truth_gap_sim_s'] if calibrated else None,ref)
    finger_witnesses=finger_feedback['samples']
    if finger_feedback['fingers_open_before_detach'] is not None:
        catalog_measurements['fingers_open_before_detach']=finger_feedback['fingers_open_before_detach']
    catalog_results={i:evaluate(i,catalog_measurements,thresholds) for i in ('I21','I22','I23')}
    complete_invariants=all(v['holds'] is True for v in catalog_results.values())
    result='UNVERIFIED' if phase_sequence_result=='PASS' and not complete_invariants else phase_sequence_result
    accepted_for_grasp=any(a['contract_rechecked'] and start is not None and
                           a['evidence']['source_stamp_ns']/1e9 <= start and
                           stamp(grasp) is not None and a['evidence']['source_stamp_ns']/1e9 >= stamp(grasp)
                           for a in consumer_acceptance)
    if result=='PASS' and (not accepted_for_grasp or oracle.get('mission_oracle_disagreement') is True):
        result='UNVERIFIED'
    if not errors and any(v['holds'] is False for v in catalog_results.values()):result='FAIL'
    return dict(schema_version=1,diagnostic_only=True,qualified=False,source_run=manifest.get('replay_source'),phases=ordered,
                result=result,phase_sequence_result=phase_sequence_result,consumer_acceptance=consumer_acceptance,oracle=oracle,limits=limits,
                complete_invariant_verification=complete_invariants,catalog_results=catalog_results,
                catalog_measurements=catalog_measurements,finger_witnesses=finger_witnesses,finger_feedback=finger_feedback,
                accepted_observation_consumed_for_grasp=accepted_for_grasp,duplicate_events=duplicates,
                input_events=len(events),invalid_truth=invalid,integrity_errors=errors,
                first_unsupported_phase=next((p['phase'] for p in ordered if p['status']!='PASS'),None),
                unresolved_catalog=[i for i,v in catalog_results.items() if v['holds'] is None],
                result_reason=('Evidence integrity errors' if errors else 'Observed phase or catalog failure' if result=='FAIL' else 'All phases, manipulation catalog and existing oracle pass' if result=='PASS' else 'Incomplete phase/catalog evidence or existing oracle not PASS'),
                analyzer_manipulation_evidence=[e for e in trace if set(e.get('invariants',[])) & {'I20','I21','I22','I23'}],
                limitations=['No replacement mission oracle. Plugin attachment is not physical evidence.',
                             'Missing calibrated limits remain null. Engineering/replay never qualifies M1.'])


def run(directory, output, limits=None):
    from replay import check_dev
    root=Path(directory);manifest=json.loads((root/'manifest.json').read_text());check_dev(manifest)
    events=[];errors=[];path=root/'logs/events.jsonl'
    if path.exists():
        with path.open() as stream:
            for n,line in enumerate(stream):
                try:events.append(json.loads(line))
                except ValueError:errors.append(f'truncated/invalid events.jsonl line {n+1}')
    else:errors.append('Missing events.jsonl')
    oracle={'oracle_verdict':'UNSCORABLE'}
    if (root/'oracle.json').exists():
        try:oracle=json.loads((root/'oracle.json').read_text())
        except ValueError:errors.append('Invalid oracle.json')
    trace=[]
    if (root/'logs/normalized.jsonl').exists():
        try:trace=[json.loads(l) for l in (root/'logs/normalized.jsonl').read_text().splitlines()]
        except ValueError:errors.append('Invalid normalized.jsonl')
    thresholds=json.loads(Path(__file__).with_name('thresholds.json').read_text())['thresholds']
    result=verify(events,manifest,oracle,trace,thresholds,limits,errors)
    result['source_hashes']={str(p.relative_to(root)):file_hash(p) for p in [root/'manifest.json',path,root/'oracle.json'] if p.exists()}
    result['limitations'].append('Sidecar analysis; use eval replay for MCAP integrity validation.')
    if Path(output).exists():raise FileExistsError(output)
    atomic_json(Path(output),result);return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--limits',type=Path,help='Explicit diagnostic limits JSON with provenance; never qualifies a run')
    a=p.parse_args();run(a.run,a.output,json.loads(a.limits.read_text()) if a.limits else None)
