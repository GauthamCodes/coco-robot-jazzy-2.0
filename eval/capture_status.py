"""Honest capture lifecycle summary; file existence alone is not a trace."""
import json


def summarize(root, state, reasons=()):
    ready=(root/'logs/recorder_ready.json').exists()
    path=root/'logs/events.jsonl'; count=0; errors=[]; terminal=False
    if path.exists():
        for n,line in enumerate(path.open(),1):
            try:
                e=json.loads(line); count+=1
                from normalize import payload
                d=payload(e)
                terminal |= (e.get('topic')=='/diagnostics/manipulation_checkpoints' and
                             isinstance(d,dict) and d.get('checkpoint')=='control_finished')
            except (ValueError,TypeError,AttributeError):
                errors.append(f'Invalid event line {n}')
    if not ready:errors.append('Recorder never acknowledged readiness')
    if count==0:errors.append('No recorded events; not a mission trace')
    if state.startswith(('ABORTED','BLOCKED')):errors.extend(reasons or [state])
    if not terminal:errors.append('No recorded control_finished checkpoint')
    return dict(recorder_ready=ready,event_count=count,has_trace=count>0,
                control_finished_recorded=terminal,capture_complete=not errors,
                incomplete_reasons=errors,runner_state=state,
                integrity='MCAP integrity must be validated by eval replay')
