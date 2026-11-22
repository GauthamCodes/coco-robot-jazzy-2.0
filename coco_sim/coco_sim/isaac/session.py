# Copyright 2026 Gautham Anil
# SPDX-License-Identifier: Apache-2.0
"""Pure helpers for scoped session ownership and diagnostic classification."""
from pathlib import Path


def process_identity(pid, proc_root=Path('/proc')):
    """Include kernel start ticks so a reused PID is never mistaken for ours."""
    try:
        text = (proc_root / str(pid) / 'stat').read_text()
        fields = text[text.rindex(')') + 2 :].split()
        return {
            'pid': int(pid),
            'state': fields[0],
            'ppid': int(fields[1]),
            'pgid': int(fields[2]),
            'session': int(fields[3]),
            'start_ticks': int(fields[19]),
        }
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return None


def same_process(record, proc_root=Path('/proc')):
    current = process_identity(record['pid'], proc_root)
    return current is not None and current['start_ticks'] == record['start_ticks']


def session_members(session, proc_root=Path('/proc')):
    """Only the POSIX session created by this harness, never a name sweep."""
    members = []
    for entry in proc_root.iterdir():
        if entry.name.isdigit():
            identity = process_identity(int(entry.name), proc_root)
            if identity and identity['session'] == session and identity['state'] != 'Z':
                members.append(identity)
    return members


def nav_parameters(base, map_path, ack_ms):
    """Copy shared navigation settings; change only Isaac transport/map settings."""
    import copy

    result = copy.deepcopy(base)
    result['map_server']['ros__parameters']['yaml_filename'] = str(map_path)
    if ack_ms is not None:
        result['bt_navigator']['ros__parameters']['default_server_timeout'] = ack_ms
    return result


def classify_nav(log, fallback):
    if 'acknowledge goal request for compute_path_to_pose' in log:
        return 'planner failure'
    if 'acknowledge goal request for follow_path' in log:
        return 'controller failure'
    return fallback
