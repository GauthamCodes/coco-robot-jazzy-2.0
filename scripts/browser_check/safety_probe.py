# Copyright 2026 Gautham Anil -- Apache-2.0.
"""
Try to reach ROS through the live platform socket, and record the replies.

    python3 safety_probe.py ws://127.0.0.1:8080/ws

Each attempt is something a hostile or confused client might send. Every
one must come back as an error and publish nothing; the orchestrating
script checks the wheel topic's publisher list before and after.
"""

import asyncio
import json
import sys

from tornado.websocket import websocket_connect

ATTEMPTS = [
    ('rosbridge_publish_to_wheels',
     {'op': 'publish', 'topic': '/diff_drive_controller/cmd_vel',
      'msg': {'linear': {'x': 0.5}, 'angular': {'z': 0.0}}}),
    ('rosbridge_advertise',
     {'op': 'advertise', 'topic': '/cmd_vel', 'type': 'geometry_msgs/Twist'}),
    ('rosbridge_call_service',
     {'op': 'call_service', 'service': '/mission/start'}),
    ('drive_with_topic_field',
     {'type': 'drive', 'id': 4, 'linear': 0.3, 'angular': 0.0,
      'topic': '/diff_drive_controller/cmd_vel'}),
    ('drive_with_target_field',
     {'type': 'drive', 'id': 5, 'linear': 0.3, 'angular': 0.0,
      'target': 'wheels'}),
    ('subscribe_to_a_topic_name',
     {'type': 'subscribe', 'id': 6, 'streams': ['/scan']}),
    ('unknown_command_publish',
     {'type': 'publish', 'id': 7, 'topic': '/cmd_vel', 'data': {}}),
    ('mission_with_service_field',
     {'type': 'mission', 'id': 8, 'action': 'start',
      'service': '/mission/start'}),
]


async def main(url):
    ws = await websocket_connect(url)
    welcome = json.loads(await ws.read_message())
    results = {'protocol': welcome.get('protocol'),
               'commands': welcome.get('commands'), 'attempts': []}
    for name, frame in ATTEMPTS:
        ws.write_message(json.dumps(frame))
        while True:
            raw = await asyncio.wait_for(ws.read_message(), 5)
            if isinstance(raw, bytes):
                continue
            reply = json.loads(raw)
            if reply.get('type') in ('error', 'ack'):
                break
        results['attempts'].append({'attempt': name, 'reply': reply})
    ws.close()
    results['all_refused'] = all(
        a['reply'].get('type') == 'error' for a in results['attempts'])
    print(json.dumps(results, indent=1))


asyncio.run(main(sys.argv[1]))
