# WEB_API — the `coco.v1` protocol

The wire contract between a browser (or any client) and `platform_server`.

Implementation: `coco_web/coco_web/protocol.py`. Tests:
`coco_web/test/test_protocol.py`, `coco_web/test/test_safety.py`. Where
this document and the code disagree, the code is right and this document
is a bug.

---

## Endpoints

| Method | Path | Purpose |
|---|---|---|
| `GET` | `/` | the control interface |
| `WS` | `/ws` | the protocol below |
| `GET` | `/healthz` | **200 only when ready**, else 503 with what is missing |
| `GET` | `/api/session` | the session document, for scripts and tests |
| `GET` | `http://<host>:8081/stream?topic=…` | MJPEG, served by `web_video_server` |

`/healthz` is the readiness contract. It is 503 while Gazebo is still
spawning, which is what makes Docker's `HEALTHCHECK` meaningful — see
`PRODUCT_ARCHITECTURE.md`.

---

## Versioning

`PROTOCOL_VERSION = "coco.v1"`, sent in every `welcome`.

Bump it when an existing field **changes meaning or disappears**. Do not
bump it to add a field: clients must tolerate unknown fields, and the
shipped UI does. A client may send its version in `hello`; a mismatch is
refused with `protocol_mismatch` rather than served, because silently
serving a client that expects a different contract is how a changed field
becomes a mystery bug in someone else's UI.

---

## The design rule

**A client names an intent. It never names a topic, a service, or a
message type.**

No frame in the schema has a field that could carry one. Extra keys are
**rejected, not ignored** — a client that thinks it is talking to
rosbridge gets a clear error instead of a silent no-op that leaves it
believing it just drove the robot.

```jsonc
// rosbridge. Under the old panel's bridge this published to the wheels.
// Here it does not parse.
{"op":"publish","topic":"/diff_drive_controller/cmd_vel","msg":{…}}
// -> {"type":"error","code":"unknown_type","message":"… this server
//     speaks coco.v1, not the rosbridge protocol"}
```

---

## Client → server

Every frame takes an optional `id` (string or integer), echoed in the
`ack` or `error` so a client can correlate.

| `type` | Fields | Effect |
|---|---|---|
| `hello` | `protocol?`, `client?` | handshake; refuses a version mismatch |
| `ping` | `t` | answered with `pong` carrying the same `t` |
| `drive` | `linear`, `angular` | one teleop velocity, **clamped** |
| `stop` | — | publishes an explicit zero |
| `set_mode` | `mode` | `teleop` \| `auto` \| `stop` |
| `select_target` | `colour` | validated against the mission's own table |
| `mission` | `action` | `start` \| `abort` |
| `nav_goal` | `x`, `y` | a map-frame goal |
| `set_arm` | `shoulder`, `elbow` | clamped to the URDF's limits |
| `set_gripper` | `grip` | clamped; the second joint mirrors |
| `subscribe` | `streams` | `telemetry` \| `lidar` \| `map` \| `path` |

### Modes

The browser says `teleop` / `auto` / `stop`. `cmd_vel_arbiter` latches
`teleop` / `nav` / `idle`. The translation lives in
`protocol.mode_to_arbiter`, so the UI stays in product language and the
ROS side keeps its own. A user should be able to think *"drive COCO up
the ramp"* without meeting either spelling.

### Limits

`MAX_LINEAR = 0.5 m/s`, `MAX_ANGULAR = 1.2 rad/s`. These bound what the
*browser* may ask for; the robot's real limits live in the controller and
the velocity smoother. Clamping happens inside `decode()`, so an
over-limit request is **accepted and clamped**, not refused — a stick
dragged past the pad should drive at full speed, not stop.

Arm and gripper limits come from `coco_config.joint_limits.ARM_LIMITS`,
which `coco_config`'s own tests check against the URDF. They are **not**
re-typed here or in the browser. The old panel hard-coded them in its
HTML and both had already drifted — its gripper slider said `0.05 .. 0.5`
where the URDF says `-0.35 .. 1.1`.

---

## Server → client

### `welcome` — first frame on every connection

```jsonc
{
  "type": "welcome",
  "protocol": "coco.v1",
  "session": { "id": "4cd684e5538d", "state": "degraded", … },
  "streams": { "camera": {"topic":"/camera/image_raw","port":8081,
                          "path":"/stream?…","encoding":"mjpeg"},
               "annotated": {…}, "depth": null },
  "limits":  { "linear":0.5, "angular":1.2,
               "colours":["red","green","blue","yellow"],
               "modes":["teleop","auto","stop"],
               "arm":{"shoulder":[-3.84,1.0],"elbow":[-1.6,1.6]},
               "gripper":[-0.35,1.1] },
  "commands": ["drive","hello","mission", … ]
}
```

`commands` is the server's own list of what it will parse, so a client
can feature-detect instead of guessing. A test asserts it matches the
schema exactly — the server may not advertise what it cannot accept.

The current `map` frame follows immediately, if one is known.

### `telemetry` — 10 Hz

```jsonc
{
  "type":"telemetry", "seq": 1234, "t": 1789815538.2,
  "robot":   { "pose":{"x":1.2,"y":-0.4,"z":0.0,"yaw":0.31},
               "velocity":{"linear":0.22,"angular":-0.05},
               "online":true },
  "mission": { "online":true, "state":"CLIMB", "active":true,
               "colour":"blue", "detail":null, "raw":"state=CLIMB …" },
  "nav":     { "online":true, "path":[[x,y],…], "active_source":"nav" },
  "sensors": { "lidar":{"angle_min":-3.14,"angle_step":0.026,
                        "ranges":[1.23,null,…],"floor":0.15},
               "perception":{"online":true,"found":false,"seen":"green",…},
               "streams":{…} },
  "platform":{ "session":{…}, "arbiter":{"mode":"nav","active":"nav",
                                         "ages":{…},"raw":"…"} }
}
```

`seq` is monotonic per connection so a client can detect its own dropped
frames. The server **never re-sends**: stale telemetry is worse than a
gap.

`raw` carries the original `key=value` status line from the arbiter, the
executive and the perception node. Those lines are emitted in that shape
precisely so a reader needs no parser, and the old panel printed them
verbatim. That is fine for a debugging dashboard and wrong for a product,
so the fields are parsed into meaning — and `raw` is kept anyway, because
when something is wrong the raw line is what an engineer wants.

`arbiter.active` is **which source currently owns the wheels**. `null`
means the arbiter is holding them still. This is the field to watch if
the robot is not moving.

`lidar.ranges` is downsampled server-side to ≤ 240 points, taking the
**minimum** of each bucket — for an obstacle picture the nearest return
is the one that matters, and averaging it against the empty space beside
it is how a thin obstacle vanishes from the plot while still being there.
`null` is a no-return, drawn as a gap rather than a false wall at max
range. `floor` is 0.15 m, the LiDAR's own floor: a 0.15 reading means "at
or below", not "0.15 m away". C2-NAV.49 measured that saturation, which
is why `min_scan_m` is useless as a clearance metric.

### `map` — on connect, then on change only

```jsonc
{"type":"map","width":254,"height":199,"resolution":0.05,
 "origin":{"x":-6.1,"y":-5.2},"data":"<base64 of raw int8 cells>"}
```

Not in the telemetry tick. A 254×199 grid is ~50 000 cells; re-sending it
ten times a second to redraw a picture that does not move is a cost that
only shows up on someone's phone. `-1` (unknown) arrives as byte 255.

### `ack`, `error`, `pong`

```jsonc
{"type":"ack","id":2,"command":"drive","ok":true}
{"type":"error","id":6,"code":"unexpected_fields","message":"…"}
{"type":"pong","t":1234.5}
```

### Error codes

Stable slugs; a UI may branch on them.

| Code | Meaning |
|---|---|
| `bad_json`, `bad_frame`, `no_type`, `bad_id` | not a well-formed frame |
| `unknown_type` | not a `coco.v1` command — including every rosbridge op |
| `unexpected_fields` | a known command carrying a field it does not accept |
| `protocol_mismatch` | the client named a different version |
| `bad_velocity`, `bad_goal`, `bad_arm`, `bad_grip` | non-numeric, NaN or infinite |
| `bad_mode`, `bad_colour`, `bad_action`, `unknown_stream` | outside the allowed set |
| `refused` | well-formed, but the robot would not do it |
| `command_failed` | the server raised while acting on it |

Non-finite floats are scrubbed from every outbound frame. `json.dumps`
emits bare `NaN` by default, which is not JSON and which `JSON.parse`
rejects — so one bad range reading would otherwise cost the client the
whole frame, mission state included.

---

## What the server may touch

Everything, exhaustively. `coco_web/coco_web/safety.py` is the list, and
`assert_publish_safe()` checks it against the wheel topics at node
construction.

**Publishes**

| Intent | Topic | Type |
|---|---|---|
| `drive` | `/cmd_vel_teleop` | `geometry_msgs/TwistStamped` |
| `set_mode` | `/mission/mode` | `std_msgs/String` |
| `select_target` | `/mission/target_colour` | `std_msgs/String` |
| `nav_goal` | `/goal_pose` | `geometry_msgs/PoseStamped` |
| `set_arm` | `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` |
| `set_gripper` | `/gripper_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` |

**Calls**: `/mission/start`, `/mission/abort` (both `std_srvs/Trigger`).

**Never publishes**: `/diff_drive_controller/cmd_vel`, `/cmd_vel`,
`/cmd_vel_smoothed`, `/cmd_vel_nav`. The first is the arbiter's output
and the real wheel topic; the rest are lanes upstream of it. A node
configured to publish any of them **refuses to start**.

Verified live: with the server running, `ros2 node info
/coco_web_platform` lists `/cmd_vel_teleop` as its only velocity
publisher, and `/diff_drive_controller/cmd_vel` does not exist on the
graph at all.

---

## Connection lifecycle

1. Client opens `/ws`. Server sends `welcome`, then `map` if known.
2. Client may send `hello` to assert a version.
3. Telemetry flows at 10 Hz until close.
4. Client sends `drive` continuously (~10 Hz) while a stick or key is
   held, and an explicit `stop` on release.
5. On close, the client is detached. **If it was the last one, the server
   publishes a zero velocity.**

### Watchdogs, and why there are three

| Where | Triggers | Timeout |
|---|---|---|
| browser | release, blur, tab hidden, `Space` | immediate, 3 explicit zeros |
| `platform_server` | `drive` frames stop arriving | 0.5 s |
| `cmd_vel_arbiter` | the teleop source goes stale | 0.3 s |
| `diff_drive_controller` | no command at all | 0.5 s |

A client that stops sending has merely gone quiet; a client that sends
`stop` has decided. Both end with the robot stationary, but only one does
so immediately — which is why the browser sends explicit zeros rather
than just ceasing, and sends three, so a single dropped frame cannot
strand the robot at speed. The controller's own watchdog is the backstop,
not the plan.

### Reconnect

The shipped client reconnects with exponential backoff from 500 ms to a
10 s ceiling. The old panel retried at a flat 2 s forever; a tab left
open overnight against a stopped server should not spend the night
reconnecting.

---

## Talking to it without a browser

```python
import json
from tornado.ioloop import IOLoop
from tornado.websocket import websocket_connect

async def main():
    ws = await websocket_connect('ws://localhost:8080/ws')
    print(json.loads(await ws.read_message())['protocol'])   # coco.v1
    ws.write_message(json.dumps({'type': 'select_target', 'colour': 'blue'}))
    ws.write_message(json.dumps({'type': 'mission', 'action': 'start'}))
    while True:
        frame = json.loads(await ws.read_message())
        if frame['type'] == 'telemetry':
            print(frame['mission']['state'])

IOLoop.current().run_sync(main)
```

```bash
curl -s localhost:8080/healthz | head -20      # 200 only when ready
curl -s localhost:8080/api/session
```

---

## Security

P0.1 has **no authentication**, accepts **any origin**, and binds
`0.0.0.0`. That is a deliberate, recorded decision for a single-user
local appliance — see the security section of
`PRODUCT_ARCHITECTURE.md`, which also says what must change before this
is exposed beyond a trusted LAN, and why the closed command vocabulary is
what makes that change tractable.

`bind:=127.0.0.1` refuses the LAN today if you want that.
