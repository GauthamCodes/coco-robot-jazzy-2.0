# WEB_API — the `coco.v1` protocol

The wire contract between a browser (or any client) and `platform_server`.

Implementation: `coco_web/coco_web/protocol.py`, `streams.py`, `binary.py`,
`session.py`, `platform_server.py`.
Tests: `coco_web/test/test_protocol.py`, `test_streams.py`,
`test_binary.py`, `test_safety.py`, `test_session.py`, and
`test_platform_server.py` — the last runs the real server over real
WebSockets. Where this document and the code disagree, the code is right
and this document is a bug.

---

## Locked decisions (P0.2)

These were decided before P0.2 was built and are not reopened here.

| # | Decision | Where it is enforced |
|---|---|---|
| 1 | The protocol stays **`coco.v1`**. No `coco.v2`. | `protocol.PROTOCOL_VERSION`; every P0.2 change is additive |
| 2 | **Explicit per-client subscriptions.** The default set is P0.1's useful telemetry; **camera and depth need an explicit `subscribe`**; **no ROS topic name appears in the public protocol** | `streams.Subscription`; `test_platform_server.py::test_no_topic_name_appears_in_any_frame_a_client_receives` |
| 3 | Camera: **binary WebSocket is primary; MJPEG stays available** | `binary.py`; `platform.launch.py video:=true` keeps `web_video_server` |
| 4 | Session lifecycle and health are **two separate axes**: lifecycle `CREATED STARTING READY RUNNING STOPPING STOPPED FAILED`; health `HEALTHY DEGRADED UNHEALTHY` | `session.LIFECYCLE_STATES`, `session.HEALTH_STATES`; a test asserts they share no value |
| 5 | Depth is **browser visualisation only**. Nav2 depth fusion stays **OFF** | `platform.launch.py` includes no Nav2 or depth-cloud launch; a test builds it and checks |
| 6 | **Single user, single session.** No multi-user scheduling | `session.SessionRegistry.max_sessions = 1`, enforced |

**No public message may contain a field that lets a browser specify a ROS
topic, a ROS service, or an arbitrary command target.** Extra fields are
rejected, and a live probe (`scripts/browser_check/safety_probe.py`) sends
rosbridge `publish`/`advertise`/`call_service`, a `drive` with a `topic`
or `target` field, a topic as a stream name, and a `mission` with a
`service` field — all eight are refused.

---

## Endpoints

| Method | Path | Purpose |
|---|---|---|
| `GET` | `/` | the control interface |
| `WS` | `/ws` | the protocol below |
| `GET` | `/healthz` | **200 only when ready**, else 503 with what is missing |
| `GET` | `/api/session` | the session document, for scripts and tests |
| `GET` | `/api/metrics` | measured rates, drops, CPU — see *Performance* |
| `GET` | `http://<host>:8081/stream?topic=…` | MJPEG, if `web_video_server` was started |

`/healthz` is the readiness contract. It is 503 while Gazebo is still
spawning, which is what makes Docker's `HEALTHCHECK` meaningful — see
`PRODUCT_ARCHITECTURE.md`.

> **`/healthz` 200 does not mean "localised".** It means every *required*
> component is up. AMCL may still have no pose, and a mission started in
> that window fails immediately with `NAVIGATION_FAILED` and
> `bt_navigator` logging *"Initial robot pose is not available"*. A client
> that starts missions should wait for `nav.online` in telemetry, not
> just for `/healthz`.

---

## Versioning

`PROTOCOL_VERSION = "coco.v1"`, sent in every `welcome`.

Bump it when an existing field **changes meaning or disappears**. Do not
bump it to add a field: clients must tolerate unknown fields, and the
shipped UI does. A client may send its version in `hello`; a mismatch is
refused with `protocol_mismatch`.

**P0.2 did not bump it, and that was a constraint on the design rather
than a discovery.** Subscriptions are honoured now, but the default set
is exactly what P0.1 sent, so a client written against P0.1 that never
sends `subscribe` sees no change. Binary frames are opt-in through
`hello.binary`, so a client that cannot parse one never receives one.
Everything else added is a new frame type or a new field.

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
// -> {"type":"error","code":"no_type","message":"frame needs a string \"type\""}
```

*(Verified against the running server — that is the literal reply.)*

**One bounded exception, stated rather than hidden.** The MJPEG
descriptor in `welcome.streams` contains a topic string, because a
`web_video_server` URL requires one, and locked decision 3 keeps MJPEG.
It is display-only, on a different server, and there is no path from it
to publishing. Nothing in the binary sensor path names a topic, and a
test asserts that.

**A leak closed in P0.2's second pass.** The session's component
`detail` strings rode every telemetry frame and named topics
(`/diff_drive_controller/odom`, `/mission/state`, `/plan or /amcl_pose`),
and a mission refusal said `/mission/start is not available`. Both now
say what the evidence *is* in words ("the wheel controller is reporting
odometry", "cannot start the mission: the mission executive is not
running"). The topic behind each word is `platform_server.
COMPONENT_EVIDENCE`, for engineers; it is never sent.

---

## Client → server

Every frame takes an optional `id` (string or integer), echoed in the
`ack` or `error` so a client can correlate.

| `type` | Fields | Effect |
|---|---|---|
| `hello` | `protocol?`, `client?`, `binary?` | handshake; `binary` declares frame support |
| `ping` | `t` | answered with `pong` carrying the same `t` |
| `drive` | `linear`, `angular` | one teleop velocity, **clamped** |
| `stop` | — | publishes an explicit zero; **always honoured** |
| `set_mode` | `mode` | `teleop` \| `auto` \| `stop` |
| `select_target` | `colour` | validated against the mission's own table |
| `mission` | `action` | `start` \| `abort` |
| `nav_goal` | `x`, `y` | a map-frame goal |
| `set_arm` | `shoulder`, `elbow` | clamped to the URDF's limits |
| `set_gripper` | `grip` | clamped; the second joint mirrors |
| `subscribe` | `streams` | add streams; unknown names refused |
| `unsubscribe` | `streams` | drop streams; `telemetry` cannot be dropped |
| `set_stream` | `stream`, `fps?`, `quality?`, `scale?` | clamped, not refused |

### Modes

The browser says `teleop` / `auto` / `stop`. `cmd_vel_arbiter` latches
`teleop` / `nav` / `idle`. The translation lives in
`protocol.mode_to_arbiter`, so the UI stays in product language.

### Limits

`MAX_LINEAR = 0.5 m/s`, `MAX_ANGULAR = 1.2 rad/s`, clamped inside
`decode()`. Arm and gripper limits come from
`coco_config.joint_limits.ARM_LIMITS` and are **not** re-typed here or in
the browser.

### Who may drive

Multiple clients are supported — the phone in your hand and the laptop on
the desk. **Exactly one holds the stick.** The first client to send
`drive` claims it; another client's `drive` is refused with
`not_in_control` until the holder disconnects or goes quiet for
`DRIVE_TIMEOUT_S` (0.5 s), so a backgrounded tab cannot lock anyone out.

**`stop` is honoured from any client, always, held stick or not.** A
safety control that depends on who is in control is not a safety
control, and the person who can see the robot about to hit something may
be the one watching rather than the one driving.

---

## Subscriptions

```jsonc
{"type":"subscribe","streams":["camera"]}
{"type":"unsubscribe","streams":["depth"]}
```

| Stream | Carried as | Default |
|---|---|---|
| `telemetry` | the 10 Hz JSON tick | **on** (cannot be turned off) |
| `mission` | the `mission` block of that tick | **on** |
| `lidar` | JSON in the tick, **or** a binary frame | **on** |
| `map` | its own JSON frame, on change | **on** |
| `path` | the `nav.path` array of the tick | **on** |
| `camera` | binary frame only | off |
| `depth` | binary frame only | off |

The default set is the P0.1 set, deliberately. Unknown names are
**refused**, not ignored: a client asking for `cameras` and given nothing
concludes the camera is broken and goes looking in the wrong place.

`telemetry` cannot be unsubscribed — it carries the session state and
readiness that tell a user *why* nothing is happening, and a client
without it has no way to discover that it is broken.

**`lidar` has two forms.** A client that declared `binary: true` gets the
compact frame and the JSON `sensors.lidar` block is set to `null`, so the
scan is never sent twice. A client that did not gets the JSON block and
**never** a binary frame.

`camera` and `depth` create their ROS subscriptions only while at least
one client is watching, and destroy them when the last leaves. Nothing
decodes or re-encodes for an empty room.

**Depth is on for display, off for navigation — two different things.**
The depth *image* (`/camera/depth/image_raw`, 32FC1, which the gz bridge
always publishes and `target_finder` already reads) is what the browser
shows; `depth_topic` now defaults to it, and its ROS subscription exists
only while a client is subscribed to `depth`. Depth *fusion* —
`nav.launch.py depth_cloud:=true`, a PointCloud2 for the costmaps — is a
different switch in a different package, stays **off**, and nothing in
`coco_web` can start it. P0.2's first pass defaulted `depth_topic` to
empty, treating the picture as though it were the fusion, so the depth
pane could never show anything unless someone knew the parameter.
`depth_topic:=''` still disables the stream entirely.

### Negotiating a stream

```jsonc
{"type":"set_stream","stream":"camera","fps":5,"quality":40,"scale":0.5}
```

Clamped to the server's bounds rather than refused, the same rule
velocity follows. Camera: 1–15 fps (default 10), quality 10–95 (default
60), scale 0.25–1.0. Depth: the same, default 5 fps. The cap is the
sensor's own 15 Hz — advertising more would only be a promise to drop.

**The frame is encoded once for every viewer.** Where clients disagree,
the more demanding setting wins. Per-client encoding would cost a JPEG
per viewer to save bandwidth nobody is short of on a LAN. This is a real
trade and it is written down rather than discovered.

---

## Binary frames

Sensor payloads move as **binary** WebSocket messages; control and
metadata stay JSON text. The split is along a real seam: text frames are
things a person might read, binary frames are things a machine decodes.

```
off  size  field
0    4     magic  b'COCO'
4    1     format version (1)
5    1     stream id  (1=lidar, 2=camera, 3=depth)
6    2     header length, BIG-endian uint16
8    H     UTF-8 JSON header
8+H  ...   payload
```

Big-endian because `DataView.getUint16(offset)` is big-endian by default:
the browser reads it with no flag and no comment explaining a flag.

**Every frame is self-describing.** The JSON header always carries
`stream`, `seq`, `t` and `dropped`, plus whatever that stream needs:

```jsonc
// lidar — payload is uint16 big-endian millimetres, 0 = no return
{"stream":"lidar","seq":394,"t":1789841377.5,"count":240,
 "angle_min":-2.0944,"angle_step":0.01749,"scale":1000.0,
 "floor":0.15,"no_return":0,"dropped":0}

// camera — payload is JPEG bytes
{"stream":"camera","seq":88,"t":1789841383.6,"w":320,"h":240,
 "enc":"jpeg","quality":60,"dropped":0}

// depth — payload is a greyscale JPEG; near is bright
{"stream":"depth","seq":12,"t":1789841390.1,"w":320,"h":240,
 "enc":"jpeg","min_m":0.1,"max_m":8.0,
 "palette":"grey_near_bright","dropped":0}
```

`dropped` rides in the header so loss is **visible** rather than silent:
a client that missed frames can say so instead of showing stale data as
though it were current.

**The payload is never a serialized ROS message.** It is always something
`binary.py` constructed. Putting CDR on a public socket would make the
browser a ROS client, which is the boundary the closed command vocabulary
exists to hold.

### Malformed frames

`binary.decode_frame()` is the reference reader and every failure below
is a test: `bad_magic`, `short_frame`, `bad_version`, `truncated_header`,
`header_too_large`, `bad_header_encoding`, `bad_header_json`,
`bad_header`, `unknown_stream`. The header-length field indexes into the
buffer, so it is checked *before* it slices.

**Hardened in P0.2's second pass (Codex, integrated).** Every header now
carries `payload_bytes`, the exact payload length, and LiDAR headers name
their encoding (`"enc":"uint16be-mm"`); both are additive, and a frame
without `payload_bytes` (the first pass's) is still read, its JPEG end
marker catching truncation. The reader — and the **encoder**, on the way
out — also refuse: duplicate header keys (`bad_header_json`), a stream
name disagreeing with the prefix id, non-integer or out-of-range
`seq`/`dropped`/`w`/`h`/`quality`, a non-finite or negative timestamp, a
depth range that is not `0 ≤ min_m < max_m`, a LiDAR count that does not
match the payload, any `topic`/`service`/`message_type`/`target` key
(`bad_metadata`), a length mismatch (`bad_payload_length`), and anything
over 8 MiB (`payload_too_large`, `bad_buffer`).

Because the encoder validates, a frame it refuses is **skipped for that
stream only**: `platform_server` builds each stream's frame separately, so
one bad depth image cannot cost the same tick's LiDAR. A test sends an
inverted depth range and still receives the scan.

### Measured

| | |
|---|---|
| LiDAR frame | **668.8 bytes** mean (240 points), **10.0 Hz** |
| Camera frame | **3 467 bytes** mean JPEG at quality 60, 320×240 |
| Camera rate | **6.17 fps** observed under a 10 fps cap |
| Depth rate | **3.8 fps** observed under a 5 fps cap |
| Drops | **0** on every stream, in every probe |

A 240-point scan as JSON is roughly 2 kB; a test asserts the binary form
is less than half that.

**Re-measured in P0.2's second pass, through a real browser**, during two
complete missions (sim real-time factor ≈ 0.4, so ROS-side rates read
lower than their sim-time values): LiDAR frame **669.5 B**; camera JPEG
**3 572 B**; camera out **6.4–7.0 fps**; telemetry JSON **4.1 kB** per
frame at 10 Hz; **≤ 66 kB/s** in total to one browser; first camera frame
**0.34–0.48 s** after the subscribe click, first depth frame
**0.48–0.68 s**; **0 drops**, peak socket buffer **0 B**. A mission
transition reaches the page's DOM **62.7–74.9 ms** after the ROS message
(in-page MutationObserver, all 15 transitions of a fetch).

---

## Server → client

### `welcome` — first frame on every connection

```jsonc
{
  "type": "welcome",
  "protocol": "coco.v1",
  "session": { "id": "5efea1753235", "state": "ready",
               "lifecycle": "READY", "connection": "CONNECTED", … },
  "streams": { "camera": {"topic":"/camera/image_raw","port":8081,…},
               "annotated": {…}, "depth": null },
  "subscriptions": { "streams":["telemetry","mission","lidar","map","path"],
                     "available":[…], "default":[…],
                     "binary":false, "binary_streams":["camera","depth"],
                     "binary_capable":["lidar","camera","depth"],
                     "config":{…}, "sent":{…}, "dropped":{…} },
  "world":   { "frame":"map", "offset_x":2.0,
               "ramp":{"x0":3.0,"x1":5.0,"width":2.5},
               "platform":{"x0":5.0,"x1":6.5,"width":2.5},
               "home":{"x":0.0,"y":0.0},
               "targets":[{"colour":"red","x":6.05,"y":-0.75,…}, …] },
  "limits":  { "linear":0.5, "angular":1.2, "colours":[…], "modes":[…],
               "arm":{…}, "gripper":[…] },
  "commands": ["drive","hello","mission", … ]
}
```

`commands` is the server's own list of what it will parse. A test asserts
it matches the schema exactly — the server may not advertise what it
cannot accept, and a second test asserts the shipped page sends nothing
outside it.

**`world`** is the simulation's geometry in **map** coordinates, from
`coco_config`, so the page re-types no numbers and knows no frame offset.
The shift is `-SPAWN_XY[0]`, derived rather than copied —
`mission_states` derives `WORLD_TO_MAP_X` the same way from the same
config, and a test pins that expression. Drawing the ramp two metres out
looks exactly like broken localisation.

Note the two meanings of "streams": `streams` is MJPEG descriptors,
`subscriptions` is the WebSocket stream document. The older name is
load-bearing for existing clients, so both are kept.

### `telemetry` — 10 Hz

```jsonc
{
  "type":"telemetry", "seq": 1234, "t": 1789815538.2,
  "robot":   { "pose":{"x":1.2,"y":-0.4,"z":0.0,"yaw":0.31},
               "frame":"map", "localised":true,
               "velocity":{"linear":0.22,"angular":-0.05},
               "online":true },
  "mission": { …see below… },
  "nav":     { "online":true, "path":[[x,y],…], "active_source":"nav" },
  "sensors": { "lidar":{…} | null,
               "perception":{"online":true,"found":false,"seen":"green",…},
               "grasp":{"phase":"pick:hover above target",…},
               "streams":{…} },
  "platform":{ "session":{…}, "arbiter":{…}, "perf":{…},
               "connection":"CONNECTED", "pilot":"7f2a1c" }
}
```

`seq` is monotonic per connection so a client can detect its own dropped
frames. The server **never re-sends**: stale telemetry is worse than a gap.

**`robot.pose` is in the map frame** once `robot.localised` is true —
the frame the map, the plan and the world geometry are drawn in. Each
AMCL pose fixes the map→odom correction (against the odometry sample
nearest its stamp) and every odometry update is reported through it, so
the pose moves at odometry rate but does not drift. Before the first AMCL
pose, `frame` is `odom` and `localised` is false; that coincides with the
map only at spawn. P0.1 and P0.2's first pass sent whichever pose arrived
last — nearly always odometry — and after a ramp climb a robot verified
home was drawn at (0.61, 3.71), outside the arena. `frame` and
`localised` are new, additive fields.

A section the client did not subscribe to is `null` (or `[]` for `path`),
which is already its meaning before the first message arrives — so a
client needs no new branch.

### The mission block

This is the whole of the executive's state, translated.

```jsonc
"mission": {
  "online": true,
  "phase": "NAVIGATING",          // the platform's ten-word vocabulary
  "state": "CLIMB",               // the executive's own name
  "previous": "ALIGN_FOR_CLIMB",
  "words": "Climbing the ramp",   // operator wording, for Play mode
  "known": true,                  // false: a state this build never heard of
  "active": true,                 // may the UI offer Start?
  "recovering": false,
  "colour": "green",
  "reason": null,                 // one of 48 structured failure codes
  "reason_known": true,           // false: an unrecognised code
  "reason_words": null,           // plain English, where it exists
  "result": null,                 // fetch | traverse | aborted
  "owner": "ramp_driver",
  "mode": "rl",
  "event": "enter",               // enter = a real transition
  "step": 5, "steps": 16,         // the executive's own ordinal
  "elapsed": 12.3, "timeout": 180.0,
  "attempt": 1, "retries": 0,
  "changed_at": 1789815526.0,
  "detail": null,                 // alias of `reason`, for P0.1 clients
  "raw": "state=CLIMB prev=… "
}
```

**Phases**: `IDLE`, `STARTING`, `SEARCHING`, `APPROACHING`, `GRASPING`,
`NAVIGATING`, `RETURNING`, `COMPLETED`, `FAILED`, `STOPPED`.

- `ABORT` with `reason=OPERATOR_ABORT` is **STOPPED**, not FAILED.
  Reporting someone's own decision back to them in red is wrong.
- `RECOVERY` and `RELOCALIZE` keep the phase of the state they are
  retrying and set `recovering`. "Recovering" alone loses which leg.
- `step` is `null` for states that are not on the nominal chain
  (`RECOVERY`, `RELOCALIZE`, `ABORT`). They are states, not positions.

**There is no percentage and no ETA, deliberately.**
`mission_executive.py:603` sends its Nav2 goal with **no
`feedback_callback`**, so distance-remaining is not published on any
topic and progress *within* a leg is unobservable. P0.1's browser
interpolated one from a hard-coded fifteen-entry list; that is gone, and
a test asserts it stays gone. What is reported instead is the step
number, and the state's own `elapsed`/`timeout`.

`colour` comes from `/mission/target_colour`. P0.1 looked for it on
`/mission/state`, which has never carried one.

### `map` — on connect, then on change only

```jsonc
{"type":"map","width":243,"height":175,"resolution":0.05,
 "origin":{"x":-2.119,"y":-4.910},"data":"<base64 of raw int8 cells>"}
```

`-1` (unknown) arrives as byte 255. Subscribing to `map` mid-session
delivers the current grid immediately — for a static map, "on change" is
otherwise never.

### `subscription` — after a subscribe, unsubscribe or set_stream

Echoes the client's current stream set and per-stream config.

### `ack`, `error`, `pong`

```jsonc
{"type":"ack","id":2,"command":"drive","ok":true}
{"type":"error","id":6,"code":"unexpected_fields","message":"…"}
{"type":"pong","t":1234.5}
```

### Error codes

| Code | Meaning |
|---|---|
| `bad_json`, `bad_frame`, `no_type`, `bad_id` | not a well-formed frame |
| `unknown_type` | not a `coco.v1` command — including every rosbridge op |
| `unexpected_fields` | a known command carrying a field it does not accept |
| `protocol_mismatch` | the client named a different version |
| `bad_velocity`, `bad_goal`, `bad_arm`, `bad_grip` | non-numeric, NaN or infinite |
| `bad_mode`, `bad_colour`, `bad_action`, `unknown_stream` | outside the allowed set |
| `bad_binary`, `bad_streams`, `bad_stream`, `bad_stream_config` | malformed subscription or tuning |
| `not_in_control` | another client holds the stick — **resolves itself** |
| `refused` | well-formed, but the robot would not do it |
| `command_failed` | the server raised while acting on it |

Non-finite floats are scrubbed from every outbound frame. `json.dumps`
emits bare `NaN` by default, which `JSON.parse` rejects — one bad range
reading would otherwise cost the client the whole frame, mission state
included.

---

## Performance

`GET /api/metrics`, and the same document as `platform.perf` in
telemetry. Every number is **measured**, never a configured value.

```jsonc
{"clients":1, "frames_per_s":10.0, "bytes_per_s":24100.0,
 "cpu_percent":67.3, "peak_buffer_bytes":0,
 "mission_latency_ms":34.7,
 "streams":{"camera":{"in_hz":15.0,"out_hz":6.2,"sent":38,
                      "dropped":0,"bytes_per_s":21400.0}, …}}
```

`in_hz` is what ROS delivered; `out_hz` is what reached clients. **The
gap between them is the dropping** — which is the difference between "the
video is choppy" and "the encoder is keeping up and the socket is not".

`cpu_percent` is a share of **one core** (100 = one core saturated), from
`os.times()`. It covers this process only; the simulator's CPU is
Gazebo's business and is not attributed here.

`mission_latency_ms` is measured from the `/mission/state` callback to
the frame that carried it out. Only a *changed* line starts the clock —
the executive re-asserts the same line at 2 Hz, and timing a repeat would
report the tick interval instead of the lag. **Measured: 28.7–43.6 ms**
over 48 samples during a live mission.

### Backpressure

**One frame in flight per stream per client.** While a write has not
flushed, the next frame for that stream is dropped and counted. A second
bound sits on tornado's own write buffer (1 MiB), because a stalled TCP
connection can leave a write pending forever.

**Control frames are never dropped.** `ack`, `error`, `pong` and the
telemetry tick always write. Starving the path that carries mission state
and the STOP acknowledgement to keep video smooth would be the wrong
trade.

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
The Trigger reply is an **acceptance, not an outcome** — the result
arrives on `/mission/state`.

**Never publishes**: `/diff_drive_controller/cmd_vel`, `/cmd_vel`,
`/cmd_vel_smoothed`, `/cmd_vel_nav`. A node configured to publish any of
them **refuses to start**.

**Verified live on the P0.2 build:**

```
$ ros2 node info /coco_web_platform      # only velocity pub:
    /cmd_vel_teleop: geometry_msgs/msg/TwistStamped
$ ros2 topic info /diff_drive_controller/cmd_vel -v
    Publisher count: 1                   # cmd_vel_arbiter, alone
$ ros2 run coco_web platform_server --ros-args \
      -p teleop_topic:=/diff_drive_controller/cmd_vel
    coco_web.safety.UnsafeTopicError: the web layer may not publish …
```

---

## Connection lifecycle

1. Client opens `/ws`. Server sends `welcome`, then `map` if known.
2. Client sends `hello`, declaring `binary` support if it has it.
3. Client `subscribe`s to anything beyond the default set.
4. Telemetry flows at 10 Hz; sensor frames at their negotiated rates.
5. Client sends `drive` continuously (~10 Hz) while a stick or key is
   held, and an explicit `stop` on release.
6. On close, the client is detached, its camera subscription is
   reconciled away, and **if it was the last one the server publishes a
   zero velocity**.

### Session lifecycle and health: two axes

`session.lifecycle` is **where the session is in its life**: `CREATED`,
`STARTING`, `READY`, `RUNNING`, `STOPPING`, `STOPPED`, `FAILED`.

`session.health` is **whether everything it should have is working**:

| Health | Meaning | `/healthz` |
|---|---|---|
| `HEALTHY` | every required component up, and every *expected* one | 200 |
| `DEGRADED` | the robot is drivable, but something expected is down | 200 |
| `UNHEALTHY` | a required component is down, or the session failed/stopped | 503 |

*Expected* is either of two facts: the launch file declared it
(`expected_components`; `mission.launch.py` declares
`lidar,navigation,perception` plus `mission` unless `executive:=false`),
or it was up earlier in this session and has since gone. The second is
what catches a navigation stack that died mid-run on an appliance that
declared nothing. `session.degraded_by` names what is missing;
`session.missing` names required components that are down.

They are two axes because one enum cannot say the two combinations an
operator most needs told apart: **READY + DEGRADED** (you can drive, but
navigation has died) and **RUNNING + HEALTHY** (a fetch in progress with
nothing wrong). A test asserts the two vocabularies share no value.

`lifecycle` distinguishes a component that **never came up** (`STARTING`)
from one that came up and **fell over** (`FAILED`). Showing those
identically sends someone debugging a simulator that is merely booting.

**`session.state` is kept, unchanged, for P0.1 clients** — readiness,
`starting` → `degraded` → `ready` → `stopped` — because `coco.v1` may not
change what an existing field means. Note its `degraded` is *not* health
`DEGRADED`: it predates the health axis and means "some required
component is down". `/healthz` is 200 exactly when `state` is `ready`,
which is exactly when health is not `UNHEALTHY`; a test sweeps all 256
component combinations to pin that equivalence.

A mission running never changes `/healthz`. If it did, Docker's
`HEALTHCHECK` would call the container unhealthy for the whole fetch and
a restart policy would kill the robot mid-climb.

### What counts as evidence that COCO is up

"Ready" is never inferred from a generic simulator being alive. Each
component is observed from data **arriving**, not from a publisher
existing:

| Component | Required | Evidence (words sent to clients) | Read from |
|---|---|---|---|
| `ros` | yes | the platform node is running | the rclpy executor |
| `simulator` | yes | COCO's own simulator is stepping | a `/clock` publisher **and** `/model/coco/odometry` arriving within 3 s |
| `robot` | yes | the wheel controller is reporting odometry | `/diff_drive_controller/odom` arriving |
| `arbiter` | yes | the command arbiter is reporting | `/cmd_vel_arbiter/status` arriving |
| `lidar` | expected | LiDAR scans are arriving | `/scan` arriving |
| `navigation` | if declared/seen | navigation is publishing a pose or a plan | `/plan` or `/amcl_pose` arriving |
| `perception` | if declared/seen | the target finder is reporting | `/perception/status` arriving |
| `mission` | if declared/seen | the mission executive is reporting | `/mission/state` arriving |

`/clock` alone is not evidence: an unrelated project's Gazebo publishes
one on the same graph, which was measured on the development machine. A
**publisher** is not evidence either: a `parameter_bridge` orphaned by a
killed simulator keeps its publisher and sends nothing. So the simulator
probe is a raw (never deserialised) subscription to COCO's own model
odometry, the gz plugin's output. A paused simulator therefore reads as
down, which is what it is to someone trying to drive.

### `platform.connection`

`CONNECTED`, `SIMULATOR_STARTING`, `SIMULATOR_READY`,
`MISSION_RUNNING`, `ERROR`.

**`CONNECTING` and `DISCONNECTED` are deliberately not here.** They are
facts about a socket, which only the client can observe — a server
reporting `DISCONNECTED` would be reporting it down a connection. The
shipped page owns those two and shows all seven together.

### Watchdogs, and why there are four

| Where | Triggers | Timeout |
|---|---|---|
| browser | release, blur, tab hidden, `Space` | immediate, 3 explicit zeros |
| `platform_server` | `drive` frames stop arriving | 0.5 s |
| `cmd_vel_arbiter` | the teleop source goes stale | 0.3 s |
| `diff_drive_controller` | no command at all | 0.5 s |

A client that stops sending has merely gone quiet; a client that sends
`stop` has decided. Both end with the robot stationary, but only one does
so immediately.

### Reconnect

Exponential backoff, 500 ms to a 10 s ceiling. WebSocket ping/pong runs
at 10 s with a 30 s timeout, so a client whose network vanished without a
FIN is reaped — which matters here because **the last client
disconnecting is what stops the robot**, and a ghost client keeps the
platform believing someone is watching.

On reconnect the server has a fresh `Subscription` at its defaults, so a
client must re-assert anything it had subscribed to. A **changed session
id** means the server restarted: the shipped page clears its view rather
than drawing a pose from a robot that no longer exists.

### Heartbeat — the client's half

Server-side ping/pong reaps a dead *client*. The page needs the mirror
image: a server that froze keeps its socket open and sends nothing, and
the browser's WebSocket does not notice. Telemetry is 10 Hz and never
dropped, so **4 s of silence** (`SILENCE_MS`) means the connection is
dead. The page then abandons the socket without waiting for a close
handshake a frozen peer will never answer, clears every chip that could
still claim the robot is healthy, greys the view, and reconnects on its
own clock. Measured in a real browser with the server SIGSTOPped:
declared disconnected and already reconnecting at 5.8 s; on SIGCONT it
recovered to the same session with no spurious reset.

### STOP, from the page

STOP is reachable in every state: it stacks above the not-ready curtain
(which covered it, unreachable by mouse, in the first pass) and is pinned
full-width to the bottom edge on a phone. It clears held keys, so a W
still held by the other hand does not drive the robot off again on the
next 100 ms tick. With no connection, the page says nothing was sent —
and that COCO stops by itself: the drive watchdog zeroes a stick that
goes quiet for 0.5 s, and the last client leaving publishes a stop.

### Protocol strictness added in P0.2's second pass (Codex, integrated)

`decode()` now rejects a frame with **duplicate keys** (`bad_json`) —
`{"type":"stop","type":"drive"}` would otherwise be whichever key the
parser kept last — and turns integers too large for a float, and
pathologically nested JSON, into `bad_json`/`bad_velocity`-class errors
instead of exceptions escaping the handler. The subscription document
gains `queue_depth` (0 or 1 — the one frame in flight) and
`send_attempts` per stream. All additive.

---

## Talking to it without a browser

```python
import json
from tornado.ioloop import IOLoop
from tornado.websocket import websocket_connect

async def main():
    ws = await websocket_connect('ws://localhost:8080/ws')
    print(json.loads(await ws.read_message())['protocol'])   # coco.v1
    ws.write_message(json.dumps({'type': 'hello', 'binary': False}))
    ws.write_message(json.dumps({'type': 'select_target', 'colour': 'blue'}))
    ws.write_message(json.dumps({'type': 'mission', 'action': 'start'}))
    while True:
        frame = await ws.read_message()
        if isinstance(frame, bytes):
            continue                      # a sensor frame; see binary.py
        frame = json.loads(frame)
        if frame['type'] == 'telemetry':
            m = frame['mission']
            print(m['state'], m['phase'], m['step'], '/', m['steps'])

IOLoop.current().run_sync(main)
```

```bash
curl -s localhost:8080/healthz | head -20      # 200 only when ready
curl -s localhost:8080/api/session
curl -s localhost:8080/api/metrics
```

---

## Security

P0.2, like P0.1, has **no authentication**, accepts **any origin**, and
binds `0.0.0.0`. That is a deliberate, recorded decision for a
single-user local appliance — see `PRODUCT_ARCHITECTURE.md`, which also
says what must change before this is exposed beyond a trusted LAN.

The command surface is what makes that tractable: even with no
authentication, a client that reaches the socket can only do the
thirteen things the protocol names, at velocities the server clamps,
against an allowlist checked at node construction.

`bind:=127.0.0.1` refuses the LAN today if you want that.
