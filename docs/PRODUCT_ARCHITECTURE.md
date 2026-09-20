# PRODUCT ARCHITECTURE

How COCO becomes something you open in a browser, and what must stay true
while it does.

Companion documents: `WEB_API.md` (the wire protocol), `DOCKER.md` (the
runtime), `ROADMAP.md` Track 3 (the sequence). The robotics stack itself
is described in `ARCHITECTURE.md`, and this document does not restate it.

---

## The one rule

**ROS 2 and Gazebo remain the simulation authority. The web layer is a
client.**

Everything below follows from that. The browser does not own robot
physics, mission state, or the command path. It asks; the robot decides.
When the two disagree, the robot is right.

Concretely, and testably:

- No robot state lives only in the browser. Refreshing the page loses
  nothing but a scroll position.
- The web layer publishes no velocity that the arbiter did not select.
- The mission state machine is `coco_mission`'s. The UI renders it and
  never advances it.
- Anything the UI can do, a terminal can already do. `/mission/start` is
  the same `std_srvs/Trigger` either way — the same door, with a button
  on it.

---

## The shape

```
    browser (any device on the LAN)
        |
        |  HTTP        page, /healthz, /api/session          :8080
        |  WebSocket   coco.v1 — commands up, telemetry down  :8080/ws
        |  MJPEG       camera frames, binary, out of band     :8081
        v
  +---------------------------------------------------------------+
  |  platform_server        (coco_web, ONE ROS 2 node)            |
  |                                                               |
  |    tornado  HTTP + WS  <--->  rclpy  subscriptions/publishers |
  |                                                               |
  |    protocol.py   closed vocabulary, versioned                 |
  |    safety.py     publish allowlist, checked at construction   |
  |    session.py    SimulationSession + readiness                |
  |    telemetry.py  status lines -> meaning                      |
  +---------------------------------------------------------------+
        |  publishes ONLY:  /cmd_vel_teleop  /mission/mode
        |                   /mission/target_colour  /goal_pose
        |                   /arm_controller/...  /gripper_controller/...
        |  calls    ONLY:   /mission/start  /mission/abort
        v
  +---------------------------------------------------------------+
  |  the robot                                                    |
  |                                                               |
  |   Nav2 -> /cmd_vel -> cmd_vel_relay -> /cmd_vel_gated ----+   |
  |   browser --------> /cmd_vel_teleop ----------------------+   |
  |   RL policy ------> /cmd_vel_rl --------------------------+   |
  |   approach -------> /cmd_vel_approach --------------------+   |
  |                                                           |   |
  |                                            cmd_vel_arbiter <--+
  |                                                    |          |
  |                              /diff_drive_controller/cmd_vel   |
  |                                                    |          |
  |                             collision monitor -> smoother     |
  |                                                    |          |
  |                                                 wheels        |
  |                                                               |
  |   mission_executive owns /mission/mode and /mission/state     |
  |   Gazebo Harmonic owns physics and the clock                  |
  +---------------------------------------------------------------+
```

The browser enters that diagram at exactly one place — as one more
arbiter *input*, ranked below nothing and above nothing, preempting by
the same rule every other teleop source does.

---

## Service boundaries

| Boundary | What crosses it | Who owns correctness |
|---|---|---|
| browser ↔ platform_server | coco.v1 frames (JSON) | `protocol.py` — every frame validated, extra keys refused |
| browser ↔ web_video_server | MJPEG bytes | out of band on purpose; see *Camera*, below |
| platform_server ↔ ROS graph | an allowlist of topics and two services | `safety.py` — checked at node construction |
| arbiter ↔ wheels | one `TwistStamped` publisher | `cmd_vel_arbiter`, and only it |
| container ↔ host | two TCP ports | `docker-compose.yml` |

### Why the command boundary is shaped this way

The panel this replaces spoke **rosbridge**. rosbridge is a generic
ROS-over-WebSocket bridge: any browser tab could publish any topic, call
any service, and enumerate the graph through `rosapi`. The old panel's
HTML was well-behaved, but nothing *made* it so. A second tab, a stale
cache, or a curious user could publish `/diff_drive_controller/cmd_vel`
directly and become a second wheel publisher.

That is not a theoretical failure here. `cmd_vel_arbiter` warns when it
sees a second publisher on its output, but a warning is not a boundary —
by the time it fires, two sources are interleaving and the robot tracks
their average. C2-NAV.41 measured exactly that reaching the wheels
21.31 % of the time.

So the protocol is a **closed vocabulary**. A client names an intent
("drive", "start the mission"). It never names a topic, a service, or a
message type, because no frame in the schema has a field that could carry
one. Three independent mechanisms, none of which relies on the others:

1. **The schema cannot express a topic.** `coco_web/test/test_safety.py`
   asserts that no frame type accepts `topic`, `msg_type`, `service`,
   `op` or `msg`, and that a real rosbridge publish frame aimed at the
   wheels fails to parse.
2. **The allowlist is checked at construction.** `assert_publish_safe()`
   runs over every topic the node is about to publish — *including values
   that came from ROS parameters*, which is the path a hurried
   `-p teleop_topic:=/diff_drive_controller/cmd_vel` would take. It
   raises and the node refuses to start.
3. **Velocity is clamped in `decode()`**, at the boundary, so no call
   site can forget to. NaN becomes 0.0, because NaN fails every
   comparison and would otherwise pass straight through a min/max clamp
   into the controller.

---

## Session model

`SimulationSession` is one COCO simulation and the clients attached to
it: id, state, created_at, per-component health, connected clients,
active mission, selected colour.

**P0.1 serves exactly one session, and enforces it** — `SessionRegistry`
refuses a second rather than letting two clients silently share one
robot. Two *clients* on one session is fine and supported; two *sessions*
would mean two people driving the same wheels.

The registry exists now, at one session, because the thing that makes
multi-session hard later is not the container scheduler — it is code that
assumes the session is a global. Every call site already asks a registry
for a session by id.

### Readiness is not a boolean

A ROS + Gazebo stack does not start, it **converges**. rclpy is up in
milliseconds, the clock a second later, the controllers several seconds
after that, Nav2's lifecycle later still. Reporting "ready" when the web
server binds its port is how a developer presses Start against a
simulator that has not spawned the robot, and then files the timeout as a
mission bug.

| State | Meaning |
|---|---|
| `starting` | nothing has reported yet |
| `degraded` | some required component is down — the UI names which |
| `ready` | every required component is up |
| `stopped` | terminal; will not become ready again |

### Two axes, not one (P0.2)

`state` above is **readiness**, and it is what `/healthz` reports.
`lifecycle` is a second, orthogonal axis — `CREATED`, `STARTING`,
`READY`, `RUNNING`, `STOPPING`, `STOPPED`, `FAILED` — saying where the
session is in its life.

They are kept apart because merging them breaks something concrete. If
`RUNNING` displaced `ready` the moment a mission started, Docker's
`HEALTHCHECK` would mark the container unhealthy for the entire duration
of a fetch, and a `restart:` policy would then kill the robot mid-climb.
Two axes cost one extra field and avoid that. A test asserts `/healthz`
is still 200 with a mission running.

`lifecycle` also distinguishes a component that **never came up**
(`STARTING`) from one that came up and **fell over** (`FAILED`), using a
`converged` flag. Reporting those identically sends someone debugging a
simulator that is merely still booting.

**`platform.connection`** is the third thing a UI needs: `CONNECTED`,
`SIMULATOR_STARTING`, `SIMULATOR_READY`, `MISSION_RUNNING`, `ERROR`.
`CONNECTING` and `DISCONNECTED` are deliberately absent — they are facts
about a socket, which only the client can observe, and a server
reporting `DISCONNECTED` would be reporting it down a connection.

> **`/healthz` 200 is not "localised".** It means the required components
> are up. AMCL may still have no pose, and a mission started in that
> window aborts immediately with `NAVIGATION_FAILED`. Measured on this
> machine, twice. A client that starts missions should wait for
> `nav.online`.

Required: `ros`, `simulator`, `robot`, `arbiter`.
Reported but **not** required: `mission`, `perception`, `navigation` —
the appliance is useful for manual driving without them, and a
permanently red light is worse than an honest amber one.

`GET /healthz` returns **200 only when `ready`**, 503 otherwise, with the
missing components named in the body. Docker's `HEALTHCHECK` uses it, so
"the container is healthy" and "the robot can be driven" are the same
statement.

**One measured subtlety.** `simulator` is not "something publishes
`/clock`". With no COCO simulator running at all, a probe on the
development machine found `count_publishers('/clock') == 1`, because an
unrelated project's Gazebo was up on the same ROS graph — and the health
check cheerfully reported the simulator as up. It now also requires
`/model/coco/odometry`, the gz plugin's own output, which appears before
`ros2_control` activates and so keeps "simulator up, robot not yet"
reportable.

---

## Camera and depth

**P0.1 kept images off the WebSocket entirely**: `web_video_server`
served MJPEG on `:8081` and the browser got its *metadata*. That is
efficient, and it is out of band — which is exactly the problem. A
stream the platform does not carry is one it cannot subscribe per
client, cannot rate-limit, and cannot count. *"A slow browser must not
grow ROS memory"* is unprovable about a socket this process does not own.

**P0.2 encodes JPEG in `platform_server` and sends binary frames**
(`binary.py`), subject to the same subscription, rate and backpressure
rules as every other stream. MJPEG stays available behind `video:=` for
one release, the way the rosbridge panel was retired.

Base64 in JSON was never the alternative: it inflates every frame by a
third. A *binary* frame alongside JSON text frames keeps image bytes off
the JSON parser without putting them out of reach of the platform's own
accounting. Measured: a 320×240 frame is **3 467 bytes** at quality 60.

Two costs, stated rather than discovered:

- **The frame is encoded once for every viewer.** Quality and scale are
  shared and the most demanding subscriber wins. Encoding per client
  would cost a JPEG per viewer to save bandwidth nobody is short of on a
  LAN.
- **The encoder competes with the simulator for CPU.** Measured at
  **67.3 % of one core** with a camera subscriber attached. The camera
  and depth ROS subscriptions therefore exist only while someone is
  watching and are destroyed when the last viewer leaves.

**Depth stays off** unless `depth_topic` is configured, which is where
C2-NAV.43 left it (a candidate, off by default, 18/21 against 16/21 but
with stale marks 3.2× worse). The browser must not be the thing that
quietly turns it on — and what P0.2 ships is a **visualisation**, a
greyscale picture carrying the metre range it mapped. Nothing in this
path feeds a costmap, the UI says so on the pane, and a test asserts the
page says so.

> **DEPTH VISUALISATION ≠ DEPTH NAVIGATION FUSION.** Two separate
> switches: `platform.launch.py depth_topic:=` shows the camera in the
> browser; `nav.launch.py depth_cloud:=true` builds a PointCloud2 for
> navigation. P0.2 touches only the first.

WebRTC remains a P0.3 question. Binary WebSocket frames proved adequate:
zero drops in every probe.

---

## Security boundaries

**P0.1 has no authentication, and that is a decision, not an oversight.**

The threat model is a single developer on their own machine or LAN. What
follows from that:

| Today | Why | What changes it |
|---|---|---|
| No auth on `/ws` or HTTP | nothing to steal; a login would break "open it from your phone" | P1.0, the moment it leaves localhost |
| `check_origin` returns True | same reason; an origin lock protects nothing here | P1.0 |
| Binds `0.0.0.0` | the LAN workflow is the point | `bind:=127.0.0.1` refuses it today |
| Container publishes only 8080/8081 | keeps the ROS graph off the host network | — |

**The one thing that is already hard:** the command surface. Even with no
authentication, a client that reaches the socket can only do the eleven
things the protocol names, at velocities the server clamps. That property
is what makes adding auth later a matter of gating the socket rather than
auditing what a bridge might let through.

**Do not expose this to the public internet.** `ROADMAP.md` P1.0 is where
that becomes a supported thing, and it is gated on authentication, TLS
and per-session isolation — not on it happening to work.

---

## Local development

```bash
# Docker (the documented path)
./scripts/run_platform.sh          # or: docker compose build && docker compose up
open http://localhost:8080

# Native, if you already have ROS 2 Jazzy + Gazebo Harmonic
./scripts/run_platform.sh --native
```

Details, including what is and is not verified, in `DOCKER.md`.

To run the web layer alone against a simulator you started yourself:

```bash
ros2 launch coco_web platform.launch.py          # :8080, starts an arbiter
ros2 launch coco_web platform.launch.py arbiter:=false   # one already runs
```

`mission.launch.py` starts it automatically (`platform:=true`, the
default) and passes `arbiter:=false`, because the mission stack already
started one. That argument is load-bearing: two arbiters means two
publishers on the wheel topic.

---

## Where this goes, and what has to change

P0.1 is deliberately single-session. The parts that are already shaped
for more than one:

- sessions are addressed by id, never by a global;
- the protocol carries a session document in `welcome` and telemetry;
- readiness is per-component, so a supervisor can tell *why* a session is
  not serving.

The parts that are **not**, and must not be pretended otherwise:

- **One ROS graph per machine.** Two COCO sessions on one graph would
  share `/mission/mode`, `/cmd_vel_teleop` and the wheels. Multi-session
  needs one container per session with its own `ROS_DOMAIN_ID`, which is
  a runtime change, not a code change.
- **One Gazebo at a time**, on a host, always. The `--native` path
  refuses to start when it finds another simulator rather than sweeping
  it away.
- **`SessionRegistry.max_sessions = 1`.** Raising that number alone does
  not make the platform multi-user; it makes it wrong. The isolation has
  to arrive first.

At P1.1 the web tier does split from the simulation container, because
then it is fronting many of them. That is the point at which the split
`DOCKER.md` currently argues against becomes necessary.
