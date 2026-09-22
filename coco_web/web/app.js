// Copyright 2026 Gautham Anil — Apache-2.0.
//
// COCO 2.0 browser client.
//
// Speaks coco.v1 (see docs/WEB_API.md) over a single WebSocket to
// platform_server. There is no ROS in this file: no topic names, no
// message types, no service paths. The client names intents and the
// server decides what that means on the graph, which is what stops a
// browser tab from becoming a second wheel publisher.
//
// The operator-facing vocabulary is deliberate. The buttons say "Manual",
// "Auto" and "Hold"; the wire says teleop/auto/stop; the arbiter latches
// teleop/nav/idle. A user should be able to think "drive COCO up the
// ramp" without meeting any of the other two spellings.
//
// What P0.2 changed here
// ----------------------
// The mission progress bar used to be computed from a fifteen-entry
// PHASES list hard-coded in this file. The executive owns that state
// machine and this file was guessing at it; the server now sends the
// real step number from the executive's own chain, and the guess is gone.
//
// Sensor frames arrive as BINARY WebSocket messages (see binary.py).
// This client declares `binary: true` in hello, so a client that cannot
// parse them simply never receives them.

"use strict";

const $ = (id) => document.getElementById(id);
const WS_URL = `ws://${location.host}/ws`;

// ── connection ────────────────────────────────────────────────────────
// Reconnect backs off rather than hammering at a fixed 2 s the way the
// old panel did: a server that is down stays down for a while, and a tab
// left open overnight should not spend the night reconnecting.
let ws = null;
let backoff = 500;
let limits = null;
let world = null;
let sessionId = null;
let seenWelcome = false;
let lastFrameAt = 0;
let reconnectAt = 0;

// The liveness watchdog. Telemetry is sent at 10 Hz and the server never
// drops it, so silence this long means the connection is dead even if the
// socket has not noticed -- a laptop resumed from sleep, a Wi-Fi handover,
// a server machine that vanished without a FIN. Without this the page
// keeps drawing the last pose as though it were live.
const SILENCE_MS = 4000;

function connect() {
  reconnectAt = 0;
  setConnState("CONNECTING");
  const sock = new WebSocket(WS_URL);
  ws = sock;
  sock.binaryType = "arraybuffer";
  // Stale-socket suppression (from Codex's transport): every handler
  // checks it still belongs to the CURRENT socket. The silence watchdog
  // already detaches handlers from a socket it abandons; this also covers
  // an event queued on the old socket before that happened, which would
  // otherwise schedule a second reconnect or paint a frame from it.
  const current = () => sock === ws;
  sock.onopen = () => {
    if (!current()) { return; }
    backoff = 500;
    lastFrameAt = performance.now();
    setConn("connecting");
    send({ type: "hello", client: "coco-web-ui", binary: true });
  };
  sock.onclose = () => {
    if (!current()) { return; }
    setConn("disconnected");
    scheduleReconnect();
  };
  sock.onerror = () => { /* onclose always follows; handled there. */ };
  sock.onmessage = (event) => {
    if (!current()) { return; }
    lastFrameAt = performance.now();
    document.body.classList.remove("stale");
    if (event.data instanceof ArrayBuffer) { onBinary(event.data); return; }
    let frame;
    try { frame = JSON.parse(event.data); } catch { return; }
    handle(frame);
  };
}

function scheduleReconnect() {
  reconnectAt = performance.now() + backoff;
  setTimeout(connect, backoff);
  backoff = Math.min(backoff * 2, 10000);
}

setInterval(() => {
  if (!ws) { return; }
  if (ws.readyState === WebSocket.OPEN && seenWelcome &&
      performance.now() - lastFrameAt > SILENCE_MS) {
    // Abandon the silent socket rather than waiting for it to close: a
    // frozen peer never answers the close handshake, so `onclose` can be
    // minutes away. Detach it, close it, and reconnect on our own clock.
    setConnState("DISCONNECTED",
      `No data from COCO for ${SILENCE_MS / 1000} s — reconnecting.`);
    stopDriving();
    const dead = ws;
    dead.onclose = null;
    dead.onmessage = null;
    dead.onerror = null;
    try { dead.close(); } catch { /* already closing */ }
    setConn("disconnected");
    scheduleReconnect();
    return;
  }
  if (connState === "DISCONNECTED" && reconnectAt) {
    const s = Math.max(0, (reconnectAt - performance.now()) / 1000);
    $("waitingWhy").textContent =
      `The page lost its connection to COCO. Retrying in ${s.toFixed(0)} s.`;
  }
}, 500);

function send(frame) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(frame));
    return true;
  }
  return false;
}

// CONNECTING and DISCONNECTED are ours: they are facts about a socket,
// which the server cannot observe. Everything else comes from the server
// in telemetry, because only it knows whether there is a robot down there.
function setConn(state) {
  const pill = $("conn");
  const up = state !== "disconnected";
  pill.classList.toggle("on", state === "connected");
  pill.textContent = state === "disconnected" ? "reconnecting…" : state;
  if (!up) {
    if (connState !== "DISCONNECTED") {
      setConnState("DISCONNECTED", "The page lost its connection to COCO.");
    }
    seenWelcome = false;
    // Nothing on screen may look live once the data stopped: not the
    // picture, and not a chip still saying "Healthy" about a robot this
    // page can no longer hear.
    document.body.classList.add("stale");
    $("healthChip").hidden = true;
    $("missingChip").hidden = true;
  }
}

const CONN_WORDS = {
  CONNECTING: ["Connecting", "Opening the connection to COCO."],
  DISCONNECTED: ["Disconnected", "The page lost its connection to COCO."],
  SIMULATOR_STARTING: ["Starting COCO", "The simulator is still coming up."],
  SIMULATOR_READY: ["Almost ready", "The simulator is up; the robot is still starting."],
  CONNECTED: ["Ready", ""],
  MISSION_RUNNING: ["Mission running", ""],
  ERROR: ["Something is wrong", "A part of the stack that was running has stopped."],
};

let connState = "CONNECTING";

function setConnState(state, why) {
  connState = state;
  const [word, blurb] = CONN_WORDS[state] || [state, ""];
  const chip = $("connState");
  chip.textContent = word;
  chip.className = "chip" +
    (state === "CONNECTED" || state === "MISSION_RUNNING" ? " ready" :
     state === "ERROR" || state === "DISCONNECTED" ? " bad" : " warn");
  // The controls are hidden, not merely disabled, until COCO can
  // actually be driven. A greyed-out joystick over a simulator that has
  // not spawned invites pressing it and filing the silence as a bug.
  const blocked = state !== "CONNECTED" && state !== "MISSION_RUNNING";
  $("waiting").hidden = !blocked;
  document.body.classList.toggle("blocked", blocked);
  $("waitingWhat").textContent = word;
  $("waitingWhy").textContent = why || blurb;
  $("sConn").textContent = state;
  $("connRead").textContent = state;
}

// Health is the second axis. It is shown beside the connection, never
// folded into it: DEGRADED is a robot you can still drive.
const HEALTH_WORDS = {
  HEALTHY: ["Healthy", "ready"],
  DEGRADED: ["Degraded", "warn"],
  UNHEALTHY: ["Unhealthy", "bad"],
};

function setHealth(health, degradedBy, missing) {
  const chip = $("healthChip");
  const [word, cls] = HEALTH_WORDS[health] || [health || "—", ""];
  chip.hidden = !health;
  chip.textContent = word;
  chip.className = "chip " + cls;
  const why = health === "DEGRADED" ? degradedBy
    : health === "UNHEALTHY" ? missing : [];
  chip.title = why && why.length
    ? `Health: ${word} — ${why.map(componentWord).join(", ")}`
    : `Health: ${word}`;
  $("healthRead").textContent = health || "—";
  $("healthRead").className = cls ? `st-${cls}` : "";
  $("degradedRead").textContent =
    degradedBy && degradedBy.length ? degradedBy.join(", ") : "nothing";
  $("missingRead").textContent =
    missing && missing.length ? missing.join(", ") : "nothing";
}

// Component names in the words Play mode uses. Engineering shows the
// raw name beside them.
const COMPONENT_NAMES = {
  ros: "the platform", simulator: "the simulator", robot: "the wheels",
  arbiter: "the command arbiter", mission: "the mission system",
  perception: "the camera finder", navigation: "navigation",
  lidar: "the LiDAR",
};
function componentWord(name) { return COMPONENT_NAMES[name] || name; }

// ── inbound JSON frames ───────────────────────────────────────────────
function handle(frame) {
  switch (frame.type) {
    case "welcome":      onWelcome(frame); break;
    case "telemetry":    onTelemetry(frame); break;
    case "map":          onMap(frame); break;
    case "subscription": onSubscription(frame); break;
    case "pong":         onPong(frame); break;
    case "error":        onError(frame); break;
    case "ack":          break;
    default:             break;
  }
}

function onWelcome(frame) {
  seenWelcome = true;
  limits = frame.limits || {};
  world = frame.world || null;
  $("protoRead").textContent = `protocol ${frame.protocol}`;
  // A changed session id means the server restarted under us. The view
  // is about a robot that no longer exists, so it is cleared rather than
  // left showing a pose from before the restart.
  const id = (frame.session && frame.session.id) || null;
  if (sessionId && id && id !== sessionId) { resetView(); }
  sessionId = id;
  $("sessionRead").textContent = id || "—";
  buildTargets(limits.colours || []);
  buildArm(limits);
  attachAnnotated(frame.streams || {});
  setConn("connected");
  // Re-assert stream choices: after a reconnect the server has a fresh
  // Subscription at its defaults, and the camera checkbox may be ticked.
  syncStreams();
}

function resetView() {
  mapImage = null;
  mapMeta = null;
  lastTelemetry = null;
  lidar = null;
  $("mapRead").textContent = "walls: waiting for the map";
  clearCanvas($("cam"));
  clearCanvas($("depth"));
  toast("COCO restarted — the view was reset");
}

function onSubscription(frame) {
  const on = new Set(frame.streams || []);
  $("camStatus").textContent = on.has("camera")
    ? "subscribed, waiting for a frame…" : "not subscribed";
  $("depthStatus").textContent = on.has("depth")
    ? "subscribed, waiting for a frame…" : "not subscribed";
}

function onError(frame) {
  // "Someone else is driving" resolves itself; it gets a note rather
  // than the red treatment a real refusal gets.
  const soft = frame.code === "not_in_control";
  toast(frame.message || frame.code, !soft);
  if (soft) { $("pilotNote").hidden = false;
              $("pilotNote").textContent = frame.message; }
}

// ── binary sensor frames ──────────────────────────────────────────────
// Decoded and validated by frame.js (window.cocoFrame): exact payload
// lengths, the kind byte agreeing with the header, sane metadata. A frame
// that fails is counted and dropped, never half-drawn.
let rejectedFrames = 0;
let rejectedLoggedAt = 0;

function onBinary(buffer) {
  let frame;
  try {
    frame = window.cocoFrame.decodeFrame(buffer);
  } catch (err) {
    rejectedFrames += 1;
    if (performance.now() - rejectedLoggedAt > 5000) {
      rejectedLoggedAt = performance.now();
      console.warn(`coco: refused ${rejectedFrames} sensor frame(s): ` +
                   err.message);
    }
    return;
  }
  const { stream, header, payload } = frame;
  if (stream === "lidar") { onLidar(frame); }
  else if (stream === "camera") { onImage("cam", header, payload); }
  else if (stream === "depth") { onImage("depth", header, payload); }
}

let lidar = null;

function onLidar(frame) {
  const header = frame.header;
  lidar = { angle_min: header.angle_min, angle_step: header.angle_step,
            ranges: frame.ranges, floor: header.floor };
}

// Each image stream keeps ONE pending bitmap. decode() is async, and
// without this a burst would queue decodes faster than they complete and
// paint them out of order.
const decoding = { cam: false, depth: false };

function onImage(id, header, payload) {
  const status = id === "cam" ? "camStatus" : "depthStatus";
  const label = id === "cam" ? "camera" : "depth";
  // A frame already in flight when the box was unticked must not repaint
  // a pane the user just switched off.
  if (!$(id === "cam" ? "camOn" : "depthOn").checked) { return; }
  if (decoding[id]) { return; }
  decoding[id] = true;
  const blob = new Blob([payload], { type: "image/jpeg" });
  createImageBitmap(blob).then((bitmap) => {
    const canvas = $(id);
    canvas.width = header.w;
    canvas.height = header.h;
    canvas.getContext("2d").drawImage(bitmap, 0, 0);
    bitmap.close();
    const drops = header.dropped ? ` · ${header.dropped} dropped` : "";
    const span = (header.min_m !== undefined)
      ? ` · ${header.min_m.toFixed(1)}–${header.max_m.toFixed(1)} m` : "";
    $(status).textContent = `${label}: ${header.w}×${header.h}${span}${drops}`;
  }).catch(() => {
    $(status).textContent = `${label}: could not decode a frame`;
  }).finally(() => { decoding[id] = false; });
}

function clearCanvas(canvas) {
  canvas.getContext("2d").clearRect(0, 0, canvas.width, canvas.height);
}

// ── telemetry ─────────────────────────────────────────────────────────
let lastTelemetry = null;

function onTelemetry(frame) {
  lastTelemetry = frame;
  const platform = frame.platform || {};
  const sess = platform.session || {};
  const arb = platform.arbiter || {};

  setConnState(platform.connection || sess.connection || "CONNECTED");
  const missing = sess.missing || [];
  const degradedBy = sess.degraded_by || [];
  setHealth(platform.health || sess.health, degradedBy, missing);
  // One amber chip naming what is absent, in words: the required parts
  // while COCO is coming up, the expected-but-lost ones after that.
  const absent = missing.length ? missing : degradedBy;
  const warn = $("missingChip");
  warn.hidden = absent.length === 0;
  warn.textContent = absent.length
    ? `${missing.length ? "waiting on" : "not running:"} ` +
      absent.map(componentWord).join(", ")
    : "";
  $("lifecycleRead").textContent = sess.lifecycle || "—";

  const pose = frame.robot && frame.robot.pose;
  // Heading wrapped to (-180, 180]: a yaw that has been accumulated
  // anywhere upstream must still read as a compass angle.
  const deg = pose
    ? ((((pose.yaw * 180 / Math.PI) + 180) % 360) + 360) % 360 - 180 : 0;
  // `localised` false means the position is wheel odometry alone -- exact
  // at spawn, drifting after that -- so the page says it is still finding
  // its place rather than presenting it as a fix.
  const settling = frame.robot && frame.robot.localised === false;
  $("poseRead").textContent = pose
    ? `x ${pose.x.toFixed(2)}  y ${pose.y.toFixed(2)}  ` +
      `θ ${deg.toFixed(0)}°` + (settling ? " · finding its position" : "")
    : "pose —";
  const vel = frame.robot && frame.robot.velocity;
  $("velRead").textContent = vel
    ? `${vel.linear.toFixed(2)} m/s · ${vel.angular.toFixed(2)} rad/s`
    : "— m/s";
  // Which source actually owns the wheels. The arbiter's own word for it,
  // translated: "nav" is what the operator calls Auto.
  const src = arb.active;
  const srcWord =
    src === "teleop" ? "you" :
    src === "nav" ? "navigation" :
    src === "rl" ? "ramp policy" :
    src === "approach" ? "visual approach" : "idle";
  $("srcRead").textContent = "wheels: " + srcWord;
  $("sSource").textContent = srcWord;
  $("sMode").textContent = arb.mode || "—";
  $("sPilot").textContent = platform.pilot
    ? (platform.pilot === myPilotHint ? "this browser" : "another browser")
    : "nobody";

  updateMission(frame.mission || {});
  updatePerception((frame.sensors && frame.sensors.perception) || {});
  updateComponents(sess.components || {});
  updatePerf(platform.perf || {});
  updateNavigation(frame, srcWord);

  $("rawArbiter").textContent = "arbiter: " + (arb.raw || "offline");
  $("rawMission").textContent =
    "mission: " + ((frame.mission && frame.mission.raw) || "offline");
  $("rawPerception").textContent = "perception: " +
    ((frame.sensors && frame.sensors.perception &&
      frame.sensors.perception.raw) || "offline");
  $("rawGrasp").textContent = "grasp: " +
    ((frame.sensors && frame.sensors.grasp &&
      frame.sensors.grasp.raw) || "offline");
  $("seqRead").textContent = `seq ${frame.seq}`;

  draw(frame);
}

// ── mission ───────────────────────────────────────────────────────────
// No PHASES list and no percentage. The server sends `words` (operator
// wording for the executive's own state), `step`/`steps` from the
// executive's nominal chain, and `elapsed`/`timeout` for the state it is
// in. There is no overall ETA because nothing publishes one.
let missionActive = false;

// The ten-word phase vocabulary, coloured by what it means to an
// operator. The word itself comes from the server; only the colour is
// decided here.
const PHASE_CLASS = {
  IDLE: "idle", COMPLETED: "done", FAILED: "failed", STOPPED: "stopped",
};
// The executive's own `result` values, in words. Anything else is shown
// as it arrived rather than given invented prose.
const RESULT_WORDS = {
  fetch: "Result: the cylinder was fetched and brought home",
  traverse: "Result: crossed the ramp and came back",
  aborted: "Result: the mission was aborted",
};

function updateMission(mission) {
  missionActive = !!mission.active;
  $("missionPhase").textContent =
    mission.words || mission.state || "Mission system offline";

  const badge = $("missionBadge");
  const phase = mission.online ? (mission.phase || "IDLE") : "OFFLINE";
  badge.textContent = mission.recovering ? `${phase} · RETRYING` : phase;
  badge.className = "badge " + (mission.online
    ? (PHASE_CLASS[phase] || "active") : "idle");

  // The colour the MISSION holds is the truth; the swatch follows it, so
  // a reloaded page shows what the robot is actually fetching.
  syncColour(mission.colour);
  const target = $("missionTarget");
  target.innerHTML = "";
  if (mission.colour) {
    const sw = document.createElement("span");
    sw.className = "sw-dot";
    sw.style.background = SWATCH[mission.colour] || "#888";
    target.append("target ", sw, mission.colour);
  } else {
    target.textContent = selectedColour
      ? `target ${selectedColour} (not yet confirmed by the robot)`
      : "no colour chosen";
  }

  let detail = "";
  if (mission.recovering) {
    detail = `Retrying — attempt ${mission.attempt || "?"}`;
  } else if (mission.reason) {
    detail = mission.reason_words ||
      (mission.reason_known ? mission.reason
                            : `unrecognised code ${mission.reason}`);
  }
  $("missionDetail").textContent = detail;
  const why = $("missionWhy");
  why.hidden = !mission.result;
  why.textContent = mission.result
    ? (RESULT_WORDS[mission.result] || `Result: ${mission.result}`) : "";

  // Step N of M, from the executive's chain. A state that is not ON the
  // chain (RECOVERY, ABORT) reports no step, and the bar holds rather
  // than jumping somewhere the mission is not.
  // `elapsed` and `timeout` are the executive's ROS clock. Under
  // use_sim_time that is simulated seconds, which run at the real-time
  // factor (~0.4 with a browser attached), so they are labelled as such
  // rather than read as a stopwatch. See mission_view.timing().
  const unit = mission.timing && mission.timing.ros_is_sim ? "sim s" : "s";
  if (mission.step && mission.steps) {
    $("stepBar").style.width = `${(mission.step / mission.steps) * 100}%`;
    let line = `Step ${mission.step} of ${mission.steps}`;
    if (mission.elapsed !== null && mission.elapsed !== undefined) {
      line += ` · ${mission.elapsed.toFixed(0)} ${unit}`;
      if (mission.timeout) {
        line += ` of ${mission.timeout.toFixed(0)} ${unit}`;
      }
    }
    $("missionStep").textContent = line;
  } else if (!mission.online) {
    $("stepBar").style.width = "0";
    $("missionStep").textContent = "";
  }

  $("missionAbort").disabled = !missionActive;
  $("missionStart").disabled = missionActive || !selectedColour ||
    !mission.online;
  // The executive owns the arm during a run; offering the sliders then
  // would be offering a fight the operator cannot win.
  ["sh", "el", "gr"].forEach((id) => {
    $(id).disabled = missionActive || !limits || !limits.arm;
  });

  $("mState").textContent = mission.state || "—";
  $("mPrev").textContent = mission.previous || "—";
  $("mOwner").textContent = mission.owner || "—";
  $("mMode").textContent = mission.mode || "—";
  $("mElapsed").textContent = mission.elapsed === null ||
    mission.elapsed === undefined ? "—"
    : `${mission.elapsed.toFixed(1)} ${unit}` +
      (mission.timeout ? ` / ${mission.timeout.toFixed(0)} ${unit}` : "");
  // The wall-clock observation, kept apart from the sim-clock numbers:
  // when THIS server first saw the state, which is not the transition.
  const seen = mission.timing && mission.timing.wall_first_seen;
  $("mElapsed").title = seen
    ? `first seen by the platform ${Math.max(0, Date.now() / 1000 - seen)
      .toFixed(0)} s ago (wall clock)` : "";
  $("mAttempt").textContent = mission.attempt === null ||
    mission.attempt === undefined ? "—"
    : `${mission.attempt} (of ${(mission.retries || 0) + 1})`;
  $("mReason").textContent = mission.reason
    ? mission.reason + (mission.reason_known ? "" : " (unrecognised)") : "—";
  $("mResult").textContent = mission.result || "—";
}

function updatePerception(perception) {
  $("visionStatus").textContent = perception.online
    ? (perception.found
        ? `target in view${perception.seen ? ` (${perception.seen})` : ""}`
        : `searching${perception.seen ? ` — sees ${perception.seen}` : ""}`)
    : "perception: offline";
}

function updateComponents(components) {
  const body = $("components").querySelector("tbody");
  const names = Object.keys(components).sort();
  if (body.childElementCount !== names.length) { body.innerHTML = ""; }
  names.forEach((name, i) => {
    let row = body.children[i];
    if (!row) {
      row = document.createElement("tr");
      row.innerHTML = "<td></td><td class='st'></td>";
      body.appendChild(row);
    }
    const c = components[name];
    row.children[0].textContent = name;
    row.children[1].textContent = c.up ? "up" : "down";
    row.children[1].className = "st " + (c.up ? "up" : "down");
    row.children[1].title = c.detail || "";
  });
}

function updatePerf(perf) {
  const bytes = (n) => n > 1e6 ? `${(n / 1e6).toFixed(2)} MB/s`
                    : n > 1e3 ? `${(n / 1e3).toFixed(1)} kB/s`
                    : `${Math.round(n || 0)} B/s`;
  $("pFrames").textContent = `${perf.frames_per_s ?? "—"} /s`;
  $("pBytes").textContent = bytes(perf.bytes_per_s);
  $("pCpu").textContent = `${perf.cpu_percent ?? "—"} %`;
  $("pBuffer").textContent = `${perf.peak_buffer_bytes ?? 0} B`;
  $("pLatency").textContent = perf.mission_latency_ms === null ||
    perf.mission_latency_ms === undefined
    ? "not yet measured" : `${perf.mission_latency_ms} ms`;
  $("pClients").textContent = perf.clients ?? "—";

  const body = $("pStreams");
  const names = Object.keys(perf.streams || {});
  if (body.childElementCount !== names.length) { body.innerHTML = ""; }
  names.forEach((name, i) => {
    let row = body.children[i];
    if (!row) {
      row = document.createElement("tr");
      row.innerHTML = "<td></td><td></td>";
      body.appendChild(row);
    }
    const s = perf.streams[name];
    row.children[0].textContent = name;
    // in vs out is the whole point: the gap between them is the dropping.
    row.children[1].textContent =
      `in ${s.in_hz} Hz · out ${s.out_hz} Hz · dropped ${s.dropped}`;
  });
}

// ── navigation & sensors (Engineering) ────────────────────────────────
// Everything here is read from telemetry; nothing is estimated. The path
// length is the polyline the server sent, summed -- not a distance to go,
// which nothing publishes.
function updateNavigation(frame, srcWord) {
  const nav = frame.nav || {};
  const mission = frame.mission || {};
  const perf = (frame.platform && frame.platform.perf) || {};
  const rates = perf.streams || {};
  $("navRead").textContent = nav.online ? "running" : "not running";
  $("navSource").textContent = srcWord;
  const path = nav.path || [];
  let length = 0;
  for (let i = 1; i < path.length; i++) {
    length += Math.hypot(path[i][0] - path[i - 1][0],
                         path[i][1] - path[i - 1][1]);
  }
  $("navPath").textContent = path.length
    ? `${length.toFixed(2)} m, ${path.length} points` : "none";
  $("navPhase").textContent = mission.online
    ? `${mission.phase || "—"} (${mission.state || "—"})` : "offline";

  const scan = lidar;
  if (scan && scan.ranges && scan.ranges.length) {
    const hits = scan.ranges.filter((r) => r !== null);
    const closest = hits.length ? Math.min(...hits) : null;
    const hz = rates.lidar ? ` · ${rates.lidar.in_hz} Hz` : "";
    $("lidarRead").textContent = `${scan.ranges.length} rays · ` +
      (closest === null ? "no returns" : `closest ${closest.toFixed(2)} m`) +
      hz;
  } else {
    $("lidarRead").textContent = "no scan";
  }
  const rate = (name) => rates[name]
    ? `${rates[name].out_hz} fps to browsers (${rates[name].in_hz} Hz from ` +
      `the robot) · ${rates[name].dropped} dropped`
    : "not subscribed";
  $("camRate").textContent = rate("camera");
  $("depthRate").textContent = rate("depth");
}

// ── stream subscriptions ──────────────────────────────────────────────
// The heavy streams are opt-in, so the page asks for them only while its
// checkbox is ticked. Unticking unsubscribes, which closes the ROS
// subscription server-side when nobody else is watching.
function syncStreams() {
  const want = [];
  const drop = [];
  ($("camOn").checked ? want : drop).push("camera");
  ($("depthOn").checked ? want : drop).push("depth");
  if (want.length) { send({ type: "subscribe", streams: want }); }
  if (drop.length) { send({ type: "unsubscribe", streams: drop }); }
}

$("camOn").onchange = () => { syncStreams(); if (!$("camOn").checked) {
  clearCanvas($("cam")); } };
$("depthOn").onchange = () => { syncStreams(); if (!$("depthOn").checked) {
  clearCanvas($("depth")); } };

// ── targets ───────────────────────────────────────────────────────────
// Swatches are drawn from the colour names the SERVER advertised, which
// come from coco_config's own table. The panel does not keep its own
// list of what COCO can fetch.
const SWATCH = {
  red: "#d91a1a", green: "#1ab326", blue: "#1a40d9", yellow: "#e5cc1a",
};
let selectedColour = null;
let colourPickedAt = 0;

function buildTargets(colours) {
  const host = $("targets");
  host.innerHTML = "";
  colours.forEach((colour) => {
    const button = document.createElement("button");
    button.dataset.colour = colour;
    button.innerHTML = `<span class="sw" style="background:${
      SWATCH[colour] || "#888"}"></span>${colour}`;
    button.onclick = () => {
      colourPickedAt = performance.now();
      highlightColour(colour);
      send({ type: "select_target", colour });
      $("missionStart").disabled = missionActive;
    };
    host.appendChild(button);
  });
  if (selectedColour) { highlightColour(selectedColour); }
}

function highlightColour(colour) {
  selectedColour = colour;
  [...$("targets").children].forEach(
    (b) => b.classList.toggle("on", b.dataset.colour === colour));
}

// Adopt the colour the mission reports, unless this page picked one a
// moment ago and the echo is still on its way back.
function syncColour(colour) {
  if (!colour || colour === selectedColour) { return; }
  if (performance.now() - colourPickedAt < 2000) { return; }
  highlightColour(colour);
}

$("missionStart").onclick = () => send({ type: "mission", action: "start" });
$("missionAbort").onclick = () => send({ type: "mission", action: "abort" });

// ── drive ─────────────────────────────────────────────────────────────
let uiMode = "teleop";
let joyLin = 0, joyAng = 0, driving = false;
let pendingStop = 0;
// Not an identity, just a hint for the readout: the server tells us which
// client holds the stick, and this is how the page recognises itself.
let myPilotHint = null;

function setMode(mode) {
  uiMode = mode;
  $("modeTeleop").classList.toggle("on", mode === "teleop");
  $("modeAuto").classList.toggle("on", mode === "auto");
  $("modeStop").classList.toggle("on", mode === "stop");
  $("world").classList.toggle("clickable", mode === "auto");
  $("viewHint").textContent = mode === "auto"
    ? "click the map to send COCO there"
    : "switch to Auto to click a destination";
  send({ type: "set_mode", mode });
}

$("modeTeleop").onclick = () => setMode("teleop");
$("modeAuto").onclick = () => setMode("auto");
$("modeStop").onclick = () => { stopDriving(); setMode("stop"); };
$("estop").onclick = () => {
  // Drop held keys too. Otherwise a W still held while the other hand
  // clicks STOP is read by the 10 Hz loop on its next tick and the robot
  // drives off again 100 ms after being stopped. A key must be pressed
  // afresh to drive after a STOP.
  held.clear();
  stopDriving();
  const sent = send({ type: "stop" });
  setMode("stop");
  $("pilotNote").hidden = true;
  // Honest either way. With no connection nothing was sent, and the page
  // must not claim otherwise -- but COCO does not need it: the server
  // zeroes a stick that goes quiet for 0.5 s and publishes a stop when
  // the last page disconnects.
  toast(sent ? "Stopped"
             : "Not connected — COCO stops by itself when it loses the page",
        !sent);
};

function stopDriving() {
  driving = false; joyLin = 0; joyAng = 0;
  // Send a few explicit zeros rather than merely ceasing to send, so one
  // dropped frame cannot strand the robot at speed. The server's own
  // watchdog is the backstop, not the plan.
  pendingStop = 3;
}

// nipplejs reports force > 1 when dragged past the pad, which would ask
// for more than the server's limit. Clamping here too keeps the readout
// honest; the server clamps regardless.
window.addEventListener("load", () => {
  if (typeof nipplejs === "undefined") { return; }
  const stick = nipplejs.create({
    zone: $("joy"), mode: "static",
    position: { left: "50%", top: "50%" }, size: 130, color: "#ff8c2a",
  });
  stick.on("start", () => { if (uiMode !== "teleop") setMode("teleop"); });
  stick.on("move", (_e, data) => {
    driving = true;
    const force = Math.min(data.force, 1);
    const maxLin = (limits && limits.linear) || 0.5;
    const maxAng = (limits && limits.angular) || 1.2;
    joyLin = Math.sin(data.angle.radian) * force * maxLin;
    joyAng = -Math.cos(data.angle.radian) * force * maxAng;
  });
  stick.on("end", stopDriving);
});

// Keyboard driving. Held keys, not key repeat: repeat rate is an OS
// setting and would make the robot's speed depend on it.
const held = new Set();
const KEYS = { w: "w", a: "a", s: "s", d: "d",
               ArrowUp: "w", ArrowLeft: "a",
               ArrowDown: "s", ArrowRight: "d" };

document.addEventListener("keydown", (event) => {
  if (event.target.matches("input, textarea")) { return; }
  if (event.code === "Space") {
    event.preventDefault();
    held.clear();                  // same reason as the STOP button
    stopDriving();
    send({ type: "stop" });
    return;
  }
  const key = KEYS[event.key];
  if (!key) { return; }
  event.preventDefault();
  // Auto-repeat of a key that was held THROUGH a stop is not a fresh
  // press: it must not re-arm driving.
  if (event.repeat && !held.has(key)) { return; }
  if (uiMode !== "teleop") { setMode("teleop"); }
  held.add(key);
});
document.addEventListener("keyup", (event) => {
  const key = KEYS[event.key];
  if (key) { held.delete(key); if (held.size === 0) { stopDriving(); } }
});

// Releasing focus must stop the robot: a held key with the tab hidden is
// a robot driving with nobody watching.
window.addEventListener("blur", () => { held.clear(); stopDriving(); });
document.addEventListener("visibilitychange", () => {
  if (document.hidden) { held.clear(); stopDriving(); }
});

function keyboardVelocity() {
  const maxLin = (limits && limits.linear) || 0.5;
  const maxAng = (limits && limits.angular) || 1.2;
  let lin = 0, ang = 0;
  if (held.has("w")) { lin += maxLin * 0.6; }
  if (held.has("s")) { lin -= maxLin * 0.6; }
  if (held.has("a")) { ang += maxAng * 0.6; }
  if (held.has("d")) { ang -= maxAng * 0.6; }
  return [lin, ang];
}

// One 10 Hz loop owns every velocity frame, so the joystick and the
// keyboard cannot both be publishing at once.
setInterval(() => {
  let lin = joyLin, ang = joyAng;
  if (held.size > 0) { [lin, ang] = keyboardVelocity(); driving = true; }
  if (!driving && pendingStop <= 0) { return; }
  if (!driving) { pendingStop--; lin = 0; ang = 0; }
  send({ type: "drive", linear: lin, angular: ang });
}, 100);

// ── round-trip time ───────────────────────────────────────────────────
let pingSent = 0;
setInterval(() => {
  if (!seenWelcome) { return; }
  pingSent = performance.now();
  send({ type: "ping", t: pingSent });
}, 2000);

function onPong() {
  $("rttRead").textContent = `rtt ${Math.round(performance.now() - pingSent)} ms`;
}

// ── arm ───────────────────────────────────────────────────────────────
function buildArm(cfg) {
  if (!cfg.arm) { return; }
  const [shLo, shHi] = cfg.arm.shoulder;
  const [elLo, elHi] = cfg.arm.elbow;
  const [grLo, grHi] = cfg.gripper || [0, 1];
  Object.entries({ sh: [shLo, shHi], el: [elLo, elHi], gr: [grLo, grHi] })
    .forEach(([id, [lo, hi]]) => {
      const input = $(id);
      input.min = lo; input.max = hi;
      input.value = Math.min(hi, Math.max(lo, 0));
    });
  ["sh", "el"].forEach((id) => { $(id).oninput = onArmInput; });
  $("gr").oninput = onGripInput;
  refreshArmLabels();
}

function refreshArmLabels() {
  $("vSh").textContent = (+$("sh").value).toFixed(2);
  $("vEl").textContent = (+$("el").value).toFixed(2);
  $("vGr").textContent = (+$("gr").value).toFixed(2);
}

let armTimer = null;
function onArmInput() {
  refreshArmLabels();
  clearTimeout(armTimer);
  armTimer = setTimeout(() => send({
    type: "set_arm", shoulder: +$("sh").value, elbow: +$("el").value,
  }), 150);
}
function onGripInput() {
  refreshArmLabels();
  clearTimeout(armTimer);
  armTimer = setTimeout(
    () => send({ type: "set_gripper", grip: +$("gr").value }), 150);
}

// ── the annotated view ────────────────────────────────────────────────
// Still MJPEG, kept for one release, but served by the platform itself
// under an alias (/video/annotated): the page names a view, never a
// topic, and never talks to web_video_server directly. The path is
// resolved against this page's own origin, so a mapped port still works.
// The camera and depth panes above use the binary WebSocket path instead.
function attachAnnotated(all) {
  const stream = all.annotated;
  const img = $("vision");
  // Hidden until a frame actually loads: a broken image shows its alt
  // text in a black box, which reads as "the camera is broken".
  img.hidden = true;
  if (!stream) { $("visionStatus").textContent = "perception: not enabled";
                 return; }
  img.onload = () => { img.hidden = false; };
  img.onerror = () => {
    img.hidden = true;
    $("visionStatus").textContent = "perception: annotated view unavailable";
  };
  img.src = new URL(stream.path, location.href).href;
}

// ── the world view ────────────────────────────────────────────────────
// One canvas draws the map, the world's own furniture, the LiDAR return,
// the plan and the robot in a shared frame. Everything is in metres
// until the last moment.
const canvas = $("world");
const ctx = canvas.getContext("2d");
let mapImage = null;      // offscreen canvas holding the occupancy grid
let mapMeta = null;

function onMap(frame) {
  const { width, height, resolution, origin } = frame;
  const raw = atob(frame.data);
  const off = document.createElement("canvas");
  off.width = width; off.height = height;
  const image = off.getContext("2d").createImageData(width, height);
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      // int8 arrives as an unsigned byte; -1 (unknown) is 255.
      const v = raw.charCodeAt(y * width + x);
      const unknown = v === 255;
      const occupied = !unknown && v > 50;
      // ROS row 0 is the bottom; canvas row 0 is the top.
      const o = ((height - 1 - y) * width + x) * 4;
      const shade = unknown ? 26 : occupied ? 8 : 200;
      image.data[o] = image.data[o + 1] = image.data[o + 2] = shade;
      image.data[o + 3] = 255;
    }
  }
  off.getContext("2d").putImageData(image, 0, 0);
  mapImage = off;
  mapMeta = { width, height, resolution, origin };
  // The walls and obstacles drawn are the navigation map's -- the same
  // grid the planner uses -- not a second model of the world.
  $("mapRead").textContent =
    `walls: navigation map, ${(width * resolution).toFixed(1)} × ` +
    `${(height * resolution).toFixed(1)} m`;
}

// World -> canvas. With a map we fit the map; without one we show a
// fixed 12 m window around the origin so manual driving still has a
// frame of reference before Nav2 is up.
function transform() {
  const W = canvas.width, H = canvas.height;
  if (mapMeta) {
    const mw = mapMeta.width * mapMeta.resolution;
    const mh = mapMeta.height * mapMeta.resolution;
    const scale = Math.min(W / mw, H / mh) * 0.95;
    return {
      scale,
      ox: W / 2 - (mapMeta.origin.x + mw / 2) * scale,
      oy: H / 2 + (mapMeta.origin.y + mh / 2) * scale,
    };
  }
  const scale = Math.min(W, H) / 12;
  return { scale, ox: W / 2, oy: H / 2 };
}

function toPixels(t, x, y) {
  return [t.ox + x * t.scale, t.oy - y * t.scale];
}

function draw(frame) {
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "#0c0e12";
  ctx.fillRect(0, 0, W, H);
  const t = transform();

  if (mapImage && mapMeta && $("showMap").checked) {
    const mw = mapMeta.width * mapMeta.resolution * t.scale;
    const mh = mapMeta.height * mapMeta.resolution * t.scale;
    const [px, py] = toPixels(
      t, mapMeta.origin.x, mapMeta.origin.y + mapMeta.height * mapMeta.resolution);
    ctx.imageSmoothingEnabled = false;
    ctx.globalAlpha = 0.85;
    ctx.drawImage(mapImage, px, py, mw, mh);
    ctx.globalAlpha = 1;
  } else {
    drawGrid(t);
  }

  if (world) { drawWorld(t); }
  const pose = frame.robot && frame.robot.pose;
  if ($("showPath").checked) { drawPath(t, (frame.nav && frame.nav.path) || []); }
  if (pose && $("showLidar").checked) { drawLidar(t, pose); }
  if (pose) { drawRobot(t, pose); }
  drawMissionLabel(frame.mission || {});
}

// The mission's phase, on the world itself, so someone watching only the
// map still knows what COCO is doing. Same words as the mission card.
function drawMissionLabel(mission) {
  if (!mission.online) { return; }
  const text = `${mission.phase || "IDLE"} · ${mission.words || ""}`;
  ctx.save();
  ctx.font = "600 13px system-ui, sans-serif";
  const w = ctx.measureText(text).width + 16;
  ctx.fillStyle = "rgba(12, 14, 18, 0.78)";
  ctx.fillRect(10, 10, w, 24);
  ctx.fillStyle = mission.phase === "FAILED" ? "#ff8a8a"
    : mission.phase === "COMPLETED" ? "#3ecf6c" : "#ff8c2a";
  ctx.fillText(text, 18, 27);
  ctx.restore();
}

// The ramp, the platform and the four cylinders -- the actual simulation
// geometry, sent by the server from coco_config in map coordinates. This
// page re-types none of it.
function drawWorld(t) {
  const box = (x0, x1, width, fill, stroke) => {
    const [ax, ay] = toPixels(t, x0, width / 2);
    const [bx, by] = toPixels(t, x1, -width / 2);
    ctx.fillStyle = fill;
    ctx.fillRect(ax, ay, bx - ax, by - ay);
    ctx.strokeStyle = stroke;
    ctx.lineWidth = 1;
    ctx.strokeRect(ax, ay, bx - ax, by - ay);
  };
  box(world.ramp.x0, world.ramp.x1, world.ramp.width,
      "rgba(90, 110, 150, 0.18)", "rgba(120, 150, 200, 0.45)");
  box(world.platform.x0, world.platform.x1, world.platform.width,
      "rgba(90, 110, 150, 0.30)", "rgba(120, 150, 200, 0.6)");

  ctx.save();
  ctx.font = "11px system-ui, sans-serif";
  ctx.fillStyle = "rgba(160, 175, 200, 0.75)";
  const [rx, ry] = toPixels(t, (world.ramp.x0 + world.ramp.x1) / 2,
                            world.ramp.width / 2 + 0.25);
  ctx.textAlign = "center";
  ctx.fillText("ramp", rx, ry);
  ctx.restore();

  (world.targets || []).forEach((target) => {
    const [px, py] = toPixels(t, target.x, target.y);
    const chosen = target.colour === selectedColour;
    ctx.beginPath();
    ctx.arc(px, py, Math.max(4, (target.diameter / 2) * t.scale), 0,
            Math.PI * 2);
    ctx.fillStyle = SWATCH[target.colour] || "#888";
    ctx.globalAlpha = chosen ? 1 : 0.55;
    ctx.fill();
    ctx.globalAlpha = 1;
    if (chosen) {
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(px, py, Math.max(9, (target.diameter / 2) * t.scale + 5), 0,
              Math.PI * 2);
      ctx.stroke();
    }
  });

  if (world.home) {
    const [hx, hy] = toPixels(t, world.home.x, world.home.y);
    ctx.strokeStyle = "rgba(62, 207, 108, 0.6)";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(hx - 7, hy); ctx.lineTo(hx + 7, hy);
    ctx.moveTo(hx, hy - 7); ctx.lineTo(hx, hy + 7);
    ctx.stroke();
  }
}

function drawGrid(t) {
  ctx.strokeStyle = "#1a1f28";
  ctx.lineWidth = 1;
  for (let m = -10; m <= 10; m++) {
    const [x1, y1] = toPixels(t, m, -10);
    const [x2, y2] = toPixels(t, m, 10);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    const [x3, y3] = toPixels(t, -10, m);
    const [x4, y4] = toPixels(t, 10, m);
    ctx.beginPath(); ctx.moveTo(x3, y3); ctx.lineTo(x4, y4); ctx.stroke();
  }
}

function drawLidar(t, pose) {
  if (!lidar || !lidar.ranges || !lidar.ranges.length) { return; }
  ctx.fillStyle = "rgba(255, 140, 42, 0.75)";
  lidar.ranges.forEach((range, i) => {
    if (range === null) { return; }          // no return: draw a gap
    const angle = pose.yaw + lidar.angle_min + i * lidar.angle_step;
    const [px, py] = toPixels(
      t, pose.x + range * Math.cos(angle), pose.y + range * Math.sin(angle));
    ctx.fillRect(px - 1.5, py - 1.5, 3, 3);
  });
}

function drawPath(t, path) {
  if (!path.length) { return; }
  ctx.strokeStyle = "#3ecf6c";
  ctx.lineWidth = 2;
  ctx.beginPath();
  path.forEach(([x, y], i) => {
    const [px, py] = toPixels(t, x, y);
    if (i === 0) { ctx.moveTo(px, py); } else { ctx.lineTo(px, py); }
  });
  ctx.stroke();
}

function drawRobot(t, pose) {
  const [px, py] = toPixels(t, pose.x, pose.y);
  // 0.25 m is the collision monitor's stop radius and the local costmap's
  // robot_radius (C2-NAV.48/49). Drawing it is the difference between
  // "why did it stop there" and "ah, that is its footprint".
  ctx.strokeStyle = "rgba(255, 140, 42, 0.35)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(px, py, 0.25 * t.scale, 0, Math.PI * 2);
  ctx.stroke();

  ctx.save();
  ctx.translate(px, py);
  ctx.rotate(-pose.yaw);
  ctx.fillStyle = "#ff8c2a";
  ctx.beginPath();
  ctx.moveTo(11, 0); ctx.lineTo(-7, 7); ctx.lineTo(-7, -7);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

// Click to navigate. Only in Auto: sending a goal while the operator is
// driving by hand is a surprise, and teleop preempts it anyway.
canvas.addEventListener("click", (event) => {
  if (uiMode !== "auto") {
    toast("Switch to Auto to send COCO somewhere");
    return;
  }
  const rect = canvas.getBoundingClientRect();
  const px = (event.clientX - rect.left) * (canvas.width / rect.width);
  const py = (event.clientY - rect.top) * (canvas.height / rect.height);
  const t = transform();
  const x = (px - t.ox) / t.scale;
  const y = (t.oy - py) / t.scale;
  send({ type: "nav_goal", x, y });
  toast(`Going to ${x.toFixed(2)}, ${y.toFixed(2)}`);
});

// ── interface mode ────────────────────────────────────────────────────
$("uiPlay").onclick = () => setUiMode("play");
$("uiEng").onclick = () => setUiMode("engineering");

function setUiMode(mode) {
  document.body.dataset.uiMode = mode;
  $("uiPlay").classList.toggle("on", mode === "play");
  $("uiEng").classList.toggle("on", mode === "engineering");
  $("uiPlay").setAttribute("aria-selected", String(mode === "play"));
  $("uiEng").setAttribute("aria-selected", String(mode === "engineering"));
  // Depth is an engineering view. Leaving it subscribed in Play mode
  // would keep a ROS subscription and a JPEG encoder alive for a pane
  // nobody can see.
  if (mode === "play" && $("depthOn").checked) {
    $("depthOn").checked = false;
    syncStreams();
  }
  try { localStorage.setItem("coco.uiMode", mode); } catch { /* private mode */ }
}

try {
  const saved = localStorage.getItem("coco.uiMode");
  if (saved) { setUiMode(saved); }
} catch { /* private mode: keep the default */ }

// ── toast ─────────────────────────────────────────────────────────────
let toastTimer = null;
function toast(message, bad) {
  const node = $("toast");
  node.textContent = message;
  node.className = "toast" + (bad ? " bad" : "");
  node.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { node.hidden = true; }, 3200);
}

setConnState("CONNECTING");
connect();
