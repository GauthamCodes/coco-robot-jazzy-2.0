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
let streams = null;
let seenWelcome = false;

function connect() {
  ws = new WebSocket(WS_URL);
  ws.onopen = () => {
    backoff = 500;
    setConn(true);
    send({ type: "hello", client: "coco-web-ui" });
  };
  ws.onclose = () => {
    setConn(false);
    setTimeout(connect, backoff);
    backoff = Math.min(backoff * 2, 10000);
  };
  ws.onerror = () => { /* onclose always follows; handled there. */ };
  ws.onmessage = (event) => {
    let frame;
    try { frame = JSON.parse(event.data); } catch { return; }
    handle(frame);
  };
}

function send(frame) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(frame));
    return true;
  }
  return false;
}

function setConn(up) {
  const pill = $("conn");
  pill.classList.toggle("on", up);
  pill.textContent = up ? "connected" : "reconnecting…";
  if (!up) {
    $("sessionState").textContent = "offline";
    $("sessionState").className = "chip";
  }
}

// ── inbound frames ────────────────────────────────────────────────────
function handle(frame) {
  switch (frame.type) {
    case "welcome":   onWelcome(frame); break;
    case "telemetry": onTelemetry(frame); break;
    case "map":       onMap(frame); break;
    case "pong":      onPong(frame); break;
    case "error":     onError(frame); break;
    case "ack":       break;
    default:          break;
  }
}

function onWelcome(frame) {
  seenWelcome = true;
  limits = frame.limits || {};
  streams = frame.streams || {};
  $("protoRead").textContent = `protocol ${frame.protocol}`;
  buildTargets(limits.colours || []);
  buildArm(limits);
  attachStreams(streams);
}

function onError(frame) {
  toast(frame.message || frame.code, true);
}

// ── telemetry ─────────────────────────────────────────────────────────
let lastTelemetry = null;

function onTelemetry(frame) {
  lastTelemetry = frame;
  const sess = (frame.platform && frame.platform.session) || {};
  const arb = (frame.platform && frame.platform.arbiter) || {};

  // session
  const chip = $("sessionState");
  chip.textContent = sess.state || "—";
  chip.className = "chip" + (sess.state === "ready" ? " ready" : "");
  const missing = sess.missing || [];
  const warn = $("missingChip");
  warn.hidden = missing.length === 0;
  warn.textContent = missing.length ? `waiting on ${missing.join(", ")}` : "";

  // robot
  const pose = frame.robot && frame.robot.pose;
  $("poseRead").textContent = pose
    ? `x ${pose.x.toFixed(2)}  y ${pose.y.toFixed(2)}  ` +
      `θ ${(pose.yaw * 180 / Math.PI).toFixed(0)}°`
    : "pose —";
  const vel = frame.robot && frame.robot.velocity;
  $("velRead").textContent = vel
    ? `${vel.linear.toFixed(2)} m/s · ${vel.angular.toFixed(2)} rad/s`
    : "— m/s";
  // Which source actually owns the wheels. The arbiter's own word for it,
  // translated: "nav" is what the operator calls Auto.
  const src = arb.active;
  $("srcRead").textContent = "wheels: " + (
    src === "teleop" ? "you" :
    src === "nav" ? "navigation" :
    src === "rl" ? "ramp policy" :
    src === "approach" ? "visual approach" :
    "idle");

  updateMission(frame.mission || {});
  updatePerception((frame.sensors && frame.sensors.perception) || {});
  updateComponents(sess.components || {});

  $("rawArbiter").textContent = "arbiter: " + (arb.raw || "offline");
  $("rawMission").textContent =
    "mission: " + ((frame.mission && frame.mission.raw) || "offline");
  $("rawPerception").textContent = "perception: " +
    ((frame.sensors && frame.sensors.perception &&
      frame.sensors.perception.raw) || "offline");
  $("seqRead").textContent = `seq ${frame.seq}`;

  draw(frame);
}

// ── mission ───────────────────────────────────────────────────────────
// The ordered phases, used only to turn a state name into a progress
// fraction. It is a PRESENTATION list: the executive owns the real state
// machine, and a state missing from here simply shows no progress rather
// than breaking the panel.
const PHASES = [
  "LOCALIZE", "NAVIGATE_TO_RAMP", "ALIGN_FOR_CLIMB", "CLIMB",
  "VERIFY_CLIMB", "SEARCH_TARGET", "STOW_ARM", "APPROACH_TARGET",
  "GRASP", "VERIFY_GRASP", "DESCEND", "RETURN_HOME", "PLACE",
  "VERIFY_PLACEMENT", "COMPLETE",
];

// Operator-facing wording for each state. The state name itself stays
// visible in Engineering mode; Play mode should read like a sentence.
const PHASE_WORDS = {
  IDLE: "No mission running",
  LOCALIZE: "Working out where it is",
  NAVIGATE_TO_RAMP: "Driving to the ramp",
  ALIGN_FOR_CLIMB: "Lining up with the ramp",
  CLIMB: "Climbing",
  VERIFY_CLIMB: "Checking it made it up",
  SEARCH_TARGET: "Looking for the cylinder",
  STOW_ARM: "Stowing the arm",
  APPROACH_TARGET: "Closing in on the cylinder",
  GRASP: "Grasping",
  VERIFY_GRASP: "Checking the grip",
  DESCEND: "Coming back down",
  RETURN_HOME: "Heading home",
  PLACE: "Putting it down",
  VERIFY_PLACEMENT: "Checking the placement",
  COMPLETE: "Done — cylinder delivered",
  ABORT: "Mission aborted",
  RECOVERY: "Recovering",
  RELOCALIZE: "Relocalising",
};

let missionActive = false;

function updateMission(mission) {
  missionActive = !!mission.active;
  const state = mission.state;
  $("missionPhase").textContent =
    PHASE_WORDS[state] || (state ? state : "Mission system offline");
  $("missionDetail").textContent = mission.detail
    ? mission.detail
    : (state && PHASE_WORDS[state] ? `state=${state}` : "");

  const index = PHASES.indexOf(state);
  $("progressBar").style.width =
    index >= 0 ? `${((index + 1) / PHASES.length) * 100}%` : "0";

  $("missionAbort").disabled = !missionActive;
  $("missionStart").disabled = missionActive || !selectedColour ||
    !mission.online;
  // The executive owns the arm during a run; offering the sliders then
  // would be offering a fight the operator cannot win.
  ["sh", "el", "gr"].forEach((id) => {
    $(id).disabled = missionActive || !limits || !limits.arm;
  });
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

// ── targets ───────────────────────────────────────────────────────────
// Swatches are drawn from the colour names the SERVER advertised, which
// come from coco_config's own table. The panel does not keep its own
// list of what COCO can fetch.
const SWATCH = {
  red: "#d91a1a", green: "#1ab326", blue: "#1a40d9", yellow: "#e5cc1a",
};
let selectedColour = null;

function buildTargets(colours) {
  const host = $("targets");
  host.innerHTML = "";
  colours.forEach((colour) => {
    const button = document.createElement("button");
    button.innerHTML = `<span class="sw" style="background:${
      SWATCH[colour] || "#888"}"></span>${colour}`;
    button.onclick = () => {
      selectedColour = colour;
      [...host.children].forEach(
        (b) => b.classList.toggle("on", b === button));
      send({ type: "select_target", colour });
      $("missionStart").disabled = missionActive;
    };
    host.appendChild(button);
  });
}

$("missionStart").onclick = () => send({ type: "mission", action: "start" });
$("missionAbort").onclick = () => send({ type: "mission", action: "abort" });

// ── drive ─────────────────────────────────────────────────────────────
let uiMode = "teleop";
let joyLin = 0, joyAng = 0, driving = false;
let pendingStop = 0;

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
  stopDriving();
  send({ type: "stop" });
  setMode("stop");
  toast("Stopped");
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
    stopDriving();
    send({ type: "stop" });
    return;
  }
  const key = KEYS[event.key];
  if (!key) { return; }
  event.preventDefault();
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

// ── camera ────────────────────────────────────────────────────────────
// MJPEG straight from web_video_server, not through the WebSocket. The
// server tells us the port and path; it never tells the browser a topic
// it could then publish to.
function attachStreams(all) {
  const bind = (imgId, statusId, stream, label) => {
    const img = $(imgId);
    if (!stream) { $(statusId).textContent = `${label}: not enabled`; return; }
    img.src = `http://${location.hostname}:${stream.port}${stream.path}`;
    img.onload = () => { $(statusId).textContent = `${label}: live`; };
    img.onerror = () => {
      $(statusId).textContent =
        `${label}: no stream on port ${stream.port}`;
    };
  };
  bind("cam", "camStatus", all.camera, "camera");
  bind("vision", "visionStatus", all.annotated, "perception");
}

// ── the world view ────────────────────────────────────────────────────
// One canvas draws the map, the LiDAR return, the plan and the robot in
// a shared world frame. Everything is in metres until the last moment.
const canvas = $("world");
const ctx = canvas.getContext("2d");
let mapImage = null;      // ImageBitmap-ish offscreen canvas
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

  if (mapImage && mapMeta) {
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

  const pose = frame.robot && frame.robot.pose;
  if ($("showPath").checked) { drawPath(t, (frame.nav && frame.nav.path) || []); }
  if (pose && $("showLidar").checked) {
    drawLidar(t, pose, (frame.sensors && frame.sensors.lidar) || null);
  }
  if (pose) { drawRobot(t, pose); }
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

function drawLidar(t, pose, lidar) {
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

connect();
