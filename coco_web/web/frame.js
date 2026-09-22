// Copyright 2026 Gautham Anil — Apache-2.0.
//
// coco.v1 binary sensor frames, decoded and VALIDATED, for the page.
//
// Ported from Codex's standalone transport (coco_web/transport/client.mjs
// on codex/p02-hardening), which was not adopted wholesale: the page's
// own transport in app.js is the one a real browser has driven, and
// swapping it would void that evidence. What the page lacked, and this
// file supplies, is the decoder's strictness. app.js checked the magic,
// the version and the header length and then trusted the rest -- so a
// LiDAR header whose count outran its payload threw a RangeError out of
// the message handler, and a frame whose kind byte disagreed with its
// header was drawn as whichever the header claimed.
//
// Layout (binary.py): "COCO" | version 1 | kind | uint16 header length |
// JSON header | payload. Big-endian throughout.
//
// Loaded as a classic script by index.html (window.cocoFrame) and
// require()d by Node in test/frame_decode.test.js, against frames the
// Python encoder actually produced.

(function (root) {
  "use strict";

  const MAGIC = 0x434f434f;                       // "COCO"
  const KINDS = { 1: "lidar", 2: "camera", 3: "depth" };
  const MAX_HEADER = 8192;
  const MAX_PAYLOAD = 8 * 1024 * 1024;
  const MAX_SIDE = 8192;
  // A server that put a ROS target in sensor metadata has a bug; the page
  // refuses the frame rather than display something that names one.
  const ROS_KEYS = ["topic", "service", "message_type", "target"];

  const isInt = (v, min = 0, max = Number.MAX_SAFE_INTEGER) =>
    Number.isSafeInteger(v) && v >= min && v <= max;
  const isNum = (v) => typeof v === "number" && Number.isFinite(v);

  function fail(reason) { throw new Error(reason); }

  // Returns {stream, header, payload, ranges?}. Throws Error(reason) for
  // anything that is not a complete, self-consistent coco.v1 frame.
  function decodeFrame(buffer) {
    if (!(buffer instanceof ArrayBuffer)) { fail("not an ArrayBuffer"); }
    if (buffer.byteLength < 8) { fail("short frame"); }
    const view = new DataView(buffer);
    if (view.getUint32(0) !== MAGIC) { fail("bad magic"); }
    if (view.getUint8(4) !== 1) { fail("unknown version"); }
    const stream = KINDS[view.getUint8(5)];
    if (!stream) { fail("unknown kind"); }
    const headerLen = view.getUint16(6);
    const end = 8 + headerLen;
    if (headerLen > MAX_HEADER || end > buffer.byteLength) {
      fail("bad header length");
    }
    let header;
    try {
      header = JSON.parse(new TextDecoder("utf-8", { fatal: true })
        .decode(new Uint8Array(buffer, 8, headerLen)));
    } catch (e) { fail("header is not JSON"); }
    if (!header || typeof header !== "object" || Array.isArray(header)) {
      fail("header is not an object");
    }
    const payload = new Uint8Array(buffer, end);
    const size = payload.byteLength;
    if (size > MAX_PAYLOAD) { fail("payload too large"); }
    if (header.stream !== stream) { fail("kind and header disagree"); }
    if (!isInt(header.seq) || !isNum(header.t) || header.t < 0 ||
        !isInt(header.dropped === undefined ? 0 : header.dropped)) {
      fail("bad common metadata");
    }
    if (ROS_KEYS.some((k) => Object.prototype.hasOwnProperty.call(header, k))) {
      fail("ROS name in sensor metadata");
    }
    if (header.payload_bytes !== undefined &&
        header.payload_bytes !== size) {
      fail("payload length mismatch");
    }
    if (stream === "lidar") {
      if (!isInt(header.count, 0, 4 * 1024 * 1024) ||
          header.count * 2 !== size) {
        fail("ray count mismatch");
      }
      if (!isNum(header.angle_min) || !isNum(header.angle_step) ||
          header.scale !== 1000 || header.no_return !== 0) {
        fail("bad lidar metadata");
      }
      const rays = new DataView(buffer, end, size);
      const ranges = new Array(header.count);
      for (let i = 0; i < header.count; i++) {
        const mm = rays.getUint16(i * 2);
        // 0 is "no return" -- a gap, not a wall at zero metres.
        ranges[i] = mm === header.no_return ? null : mm / header.scale;
      }
      return { stream, header, payload, ranges };
    }
    if (!isInt(header.w, 1, MAX_SIDE) || !isInt(header.h, 1, MAX_SIDE) ||
        header.enc !== "jpeg" || size === 0) {
      fail("bad image metadata");
    }
    if (stream === "depth" && !(isNum(header.min_m) && isNum(header.max_m) &&
        header.min_m >= 0 && header.min_m < header.max_m)) {
      fail("bad depth range");
    }
    return { stream, header, payload };
  }

  const api = { decodeFrame, KINDS };
  if (typeof module !== "undefined" && module.exports) {
    module.exports = api;
  } else {
    root.cocoFrame = api;
  }
})(typeof self !== "undefined" ? self : this);
