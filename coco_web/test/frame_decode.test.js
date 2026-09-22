// Copyright 2026 Gautham Anil — Apache-2.0.
//
// The page's frame decoder (web/frame.js), run by Node's own test runner
// against frames the PYTHON encoder produced. Driven by
// test_frontend_frame.py, which writes the fixtures and passes their path
// in COCO_FRAME_FIXTURES. No npm, no dependencies.

"use strict";

const test = require("node:test");
const assert = require("node:assert");
const fs = require("node:fs");
const path = require("node:path");

const { decodeFrame } = require(path.join(__dirname, "..", "web", "frame.js"));
const fixtures = JSON.parse(
  fs.readFileSync(process.env.COCO_FRAME_FIXTURES, "utf8"));

function buffer(b64) {
  const bytes = Buffer.from(b64, "base64");
  return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.length);
}

test("every frame the server encodes decodes, to the same values", () => {
  for (const [name, spec] of Object.entries(fixtures.valid)) {
    const frame = decodeFrame(buffer(spec.blob));
    assert.strictEqual(frame.stream, spec.stream, name);
    assert.strictEqual(frame.header.seq, spec.seq, name);
    assert.strictEqual(frame.header.dropped, spec.dropped, name);
    assert.strictEqual(frame.payload.byteLength, spec.payload_bytes, name);
    if (spec.ranges) {
      assert.deepStrictEqual(frame.ranges, spec.ranges, name);
    }
  }
});

test("every malformed frame is refused, with a reason, never half-read", () => {
  for (const [name, b64] of Object.entries(fixtures.invalid)) {
    assert.throws(() => decodeFrame(buffer(b64)), Error, name);
  }
});

test("a non-ArrayBuffer is refused", () => {
  assert.throws(() => decodeFrame("COCO"), /ArrayBuffer/);
});
