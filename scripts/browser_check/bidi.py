# Copyright 2026 Gautham Anil -- Apache-2.0.
"""
Minimal WebDriver BiDi client for headless Firefox, on tornado.

Why this exists: P0.2's first pass could not drive a browser (no Chrome
extension on the development machine), so the page was covered only by
static asset tests -- and the first real render found the not-ready
curtain permanently drawn over the STOP button. Firefox (>= 129) speaks
WebDriver BiDi natively over a WebSocket, and tornado is already a
dependency, so no Selenium, Playwright or driver binary is needed.

The profile lives under $HOME, not /tmp or a dot-directory: the snap
Firefox on Ubuntu cannot read either.
"""
import asyncio
import base64
import json
import os
import subprocess
import time

from tornado.websocket import websocket_connect

HERE = os.path.dirname(os.path.abspath(__file__))


class Bidi:
    def __init__(self, ws):
        self.ws, self.n, self.events = ws, 0, []

    @classmethod
    async def connect(cls, port=9223, timeout=40):
        t0 = time.time()
        while True:
            try:
                ws = await websocket_connect(f'ws://127.0.0.1:{port}/session')
                return cls(ws)
            except Exception:
                if time.time() - t0 > timeout:
                    raise
                await asyncio.sleep(0.5)

    async def cmd(self, method, **params):
        self.n += 1
        my = self.n
        await self.ws.write_message(json.dumps(
            {'id': my, 'method': method, 'params': params}))
        while True:
            raw = await self.ws.read_message()
            if raw is None:
                raise RuntimeError('bidi socket closed')
            msg = json.loads(raw)
            if msg.get('id') == my:
                if msg.get('type') == 'error':
                    raise RuntimeError(
                        f"{method}: {msg.get('error')}: {msg.get('message')}")
                return msg.get('result')
            self.events.append(msg)

    async def eval(self, ctx, expr, await_promise=True):
        r = await self.cmd(
            'script.evaluate', expression=expr, target={'context': ctx},
            awaitPromise=await_promise, resultOwnership='none',
            serializationOptions={'maxObjectDepth': 6})
        if r.get('type') == 'exception':
            raise RuntimeError(
                'JS: ' + json.dumps(r.get('exceptionDetails'))[:800])
        return unwrap(r['result'])

    async def shot(self, ctx, path):
        r = await self.cmd('browsingContext.captureScreenshot', context=ctx)
        with open(path, 'wb') as handle:
            handle.write(base64.b64decode(r['data']))

    async def keys(self, ctx, actions):
        await self.cmd('input.performActions', context=ctx, actions=[
            {'type': 'key', 'id': 'kb', 'actions': actions}])

    async def click(self, ctx, x, y):
        await self.cmd('input.performActions', context=ctx, actions=[{
            'type': 'pointer', 'id': 'mouse',
            'parameters': {'pointerType': 'mouse'},
            'actions': [
                {'type': 'pointerMove', 'x': int(x), 'y': int(y)},
                {'type': 'pointerDown', 'button': 0},
                {'type': 'pause', 'duration': 50},
                {'type': 'pointerUp', 'button': 0}]}])

    async def click_id(self, ctx, element_id):
        """Click the centre of an element, by id, as a real pointer."""
        box = await self.eval(ctx, (
            f'(() => {{ const e = document.getElementById({json.dumps(element_id)});'
            ' e.scrollIntoView({block: "center"});'
            ' const r = e.getBoundingClientRect();'
            ' return [r.left + r.width / 2, r.top + r.height / 2]; })()'))
        await self.click(ctx, box[0], box[1])
        return box


def unwrap(v):
    t = v.get('type')
    if t in ('string', 'number', 'boolean'):
        return v.get('value')
    if t in ('null', 'undefined'):
        return None
    if t == 'array':
        return [unwrap(x) for x in v.get('value', [])]
    if t == 'object':
        return {(k if isinstance(k, str) else unwrap(k)): unwrap(x)
                for k, x in v.get('value', [])}
    return v.get('value', t)


def launch(port=9223, width=1400, height=1000):
    # A FRESH profile every launch: a reused one serves style.css/app.js
    # from its cache and the browser tests yesterday's page.
    import shutil
    prof = os.path.expanduser('~/coco_ff_profile')
    shutil.rmtree(prof, ignore_errors=True)
    os.makedirs(prof, exist_ok=True)
    log = open(os.path.join(HERE, 'firefox.log'), 'w')
    return subprocess.Popen(
        ['firefox', '--headless', '--no-remote', '--profile', prof,
         f'--remote-debugging-port={port}',
         f'--window-size={width},{height}', 'about:blank'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
