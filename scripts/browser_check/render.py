"""
Load the COCO page in headless Firefox, exercise it, screenshot it.

Usage: python3 render.py <url> <outdir> [--width W --height H]
Prints a JSON report: JS errors, key DOM readouts per step.
"""
import asyncio
import json
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bidi import Bidi, launch  # noqa: E402

URL = sys.argv[1]
OUT = sys.argv[2]
W = int(sys.argv[3]) if len(sys.argv) > 3 else 1400
H = int(sys.argv[4]) if len(sys.argv) > 4 else 1000
os.makedirs(OUT, exist_ok=True)

READ = '''(() => { const t = (id) => { const e = document.getElementById(id);
  return e ? (e.hidden ? '[hidden]' : e.textContent.trim()) : '[missing]'; };
  return { conn: t('connState'), stepRead: t('missionStep'), health: t('healthChip'),
    missing: t('missingChip'), badge: t('missionBadge'),
    target: t('missionTarget'), phase: t('missionPhase'),
    why: t('missionWhy'), pose: t('poseRead'),
    map: t('mapRead'), cam: t('camStatus'), depth: t('depthStatus'),
    lifecycle: t('lifecycleRead'), healthRead: t('healthRead'),
    nav: t('navRead'), lidar: t('lidarRead'), camRate: t('camRate'),
    waiting: document.getElementById('waiting').hidden ? 'hidden' : t('waitingWhat'),
    mode: document.body.dataset.uiMode,
    stopHit: (() => { const s = document.getElementById('estop');
      const r = s.getBoundingClientRect();
      if (r.bottom < 0 || r.top > innerHeight) { return 'offscreen'; }
      const e = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2);
      return e === s ? 'STOP' : (e ? (e.id || e.className || e.tagName) : null); })(),
    stale: document.body.classList.contains('stale') }; })()'''


async def main():
    p = launch(port=9225, width=W, height=H)
    report = {'steps': [], 'errors': []}
    try:
        b = await Bidi.connect(port=9225)
        await b.cmd('session.new', capabilities={})
        await b.cmd('session.subscribe', events=['log.entryAdded'])
        tree = await b.cmd('browsingContext.getTree')
        ctx = tree['contexts'][0]['context']
        await b.cmd('browsingContext.setViewport', context=ctx,
                     viewport={'width': W, 'height': H})
        await b.cmd('browsingContext.navigate', context=ctx, url=URL,
                    wait='complete')
        await asyncio.sleep(3)

        async def step(name):
            report['steps'].append({'step': name, **await b.eval(ctx, READ)})
            await b.shot(ctx, os.path.join(OUT, f'{name}.png'))

        await b.eval(ctx, 'localStorage.clear(); 1')
        await step('01_play')
        await b.click_id(ctx, 'camOn')
        await asyncio.sleep(2)
        await step('02_play_camera')
        await b.click_id(ctx, 'uiEng')
        await asyncio.sleep(0.5)
        await b.click_id(ctx, 'depthOn')
        await asyncio.sleep(2)
        await b.eval(ctx, 'window.scrollTo(0, 0); 1')
        await step('03_engineering')
        await b.eval(ctx, 'window.scrollTo(0, document.body.scrollHeight); 1')
        await asyncio.sleep(0.5)
        await step('04_engineering_bottom')
        await b.eval(ctx, 'window.scrollTo(0, 0); 1')
        await b.click_id(ctx, 'uiPlay')
        await asyncio.sleep(1)
        await step('05_back_to_play')
        for event in b.events:
            if event.get('method') == 'log.entryAdded':
                entry = event['params']
                if entry.get('level') in ('error', 'warn') or \
                        entry.get('type') == 'javascript':
                    report['errors'].append(
                        {'level': entry.get('level'),
                         'text': entry.get('text'),
                         'source': (entry.get('stackTrace') or {})})
        await b.cmd('session.end')
    finally:
        os.killpg(p.pid, signal.SIGTERM)
    print(json.dumps(report, indent=1))


asyncio.run(main())
