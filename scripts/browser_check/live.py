# Copyright 2026 Gautham Anil -- Apache-2.0.
"""
The live end-to-end: a real browser drives a real COCO simulation.

    python3 live.py <url> <outdir> <recorder.jsonl> [colour]

Every action is a real key press or pointer click on the shipped page --
nothing is sent over the socket by this script. Writes <outdir>/actions.json
(wall-clock timeline) and screenshots; wheel_recorder.py supplies what
reached the wheels, and analyse_live.py joins the two.

Order is deliberate: manual driving and the two stops happen first and
the robot is driven back towards home after each, so the mission starts
near its spawn; the disconnect test kills the browser, so a new one is
launched for the mission.
"""

import asyncio
import json
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bidi import Bidi, launch  # noqa: E402

URL, OUT, REC = sys.argv[1], sys.argv[2], sys.argv[3]
COLOUR = sys.argv[4] if len(sys.argv) > 4 else 'green'
os.makedirs(OUT, exist_ok=True)
ACTIONS = []

READ = '''(() => { const t = (id) => { const e = document.getElementById(id);
  return e ? (e.hidden ? '[hidden]' : e.textContent.trim()) : '[missing]'; };
  return { conn: t('connState'), health: t('healthChip'),
    missing: t('missingChip'), badge: t('missionBadge'),
    target: t('missionTarget'), phase: t('missionPhase'),
    step: t('missionStep'), detail: t('missionDetail'), why: t('missionWhy'),
    pose: t('poseRead'), vel: t('velRead'), src: t('srcRead'),
    map: t('mapRead'), cam: t('camStatus'), depth: t('depthStatus'),
    lidar: t('lidarRead'), lifecycle: t('lifecycleRead'),
    healthRead: t('healthRead'), degraded: t('degradedRead'),
    mState: t('mState'), mReason: t('mReason'), mResult: t('mResult'),
    rtt: t('rttRead'), waiting: t('waiting') === '[hidden]' ? 'hidden'
      : t('waitingWhat') }; })()'''


# Is STOP the element a click at its own centre would hit? The first real
# render found it covered by the "COCO is starting" curtain.
STOPHIT = '''(() => { const s = document.getElementById('estop');
  const r = s.getBoundingClientRect();
  const hit = document.elementFromPoint(r.left + r.width / 2,
                                        r.top + r.height / 2);
  return { stopHit: !!hit && (hit === s || s.contains(hit)),
           visible: r.width > 0 && r.height > 0 && !s.hidden,
           curtain: !document.getElementById('waiting').hidden }; })()'''

# The retained MJPEG view, now served by the platform at /video/annotated.
VISION = '''(() => { const i = document.getElementById('vision');
  let path = ''; try { path = new URL(i.src).pathname; } catch (e) {}
  return { path: path, loaded: !i.hidden && i.naturalWidth > 0,
           w: i.naturalWidth, h: i.naturalHeight,
           status: document.getElementById('visionStatus').textContent }; })()'''


def act(name, **extra):
    row = {'t': round(time.time(), 4), 'action': name, **extra}
    ACTIONS.append(row)
    print(json.dumps(row), flush=True)
    with open(os.path.join(OUT, 'actions.json'), 'w') as handle:
        json.dump(ACTIONS, handle, indent=1)
    return row


def localised():
    """Read the recorder's last graph row: is map -> odom available?"""
    try:
        with open(REC) as handle:
            rows = [json.loads(line) for line in handle if '"graph"' in line]
        return bool(rows) and rows[-1].get('localised', False)
    except (OSError, ValueError):
        return False


async def open_page():
    proc = launch(width=1400, height=1000)
    b = await Bidi.connect()
    await b.cmd('session.new', capabilities={})
    await b.cmd('session.subscribe', events=['log.entryAdded'])
    ctx = (await b.cmd('browsingContext.getTree'))['contexts'][0]['context']
    await b.cmd('browsingContext.setViewport', context=ctx,
                viewport={'width': 1400, 'height': 1000})
    await b.cmd('browsingContext.navigate', context=ctx, url=URL,
                wait='complete')
    return proc, b, ctx


async def wait_for(b, ctx, key, predicate, timeout, every=0.5):
    t0 = time.time()
    while time.time() - t0 < timeout:
        state = await b.eval(ctx, READ)
        if predicate(state.get(key, '')):
            return state, round(time.time() - t0, 2)
        await asyncio.sleep(every)
    return await b.eval(ctx, READ), None


async def unfocus(b, ctx):
    # Key handlers ignore keys aimed at an <input>; after a checkbox click
    # the focus is on one.
    await b.eval(ctx, 'document.activeElement && document.activeElement.blur(); 1')


async def hold(b, ctx, key, seconds, label):
    await unfocus(b, ctx)
    act(f'{label}_down', key=key)
    await b.keys(ctx, [{'type': 'keyDown', 'value': key},
                       {'type': 'pause', 'duration': int(seconds * 1000)},
                       {'type': 'keyUp', 'value': key}])
    act(f'{label}_up', key=key)


async def joystick(b, ctx, dx, dy, seconds, label):
    """Drag the nipplejs pad from its centre by (dx, dy) px, hold, release."""
    box = await b.eval(ctx, '''(() => { const e = document.getElementById('joy');
      e.scrollIntoView({block: "center"});
      const r = e.getBoundingClientRect();
      return [r.left + r.width / 2, r.top + r.height / 2]; })()''')
    x, y = int(box[0]), int(box[1])
    act(f'{label}_down', dx=dx, dy=dy)
    await b.cmd('input.performActions', context=ctx, actions=[{
        'type': 'pointer', 'id': 'joy', 'parameters': {'pointerType': 'mouse'},
        'actions': [
            {'type': 'pointerMove', 'x': x, 'y': y},
            {'type': 'pointerDown', 'button': 0},
            {'type': 'pointerMove', 'x': x + dx // 2, 'y': y + dy // 2,
             'duration': 100},
            {'type': 'pointerMove', 'x': x + dx, 'y': y + dy,
             'duration': 100},
            {'type': 'pause', 'duration': int(seconds * 1000)},
            {'type': 'pointerUp', 'button': 0}]}])
    act(f'{label}_up')
    await b.cmd('input.releaseActions', context=ctx)


async def errors(b):
    found = []
    for event in b.events:
        if event.get('method') == 'log.entryAdded':
            entry = event['params']
            if entry.get('level') == 'error' or entry.get('type') == 'javascript':
                found.append(entry.get('text'))
    return found


async def main():
    proc, b, ctx = await open_page()
    try:
        # STOP must be clickable over the "starting" curtain, too.
        act('stop_hit_at_open', **(await b.eval(ctx, STOPHIT)),
            conn=(await b.eval(ctx, READ))['conn'])
        state, took = await wait_for(
            b, ctx, 'conn', lambda v: v in ('Ready', 'Mission running'), 420)
        act('page_ready', seconds=took, state=state)
        act('stop_hit_ready', **(await b.eval(ctx, STOPHIT)))
        await b.shot(ctx, os.path.join(OUT, '01_ready.png'))
        await b.click_id(ctx, 'uiEng')
        vision, took = None, None
        t0 = time.time()
        while time.time() - t0 < 20:
            vision = await b.eval(ctx, VISION)
            if vision['loaded']:
                took = round(time.time() - t0, 2)
                break
            await asyncio.sleep(0.5)
        act('annotated_mjpeg', seconds=took, **vision)
        await b.click_id(ctx, 'uiPlay')

        state, took = await wait_for(b, ctx, 'lidar',
                                     lambda v: 'rays' in v, 30)
        act('telemetry_and_lidar', seconds=took, pose=state['pose'],
            lidar=state['lidar'], map=state['map'])

        await b.click_id(ctx, 'camOn')
        act('camera_subscribe_click')
        state, took = await wait_for(b, ctx, 'cam',
                                     lambda v: v.startswith('camera: '), 30,
                                     every=0.1)
        act('camera_first_frame', seconds=took, cam=state['cam'])

        await b.click_id(ctx, 'uiEng')
        await b.click_id(ctx, 'depthOn')
        act('depth_subscribe_click')
        state, took = await wait_for(b, ctx, 'depth',
                                     lambda v: v.startswith('depth: '), 30,
                                     every=0.1)
        act('depth_first_frame', seconds=took, depth=state['depth'])
        await b.eval(ctx, 'window.scrollTo(0, 0); 1')
        await b.shot(ctx, os.path.join(OUT, '02_engineering_live.png'))
        await b.click_id(ctx, 'uiPlay')
        await asyncio.sleep(1.0)
        act('back_to_play', depth=(await b.eval(ctx, READ))['depth'])

        # Manual driving: out and back.
        await b.click_id(ctx, 'modeTeleop')
        await asyncio.sleep(0.5)
        await hold(b, ctx, 'w', 2.0, 'drive_forward')
        await asyncio.sleep(1.5)
        await hold(b, ctx, 's', 2.0, 'drive_back')
        await asyncio.sleep(1.5)
        await b.shot(ctx, os.path.join(OUT, '03_after_manual.png'))

        # The joystick: a real pointer drag on the nipplejs pad, up
        # (forward), held, released; then back down to where it started.
        await joystick(b, ctx, 0, -45, 1.5, 'joy_forward')
        await asyncio.sleep(1.5)
        await joystick(b, ctx, 0, 45, 1.5, 'joy_back')
        await asyncio.sleep(1.5)

        # STOP with W still held by the other hand.
        await unfocus(b, ctx)
        act('stop_test_w_down')
        await b.keys(ctx, [{'type': 'keyDown', 'value': 'w'}])
        await asyncio.sleep(1.5)
        await b.click_id(ctx, 'estop')
        act('stop_clicked_w_still_held')
        await asyncio.sleep(2.5)
        act('stop_test_w_up')
        await b.keys(ctx, [{'type': 'keyUp', 'value': 'w'}])
        await b.cmd('input.releaseActions', context=ctx)
        await asyncio.sleep(1.0)
        await b.click_id(ctx, 'modeTeleop')
        await hold(b, ctx, 's', 1.5, 'return_after_stop')
        await asyncio.sleep(1.5)
        act('js_errors_before_disconnect', errors=await errors(b))

        # Disconnect while driving: kill the browser outright.
        await unfocus(b, ctx)
        act('disconnect_test_w_down')
        await b.keys(ctx, [{'type': 'keyDown', 'value': 'w'}])
        await asyncio.sleep(1.0)
        os.killpg(proc.pid, signal.SIGKILL)
        act('browser_killed_while_driving')
    except Exception as exc:          # noqa: BLE001
        act('error', error=repr(exc))
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        raise
    await asyncio.sleep(4.0)

    # A new browser for the mission.
    proc, b, ctx = await open_page()
    try:
        state, took = await wait_for(
            b, ctx, 'conn', lambda v: v in ('Ready', 'Mission running'), 60)
        act('reconnected_new_browser', seconds=took)
        await b.click_id(ctx, 'modeTeleop')
        await hold(b, ctx, 's', 1.0, 'return_after_disconnect')
        await asyncio.sleep(1.5)

        t0 = time.time()
        while not localised() and time.time() - t0 < 180:
            await asyncio.sleep(1.0)
        act('localised', ok=localised(), waited=round(time.time() - t0, 1))

        await b.eval(ctx, 'window.scrollTo(0, 0); 1')
        box = await b.eval(ctx, (
            f'(() => {{ const e = document.querySelector('
            f'\'#targets button[data-colour="{COLOUR}"]\');'
            ' e.scrollIntoView({block: "center"});'
            ' const r = e.getBoundingClientRect();'
            ' return [r.left + r.width / 2, r.top + r.height / 2]; })()'))
        await b.click(ctx, box[0], box[1])
        act('colour_clicked', colour=COLOUR)
        state, took = await wait_for(
            b, ctx, 'target', lambda v: COLOUR in v, 10)
        act('colour_confirmed', target=state['target'], seconds=took)
        await asyncio.sleep(1.0)
        # Record every state the PAGE renders, with the page's own clock.
        # Polling from here every 250 ms misses a state shorter than that
        # (run 1 missed LOCALIZE); a MutationObserver misses nothing. It
        # only reads the DOM -- it sends nothing to the server.
        await b.eval(ctx, '''(() => {
          window.__states = [];
          const cell = document.getElementById('mState');
          const badge = document.getElementById('missionBadge');
          const note = () => window.__states.push({t: Date.now() / 1000,
            state: cell.textContent, badge: badge.textContent});
          note();
          new MutationObserver(note).observe(cell,
            {childList: true, characterData: true, subtree: true});
          return 1; })()''')
        await b.click_id(ctx, 'missionStart')
        act('mission_start_clicked')

        seen, last = [], None
        t0 = time.time()
        shots = 0
        at_running_checked = False
        while time.time() - t0 < 900:
            s = await b.eval(ctx, READ)
            key = (s['badge'], s['phase'], s['mState'])
            if key != last:
                last = key
                row = act('mission_view', badge=s['badge'], phase=s['phase'],
                          state=s['mState'], step=s['step'],
                          detail=s['detail'], why=s['why'],
                          target=s['target'], conn=s['conn'],
                          health=s['health'])
                seen.append(row)
                if shots < 12:
                    shots += 1
                    await b.shot(ctx, os.path.join(
                        OUT, f'm{shots:02d}_{s["mState"]}.png'))
                if s['conn'] == 'Mission running' and \
                        not at_running_checked:
                    at_running_checked = True
                    act('stop_hit_mission_running',
                        **(await b.eval(ctx, STOPHIT)))
            if s['badge'] in ('COMPLETED', 'FAILED', 'STOPPED') \
                    and time.time() - t0 > 5:
                break
            await asyncio.sleep(0.25)
        final = await b.eval(ctx, READ)
        act('mission_end', final=final, seconds=round(time.time() - t0, 1))
        act('page_state_log', states=await b.eval(ctx, 'window.__states'))
        await b.shot(ctx, os.path.join(OUT, '09_mission_end.png'))
        act('js_errors_mission', errors=await errors(b))
        await b.cmd('session.end')
    finally:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass


asyncio.run(main())
