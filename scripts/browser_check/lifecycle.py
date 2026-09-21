"""
Connection lifecycle in a real browser: sim-not-up, frozen server,
recovery, server restart. Drives the fake stack process itself.

Usage: python3 lifecycle.py <repo> <overlay> <outdir>
"""
import asyncio
import json
import os
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bidi import Bidi, launch  # noqa: E402

REPO, WS, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)
HERE = os.path.dirname(os.path.abspath(__file__))
ENVRUN = os.path.join(HERE, 'envrun.sh')
PORT = 8091
URL = f'http://127.0.0.1:{PORT}/'

READ = '''(() => { const t = (id) => { const e = document.getElementById(id);
  return e ? (e.hidden ? '[hidden]' : e.textContent.trim()) : '[missing]'; };
  const s = document.getElementById('estop'); const r = s.getBoundingClientRect();
  const hit = document.elementFromPoint(r.left + r.width / 2, r.top + r.height / 2);
  return { conn: t('connState'), health: t('healthChip'), missing: t('missingChip'),
    waiting: t('waiting') === '[hidden]' ? 'hidden' : t('waitingWhat'),
    why: t('waitingWhy'), session: t('sessionRead'), toast: t('toast'),
    stale: document.body.classList.contains('stale'),
    stopReachable: hit === s }; })()'''


def stack(mode=''):
    args = [ENVRUN, REPO, WS, 'python3', os.path.join(HERE, 'fakestack.py'),
            REPO, str(PORT)] + ([mode] if mode else [])
    return subprocess.Popen(args, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)


def wait_port(up=True, timeout=20):
    import socket
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            socket.create_connection(('127.0.0.1', PORT), 0.3).close()
            if up:
                return True
        except OSError:
            if not up:
                return True
        time.sleep(0.2)
    return False


async def main():
    rows = []
    srv = stack('nosim')
    assert wait_port()
    ff = launch(port=9224)
    try:
        b = await Bidi.connect(port=9224)
        await b.cmd('session.new', capabilities={})
        ctx = (await b.cmd('browsingContext.getTree'))['contexts'][0]['context']
        await b.cmd('browsingContext.setViewport', context=ctx,
                    viewport={'width': 1400, 'height': 1000})
        await b.cmd('browsingContext.navigate', context=ctx, url=URL,
                    wait='complete')

        async def snap(name):
            row = {'step': name, 't': round(time.time(), 1),
                   **await b.eval(ctx, READ)}
            rows.append(row)
            await b.shot(ctx, os.path.join(OUT, f'{name}.png'))
            return row

        await asyncio.sleep(2.5)
        await snap('a_sim_not_up')

        # Swap to a live-sim server (new process = new session id).
        os.killpg(srv.pid, signal.SIGTERM)
        wait_port(up=False)
        await asyncio.sleep(0.5)
        await snap('b_server_gone')
        srv = stack()
        assert wait_port()
        await asyncio.sleep(4)
        first = await snap('c_server_back_new_session')

        # Freeze the server: TCP stays open, nothing is sent. Only the
        # page's own heartbeat can notice.
        os.killpg(srv.pid, signal.SIGSTOP)
        frozen_at = time.time()
        await asyncio.sleep(2.0)
        await snap('d_frozen_2s')
        await asyncio.sleep(3.5)
        row = await snap('e_frozen_5_5s')
        row['seconds_frozen'] = round(time.time() - frozen_at, 1)
        os.killpg(srv.pid, signal.SIGCONT)
        await asyncio.sleep(6)
        after = await snap('f_thawed')
        rows.append({'same_session_after_thaw':
                     after['session'] == first['session']})

        # A required component lost AFTER convergence: ERROR, not
        # "starting" -- and STOP must still be reachable over the curtain.
        os.killpg(srv.pid, signal.SIGTERM)
        wait_port(up=False)
        srv = stack('lose')
        assert wait_port()
        await asyncio.sleep(4)
        await snap('g_converged_before_loss')
        await asyncio.sleep(7)
        await snap('h_arbiter_lost_after_convergence')
        await b.cmd('session.end')
    finally:
        os.killpg(ff.pid, signal.SIGTERM)
        try:
            os.killpg(srv.pid, signal.SIGCONT)
            os.killpg(srv.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    print(json.dumps(rows, indent=1))


asyncio.run(main())
