"""Does SimulationApp's DEFAULT start (viewport wait NOT bypassed) return?
PROBE_HEADLESS=1|0, PROBE_MULTIGPU=1|0."""
import os
import sys
import time

T0 = time.time()
HEADLESS = os.environ.get('PROBE_HEADLESS', '1') == '1'
MULTI = os.environ.get('PROBE_MULTIGPU', '1') == '1'
print('[%6.1fs] PROBE headless=%s multi_gpu=%s ICD=%s' % (
    0, HEADLESS, MULTI, os.environ.get('VK_ICD_FILENAMES')), flush=True)
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({'headless': HEADLESS, 'multi_gpu': MULTI, 'active_gpu': 0,
                     'width': 1280, 'height': 720})
print('[%6.1fs] PROBE SimulationApp RETURNED (viewport delivered a frame)' % (time.time() - T0), flush=True)
t = time.time()
for _ in range(120):
    app.update()
print('[%6.1fs] PROBE 120 updates in %.2fs' % (time.time() - T0, time.time() - t), flush=True)
if not HEADLESS:
    try:
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        p = os.path.join(os.environ.get('PROBE_LOG', '.'), 'gui_viewport.png')
        capture_viewport_to_file(get_active_viewport(), p)
        for _ in range(30):
            app.update()
        print('[%6.1fs] PROBE screenshot bytes=%s' % (time.time() - T0, os.path.getsize(p) if os.path.exists(p) else 0), flush=True)
    except Exception as e:  # noqa: BLE001
        print('PROBE screenshot failed: %s' % e, flush=True)
app.close()
sys.exit(0)
