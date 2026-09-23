#!/usr/bin/env bash
# Physics-only Isaac loop WITHOUT the ROS 2 bridge, clean env, peak RSS.
TMP=/home/gautham/.claude/jobs/d213ad33/tmp
cd "$TMP" || exit 9
python3 - <<'EOF'
s = open('isaac_smoke5.py').read()
s = s.replace("isaac_stack5.txt", "isaac_stack6.txt")
a = s.index("try:\n    from isaacsim.core.utils.extensions import enable_extension")
b = s.index("app.close()")
loop = '''try:
    cube.set_world_pose(position=np.array([0.0, 0.0, 3.0]))
    stamp('NO BRIDGE: physics-only loop for 60 s wall')
    end = time.time() + 60
    n = 0
    zs = []
    while time.time() < end:
        world.step(render=False)
        zs.append(float(cube.get_world_pose()[0][2]))
        time.sleep(0.01)
        n += 1
    stamp('loop done: %d steps; cube z first %.3f min %.3f last %.3f'
          % (n, zs[0], min(zs), zs[-1]))
except Exception as e:  # noqa: BLE001
    import traceback
    traceback.print_exc()
    stamp('LOOP FAILED: %s: %s' % (type(e).__name__, e))
    rc = 7

'''
s = s[:a] + loop + s[b:]
open('isaac_smoke6.py', 'w').write(s)
EOF
echo "ros refs in script: $(grep -c 'ros2\|rclpy' isaac_smoke6.py)"
env -i HOME=/home/gautham PATH=/usr/bin:/bin \
  timeout 300 /usr/bin/time -v /home/gautham/isaac-sim/venv/bin/python -u \
  isaac_smoke6.py > isaac_smoke6.log 2>&1
echo "exit=$?"
grep -aE '^\[ *[0-9.]+s\] ' isaac_smoke6.log | grep -avE '\[ext:|Warning|startup' | tail -6
grep -aE 'LLVM|Maximum resident|Exit status|Elapsed \(wall' isaac_smoke6.log
