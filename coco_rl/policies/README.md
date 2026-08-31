# Shipped ramp policy

`phase5_24deg_s0p0.zip` — the last stage of the shipped PPO curriculum, and
the policy every climb number in this project was measured with. It is
installed to `share/coco_rl/policies/` and is the default the mission launch
file loads, so a fresh clone climbs the ramp with no extra setup.

It is committed by an explicit exception in `.gitignore`, which otherwise
ignores `*.zip` to keep training artefacts out of the repository. This one
file is shipped because it is *part of the release*: without it the mission
drives to the ramp, discovers it has no policy, and stops.

To run a different policy instead:

```bash
ros2 launch coco_mission mission.launch.py policy:=/path/to/other.zip
```

`train_curriculum.sh` in the repository root reproduces the curriculum that
produced this file. See `docs/RESULTS.md` for what it was measured to do
(deterministic evaluation: **10/10** on both the 18° and the 24° grade).
