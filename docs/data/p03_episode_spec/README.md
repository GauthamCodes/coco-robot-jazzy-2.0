# Episode specification evidence — 2026-09-24

`episode_evidence.txt` is the output of `episode_evidence.sh` at `16e575b`:

- the same seed, generated twice, gives **byte-identical** manifests
  (sha256 shown) at every level;
- different seeds give different layouts (`positions`) and different lane
  orders (`colours`);
- the derived envelope in numbers;
- **10000 seeds per level, every one passing `validate_episode`**;
- the robot's entire view of one episode:
  `{'episode_id': …, 'requested_colour': …}`.

`corridor_count.sh` counts episodes where one cylinder stands in another's
approach corridor. Its output, measured:

| generator | fixed | colours | positions |
|---|---|---|---|
| `f971078` (before the corridor rule) | 0 / 10000 | 0 / 10000 | **1012 / 10000** (276 blocking the requested target) |
| `16e575b` (after) | 0 / 10000 | 0 / 10000 | **0 / 10000** |

Both scripts expect a COCO overlay (`COCO_WS=$HOME/coco_ws_build`) and run
from the `coco_sim` package directory, so the source `coco_sim` shadows
the installed one.

**Not verified:** none of these episodes has been spawned in Gazebo or
driven. The `colours` and `positions` levels are geometrically valid only.
