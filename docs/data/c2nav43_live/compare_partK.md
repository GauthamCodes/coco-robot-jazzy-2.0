### Per run

| arm | run | legs | ordinary | entry | exit | tour sim s | min true clear m | STOP n / s | deadlocks | timeouts | bypass rows | exceeded | stale drops | loop misses | phantom cells (grids) | ramp cells max | Nav2 cores | depth cores | readback |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| baseline_lidar_only | `baseline_lidar_only_r01` | 6/7 | 5/5 | 1/1 | 0/1 | 313.22 | 0.2452 | 1 / 70.17 | ['enclosure_exit'] | 1 | 0 | 3/3121 | 0 | 8 | 7567 (283/625) | 76 | 1.027 | None | 13ok/13ok/1ok |
| baseline_lidar_only | `baseline_lidar_only_r02` | 5/7 | 5/5 | 0/1 | 0/1 | 251.14 | 0.2459 | 1 / 63.19 | ['enclosure_entry', 'enclosure_exit'] | 2 | 0 | 7/1734 | 0 | 5 | 2003 (106/521) | 142 | 1.196 | None | 13ok/13ok/1ok |
| baseline_lidar_only | `baseline_lidar_only_r04` | 5/7 | 5/5 | 0/1 | 0/1 | 258.41 | 0.2449 | 1 / 48.04 | ['enclosure_entry', 'enclosure_exit'] | 2 | 0 | 7/1882 | 0 | 2 | 5868 (171/634) | 101 | 1.906 | None | 13ok/13ok/1ok |
| depth_fusion | `depth_fusion_r01` | 6/7 | 5/5 | 1/1 | 0/1 | 228.56 | 0.2454 | 1 / 70.16 | ['enclosure_exit'] | 1 | 0 | 2/2285 | 0 | 6 | 9953 (230/498) | 494 | 1.139 | 0.119 | 23ok/13ok/3ok |
| depth_fusion | `depth_fusion_r02` | 6/7 | 5/5 | 1/1 | 0/1 | 215.87 | 0.2434 | 1 / 70.4 | ['enclosure_exit'] | 1 | 2 | 63/2161 | 0 | 3 | 12810 (246/491) | 454 | 1.195 | 0.125 | 23ok/13ok/3ok |
| depth_fusion | `depth_fusion_r03` | 6/7 | 5/5 | 0/1 | 1/1 | 253.41 | 0.2658 | 0 / 0.0 | -- | 1 | 0 | 12/2535 | 0 | 2 | 27305 (359/665) | 588 | 1.891 | 0.176 | 23ok/13ok/3ok |

### Per arm

| arm | legs | ordinary | entry | exit | median tour sim s | min true clear m | STOP n / s | deadlocks | timeouts | goal err med (succ) m | abs yaw err med (succ) rad | entry abs yaw err rad | bypass rows | exceeded | STOP rows driven | phantom cells (grids) | Nav2 cores med | depth cores med | controller / costmap missed-rate lines |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| baseline_lidar_only | 16/21 | 15/15 | 1/3 | 0/3 | 258.41 | 0.2449 | 3 / 181.4 | ['enclosure_exit', 'enclosure_entry', 'enclosure_exit', 'enclosure_entry', 'enclosure_exit'] | 5 | 0.088 | 0.421 | [0.493, 2.556, 2.364] | 0 | 17/6737 | 25/1813 | 15438 (560/1780) | 1.196 | None | 13 / 0 |
| depth_fusion | 18/21 | 15/15 | 2/3 | 1/3 | 228.56 | 0.2434 | 2 / 140.56 | ['enclosure_exit', 'enclosure_exit'] | 3 | 0.087 | 0.424 | [0.487, 0.508, 2.217] | 2 | 77/6981 | 15/1406 | 50068 (835/1654) | 1.195 | 0.125 | 8 / 0 |

### Phantom local-costmap cells by sensor view (runs recorded after 02cd725)

| arm | runs | grids | in camera view: cells (grids) | in LiDAR view: cells (grids) | in neither: cells (grids) |
|---|---|---|---|---|---|
| baseline_lidar_only | 2 | 1155 | 1124 (42) | 3584 (148) | 4287 (224) |
| depth_fusion | 2 | 1156 | 5896 (193) | 21999 (479) | 18116 (539) |

### Per leg

| leg | arm | succeeded | median s | goal err m | abs yaw err rad | true clear m | PolygonStop s (legs) | DWB zero-vx | cmd<0.05 |
|---|---|---|---|---|---|---|---|---|---|
| `open_space` | baseline_lidar_only | 3/3 | 19.94 | 0.12 | 0.408 | 0.5155 | 0.00 (0) | 0.286 | 0.479 |
| `open_space` | depth_fusion | 3/3 | 15.83 | 0.117 | 0.424 | 0.4908 | 0.00 (0) | 0.244 | 0.333 |
| `wall_adjacent` | baseline_lidar_only | 3/3 | 35.58 | 0.089 | 0.429 | 0.3691 | 0.00 (0) | 0.418 | 0.702 |
| `wall_adjacent` | depth_fusion | 3/3 | 19.32 | 0.115 | 0.424 | 0.4020 | 0.00 (0) | 0.606 | 0.756 |
| `wall_parallel` | baseline_lidar_only | 3/3 | 24.22 | 0.093 | 0.218 | 0.3844 | 0.00 (0) | 0.177 | 0.4 |
| `wall_parallel` | depth_fusion | 3/3 | 17.39 | 0.082 | 0.191 | 0.4664 | 0.00 (0) | 0.139 | 0.317 |
| `obstacle_corner` | baseline_lidar_only | 3/3 | 18.39 | 0.087 | 0.485 | 0.5023 | 0.00 (0) | 0.183 | 0.327 |
| `obstacle_corner` | depth_fusion | 3/3 | 18.85 | 0.087 | 0.415 | 0.4745 | 0.00 (0) | 0.218 | 0.352 |
| `corridor_gate` | baseline_lidar_only | 3/3 | 25.91 | 0.048 | 0.406 | 0.4247 | 0.00 (0) | 0.349 | 0.53 |
| `corridor_gate` | depth_fusion | 3/3 | 25.39 | 0.05 | 0.459 | 0.4125 | 0.00 (0) | 0.286 | 0.359 |
| `enclosure_entry` | baseline_lidar_only | 1/3 | 69.17 | 0.913 | 2.364 | 0.2449 | 111.23 (2) | 0.049 | 0.284 |
| `enclosure_entry` | depth_fusion | 2/3 | 57.83 | 0.094 | 0.508 | 0.2922 | 0.00 (0) | 0.325 | 0.508 |
| `enclosure_exit` | baseline_lidar_only | 0/3 | 77.18 | 2.846 | 2.364 | 0.2449 | 70.17 (1) | 0.0 | 0.209 |
| `enclosure_exit` | depth_fusion | 1/3 | 77.33 | 3.126 | 1.777 | 0.2434 | 140.56 (2) | 0.015 | 0.275 |
