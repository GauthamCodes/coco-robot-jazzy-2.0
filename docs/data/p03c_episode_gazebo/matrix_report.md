| run | seed | episode_id | requested | region (frozen) | target x, y | spawn xy / dz err | result | reason | sim s | wall s | approach x / y | home err m | bypass | checks |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| fixed_red | 0 | ep-7916a30396de | red | lane_1 (lane_1) | 4.0500, -0.7500 | 0.000 mm / 0.002 mm | complete | -- | 149.102 | 304.0 | 0.1545 / -0.0000 | 0.119 | 0 | PASS |
| fixed_green | 0 | ep-e89ab8905964 | green | lane_2 (lane_2) | 4.0500, -0.2500 | 0.000 mm / 0.000 mm | complete | -- | 198.506 | 389.0 | 0.1544 / -0.0000 | 0.019 | 0 | PASS |
| fixed_blue | 0 | ep-be0f51920fb2 | blue | lane_3 (lane_3) | 4.0500, +0.2500 | 0.010 mm / 0.000 mm | complete | -- | 155.474 | 285.0 | 0.1542 / +0.0000 | 0.104 | 0 | FAIL |
| fixed_yellow | 0 | ep-83b636dd724b | yellow | lane_4 (lane_4) | 4.0500, +0.7500 | 0.000 mm / 0.000 mm | complete | -- | 318.594 | 767.0 | 0.1539 / -0.0000 | 0.058 | 0 | PASS |
| colours_s1 | 1 | ep-25c3b9083208 | yellow | lane_1 (lane_4) | 4.0500, -0.7500 | 0.000 mm / 0.000 mm | complete | -- | 153.582 | 297.0 | 0.1543 / -0.0000 | 0.075 | 0 | PASS |
| colours_s2 | 2 | ep-8236c0e9a885 | red | lane_4 (lane_1) | 4.0500, +0.7500 | 0.000 mm / 0.000 mm | complete | -- | 157.04 | 360.0 | 0.1545 / -0.0000 | 0.113 | 0 | PASS |
| colours_s4 | 4 | ep-216ea18e9365 | green | lane_4 (lane_2) | 4.0500, +0.7500 | 0.000 mm / 0.000 mm | failed | DESCENT_TIMEOUT | 146.516 | 278.0 | -- | 6.575 | 0 | PASS |
| positions_s1 | 1 | ep-2ba30b0dccf6 | yellow | lane_1 (lane_4) | 4.1441, -0.7503 | 0.005 mm / 0.000 mm | complete | -- | 164.08 | 309.0 | 0.1543 / -0.0000 | 0.082 | 0 | PASS |
| positions_s2 | 2 | ep-8ef4f54d7715 | red | lane_4 (lane_1) | 4.0634, +0.7609 | 0.005 mm / 0.000 mm | failed | TARGET_NOT_FOUND | 180.812 | 478.0 | -- | 0.087 | 0 | PASS |
| positions_s4 | 4 | ep-422c6f25f3a7 | green | lane_4 (lane_2) | 4.4186, +0.7235 | 0.006 mm / 0.000 mm | complete | -- | 318.658 | 625.0 | 0.1543 / -0.0004 | 0.079 | 0 | PASS |
