# Reference measurements the MuJoCo model is calibrated against

## `yaw_gazebo_baseline.csv`

Gazebo Harmonic (gz-sim 8.11, DART), the deployed robot in
`coco_world.world`, driven through `diff_drive_controller` exactly as the
mission does. Each row is one commanded yaw magnitude, held for a 5 s arc
at `MAX_LIN = 0.4 m/s` after a 2 s settle.

`cmd_yaw` is the commanded arc in radians, `cmd_rate` the rate in rad/s
(`cmd_yaw / 5`), `achieved` the yaw actually turned.

Both signs are recorded because **Gazebo is not self-consistent at the
top of this range**: at 2.5 rad the +ve and -ve arcs differ by 1.36x. The
calibration therefore fits the magnitude average, and no route or reward
in M7 may require sustained commanded yaw above ~1 rad/s, because the
reference itself is not repeatable there.

This file is the target `coco_sim.calibrate` fits to. It is committed so
the calibration has a reproduction path; a fitted constant whose reference
data lives only in a scratch directory is not reproducible.
