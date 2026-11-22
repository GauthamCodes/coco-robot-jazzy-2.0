#!/usr/bin/env python3
"""The DWB scoring arithmetic behind the enclosure stall.

Everything here comes from the shipped nav2_params.yaml and from the
Jazzy headers on this machine:

  dwb_critics/map_grid.hpp:69
      getScale() = costmap resolution * 0.5 * scale
  dwb_core/trajectory_critic.hpp:177
      getScale() = scale                       (BaseObstacle uses this)
  nav2_costmap_2d InflationLayer::computeCost
      cost(d) = 252 * exp(-cost_scaling_factor * (d - inscribed_radius))

DWB MINIMISES the total, and BaseObstacle with sum_scores:false scores
the FINAL pose of the trajectory only. So in a cost field that rises
along the direction of travel, the cheapest trajectory is the one that
travels least -- and the goal/path critics have to outbid that.
"""
import math

RES = 0.05
INSCRIBED = 0.20            # = robot_radius
CSF = 5.0                   # cost_scaling_factor
INFLATION = 0.50

BASE_OBSTACLE_SCALE = 8.0
PATH_DIST_SCALE = 32.0
PATH_ALIGN_SCALE = 32.0
GOAL_DIST_SCALE = 24.0
GOAL_ALIGN_SCALE = 24.0


def cost(d):
    if d <= 0:
        return 254.0
    if d <= INSCRIBED:
        return 253.0
    return 252.0 * math.exp(-CSF * (d - INSCRIBED))


def dist_for_cost(c):
    return INSCRIBED + math.log(252.0 / c) / CSF


def mapgrid_scale(s):
    return RES * 0.5 * s


print('EFFECTIVE CRITIC WEIGHTS (what the score actually multiplies)')
print(f'  BaseObstacle  scale {BASE_OBSTACLE_SCALE:5.1f} -> '
      f'{BASE_OBSTACLE_SCALE:6.2f} per unit of COST (cost runs 0..252)')
for n, s in (('PathDist', PATH_DIST_SCALE), ('PathAlign', PATH_ALIGN_SCALE),
             ('GoalDist', GOAL_DIST_SCALE), ('GoalAlign', GOAL_ALIGN_SCALE)):
    e = mapgrid_scale(s)
    print(f'  {n:<13} scale {s:5.1f} -> {e:6.2f} per CELL '
          f'({e / RES:5.2f} per metre)')

print('\nINFLATION COST vs CLEARANCE (inflation_radius 0.50, '
      'inscribed 0.20)')
print(f'  {"d (m)":>7}{"cost":>7}{"BaseObstacle term":>20}')
for d in (0.20, 0.25, 0.30, 0.316, 0.35, 0.375, 0.40, 0.45, 0.497, 0.50):
    c = cost(d)
    print(f'  {d:7.3f}{c:7.1f}{c * BASE_OBSTACLE_SCALE:20.1f}')

print('\nTHE TRADE, per 0.05 m cell of forward progress')
print('  advancing one cell toward the goal is worth:')
gain = mapgrid_scale(GOAL_DIST_SCALE) + mapgrid_scale(PATH_DIST_SCALE)
print(f'    GoalDist {mapgrid_scale(GOAL_DIST_SCALE):.2f} + '
      f'PathDist {mapgrid_scale(PATH_DIST_SCALE):.2f} = {gain:.2f}')
print('  advancing one cell INTO the inflation gradient costs:')
print(f'  {"from d":>8}{"to d":>8}{"dCost":>9}{"dBaseObs":>11}'
      f'{"ratio vs gain":>16}')
for d in (0.50, 0.45, 0.40, 0.375, 0.35, 0.30, 0.25):
    d2 = d - RES
    dc = cost(d2) - cost(d)
    db = dc * BASE_OBSTACLE_SCALE
    print(f'  {d:8.3f}{d2:8.3f}{dc:9.1f}{db:11.1f}{db / gain:15.1f}x')

print('\nMEASURED CHECK -- enclosure_entry rep0, worst crawl')
print('  BaseObstacle term on the CHOSEN trajectory: 456.0')
c = 456.0 / BASE_OBSTACLE_SCALE
print(f'  -> raw cost {c:.1f} -> clearance {dist_for_cost(c):.3f} m')
print('  robot was at world (-2.305, 2.852); nearest geometry is')
print('    box_obstacle_1 NE corner (-2.75, 2.65):  '
      f'{math.dist((-2.305, 2.852), (-2.75, 2.65)):.3f} m')
print('    north wall inner face  y = 3.40:         '
      f'{3.40 - 2.852:.3f} m')
print('  the two agree, so the cost field is behaving as the formula says.')

print('\nTHE CHANNEL IT REFUSED TO ENTER')
gap = 3.40 - 2.65
print(f'  box_obstacle_1 north face y=2.65 to north wall y=3.40: '
      f'{gap:.2f} m')
print(f'  non-inscribed band: {gap - 2 * INSCRIBED:.2f} m')
print(f'  zero-cost band:     {max(0.0, gap - 2 * INFLATION):.2f} m  '
      f'<-- there is none')
mid = gap / 2
print(f'  best achievable clearance at the centre: {mid:.3f} m '
      f'-> cost {cost(mid):.0f} -> BaseObstacle {cost(mid) * 8:.0f}')
print(f'  entering it from the stall point raises BaseObstacle by '
      f'{cost(mid) * 8 - 456:.0f}')
adv = 0.5 / RES
print(f'  the ~0.5 m of progress that buys is worth {adv * gain:.0f}')
print(f'  penalty/reward = {(cost(mid) * 8 - 456) / (adv * gain):.0f}x '
      f'against moving')

print('\nCOLLISION MONITOR ZONE GEOMETRY (the polygons are SQUARES)')
for name, half, act in (('PolygonSlow', 0.40, 'slowdown x0.3'),
                        ('PolygonLimit', 0.55, 'limit 0.4 m/s, 0.5 rad/s')):
    print(f'  {name:<14} half-width {half:.2f} m -> fires out to '
          f'{half * math.sqrt(2):.3f} m on the diagonal   [{act}]')
print(f'  PolygonStop    circle r 0.25 m -> fires at 0.250 m, uniformly')
print('  measured: wall_adjacent held SLOWDOWN 57.25 s with the closest')
print('  laser return at 0.498 m -- between 0.40 and 0.566, i.e. exactly')
print('  the band the square reaches and a circle would not.')
