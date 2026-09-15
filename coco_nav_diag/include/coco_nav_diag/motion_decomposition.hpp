// C2-NAV.39: the (rot1, trans, rot2) split nav2_amcl's
// DifferentialMotionModel::odometryUpdate applies to an odom-frame delta,
// including its 1 cm cutoff below which rot1 is forced to zero. Kept as a
// pure function so test_motion_decomposition_oracle can compare it against
// the INSTALLED motion model rather than against a re-derivation.
#ifndef COCO_NAV_DIAG__MOTION_DECOMPOSITION_HPP_
#define COCO_NAV_DIAG__MOTION_DECOMPOSITION_HPP_

#include <cmath>

#include "nav2_amcl/angleutils.hpp"

namespace coco_nav_diag
{

struct DifferentialDecomposition
{
  double rot1 = 0.0;
  double trans = 0.0;
  double rot2 = 0.0;
};

inline DifferentialDecomposition computeDifferentialDecomposition(
  double delta_x, double delta_y, double delta_yaw, double anchor_yaw)
{
  DifferentialDecomposition d;
  d.trans = std::sqrt(delta_x * delta_x + delta_y * delta_y);
  if (d.trans < 0.01) {
    d.rot1 = 0.0;
  } else {
    d.rot1 = nav2_amcl::angleutils::angle_diff(std::atan2(delta_y, delta_x), anchor_yaw);
  }
  d.rot2 = nav2_amcl::angleutils::angle_diff(delta_yaw, d.rot1);
  return d;
}

}  // namespace coco_nav_diag

#endif  // COCO_NAV_DIAG__MOTION_DECOMPOSITION_HPP_
