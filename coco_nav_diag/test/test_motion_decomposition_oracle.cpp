// C2-NAV.39: the (rot1, trans, rot2) split the diagnostic records must be
// the split the INSTALLED nav2_amcl::DifferentialMotionModel applies. This
// test contains no motion-model equations of its own: it runs the deployed
// odometryUpdate() with all noise alphas at zero, which makes every particle
// move by exactly (trans, rot1, rot2), and compares that displacement with
// the recomposition of computeDifferentialDecomposition(). Same oracle
// pattern as docs/data/c2nav32_oracle.cpp.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <vector>

#include "coco_nav_diag/motion_decomposition.hpp"
#include "nav2_amcl/angleutils.hpp"
#include "nav2_amcl/motion_model/differential_motion_model.hpp"
#include "nav2_amcl/pf/pf.hpp"

namespace
{

pf_vector_t zeroPose(void *)
{
  pf_vector_t v;
  v.v[0] = v.v[1] = v.v[2] = 0.0;
  return v;
}

struct Case
{
  const char * name;
  double anchor_x, anchor_y, anchor_yaw;
  double dx, dy, dyaw;
};

const std::vector<Case> kCases = {
  {"forward along heading", 1.0, -2.0, 0.4, 0.30 * std::cos(0.4), 0.30 * std::sin(0.4), 0.05},
  {"sideways relative to heading", -2.0, 0.0, 0.0, 0.0, 0.12, -0.3},
  {"reverse", 0.5, 0.5, 0.0, -0.20, 0.0, 0.0},
  {"below 1 cm cutoff", 0.0, 0.0, 1.2, 0.004, 0.003, 0.2},
  {"pure rotation", 3.0, 1.0, -2.5, 0.0, 0.0, -0.5},
  {"heading across +pi", -1.0, 2.0, 3.10, 0.05 * std::cos(3.2), 0.05 * std::sin(3.2), 0.2},
};

void expectInstalledModelMatchesDecomposition(const Case & c)
{
  SCOPED_TRACE(c.name);
  const int n = 6;
  pf_t * pf = pf_alloc(n, n, 0.0, 0.0, &zeroPose);
  ASSERT_NE(pf, nullptr);
  pf_sample_set_t * set = pf->sets + pf->current_set;
  set->sample_count = n;
  for (int i = 0; i < n; ++i) {
    set->samples[i].pose.v[0] = c.anchor_x + 0.1 * i;
    set->samples[i].pose.v[1] = c.anchor_y - 0.05 * i;
    set->samples[i].pose.v[2] = c.anchor_yaw;
    set->samples[i].weight = 1.0 / n;
  }

  pf_vector_t delta;
  delta.v[0] = c.dx;
  delta.v[1] = c.dy;
  delta.v[2] = c.dyaw;
  pf_vector_t pose;
  pose.v[0] = c.anchor_x + c.dx;
  pose.v[1] = c.anchor_y + c.dy;
  pose.v[2] = c.anchor_yaw + c.dyaw;

  nav2_amcl::DifferentialMotionModel model;
  model.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
  srand48(1);
  model.odometryUpdate(pf, pose, delta);

  const auto d = coco_nav_diag::computeDifferentialDecomposition(
    c.dx, c.dy, c.dyaw, c.anchor_yaw);
  set = pf->sets + pf->current_set;
  ASSERT_EQ(set->sample_count, n);
  for (int i = 0; i < n; ++i) {
    const double x0 = c.anchor_x + 0.1 * i;
    const double y0 = c.anchor_y - 0.05 * i;
    EXPECT_NEAR(set->samples[i].pose.v[0], x0 + d.trans * std::cos(c.anchor_yaw + d.rot1), 1e-12);
    EXPECT_NEAR(set->samples[i].pose.v[1], y0 + d.trans * std::sin(c.anchor_yaw + d.rot1), 1e-12);
    EXPECT_NEAR(
      nav2_amcl::angleutils::angle_diff(
        set->samples[i].pose.v[2], c.anchor_yaw + d.rot1 + d.rot2), 0.0, 1e-12);
  }
  pf_free(pf);
}

}  // namespace

TEST(MotionDecompositionOracle, InstalledDifferentialModelAppliesTheRecordedSplit)
{
  for (const auto & c : kCases) {
    expectInstalledModelMatchesDecomposition(c);
  }
}

TEST(MotionDecompositionOracle, RecomposesTheDeltaAboveTheCutoff)
{
  for (const auto & c : kCases) {
    SCOPED_TRACE(c.name);
    const auto d = coco_nav_diag::computeDifferentialDecomposition(
      c.dx, c.dy, c.dyaw, c.anchor_yaw);
    if (d.trans < 0.01) {
      EXPECT_EQ(d.rot1, 0.0);
      continue;
    }
    EXPECT_NEAR(d.trans * std::cos(c.anchor_yaw + d.rot1), c.dx, 1e-12);
    EXPECT_NEAR(d.trans * std::sin(c.anchor_yaw + d.rot1), c.dy, 1e-12);
    EXPECT_NEAR(nav2_amcl::angleutils::angle_diff(d.rot1 + d.rot2, c.dyaw), 0.0, 1e-12);
  }
}
