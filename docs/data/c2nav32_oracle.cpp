// C2-NAV.32 — oracle harness around the DEPLOYED nav2_amcl motion model.
//
// This file contains NO motion-model equations.  It links against the
// installed `libmotions_lib.so` / `libpf_lib.so` (ros-jazzy-nav2-amcl
// 1.3.11-1noble.20260412.054619) and calls
// `nav2_amcl::DifferentialMotionModel::odometryUpdate` itself, so whatever
// the deployed binary does is what this program measures.  Nothing here
// re-implements, re-derives or approximates it.
//
// Protocol (deliberately dumb, so the Python side owns all analysis):
//
//   c2nav32_oracle a1 a2 a3 a4 a5 seed px py pyaw dx dy dyaw  < in > out
//
//   stdin :  N, then N lines of "x y yaw"      (the particle set)
//   stdout:  N lines of "x y yaw"              (after ONE odometryUpdate)
//
// `seed` is passed to srand48(), which is the generator libpf_lib.so draws
// from (verified: it imports drand48/srand48 and nothing else random), so
// runs are bit-reproducible.
//
// Diagnostic only.  Reads nothing from the repository, writes nothing but
// stdout, starts no ROS and no simulator.

#include <cstdio>
#include <cstdlib>
#include <vector>

#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/motion_model/differential_motion_model.hpp"

// pf_alloc wants a random-pose callback.  odometryUpdate never invokes it;
// it exists only so the allocation is the one the filter really performs.
static pf_vector_t dummy_pose_fn(void *)
{
  pf_vector_t v;
  v.v[0] = v.v[1] = v.v[2] = 0.0;
  return v;
}

int main(int argc, char ** argv)
{
  if (argc != 13) {
    std::fprintf(stderr,
      "usage: %s a1 a2 a3 a4 a5 seed px py pyaw dx dy dyaw\n", argv[0]);
    return 2;
  }
  const double a1 = atof(argv[1]), a2 = atof(argv[2]), a3 = atof(argv[3]);
  const double a4 = atof(argv[4]), a5 = atof(argv[5]);
  const long seed = atol(argv[6]);

  pf_vector_t pose, delta;
  pose.v[0] = atof(argv[7]);
  pose.v[1] = atof(argv[8]);
  pose.v[2] = atof(argv[9]);
  delta.v[0] = atof(argv[10]);
  delta.v[1] = atof(argv[11]);
  delta.v[2] = atof(argv[12]);

  int n = 0;
  if (std::scanf("%d", &n) != 1 || n <= 0) {
    std::fprintf(stderr, "bad particle count\n");
    return 2;
  }
  std::vector<double> px(n), py(n), pa(n);
  for (int i = 0; i < n; i++) {
    if (std::scanf("%lf %lf %lf", &px[i], &py[i], &pa[i]) != 3) {
      std::fprintf(stderr, "short particle list at %d\n", i);
      return 2;
    }
  }

  pf_t * pf = pf_alloc(n, n, 0.0, 0.0, &dummy_pose_fn);
  if (!pf) {
    std::fprintf(stderr, "pf_alloc failed\n");
    return 3;
  }

  pf_sample_set_t * set = pf->sets + pf->current_set;
  set->sample_count = n;
  for (int i = 0; i < n; i++) {
    set->samples[i].pose.v[0] = px[i];
    set->samples[i].pose.v[1] = py[i];
    set->samples[i].pose.v[2] = pa[i];
    set->samples[i].weight = 1.0 / n;
  }

  nav2_amcl::DifferentialMotionModel mm;
  mm.initialize(a1, a2, a3, a4, a5);

  srand48(seed);
  mm.odometryUpdate(pf, pose, delta);

  set = pf->sets + pf->current_set;
  for (int i = 0; i < set->sample_count; i++) {
    std::printf("%.17g %.17g %.17g\n",
      set->samples[i].pose.v[0],
      set->samples[i].pose.v[1],
      set->samples[i].pose.v[2]);
  }
  pf_free(pf);
  return 0;
}
