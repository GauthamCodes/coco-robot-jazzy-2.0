// C2-NAV.33 - oracle harness around the DEPLOYED nav2_amcl particle filter.
//
// This file contains NO particle-filter equations.  It links against the
// installed `libpf_lib.so` (ros-jazzy-nav2-amcl 1.3.11-1noble.20260412.054619)
// and calls `pf_update_sensor` and `pf_update_resample` themselves, so
// whatever the deployed binary does to the weights is what this program
// reports.  Nothing here re-implements, re-derives or approximates it.
//
// It answers exactly one question, the gate for the whole session:
//
//   Does the sample set that `AmclNode::publishParticleCloud` is handed --
//   `pf_->sets + pf_->current_set` -- carry NON-FLAT importance weights when
//   the resample step is skipped, and FLAT ones when it is not?
//
// Protocol:
//   c2nav33_oracle N seed mu_y sigma_y
//     stdout, one "phase key value..." line per fact:
//       init      <current_set>
//       sensor    <current_set> <total_returned>
//       s <i> <x> <y> <yaw> <w>      (after pf_update_sensor, pre-resample)
//       resample  <current_set>
//       r <i> <x> <y> <yaw> <w>      (after pf_update_resample)
//
// The sensor model here is a deliberately crude Gaussian in y.  Its JOB is
// only to be non-uniform: this program tests the PLUMBING of the weights
// through the deployed filter, and makes no claim about the real
// likelihood-field model (that is C2-NAV.31's subject).
//
// Diagnostic only.  Reads nothing from the repository, writes nothing but
// stdout, starts no ROS and no simulator.

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "nav2_amcl/pf/pf.hpp"

static pf_vector_t dummy_pose_fn(void *)
{
  pf_vector_t v;
  v.v[0] = v.v[1] = v.v[2] = 0.0;
  return v;
}

struct SensorCfg { double mu_y; double sigma_y; };

// Assigns a weight to every sample and returns the total, which is the
// contract pf_sensor_model_fn_t declares in the installed pf.hpp.
static double sensor_fn(void * data, pf_sample_set_t * set)
{
  SensorCfg * cfg = static_cast<SensorCfg *>(data);
  double total = 0.0;
  for (int i = 0; i < set->sample_count; i++) {
    const double dy = (set->samples[i].pose.v[1] - cfg->mu_y) / cfg->sigma_y;
    const double w = std::exp(-0.5 * dy * dy);
    set->samples[i].weight = w;
    total += w;
  }
  return total;
}

int main(int argc, char ** argv)
{
  if (argc != 5) {
    std::fprintf(stderr, "usage: %s N seed mu_y sigma_y\n", argv[0]);
    return 2;
  }
  const int n = atoi(argv[1]);
  const long seed = atol(argv[2]);
  SensorCfg cfg;
  cfg.mu_y = atof(argv[3]);
  cfg.sigma_y = atof(argv[4]);
  if (n <= 0 || cfg.sigma_y <= 0.0) {
    std::fprintf(stderr, "bad N or sigma_y\n");
    return 2;
  }

  srand48(seed);

  // min == max so the KLD sampler cannot resize the set, which would make
  // the two dumps different lengths for a reason that is not the weights.
  // alpha_slow / alpha_fast are the deployed values (recovery_alpha_* 0.0),
  // so no random poses are injected on resample.
  pf_t * pf = pf_alloc(n, n, 0.0, 0.0, dummy_pose_fn);
  if (!pf) { std::fprintf(stderr, "pf_alloc failed\n"); return 1; }

  pf_vector_t mean;
  mean.v[0] = 0.0; mean.v[1] = 0.0; mean.v[2] = 0.0;
  pf_matrix_t cov;
  for (int a = 0; a < 3; a++) {
    for (int b = 0; b < 3; b++) { cov.m[a][b] = 0.0; }
  }
  cov.m[0][0] = 0.25 * 0.25;
  cov.m[1][1] = 0.25 * 0.25;
  cov.m[2][2] = 0.10 * 0.10;
  pf_init(pf, mean, cov);
  std::printf("init %d\n", pf->current_set);

  // pf_update_sensor returns void in the installed pf.hpp; the total is
  // the sensor_fn's own return value, which the filter consumes internally.
  pf_update_sensor(pf, sensor_fn, &cfg);
  {
    pf_sample_set_t * set = pf->sets + pf->current_set;
    std::printf("sensor %d %d\n", pf->current_set, set->sample_count);
    for (int i = 0; i < set->sample_count; i++) {
      std::printf("s %d %.17g %.17g %.17g %.17g\n", i,
        set->samples[i].pose.v[0], set->samples[i].pose.v[1],
        set->samples[i].pose.v[2], set->samples[i].weight);
    }
  }

  pf_update_resample(pf, nullptr);
  {
    pf_sample_set_t * set = pf->sets + pf->current_set;
    std::printf("resample %d %d\n", pf->current_set, set->sample_count);
    for (int i = 0; i < set->sample_count; i++) {
      std::printf("r %d %.17g %.17g %.17g %.17g\n", i,
        set->samples[i].pose.v[0], set->samples[i].pose.v[1],
        set->samples[i].pose.v[2], set->samples[i].weight);
    }
  }

  pf_free(pf);
  return 0;
}
