#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_bool(verbose, false,
            "Whether or not the curves' values should be printed.");

static int sim_time = 1000;

std::vector<double> original(2 * sim_time + 1);
std::vector<double> lin(2 * sim_time + 1), lin_cp(2 * sim_time + 1);
std::vector<double> expo(2 * sim_time + 1), expo_cp(2 * sim_time + 1);
std::vector<double> hvs(2 * sim_time + 1), hvs_cp(2 * sim_time + 1);
double lin_res, expo_res, hvs_res;

void calc_scores(const std::vector<double>& c) {
  lin_res = expo_res = hvs_res = 0.0;
  for (int t = 1; t <= 2 * sim_time; t++) {
    lin_res += lin_cp[t] * c[t];
    expo_res += expo_cp[t] * c[t];
    hvs_res += hvs_cp[t] * c[t];
  }
  lin_res /= 2 * sim_time;
  expo_res /= 2 * sim_time;
  hvs_res /= 2 * sim_time;
}

void calc_scores_lin(const std::vector<double>& c) {
  lin_res = expo_res = hvs_res = 0.0;
  for (int t = 0; t <= 2 * sim_time; t++) {
    lin_res += lin_cp[t] * (t / (2.0 * sim_time) * c[0] + c[1]);
    expo_res += expo_cp[t] * (t / (2.0 * sim_time) * c[0] + c[1]);
    hvs_res += hvs_cp[t] * (t / (2.0 * sim_time) * c[0] + c[1]);
  }
  lin_res /= 2 * sim_time;
  expo_res /= 2 * sim_time;
  hvs_res /= 2 * sim_time;
}

double eval(const std::vector<double>& c) {
  using std::max;
  using std::min;

  double total_min = std::numeric_limits<double>::max();
  double total_max = std::numeric_limits<double>::min();

  for (double complete_time = 1; complete_time <= 2 * sim_time;
       complete_time++) {
    original[complete_time] = (sim_time - complete_time) / sim_time;
    lin_cp[0] = expo_cp[0] = hvs_cp[0];
    for (int t = 1; t <= 2 * sim_time; t++) {
      double B = -2.0 * 3.321928094887362;
      double A = -B / complete_time;
      lin_cp[t] = min(1.0, (double)t / complete_time);
      expo_cp[t] = min(1.0, pow(2.0, A * t + B));
      hvs_cp[t] = static_cast<int>(t >= complete_time);
    }

    if (c.size() == 2) {
      calc_scores_lin(c);
    } else {
      calc_scores(c);
    }

    lin[complete_time] = lin_res;
    expo[complete_time] = expo_res;
    hvs[complete_time] = hvs_res;
    total_min = min(total_min, min(lin_res, min(expo_res, hvs_res)));
    total_max = max(total_max, max(lin_res, max(expo_res, hvs_res)));
  }

  const double diff = total_max - total_min;
  double res = 0;
  for (int t = 1; t <= 2 * sim_time; t++) {
    lin[t] = 2.0 / diff * (lin[t] - total_min) - 1.0;
    expo[t] = 2.0 / diff * (expo[t] - total_min) - 1.0;
    hvs[t] = 2.0 / diff * (hvs[t] - total_min) - 1.0;
    const double lin_diff = fabs(lin[t] - original[t]);
    const double expo_diff = fabs(expo[t] - original[t]);
    const double hvs_diff = fabs(hvs[t] - original[t]);
    res += lin_diff * lin_diff + expo_diff * expo_diff + hvs_diff * hvs_diff;
    if (FLAGS_verbose) {
      printf("%.10f %.10f %.10f %.10f\n", original[t], lin[t], expo[t], hvs[t]);
    }
  }

  return res;
}

int main(int argc, char* argv[]) {
  gflags::AllowCommandLineReparsing();
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(argc > 2);

  std::vector<double> cs;
  if (argc == 3) {
    cs.push_back(atof(argv[1]));
    cs.push_back(atof(argv[2]));
  } else {
    sim_time = (argc - 1) / 2;
    original.resize(2 * sim_time + 1);
    lin.resize(2 * sim_time + 1);
    lin_cp.resize(2 * sim_time + 1);
    expo.resize(2 * sim_time + 1);
    expo_cp.resize(2 * sim_time + 1);
    hvs.resize(2 * sim_time + 1);
    hvs_cp.resize(2 * sim_time + 1);
    for (int i = 1; i < argc; i++) {
      cs.push_back(atof(argv[i]));
    }
  }

  printf("%.3f\n", eval(cs));
  return 0;
}
