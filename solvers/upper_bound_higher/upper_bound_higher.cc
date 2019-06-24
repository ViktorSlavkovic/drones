#include "solvers/upper_bound_higher/upper_bound_higher.h"

#include <algorithm>
#include <map>
#include <numeric>
#include <vector>

#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "solvers/util/load_splitter.h"

DEFINE_bool(
    ub_ultimate_closest, false,
    "If true, it will be assumed that for each order everything is delivered "
    "from it's closest location (either order or warehouse).");

namespace drones {

int UpperBoundHigher::Calc() {
  if (upper_bound_ >= 0) {
    return upper_bound_;
  }

  // Find the closest warehouse for each order.
  std::vector<int> closest_wh;
  for (int o = 0; o < problem_.no(); o++) {
    int cw = -1;
    int cwd = std::numeric_limits<int>::max();
    for (int w = 0; w < problem_.nw(); w++) {
      int wd = problem_.dist().src(w).dst(problem_.nw() + o);
      if (cw == -1 || wd < cwd) {
        cw = w;
        cwd = wd;
      }
    }
    closest_wh.push_back(cw);
  }

  // Find the closest order (other than itself) for each order.
  // -1 means that it's the only one.
  std::vector<int> closest_order;
  for (int o = 0; o < problem_.no(); o++) {
    int co = -1;
    int cod = std::numeric_limits<int>::max();
    for (int o1 = 0; o1 < problem_.no(); o1++) {
      if (o1 == o) continue;
      int o1d = problem_.dist().src(problem_.nw() + o1).dst(problem_.nw() + o);
      if (co == -1 || o1d < cod) {
        co = o1;
        cod = o1d;
      }
    }
    closest_order.push_back(co);
  }

  // Prepare product weights map for the splitting.
  std::map<int, int> prod_weights;
  for (int p = 0; p < problem_.np(); p++) {
    prod_weights[p] = problem_.product(p).m();
  }

  // Get the order durations.
  std::vector<int> order_durations;
  std::map<int, int> order_request;
  for (int o = 0; o < problem_.no(); o++) {
    order_request.clear();
    for (int p = 0; p < problem_.np(); p++) {
      int req = problem_.order(o).request(p);
      if (req > 0) {
        order_request[p] = req;
      }
    }
    auto split =
        util::LoadSplitter::Split(order_request, prod_weights, problem_.m());

    // We'll assume that everything is being delivered from the closest
    // warehouse and that flying to the closest warehouse took no time.
    // Also, we'll ignore the 1 tick per LOAD/UNLOAD/DELIVER to actually
    // load/unload the goods.
    int dur = (2 * split.total_times - 1) *
              problem_.dist().src(closest_wh[o]).dst(problem_.nw() + o);
    if (FLAGS_ub_ultimate_closest && closest_order[o] > -1 &&
        problem_.dist()
                .src(problem_.nw() + closest_order[o])
                .dst(problem_.nw() + o) <
            problem_.dist().src(closest_wh[o]).dst(problem_.nw() + o)) {
      dur = (2 * split.total_times - 1) *
            problem_.dist()
                .src(problem_.nw() + closest_order[o])
                .dst(problem_.nw() + o);
    }
    order_durations.push_back(dur);
  }
  std::sort(order_durations.begin(), order_durations.end());

  double finish_time = 0;
  for (int dir : order_durations) {
    finish_time += static_cast<double>(dir) / problem_.nd();
    if (finish_time > problem_.t() - 1) break;
    upper_bound_ += ceil(100.0 * (problem_.t() - finish_time) / problem_.t());
  }
  return upper_bound_;
}

}  // namespace drones
