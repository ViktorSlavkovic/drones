#include "solvers/upper_bound/upper_bound.h"

#include <algorithm>
#include <map>
#include <numeric>
#include <vector>

#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "solvers/util/load_splitter.h"

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace drones {

using operations_research::Assignment;
using operations_research::DefaultRoutingSearchParameters;
using operations_research::FirstSolutionStrategy;
using operations_research::RoutingIndexManager;
using operations_research::RoutingModel;
using operations_research::RoutingSearchParameters;

int UpperBound::Calc() {
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

  // Prepare product weights map for the splitting.
  std::map<int, int> prod_weights;
  for (int p = 0; p < problem_.np(); p++) {
    prod_weights[p] = problem_.product(p).m();
  }

  // Split each order into turns.
  std::vector<util::LoadSplitter::CompleteSplit> order_splits;
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

    int dur = 0;
    bool first = true;
    for (const auto& repeated_split : split.repeated_splits) {
      int load_unload_times = 2 * repeated_split.times;
      int travel_times = load_unload_times;
      // We don't count the first travel time since we don't know where it will
      // be from, we'll add that in distance matrix.
      if (first) {
        travel_times--;
        first = false;
      }
      dur += repeated_split.single_split.size() * load_unload_times;
      dur += problem_.dist().src(closest_wh[o]).dst(problem_.nw() + o) *
             travel_times;
    }
    order_durations.push_back(dur);
    order_splits.push_back(split);
  }

  // [0, No - 1] - pairs (closest_wh[o], o) as our fictive graph's nodes.
  //               Distances between nodes are calculated as order to warehouse
  //               distances.
  // [No, No]    - starting point (warehouse 0), dist from any node to here is
  //               0 and dist from here to any node is warehouse 0 to node's
  //               warehouse distance.
  // Node visiting time is added to all distances: time needed to perform all
  // Load and Deliver commands.
  std::vector<std::vector<int>> distances(problem_.no() + 1);
  for (int i = 0; i < problem_.no(); i++) {
    for (int j = 0; j < problem_.no(); j++) {
      if (i == j) {
        distances[i].push_back(0);  // Irrelevant.
      } else {
        distances[i].push_back(
            problem_.dist().src(problem_.nw() + i).dst(closest_wh[j]) +
            order_durations[j]);
      }
    }
    distances[i].push_back(0);
  }
  for (int j = 0; j < problem_.no(); j++) {
    distances[problem_.no()].push_back(
        problem_.dist().src(0).dst(closest_wh[j]) + order_durations[j]);
  }
  distances[problem_.no()].push_back(0);

  // Solve the TSP and get the path:
  std::vector<int> order_perm;
  {
    // (1 / 5) Create the model object.
    RoutingIndexManager manager(
        problem_.no() + 1 /* number of nodes */, 1 /* number of vehicles */,
        RoutingIndexManager::NodeIndex{problem_.no()} /* depot index */);
    RoutingModel model(manager);

    // (2 / 5) Define cost of each arc.
    const int transit_callback_index = model.RegisterTransitCallback(
        [&distances, &manager](int64_t from_index,
                               int64_t to_index) -> int64_t {
          // Convert from routing variable Index to distance matrix NodeIndex.
          auto from_node = manager.IndexToNode(from_index).value();
          auto to_node = manager.IndexToNode(to_index).value();
          return distances[from_node][to_node];
        });
    model.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

    // (3 / 5) Set first solution heuristic.
    RoutingSearchParameters search_params = DefaultRoutingSearchParameters();
    search_params.set_first_solution_strategy(
        FirstSolutionStrategy::PATH_CHEAPEST_ARC);

    // (4 / 5) Solve the problem.
    const Assignment* solution = model.SolveWithParameters(search_params);

    // (5 / 5) Print the solution.
    int64 index = model.Start(0 /* vehicle idx */);
    // Skip the first node (depot).
    index = solution->Value(model.NextVar(index));
    while (model.IsEnd(index) == false) {
      order_perm.push_back(manager.IndexToNode(index).value());
      index = solution->Value(model.NextVar(index));
    }
    CHECK(order_perm.size() == problem_.no());
  }

  double finish_time = 0;
  int prev_o = problem_.no();
  for (int o : order_perm) {
    finish_time += static_cast<double>(distances[prev_o][o]) / problem_.nd();
    prev_o = o;
    upper_bound_ +=
        static_cast<int>(100.0 * (problem_.t() - finish_time) / problem_.t());
  }

  return upper_bound_;
}

}  // namespace drones
