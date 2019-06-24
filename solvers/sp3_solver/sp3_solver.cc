#include "solvers/sp3_solver/sp3_solver.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "gflags/gflags.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

DEFINE_bool(sp3_do_tsp, false,
            "If set, find the order permutation by solving a TSP.");

namespace drones {

using operations_research::Assignment;
using operations_research::DefaultRoutingSearchParameters;
using operations_research::FirstSolutionStrategy;
using operations_research::RoutingIndexManager;
using operations_research::RoutingModel;
using operations_research::RoutingSearchParameters;

std::unique_ptr<Solution> Sp3Solver::Solve() {
  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  auto* drone_commands = solution->add_drone_desc();
  std::vector<std::pair<int, int>> distances;

  std::vector<int> order_perm;

  if (FLAGS_sp3_do_tsp) {
    LOG(INFO) << "Finding oder perm using TSP.";
    // Build up distances.
    std::vector<std::vector<int>> distances(problem_.no() + 1);
    for (int i = 0; i < problem_.no(); i++) {
      for (int j = 0; j < problem_.no(); j++) {
        if (i == j) {
          distances[i].push_back(0);  // Irrelevant.
        } else {
          distances[i].push_back(problem_.dist().src(problem_.nw() + i).dst(0) +
                                 problem_.dist().src(problem_.nw() + j).dst(0) *
                                     2 * (problem_.order(j).request(0) - 1) +
                                 2 * problem_.order(j).request(0));
        }
      }
      distances[i].push_back(0);
    }
    for (int j = 0; j < problem_.no(); j++) {
      distances[problem_.no()].push_back(
          problem_.dist().src(problem_.nw() + j).dst(0) * 2 *
              (problem_.order(j).request(0) - 1) +
          2 * problem_.order(j).request(0));
    }
    distances[problem_.no()].push_back(0);
    // Solve the TSP and get the path:
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
  } else {
    LOG(INFO) << "Finding oder perm using sorting.";
    for (int o = 0; o < problem_.no(); o++) {
      distances.push_back({problem_.dist().src(problem_.nw() + o).dst(0) *
                               problem_.order(o).request(0),
                           o});
    }
    std::sort(distances.begin(), distances.end());
    for (const auto& d_o : distances) {
      order_perm.push_back(d_o.second);
    }
  }
  LOG(INFO) << "Built oder perm.";

  int curr_time = 1;
  int prev_loc = 0;
  for (const auto& o : order_perm) {
    int curr_time_bak = curr_time;
    int prev_loc_bak = prev_loc;
    int num_commands = 0;
    bool trimmed = false;
    for (int i = 0; i < problem_.order(o).request(0); i++) {
      // Add a LOAD command.
      {
        auto* cmd = drone_commands->add_drone_command();
        num_commands++;
        cmd->set_type(DroneCommand_CommandType_LOAD);
        cmd->set_drone(0);
        cmd->set_warehouse(0);
        cmd->set_product(0);
        cmd->set_num_items(1);
        cmd->set_start_time(curr_time);
        curr_time += problem_.dist().src(prev_loc).dst(0) + 1;
        if (curr_time > problem_.t() + 1) {
          trimmed = true;
          break;
        }
        prev_loc = 0;
      }
      // Add a DELIVER command.
      {
        auto* cmd = drone_commands->add_drone_command();
        num_commands++;
        cmd->set_type(DroneCommand_CommandType_DELIVER);
        cmd->set_drone(0);
        cmd->set_order(o);
        cmd->set_product(0);
        cmd->set_num_items(1);
        cmd->set_start_time(curr_time);
        curr_time += problem_.dist().src(prev_loc).dst(problem_.nw() + o) + 1;
        if (curr_time > problem_.t() + 1) {
          trimmed = true;
          break;
        }
        prev_loc = problem_.nw() + o;
      }
    }
    if (trimmed) {
      curr_time = curr_time_bak;
      prev_loc = prev_loc_bak;
      while (num_commands--) {
        drone_commands->mutable_drone_command()->RemoveLast();
      }
    }
  }
  return solution;
}

bool Sp3Solver::CanSolve(const ProblemType& problem_type) const {
  return problem_type.has_nd_1() && problem_type.nd_1() &&
         problem_type.has_m_m() && problem_type.m_m() &&
         problem_type.has_nw_1() && problem_type.nw_1() &&
         problem_type.has_s0_inf() && problem_type.s0_inf() &&
         problem_type.has_np_1() && problem_type.np_1();
}

}  // namespace drones
