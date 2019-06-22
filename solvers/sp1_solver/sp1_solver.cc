#include "solvers/sp1_solver/sp1_solver.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace drones {

std::unique_ptr<Solution> Sp1Solver::Solve() {
  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  auto* drone_commands = solution->add_drone_desc();
  std::vector<std::pair<int, int>> distances;
  for (int order = 0; order < problem_.no(); order++) {
    distances.push_back(
        {problem_.dist().src(0).dst(problem_.nw() + order), order});
  }
  std::sort(distances.begin(), distances.end());
  int curr_time = 0;
  int prev_dist = 0;
  for (const auto& order : distances) {
    if (curr_time + prev_dist + order.first + 2 > problem_.t() + 1) break;
    // Add a LOAD command.
    {
      auto* cmd = drone_commands->add_drone_command();
      cmd->set_type(DroneCommand_CommandType_LOAD);
      cmd->set_drone(0);
      cmd->set_warehouse(0);
      cmd->set_product(0);
      cmd->set_num_items(1);
      cmd->set_start_time(curr_time);
      curr_time += prev_dist + 1;
    }
    // Add a DELIVER command.
    {
      auto* cmd = drone_commands->add_drone_command();
      cmd->set_type(DroneCommand_CommandType_DELIVER);
      cmd->set_drone(0);
      cmd->set_order(order.second);
      cmd->set_product(0);
      cmd->set_num_items(1);
      cmd->set_start_time(curr_time);
      curr_time += order.first + 1;
    }

    prev_dist = order.first;
  }
  return solution;
}

bool Sp1Solver::CanSolve(const ProblemType& problem_type) const {
  return problem_type.has_nd_1() && problem_type.nd_1() &&
         problem_type.has_m_m() && problem_type.m_m() &&
         problem_type.has_nw_1() && problem_type.nw_1() &&
         problem_type.has_s0_inf() && problem_type.s0_inf() &&
         problem_type.has_np_1() && problem_type.np_1() &&
         problem_type.has_ipo_1() && problem_type.ipo_1();
}

}  // namespace drones
