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
    int dx = problem_.order(order).location().x() -
             problem_.warehouse(0).location().x();
    int dy = problem_.order(order).location().y() -
             problem_.warehouse(0).location().y();
    int d = static_cast<int>(ceil(sqrt(dx * dx + dy * dy)));
    distances.push_back({d, order});
  }
  std::sort(distances.begin(), distances.end());
  // Add the initial LOAD command.
  {
    auto* cmd = drone_commands->add_drone_command();
    cmd->set_type(DroneCommand_CommandType_LOAD);
    cmd->set_drone(0);
    cmd->set_warehouse(0);
    cmd->set_product(0);
    cmd->set_num_items(1);
    cmd->set_start_time(1);
  }
  int curr_time = 2;
  bool trimmed = false;
  for (const auto& order : distances) {
    // Add a DELIVER command.
    {
      int next_time = curr_time + order.first + 1;
      if (next_time > problem_.t() + 1) {
        trimmed = true;
        break;
      }
      auto* cmd = drone_commands->add_drone_command();
      cmd->set_type(DroneCommand_CommandType_DELIVER);
      cmd->set_drone(0);
      cmd->set_order(order.second);
      cmd->set_product(0);
      cmd->set_num_items(1);
      cmd->set_start_time(curr_time);
      curr_time = next_time;
    }
    // Add a LOAD command.
    {
      int next_time = curr_time + order.first + 1;
      if (next_time > problem_.t() + 1) {
        trimmed = true;
        break;
      }
      auto* cmd = drone_commands->add_drone_command();
      cmd->set_type(DroneCommand_CommandType_LOAD);
      cmd->set_drone(0);
      cmd->set_warehouse(0);
      cmd->set_product(0);
      cmd->set_num_items(1);
      cmd->set_start_time(curr_time);
      curr_time = next_time;
    }
  }
  // Remove the last LOAD since it might be invalid.
  if (!trimmed) {
    drone_commands->mutable_drone_command()->RemoveLast();
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
