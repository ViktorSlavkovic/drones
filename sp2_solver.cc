#include "sp2_solver.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace drones {

std::unique_ptr<Solution> Sp2Solver::Solve() {
  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  auto* drone_commands = solution->add_drone_desc();
  std::vector<std::pair<int, int>> distances;
  
  for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
    int dx = problem_.order(0).location().x() -
             problem_.warehouse(warehouse).location().x();
    int dy = problem_.order(0).location().y() -
             problem_.warehouse(warehouse).location().y();
    int d = static_cast<int>(ceil(sqrt(dx * dx + dy * dy)));
    distances.push_back({d, warehouse});
  }
  int d0 = distances.front().first;
  std::sort(distances.begin(), distances.end());
  
  // It's definitely fastest to bring one item from the warehouse 0.
  //
  // If the simulation time is too short to execute the initial LOAD+DELIVER
  // pair, return an empty solution, there's no point in trying anything.
  if (1 + d0 + 1 > problem_.t()) {
    return solution;
  }
  // Add the initial LOAD command.
  {
    auto* cmd = drone_commands->add_drone_command();
    cmd->set_type(DroneCommand_CommandType_LOAD);
    cmd->set_drone_id(0);
    cmd->set_warehouse(0);
    cmd->set_product(0);
    cmd->set_num_items(1);
    cmd->set_start_time(1);
  }
  // Add the initial DELIVER command.
  {
    auto* cmd = drone_commands->add_drone_command();
    cmd->set_type(DroneCommand_CommandType_DELIVER);
    cmd->set_drone_id(0);
    cmd->set_order(0);
    cmd->set_product(0);
    cmd->set_num_items(1);
    cmd->set_start_time(2);
  }

  int curr_time = d0 + 3;
  int items_left = problem_.order(0).request(0) - 1;
  bool trimmed = false;
  for (const auto& warehouse : distances) {
    int warehouse_stock = problem_.warehouse(warehouse.second).stock(0);
    // We took one from WH0 at the beginning!
    if (warehouse.second == 0) {
      warehouse_stock--;
    }
    int taking = std::min(items_left, warehouse_stock);
    items_left -= taking;
    while (taking--) {
      // Add a LOAD command.
      {
        int next_time = curr_time + warehouse.first + 1;
        if (next_time > problem_.t() + 1) {
          trimmed = true;
          break;
        }
        auto* cmd = drone_commands->add_drone_command();
        cmd->set_type(DroneCommand_CommandType_LOAD);
        cmd->set_drone_id(0);
        cmd->set_warehouse(warehouse.second);
        cmd->set_product(0);
        cmd->set_num_items(1);
        cmd->set_start_time(curr_time);
        curr_time = next_time;
      }
      // Add a DELIVER command.
      {
        int next_time = curr_time + warehouse.first + 1;
        if (next_time > problem_.t() + 1) {
          trimmed = true;
          break;
        }
        auto* cmd = drone_commands->add_drone_command();
        cmd->set_type(DroneCommand_CommandType_DELIVER);
        cmd->set_drone_id(0);
        cmd->set_order(0);
        cmd->set_product(0);
        cmd->set_num_items(1);
        cmd->set_start_time(curr_time);
        curr_time = next_time;
      }
    }
    if (trimmed) {
      break;
    }
  } 
  return solution;
}

bool Sp2Solver::CanSolve(const ProblemType& problem_type) const {
  return problem_type.has_nd_1() && problem_type.nd_1() &&
         problem_type.has_m_m() && problem_type.m_m() &&
         problem_type.has_np_1() && problem_type.np_1() &&
         problem_type.has_no_1() && problem_type.no_1();
}

}  // namespace drones
