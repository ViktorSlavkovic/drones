#include "solvers/sp4_solver/sp4_solver.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>
#include <vector>

#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "glog/logging.h"
#include "solvers/util/allocator.h"

namespace drones {

std::unique_ptr<Solution> Sp4Solver::Solve() {
  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  auto* drone_commands = solution->add_drone_desc();
  
  util::Allocator::DistFn dist_fn = [&](int o, int w, int p) {
    double dx =
        problem_.warehouse(w).location().x() - problem_.order(o).location().x();
    double dy =
        problem_.warehouse(w).location().y() - problem_.order(o).location().y();
    double d = ceil(sqrt(dx * dx + dy * dy)) + 1;
    return 2.0 * d;
  };

  auto alloc = util::Allocator::AllocateWithDistFn(problem_, dist_fn);
  if (!util::Allocator::VerifyAlloc(problem_, alloc)) {
    LOG(ERROR) << "Invalid alloc!";
    return nullptr;
  }
  LOG(INFO) << "Successfully allocated.";

  // Let's compute approximate completion time for each order:
  //   * we know exact LOAD time for all items except the first one, since we
  //     know all the alloc pairs and we're always coming from the current
  //     order. But, for the sake of simplicity, let's assume the first one is
  //     coming from the current order as well. Summed up,they are Lall
  //   * we know exact DELIVER time for all items, since we know all the alloc
  //     pairs, all summed up, they are Dall
  //   So, order completion time is: T = Lall + Dall
  //
  // Vector order_durations stores pairs {duration, order}, in ascending order.
  std::vector<std::pair<int, int>> order_durations;
  {
    double molx = 0;
    double moly = 0;
    for (const auto& order : problem_.order()) {
      molx += order.location().x();
      moly += order.location().y();
    }
    molx /= problem_.no();
    moly /= problem_.no();

    for (int o = 0; o < problem_.no(); o++) {
      int ox = problem_.order(o).location().x();
      int oy = problem_.order(o).location().y();
      int completion_time = 0;
      for (const auto& wp_items : alloc[o]) {
        CHECK(wp_items.second >= 0);
        int dx = ox - problem_.warehouse(wp_items.first.first).location().x();
        int dy = oy - problem_.warehouse(wp_items.first.first).location().y();
        completion_time +=
            2 * wp_items.second * (ceil(sqrt(dx * dx + dy * dy)) + 1);
      }
      order_durations.push_back({completion_time, o});
    }
    std::sort(order_durations.begin(), order_durations.end());
  }

  // We're doing a greedy solution, taking orders from order_durations, and
  // welding them together the best we can, by choosing the best starting
  // warehouse for every order.
  std::vector<int> starting_warehouse(problem_.no());
  {
    // The first order is a special case, we're considering distance from
    // warehouse 0, not from the previous order's location.
    int last_x = problem_.warehouse(0).location().x();
    int last_y = problem_.warehouse(0).location().y();
    for (const auto& od : order_durations) {
      int o = od.second;
      int last_w = -1;
      int best_w = -1;
      int min_dist = std::numeric_limits<int>::max();
      for (const auto& wp_items : alloc[o]) {
        CHECK(wp_items.second >= 0);
        int w = wp_items.first.first;
        if (w == last_w) continue;
        if (wp_items.second == 0) continue;
        last_w = w;
        int dx = last_x - problem_.warehouse(w).location().x();
        int dy = last_y - problem_.warehouse(w).location().y();
        int d = ceil(sqrt(dx * dx + dy * dy));
        if (d < min_dist) {
          best_w = w;
          min_dist = d;
          if (d == 0) {
            break;
          }
        }
      }
      CHECK(best_w >= 0);
      starting_warehouse[o] = best_w;
      last_x = problem_.order(o).location().x();
      last_y = problem_.order(o).location().y();
    }
  }

  int start_time = 1;
  auto loc = problem_.warehouse(0).location();
  for (const auto& od : order_durations) {
    int o = od.second;
    // Handle the first load.
    {
      int w = starting_warehouse[o];
      auto it = alloc[o].lower_bound({w, 0});
      while (it->second == 0) {
        it++;
      }
      CHECK(it->second > 0);
      CHECK(it->first.first == w);
      int dx = loc.x() - problem_.warehouse(w).location().x();
      int dy = loc.y() - problem_.warehouse(w).location().y();
      int finish_time = start_time + ceil(sqrt(dx * dx + dy * dy));
      CHECK_EQ(dx == 0 && dy == 0, start_time == finish_time);
      if (finish_time > problem_.t()) {
        break;
      }
      auto* cmd = drone_commands->add_drone_command();
      cmd->set_type(DroneCommand_CommandType_LOAD);
      cmd->set_drone(0);
      cmd->set_warehouse(w);
      cmd->set_product(it->first.second);
      cmd->set_num_items(1);
      cmd->set_start_time(start_time);
      start_time = finish_time + 1;
      loc = problem_.warehouse(w).location();
    }
    // Handle the first deliver.
    {
      int w = starting_warehouse[o];
      auto it = alloc[o].lower_bound({w, 0});
      while (it->second == 0) {
        it++;
      }
      CHECK(it->second > 0);
      CHECK(it->first.first == w);
      int dx = loc.x() - problem_.order(o).location().x();
      int dy = loc.y() - problem_.order(o).location().y();
      int finish_time = start_time + ceil(sqrt(dx * dx + dy * dy));
      CHECK_EQ(dx == 0 && dy == 0, start_time == finish_time);
      if (finish_time > problem_.t()) {
        break;
      }
      auto* cmd = drone_commands->add_drone_command();
      cmd->set_type(DroneCommand_CommandType_DELIVER);
      cmd->set_drone(0);
      cmd->set_order(o);
      cmd->set_product(it->first.second);
      cmd->set_num_items(1);
      cmd->set_start_time(start_time);
      start_time = finish_time + 1;
      loc = problem_.order(o).location();
      (it->second)--;
    }

    bool trimmed = false;

    for (const auto& wp_items : alloc[o]) {
      int w = wp_items.first.first;
      int p = wp_items.first.second;
      int num_items = wp_items.second;
      CHECK(num_items >= 0);
      while (num_items--) {
        // LOAD.
        {
          int dx = loc.x() - problem_.warehouse(w).location().x();
          int dy = loc.y() - problem_.warehouse(w).location().y();
          int finish_time = start_time + ceil(sqrt(dx * dx + dy * dy));
          CHECK_EQ(dx == 0 && dy == 0, start_time == finish_time);
          if (finish_time > problem_.t()) {
            trimmed = true;
            break;
          }
          auto* cmd = drone_commands->add_drone_command();
          cmd->set_type(DroneCommand_CommandType_LOAD);
          cmd->set_drone(0);
          cmd->set_warehouse(w);
          cmd->set_product(p);
          cmd->set_num_items(1);
          cmd->set_start_time(start_time);
          start_time = finish_time + 1;
          loc = problem_.warehouse(w).location();
        }
        // DELIVER
        {
          int dx = loc.x() - problem_.order(o).location().x();
          int dy = loc.y() - problem_.order(o).location().y();
          int finish_time = start_time + ceil(sqrt(dx * dx + dy * dy));
          CHECK_EQ(dx == 0 && dy == 0, start_time == finish_time);
          if (finish_time > problem_.t()) {
            trimmed = true;
            break;
          }
          auto* cmd = drone_commands->add_drone_command();
          cmd->set_type(DroneCommand_CommandType_DELIVER);
          cmd->set_drone(0);
          cmd->set_order(o);
          cmd->set_product(p);
          cmd->set_num_items(1);
          cmd->set_start_time(start_time);
          start_time = finish_time + 1;
          loc = problem_.order(o).location();
        }
      }
      if (trimmed) {
        break;
      }
    }
    if (trimmed) {
      break;
    }
  }
  return solution;
}

bool Sp4Solver::CanSolve(const ProblemType& problem_type) const {
  return problem_type.has_nd_1() && problem_type.nd_1() &&
         problem_type.has_m_m() && problem_type.m_m();
}

}  // namespace drones
