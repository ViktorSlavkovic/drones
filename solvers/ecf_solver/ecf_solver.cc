#include "solvers/ecf_solver/ecf_solver.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <utility>
#include <vector>

#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "glog/logging.h"
#include "solvers/util/allocator.h"
#include "solvers/util/lp_util.h"

namespace drones {

std::unique_ptr<Solution> EcfSolver::Solve() {
  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  for (int d = 0; d < problem_.nd(); d++) {
    solution->add_drone_desc();
  }

  auto alloc = util::Allocator::Allocate(problem_);
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

  // Drones.
  std::vector<int> drone_busy_until(problem_.nd(), 0);
  std::vector<Location> drone_location(problem_.nd(),
                                       problem_.warehouse(0).location());

  // For each order, we'll repeatedly match a warehouse and a drone (earliest
  // ending time of LOAD + DELIVER) and do a knapsack packing of the items to
  // maximize used volume.
  //
  // Important: Sure, finding the best match depends on the number of product
  // types we're packing since loading of each costs 1 time moment, but we'll
  // assume that the cost of travel is much greater than that!
  //
  // best volume usage at each volume
  std::vector<int> knapsack_best(problem_.m() + 1);
  // [volume][product] -> num_items - taken items at each volume
  std::vector<std::map<int, int>> knapsack_taken(problem_.m() + 1);

  // Handle orders one by one.
  int num_order = 0;
  for (const auto& od : order_durations) {
    num_order++;
    LOG(INFO) << "Order " << num_order << "/" << order_durations.size();

    int o = od.second;

    // [warehouse][product] -> num_items
    std::map<int, std::map<int, int>> pending_warehouses;
    for (const auto& wp_items : alloc[o]) {
      CHECK(wp_items.second > 0);
      pending_warehouses[wp_items.first.first][wp_items.first.second] +=
          wp_items.second;
    }
    CHECK(!pending_warehouses.empty());

    // Number of commands executed by each drone for this order. Used to
    // roll-back the commands in case of an unsuccessfull matching, so that an
    // another order can be given a chance.
    std::vector<int> num_drone_commands(problem_.nd(), 0);

    auto calc_total_pending_items = [&]() {
      int res = 0;
      for (const auto& w_pi : pending_warehouses) {
        int total_wh = 0;
        for (const auto& pi : w_pi.second) {
          CHECK(pi.second > 0);
          total_wh += pi.second;
        }
        CHECK(total_wh > 0);
        res += total_wh;
      }
      return res;
    };

    int total_pending_items = calc_total_pending_items() +
                              1;  // 1 to make sure the check at the loop
                                  // beginning is valid in the first pass.
    CHECK(total_pending_items > 1);
    LOG(INFO) << "  Initial pending items: " << total_pending_items - 1;
    int num_warehouse = 0;

    while (!pending_warehouses.empty()) {
      int curr_total_pending_items = calc_total_pending_items();
      CHECK(curr_total_pending_items > 0)
          << curr_total_pending_items << " > " << 0;
      CHECK(curr_total_pending_items < total_pending_items)
          << curr_total_pending_items << " < " << total_pending_items;
      total_pending_items = curr_total_pending_items;
      LOG(INFO) << "  Pending items: " << total_pending_items;

      // Find the best (warehouse, drone) match.
      // LOG(INFO) << "    Finding the best match.";
      int best_w = -1;
      int best_d = -1;
      int best_t = std::numeric_limits<int>::max();  // Earliest ending time.
      bool match_found = false;
      for (const auto& w_pi : pending_warehouses) {
        int w = w_pi.first;
        for (int d = 0; d < problem_.nd(); d++) {
          int t = drone_busy_until[d];
          // LOAD dutation = dist to warehouse + 1
          {
            int dx =
                drone_location[d].x() - problem_.warehouse(w).location().x();
            int dy =
                drone_location[d].y() - problem_.warehouse(w).location().y();
            int dist = ceil(sqrt(dx * dx + dy * dy));
            t += dist + 1;
          }
          // DELIVER duration = dist warehouse->order + 1
          {
            int dx = problem_.warehouse(w).location().x() -
                     problem_.order(o).location().x();
            int dy = problem_.warehouse(w).location().y() -
                     problem_.order(o).location().y();
            int dist = ceil(sqrt(dx * dx + dy * dy));
            t += dist + 1;
          }
          // First compare by completion time, then by start time.
          if (t <= problem_.t() &&
              (t < best_t || (t == best_t && drone_busy_until[d] >
                                                 drone_busy_until[best_d]))) {
            best_d = d;
            best_w = w;
            best_t = t;
            match_found = true;
          }
        }
      }

      // If there's no match, we can't fit any commands in the remaining time,
      // so we finish here.
      if (!match_found) {
        // Roll-back.
        LOG(INFO) << "    No match found.";
      roll_back:
        LOG(INFO) << "    Rolling back.";
        for (int d = 0; d < problem_.nd(); d++) {
          while (num_drone_commands[d]--) {
            solution->mutable_drone_desc(d)
                ->mutable_drone_command()
                ->RemoveLast();
          }
        }
        break;
      }

      // Yay, we have the match! Let's knapsack-pack the product items then.
      // LOG(INFO) << "    Knapsack";
      knapsack_best[0] = 0;
      knapsack_taken[0].clear();
      int drone_cap = problem_.m();
      for (int volume = 1; volume <= drone_cap; volume++) {
        knapsack_best[volume] = 0;
        knapsack_taken[volume].clear();
        for (const auto& pi : pending_warehouses[best_w]) {
          int p = pi.first;
          int pm = problem_.product(p).m();
          if (volume < pm) continue;
          // Let's try to add one item of p to volume - pm case.
          if (pm + knapsack_best[volume - pm] > knapsack_best[volume] &&
              knapsack_taken[volume - pm][p] + 1 <= pi.second) {
            // Ouch! Expensive!
            knapsack_taken[volume] = knapsack_taken[volume - pm];
            knapsack_taken[volume][p]++;
            knapsack_best[volume] = knapsack_best[volume - pm] + pm;
            continue;
          }
          // If not, let's see if original volume - pm case is still better
          // then what we have now.
          if (knapsack_best[volume - pm] > knapsack_best[volume]) {
            // Ouch! Expensive!
            knapsack_taken[volume] = knapsack_taken[volume - pm];
            knapsack_best[volume] = knapsack_best[volume - pm];
          }
        }
      }
      CHECK(knapsack_best[drone_cap] > 0);

      int curr_drone_busy_until = drone_busy_until[best_d];

      // Let's generate the LOAD commands.
      // LOG(INFO) << "    LOAD commands.";
      {
        int dx = drone_location[best_d].x() -
                 problem_.warehouse(best_w).location().x();
        int dy = drone_location[best_d].y() -
                 problem_.warehouse(best_w).location().y();
        int dist = ceil(sqrt(dx * dx + dy * dy));
        for (const auto& pi : knapsack_taken[problem_.m()]) {
          if (pi.second < 1) continue;
          num_drone_commands[best_d]++;
          auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
          cmd->set_drone(best_d);
          cmd->set_type(DroneCommand_CommandType_LOAD);
          cmd->set_warehouse(best_w);
          cmd->set_product(pi.first);
          cmd->set_num_items(pi.second);
          cmd->set_start_time(curr_drone_busy_until + 1);
          curr_drone_busy_until += dist + 1;
          if (curr_drone_busy_until > problem_.t()) {
            LOG(INFO) << "    Exceeded!";
            goto roll_back;
          }
          dist = 0;
        }
      }

      // Let's generate the DELIVER commands.
      // LOG(INFO) << "    DELIVER commands.";
      {
        int dx = problem_.order(o).location().x() -
                 problem_.warehouse(best_w).location().x();
        int dy = problem_.order(o).location().y() -
                 problem_.warehouse(best_w).location().y();
        int dist = ceil(sqrt(dx * dx + dy * dy));
        for (const auto& pi : knapsack_taken[problem_.m()]) {
          if (pi.second < 1) continue;
          num_drone_commands[best_d]++;
          auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
          cmd->set_drone(best_d);
          cmd->set_type(DroneCommand_CommandType_DELIVER);
          cmd->set_order(o);
          cmd->set_product(pi.first);
          cmd->set_num_items(pi.second);
          cmd->set_start_time(curr_drone_busy_until + 1);
          curr_drone_busy_until += dist + 1;
          if (curr_drone_busy_until > problem_.t()) {
            LOG(INFO) << "    Exceeded!";
            goto roll_back;
          }
          dist = 0;
        }
      }

      // If we're definitely doing all this (not rolling back), update drone
      // info and pending_warehouses.
      // LOG(INFO) << "    Success, updating.";
      drone_location[best_d] = problem_.order(o).location();
      drone_busy_until[best_d] = curr_drone_busy_until;
      for (const auto& pi : knapsack_taken[problem_.m()]) {
        pending_warehouses[best_w][pi.first] -= pi.second;
        if (pending_warehouses[best_w][pi.first] == 0) {
          pending_warehouses[best_w].erase(pi.first);
          if (pending_warehouses[best_w].empty()) {
            pending_warehouses.erase(best_w);
            num_warehouse++;
            LOG(INFO) << "  Warehouses done: " << num_warehouse;
          }
        }
      }
    }
  }
  return solution;
}

bool EcfSolver::CanSolve(const ProblemType& problem_type) const { return true; }

}  // namespace drones
