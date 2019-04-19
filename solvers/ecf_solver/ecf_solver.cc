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
#include "solvers/util/load_splitter.h"
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

  std::map<int, int> product_weights;
  for (int p = 0; p < problem_.np(); p++) {
    product_weights[p] = problem_.product(p).m();
  }

  LOG(INFO) << "Making the (order, warehouse) splits to drones.";
  // [order][warehouse] -> split
  std::vector<std::map<int, util::LoadSplitter::CompleteSplit>> order_splits(
      problem_.no());
  for (int o = 0; o < problem_.no(); o++) {
    // [product] -> num items
    std::map<int, int> from_curr_w;
    int prev_w = -1;
    auto calc_and_add_split = [&]() {
      if (from_curr_w.empty()) return;
      CHECK(prev_w >= 0);
      CHECK(prev_w < problem_.no());
      auto split =
          util::LoadSplitter::Split(from_curr_w, product_weights, problem_.m());
      order_splits[o][prev_w] = split;
    };
    for (const auto& wp_i : alloc[o]) {
      int w = wp_i.first.first;
      int p = wp_i.first.second;
      int n = wp_i.second;
      CHECK(n > 0);
      if (prev_w != w) {
        calc_and_add_split();
        from_curr_w.clear();
      }
      from_curr_w[p] = n;
      prev_w = w;
    }
    calc_and_add_split();
  }
  LOG(INFO) << "Successfully split.";

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
    // Sum of all locations devided by num locations - 1, so that we can easily
    // calculate mean of other locations for each location.
    double mox = 0.0;
    double moy = 0.0;
    if (problem_.no() > 1) {
      for (int o1 = 0; o1 < problem_.no(); o1++) {
        mox += static_cast<double>(problem_.order(o1).location().x()) /
               (problem_.no() - 1);
        moy += static_cast<double>(problem_.order(o1).location().y()) /
               (problem_.no() - 1);
      }
    }

    for (int o = 0; o < problem_.no(); o++) {
      int ox = problem_.order(o).location().x();
      int oy = problem_.order(o).location().y();
      // Mean prevoius order location.
      double mpox = 0;
      double mpoy = 0;
      if (problem_.no() > 1) {
        mpox = mox - static_cast<double>(ox) / (problem_.no() - 1);
        mpoy = moy - static_cast<double>(oy) / (problem_.no() - 1);
      }
      // Mean amoung current warehouses.
      double mwx = 0;
      double mwy = 0;
      int completion_time = 0;
      for (const auto& w_split : order_splits[o]) {
        double wx = problem_.warehouse(w_split.first).location().x();
        double wy = problem_.warehouse(w_split.first).location().y();
        mwx += wx / order_splits[o].size();
        mwy += wy / order_splits[o].size();
        double dx = ox - wx;
        double dy = oy - wy;
        completion_time += (2.0 - 1.0 / order_splits[o].size()) *
                           w_split.second.total_times *
                           (ceil(sqrt(dx * dx + dy * dy)) + 1);
      }
      double dx = mpox - mwx;
      double dy = mpoy - mwy;
      completion_time += ceil(sqrt(dx * dx + dy * dy)) + 1;
      order_durations.push_back({completion_time, o});
    }
    std::sort(order_durations.begin(), order_durations.end());
  }

  // Drones.
  std::vector<int> drone_busy_until(problem_.nd(), 0);
  std::vector<Location> drone_location(problem_.nd(),
                                       problem_.warehouse(0).location());

  // For each order, we'll repeatedly match a warehouse and a drone (earliest
  // ending time of LOAD + DELIVER) and pack the drone according to the
  // previously calculated split.
  //
  // Note: Sure, finding the best match depends on the number of product types
  // we're packing since loading of each costs 1 time moment, but we'll assume
  // that the cost of travel is much greater than that!

  // Handle orders one by one.
  int num_order = 0;
  for (const auto& od : order_durations) {
    num_order++;
    LOG(INFO) << "Order " << num_order << "/" << order_durations.size();

    int o = od.second;

    // Number of commands executed by each drone for this order. Used to
    // roll-back the commands in case of an unsuccessfull matching, so that an
    // another order can be given a chance.
    std::vector<int> num_drone_commands(problem_.nd(), 0);

    auto calc_total_pending_items = [&]() {
      int res = 0;
      for (const auto& w_split : order_splits[o]) {
        int total_wh = w_split.second.total_num_items;
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
    while (!order_splits[o].empty()) {
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
      for (const auto& w_split : order_splits[o]) {
        int w = w_split.first;
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

      auto load = order_splits[o][best_w].repeated_splits.front().single_split;

      int curr_drone_busy_until = drone_busy_until[best_d];

      // Let's generate the LOAD commands.
      // LOG(INFO) << "    LOAD commands.";
      {
        int dx = drone_location[best_d].x() -
                 problem_.warehouse(best_w).location().x();
        int dy = drone_location[best_d].y() -
                 problem_.warehouse(best_w).location().y();
        int dist = ceil(sqrt(dx * dx + dy * dy));
        for (const auto& pi : load) {
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
        for (const auto& pi : load) {
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
      order_splits[o][best_w].total_times--;
      order_splits[o][best_w].total_num_items -=
          order_splits[o][best_w].repeated_splits.front().num_items;
      if ((order_splits[o][best_w].repeated_splits.front().times -= 1) == 0) {
        order_splits[o][best_w].repeated_splits.pop_front();
        if (order_splits[o][best_w].total_times == 0) {
          order_splits[o].erase(best_w);
          num_warehouse++;
          LOG(INFO) << "  Warehouses done: " << num_warehouse;
        }
      }
    }
  }
  return solution;
}

bool EcfSolver::CanSolve(const ProblemType& problem_type) const { return true; }

}  // namespace drones
