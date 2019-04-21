#include "solvers/cwf_solver/cwf_solver.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <random>
#include <utility>
#include <vector>

#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "solvers/util/allocator.h"
#include "solvers/util/load_splitter.h"
#include "solvers/util/lp_util.h"

namespace drones {

std::unique_ptr<Solution> CwfSolver::Solve() {
  // Closest warehouses.
  std::vector<int> closest_w(problem_.no());
  for (int o = 0; o < problem_.no(); o++) {
    int best_w = -1;
    double min_dist = std::numeric_limits<double>::max();
    for (int w = 0; w < problem_.nw(); w++) {
      double dx = problem_.order(o).location().x() -
                  problem_.warehouse(w).location().x();
      double dy = problem_.order(o).location().y() -
                  problem_.warehouse(w).location().y();
      double dist = dx * dx + dy * dy;
      if (dist < min_dist) {
        min_dist = dist;
        best_w = w;
      }
    }
    CHECK(best_w >= 0);
    closest_w[o] = best_w;
  }

  // Allocate.
  util::Allocator::DistFn dist_fn = [&](int o, int w, int p) {
    double dx_o =
        problem_.warehouse(w).location().x() - problem_.order(o).location().x();
    double dy_o =
        problem_.warehouse(w).location().y() - problem_.order(o).location().y();
    double d_o = ceil(sqrt(dx_o * dx_o + dy_o * dy_o)) + 1;
    double dx_w = problem_.warehouse(w).location().x() -
                  problem_.warehouse(closest_w.at(o)).location().x();
    double dy_w = problem_.warehouse(w).location().y() -
                  problem_.warehouse(closest_w.at(o)).location().y();
    double d_w = ceil(sqrt(dx_w * dx_w + dy_w * dy_w)) + 1;
    double d = std::min(d_o, d_w);
    return 2.0 * d;
  };
  auto alloc = util::Allocator::AllocateWithDistFn(problem_, dist_fn);
  CHECK(util::Allocator::VerifyAlloc(problem_, alloc)) << "Invalid alloc!";
  LOG(INFO) << "Successfully allocated.";
  
  // Needed for LoadSplitter::Split.
  std::map<int, int> product_weights;
  for (int p = 0; p < problem_.np(); p++) {
    product_weights[p] = problem_.product(p).m();
  }
  
  // Oder permutation.
  std::vector<int> order_perm;
  {
    std::vector<std::pair<int, int>> order_durations;

    for (int o = 0; o < problem_.no(); o++) {
      int ox = problem_.order(o).location().x();
      int oy = problem_.order(o).location().y();

      // Number of warehouses used for this order.
      int total_wh = 0;
      double mwx = 0.0;
      double mwy = 0.0;
      for (int w = 0; w < problem_.nw(); w++) {
        for (int p = 0; p < problem_.np(); p++) {
          if (alloc[o].count({w, p})) {
            mwx += problem_.warehouse(w).location().x();
            mwy += problem_.warehouse(w).location().y();
            total_wh++;
            break;
          }
        }
      }
      mwx /= total_wh;
      mwy /= total_wh;

      int completion_time = 0;

      int num_items = 0;
      for (const auto& wp_items : alloc[o]) {
        int w = wp_items.first.first;
        int n = wp_items.second;
        CHECK(n > 0);

        num_items += n;

        if (w == closest_w[o]) continue;

        double wx = problem_.warehouse(closest_w[o]).location().x();
        double wy = problem_.warehouse(closest_w[o]).location().y();
        double dx = mwx - wx;
        double dy = mwy - wy;
        completion_time += n * 2 * (ceil(sqrt(dx * dx + dy * dy)) + 1);
      }

      double dx = ox - problem_.warehouse(closest_w[o]).location().x();
      double dy = oy - problem_.warehouse(closest_w[o]).location().y();
      completion_time += num_items * 2 * (ceil(sqrt(dx * dx + dy * dy)) + 1);
      order_durations.push_back({completion_time, o});
    }
    std::sort(order_durations.begin(), order_durations.end());
    for (const auto& dur_ord : order_durations) {
      order_perm.push_back(dur_ord.second);
    }
  }

  // Init solution.
  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  for (int d = 0; d < problem_.nd(); d++) {
    solution->add_drone_desc();
  }

  // Warehouse surpluses.
  std::vector<std::map<int, int>> surplus(problem_.nw());
  for (int w = 0; w < problem_.nw(); w++) {
    for (int p = 0; p < problem_.np(); p++) {
      if (problem_.warehouse(w).stock(p) > 0) {
        surplus[w][p]++;
      }
    }
  }

  // What is needed for the current order.
  std::map<int, int> needed;

  // Which warehouses have what left to deliver to the current oder's closest
  // warehouse.
  std::map<int, std::map<int, int>> pending_warehouses;

  // Drones book-keeping.
  std::vector<int> drone_busy_until(problem_.nd(), 0);
  std::vector<Location> drone_location(problem_.nd(),
                                       problem_.warehouse(0).location());

  // Handle orders one by one.
  for (int o : order_perm) {
    const double cwx = problem_.warehouse(closest_w[o]).location().x();
    const double cwy = problem_.warehouse(closest_w[o]).location().y();

    // Set needed.
    needed.clear();
    for (int p = 0; p < problem_.np(); p++) {
      if (problem_.order(o).request(p) > 0) {
        needed[p] = problem_.order(o).request(p);
      }
    }

    // Set pending_warehouses.
    pending_warehouses.clear();
    for (const auto& wp_i : alloc[o]) {
      if (wp_i.first.first == closest_w[o]) continue;
      CHECK(wp_i.second > 0);
      pending_warehouses[wp_i.first.first][wp_i.first.second] = wp_i.second;
    }

    // Number of commands executed by each drone for this order. Used to
    // roll-back the commands in case of an unsuccessfull matching, so that an
    // another order can be given a chance.
    std::vector<int> num_drone_commands(problem_.nd(), 0);
    auto backup_drone_busy_until = drone_busy_until;
    auto backup_drone_location = drone_location;

    auto roll_back = [&]() {
      LOG(INFO) << "Rolling back.";
      for (int d = 0; d < problem_.nd(); d++) {
        while (num_drone_commands[d]--) {
          solution->mutable_drone_desc(d)
              ->mutable_drone_command()
              ->RemoveLast();
        }
      }
      drone_location = backup_drone_location;
      drone_busy_until = backup_drone_busy_until;
    };

    // Unloads of each product in the closest_w[o].
    // [p][t] -> set of numbers of items of product p unloaded at t
    std::vector<std::multimap<int, int>> product_unloads(problem_.np());
    // Waiting line for each product in closest_w[o].
    // [p] -> {last not completely taken unload, what's left}
    std::vector<std::pair<int, int>> wait_line(problem_.np(), {-1, 0});
    for (const auto& wp_i : alloc[o]) {
      if (wp_i.first.first == closest_w[o]) {
        product_unloads[wp_i.first.second].insert({-1, wp_i.second});
        wait_line[wp_i.first.second] = {-1, wp_i.second};
      }
    }

    // This is just bringing all the items to the closest warehouse.
    bool brought_all = true;
    while (!pending_warehouses.empty()) {
      // Find the best (w, d) match.
      int best_w = -1;
      int best_d = -1;
      int best_t = std::numeric_limits<int>::max();  // Earliest.
      bool match_found = false;
      for (const auto& w_stock : pending_warehouses) {
        int w = w_stock.first;
        for (int d = 0; d < problem_.nd(); d++) {
          int t = drone_busy_until[d];
          // LOAD dutation = dist to warehouse + 1
          {
            double dx =
                drone_location[d].x() - problem_.warehouse(w).location().x();
            double dy =
                drone_location[d].y() - problem_.warehouse(w).location().y();
            int dist = ceil(sqrt(dx * dx + dy * dy));
            t += dist + 1;
          }
          // UNLOAD duration = dist warehouse->closest warehouse + 1
          {
            double dx = problem_.warehouse(w).location().x() - cwx;
            double dy = problem_.warehouse(w).location().y() - cwy;
            int dist = ceil(sqrt(dx * dx + dy * dy));
            t += dist + 1;
          }
          // First compare by completion time, then by start time.
          // Sure, this duration approximation is considering only one product,
          // type, so it's too optimistic, but if it exceeds the simulation
          // time, the real duration will exceed as well.
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

      if (!match_found) {
        brought_all = false;
        break;
      }

      // Great, we have the match, let's see what we're taking.
      auto taking = util::LoadSplitter::SplitOnce(pending_warehouses[best_w],
                                                  product_weights, problem_.m())
                        .single_split;

      // Let's generate the LOAD commands.
      {
        double dx = drone_location[best_d].x() -
                    problem_.warehouse(best_w).location().x();
        double dy = drone_location[best_d].y() -
                    problem_.warehouse(best_w).location().y();
        int dist = ceil(sqrt(dx * dx + dy * dy));
        for (const auto& pi : taking) {
          if (pi.second < 1) continue;
          num_drone_commands[best_d]++;
          auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
          cmd->set_drone(best_d);
          cmd->set_type(DroneCommand_CommandType_LOAD);
          cmd->set_warehouse(best_w);
          cmd->set_product(pi.first);
          cmd->set_num_items(pi.second);
          cmd->set_start_time(drone_busy_until[best_d] + 1);
          drone_busy_until[best_d] += dist + 1;
          if (drone_busy_until[best_d] > problem_.t()) {
            LOG(INFO) << "    Exceeded!";
            brought_all = false;
            break;
          }
          dist = 0;
        }
      }

      if (!brought_all) break;

      // Let's generate the UNLOAD commands.
      {
        double dx = cwx - problem_.warehouse(best_w).location().x();
        double dy = cwy - problem_.warehouse(best_w).location().y();
        int dist = ceil(sqrt(dx * dx + dy * dy));
        for (const auto& pi : taking) {
          if (pi.second < 1) continue;
          num_drone_commands[best_d]++;
          auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
          cmd->set_drone(best_d);
          cmd->set_type(DroneCommand_CommandType_UNLOAD);
          cmd->set_warehouse(closest_w[o]);
          cmd->set_product(pi.first);
          cmd->set_num_items(pi.second);
          cmd->set_start_time(drone_busy_until[best_d] + 1);
          drone_busy_until[best_d] += dist + 1;
          product_unloads[pi.first].insert(
              {drone_busy_until[best_d], pi.second});
          if (drone_busy_until[best_d] > problem_.t()) {
            LOG(INFO) << "    Exceeded!";
            brought_all = false;
            break;
          }
          dist = 0;
        }
      }

      if (!brought_all) break;

      // We haven't rolled-back yet, which doesn't mean we won't roll-back later
      // in this order. Still, we have some backed-up/irrelevant updates to do.
      drone_location[best_d] = problem_.warehouse(closest_w[o]).location();
      // Remove taken from pending_warehouses.
      for (const auto& pi : taking) {
        if ((pending_warehouses[best_w][pi.first] -= pi.second) == 0) {
          pending_warehouses[best_w].erase(pi.first);
          if (pending_warehouses[best_w].empty()) {
            pending_warehouses.erase(best_w);
          }
        }
      }
    }

    // If we can't even bring the items to the closest warehouse, there's no
    // point in going further with this order.
    if (!brought_all) {
      roll_back();
      continue;
    }

    // At this point, we have it all in the closest warehouse, so let's deliver
    // it all.
    auto order_split =
        util::LoadSplitter::Split(needed, product_weights, problem_.m());
    bool success = true;
    while (order_split.total_times > 0) {
      auto& curr_split = order_split.repeated_splits.front();
      CHECK(curr_split.times > 0);

      // Find the most suitable drone.
      int best_d = -1;
      int best_wait_time = std::numeric_limits<int>::max();
      int best_load_time = std::numeric_limits<int>::max();
      std::map<int, std::pair<int, int>> wait_line_changes;
      std::map<int, std::pair<int, int>> best_wait_line_changes;
      for (int d = 0; d < problem_.nd(); d++) {
        double drone_x = drone_location[d].x();
        double drone_y = drone_location[d].y();
        wait_line_changes.clear();
        int wait_time = 0;
        int load_time = -1;
        for (const auto& pi : curr_split.single_split) {
          int p = pi.first;
          int n = pi.second;
          CHECK(n > 0);
          int load_dur =
              ceil(sqrt(pow(drone_x - cwx, 2) + pow(drone_y - cwy, 2))) + 1;
          drone_x = cwx;
          drone_y = cwy;
          int ideal_load_time = drone_busy_until[d] + wait_time + load_dur;
          int last_delivery = wait_line[p].first;
          int leftover = wait_line[p].second;
          int possible_load_time = std::max(ideal_load_time, last_delivery + 1);
          if (leftover < n) {
            auto it = product_unloads[p].upper_bound(last_delivery + 1);
            while (it != product_unloads[p].end()) {
              last_delivery = it->first;
              leftover += it->second;
              it++;
              if (leftover >= n) break;
            }
            for (; it != product_unloads[p].end() && it->first == last_delivery;
                 it++) {
              leftover += it->second;
            }
            possible_load_time = std::max(ideal_load_time, last_delivery + 1);
          }
          leftover -= n;
          CHECK(leftover >= 0);
          wait_line_changes[p] = {last_delivery, leftover};
          wait_time += possible_load_time - ideal_load_time;
          load_time = possible_load_time;
        }
        if (load_time < best_load_time ||
            (load_time == best_load_time && wait_time < best_wait_time)) {
          best_load_time = load_time;
          best_wait_time = wait_time;
          best_d = d;
          best_wait_line_changes = wait_line_changes;
        }
      }

      // Generate WAIT (if needed) and LOADs.
      if (best_load_time > problem_.t()) {
        LOG(INFO) << "    Exceeded";
        success = false;
        break;
      }
      if (best_wait_time > 0) {
        num_drone_commands[best_d]++;
        auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
        cmd->set_drone(best_d);
        cmd->set_type(DroneCommand_CommandType_WAIT);
        cmd->set_duration(best_wait_time);
        cmd->set_start_time(drone_busy_until[best_d] + 1);
        drone_busy_until[best_d] += best_wait_time;
      }
      {
        int dx = drone_location[best_d].x() - cwx;
        int dy = drone_location[best_d].y() - cwy;
        int dist = ceil(sqrt(dx * dx + dy * dy));
        for (const auto& pi : curr_split.single_split) {
          CHECK(pi.second > 0);
          num_drone_commands[best_d]++;
          auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
          cmd->set_drone(best_d);
          cmd->set_type(DroneCommand_CommandType_LOAD);
          cmd->set_warehouse(closest_w[o]);
          cmd->set_product(pi.first);
          cmd->set_num_items(pi.second);
          cmd->set_start_time(drone_busy_until[best_d] + 1);
          wait_line[pi.first] = best_wait_line_changes[pi.first];
          drone_busy_until[best_d] += dist + 1;
          dist = 0;
        }
      }

      // Generate DELIVERs.
      {
        int dx = problem_.order(o).location().x() - cwx;
        int dy = problem_.order(o).location().y() - cwy;
        int dist = ceil(sqrt(dx * dx + dy * dy));
        for (const auto& pi : curr_split.single_split) {
          CHECK(pi.second > 0);
          num_drone_commands[best_d]++;
          auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
          cmd->set_drone(best_d);
          cmd->set_type(DroneCommand_CommandType_DELIVER);
          cmd->set_order(o);
          cmd->set_product(pi.first);
          cmd->set_num_items(pi.second);
          cmd->set_start_time(drone_busy_until[best_d] + 1);
          drone_busy_until[best_d] += dist + 1;
          if (drone_busy_until[best_d] > problem_.t()) {
            LOG(INFO) << "    Exceeded!";
            success = false;
            break;
          }
          dist = 0;
        }
        if (!success) break;
      }

      // We haven't rolled-back yet, which doesn't mean we won't roll-back later
      // in this order. Still, we have some backed-up/irrelevant updates to do.
      drone_location[best_d] = problem_.order(o).location();
      // Update order_split.
      order_split.total_times--;
      order_split.total_num_items -= curr_split.num_items;
      if (--curr_split.times == 0) {
        order_split.repeated_splits.pop_front();
      }
    }

    if (!success) {
      roll_back();
      continue;
    }
  }

  return solution;
}

}  // namespace drones
