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

CwfSolver::CwfSolver(const Problem& problem)
    : ProblemSolver(problem), closest_w_(problem.no()), product_weights_() {
  CalcClosestWarehouses();
  for (int p = 0; p < problem_.np(); p++) {
    product_weights_[p] = problem_.product(p).m();
  }
}

void CwfSolver::CalcClosestWarehouses() {
  CHECK(closest_w_.size() == problem_.no());
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
    closest_w_[o] = best_w;
  }
}

double CwfSolver::EstimateOrderDuration(const util::Allocator::Alloc& alloc,
                                        int o) const {
  int ox = problem_.order(o).location().x();
  int oy = problem_.order(o).location().y();
  int cwx = problem_.warehouse(closest_w_[o]).location().x();
  int cwy = problem_.warehouse(closest_w_[o]).location().y();

  // Let's make things easier.
  // [w][p] -> num items
  std::map<int, std::map<int, int>> wpn;
  for (const auto& wp_i : alloc[o]) {
    CHECK(wp_i.second > 0);
    wpn[wp_i.first.first][wp_i.first.second] = wp_i.second;
  }

  // Make the splits.
  std::map<int, util::LoadSplitter::CompleteSplit> warehouse_splits;
  for (const auto& w_pn : wpn) {
    warehouse_splits[w_pn.first] =
        util::LoadSplitter::Split(w_pn.second, product_weights_, problem_.m());
  }

  std::map<int, int> from_closest = wpn[closest_w_[o]];

  double completion_time = 0;

  // For each warehouse, carry stuff to the nearest warehouse if it's closer,
  // otherwise, carry directly to the customer.
  for (const auto& w_split : warehouse_splits) {
    int w = w_split.first;
    if (w == closest_w_[o]) continue;

    double dox = ox - problem_.warehouse(w).location().x();
    double doy = oy - problem_.warehouse(w).location().y();
    double dcwx = cwx - problem_.warehouse(w).location().x();
    double dcwy = cwy - problem_.warehouse(w).location().y();
    int dist_o = ceil(sqrt(dox * dox + doy * doy));
    int dist_cw = ceil(sqrt(dcwx * dcwx + dcwy * dcwy));

    if (dist_o < dist_cw) {
      for (const auto& rsplit : w_split.second.repeated_splits) {
        // Unloads will take times * dist * num_products in single split, and
        // loads depend on where the drones are coming from, which can be
        // anywhere.
        // TODO(viktors): For now we'll approximate anywhere with order location
        // (rationale here is going back multiple times, but there are many
        // drones which can come from anywhere really. Consider using mean of
        // all locations for first load and order location for all the rest.
        // This is probably not very important.
        completion_time +=
            2.0 * rsplit.times * (dist_o + rsplit.single_split.size());
      }
    } else {
      for (const auto& rsplit : w_split.second.repeated_splits) {
        // Unloads will take times * dist * num_products in single split, and
        // loads depend on where the drones are coming from, which can be
        // anywhere.
        // TODO(viktors): Same as above, we'll use the closest warehouse as src
        // in LOADs.
        completion_time +=
            2.0 * rsplit.times * (dist_cw + rsplit.single_split.size());
        for (const auto& pi : rsplit.single_split) {
          from_closest[pi.first] += pi.second * rsplit.times;
        }
      }
    }
  }

  // Carry stuff from the closest warehouse to the order destination.
  {
    auto split =
        util::LoadSplitter::Split(from_closest, product_weights_, problem_.m());
    double dx = ox - cwx;
    double dy = oy - cwy;
    int dist = ceil(sqrt(dx * dx + dy * dy));
    for (const auto& rsplit : split.repeated_splits) {
      // TODO(viktors): Approx as above.
      completion_time +=
          2.0 * rsplit.times * (dist + rsplit.single_split.size());
    }
  }
  return completion_time;
}

std::vector<int> CwfSolver::GenerateOrderPermutation(const util::Allocator::Alloc& alloc) const {
  std::vector<std::pair<double, int>> order_durations;
  for (int o = 0; o < problem_.no(); o++) {
    order_durations.push_back({EstimateOrderDuration(alloc, o), o});
  }
  std::sort(order_durations.begin(), order_durations.end());
  std::vector<int> res;
  for (const auto& dur_ord : order_durations) {
    res.push_back(dur_ord.second);
  }
  return res; 
}

std::unique_ptr<Solution> CwfSolver::Solve() {
  // Allocate.
  util::Allocator::DistFn dist_fn = [&](int o, int w, int p) {
    double dx_o =
        problem_.warehouse(w).location().x() - problem_.order(o).location().x();
    double dy_o =
        problem_.warehouse(w).location().y() - problem_.order(o).location().y();
    double d_o = ceil(sqrt(dx_o * dx_o + dy_o * dy_o)) + 1;
    double dx_w = problem_.warehouse(w).location().x() -
                  problem_.warehouse(closest_w_.at(o)).location().x();
    double dy_w = problem_.warehouse(w).location().y() -
                  problem_.warehouse(closest_w_.at(o)).location().y();
    double d_w = ceil(sqrt(dx_w * dx_w + dy_w * dy_w)) + 1;
    double d = std::min(d_o, d_w);
    return 2.0 * d;
  };
  auto alloc = util::Allocator::AllocateWithDistFn(problem_, dist_fn);
  CHECK(util::Allocator::VerifyAlloc(problem_, alloc)) << "Invalid alloc!";
  LOG(INFO) << "Successfully allocated.";

  // Oder permutation.
  std::vector<int> order_perm = GenerateOrderPermutation(alloc);

  // Init solution.
  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  for (int d = 0; d < problem_.nd(); d++) {
    solution->add_drone_desc();
  }

////////////////////////////////////////////////////////////////////////////////

  // What is needed for the current order.
  std::map<int, int> needed;

  // Pending load (as a CompleteSplit) for each warehouses. This load has to be
  // taken either to warehouse closest_w[o] or to order o directly. 
  std::map<int, util::LoadSplitter::CompleteSplit> pending_load;

  // Drones book-keeping.
  std::vector<int> drone_busy_until(problem_.nd(), 0);
  std::vector<Location> drone_location(problem_.nd(),
                                       problem_.warehouse(0).location());

  // Handle orders one by one.
  for (int o : order_perm) {
    int ox = problem_.order(o).location().x();
    int oy = problem_.order(o).location().y();
    int cwx = problem_.warehouse(closest_w_[o]).location().x();
    int cwy = problem_.warehouse(closest_w_[o]).location().y();

    // Set needed.
    needed.clear();
    for (int p = 0; p < problem_.np(); p++) {
      if (problem_.order(o).request(p) > 0) {
        needed[p] = problem_.order(o).request(p);
      }
    }

    // Set pending_loads.
    pending_load.clear();
    {
      std::map<int, std::map<int, int>> aggregate;
      for (const auto& wp_i : alloc[o]) {
        if (wp_i.first.first == closest_w_[o]) continue;
        CHECK(wp_i.second > 0);
        aggregate[wp_i.first.first][wp_i.first.second] = wp_i.second;
      }
      for (const auto& w_stock : aggregate) {
        pending_load[w_stock.first] = util::LoadSplitter::Split(
            w_stock.second, product_weights_, problem_.m()); 
      }
    }

    // Unloads of each product in the closest_w[o].
    // [p][t] -> set of numbers of items of product p unloaded at t
    std::vector<std::multimap<int, int>> product_unloads(problem_.np());
    // Waiting line for each product in closest_w[o].
    // [p] -> {last not completely taken unload, what's left}
    std::vector<std::pair<int, int>> wait_line(problem_.np(), {-1, 0});
    for (const auto& wp_i : alloc[o]) {
      if (wp_i.first.first == closest_w_[o]) {
        product_unloads[wp_i.first.second].insert({-1, wp_i.second});
        wait_line[wp_i.first.second] = {-1, wp_i.second};
      }
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

    // This is just bringing all the items to the closest warehouse.
    bool brought_all = true;
    while (!pending_load.empty()) {
      // Find the best (w, d) match.
      int best_w = -1;
      int best_d = -1;
      int best_t = std::numeric_limits<int>::max();  // Earliest.
      bool best_unload = true;
      bool match_found = false;
      for (const auto& w_split: pending_load) {
        int w = w_split.first;
        const auto& rsplit = w_split.second.repeated_splits.front();
        for (int d = 0; d < problem_.nd(); d++) {
          int t = drone_busy_until[d];
          // LOAD dutation = dist to warehouse + 1
          {
            double dx =
                drone_location[d].x() - problem_.warehouse(w).location().x();
            double dy =
                drone_location[d].y() - problem_.warehouse(w).location().y();
            int dist = ceil(sqrt(dx * dx + dy * dy));
            t += dist + rsplit.single_split.size();
          }
          // UNLOAD vs LOAD duration...
          bool unload = false;
          {
            double dox = ox - problem_.warehouse(w).location().x();
            double doy = oy - problem_.warehouse(w).location().y();
            double dcwx = cwx - problem_.warehouse(w).location().x();
            double dcwy = cwy - problem_.warehouse(w).location().y();
            int dist_o = ceil(sqrt(dox * dox + doy * doy));
            int dist_cw = ceil(sqrt(dcwx * dcwx + dcwy * dcwy));
            if (dist_o < dist_cw) {
              t += dist_o + rsplit.single_split.size();
              unload = false;
            } else {
              t += dist_cw + rsplit.single_split.size();
              unload = true;
            }
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
            best_unload = unload;
            match_found = true;
          }
        }
      }

      if (!match_found) {
        brought_all = false;
        break;
      }

      // Great, we have the match, let's see what we're taking.
      auto taking = pending_load[best_w].repeated_splits.front().single_split;

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

      // Let's generate the UNLOAD/LOAD commands.
      if (best_unload) {
        double dx = cwx - problem_.warehouse(best_w).location().x();
        double dy = cwy - problem_.warehouse(best_w).location().y();
        int dist = ceil(sqrt(dx * dx + dy * dy));
        for (const auto& pi : taking) {
          if (pi.second < 1) continue;
          num_drone_commands[best_d]++;
          auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
          cmd->set_drone(best_d);
          cmd->set_type(DroneCommand_CommandType_UNLOAD);
          cmd->set_warehouse(closest_w_[o]);
          cmd->set_product(pi.first);
          cmd->set_num_items(pi.second);
          cmd->set_start_time(drone_busy_until[best_d] + 1);
          drone_busy_until[best_d] += dist + 1;
          drone_location[best_d] = problem_.warehouse(closest_w_[o]).location();
          product_unloads[pi.first].insert(
              {drone_busy_until[best_d], pi.second});
          if (drone_busy_until[best_d] > problem_.t()) {
            LOG(INFO) << "    Exceeded!";
            brought_all = false;
            break;
          }
          dist = 0;
        }
      } else {
        double dx = ox - problem_.warehouse(best_w).location().x();
        double dy = oy - problem_.warehouse(best_w).location().y();
        int dist = ceil(sqrt(dx * dx + dy * dy));
        for (const auto& pi : taking) {
          if (pi.second < 1) continue;
          num_drone_commands[best_d]++;
          auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
          cmd->set_drone(best_d);
          cmd->set_type(DroneCommand_CommandType_DELIVER);
          cmd->set_order(o);
          cmd->set_product(pi.first);
          cmd->set_num_items(pi.second);
          cmd->set_start_time(drone_busy_until[best_d] + 1);
          drone_busy_until[best_d] += dist + 1;
          drone_location[best_d] = problem_.order(o).location();
          needed[pi.first] -= pi.second;
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
      //
      // Remove taken from pending_load.
      auto& rsplit = pending_load[best_w].repeated_splits.front();
      pending_load[best_w].total_num_items -= rsplit.num_items;
      if (--rsplit.times == 0) {
        // Warning: rsplit doesn't exist anymore!
        pending_load[best_w].repeated_splits.pop_front();
      }
      if (--pending_load[best_w].total_times == 0) {
        pending_load.erase(best_w);
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
        util::LoadSplitter::Split(needed, product_weights_, problem_.m());
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
          cmd->set_warehouse(closest_w_[o]);
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
