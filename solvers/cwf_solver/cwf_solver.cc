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

DEFINE_bool(cwf_alloc_use_cw_o, false,
            "Wheter or not to consider distance from the closest warehouse to "
            "the order when comparing with the case when the order location "
            "itself is closer.");
DEFINE_int32(cwf_alloc_iter, 1, "Number of iterations in IterativeAlloc.");
DEFINE_int32(cwf_perm_iter, 100,
             "Number of order permutation improvement iterations.");

namespace drones {

CwfSolver::CwfSolver(const Problem& problem)
    : ProblemSolver(problem),
      closest_w_(problem.no()),
      product_weights_(),
      alloc_(),
      order_feedback_() {
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

double CwfSolver::EstimateOrderDuration(int o) const {
  int ox = problem_.order(o).location().x();
  int oy = problem_.order(o).location().y();
  int cwx = problem_.warehouse(closest_w_[o]).location().x();
  int cwy = problem_.warehouse(closest_w_[o]).location().y();

  // Let's make things easier.
  // [w][p] -> num items
  std::map<int, std::map<int, int>> wpn;
  for (const auto& wp_i : alloc_[o]) {
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
      order_feedback_[o].w_o[w] = {w_split.second.total_num_items,
                                   w_split.second.total_times};
      CHECK(w_split.second.total_num_items > 0);
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
      order_feedback_[o].w_cw[w] = {w_split.second.total_num_items,
                                    w_split.second.total_times};
      CHECK(w_split.second.total_num_items > 0);
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
  if (from_closest.empty()) {
    order_feedback_[o].cw_o = {1, 1};
  } else {
    auto split =
        util::LoadSplitter::Split(from_closest, product_weights_, problem_.m());
    order_feedback_[o].cw_o = {split.total_num_items, split.total_times};
    CHECK(split.total_num_items > 0);
    double dx = ox - cwx;
    double dy = oy - cwy;
    int dist = ceil(sqrt(dx * dx + dy * dy));
    for (const auto& rsplit : split.repeated_splits) {
      // TODO(viktors): Approx as above.
      completion_time +=
          2.0 * rsplit.times * (dist + rsplit.single_split.size());
    }
  }

  // Fill the rest of the feedbacks with averages (first time only).
  if (std::min(order_feedback_[o].w_o.size(), order_feedback_[o].w_cw.size()) <
      problem_.nw()) {
    int items_w_o = 0;
    int turns_w_o = 0;
    for (const auto& w_it : order_feedback_[o].w_o) {
      items_w_o += w_it.second.first;
      turns_w_o += w_it.second.second;
    }
    // If w_o is empty, then all warehouses ship to the closest warehouse first.
    if (items_w_o == 0) {
      items_w_o = 1.0;
      turns_w_o = 1.0;
    }

    int items_w_cw = 0;
    int turns_w_cw = 0;
    for (const auto& w_it : order_feedback_[o].w_cw) {
      items_w_cw += w_it.second.first;
      turns_w_cw += w_it.second.second;
    }
    // If w_o is empty, then all warehouses ship to the closest warehouse first.
    if (items_w_cw == 0) {
      items_w_cw = 1.0;
      turns_w_cw = 1.0;
    }

    for (int w = 0; w < problem_.nw(); w++) {
      if (order_feedback_[o].w_o.count(w) == 0) {
        order_feedback_[o].w_o[w] = {items_w_o, turns_w_o};
      }
      if (order_feedback_[o].w_cw.count(w) == 0) {
        order_feedback_[o].w_cw[w] = {items_w_cw, turns_w_cw};
      }
    }
  }

  return completion_time;
}

std::vector<int> CwfSolver::GenerateOrderPermutation() const {
  std::vector<std::pair<double, int>> order_durations;
  for (int o = 0; o < problem_.no(); o++) {
    order_durations.push_back({EstimateOrderDuration(o), o});
  }
  std::sort(order_durations.begin(), order_durations.end());
  std::vector<int> res;
  for (const auto& dur_ord : order_durations) {
    res.push_back(dur_ord.second);
  }
  return res;
}

void CwfSolver::IterativeAlloc(int num_iter) {
  CHECK(num_iter > 0);

  util::Allocator::DistFn dist_fn = [&](int o, int w, int p) {
    double c_o = 1.0;
    double dx_o =
        problem_.warehouse(w).location().x() - problem_.order(o).location().x();
    double dy_o =
        problem_.warehouse(w).location().y() - problem_.order(o).location().y();
    double d_o = ceil(sqrt(dx_o * dx_o + dy_o * dy_o)) + 1;

    double c_w = 1.0;
    double dx_w = problem_.warehouse(w).location().x() -
                  problem_.warehouse(closest_w_.at(o)).location().x();
    double dy_w = problem_.warehouse(w).location().y() -
                  problem_.warehouse(closest_w_.at(o)).location().y();
    double d_w = ceil(sqrt(dx_w * dx_w + dy_w * dy_w)) + 1;

    double c_cwo = 1.0;
    double dx_cwo = problem_.warehouse(closest_w_.at(o)).location().x() -
                    problem_.order(o).location().x();
    double dy_cwo = problem_.warehouse(closest_w_.at(o)).location().y() -
                    problem_.order(o).location().y();
    double d_cwo = ceil(sqrt(dx_cwo * dx_cwo + dy_cwo * dy_cwo));

    if (!order_feedback_.empty()) {
      CHECK(order_feedback_[o].w_o[w].first != 0);
      CHECK(order_feedback_[o].w_cw[w].first != 0);
      CHECK(order_feedback_[o].cw_o.first != 0);
      c_o = static_cast<double>(order_feedback_[o].w_o[w].second) /
            order_feedback_[o].w_o[w].first;
      c_w = static_cast<double>(order_feedback_[o].w_cw[w].second) /
            order_feedback_[o].w_cw[w].first;
      c_cwo = static_cast<double>(order_feedback_[o].cw_o.second) /
              order_feedback_[o].cw_o.first;
    }
    double d = (w = closest_w_.at(o))
                   ? d_cwo * c_cwo
                   : std::min(d_o * c_o, d_w * c_w + d_cwo * c_cwo);
    return FLAGS_cwf_alloc_use_cw_o ? d : std::min(d_o * c_o, d_w * c_w);
  };

  for (int iter = 1; iter <= num_iter; iter++) {
    LOG_EVERY_N(INFO, num_iter / 10) << absl::Substitute(
        "<><><><><><> IterativeAlloc:: Iter $0 / $1\n", iter, num_iter);
    alloc_ = util::Allocator::AllocateWithDistFn(problem_, dist_fn);
    CHECK(util::Allocator::VerifyAlloc(problem_, alloc_)) << "Invalid alloc!";
    GenerateOrderPermutation();
  }
  LOG(INFO) << "Successfully allocated.";
}

std::unique_ptr<Solution> CwfSolver::PackSolution(
    const std::vector<int>& order_permutation, int* score,
    bool enable_log) const {
  // Check if the permutation is valid.
  {
    std::vector<int> dummy;
    for (int o = 0; o < problem_.no(); o++) dummy.push_back(o);
    auto dummy1 = order_permutation;
    std::sort(dummy1.begin(), dummy1.end());
    CHECK(dummy == dummy1);
  }

  // Init solution.
  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  for (int d = 0; d < problem_.nd(); d++) {
    solution->add_drone_desc();
  }

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
  int res_score = 0;
  for (int o : order_permutation) {
    int ox = problem_.order(o).location().x();
    int oy = problem_.order(o).location().y();
    int cwx = problem_.warehouse(closest_w_[o]).location().x();
    int cwy = problem_.warehouse(closest_w_[o]).location().y();

    int max_deliver_finish = -1;

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
      for (const auto& wp_i : alloc_[o]) {
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
    for (const auto& wp_i : alloc_[o]) {
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
      LOG_IF(WARNING, enable_log) << "Rolling back.";
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
      for (const auto& w_split : pending_load) {
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
            LOG_IF(WARNING, enable_log) << "    Exceeded!";
            brought_all = false;
            break;
          }
          dist = 0;
        }
      }

      if (!brought_all) break;

      // Let's generate the UNLOAD/DELIVER commands.
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
            LOG_IF(WARNING, enable_log) << "    Exceeded!";
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
          max_deliver_finish =
              std::max(max_deliver_finish, drone_busy_until[best_d]);
          drone_location[best_d] = problem_.order(o).location();
          needed[pi.first] -= pi.second;
          if (drone_busy_until[best_d] > problem_.t()) {
            LOG_IF(WARNING, enable_log) << "    Exceeded!";
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
      CHECK(curr_split.num_items > 0);

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
            auto it = product_unloads[p].upper_bound(last_delivery);
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
          CHECK(leftover >= n);
          leftover -= n;
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
        LOG_IF(WARNING, enable_log) << "    Exceeded";
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
          max_deliver_finish =
              std::max(max_deliver_finish, drone_busy_until[best_d]);
          if (drone_busy_until[best_d] > problem_.t()) {
            LOG_IF(WARNING, enable_log) << "    Exceeded!";
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
    res_score +=
        ceil(100.0 * (problem_.t() - max_deliver_finish + 1) / problem_.t());
  }

  if (score != nullptr) *score = res_score;
  return solution;
}

std::vector<int> CwfSolver::ImporveOrderPermutation(
    std::vector<int> order_permutation, int num_iter) const {
  if (num_iter == 0) return order_permutation;
  auto eval = [&](const std::vector<int>& perm) {
    int score = -1;
    auto solution = PackSolution(perm, &score, false);
    if (solution == nullptr) {
      score = -1;
    }
    return static_cast<double>(score);
  };

  std::random_device rand_dev;
  std::default_random_engine rand_eng(rand_dev());
  std::uniform_int_distribution<int> rand_idx(0, order_permutation.size() - 1);
  std::uniform_real_distribution<double> rand_prob(0.0, 1.0);

  auto curr = order_permutation;
  double curr_val = eval(curr);
  auto best = curr;
  double best_val = curr_val;
  double init_val = curr_val;
  double T = eval(curr) / 100.0;
  double cooling_rate = pow(0.001 / T, 1.0 / num_iter);
  for (int iter = 0; iter < num_iter; iter++) {
    LOG_EVERY_N(INFO, num_iter / 10) << absl::Substitute(
        "ImporveOrderPermutation(): Iter $0 / $1", iter, num_iter);

    auto neighbour = curr;
    int idx1 = rand_idx(rand_eng);
    int idx2 = rand_idx(rand_eng);
    std::swap(neighbour[idx1], neighbour[idx2]);
    double neighbour_val = eval(neighbour);

    double dval = curr_val - neighbour_val;
    double p = (dval == 0.0 ? 0.5 : exp(dval / T));
    if (dval < 0 || p >= rand_prob(rand_eng)) {
      curr = neighbour;
      curr_val = neighbour_val;
      if (curr_val > best_val) {
        best = curr;
        best_val = curr_val;
      }
    }
    T = T * (1 - cooling_rate);
  }
  LOG(INFO) << absl::Substitute("ImporveOrderPermutation(): Result: $0 -> $1",
                                init_val, best_val);
  return best;
}

std::unique_ptr<Solution> CwfSolver::Solve() {
  IterativeAlloc(FLAGS_cwf_alloc_iter);
  std::vector<int> order_perm = GenerateOrderPermutation();
  order_perm = ImporveOrderPermutation(order_perm, FLAGS_cwf_perm_iter);
  return PackSolution(order_perm);
}

}  // namespace drones
