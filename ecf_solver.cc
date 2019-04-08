#include "ecf_solver.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <numeric>
#include <queue>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "glog/logging.h"
#include "lp_util.h"
#include "ortools/linear_solver/linear_solver.h"
#include "ortools/linear_solver/linear_solver.pb.h"
#include "sp4_solver.pb.h"

namespace drones {

using lin_prog::Polynomial;
using lin_prog::SavyProtoHash;
using operations_research::MPSolver;
using operations_research::MPVariable;
using sp4_solver::VariableDesc;

std::unique_ptr<Solution> EcfSolver::Solve() {
  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  for (int d = 0; d < problem_.nd(); d++) {
    solution->add_drone_desc();
  }

  auto alloc = Allocate();
  if (!VerifyAlloc(alloc)) {
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
      if (wp_items.second <= 0) continue;
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
      for (const auto& w_pi: pending_warehouses) {
        int total_wh = 0;
        for (const auto& pi: w_pi.second) {
          CHECK(pi.second > 0);
          total_wh += pi.second;
        }
        CHECK(total_wh > 0);
        res += total_wh;
      }
      return res;
    };
    
    int total_pending_items = calc_total_pending_items() + 1; // 1 to make sure the check at the loop beginning is valid in the first pass.
    CHECK(total_pending_items > 1);
    LOG(INFO) << "  Initial pending items: " << total_pending_items - 1;
    int num_warehouse = 0;

    while (!pending_warehouses.empty()) {
      int curr_total_pending_items = calc_total_pending_items();
      CHECK(curr_total_pending_items > 0) << curr_total_pending_items << " > " << 0;
      CHECK(curr_total_pending_items < total_pending_items) << curr_total_pending_items << " < " << total_pending_items; 
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
          num_drone_commands[best_d]++;
          auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
          cmd->set_drone_id(best_d);
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
          num_drone_commands[best_d]++;
          auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
          cmd->set_drone_id(best_d);
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

bool EcfSolver::CanSolve(const ProblemType& problem_type) const {
  return true;
}

EcfSolver::Alloc EcfSolver::Allocate() const {
  // Create solver.
  MPSolver solver("delivery", MPSolver::GLOP_LINEAR_PROGRAMMING);
  const double inf = solver.infinity();

  // Create all decision variables and the objective.
  LOG(INFO) << "Allocate(): Generating vars and the objective.";
  std::unordered_map<std::string, MPVariable*> vars;
  auto* objective = solver.MutableObjective();
  {
    VariableDesc var_desc;
    for (int order = 0; order < problem_.no(); order++) {
      var_desc.set_order(order);
      for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
        var_desc.set_warehouse(warehouse);
        int dx = problem_.warehouse(warehouse).location().x() -
                 problem_.order(order).location().x();
        int dy = problem_.warehouse(warehouse).location().y() -
                 problem_.order(order).location().y();
        double d = ceil(sqrt(dx * dx + dy * dy)) + 1;

        for (int product = 0; product < problem_.np(); product++) {
          if (problem_.order(order).request(product) == 0) continue;
          if (problem_.warehouse(warehouse).stock(product) == 0) continue;
          var_desc.set_product(product);

          const auto& var_hash = SavyProtoHash(var_desc);
          vars[var_hash] = solver.MakeNumVar(
              0, problem_.order(order).request(product), var_hash);
          objective->SetCoefficient(vars[var_hash], 2 * d);
        }
      }
    }
    objective->SetMinimization();
  }

  // Generate constraints.
  // (1) Can't take more from a warehouse then there's in it.
  LOG(INFO) << "Allocate(): Generating warehouse per-product constraints.";
  {
    VariableDesc var_desc;
    for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
      var_desc.set_warehouse(warehouse);
      for (int product = 0; product < problem_.np(); product++) {
        if (problem_.warehouse(warehouse).stock(product) == 0) continue;
        var_desc.set_product(product);
        auto* constraint = solver.MakeRowConstraint(
            -inf, problem_.warehouse(warehouse).stock(product));
        for (int order = 0; order < problem_.no(); order++) {
          if (problem_.order(order).request(product) == 0) continue;
          var_desc.set_order(order);
          constraint->SetCoefficient(vars[SavyProtoHash(var_desc)], 1);
        }
      }
    }
  }
  // (2) Make sure to deliver enough.
  LOG(INFO) << "Allocate(): Generating order per-product constraints.";
  {
    VariableDesc var_desc;
    for (int order = 0; order < problem_.no(); order++) {
      var_desc.set_order(order);
      for (int product = 0; product < problem_.np(); product++) {
        if (problem_.order(order).request(product) == 0) continue;
        var_desc.set_product(product);
        auto* constraint = solver.MakeRowConstraint(
            problem_.order(order).request(product), inf);
        for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
          if (problem_.warehouse(warehouse).stock(product) == 0) continue;
          var_desc.set_warehouse(warehouse);
          constraint->SetCoefficient(vars[SavyProtoHash(var_desc)], 1);
        }
      }
    }
  }

  LOG(INFO) << "Allocate(): Number of variables = " << solver.NumVariables();
  LOG(INFO) << "Allocate(): Number of constraints = "
            << solver.NumConstraints();

  LOG(INFO) << "Allocate(): Starting the solver";
  auto status = solver.Solve();
  LOG(INFO) << "Allocate(): solver.Solve(): " << status;
  CHECK(status == MPSolver::ResultStatus::OPTIMAL ||
        status == MPSolver::ResultStatus::FEASIBLE);

  LOG(INFO) << "Re-packing the results.";
  // [order][{warehouse, product}] -> num_items
  EcfSolver::Alloc res(problem_.no());
  // [order][product] -> requested - given
  std::vector<std::map<int, int>> order_balance(problem_.no());
  // [warehouse][product] -> stock - taken
  std::vector<std::map<int, int>> warehouse_balance(problem_.nw());
  {
    VariableDesc var_desc;
    for (int order = 0; order < problem_.no(); order++) {
      var_desc.set_order(order);
      for (int product = 0; product < problem_.np(); product++) {
        int requested = problem_.order(order).request(product);
        if (requested == 0) continue;
        var_desc.set_product(product);
        order_balance[order][product] = requested;
        for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
          if (problem_.warehouse(warehouse).stock(product) == 0) continue;
          var_desc.set_warehouse(warehouse);
          const auto& var_hash = SavyProtoHash(var_desc);
          int warehouse_provides = round(vars[var_hash]->solution_value());
          CHECK(warehouse_provides >= 0);
          warehouse_balance[warehouse][product] -= warehouse_provides;
          order_balance[order][product] -= warehouse_provides;
          res[order][std::make_pair(warehouse, product)] = warehouse_provides;
        }
      }
    }
    for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
      for (int product = 0; product < problem_.np(); product++) {
        warehouse_balance[warehouse][product] +=
            problem_.warehouse(warehouse).stock(product);
      }
    }
  }

  vars.clear();
  solver.Clear();

  std::random_device rd;
  std::default_random_engine rand_eng(rd());

  // Fix negative order balances (when given more than needed) by returning to
  // the furthest warehouses.
  LOG(INFO) << "Fixing negative order balances.";
  {
    // Generate a random permutation of orders.
    std::vector<int> order_perm(problem_.no());
    for (int o = 0; o < problem_.no(); o++) {
      order_perm[o] = o;
    }
    for (int o = 1; o < problem_.no(); o++) {
      std::swap(order_perm[o], order_perm[rand_eng() % o]);
    }
    // Return.
    std::vector<std::pair<int, int>> dist_warehouse;
    for (int o : order_perm) {
      for (int p = 0; p < problem_.np(); p++) {
        if (order_balance[o][p] >= 0) continue;
        dist_warehouse.clear();
        for (int w = 0; w < problem_.nw(); w++) {
          int giving = res[o][std::make_pair(w, p)];
          if (giving > 0) {
            int dx = problem_.warehouse(w).location().x() -
                     problem_.order(o).location().x();
            int dy = problem_.warehouse(w).location().y() -
                     problem_.order(o).location().y();
            int d = ceil(sqrt(dx * dx + dy * dy));
            dist_warehouse.push_back({d, w});
          }
        }
        CHECK(!dist_warehouse.empty());
        std::make_heap(dist_warehouse.begin(), dist_warehouse.end());
        while (order_balance[o][p] < 0) {
          CHECK(!dist_warehouse.empty());
          std::pop_heap(dist_warehouse.begin(), dist_warehouse.end());
          int w = dist_warehouse.back().second;
          dist_warehouse.pop_back();
          int give_back =
              std::min(res[o][std::make_pair(w, p)], -order_balance[o][p]);
          res[o][std::make_pair(w, p)] -= give_back;
          warehouse_balance[w][p] += give_back;
          order_balance[o][p] += give_back;
          CHECK((res[o][std::make_pair(w, p)] >= 0));
        }
      }
    }
  }

  // Fix negative warehouse balances (when taken more than possible) by taking
  // back from the furthest order.
  LOG(INFO) << "Fixing negative warehouse balances.";
  {
    // Generate a random permutation of warehouses.
    std::vector<int> warehouse_perm(problem_.nw());
    for (int w = 0; w < problem_.nw(); w++) {
      warehouse_perm[w] = w;
    }
    for (int w = 1; w < problem_.nw(); w++) {
      std::swap(warehouse_perm[w], warehouse_perm[rand_eng() % w]);
    }
    // Reclaim.
    std::vector<std::pair<int, int>> dist_order;
    for (int w : warehouse_perm) {
      for (int p = 0; p < problem_.np(); p++) {
        if (warehouse_balance[w][p] >= 0) continue;
        dist_order.clear();
        for (int o = 0; o < problem_.no(); o++) {
          int giving = res[o][std::make_pair(w, p)];
          if (giving > 0) {
            int dx = problem_.warehouse(w).location().x() -
                     problem_.order(o).location().x();
            int dy = problem_.warehouse(w).location().y() -
                     problem_.order(o).location().y();
            int d = ceil(sqrt(dx * dx + dy * dy));
            dist_order.push_back({d, o});
          }
        }
        std::make_heap(dist_order.begin(), dist_order.end());
        while (warehouse_balance[w][p] < 0) {
          std::pop_heap(dist_order.begin(), dist_order.end());
          int o = dist_order.back().second;
          dist_order.pop_back();
          int take_back =
              std::min(res[o][std::make_pair(w, p)], -warehouse_balance[w][p]);
          res[o][std::make_pair(w, p)] -= take_back;
          CHECK((res[o][std::make_pair(w, p)] >= 0));
          warehouse_balance[w][p] += take_back;
          order_balance[o][p] += take_back;
        }
      }
    }
  }

  // Fix positive order balances (when given less than needed) by taking from
  // the closest warehouse.
  LOG(INFO) << "Fixing positive order balances.";
  {
    // Generate a random permutation of orders.
    std::vector<int> order_perm(problem_.no());
    for (int o = 0; o < problem_.no(); o++) {
      order_perm[o] = o;
    }
    for (int o = 1; o < problem_.no(); o++) {
      std::swap(order_perm[o], order_perm[rand_eng() % o]);
    }

    // Take.
    auto gt = std::greater<std::pair<int, int>>();
    std::vector<std::pair<int, int>> dist_warehouse;
    for (int o : order_perm) {
      for (int p = 0; p < problem_.np(); p++) {
        if (order_balance[o][p] <= 0) continue;
        dist_warehouse.clear();
        for (int w = 0; w < problem_.nw(); w++) {
          if (warehouse_balance[w][p] <= 0) continue;
          int dx = problem_.warehouse(w).location().x() -
                   problem_.order(o).location().x();
          int dy = problem_.warehouse(w).location().y() -
                   problem_.order(o).location().y();
          int d = ceil(sqrt(dx * dx + dy * dy));
          dist_warehouse.push_back({d, w});
        }
        std::make_heap(dist_warehouse.begin(), dist_warehouse.end(), gt);
        while (order_balance[o][p] > 0) {
          std::pop_heap(dist_warehouse.begin(), dist_warehouse.end(), gt);
          int w = dist_warehouse.back().second;
          dist_warehouse.pop_back();
          int take = std::min(warehouse_balance[w][p], order_balance[o][p]);
          res[o][std::make_pair(w, p)] += take;
          CHECK((res[o][std::make_pair(w, p)] >= 0));
          warehouse_balance[w][p] -= take;
          order_balance[o][p] -= take;
        }
      }
    }
  }
  return res;
}

bool EcfSolver::VerifyAlloc(const EcfSolver::Alloc& alloc) const {
  // [order][product] -> requested - given
  std::vector<std::map<int, int>> order_balance(problem_.no());
  // [warehouse][product] -> stock - taken
  std::vector<std::map<int, int>> warehouse_balance(problem_.nw());

  for (int o = 0; o < problem_.no(); o++) {
    for (int p = 0; p < problem_.np(); p++) {
      int requested = problem_.order(o).request(p);
      if (requested == 0) continue;
      order_balance[o][p] = requested;
      for (int w = 0; w < problem_.nw(); w++) {
        if (problem_.warehouse(w).stock(p) == 0) continue;
        if (alloc[o].count(std::make_pair(w, p)) == 0) continue;
        int warehouse_provides = alloc[o].at(std::make_pair(w, p));
        CHECK(warehouse_provides >= 0);
        warehouse_balance[w][p] -= warehouse_provides;
        order_balance[o][p] -= warehouse_provides;
      }
      if (order_balance[o][p] != 0) {
        LOG(ERROR) << "Non-zero order balance found!";
        return false;
      }
    }
  }

  for (int w = 0; w < problem_.nw(); w++) {
    for (int p = 0; p < problem_.np(); p++) {
      warehouse_balance[w][p] += problem_.warehouse(w).stock(p);
      if (warehouse_balance[w][p] < 0) {
        LOG(ERROR) << "Negative warehouse balance found!";
        return false;
      }
    }
  }

  return true;
}

}  // namespace drones
