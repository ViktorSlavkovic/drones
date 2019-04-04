#include "sp4_solver.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <numeric>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/strings/substitute.h"
#include "absl/strings/str_join.h"
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

std::unique_ptr<Solution> Sp4Solver::Solve() {
  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  auto* drone_commands = solution->add_drone_desc();

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
      cmd->set_drone_id(0);
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
      cmd->set_drone_id(0);
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
          cmd->set_drone_id(0);
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
          cmd->set_drone_id(0);
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

Sp4Solver::Alloc Sp4Solver::Allocate() const {
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
  Sp4Solver::Alloc res(problem_.no());
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
          int give_back = std::min(res[o][std::make_pair(w, p)], -order_balance[o][p]);
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
          int take_back = std::min(res[o][std::make_pair(w, p)], -warehouse_balance[w][p]);
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

bool Sp4Solver::VerifyAlloc(const Sp4Solver::Alloc& alloc) const {
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
