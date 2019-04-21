#include "solvers/util/allocator.h"

#include <map>
#include <random>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "ortools/linear_solver/linear_solver.h"
#include "ortools/linear_solver/linear_solver.pb.h"
#include "solvers/util/alloc.pb.h"
#include "solvers/util/lp_util.h"

namespace drones {
namespace util {

using lp::Polynomial;
using lp::SavyProtoHash;
using operations_research::MPSolver;
using operations_research::MPVariable;

Allocator::Alloc Allocator::AllocateWithDistFn(const Problem& problem,
                                               const DistFn& dist_fn) {
  // Create solver.
  MPSolver solver("delivery", MPSolver::GLOP_LINEAR_PROGRAMMING);
  const double inf = solver.infinity();

  // Create all decision variables and the objective.
  LOG(INFO) << "Allocate(): Generating vars and the objective.";
  std::unordered_map<std::string, MPVariable*> vars;
  auto* objective = solver.MutableObjective();
  {
    alloc::VariableDesc var_desc;
    for (int order = 0; order < problem.no(); order++) {
      var_desc.set_order(order);
      for (int warehouse = 0; warehouse < problem.nw(); warehouse++) {
        var_desc.set_warehouse(warehouse);
        for (int product = 0; product < problem.np(); product++) {
          if (problem.order(order).request(product) == 0) continue;
          if (problem.warehouse(warehouse).stock(product) == 0) continue;
          var_desc.set_product(product);
          const auto& var_hash = SavyProtoHash(var_desc);
          vars[var_hash] = solver.MakeNumVar(
              0, problem.order(order).request(product), var_hash);
          double coef = dist_fn(order, warehouse, product);
          objective->SetCoefficient(vars[var_hash], coef);
        }
      }
    }
    objective->SetMinimization();
  }

  // Generate constraints.
  // (1) Can't take more from a warehouse then there's in it.
  LOG(INFO) << "Allocate(): Generating warehouse per-product constraints.";
  {
    alloc::VariableDesc var_desc;
    for (int warehouse = 0; warehouse < problem.nw(); warehouse++) {
      var_desc.set_warehouse(warehouse);
      for (int product = 0; product < problem.np(); product++) {
        if (problem.warehouse(warehouse).stock(product) == 0) continue;
        var_desc.set_product(product);
        auto* constraint = solver.MakeRowConstraint(
            -inf, problem.warehouse(warehouse).stock(product));
        for (int order = 0; order < problem.no(); order++) {
          if (problem.order(order).request(product) == 0) continue;
          var_desc.set_order(order);
          constraint->SetCoefficient(vars[SavyProtoHash(var_desc)], 1);
        }
      }
    }
  }
  // (2) Make sure to deliver enough.
  LOG(INFO) << "Allocate(): Generating order per-product constraints.";
  {
    alloc::VariableDesc var_desc;
    for (int order = 0; order < problem.no(); order++) {
      var_desc.set_order(order);
      for (int product = 0; product < problem.np(); product++) {
        if (problem.order(order).request(product) == 0) continue;
        var_desc.set_product(product);
        auto* constraint = solver.MakeRowConstraint(
            problem.order(order).request(product), inf);
        for (int warehouse = 0; warehouse < problem.nw(); warehouse++) {
          if (problem.warehouse(warehouse).stock(product) == 0) continue;
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
  Alloc res(problem.no());
  // [order][product] -> requested - given
  std::vector<std::map<int, int>> order_balance(problem.no());
  // [warehouse][product] -> stock - taken
  std::vector<std::map<int, int>> warehouse_balance(problem.nw());
  {
    alloc::VariableDesc var_desc;
    for (int order = 0; order < problem.no(); order++) {
      var_desc.set_order(order);
      for (int product = 0; product < problem.np(); product++) {
        int requested = problem.order(order).request(product);
        if (requested == 0) continue;
        var_desc.set_product(product);
        order_balance[order][product] = requested;
        for (int warehouse = 0; warehouse < problem.nw(); warehouse++) {
          if (problem.warehouse(warehouse).stock(product) == 0) continue;
          var_desc.set_warehouse(warehouse);
          const auto& var_hash = SavyProtoHash(var_desc);
          int warehouse_provides = round(vars[var_hash]->solution_value());
          CHECK(warehouse_provides >= 0);
          warehouse_balance[warehouse][product] -= warehouse_provides;
          order_balance[order][product] -= warehouse_provides;
          if (warehouse_provides == 0) continue;
          res[order][std::make_pair(warehouse, product)] = warehouse_provides;
        }
      }
    }
    for (int warehouse = 0; warehouse < problem.nw(); warehouse++) {
      for (int product = 0; product < problem.np(); product++) {
        warehouse_balance[warehouse][product] +=
            problem.warehouse(warehouse).stock(product);
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
    std::vector<int> order_perm(problem.no());
    for (int o = 0; o < problem.no(); o++) {
      order_perm[o] = o;
    }
    for (int o = 1; o < problem.no(); o++) {
      std::swap(order_perm[o], order_perm[rand_eng() % o]);
    }
    // Return.
    std::vector<std::pair<double, int>> dist_warehouse;
    for (int o : order_perm) {
      for (int p = 0; p < problem.np(); p++) {
        if (order_balance[o][p] >= 0) continue;
        dist_warehouse.clear();
        for (int w = 0; w < problem.nw(); w++) {
          int giving = res[o][std::make_pair(w, p)];
          if (giving > 0) {
            double d = dist_fn(o, w, p);
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
          auto wp = std::make_pair(w, p);
          CHECK(res[o][wp] >= give_back);
          if ((res[o][wp] -= give_back) == 0) res[o].erase(wp);
          warehouse_balance[w][p] += give_back;
          order_balance[o][p] += give_back;
        }
      }
    }
  }

  // Fix negative warehouse balances (when taken more than possible) by taking
  // back from the furthest order.
  LOG(INFO) << "Fixing negative warehouse balances.";
  {
    // Generate a random permutation of warehouses.
    std::vector<int> warehouse_perm(problem.nw());
    for (int w = 0; w < problem.nw(); w++) {
      warehouse_perm[w] = w;
    }
    for (int w = 1; w < problem.nw(); w++) {
      std::swap(warehouse_perm[w], warehouse_perm[rand_eng() % w]);
    }
    // Reclaim.
    std::vector<std::pair<double, int>> dist_order;
    for (int w : warehouse_perm) {
      for (int p = 0; p < problem.np(); p++) {
        if (warehouse_balance[w][p] >= 0) continue;
        dist_order.clear();
        for (int o = 0; o < problem.no(); o++) {
          int giving = res[o][std::make_pair(w, p)];
          if (giving > 0) {
            double d = dist_fn(o, w, p);
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
          auto wp = std::make_pair(w, p);
          CHECK(res[o][wp] >= take_back);
          if ((res[o][wp] -= take_back) == 0) res[o].erase(wp);
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
    std::vector<int> order_perm(problem.no());
    for (int o = 0; o < problem.no(); o++) {
      order_perm[o] = o;
    }
    for (int o = 1; o < problem.no(); o++) {
      std::swap(order_perm[o], order_perm[rand_eng() % o]);
    }

    // Take.
    auto gt = std::greater<std::pair<double, int>>();
    std::vector<std::pair<double, int>> dist_warehouse;
    for (int o : order_perm) {
      for (int p = 0; p < problem.np(); p++) {
        if (order_balance[o][p] <= 0) continue;
        dist_warehouse.clear();
        for (int w = 0; w < problem.nw(); w++) {
          if (warehouse_balance[w][p] <= 0) continue;
          double d = dist_fn(o, w, p);
          dist_warehouse.push_back({d, w});
        }
        std::make_heap(dist_warehouse.begin(), dist_warehouse.end(), gt);
        while (order_balance[o][p] > 0) {
          std::pop_heap(dist_warehouse.begin(), dist_warehouse.end(), gt);
          int w = dist_warehouse.back().second;
          dist_warehouse.pop_back();
          int take = std::min(warehouse_balance[w][p], order_balance[o][p]);
          if (take == 0) continue;
          auto wp = std::make_pair(w, p);
          res[o][wp] += take;
          CHECK(res[o][wp] >= 0);
          warehouse_balance[w][p] -= take;
          order_balance[o][p] -= take;
        }
      }
    }
  }
  return res;
}

bool Allocator::VerifyAlloc(const Problem& problem,
                            const Allocator::Alloc& alloc) {
  // [order][product] -> requested - given
  std::vector<std::map<int, int>> order_balance(problem.no());
  // [warehouse][product] -> stock - taken
  std::vector<std::map<int, int>> warehouse_balance(problem.nw());

  for (int o = 0; o < problem.no(); o++) {
    for (int p = 0; p < problem.np(); p++) {
      int requested = problem.order(o).request(p);
      if (requested == 0) continue;
      order_balance[o][p] = requested;
      for (int w = 0; w < problem.nw(); w++) {
        if (problem.warehouse(w).stock(p) == 0) continue;
        if (alloc[o].count(std::make_pair(w, p)) == 0) continue;
        int warehouse_provides = alloc[o].at(std::make_pair(w, p));
        CHECK(warehouse_provides > 0);
        warehouse_balance[w][p] -= warehouse_provides;
        order_balance[o][p] -= warehouse_provides;
      }
      if (order_balance[o][p] != 0) {
        LOG(ERROR) << "Non-zero order balance found!";
        return false;
      }
    }
  }

  for (int w = 0; w < problem.nw(); w++) {
    for (int p = 0; p < problem.np(); p++) {
      warehouse_balance[w][p] += problem.warehouse(w).stock(p);
      if (warehouse_balance[w][p] < 0) {
        LOG(ERROR) << "Negative warehouse balance found!";
        return false;
      }
    }
  }

  return true;
}

}  // namespace util
}  // namespace drones
