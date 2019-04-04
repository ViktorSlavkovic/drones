#include "lp_solver.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <vector>

#include "absl/strings/str_join.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "lp_util.h"
#include "ortools/linear_solver/linear_solver.h"
#include "ortools/linear_solver/linear_solver.pb.h"
#include "problem.pb.h"

namespace drones {

static const std::vector<double> optimized_coefficients = {
    4.3936084,    99.83762324,  99.66419049,  84.09260497,  30.87549891,
    69.98195542,  29.95107788,  62.12556897,  24.07614345,  54.63386056,
    22.05068694,  -12.64775356, 95.18309029,  9.89912601,   43.31039049,
    69.86810173,  18.64210718,  15.57150025,  48.03709472,  52.26252223,
    56.42863973,  -39.42152674, 42.55551533,  84.03224385,  52.4222597,
    10.10345502,  -34.57383747, 85.08935195,  17.06569466,  35.65961724,
    34.34132741,  67.88252918,  4.22680695,   -1.00872254,  52.79065869,
    58.45454007,  99.43879029,  -35.74500125, -35.33057796, 64.15792966,
    -28.54123125, 5.52463473,   16.66951207,  72.03181139,  14.04742867,
    42.12119139,  -0.20021895,  0.2718654,    -40.72191587, 40.00272535};

// Does linear interpolation on the original vector of optimized coefficients
// to adjust it to the given size.
// FIXME: Fix ranges- first coefficients shouldn't match in general.
std::vector<double> InterpolateCoefficients(int size) {
  std::vector<double> res(size);
  double step = (size - 1.0) / (optimized_coefficients.size() - 1.0);
  double l = 0.0;
  int lidx = 0;
  for (int i = 0; i < size; i++) {
    if (i > l + step) {
      l += step;
      lidx++;
    }
    res[i] = optimized_coefficients[lidx] +
             (optimized_coefficients[lidx + 1] - optimized_coefficients[lidx]) *
                 ((i - l) / step);
  }
  return res;
}

LpSolver::LpSolver(const Problem& problem)
    : ProblemSolver(problem),
      aux_var_cache_(),
      simulation_time_(2 * problem.t()),
      score_coefficients_(InterpolateCoefficients(simulation_time_)) {
  CacheInitial();
}

void LpSolver::CacheInitial() {
  lp_solver::VariableDesc var;

  // Set initial DRONE_LOC
  var.set_t(0);
  var.set_type(lp_solver::VariableDesc_VariableType_DRONE_LOC);
  for (int drone = 0; drone < problem_.nd(); drone++) {
    var.set_drone(drone);
    var.set_location(0);
    aux_var_cache_[lin_prog::SavyProtoHash(var)].coef["const"] = 1;
    for (int loc = 1; loc < problem_.nw() + problem_.no(); loc++) {
      var.set_location(loc);
      aux_var_cache_[lin_prog::SavyProtoHash(var)].coef["const"] = 0;
    }
  }
  var.Clear();

  // Set initial WAREHOUSE_STATE
  var.set_t(0);
  var.set_type(lp_solver::VariableDesc_VariableType_WAREHOUSE_STATE);
  for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
    var.set_warehouse(warehouse);
    for (int product = 0; product < problem_.np(); product++) {
      var.set_product(product);
      aux_var_cache_[lin_prog::SavyProtoHash(var)].coef["const"] =
          problem_.warehouse(warehouse).stock(product);
    }
  }
  var.Clear();

  // Set initial DRONE_STATE
  var.set_t(0);
  var.set_type(lp_solver::VariableDesc_VariableType_DRONE_STATE);
  for (int drone = 0; drone < problem_.nd(); drone++) {
    var.set_drone(drone);
    for (int product = 0; product < problem_.np(); product++) {
      var.set_product(product);
      aux_var_cache_[lin_prog::SavyProtoHash(var)].coef["const"] = 0;
    }
  }
  var.Clear();

  // Set intial ORDER_STATE
  var.set_t(0);
  var.set_type(lp_solver::VariableDesc_VariableType_ORDER_STATE);
  for (int order = 0; order < problem_.nd(); order++) {
    var.set_order(order);
    for (int product = 0; product < problem_.np(); product++) {
      var.set_product(product);
      aux_var_cache_[lin_prog::SavyProtoHash(var)].coef["const"] =
          problem_.order(order).request(product);
    }
  }
  var.Clear();
}

int Distance(const Location& src, const Location& dst) {
  int dx = src.x() - dst.x();
  int dy = src.y() - dst.y();
  return ceil(sqrt(dx * dx + dy * dy));
}

LpSolver::Polynomial LpSolver::Compute(
    const drones::lp_solver::VariableDesc& var) {
  if (var.has_t()) {
    CHECK(var.t() >= 0) << "Invalid time: " << var.t();
  }

  // Check the cache.
  std::string var_hash = lin_prog::SavyProtoHash(var);
  if (aux_var_cache_.count(var_hash)) {
    return aux_var_cache_[var_hash];
  }
  // Handle different variable types.
  Polynomial res;
  drones::lp_solver::VariableDesc dep_var;
  switch (var.type()) {
    case lp_solver::VariableDesc_VariableType_TOTAL_SCORE: {
      for (int order = 0; order < problem_.no(); order++) {
        LOG(INFO) << "TOTAL_SCORE computation: order " << order << "/"
                  << problem_.no();
        dep_var.set_order(order);
        dep_var.set_type(lp_solver::VariableDesc_VariableType_ORDER_SCORE);
        res += Compute(dep_var);
      }
      break;
    }
    case lp_solver::VariableDesc_VariableType_ORDER_SCORE: {
      for (int t = 1; t <= simulation_time_; t++) {
        dep_var.set_order(var.order());
        dep_var.set_t(t);
        dep_var.set_type(
            lp_solver::VariableDesc_VariableType_ORDER_COMPLETENESS);
        auto order_completeness = Compute(dep_var);
        order_completeness *= score_coefficients_[t - 1] / simulation_time_;
        res += order_completeness;
      }
      break;
    }
    case lp_solver::VariableDesc_VariableType_ORDER_COMPLETENESS: {
      int total_order_items = 0;
      for (int product = 0; product < problem_.np(); product++) {
        total_order_items += problem_.order(var.order()).request(product);
        dep_var.set_product(product);
        dep_var.set_order(var.order());
        dep_var.set_t(var.t());
        dep_var.set_type(lp_solver::VariableDesc_VariableType_ORDER_STATE);
        res += Compute(dep_var);
      }
      res *= -1.0 / total_order_items;
      res.coef["const"] += 1;
      break;
    }
    case lp_solver::VariableDesc_VariableType_ORDER_STATE: {
      dep_var.set_order(var.order());
      dep_var.set_product(var.product());
      dep_var.set_t(var.t() - 1);
      dep_var.set_type(lp_solver::VariableDesc_VariableType_ORDER_STATE);
      res += Compute(dep_var);

      Polynomial delivery;
      dep_var.Clear();
      dep_var.set_product(var.product());
      dep_var.set_order(var.order());
      dep_var.set_type(lp_solver::VariableDesc_VariableType_DELIVER);
      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        dep_var.set_location(loc);
        auto actual_location =
            loc < problem_.nw()
                ? problem_.warehouse(loc).location()
                : problem_.order(loc - problem_.nw()).location();
        int start_time =
            var.t() -
            Distance(actual_location, problem_.order(var.order()).location()) -
            1;
        if (start_time < 1) {
          continue;
        }
        dep_var.set_t(start_time);

        for (int drone = 0; drone < problem_.nd(); drone++) {
          dep_var.set_drone(drone);

          int max_num_items =
              std::min(problem_.order(var.order()).request(var.product()),
                       problem_.m() / problem_.product(var.product()).m());
          for (int num_items = 1; num_items <= max_num_items; num_items++) {
            dep_var.set_num_items(num_items);
            delivery.coef[lin_prog::SavyProtoHash(dep_var)] = num_items;
          }
        }
      }
      delivery *= -1.0;
      res += delivery;
      break;
    }
    case lp_solver::VariableDesc_VariableType_DRONE_STATE: {
      dep_var.set_drone(var.drone());
      dep_var.set_product(var.product());
      dep_var.set_t(var.t() - 1);
      dep_var.set_type(lp_solver::VariableDesc_VariableType_DRONE_STATE);
      res += Compute(dep_var);

      Polynomial traffic;
      dep_var.Clear();
      dep_var.set_drone(var.drone());
      dep_var.set_product(var.product());

      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        dep_var.set_location(loc);
        // Handle LOAD and UNLOAD commands.
        for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
          dep_var.set_warehouse(warehouse);
          // Check time correctness.
          auto actual_location =
              loc < problem_.nw()
                  ? problem_.warehouse(loc).location()
                  : problem_.order(loc - problem_.nw()).location();
          int start_time = var.t() -
                           Distance(actual_location,
                                    problem_.warehouse(warehouse).location()) -
                           1;
          if (start_time < 1) {
            continue;
          }
          dep_var.set_t(start_time);

          int drone_cap_items =
              problem_.m() / problem_.product(var.product()).m();
          for (int num_items = 1; num_items <= drone_cap_items; num_items++) {
            dep_var.set_num_items(num_items);
            dep_var.set_type(lp_solver::VariableDesc_VariableType_LOAD);
            traffic.coef[lin_prog::SavyProtoHash(dep_var)] = num_items;
            dep_var.set_type(lp_solver::VariableDesc_VariableType_UNLOAD);
            traffic.coef[lin_prog::SavyProtoHash(dep_var)] = -num_items;
          }
        }
        dep_var.clear_warehouse();

        // Handle DELIVER commands.
        for (int order = 0; order < problem_.no(); order++) {
          dep_var.set_order(order);
          // Check time correctness.
          auto actual_location =
              loc < problem_.nw()
                  ? problem_.warehouse(loc).location()
                  : problem_.order(loc - problem_.nw()).location();
          int start_time =
              var.t() -
              Distance(actual_location, problem_.order(order).location()) - 1;
          if (start_time < 1) {
            continue;
          }
          dep_var.set_t(start_time);

          dep_var.set_type(lp_solver::VariableDesc_VariableType_DELIVER);
          int max_num_items =
              std::min(problem_.order(order).request(var.product()),
                       problem_.m() / problem_.product(var.product()).m());
          for (int num_items = 1; num_items <= max_num_items; num_items++) {
            dep_var.set_num_items(num_items);
            traffic.coef[lin_prog::SavyProtoHash(dep_var)] = -num_items;
          }
        }
      }

      res += traffic;
      break;
    }
    case lp_solver::VariableDesc_VariableType_DRONE_LOC: {
      dep_var.set_drone(var.drone());
      dep_var.set_location(var.location());
      dep_var.set_t(var.t() - 1);
      dep_var.set_type(lp_solver::VariableDesc_VariableType_WAIT);
      res.coef[lin_prog::SavyProtoHash(dep_var)] = 1;

      int dst_loc = var.location();
      Location actual_dst_loc;
      if (dst_loc < problem_.nw()) {
        int warehouse = dst_loc;
        actual_dst_loc = problem_.warehouse(warehouse).location();
        dep_var.set_warehouse(warehouse);
      } else {
        int order = dst_loc - problem_.nw();
        actual_dst_loc = problem_.order(order).location();
        dep_var.set_order(order);
      }

      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        dep_var.set_location(loc);
        // Check time correctness.
        auto actual_location =
            loc < problem_.nw()
                ? problem_.warehouse(loc).location()
                : problem_.order(loc - problem_.nw()).location();
        int start_time =
            var.t() - Distance(actual_location, actual_dst_loc) - 1;
        if (start_time < 1) {
          continue;
        }
        dep_var.set_t(start_time);

        for (int product = 0; product < problem_.np(); product++) {
          dep_var.set_product(product);
          if (dst_loc < problem_.nw()) {
            int drone_cap_items = problem_.m() / problem_.product(product).m();
            for (int num_items = 1; num_items <= drone_cap_items; num_items++) {
              dep_var.set_num_items(num_items);
              dep_var.set_type(lp_solver::VariableDesc_VariableType_UNLOAD);
              res.coef[lin_prog::SavyProtoHash(dep_var)] = 1;
              dep_var.set_type(lp_solver::VariableDesc_VariableType_LOAD);
              res.coef[lin_prog::SavyProtoHash(dep_var)] = 1;
            }
          } else {
            int max_num_items =
                std::min(problem_.order(dep_var.order()).request(product),
                         problem_.m() / problem_.product(product).m());
            for (int num_items = 1; num_items <= max_num_items; num_items++) {
              dep_var.set_type(lp_solver::VariableDesc_VariableType_DELIVER);
              res.coef[lin_prog::SavyProtoHash(dep_var)] = 1;
            }
          }
        }
      }
      break;
    }
    case lp_solver::VariableDesc_VariableType_WAREHOUSE_STATE: {
      dep_var.set_warehouse(var.warehouse());
      dep_var.set_product(var.product());
      dep_var.set_t(var.t() - 1);
      dep_var.set_type(lp_solver::VariableDesc_VariableType_WAREHOUSE_STATE);
      res += Compute(dep_var);

      Polynomial traffic;
      dep_var.Clear();
      dep_var.set_warehouse(var.warehouse());
      dep_var.set_product(var.product());

      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        dep_var.set_location(loc);
        // Check time correctness.
        auto actual_location =
            loc < problem_.nw()
                ? problem_.warehouse(loc).location()
                : problem_.order(loc - problem_.nw()).location();
        int start_time =
            var.t() -
            Distance(actual_location,
                     problem_.warehouse(var.warehouse()).location()) -
            1;
        if (start_time < 1) {
          continue;
        }
        dep_var.set_t(start_time);

        for (int drone = 0; drone < problem_.nd(); drone++) {
          dep_var.set_drone(drone);
          int drone_cap_items =
              problem_.m() / problem_.product(var.product()).m();
          for (int num_items = 1; num_items <= drone_cap_items; num_items++) {
            dep_var.set_num_items(num_items);

            dep_var.set_type(lp_solver::VariableDesc_VariableType_UNLOAD);
            traffic.coef[lin_prog::SavyProtoHash(dep_var)] = num_items;

            dep_var.set_type(lp_solver::VariableDesc_VariableType_LOAD);
            traffic.coef[lin_prog::SavyProtoHash(dep_var)] = -num_items;
          }
        }
      }
      res += traffic;
      break;
    }
    default: {
      CHECK(false) << "Not an auxiliary variable!";
    }
  }
  return aux_var_cache_[var_hash] = res;
}

std::unique_ptr<Solution> LpSolver::Solve() {
  lp_solver::VariableDesc var_desc;
  //////////////////////////////////////////////////////////////////
  /*
    var_desc.set_order(0);
    var_desc.set_product(0);
    var_desc.set_t(50);
    var_desc.set_type(lp_solver::VariableDesc_VariableType_ORDER_STATE);
    auto test = Compute(var_desc);
    LOG(INFO) << "COMPUTE FINISHED";
    for (const auto& p : test.coef) {
      LOG(INFO) << "    (" << p.first << ", " << p.second << ")";
    }
    return nullptr;
    */
  ///////////////////////////////////////////////////////////////////
  var_desc.set_type(lp_solver::VariableDesc_VariableType_TOTAL_SCORE);
  LOG(INFO) << "Computing the cost function";
  auto cost_fn = Compute(var_desc);
  LOG(INFO) << aux_var_cache_.size() << " entries in the CACHE.";
  LOG(INFO) << "Cost function depnds on " << cost_fn.coef.size()
            << " decision variables.";
  var_desc.Clear();

  using operations_research::MPConstraint;
  using operations_research::MPObjective;
  using operations_research::MPSolver;
  using operations_research::MPVariable;

  MPSolver solver("delivery", MPSolver::GLOP_LINEAR_PROGRAMMING);
  const double inf = solver.infinity();

  // Create vars.
  std::unordered_map<std::string, MPVariable*> vars;
  for (const auto& var : cost_fn.coef) {
    if (var.first == "const") continue;
    vars[var.first] = solver.MakeNumVar(0.0, 1.0, var.first);
  }

  // Set the objective.
  LOG(INFO) << "Setting the objective.";
  MPObjective* objective = solver.MutableObjective();
  for (const auto& var : cost_fn.coef) {
    if (var.first == "const") continue;
    objective->SetCoefficient(vars[var.first], var.second);
  }
  objective->SetMaximization();

  auto var_access = [&](const std::string& var) {
    if (vars.count(var) == 0) {
      vars[var] = solver.MakeNumVar(0.0, 1.0, var);
    }
    return vars[var];
  };

  // (1) WAREHOUSE_STATE >= 0
  LOG(INFO) << "Creating WAREHOUSE_STATE >= 0 constraints.";
  var_desc.set_type(lp_solver::VariableDesc_VariableType_WAREHOUSE_STATE);
  for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
    var_desc.set_warehouse(warehouse);
    for (int product = 0; product < problem_.np(); product++) {
      var_desc.set_product(product);
      for (int t = 1; t <= simulation_time_; t++) {
        var_desc.set_t(t);
        auto poly = Compute(var_desc);
        MPConstraint* c =
            solver.MakeRowConstraint(0.0 - poly.coef["const"], inf);
        for (const auto& var : poly.coef) {
          if (var.first == "const") continue;
          c->SetCoefficient(var_access(var.first), var.second);
        }
      }
    }
  }
  var_desc.Clear();

  // (2) 0 <= DRONE_STATE <= MAX_WEIGHT
  LOG(INFO) << "Creating 0 <= DRONE_STATE <= MAX_WEIGHT constraints.";
  var_desc.set_type(lp_solver::VariableDesc_VariableType_DRONE_STATE);
  for (int drone = 0; drone < problem_.nd(); drone++) {
    var_desc.set_drone(drone);
    for (int t = 1; t <= simulation_time_; t++) {
      var_desc.set_t(t);
      Polynomial drone_total;
      for (int product = 0; product < problem_.np(); product++) {
        var_desc.set_product(product);

        auto poly = Compute(var_desc);
        MPConstraint* c =
            solver.MakeRowConstraint(0.0 - poly.coef["const"], inf);
        for (const auto& var : poly.coef) {
          if (var.first == "const") continue;
          c->SetCoefficient(var_access(var.first), var.second);
        }
        poly *= problem_.product(product).m();
        drone_total += poly;
      }
      MPConstraint* c = solver.MakeRowConstraint(
          -inf, problem_.m() - drone_total.coef["const"]);
      for (const auto& var : drone_total.coef) {
        if (var.first == "const") continue;
        c->SetCoefficient(var_access(var.first), var.second);
      }
    }
  }
  var_desc.Clear();

  // (3) ORDER_STATE >= 0
  LOG(INFO) << "Creating ORDER_STATE >= 0 constraints.";
  var_desc.set_type(lp_solver::VariableDesc_VariableType_ORDER_STATE);
  for (int order = 0; order < problem_.no(); order++) {
    var_desc.set_order(order);
    for (int product = 0; product < problem_.np(); product++) {
      var_desc.set_product(product);
      for (int t = 1; t <= simulation_time_; t++) {
        var_desc.set_t(t);
        auto poly = Compute(var_desc);
        MPConstraint* c =
            solver.MakeRowConstraint(0.0 - poly.coef["const"], inf);
        for (const auto& var : poly.coef) {
          if (var.first == "const") continue;
          c->SetCoefficient(var_access(var.first), var.second);
        }
      }
    }
  }
  var_desc.Clear();

  // (4) Location constraints.
  LOG(INFO) << "Creating location constraints.";
  for (int drone = 0; drone < problem_.nd(); drone++) {
    var_desc.set_drone(drone);
    for (int t = 1; t <= simulation_time_; t++) {
      var_desc.set_t(t);
      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        var_desc.set_location(loc);

        // LOG(INFO) << "d, t, l = " << drone << ", " << t << ", " << loc;

        var_desc.set_type(lp_solver::VariableDesc_VariableType_DRONE_LOC);
        Polynomial minus_drone_loc = Compute(var_desc);
        minus_drone_loc *= -1.0;
        Polynomial poly;

        // wait - drone_loc <= 0;
        var_desc.set_type(lp_solver::VariableDesc_VariableType_WAIT);
        poly = minus_drone_loc;
        poly.coef[lin_prog::SavyProtoHash(var_desc)] += 1.0;
        {
          MPConstraint* c =
              solver.MakeRowConstraint(-inf, 0.0 - poly.coef["const"]);
          for (const auto& var : poly.coef) {
            if (var.first == "const") continue;
            c->SetCoefficient(var_access(var.first), var.second);
          }
        }

        for (int product = 0; product < problem_.np(); product++) {
          var_desc.set_product(product);
          // load - drone_loc <= 0;
          // unload - drone_loc <= 0;
          for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
            var_desc.set_warehouse(warehouse);
            int drone_cap_items = problem_.m() / problem_.product(product).m();
            for (int num_items = 1; num_items <= drone_cap_items; num_items++) {
              var_desc.set_num_items(num_items);

              poly = minus_drone_loc;
              var_desc.set_type(lp_solver::VariableDesc_VariableType_LOAD);
              poly.coef[lin_prog::SavyProtoHash(var_desc)] += 1.0;
              {
                MPConstraint* c =
                    solver.MakeRowConstraint(-inf, 0.0 - poly.coef["const"]);
                for (const auto& var : poly.coef) {
                  if (var.first == "const") continue;
                  c->SetCoefficient(var_access(var.first), var.second);
                }
              }

              poly = minus_drone_loc;
              var_desc.set_type(lp_solver::VariableDesc_VariableType_UNLOAD);
              poly.coef[lin_prog::SavyProtoHash(var_desc)] += 1.0;
              {
                MPConstraint* c =
                    solver.MakeRowConstraint(-inf, 0.0 - poly.coef["const"]);
                for (const auto& var : poly.coef) {
                  if (var.first == "const") continue;
                  c->SetCoefficient(var_access(var.first), var.second);
                }
              }
            }
          }
          var_desc.clear_warehouse();

          // deliver - drone_loc <= 0
          for (int order = 0; order < problem_.no(); order++) {
            var_desc.set_order(order);
            int max_num_items =
                std::min(problem_.order(order).request(product),
                         problem_.m() / problem_.product(product).m());
            for (int num_items = 1; num_items <= max_num_items; num_items++) {
              var_desc.set_num_items(num_items);

              poly = minus_drone_loc;
              var_desc.set_type(lp_solver::VariableDesc_VariableType_DELIVER);
              poly.coef[lin_prog::SavyProtoHash(var_desc)] += 1.0;
              {
                MPConstraint* c =
                    solver.MakeRowConstraint(-inf, 0.0 - poly.coef["const"]);
                for (const auto& var : poly.coef) {
                  if (var.first == "const") continue;
                  c->SetCoefficient(var_access(var.first), var.second);
                }
              }
            }
          }
        }
      }
    }
  }
  var_desc.Clear();

  // (5) Command overlapping constraints
  LOG(INFO) << "Creating command overlapping constraints.";
  for (int drone = 0; drone < problem_.nd(); drone++) {
    for (int t = 1; t <= simulation_time_; t++) {
      // Find all possible commands overlapping with t.
      MPConstraint* c = solver.MakeRowConstraint(-inf, 1.0);
      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        // WAIT
        var_desc.set_drone(drone);
        var_desc.set_t(t);
        var_desc.set_location(loc);
        var_desc.set_type(lp_solver::VariableDesc_VariableType_WAIT);
        c->SetCoefficient(var_access(lin_prog::SavyProtoHash(var_desc)), 1.0);

        auto actual_location =
            loc < problem_.nw()
                ? problem_.warehouse(loc).location()
                : problem_.order(loc - problem_.nw()).location();
        // LOADs and UNLOADs
        for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
          var_desc.set_warehouse(warehouse);
          int dt = Distance(actual_location,
                            problem_.warehouse(warehouse).location());
          int min_start_time = std::max(t - dt, 1);
          for (int start_time = min_start_time; start_time <= t + dt;
               start_time++) {
            var_desc.set_t(start_time);
            for (int product = 0; product < problem_.np(); product++) {
              var_desc.set_product(product);
              int drone_cap_items =
                  problem_.m() / problem_.product(product).m();
              for (int num_items = 1; num_items <= drone_cap_items;
                   num_items++) {
                var_desc.set_num_items(num_items);

                var_desc.set_type(lp_solver::VariableDesc_VariableType_LOAD);
                c->SetCoefficient(var_access(lin_prog::SavyProtoHash(var_desc)),
                                  1.0);
                var_desc.set_type(lp_solver::VariableDesc_VariableType_UNLOAD);
                c->SetCoefficient(var_access(lin_prog::SavyProtoHash(var_desc)),
                                  1.0);
              }
            }
          }
        }
        var_desc.clear_warehouse();
        // DELIVERs
        for (int order = 0; order < problem_.no(); order++) {
          var_desc.set_order(order);
          int dt = Distance(actual_location, problem_.order(order).location());
          for (int start_time = t - dt; start_time <= t + dt; start_time++) {
            var_desc.set_t(start_time);
            for (int product = 0; product < problem_.np(); product++) {
              var_desc.set_product(product);
              int max_num_items =
                  std::min(problem_.m() / problem_.product(product).m(),
                           problem_.order(order).request(product));
              for (int num_items = 1; num_items <= max_num_items; num_items++) {
                var_desc.set_num_items(num_items);

                var_desc.set_type(lp_solver::VariableDesc_VariableType_DELIVER);
                c->SetCoefficient(var_access(lin_prog::SavyProtoHash(var_desc)),
                                  1.0);
              }
            }
          }
        }
      }
    }
  }
  var_desc.Clear();

  LOG(INFO) << "Number of variables = " << solver.NumVariables();
  LOG(INFO) << "Number of constraints = " << solver.NumConstraints();

  LOG(INFO) << "Starting the solver";
  solver.Solve();

  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  for (int drone = 0; drone < problem_.nd(); drone++) {
    solution->add_drone_desc();
  }

  LOG(INFO) << "Reinterpreting the solution";
  int nz = 0;
  std::vector<int> buckets(10, 0);
  for (const auto& p : vars) {
    int b =
        static_cast<int>(p.second->solution_value() / (1.0 / buckets.size()));
    if (b >= buckets.size()) b = buckets.size() - 1;
    CHECK(b >= 0);
    buckets[b]++;

    int val = round(p.second->solution_value());
    if (val == 0) continue;
    nz++;
    CHECK(var_desc.ParseFromString(p.first)) << "Failed to parse var_desc.";
    auto* cmd =
        solution->mutable_drone_desc(var_desc.drone())->add_drone_command();
    cmd->set_drone_id(var_desc.drone());
    cmd->set_start_time(var_desc.t());
    switch (var_desc.type()) {
      case lp_solver::VariableDesc_VariableType_WAIT: {
        cmd->set_type(DroneCommand_CommandType_WAIT);
        cmd->set_duration(1);
        break;
      }
      case lp_solver::VariableDesc_VariableType_LOAD: {
        cmd->set_type(DroneCommand_CommandType_LOAD);
        cmd->set_product(var_desc.product());
        cmd->set_num_items(var_desc.num_items());
        cmd->set_warehouse(var_desc.warehouse());
        break;
      }
      case lp_solver::VariableDesc_VariableType_UNLOAD: {
        cmd->set_type(DroneCommand_CommandType_UNLOAD);
        cmd->set_product(var_desc.product());
        cmd->set_num_items(var_desc.num_items());
        cmd->set_warehouse(var_desc.warehouse());
        break;
      }
      case lp_solver::VariableDesc_VariableType_DELIVER: {
        cmd->set_type(DroneCommand_CommandType_DELIVER);
        cmd->set_product(var_desc.product());
        cmd->set_num_items(var_desc.num_items());
        cmd->set_order(var_desc.order());
        break;
      }
      default:
        CHECK(false) << "Variable type not a command!";
    }
  }
  LOG(INFO) << nz << " vars rounded to 1.";
  LOG(INFO) << "Buckets:";
  for (int b = 0; b < buckets.size(); b++) {
    double step = 1.0 / buckets.size();
    LOG(INFO) << "  " << b * step << " -> " << (b + 1) * step << " : "
              << buckets[b];
  }

  return solution;
}

}  // namespace drones
