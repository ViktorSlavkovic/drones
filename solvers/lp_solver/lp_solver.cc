#include "solvers/lp_solver/lp_solver.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <vector>

#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ortools/linear_solver/linear_solver.h"
#include "ortools/linear_solver/linear_solver.pb.h"
#include "problem.pb.h"
#include "solvers/util/lp_util.h"

namespace drones {

static const std::vector<double> optimized_coefficients = {
    3.97976974e-04,  1.09920291e-04,  1.24789101e-05,  1.00218842e-04,
    2.55923712e-05,  6.57370644e-05,  1.66619271e-05,  2.46939847e-05,
    3.71850811e-05,  7.96405444e-06,  3.37405589e-05,  7.42299103e-05,
    2.03804329e-05,  -1.02013754e-05, 7.27355911e-05,  -6.82899826e-05,
    9.10657820e-05,  4.40944297e-05,  -1.96798326e-05, 2.46923373e-05,
    3.43858149e-05,  2.47138953e-05,  2.08188792e-05,  -1.24052315e-05,
    6.49092645e-05,  3.29523753e-05,  1.27576225e-05,  -5.24360402e-05,
    9.82151767e-05,  -5.24125625e-05, 5.09604394e-05,  4.58811764e-06,
    4.34191609e-05,  1.50301652e-06,  2.43846349e-05,  7.34234518e-05,
    -7.86525213e-05, 6.11150305e-05,  -1.46308328e-05, 6.95157315e-05,
    -4.39462492e-05, 3.04064196e-05,  2.60963940e-05,  2.94883791e-05,
    -2.56231428e-06, 2.05119524e-05,  -3.43140953e-06, -1.46523141e-05,
    2.12068968e-05,  -5.85874689e-05, -1.13990709e-03};

// Does linear interpolation on the original vector of optimized coefficients
// to adjust it to the given size.
std::vector<double> InterpolateCoefficients(int size) {
  std::vector<double> res(size);
  double step = (size - 1.0) / (optimized_coefficients.size() - 1.0);
  double l = 0.0;
  int lidx = 0;
  for (int i = 0; i < size; i++) {
    while (i > l + step) {
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
      score_coefficients_(InterpolateCoefficients(2 * problem.t() + 1)) {
  CacheInitial();
}

void LpSolver::CacheInitial() {
  // Set initial DRONE_LOC at 1.
  {
    lp_solver::VariableDesc var;
    var.set_t(1);
    var.set_type(lp_solver::VariableDesc_VariableType_DRONE_LOC);
    for (int d = 0; d < problem_.nd(); d++) {
      var.set_drone(d);
      var.set_location(0);
      aux_var_cache_[util::lp::SavyProtoHash(var)].coef["const"] = 1;
      for (int loc = 1; loc < problem_.nw() + problem_.no(); loc++) {
        var.set_location(loc);
        aux_var_cache_[util::lp::SavyProtoHash(var)].coef["const"] = 0;
      }
    }
  }

  // Set initial WAREHOUSE_STATE at 0.
  {
    lp_solver::VariableDesc var;
    var.set_t(0);
    var.set_type(lp_solver::VariableDesc_VariableType_WAREHOUSE_STATE);
    for (int w = 0; w < problem_.nw(); w++) {
      var.set_warehouse(w);
      for (int p = 0; p < problem_.np(); p++) {
        var.set_product(p);
        aux_var_cache_[util::lp::SavyProtoHash(var)].coef["const"] =
            problem_.warehouse(w).stock(p);
      }
    }
  }

  // Set initial DRONE_STATE at 0.
  {
    lp_solver::VariableDesc var;
    var.set_t(0);
    var.set_type(lp_solver::VariableDesc_VariableType_DRONE_STATE);
    for (int d = 0; d < problem_.nd(); d++) {
      var.set_drone(d);
      for (int p = 0; p < problem_.np(); p++) {
        var.set_product(p);
        aux_var_cache_[util::lp::SavyProtoHash(var)].coef["const"] = 0;
      }
    }
  }

  // Set intial ORDER_STATE at 0.
  {
    lp_solver::VariableDesc var;
    var.set_t(0);
    var.set_type(lp_solver::VariableDesc_VariableType_ORDER_STATE);
    for (int o = 0; o < problem_.no(); o++) {
      var.set_order(o);
      for (int p = 0; p < problem_.np(); p++) {
        var.set_product(p);
        aux_var_cache_[util::lp::SavyProtoHash(var)].coef["const"] =
            problem_.order(o).request(p);
      }
    }
  }
}

void LpSolver::CheckAuxVar(const drones::lp_solver::VariableDesc& var,
                           const std::string& where) const {
  auto msg =
      absl::Substitute("\nwhere: $0\nvar:\n$1", where, var.DebugString());
  CHECK(var.has_type()) << msg;
  switch (var.type()) {
    case lp_solver::VariableDesc_VariableType_TOTAL_SCORE: {
      CHECK(!var.has_drone()) << msg;
      CHECK(!var.has_location()) << msg;
      CHECK(!var.has_t()) << msg;
      CHECK(!var.has_warehouse()) << msg;
      CHECK(!var.has_product()) << msg;
      CHECK(!var.has_order()) << msg;
      CHECK(!var.has_num_items()) << msg;
      break;
    }
    case lp_solver::VariableDesc_VariableType_ORDER_SCORE: {
      CHECK(!var.has_drone()) << msg;
      CHECK(!var.has_location()) << msg;
      CHECK(!var.has_t()) << msg;
      CHECK(!var.has_warehouse()) << msg;
      CHECK(!var.has_product()) << msg;
      CHECK(var.has_order()) << msg;
      CHECK(var.order() >= 0) << msg;
      CHECK(var.order() < problem_.no()) << msg;
      CHECK(!var.has_num_items()) << msg;
      break;
    }
    case lp_solver::VariableDesc_VariableType_ORDER_COMPLETENESS: {
      CHECK(!var.has_drone()) << msg;
      CHECK(!var.has_location()) << msg;
      CHECK(var.has_t()) << msg;
      CHECK(var.t() >= 1) << msg;
      CHECK(var.t() <= simulation_time_) << msg;
      CHECK(!var.has_warehouse()) << msg;
      CHECK(!var.has_product()) << msg;
      CHECK(var.has_order()) << msg;
      CHECK(var.order() >= 0) << msg;
      CHECK(var.order() < problem_.no()) << msg;
      CHECK(!var.has_num_items()) << msg;
      break;
    }
    case lp_solver::VariableDesc_VariableType_ORDER_STATE: {
      CHECK(!var.has_drone()) << msg;
      CHECK(!var.has_location()) << msg;
      CHECK(var.has_t()) << msg;
      CHECK(var.t() >= 0) << msg;
      CHECK(var.t() <= simulation_time_) << msg;
      CHECK(!var.has_warehouse()) << msg;
      CHECK(var.has_product()) << msg;
      CHECK(var.product() >= 0) << msg;
      CHECK(var.product() < problem_.np()) << msg;
      CHECK(var.has_order()) << msg;
      CHECK(var.order() >= 0) << msg;
      CHECK(var.order() < problem_.no()) << msg;
      CHECK(!var.has_num_items()) << msg;
      break;
    }
    case lp_solver::VariableDesc_VariableType_DRONE_STATE: {
      CHECK(var.has_drone()) << msg;
      CHECK(var.drone() >= 0) << msg;
      CHECK(var.drone() < problem_.nd()) << msg;
      CHECK(!var.has_location()) << msg;
      CHECK(var.has_t()) << msg;
      CHECK(var.t() >= 0) << msg;
      CHECK(var.t() <= simulation_time_) << msg;
      CHECK(!var.has_warehouse()) << msg;
      CHECK(var.has_product()) << msg;
      CHECK(var.product() >= 0) << msg;
      CHECK(var.product() < problem_.np()) << msg;
      CHECK(!var.has_order()) << msg;
      CHECK(!var.has_num_items()) << msg;
      break;
    }
    case lp_solver::VariableDesc_VariableType_DRONE_LOC: {
      CHECK(var.has_drone()) << msg;
      CHECK(var.drone() >= 0) << msg;
      CHECK(var.drone() < problem_.nd()) << msg;
      CHECK(var.has_location()) << msg;
      CHECK(var.location() >= 0) << msg;
      CHECK(var.location() < problem_.nw() + problem_.no()) << msg;
      CHECK(var.has_t()) << msg;
      CHECK(var.t() >= 1) << msg;
      CHECK(var.t() <= simulation_time_) << msg;
      CHECK(!var.has_warehouse()) << msg;
      CHECK(!var.has_product()) << msg;
      CHECK(!var.has_order()) << msg;
      CHECK(!var.has_num_items()) << msg;
      break;
    }
    case lp_solver::VariableDesc_VariableType_WAREHOUSE_STATE: {
      CHECK(!var.has_drone()) << msg;
      CHECK(!var.has_location()) << msg;
      CHECK(var.has_t()) << msg;
      CHECK(var.t() >= 0) << msg;
      CHECK(var.t() <= simulation_time_) << msg;
      CHECK(var.has_warehouse()) << msg;
      CHECK(var.warehouse() >= 0) << msg;
      CHECK(var.warehouse() < problem_.nw()) << msg;
      CHECK(var.has_product()) << msg;
      CHECK(var.product() >= 0) << msg;
      CHECK(var.product() < problem_.np()) << msg;
      CHECK(!var.has_order()) << msg;
      CHECK(!var.has_num_items()) << msg;
      break;
    }
    default:
      CHECK(false) << msg;
  }
}

void LpSolver::CheckOptVar(const drones::lp_solver::VariableDesc& var,
                           const std::string& where) const {
  auto msg =
      absl::Substitute("\nwhere: $0\nvar:\n$1", where, var.DebugString());
  CHECK(var.has_t()) << msg;
  CHECK(var.t() >= 1) << msg;
  CHECK(var.t() <= simulation_time_) << msg;
  CHECK(var.has_drone()) << msg;
  CHECK(var.drone() >= 0) << msg;
  CHECK(var.drone() < problem_.nd()) << msg;
  CHECK(var.has_location()) << msg;
  CHECK(var.location() >= 0) << msg;
  CHECK(var.location() < problem_.nw() + problem_.no()) << msg;
  CHECK(var.has_type()) << msg;
  switch (var.type()) {
    case lp_solver::VariableDesc_VariableType_LOAD:
    case lp_solver::VariableDesc_VariableType_UNLOAD: {
      CHECK(!var.has_order()) << msg;
      CHECK(var.has_warehouse()) << msg;
      CHECK(var.warehouse() >= 0) << msg;
      CHECK(var.warehouse() < problem_.nw()) << msg;
      CHECK(var.has_product()) << msg;
      CHECK(var.product() >= 0) << msg;
      CHECK(var.product() < problem_.np()) << msg;
      CHECK(var.has_num_items()) << msg;
      CHECK(var.num_items() >= 1) << msg;
      int finish_time =
          var.t() + problem_.dist().src(var.location()).dst(var.warehouse());
      CHECK(finish_time <= simulation_time_)
          << msg << absl::Substitute("\nfinish_time: $0", finish_time);
      break;
    }
    case lp_solver::VariableDesc_VariableType_DELIVER: {
      CHECK(!var.has_warehouse()) << msg;
      CHECK(var.has_order()) << msg;
      CHECK(var.order() >= 0) << msg;
      CHECK(var.order() < problem_.no()) << msg;
      CHECK(var.has_product()) << msg;
      CHECK(var.product() >= 0) << msg;
      CHECK(var.product() < problem_.np()) << msg;
      CHECK(var.has_num_items()) << msg;
      CHECK(var.num_items() >= 1) << msg;
      int finish_time =
          var.t() +
          problem_.dist().src(var.location()).dst(problem_.nw() + var.order());
      CHECK(finish_time <= simulation_time_)
          << msg << absl::Substitute("\nfinish_time: $0", finish_time);
      break;
    }
    case lp_solver::VariableDesc_VariableType_WAIT:
      CHECK(!var.has_order()) << msg;
      CHECK(!var.has_warehouse()) << msg;
      CHECK(!var.has_product()) << msg;
      CHECK(!var.has_num_items()) << msg;
      break;
    default:
      CHECK(false) << msg;
  }
}

util::lp::Polynomial LpSolver::Compute(
    const drones::lp_solver::VariableDesc& var) {
  // Check the var validity.
  CheckAuxVar(var, "COMPUTE ENTRANCE");

  // Check the cache.
  std::string var_hash = util::lp::SavyProtoHash(var);
  if (aux_var_cache_.count(var_hash)) {
    return aux_var_cache_[var_hash];
  }

  // Handle different variable types.
  util::lp::Polynomial res;
  drones::lp_solver::VariableDesc dep_var;
  switch (var.type()) {
    case lp_solver::VariableDesc_VariableType_TOTAL_SCORE: {
      dep_var.set_type(lp_solver::VariableDesc_VariableType_ORDER_SCORE);
      for (int o = 0; o < problem_.no(); o++) {
        LOG(INFO) << absl::Substitute("TOTAL_SCORE computation: order $0/$1. ",
                                      o + 1, problem_.no());
        dep_var.set_order(o);
        res += Compute(dep_var);
      }
      break;
    }
    case lp_solver::VariableDesc_VariableType_ORDER_SCORE: {
      dep_var.set_type(lp_solver::VariableDesc_VariableType_ORDER_COMPLETENESS);
      dep_var.set_order(var.order());
      for (int t = 1; t <= simulation_time_; t++) {
        dep_var.set_t(t);
        auto order_completeness = Compute(dep_var);
        order_completeness *= score_coefficients_[t - 1];
        res += order_completeness;
      }
      break;
    }
    case lp_solver::VariableDesc_VariableType_ORDER_COMPLETENESS: {
      dep_var.set_type(lp_solver::VariableDesc_VariableType_ORDER_STATE);
      dep_var.set_order(var.order());
      dep_var.set_t(var.t());
      int total_order_items = 0;
      for (int p = 0; p < problem_.np(); p++) {
        total_order_items += problem_.order(var.order()).request(p);
        dep_var.set_product(p);
        res += Compute(dep_var);
      }
      res *= -1.0 / total_order_items;
      res.coef["const"] += 1;
      break;
    }
    case lp_solver::VariableDesc_VariableType_ORDER_STATE: {
      dep_var.set_type(lp_solver::VariableDesc_VariableType_ORDER_STATE);
      dep_var.set_order(var.order());
      dep_var.set_t(var.t() - 1);
      dep_var.set_product(var.product());
      res += Compute(dep_var);
      dep_var.Clear();

      util::lp::Polynomial delivery;
      dep_var.set_type(lp_solver::VariableDesc_VariableType_DELIVER);
      dep_var.set_product(var.product());
      dep_var.set_order(var.order());

      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        dep_var.set_location(loc);

        // State at the end of the tick - after the delivery.
        int start_time =
            var.t() - problem_.dist().src(loc).dst(problem_.nw() + var.order());
        if (start_time < 1) continue;
        dep_var.set_t(start_time);

        for (int d = 0; d < problem_.nd(); d++) {
          dep_var.set_drone(d);
          int max_n =
              std::min(problem_.order(var.order()).request(var.product()),
                       problem_.m() / problem_.product(var.product()).m());
          for (int n = 1; n <= max_n; n++) {
            dep_var.set_num_items(n);
            delivery.coef[util::lp::SavyProtoHash(dep_var)] = -n;
          }
        }
      }
      res += delivery;
      break;
    }
    case lp_solver::VariableDesc_VariableType_DRONE_STATE: {
      dep_var.set_type(lp_solver::VariableDesc_VariableType_DRONE_STATE);
      dep_var.set_drone(var.drone());
      dep_var.set_t(var.t() - 1);
      dep_var.set_product(var.product());
      res += Compute(dep_var);
      dep_var.Clear();

      util::lp::Polynomial traffic;
      dep_var.set_drone(var.drone());
      dep_var.set_product(var.product());

      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        dep_var.set_location(loc);

        // This is essentially the available state at this step minus what's
        // being taken in this step - so LOAD should be endning in the
        // previous step and UNLOAD/DELIVER should be ending in this step.

        // Handle LOAD and UNLOAD commands.
        for (int w = 0; w < problem_.nw(); w++) {
          dep_var.set_warehouse(w);
          // Handle LOADs.
          int start_time = var.t() - problem_.dist().src(loc).dst(w) - 1;
          if (start_time >= 1) {
            dep_var.set_t(start_time);
            int max_n = problem_.m() / problem_.product(var.product()).m();
            for (int n = 1; n <= max_n; n++) {
              dep_var.set_num_items(n);
              dep_var.set_type(lp_solver::VariableDesc_VariableType_LOAD);
              traffic.coef[util::lp::SavyProtoHash(dep_var)] = n;
            }
          }
          // Handle UNLOADs.
          start_time = var.t() - problem_.dist().src(loc).dst(w);
          if (start_time >= 1) {
            dep_var.set_t(start_time);
            int max_n = problem_.m() / problem_.product(var.product()).m();
            for (int n = 1; n <= max_n; n++) {
              dep_var.set_num_items(n);
              dep_var.set_type(lp_solver::VariableDesc_VariableType_UNLOAD);
              traffic.coef[util::lp::SavyProtoHash(dep_var)] = -n;
            }
          }
        }
        dep_var.clear_warehouse();

        // Handle DELIVER commands.
        dep_var.set_type(lp_solver::VariableDesc_VariableType_DELIVER);
        for (int o = 0; o < problem_.no(); o++) {
          dep_var.set_order(o);

          int start_time =
              var.t() - problem_.dist().src(loc).dst(problem_.nw() + o);
          if (start_time < 1) continue;
          dep_var.set_t(start_time);

          int max_n =
              std::min(problem_.order(o).request(var.product()),
                       problem_.m() / problem_.product(var.product()).m());
          for (int n = 1; n <= max_n; n++) {
            dep_var.set_num_items(n);
            traffic.coef[util::lp::SavyProtoHash(dep_var)] = -n;
          }
        }
        dep_var.clear_order();
      }
      res += traffic;
      break;
    }
    case lp_solver::VariableDesc_VariableType_DRONE_LOC: {
      dep_var.set_type(lp_solver::VariableDesc_VariableType_WAIT);
      dep_var.set_drone(var.drone());
      dep_var.set_location(var.location());
      dep_var.set_t(var.t() - 1);
      res.coef[util::lp::SavyProtoHash(dep_var)] = 1;
      dep_var.Clear();

      dep_var.set_drone(var.drone());
      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        dep_var.set_location(loc);

        // This is now the drone's location at the beginning of the tick, so
        // we're considering where the commands which have ended in the previous
        // move have left us.
        int start_time =
            var.t() - problem_.dist().src(loc).dst(var.location()) - 1;
        if (start_time < 1) continue;
        dep_var.set_t(start_time);

        for (int p = 0; p < problem_.np(); p++) {
          dep_var.set_product(p);
          if (var.location() < problem_.nw()) {
            // LOADs and UNLOADs.
            dep_var.set_warehouse(var.location());
            int max_n = problem_.m() / problem_.product(p).m();
            for (int n = 1; n <= max_n; n++) {
              dep_var.set_num_items(n);
              dep_var.set_type(lp_solver::VariableDesc_VariableType_UNLOAD);
              res.coef[util::lp::SavyProtoHash(dep_var)] = 1;
              dep_var.set_type(lp_solver::VariableDesc_VariableType_LOAD);
              res.coef[util::lp::SavyProtoHash(dep_var)] = 1;
            }
          } else {
            // DELIVERs.
            int o = var.location() - problem_.nw();
            dep_var.set_order(o);
            int max_n = std::min(problem_.order(o).request(p),
                                 problem_.m() / problem_.product(p).m());
            for (int n = 1; n <= max_n; n++) {
              dep_var.set_num_items(n);
              dep_var.set_type(lp_solver::VariableDesc_VariableType_DELIVER);
              res.coef[util::lp::SavyProtoHash(dep_var)] = 1;
            }
          }
        }
      }
      break;
    }
    case lp_solver::VariableDesc_VariableType_WAREHOUSE_STATE: {
      dep_var.set_type(lp_solver::VariableDesc_VariableType_WAREHOUSE_STATE);
      dep_var.set_warehouse(var.warehouse());
      dep_var.set_product(var.product());
      dep_var.set_t(var.t() - 1);
      res += Compute(dep_var);
      dep_var.Clear();

      util::lp::Polynomial traffic;
      dep_var.set_warehouse(var.warehouse());
      dep_var.set_product(var.product());

      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        dep_var.set_location(loc);

        // This is essentially the available state at this step minus what's
        // being taken in this step - so UNLOAD should be endning in the
        // previous step and LOAD should be ending in this step.

        // Handle LOADs.
        dep_var.set_type(lp_solver::VariableDesc_VariableType_LOAD);
        int start_time =
            var.t() - problem_.dist().src(loc).dst(var.warehouse());
        if (start_time >= 1) {
          dep_var.set_t(start_time);
          for (int d = 0; d < problem_.nd(); d++) {
            dep_var.set_drone(d);
            int max_n = problem_.m() / problem_.product(var.product()).m();
            for (int n = 1; n <= max_n; n++) {
              dep_var.set_num_items(n);
              traffic.coef[util::lp::SavyProtoHash(dep_var)] = -n;
            }
          }
        }

        // Handle UNLOADs.
        dep_var.set_type(lp_solver::VariableDesc_VariableType_UNLOAD);
        start_time =
            var.t() - problem_.dist().src(loc).dst(var.warehouse()) - 1;
        if (start_time >= 1) {
          dep_var.set_t(start_time);
          for (int d = 0; d < problem_.nd(); d++) {
            dep_var.set_drone(d);
            int max_n = problem_.m() / problem_.product(var.product()).m();
            for (int n = 1; n <= max_n; n++) {
              dep_var.set_num_items(n);
              traffic.coef[util::lp::SavyProtoHash(dep_var)] = n;
            }
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

  var_desc.set_type(lp_solver::VariableDesc_VariableType_TOTAL_SCORE);
  LOG(INFO) << "Computing the cost functioni...";
  auto cost_fn = Compute(var_desc);
  LOG(INFO) << absl::Substitute("$0 entries in the CACHE",
                                aux_var_cache_.size());
  LOG(INFO) << absl::Substitute("Cost function depnds on $0 decision vars.",
                                cost_fn.coef.size());
  var_desc.Clear();

  using operations_research::MPConstraint;
  using operations_research::MPObjective;
  using operations_research::MPSolver;
  using operations_research::MPVariable;

  MPSolver solver(
      "delivery",
      MPSolver::GLPK_MIXED_INTEGER_PROGRAMMING);  // GLOP_LINEAR_PROGRAMMING);
  const double inf = solver.infinity();

  // Create vars.
  uint64_t var_int_names = 0;
  std::unordered_map<std::string, MPVariable*> vars;

  for (const auto& var : cost_fn.coef) {
    if (var.first == "const") continue;
    vars[var.first] =
        solver.MakeIntVar(0.0, 1.0, std::to_string(var_int_names++));
    lp_solver::VariableDesc temp;
    CHECK(temp.ParseFromString(var.first));
    CheckOptVar(temp, "cost_fn");
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
      vars[var] = solver.MakeIntVar(0.0, 1.0, std::to_string(var_int_names++));
    }
    return vars[var];
  };

  auto check_opt_vars = [&](const std::string& where) {
    for (const auto& var : vars) {
      lp_solver::VariableDesc temp;
      CHECK(temp.ParseFromString(var.first)) << where;
      CheckOptVar(temp, where);
    }
  };

  // (1) WAREHOUSE_STATE >= 0
  LOG(INFO) << "Creating WAREHOUSE_STATE >= 0 constraints.";
  var_desc.set_type(lp_solver::VariableDesc_VariableType_WAREHOUSE_STATE);
  for (int w = 0; w < problem_.nw(); w++) {
    var_desc.set_warehouse(w);
    for (int p = 0; p < problem_.np(); p++) {
      var_desc.set_product(p);
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
  check_opt_vars("(1)");

  // (2) 0 <= DRONE_STATE <= MAX_WEIGHT
  LOG(INFO) << "Creating 0 <= DRONE_STATE <= MAX_WEIGHT constraints.";
  var_desc.set_type(lp_solver::VariableDesc_VariableType_DRONE_STATE);
  for (int d = 0; d < problem_.nd(); d++) {
    var_desc.set_drone(d);
    for (int t = 1; t <= simulation_time_; t++) {
      var_desc.set_t(t);
      util::lp::Polynomial drone_total;
      for (int p = 0; p < problem_.np(); p++) {
        var_desc.set_product(p);

        auto poly = Compute(var_desc);
        MPConstraint* c =
            solver.MakeRowConstraint(0.0 - poly.coef["const"], inf);
        for (const auto& var : poly.coef) {
          if (var.first == "const") continue;
          c->SetCoefficient(var_access(var.first), var.second);
        }
        poly *= problem_.product(p).m();
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
  check_opt_vars("(2)");

  // (3) ORDER_STATE >= 0
  LOG(INFO) << "Creating ORDER_STATE >= 0 constraints.";
  var_desc.set_type(lp_solver::VariableDesc_VariableType_ORDER_STATE);
  for (int o = 0; o < problem_.no(); o++) {
    var_desc.set_order(o);
    for (int p = 0; p < problem_.np(); p++) {
      var_desc.set_product(p);
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
  check_opt_vars("(3)");

  // (4) Location constraints.
  LOG(INFO) << "Creating location constraints.";
  for (int d = 0; d < problem_.nd(); d++) {
    var_desc.set_drone(d);
    for (int t = 1; t <= simulation_time_; t++) {
      var_desc.set_t(t);
      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        var_desc.set_location(loc);

        var_desc.set_type(lp_solver::VariableDesc_VariableType_DRONE_LOC);
        util::lp::Polynomial minus_drone_loc = Compute(var_desc);
        minus_drone_loc *= -1.0;

        util::lp::Polynomial poly;

        // wait - drone_loc <= 0;
        var_desc.set_type(lp_solver::VariableDesc_VariableType_WAIT);
        poly = minus_drone_loc;
        poly.coef[util::lp::SavyProtoHash(var_desc)] += 1.0;
        {
          MPConstraint* c =
              solver.MakeRowConstraint(-inf, 0.0 - poly.coef["const"]);
          for (const auto& var : poly.coef) {
            if (var.first == "const") continue;
            c->SetCoefficient(var_access(var.first), var.second);
          }
        }

        for (int p = 0; p < problem_.np(); p++) {
          var_desc.set_product(p);
          // load - drone_loc <= 0;
          // unload - drone_loc <= 0;
          for (int w = 0; w < problem_.nw(); w++) {
            int finish_time = t + problem_.dist().src(loc).dst(w);
            if (finish_time > simulation_time_) continue;
            var_desc.set_warehouse(w);
            int max_n = problem_.m() / problem_.product(p).m();
            for (int n = 1; n <= max_n; n++) {
              var_desc.set_num_items(n);

              poly = minus_drone_loc;
              var_desc.set_type(lp_solver::VariableDesc_VariableType_LOAD);
              poly.coef[util::lp::SavyProtoHash(var_desc)] += 1.0;
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
              poly.coef[util::lp::SavyProtoHash(var_desc)] += 1.0;
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
          for (int o = 0; o < problem_.no(); o++) {
            int finish_time =
                t + problem_.dist().src(loc).dst(problem_.nw() + o);
            if (finish_time > simulation_time_) continue;
            var_desc.set_order(o);
            int max_n = std::min(problem_.order(o).request(p),
                                 problem_.m() / problem_.product(p).m());
            for (int n = 1; n <= max_n; n++) {
              var_desc.set_num_items(n);

              poly = minus_drone_loc;
              var_desc.set_type(lp_solver::VariableDesc_VariableType_DELIVER);
              CheckOptVar(var_desc, "da, da, tu");
              poly.coef[util::lp::SavyProtoHash(var_desc)] += 1.0;
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
          var_desc.clear_order();
        }
        var_desc.clear_num_items();
        var_desc.clear_product();
      }
    }
  }
  var_desc.Clear();
  check_opt_vars("(4)");

  // (5) Command overlapping constraints
  LOG(INFO) << "Creating command overlapping constraints.";
  for (int d = 0; d < problem_.nd(); d++) {
    for (int t = 1; t <= simulation_time_; t++) {
      // Find all possible commands overlapping with t.
      MPConstraint* c = solver.MakeRowConstraint(-inf, 1.0);
      for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
        // WAIT
        var_desc.set_drone(d);
        var_desc.set_t(t);
        var_desc.set_location(loc);
        var_desc.set_type(lp_solver::VariableDesc_VariableType_WAIT);
        c->SetCoefficient(var_access(util::lp::SavyProtoHash(var_desc)), 1.0);

        // LOADs and UNLOADs
        for (int w = 0; w < problem_.nw(); w++) {
          var_desc.set_warehouse(w);
          int dt = problem_.dist().src(loc).dst(w);
          int min_start_time = std::max(t - dt, 1);
          for (int start_time = min_start_time; start_time <= t; start_time++) {
            int finish_time = start_time + dt;
            if (finish_time > simulation_time_) break;
            var_desc.set_t(start_time);
            for (int p = 0; p < problem_.np(); p++) {
              var_desc.set_product(p);
              int max_n = problem_.m() / problem_.product(p).m();
              for (int n = 1; n <= max_n; n++) {
                var_desc.set_num_items(n);

                var_desc.set_type(lp_solver::VariableDesc_VariableType_LOAD);
                c->SetCoefficient(var_access(util::lp::SavyProtoHash(var_desc)),
                                  1.0);
                var_desc.set_type(lp_solver::VariableDesc_VariableType_UNLOAD);
                c->SetCoefficient(var_access(util::lp::SavyProtoHash(var_desc)),
                                  1.0);
              }
            }
          }
        }
        var_desc.clear_num_items();
        var_desc.clear_product();
        var_desc.clear_warehouse();

        // DELIVERs
        for (int o = 0; o < problem_.no(); o++) {
          var_desc.set_order(o);
          int dt = problem_.dist().src(loc).dst(problem_.nw() + o);
          for (int start_time = std::max(t - dt, 1); start_time <= t;
               start_time++) {
            int finish_time = start_time + dt;
            if (finish_time > simulation_time_) break;
            var_desc.set_t(start_time);
            for (int p = 0; p < problem_.np(); p++) {
              var_desc.set_product(p);
              int max_n = std::min(problem_.m() / problem_.product(p).m(),
                                   problem_.order(o).request(p));
              for (int n = 1; n <= max_n; n++) {
                var_desc.set_num_items(n);

                var_desc.set_type(lp_solver::VariableDesc_VariableType_DELIVER);
                c->SetCoefficient(var_access(util::lp::SavyProtoHash(var_desc)),
                                  1.0);
              }
            }
          }
        }
        var_desc.clear_num_items();
        var_desc.clear_product();
        var_desc.clear_order();
      }
    }
  }
  var_desc.Clear();
  check_opt_vars("(5)");

  LOG(INFO) << "Number of variables = " << solver.NumVariables();
  LOG(INFO) << "Number of constraints = " << solver.NumConstraints();

  LOG(INFO) << "Starting the solver";
  auto status = solver.Solve();
  LOG(INFO) << "Solver(): solver.Solve(): " << status;
  CHECK(status == MPSolver::ResultStatus::OPTIMAL ||
        status == MPSolver::ResultStatus::FEASIBLE);

  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  for (int d = 0; d < problem_.nd(); d++) {
    solution->add_drone_desc();
  }

  LOG(INFO) << "Reinterpreting the solution";
  int nz = 0;
  for (const auto& p : vars) {
    int val = round(p.second->solution_value());
    if (val == 0) continue;
    nz++;
    CHECK(var_desc.ParseFromString(p.first)) << "Failed to parse var_desc.";
    auto* cmd =
        solution->mutable_drone_desc(var_desc.drone())->add_drone_command();
    cmd->set_drone(var_desc.drone());
    cmd->set_start_time(var_desc.t());
    switch (var_desc.type()) {
      case lp_solver::VariableDesc_VariableType_WAIT: {
        if (var_desc.t() > problem_.t()) {
          solution->mutable_drone_desc(var_desc.drone())
              ->mutable_drone_command()
              ->RemoveLast();
          break;
        }
        cmd->set_type(DroneCommand_CommandType_WAIT);
        cmd->set_duration(1);
        break;
      }
      case lp_solver::VariableDesc_VariableType_LOAD: {
        if (var_desc.t() + problem_.dist()
                               .src(var_desc.location())
                               .dst(var_desc.warehouse()) >
            problem_.t()) {
          solution->mutable_drone_desc(var_desc.drone())
              ->mutable_drone_command()
              ->RemoveLast();
          break;
        }
        cmd->set_type(DroneCommand_CommandType_LOAD);
        cmd->set_product(var_desc.product());
        cmd->set_num_items(var_desc.num_items());
        cmd->set_warehouse(var_desc.warehouse());
        break;
      }
      case lp_solver::VariableDesc_VariableType_UNLOAD: {
        if (var_desc.t() + problem_.dist()
                               .src(var_desc.location())
                               .dst(var_desc.warehouse()) >
            problem_.t()) {
          solution->mutable_drone_desc(var_desc.drone())
              ->mutable_drone_command()
              ->RemoveLast();
          break;
        }
        cmd->set_type(DroneCommand_CommandType_UNLOAD);
        cmd->set_product(var_desc.product());
        cmd->set_num_items(var_desc.num_items());
        cmd->set_warehouse(var_desc.warehouse());
        break;
      }
      case lp_solver::VariableDesc_VariableType_DELIVER: {
        if (var_desc.t() + problem_.dist()
                               .src(var_desc.location())
                               .dst(problem_.nw() + var_desc.order()) >
            problem_.t()) {
          solution->mutable_drone_desc(var_desc.drone())
              ->mutable_drone_command()
              ->RemoveLast();
          break;
        }
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
  LOG(INFO) << nz << " ones.";

  for (int d = 0; d < problem_.nd(); d++) {
    std::sort(solution->mutable_drone_desc(d)->mutable_drone_command()->begin(),
              solution->mutable_drone_desc(d)->mutable_drone_command()->end(),
              [](const DroneCommand& cmd1, const DroneCommand& cmd2) {
                return cmd1.start_time() < cmd2.start_time();
              });
  }

  return solution;
}

}  // namespace drones
