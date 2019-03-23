#include "glop_problem_solver.h"

#include "glog/logging.h"
#include "problem.pb.h"

#include <algorithm>
#include <cmath>

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

GlopProblemSolver::GlopProblemSolver(const Problem& problem)
    : ProblemSolver(problem),
      aux_var_cache_(),
      simulation_time_(2 * problem.t()),
      score_coefficients_(InterpolateCoefficients(simulation_time_)),
      max_order_items_(0) {
  for (const auto& order : problem.order()) {
    for (int num_items : order.request()) {
      max_order_items_ = std::max(max_order_items_, num_items);
    }
  }
  CacheInitial();
}

void GlopProblemSolver::CacheInitial() {}

int Distance(const Location& src, const Location& dst) {
  int dx = src.x() - dst.x();
  int dy = src.y() - dst.y();
  return ceil(sqrt(dx * dx + dy * dy));
}

GlopProblemSolver::Polynomial GlopProblemSolver::Compute(
    drones::glop_solver::VariableDesc& var) {
  static std::string var_hash;
  static std::string aux_var_hash;
  CHECK(var.SerializeToString(&var_hash)) << "Failed to serialize.";
  if (aux_var_cache_.count(var_hash)) {
    return aux_var_cache_[var_hash];
  }

  Polynomial res;

  CHECK(var.has_type()) << "Unspecified variable type!";

  drones::glop_solver::VariableDesc dep_var;
  bool dont_cache = false;

  switch (var.type()) {
    case glop_solver::VariableDesc_VariableType_TOTAL_SCORE: {
      for (int order = 0; order < problem_.no(); order++) {
        dep_var.set_order(order);
        dep_var.set_type(glop_solver::VariableDesc_VariableType_ORDER_SCORE);
        res += Compute(dep_var);
      }
      break;
    }
    case glop_solver::VariableDesc_VariableType_ORDER_SCORE: {
      for (int t = 1; t <= simulation_time_; t++) {
        dep_var.set_order(var.order());
        dep_var.set_t(t);
        dep_var.set_type(
            glop_solver::VariableDesc_VariableType_ORDER_COMPLETENESS);
        auto order_completeness = Compute(dep_var);
        order_completeness *= score_coefficients_[t - 1] / simulation_time_;
        res += order_completeness;
      }
      break;
    }
    case glop_solver::VariableDesc_VariableType_ORDER_COMPLETENESS: {
      int total_order_items = 0;
      for (int product = 0; product < problem_.np(); product++) {
        total_order_items += problem_.order(var.order()).request(product);
        dep_var.set_product(product);
        dep_var.set_order(var.order());
        dep_var.set_t(var.t());
        dep_var.set_type(glop_solver::VariableDesc_VariableType_ORDER_STATE);
        res += Compute(dep_var);
      }
      res *= -1.0 / total_order_items;
      res.coef["const"] += 1;
      break;
    }
    case glop_solver::VariableDesc_VariableType_ORDER_STATE: {
      dep_var.set_order(var.order());
      dep_var.set_product(var.product());
      dep_var.set_t(var.t() - 1);
      dep_var.set_type(glop_solver::VariableDesc_VariableType_ORDER_STATE);
      res += Compute(dep_var);

      Polynomial delivery;
      dep_var.Clear();
      dep_var.set_product(var.product());
      dep_var.set_order(var.order());
      dep_var.set_type(glop_solver::VariableDesc_VariableType_DELIVER);
      for (int drone = 0; drone < problem_.nd(); drone++) {
        dep_var.set_drone(drone);
        for (int num_items = 0; num_items < max_order_items_; num_items++) {
          dep_var.set_num_items(num_items);
          for (int loc = 0; loc < problem_.nw() + problem_.no(); loc++) {
            dep_var.set_location(loc);
            auto actual_location =
                loc < problem_.nw()
                    ? problem_.warehouse(loc).location()
                    : problem_.order(loc - problem_.nw()).location();
            int start_time = var.t() -
                             Distance(actual_location,
                                      problem_.order(var.order()).location()) -
                             1;
            if (start_time < 1) {
              continue;
            }
            dep_var.set_t(start_time);
            CHECK(dep_var.SerializeToString(&aux_var_hash))
                << "Failed to serialize.";
            delivery.coef[aux_var_hash] = num_items;
          }
        }
      }
      delivery *= -1.0;
      res += delivery;
      break;
    }
    case glop_solver::VariableDesc_VariableType_DRONE_STATE: {
      break;
    }
    case glop_solver::VariableDesc_VariableType_DRONE_LOC: {
      break;
    }
    case glop_solver::VariableDesc_VariableType_WAREHOUSE_STATE: {
      break;
    }
    default: { CHECK(false) << "Not an auxiliary variable!"; }
  }

  if (!dont_cache) {
    aux_var_cache_[var_hash] = res;
  }

  return res;
}

std::unique_ptr<Solution> GlopProblemSolver::Solve() {
  glop_solver::VariableDesc var_desc;
  var_desc.set_type(glop_solver::VariableDesc_VariableType_TOTAL_SCORE);
  auto cost_fn = Compute(var_desc);
  LOG(INFO) << aux_var_cache_.size() << " entries in the CACHE.";
  LOG(INFO) << "Cost function depnds on " << cost_fn.coef.size()
            << " decision variables.";
  // TODO:
  //    1) Compute conditions.
  //    2) Solve with OR-Tools GLOP.
  //    3) Reinterpret as Solution proto.
  return nullptr;
}

}  // namespace drones
