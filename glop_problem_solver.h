#ifndef GLOP_PROBLEM_SOLVER_H_
#define GLOP_PROBLEM_SOLVER_H_

#include "glop_problem_solver.pb.h"
#include "problem_solver.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace drones {

class GlopProblemSolver : public ProblemSolver {
 public:
  explicit GlopProblemSolver(const Problem& problem);
  std::unique_ptr<Solution> Solve() override;

 private:
  // Polynomial consisting of the decision variables.
  struct Polynomial {
    std::map<std::string, double> coef;

    // Overloading basic operators for the Polynomial type.
    Polynomial& operator*=(double rhs) {
      for (auto& p : coef) {
        p.second *= rhs;
      }
      return *this;
    }

    Polynomial& operator+=(const Polynomial& rhs) {
      for (const auto& p : rhs.coef) {
        coef[p.first] += p.second;
      }
      return *this;
    }
  };

  // Cache of auxiliary variable computations.
  using Cache = std::unordered_map<std::string, Polynomial>;

  // The cache object.
  Cache aux_var_cache_;
  // Simulation time used for the LP solution - it's big enough to allow all
  // orders to end.
  const int simulation_time_;
  // Order score coefficients standing by order_completeness auxiliary
  // variables as an attempt of problem linearization.
  const std::vector<double> score_coefficients_;
  // Maximum number of ordered items of a single product.
  int max_order_items_;

  // Cache initial (at the simulation start, t = 0) values of the auxiliary
  // variables.
  void CacheInitial();

  // Computes the the auxiliary variables as decision variables polynomials.
  Polynomial Compute(drones::glop_solver::VariableDesc& var);
};

}  // namespace drones

#endif  // GLOP_PROBLEM_SOLVER_H_
