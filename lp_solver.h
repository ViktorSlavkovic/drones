#ifndef LP_SOLVER_H_
#define LP_SOLVER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "glog/logging.h"
#include "lp_solver.pb.h"
#include "lp_util.h"
#include "problem_solver.h"

namespace drones {

class LpSolver : public ProblemSolver {
 public:
  explicit LpSolver(const Problem& problem);
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override { return true; }

 private:
  using Polynomial = drones::lin_prog::Polynomial;
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

  // Cache initial (at the simulation start, t = 0) values of the auxiliary
  // variables.
  void CacheInitial();
  // Computes the the auxiliary variables as decision variables polynomials.
  Polynomial Compute(const drones::lp_solver::VariableDesc& var);
};

}  // namespace drones

#endif  // LP_SOLVER_H_
