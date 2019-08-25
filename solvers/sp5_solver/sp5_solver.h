#ifndef SOLVERS_SP5_SOLVER_SP5_SOLVER_H_
#define SOLVERS_SP5_SOLVER_SP5_SOLVER_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "glog/logging.h"
#include "solvers/lp_solver/lp_solver.pb.h"
#include "solvers/problem_solver.h"
#include "solvers/util/lp_util.h"

namespace drones {

class Sp5Solver : public ProblemSolver {
 public:
  explicit Sp5Solver(const Problem& problem);
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override { return true; }

 private:
  // Cache of auxiliary variable computations.
  using Cache = std::unordered_map<std::string, util::lp::LinComb>;

  // The cache object.
  Cache aux_var_cache_;
  // Simulation time used for the LP solution.
  const int simulation_time_;

  // Cache initial (at the simulation start, t = 0) values of the auxiliary
  // variables.
  void CacheInitial();
  
  // Check the validity of an auxiliary variable. Basically a CHECK pack, dies
  // if invalid.
  void CheckAuxVar(const drones::lp_solver::VariableDesc& var,
                   const std::string& where) const;
  // Same for the main optimization vars.
  void CheckOptVar(const drones::lp_solver::VariableDesc& var,
                   const std::string& where) const;

  // Computes the the auxiliary variables as decision variables linear
  // combinations.
  util::lp::LinComb Compute(const drones::lp_solver::VariableDesc& var);
};

}  // namespace drones

#endif  // SOLVERS_SP5_SOLVER_SP5_SOLVER_H_
