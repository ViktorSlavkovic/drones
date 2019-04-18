#ifndef SOLVERS_ECF_SOLVER_ECF_SOLVER_H_
#define SOLVERS_ECF_SOLVER_ECF_SOLVER_H_

#include <memory>

#include "solvers/problem_solver.h"

namespace drones {

// Solves the original problem.
class EcfSolver : public ProblemSolver {
 public:
  explicit EcfSolver(const Problem& problem) : ProblemSolver(problem) {}
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override;
};

}  // namespace drones

#endif  // SOLVERS_ECF_SOLVER_ECF_SOLVER_H_
