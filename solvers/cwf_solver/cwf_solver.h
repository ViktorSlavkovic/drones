#ifndef SOLVERS_CWF_SOLVER_CWF_SOLVER_H_
#define SOLVERS_CWF_SOLVER_CWF_SOLVER_H_

#include <memory>

#include "solvers/problem_solver.h"

namespace drones {

class CwfSolver : public ProblemSolver {
 public:
  explicit CwfSolver(const Problem& problem) : ProblemSolver(problem) {}
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override { return true; }
};

}  // namespace drones

#endif  // SOLVERS_CWF_SOLVER_CWF_SOLVER_H_
