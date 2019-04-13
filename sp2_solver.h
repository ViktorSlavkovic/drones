#ifndef SP2_SOLVER_H_
#define SP2_SOLVER_H_

#include <memory>

#include "problem_solver.h"

namespace drones {

// Solves subproblem 2: Nd=1, M=m, Np=1, No=1
class Sp2Solver: public ProblemSolver {
 public:
  explicit Sp2Solver(const Problem& problem): ProblemSolver(problem) {}
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override;
};

}  // namespace drones

#endif  // SP2_SOLVER_H_