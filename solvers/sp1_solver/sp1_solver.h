#ifndef SOLVERS_SP1_SOLVER_SP1_SOLVER_H_
#define SOLVERS_SP1_SOLVER_SP1_SOLVER_H_

#include <memory>

#include "solvers/problem_solver.h"

namespace drones {

// Solves subproblem 1: Nd=1, M=m, Nw=1, S0=inf, Np=1, IPO=1
class Sp1Solver : public ProblemSolver {
 public:
  explicit Sp1Solver(const Problem& problem) : ProblemSolver(problem) {}
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override;
};

}  // namespace drones

#endif  // SOLVERS_SP1_SOLVER_SP1_SOLVER_H_
