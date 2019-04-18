#ifndef SOLVERS_SP4_SOLVER_SP4_SOLVER_H_
#define SOLVERS_SP4_SOLVER_SP4_SOLVER_H_

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "solvers/problem_solver.h"

namespace drones {

// Solves subproblem 4: Nd=1, M=m
class Sp4Solver : public ProblemSolver {
 public:
  explicit Sp4Solver(const Problem& problem) : ProblemSolver(problem) {}
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override;
};

}  // namespace drones

#endif  // SOLVERS_SP4_SOLVER_SP4_SOLVER_H_
