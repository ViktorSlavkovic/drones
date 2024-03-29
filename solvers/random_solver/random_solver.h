#ifndef SOLVERS_RANDOM_SOLVER_RANDOM_SOLVER_H_
#define SOLVERS_RANDOM_SOLVER_RANDOM_SOLVER_H_

#include <memory>

#include "problem.pb.h"
#include "solution.pb.h"
#include "solvers/problem_solver.h"

namespace drones {

class RandomSolver : public ProblemSolver {
 public:
  explicit RandomSolver(const Problem& problem);
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override { return true; }
};

}  // namespace drones

#endif  // SOLVERS_RANDOM_SOLVER_RANDOM_SOLVER_H_
