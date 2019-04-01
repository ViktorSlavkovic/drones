#ifndef RANDOM_SOLVER_H_
#define RANDOM_SOLVER_H_

#include <memory>

#include "problem.pb.h"
#include "problem_solver.h"
#include "solution.pb.h"

namespace drones {

class RandomSolver : public ProblemSolver {
 public:
  explicit RandomSolver(const Problem& problem);
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override { return true; }
};

}  // namespace drones

#endif  // RANDOM_SOLVER_H_
