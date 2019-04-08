#ifndef ECF_SOLVER_H_
#define ECF_SOLVER_H_

#include <memory>
#include <map>
#include <utility>
#include <vector>

#include "problem_solver.h"

namespace drones {

// Solves the original problem.
class EcfSolver: public ProblemSolver {
 public:
  explicit EcfSolver(const Problem& problem): ProblemSolver(problem) {}
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override;
 private:
  // [order][{warehouse,product}] -> num_items
  using Alloc = std::vector<std::map<std::pair<int, int>, int>>;
  Alloc Allocate() const;
  bool VerifyAlloc(const Alloc& alloc) const; 
};

}  // namespace drones

#endif  // ECF_SOLVER_H_

