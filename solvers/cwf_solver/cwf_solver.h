#ifndef SOLVERS_CWF_SOLVER_CWF_SOLVER_H_
#define SOLVERS_CWF_SOLVER_CWF_SOLVER_H_

#include <map>
#include <memory>
#include <vector>

#include "solvers/problem_solver.h"
#include "solvers/util/allocator.h"

namespace drones {

class CwfSolver : public ProblemSolver {
 public:
  explicit CwfSolver(const Problem& problem);
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override { return true; }

 protected:
  std::vector<int> closest_w_;
  std::map<int, int> product_weights_;
  void CalcClosestWarehouses();

  double EstimateOrderDuration(const util::Allocator::Alloc& alloc,
                               int o) const;

  std::vector<int> GenerateOrderPermutation(const util::Allocator::Alloc& alloc) const;
};

}  // namespace drones

#endif  // SOLVERS_CWF_SOLVER_CWF_SOLVER_H_
