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
  struct Feedback {
    // num items taken from closest warehouse to the order, turns taken for it
    std::pair<int, int> cw_o;
    // [w] -> {num items taken from w to the order directly, turns taken for it}
    std::map<int, std::pair<int, int>> w_o;
    // [w] -> {num items taken from w to closest warehouse }
    std::map<int, std::pair<int, int>> w_cw;
  };

  std::vector<int> closest_w_;
  std::map<int, int> product_weights_;
  util::Allocator::Alloc alloc_;

  mutable std::map<int, Feedback> order_feedback_;

  std::unique_ptr<Solution> PackSolution(
      const std::vector<int>& order_permutation, int* score = nullptr,
      bool enable_log = true) const;
  std::vector<int> ImporveOrderPermutation(std::vector<int> order_permutation,
                                           int num_iter) const;
  void IterativeAlloc(int num_iter);
  void CalcClosestWarehouses();
  double EstimateOrderDuration(int o) const;
  std::vector<int> GenerateOrderPermutation() const;
};

}  // namespace drones

#endif  // SOLVERS_CWF_SOLVER_CWF_SOLVER_H_
