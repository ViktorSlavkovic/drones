#ifndef SOLVERS_ECF_SOLVER_ECF_SOLVER_H_
#define SOLVERS_ECF_SOLVER_ECF_SOLVER_H_

#include <map>
#include <memory>
#include <vector>

#include "solvers/problem_solver.h"
#include "solvers/util/load_splitter.h"

namespace drones {

// Solves the original problem.
class EcfSolver : public ProblemSolver {
 public:
  explicit EcfSolver(const Problem& problem) : ProblemSolver(problem) {}
  std::unique_ptr<Solution> Solve() override;
  bool CanSolve(const ProblemType& problem_type) const override;

 private:
  using OrderSplits =
      std::vector<std::map<int, util::LoadSplitter::CompleteSplit>>;

  OrderSplits IterativeSplit(int num_iter) const;

  std::vector<int> GenereteOrderPermutation(
      const OrderSplits& order_splits) const;

  std::unique_ptr<Solution> PackSolution(
      OrderSplits order_splits, const std::vector<int>& order_permutation,
      int* score = nullptr,
      bool enable_log = true) const;

  std::vector<int> ImporveOrderPermutation(const OrderSplits& order_splits,
                                           std::vector<int> order_permutation,
                                           int num_iter) const;
 
};

}  // namespace drones

#endif  // SOLVERS_ECF_SOLVER_ECF_SOLVER_H_
