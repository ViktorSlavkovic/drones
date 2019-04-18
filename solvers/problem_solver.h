#ifndef SOLVERS_PROBLEM_SOLVER_H_
#define SOLVERS_PROBLEM_SOLVER_H_

#include "problem.pb.h"
#include "solution.pb.h"
#include "glog/logging.h"

#include <memory>

namespace drones {

class ProblemSolver {
 public:
  explicit ProblemSolver(const Problem& problem) : problem_(problem) {}
  virtual std::unique_ptr<Solution> Solve() = 0;
  virtual bool CanSolve(const ProblemType& problem_type) const = 0;
  virtual ~ProblemSolver() {}
 protected:
  const Problem problem_;
};

}  // namespace drones

#endif  // SOLVERS_PROBLEM_SOLVER_H_
