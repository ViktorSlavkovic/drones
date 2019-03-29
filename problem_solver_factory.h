#ifndef PROBLEM_SOLVER_FACTORY_H_
#define PROBLEM_SOLVER_FACTORY_H_

#include "problem_solver.h"

#include <memory>
#include <string>

namespace drones {

class ProblemSolverFactory {
 public:
  // Supported solver types:
  //    - lp: Uses Google's GLOP, a linear programming solver.
  static std::unique_ptr<ProblemSolver> CreateSolver(
      const Problem& problem, const std::string& solver_type);
};

}  // namespace drones

#endif  // PROBLEM_SOLVER_FACTORY_H_
