#ifndef SOLVERS_PROBLEM_SOLVER_FACTORY_H_
#define SOLVERS_PROBLEM_SOLVER_FACTORY_H_

#include <memory>
#include <string>

#include "solvers/problem_solver.h"

namespace drones {

class ProblemSolverFactory {
 public:
  // Supported solver types:
  //    - sp1:    Subproblem 1 optimal solver.
  //    - sp2:    Subproblem 2 optimal solver.
  //    - sp3:    Subproblem 3 solver (with or without TSP).
  //    - sp4:    Subproblem 4 solver with allocation.
  //    - sp5:    Subproblem 5 optimal but super slow ILP solver. Uses GLPK
  //              behind the OR-Tools solver frontend.
  //    - ilp:    Linearizing the problem, uses GLPK behind the OR-Tools solver
  //              frontend.
  //    - random: A totally random solver.
  //    - ga:     Genetic algorithm solver with strategies. Can be used as a
  //              random solver with strategies by setting # of generations to
  //              one.
  //    - cwf:    Closest warehouse  first.
  //    - ecf:    Earliest completion first(ish).
  static std::unique_ptr<ProblemSolver> CreateSolver(
      const Problem& problem, const std::string& solver_type);
};

}  // namespace drones

#endif  // SOLVERS_PROBLEM_SOLVER_FACTORY_H_
