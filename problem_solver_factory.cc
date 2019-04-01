#include "problem_solver_factory.h"

#include "glog/logging.h"
#include "lp_solver.h"
#include "random_solver.h"

namespace drones {

std::unique_ptr<ProblemSolver> ProblemSolverFactory::CreateSolver(
    const Problem& problem, const std::string& solver_type) {
  std::unique_ptr<ProblemSolver> solver = nullptr;
  if (solver_type == "lp") {
    solver = std::make_unique<LpSolver>(problem);
  } else if (solver_type == "random") {
    solver = std::make_unique<RandomSolver>(problem);
  } else {
    LOG(ERROR) << "Invalid solver type: " << solver_type;
    return nullptr;
  }

  if (!solver->CanSolve(problem.problem_type())) {
    LOG(ERROR) << "The solver type is incompatible with the problem.";
    return nullptr;
  }

  return solver;
}

}  // namespace drones
