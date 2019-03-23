#include "problem_solver_factory.h"

#include "glog/logging.h"
#include "glop_problem_solver.h"

namespace drones {

std::unique_ptr<ProblemSolver> ProblemSolverFactory::CreateSolver(
    const Problem& problem, const std::string& solver_type) {
  if (solver_type == "glop") {
    return std::make_unique<GlopProblemSolver>(problem);
  }
  LOG(ERROR) << "Invalid solver type: " << solver_type;
  return nullptr;
}

}  // namespace drones
