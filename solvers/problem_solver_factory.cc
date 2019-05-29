#include "solvers/problem_solver_factory.h"

#include "glog/logging.h"
#include "solvers/cwf_solver/cwf_solver.h"
#include "solvers/ecf_solver/ecf_solver.h"
#include "solvers/lp_solver/lp_solver.h"
#include "solvers/random_solver/random_solver.h"
#include "solvers/sp1_solver/sp1_solver.h"
#include "solvers/sp2_solver/sp2_solver.h"
#include "solvers/sp4_solver/sp4_solver.h"
#include "solvers/ga_solver/ga_solver.h"

namespace drones {

std::unique_ptr<ProblemSolver> ProblemSolverFactory::CreateSolver(
    const Problem& problem, const std::string& solver_type) {
  std::unique_ptr<ProblemSolver> solver = nullptr;
  if (solver_type == "lp") {
    solver = std::make_unique<LpSolver>(problem);
  } else if (solver_type == "random") {
    solver = std::make_unique<RandomSolver>(problem);
  } else if (solver_type == "sp1") {
    solver = std::make_unique<Sp1Solver>(problem);
  } else if (solver_type == "sp2") {
    solver = std::make_unique<Sp2Solver>(problem);
  } else if (solver_type == "sp4") {
    solver = std::make_unique<Sp4Solver>(problem);
  } else if (solver_type == "ecf") {
    solver = std::make_unique<EcfSolver>(problem);
  } else if (solver_type == "cwf") {
    solver = std::make_unique<CwfSolver>(problem);
  } else if (solver_type == "ga") {
    solver = std::make_unique<GaSolver>(problem);
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
