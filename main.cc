#include "gflags/gflags.h"
#include "glog/logging.h"
#include "problem.pb.h"
#include "problem_manager.h"
#include "problem_solver_factory.h"
#include "solution.pb.h"
#include "solution_manager.h"

DEFINE_string(problem_file, "",
              "Problem file (as specified in the original HashCode problem) "
              "path.");
DEFINE_string(solution_file, "",
              "Solution file (as specified in the original HashCode problem) "
              "path.");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  auto problem =
  //     drones::ProblemManager::GenerateProblem(drones::ProblemType());
       drones::ProblemManager::LoadFromProblemFile(FLAGS_problem_file);
  CHECK(problem != nullptr);

  auto solver = drones::ProblemSolverFactory::CreateSolver(*problem, "random");
  CHECK(solver != nullptr);

  drones::Solution best_solution;
  int best_score = -1;

  for (int num_iter = 0; num_iter < 100; num_iter++) {
    LOG(INFO) << "**************** ITER: " << num_iter;

    auto solution = solver->Solve();
    CHECK(solution != nullptr);

    LOG(INFO) << "Starting simulation";
    int score = -1;
    if (drones::SolutionManager::Simulate(*solution, &score)) {
      LOG(INFO) << "Solution is valid and gives " << score;
    } else {
      LOG(ERROR) << "Invalid solution.";
    }

    if (score > best_score) {
      best_score = score;
      best_solution = *solution;
    }
  }

  LOG(INFO) << "BEST SCORE: " << best_score;
  LOG(INFO) << "Saving the solution...";
  CHECK(drones::SolutionManager::SaveToSolutionFile(best_solution,
                                                    FLAGS_solution_file))
      << "Failed to save the solution.";

  return 0;
}
