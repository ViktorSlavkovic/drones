#include "absl/strings/substitute.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "problem.pb.h"
#include "problem_manager.h"
#include "problem_solver_factory.h"
#include "solution.pb.h"
#include "solution_manager.h"

DEFINE_int32(num_problems, 1,
             "The number of problems that should be "
             "generated and tested.");
DEFINE_int32(num_iter, 10, "The number of random solver passes.");

int main(int argc, char* argv[]) {
  FLAGS_max_coord = 100;
  FLAGS_max_ipo = 50;
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drones::ProblemType sp4_type;
  sp4_type.set_nd_1(true);
  sp4_type.set_m_m(true);

  for (int tp = 1; tp <= FLAGS_num_problems; tp++) {
    auto problem = drones::ProblemManager::GenerateProblem(sp4_type);
    CHECK(problem != nullptr);

    LOG(INFO) << absl::Substitute(
        "\nTP $0/$1: No = $2, Nw = $3, Np = $4, T = $5", tp, FLAGS_num_problems,
        problem->no(), problem->nw(), problem->np(), problem->t());

    auto random_solver =
        drones::ProblemSolverFactory::CreateSolver(*problem, "random");
    CHECK(random_solver != nullptr);

    {
      LOG(INFO) << "  Running random solver...";
      int best_score = -1;
      for (int num_iter = 0; num_iter < FLAGS_num_iter; num_iter++) {
        auto solution = random_solver->Solve();
        CHECK(solution != nullptr);
        int score = -1;
        CHECK(drones::SolutionManager::Simulate(*solution, &score))
            << "Invalid solution!";
        best_score = std::max(best_score, score);
      }
      LOG(INFO) << "    got: " << best_score;
    }

    auto sp4_solver =
        drones::ProblemSolverFactory::CreateSolver(*problem, "sp4");
    CHECK(sp4_solver != nullptr);

    LOG(INFO) << absl::Substitute(
        "\nTP $0/$1: No = $2, Nw = $3, Np = $4, T = $5", tp, FLAGS_num_problems,
        problem->no(), problem->nw(), problem->np(), problem->t());
    
    {
      LOG(INFO) << "  Running sp4 solver...";
      auto solution = sp4_solver->Solve();
      CHECK(solution != nullptr);
      int score = -1;
      CHECK(drones::SolutionManager::Simulate(*solution, &score))
          << "Invalid solution!";
      LOG(INFO) << "    got: " << score;
    }
  }
  return 0;
}
