#include "absl/strings/substitute.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "problem.pb.h"
#include "problem_manager.h"
#include "problem_solver_factory.h"
#include "solution.pb.h"
#include "solution_manager.h"

DEFINE_int32(num_problems, 10,
             "The number of problems that should be "
             "generated and tested.");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drones::ProblemType sp1_type;
  sp1_type.set_nd_1(true);
  sp1_type.set_m_m(true);
  sp1_type.set_nw_1(true);
  sp1_type.set_s0_inf(true);
  sp1_type.set_np_1(true);
  sp1_type.set_ipo_1(true);

  for (int tp = 1; tp <= FLAGS_num_problems; tp++) {
    auto problem = drones::ProblemManager::GenerateProblem(sp1_type);
    CHECK(problem != nullptr);

    LOG(INFO) << absl::Substitute("\nTP $0/$1: No = $2, T = $3", tp,
                                  FLAGS_num_problems, problem->no(),
                                  problem->t());

    auto random_solver =
        drones::ProblemSolverFactory::CreateSolver(*problem, "random");
    CHECK(random_solver != nullptr);

    {
      LOG(INFO) << "  Running random solver...";
      int best_score = -1;
      for (int num_iter = 0; num_iter < 100; num_iter++) {
        auto solution = random_solver->Solve();
        CHECK(solution != nullptr);
        int score = -1;
        CHECK(drones::SolutionManager::Simulate(*solution, &score))
            << "Invalid solution!";
        best_score = std::max(best_score, score);
      }
      LOG(INFO) << "    got: " << best_score;
    }

    auto sp1_solver =
        drones::ProblemSolverFactory::CreateSolver(*problem, "sp1");
    CHECK(sp1_solver != nullptr);

    {
      LOG(INFO) << "  Running sp1 solver...";
      auto solution = sp1_solver->Solve();
      CHECK(solution != nullptr);
      int score = -1;
      CHECK(drones::SolutionManager::Simulate(*solution, &score))
          << "Invalid solution!";
      LOG(INFO) << "    got: " << score;
    }
  }
  return 0;
}
