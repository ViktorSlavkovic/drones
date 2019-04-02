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
  FLAGS_max_ipo = 50;
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drones::ProblemType sp1_type;
  sp1_type.set_nd_1(true);
  sp1_type.set_m_m(true);
  sp1_type.set_np_1(true);
  sp1_type.set_no_1(true);
  for (int tp = 1; tp <= FLAGS_num_problems; tp++) {
    auto problem = drones::ProblemManager::GenerateProblem(sp1_type);
    CHECK(problem != nullptr);

    LOG(INFO) << absl::Substitute("\nTP $0/$1: Nw = $2, Req=$3, T = $4", tp,
                                  FLAGS_num_problems, problem->nw(),
                                  problem->order(0).request(0),
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

    auto sp2_solver =
        drones::ProblemSolverFactory::CreateSolver(*problem, "sp2");
    CHECK(sp2_solver != nullptr);

    {
      LOG(INFO) << "  Running sp2 solver...";
      auto solution = sp2_solver->Solve();
      CHECK(solution != nullptr);
      int score = -1;
      CHECK(drones::SolutionManager::Simulate(*solution, &score))
          << "Invalid solution!";
      LOG(INFO) << "    got: " << score;
    }
  }
  return 0;
}
