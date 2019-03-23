#include "gflags/gflags.h"
#include "glog/logging.h"
#include "problem.pb.h"
#include "problem_manager.h"
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
      drones::ProblemManager::LoadProblemFromFile(FLAGS_problem_file);
  CHECK(problem != nullptr);

  auto solution = drones::SolutionManager::LoadFromSolutionFile(
      *problem, FLAGS_solution_file);
  CHECK(solution != nullptr);

  int score = -1;
  if (drones::SolutionManager::Simulate(*solution, &score)) {
    LOG(INFO) << "Solution is valid and gives " << score;
  } else {
    LOG(ERROR) << "Invalid solution.";
  }
  return 0;
}
