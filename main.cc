#include "absl/strings/substitute.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "google/protobuf/text_format.h"
#include "problem.pb.h"
#include "problem_manager.h"
#include "solution.pb.h"
#include "solution_manager.h"
#include "solvers/problem_solver_factory.h"
#include "solvers/upper_bound/upper_bound.h"
#include "solvers/upper_bound_higher/upper_bound_higher.h"

#include <iostream>
#include <memory>

DEFINE_string(
    problem_file, "",
    "Problem file (HashCode format) path. Problem will be read from this file "
    "unless gen_problem flag is true.");
DEFINE_bool(
    read_solution, false,
    "If true, the solution will be read from solution file specified by "
    "solution_file flag.");
DEFINE_string(
    solution_file, "",
    "Solution file (HashCode format) path. If read_solution flag is set, the "
    "solution will be read from this path, otherwise, the solution will be "
    "generated and stored to this path.");
DEFINE_bool(
    gen_problem, false,
    "If true, the problem will be generated. Use gen_problem_type flag to "
    "specify problem type.");
DEFINE_bool(
    write_gen_problem, true,
    "If true, and gen_problem is true, the genereated problem will be written "
    "to the file specified by problem_file flag.");
DEFINE_bool(
    gen_only, false,
    "If true and gen_problem is true, the program will exit without attempting "
    "to solve it. Use this to generate problem only.");
DEFINE_string(
    gen_problem_type, "{}",
    "Type of the problem being generated (ProblemType proto) as a text proto.");
DEFINE_string(solver_type, "random",
              "The solver type (see ProblemSolverFactory).");
DEFINE_bool(
    check, true,
    "If true, the solution will be checked - simulated, validated and scored.");
DEFINE_bool(
    inf_loop_end, false,
    "If true, main() won't return. Instead, it will loop indefinitely.");

std::unique_ptr<drones::Problem> get_problem() {
  if (FLAGS_gen_problem) {
    drones::ProblemType problem_type;
    CHECK(google::protobuf::TextFormat::ParseFromString(FLAGS_gen_problem_type,
                                                        &problem_type))
        << "Unable to parse " << FLAGS_gen_problem_type << " as ProblemType.";
    return drones::ProblemManager::GenerateProblem(problem_type);
  }
  return drones::ProblemManager::LoadFromProblemFile(FLAGS_problem_file);
}

std::unique_ptr<drones::Solution> get_solution(const drones::Problem& problem) {
  if (FLAGS_read_solution) {
    return drones::SolutionManager::LoadFromSolutionFile(problem,
                                                         FLAGS_solution_file);
  }
  auto solver =
      drones::ProblemSolverFactory::CreateSolver(problem, FLAGS_solver_type);
  CHECK(solver != nullptr) << "Failed to create the solver object.";
  return solver->Solve();
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  auto problem = get_problem();
  CHECK(problem != nullptr) << "Failed to create the problem object.";

  LOG(INFO) << absl::Substitute(R"(
    Nd = $0
    Np = $1
    Nw = $2
    No = $3
    T  = $4
    ProblemType: $5
  )",
                                problem->nd(), problem->np(), problem->nw(),
                                problem->no(), problem->t(),
                                problem->problem_type().DebugString());

  if (FLAGS_gen_problem) {
    if (FLAGS_write_gen_problem) {
      CHECK(drones::ProblemManager::SaveToProblemFile(*problem,
                                                      FLAGS_problem_file))
          << "Failed to save problem to: " << FLAGS_problem_file;
    }
    if (FLAGS_gen_only) {
      return 0;
    }
  }

  auto start_time = absl::Now();
  auto solution = get_solution(*problem);
  CHECK(solution != nullptr) << "Failed to create the solution object.";
  LOG(INFO) << absl::Substitute("Solver finished in: $0",
                                absl::FormatDuration(absl::Now() - start_time));

  if (FLAGS_check) {
    int score = -1;
    CHECK(drones::SolutionManager::Simulate(*solution, &score))
        << "Invalid solution!";
    std::cout << "TOTAL SCORE: " << score << std::endl;
    LOG(INFO) << "Calculating the upper bound estimate...";
    int upper_bound = drones::UpperBoundHigher(*problem).Calc();
    LOG(INFO) << absl::Substitute(
        "\nTotal Score: $0\nUpper Bound: $1\nQuality: $2 %", score, upper_bound,
        100.0 * score / upper_bound);
  }

  if (!FLAGS_read_solution && !FLAGS_solution_file.empty()) {
    CHECK(drones::SolutionManager::SaveToSolutionFile(*solution,
                                                      FLAGS_solution_file))
        << "Failed to save solution to: " << FLAGS_solution_file;
  }

  while (FLAGS_inf_loop_end) {
  }

  return 0;
}
