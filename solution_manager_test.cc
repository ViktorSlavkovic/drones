#include "solution_manager.h"
#include "gtest/gtest.h"
#include "problem_manager.h"
#include "solvers/problem_solver_factory.h"

// TODO(viktors): Consider creating a fixture because of the repeating init
// code.

namespace drones {

TEST(SolutionManagerTest, SaveLoadSolutionFile) {
  auto problem = ProblemManager::GenerateProblem(ProblemType(), 12345);
  ASSERT_NE(problem, nullptr);
  auto solver = ProblemSolverFactory::CreateSolver(*problem, "random");
  ASSERT_NE(solver, nullptr);
  auto solution = solver->Solve();
  ASSERT_NE(solution, nullptr);
  ASSERT_TRUE(
      SolutionManager::SaveToSolutionFile(*solution, "/tmp/solution.out"));
  auto loaded_solution =
      SolutionManager::LoadFromSolutionFile(*problem, "/tmp/solution.out");
  ASSERT_NE(loaded_solution, nullptr);
  EXPECT_EQ(solution->DebugString(), loaded_solution->DebugString());
}

TEST(SolutionManagerTest, SaveLoadProtoFile) {
  auto problem = ProblemManager::GenerateProblem(ProblemType(), 12345);
  ASSERT_NE(problem, nullptr);
  auto solver = ProblemSolverFactory::CreateSolver(*problem, "random");
  ASSERT_NE(solver, nullptr);
  auto solution = solver->Solve();
  ASSERT_NE(solution, nullptr);
  ASSERT_TRUE(SolutionManager::SaveToProtoFile(*solution, "/tmp/solution.out"));
  auto loaded_solution =
      SolutionManager::LoadFromProtoFile("/tmp/solution.out");
  ASSERT_NE(loaded_solution, nullptr);
  EXPECT_EQ(solution->DebugString(), loaded_solution->DebugString());
}

}  // namespace drones

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
