#include "problem_manager.h"
#include "glog/logging.h"
#include "gtest/gtest.h"

// TODO(viktors): Consider creating a fixture because of the repeating init
// code.

namespace drones {

TEST(ProblemManagerTest, SaveLoadProblemFile) {
  auto problem = ProblemManager::GenerateProblem(ProblemType(), 12345);
  ASSERT_NE(problem, nullptr);
  ASSERT_TRUE(ProblemManager::SaveToProblemFile(*problem, "/tmp/problem.in"));
  auto loaded_problem = ProblemManager::LoadFromProblemFile("/tmp/problem.in");
  ASSERT_NE(loaded_problem, nullptr);
  EXPECT_EQ(problem->DebugString(), loaded_problem->DebugString());
}

TEST(ProblemManagerTest, SaveLoadProtoFile) {
  auto problem = ProblemManager::GenerateProblem(ProblemType(), 12345);
  ASSERT_NE(problem, nullptr);
  ASSERT_TRUE(ProblemManager::SaveToProtoFile(*problem, "/tmp/problem.proto"));
  auto loaded_problem = ProblemManager::LoadFromProtoFile("/tmp/problem.proto");
  ASSERT_NE(loaded_problem, nullptr);
  EXPECT_EQ(problem->DebugString(), loaded_problem->DebugString());
}

}  // namespace drones

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
