#include "solution_manager.h"
#include "gtest/gtest.h"

namespace drones {

TEST(SolutionManagerTest, DummyTest) { EXPECT_EQ(1, 1); }

}  // namespace drones

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
