#ifndef SOLUTION_MANAGER_H_
#define SOLUTION_MANAGER_H_

#include "solution.pb.h"

#include <memory>
#include <string>
#include <utility>

namespace drones {

class SolutionManager {
 public:
  // Returns true if the solution is valid.
  static bool Simulate(const Solution& solution, int* score = nullptr);
  // Load the solution from a solution file, as specified in the Hashcode task,
  // and the given problem.
  static std::unique_ptr<Solution> LoadFromSolutionFile(
      const Problem& problem, const std::string& path);
  // Save the solution to a solution file, as specified in the Hashcode task.
  static bool SaveToSolutionFile(const Solution& solution,
                                 const std::string& path);
  // Saves the whole Solution proto to a file as a text-proto.
  static bool SaveToProtoFile(const Solution& solution,
                              const std::string& path);
  // Loads the whole Solution proto from a text-proto file.
  static std::unique_ptr<Solution> LoadFromProtoFile(const std::string& path);
};

};  // namespace drones

#endif  // SOLUTION_MANAGER_H_
