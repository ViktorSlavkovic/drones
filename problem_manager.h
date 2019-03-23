#ifndef PROBLEM_MANAGER_H_
#define PROBLEM_MANAGER_H_

#include <memory>
#include <string>

#include "problem.pb.h"

namespace drones {

class ProblemManager {
 public:
  static std::unique_ptr<Problem> LoadProblemFromFile(const std::string &path);
  static std::unique_ptr<Problem> GenerateProblem();
};

}  // namespace drones

#endif  // PROBLEM_MANAGER_H_
