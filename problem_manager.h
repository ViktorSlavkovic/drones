#ifndef PROBLEM_MANAGER_H_
#define PROBLEM_MANAGER_H_

#include <memory>
#include <random>
#include <string>

#include "problem.pb.h"
#include "gflags/gflags.h"

DECLARE_int32(max_ipo);
DECLARE_int32(max_coord);
DECLARE_int32(max_nd);
DECLARE_int32(max_nw);
DECLARE_int32(max_np);
DECLARE_int32(max_no);
DECLARE_int32(max_M);

namespace drones {

class ProblemManager {
 public:
  // Generates a problem of the given type.
  static std::unique_ptr<Problem> GenerateProblem(
      const ProblemType& problem_type, unsigned int seed);
  // Same as above, just using the std::random_device for seeding.
  static std::unique_ptr<Problem> GenerateProblem(const ProblemType& problem_type) {
    std::random_device rand_dev;
    return GenerateProblem(problem_type, rand_dev());
  }
  // Load the problem from a problem file, as specified in the Hashcode task.
  static std::unique_ptr<Problem> LoadFromProblemFile(const std::string& path);
  // Save the problem to a problem file, as specified in the Hashcode task.
  static bool SaveToProblemFile(const Problem& problem,
                                const std::string& path);
  // Loads the Problem proto from a serialized-proto file.
  static std::unique_ptr<Problem> LoadFromProtoFile(const std::string& path);
  // Saves the Problem proto to a serizalized-proto file.
  static bool SaveToProtoFile(const Problem& problem, const std::string& path);

 private:
  static ProblemType DetermineProblemType(const Problem& problem);
};

}  // namespace drones

#endif  // PROBLEM_MANAGER_H_
