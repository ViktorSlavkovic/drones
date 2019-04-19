#ifndef SOLVERS_UTIL_ALLOCATOR_H_
#define SOLVERS_UTIL_ALLOCATOR_H_

#include <map>
#include <utility>
#include <vector>

#include "problem.pb.h"

namespace drones {
namespace util {

class Allocator {
 public:
  // [order][{warehouse,product}] -> num_items
  using Alloc = std::vector<std::map<std::pair<int, int>, int>>;
  // [order][warehouse] -> coefficient
  using Feedback = std::map<int, std::map<int, double>>;

  static Alloc Allocate(const Problem& problem,
                        Feedback* feedback = nullptr);

  static bool VerifyAlloc(const Problem& problem, const Alloc& alloc);
};

}  // namespace util
}  // namespace drones

#endif  // SOLVERS_UTIL_ALLOCATOR_H_
