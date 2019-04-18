#ifndef SOLVERS_UTIL_ALLOCATOR_H_
#define SOLVERS_UTIL_ALLOCATOR_H_

#include <vector>
#include <map>
#include <utility>

#include "problem.pb.h"

namespace drones {
namespace util {

class Allocator {
 public:
  // [order][{warehouse,product}] -> num_items
  using Alloc = std::vector<std::map<std::pair<int, int>, int>>;

  static Alloc Allocate(const Problem& problem);

  static bool VerifyAlloc(const Problem& problem, const Alloc& alloc);
};

}  // namespace util
}  // namespace drones

#endif  // SOLVERS_UTIL_ALLOCATOR_H_
