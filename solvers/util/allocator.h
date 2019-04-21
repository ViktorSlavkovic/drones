#ifndef SOLVERS_UTIL_ALLOCATOR_H_
#define SOLVERS_UTIL_ALLOCATOR_H_

#include <functional>
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
  using DistFn = std::function<double(int /* order */, int /* warehouse */,
                                      int /* product */)>;

  // Determines for each order, how many items of each requestd product will
  // be served from which warehouse.
  static Alloc AllocateWithDistFn(const Problem& problem,
                                  const DistFn& dist_fn);

  static bool VerifyAlloc(const Problem& problem, const Alloc& alloc);
};

}  // namespace util
}  // namespace drones

#endif  // SOLVERS_UTIL_ALLOCATOR_H_
