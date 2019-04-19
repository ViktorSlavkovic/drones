#ifndef SOLVERS_UTIL_LOAD_SPLITTER_H_
#define SOLVERS_UTIL_LOAD_SPLITTER_H_

#include <list>
#include <map>

namespace drones {
namespace util {

class LoadSplitter {
 public:
  struct CompleteSplit {
    struct RepeatedSplit {
      int num_items;
      int times;
      std::map<int, int> single_split;
    };
    int total_num_items;
    int total_times;
    std::list<RepeatedSplit> repeated_splits;
  };

  static CompleteSplit Split(std::map<int, int> stock,
                             const std::map<int, int>& volumes,
                             int total_volume);
};

}  // namespace util
}  // namespace drones

#endif  // SOLVERS_UTIL_LOAD_SPLITTER_H_
