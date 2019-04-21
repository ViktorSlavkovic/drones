#ifndef SOLVERS_UTIL_LOAD_SPLITTER_H_
#define SOLVERS_UTIL_LOAD_SPLITTER_H_

#include <list>
#include <map>

namespace drones {
namespace util {

class LoadSplitter {
 public:
  // Describes one drone packing, maps products to numbers of ther items taken.
  using SingleSplit = std::map<int, int>;
  struct RepeatedSplit {
    // Num items in one turn.
    int num_items;
    // How many times the turn can be repeated.
    int times;
    SingleSplit single_split;
  };
  struct CompleteSplit {
    int total_num_items;
    int total_times;
    std::list<RepeatedSplit> repeated_splits;
  };

  // Pack one drone capacity from the given stock. Repeated split is returned
  // in order to export information about how many items are taken per turn and
  // how many times the same packing can be done. Returned split is compact (
  // times >= 1 and single_split has only >= 1 map values).
  // Assuming that the stock is "compact" - no products listed which are out of
  // stock.
  static RepeatedSplit SplitOnce(const std::map<int, int>& stock,
                                 const std::map<int, int>& volumes,
                                 int total_volume);

  // Split one (warehouse) stock to multiple drone turns.
  // No assumptions are made about whether the stock is "compact" (see
  // SplitOnce) or not. Returned split is compact (all repeated splits have
  // times >= 1 all single splits have only >= 1 map values).
  static CompleteSplit Split(std::map<int, int> stock,
                             const std::map<int, int>& volumes,
                             int total_volume);
};

}  // namespace util
}  // namespace drones

#endif  // SOLVERS_UTIL_LOAD_SPLITTER_H_
