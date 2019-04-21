#include "solvers/util/load_splitter.h"

#include <algorithm>
#include <map>
#include <numeric>
#include <vector>

#include "glog/logging.h"

namespace drones {
namespace util {

LoadSplitter::RepeatedSplit LoadSplitter::SplitOnce(
    const std::map<int, int>& stock, const std::map<int, int>& volumes,
    int total_volume) {
  // Best volume usage at each volume.
  static std::vector<int> knapsack_best;
  // [volume][product] -> num items
  static std::vector<std::map<int, int>> knapsack_taken;
  // Number of taken items (2 apples + 3 oranges = 5 items) at each volumes.
  static std::vector<int> knapsack_taken_items;

  knapsack_best.resize(total_volume + 1);
  knapsack_best[0] = 0;
  knapsack_taken.resize(total_volume + 1);
  knapsack_taken[0].clear();
  knapsack_taken_items.resize(total_volume + 1);
  knapsack_taken_items[0] = 0;

  for (int vol = 1; vol <= total_volume; vol++) {
    knapsack_best[vol] = 0;
    knapsack_taken[vol].clear();
    knapsack_taken_items[vol] = 0;
    for (const auto& pi : stock) {
      int p = pi.first;
      int cap = pi.second;
      int pv = volumes.at(p);
      if (vol < pv) continue;
      // Let's try to add one item of p to volume - pv case.
      if ((pv + knapsack_best[vol - pv] > knapsack_best[vol] ||
           (pv + knapsack_best[vol - pv] == knapsack_best[vol] &&
            knapsack_taken[vol - pv].size() + 1 -
                    knapsack_taken[vol - pv].count(p) <
                knapsack_taken[vol].size())) &&
          knapsack_taken[vol - pv][p] + 1 <= cap) {
        // Ouch! Expensive!
        knapsack_taken[vol] = knapsack_taken[vol - pv];
        knapsack_taken[vol][p]++;
        knapsack_taken_items[vol] = knapsack_taken_items[vol - pv] + 1;
        knapsack_best[vol] = knapsack_best[vol - pv] + pv;
        continue;
      }
      // If not, let's see if original volume - pv case is still better
      // then what we have now.
      if (knapsack_best[vol - pv] > knapsack_best[vol] ||
          (knapsack_best[vol - pv] == knapsack_best[vol] &&
           knapsack_taken[vol].size() > knapsack_taken[vol - pv].size())) {
        // Ouch! Expensive!
        knapsack_taken[vol] = knapsack_taken[vol - pv];
        knapsack_taken_items[vol] = knapsack_taken_items[vol - pv];
        knapsack_best[vol] = knapsack_best[vol - pv];
      }
    }
  }

  // Make compact.
  for (auto it = knapsack_taken[total_volume].cbegin();
       it != knapsack_taken[total_volume].cend();) {
    if (it->second == 0) {
      knapsack_taken[total_volume].erase(it++);
    } else {
      it++;
    }
  }

  // Calculate how many times we can do the same.
  int num_times = std::numeric_limits<int>::max();
  for (const auto& pi : knapsack_taken[total_volume]) {
    int max_times = stock.at(pi.first) / pi.second;
    num_times = std::min(num_times, max_times);
  }
  CHECK(num_times > 0);

  return {.num_items = knapsack_taken_items[total_volume],
          .times = num_times,
          .single_split = knapsack_taken[total_volume]};
}

LoadSplitter::CompleteSplit LoadSplitter::Split(
    std::map<int, int> stock, const std::map<int, int>& volumes,
    int total_volume) {
  // Make sure that the stock is "compact" - no products listed which are out
  // of stock.
  for (auto cit = stock.cbegin(); cit != stock.cend();) {
    if (cit->second == 0) {
      cit = stock.erase(cit);
    } else {
      cit++;
    }
  }

  CompleteSplit res{.total_num_items = 0, .total_times = 0};

  while (!stock.empty()) {
    // Knapsack-pack to fill as much volume as possible.
    auto repeated_split = SplitOnce(stock, volumes, total_volume);

    // Remove from the stock.
    for (const auto& pi : repeated_split.single_split) {
      if ((stock[pi.first] -= pi.second * repeated_split.times) == 0) {
        stock.erase(pi.first);
      }
    }

    // Add to the result.
    res.repeated_splits.push_back(repeated_split);
    res.total_num_items += repeated_split.times * repeated_split.num_items;
    res.total_times += repeated_split.times;
  }

  return res;
}

}  // namespace util
}  // namespace drones
