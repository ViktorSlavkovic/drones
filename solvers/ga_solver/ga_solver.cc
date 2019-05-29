#include "solvers/ga_solver/ga_solver.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <set>
#include <vector>

#include "absl/strings/numbers.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"
#include "absl/strings/substitute.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "solution.pb.h"
#include "solvers/util/allocator.h"
#include "solvers/util/load_splitter.h"

DEFINE_string(ga_forbidden_strategies, "",
              "Indices of the forbidden strategies as CSV (e.g. 0,1,5).");
DEFINE_int32(ga_strategies_per_drone, 1000,
             "The number of strategies choosen for each drone.");
DEFINE_int32(ga_strategy_repeats, 1,
             "Maximum number of consecutive attempts of a single strategy (by "
             "each drone, not in total for all drones). If a drone fails to "
             "execute a strategy, it will attempt the nest strategy in its "
             "queue next time.");
DEFINE_int32(ga_population_size, 100, "The population size.");
DEFINE_int32(ga_num_generations, 10, "The number of generations.");
DEFINE_int32(
    ga_selection_size, 50,
    "How many individuals are kept from each generation's population "
    "in a selection. The sum: ga_selection_size + ga_crossing_over_size + "
    "ga_mutation_size should match ga_population_size.");
DEFINE_int32(ga_max_twists, 5,
             "Maximum number of crossings (twists) in a crossingover.");
DEFINE_int32(ga_crossingover_size, 30,
             "How many individuals are added in the crossingover phase. Should "
             "be even!. The sum: ga_selection_size + ga_crossing_over_size + "
             "ga_mutation_size should match ga_population_size.");
DEFINE_double(ga_mutation_prob, 0.05,
              "The probability of a single bit flip in a mutation.");
DEFINE_int32(ga_mutation_size, 20,
             "How many individuals are added in the mutation phase. The "
             "sum: ga_selection_size + ga_crossing_over_size + "
             "ga_mutation_size should match ga_population_size.");
DEFINE_bool(ga_mutate_all_drones, false,
            "If true, all drones will be mutated. Otherwise, only one, "
            "randomly chosen, drone will be mutated.");

namespace drones {
GaSolver::GaSolver(const Problem& problem)
    : ProblemSolver(problem),
      closest_ws_(CalcClosestWarehouses(problem)),
      product_weights_(PrepareProductWeights(problem)),
      alloc_(AllocateOrDie(problem)),
      strategy_(FilterStrategies(this)),
      random_engine_(std::random_device()()),
      drone_location_(),
      drone_busy_until_(),
      pending_warehouses_(),
      pending_orders_(),
      pending_orders_weight_(),
      transactions_(),
      solution_(nullptr),
      latest_delivery_time_(),
      score_(0) {}

std::vector<std::vector<int>> GaSolver::CalcClosestWarehouses(
    const Problem& problem) {
  std::vector<std::vector<int>> closest_ws;
  std::vector<int> perm(problem.nw());
  for (int w = 0; w < problem.nw(); w++) {
    perm[w] = w;
  }
  for (int o = 0; o < problem.no(); o++) {
    std::sort(perm.begin(), perm.end(), [&](int w1, int w2) {
      return problem.dist().src(w1).dst(problem.nw() + o) <
             problem.dist().src(w2).dst(problem.nw() + o);
    });
    closest_ws.push_back(perm);
  }
  return closest_ws;
}

std::map<int, int> GaSolver::PrepareProductWeights(const Problem& problem) {
  std::map<int, int> product_weights;
  for (int p = 0; p < problem.np(); p++) {
    product_weights[p] = problem.product(p).m();
  }
  return product_weights;
}

util::Allocator::Alloc GaSolver::AllocateOrDie(const Problem& problem) {
  util::Allocator::DistFn dist_fn = [&](int o, int w, int p) {
    return 2.0 * problem.dist().src(w).dst(problem.nw() + o);
  };
  auto alloc = util::Allocator::AllocateWithDistFn(problem, dist_fn);
  CHECK(util::Allocator::VerifyAlloc(problem, alloc)) << "Invalid alloc!";
  LOG(INFO) << "Successfully allocated!";
  return alloc;
}

std::vector<std::function<bool(int)>> GaSolver::FilterStrategies(
    GaSolver* ga_solver) {
  std::set<std::string> split = absl::StrSplit(FLAGS_ga_forbidden_strategies,
                                               ",", absl::SkipWhitespace());
  std::set<int> forbidden_strategies;
  for (const std::string& s : split) {
    int strategy;
    CHECK(absl::SimpleAtoi(s, &strategy)) << "Can't parse: " << s;
    forbidden_strategies.insert(strategy);
  }

  std::vector<std::function<bool(int)>> res;
  if (forbidden_strategies.count(0) == 0) {
    res.push_back([=](int d) { return ga_solver->Strategy0(d); });
  }
  if (forbidden_strategies.count(1) == 0) {
    res.push_back([=](int d) { return ga_solver->Strategy1(d); });
  }
  if (forbidden_strategies.count(2) == 0) {
    res.push_back([=](int d) { return ga_solver->Strategy2(d); });
  }
  if (forbidden_strategies.count(3) == 0) {
    res.push_back([=](int d) { return ga_solver->Strategy3(d); });
  }
  if (forbidden_strategies.count(4) == 0) {
    res.push_back([=](int d) { return ga_solver->Strategy4(d); });
  }
  if (forbidden_strategies.count(5) == 0) {
    res.push_back([=](int d) { return ga_solver->Strategy5(d); });
  }
  if (forbidden_strategies.count(6) == 0) {
    res.push_back([=](int d) { return ga_solver->Strategy6(d); });
  }
  if (forbidden_strategies.count(7) == 0) {
    res.push_back([=](int d) { return ga_solver->Strategy7(d); });
  }
  return res;
}

void GaSolver::InitSolution() {
  drone_location_.clear();
  drone_location_.resize(problem_.nd(), 0);
  drone_busy_until_.clear();
  drone_busy_until_.resize(problem_.nd(), 0);
  pending_warehouses_.clear();
  pending_orders_.clear();
  for (int o = 0; o < problem_.no(); o++) {
    for (const auto& wp_i : alloc_[o]) {
      if (wp_i.second > 0) {
        pending_warehouses_[wp_i.first.first][o][-1][wp_i.first.second] +=
            wp_i.second;
        pending_orders_[o][wp_i.first.first][wp_i.first.second] += wp_i.second;
      }
    }
  }
  pending_orders_weight_.clear();
  pending_orders_weight_.resize(problem_.no(), 0);
  for (int o = 0; o < problem_.no(); o++) {
    for (int p = 0; p < problem_.np(); p++) {
      pending_orders_weight_[o] +=
          problem_.product(p).m() * problem_.order(o).request(p);
    }
  }
  transactions_.clear();
  solution_ = std::make_unique<Solution>();
  *solution_->mutable_problem() = problem_;
  for (int d = 0; d < problem_.nd(); d++) {
    solution_->add_drone_desc();
  }
  latest_delivery_time_.clear();
  latest_delivery_time_.resize(problem_.no(), -1);
  score_ = 0;
}

std::pair<int, std::map<int, int>> GaSolver::CanTake(int arrival, int w,
                                                     int o) {
  CHECK(0 <= w) << w;
  CHECK(w < problem_.nw()) << w;
  int when = -1;
  std::map<int, int> what;
  for (const auto& t_pns : pending_warehouses_[w][o]) {
    // Allow for consecutive unload.
    // TODO(viktors): This means we don't allow nested loads and unloads (
    //                ex. just shifted by 1). Improve?
    if (t_pns.first > when && t_pns.first - when > 1 && !what.empty()) break;
    when = t_pns.first;
    for (const auto& pn : t_pns.second) {
      what[pn.first] += pn.second;
    }
  }
  int wait_time = std::max(0, when - arrival);
  return {wait_time, what};
}

void GaSolver::Take(int arrival, int w, int o, std::map<int, int> what,
                    bool manual_relax_order) {
  CHECK(0 <= w) << w;
  CHECK(w < problem_.nw()) << w;

  // Take the latest arrivals first.
  CHECK(!what.empty());
  CHECK(!pending_warehouses_[w][o].empty());
  auto it = pending_warehouses_[w][o].upper_bound(arrival);
  CHECK(it != pending_warehouses_[w][o].begin());
  it--;
  std::set<int> to_erase;
  std::map<int, int> what_bak = what;
  while (!what.empty()) {
    for (auto what_it = what.cbegin(); what_it != what.cend();) {
      int p = what_it->first;
      int n = what_it->second;
      if (it->second.count(p) == 0) {
        what_it++;
        continue;
      }
      int take = std::min(n, it->second[p]);
      if ((what[p] -= take) == 0) {
        what.erase(what_it++);
      } else {
        what_it++;
      }
      if ((it->second[p] -= take) == 0) {
        it->second.erase(p);
        if (it->second.empty()) {
          to_erase.insert(it->first);
          break;
        }
      }
    }
    if (!what.empty()) {
      CHECK(it != pending_warehouses_[w][o].begin());
      it--;
    }
  }
  for (int t : to_erase) {
    pending_warehouses_[w][o].erase(t);
  }
  if (pending_warehouses_[w][o].empty()) {
    pending_warehouses_[w].erase(o);
    if (pending_warehouses_[w].empty()) {
      pending_warehouses_.erase(w);
    }
  }
  // This also affects pending_orders becaouse of the warehouse info in it.
  if (!manual_relax_order) {
    for (const auto& pn : what_bak) {
      if ((pending_orders_[o][w][pn.first] -= pn.second) == 0) {
        pending_orders_[o][w].erase(pn.first);
        if (pending_orders_[o][w].empty()) {
          pending_orders_[o].erase(w);
          if (pending_orders_[o].empty()) {
            pending_orders_.erase(o);
          }
        }
      }
    }
  }
}

void GaSolver::Give(int arrival, int w, int o, const std::map<int, int>& what) {
  CHECK(0 <= w) << w;
  CHECK(w < problem_.nw()) << w;

  int t = arrival + 1;
  // Warning: Keep the same order as in the command-gen part.
  for (const auto& pn : what) {
    pending_warehouses_[w][o][t++][pn.first] += pn.second;
  }

  // This also affects pending_orders becaouse of the warehouse info in it.
  for (const auto& pn : what) {
    pending_orders_[o][w][pn.first] += pn.second;
  }
}

bool GaSolver::RelaxOrder(int w, int o, std::map<int, int> what) {
  bool order_complete = false;
  for (const auto& pn : what) {
    CHECK(!order_complete);
    CHECK(pending_orders_[o][w][pn.first] >= pn.second);
    if ((pending_orders_[o][w][pn.first] -= pn.second) == 0) {
      pending_orders_[o][w].erase(pn.first);
      if (pending_orders_[o][w].empty()) {
        pending_orders_[o].erase(w);
        if (pending_orders_[o].empty()) {
          pending_orders_.erase(o);
          order_complete = true;
        }
      }
    }
    pending_orders_weight_[o] -= problem_.product(pn.first).m() * pn.second;
  }
  return order_complete;
}

void GaSolver::CommandTransactionStart(int d) {
  CHECK(transactions_.count(d) == 0)
      << "There's another transaction happening.";
  CommandTransactionData data{.curr_drone_location = drone_location_[d],
                              .curr_drone_busy_until = drone_busy_until_[d],
                              .gen_commands = 0};
  transactions_[d] = data;
}

void GaSolver::CommandTransactionRollBack(int d) {
  CHECK(transactions_.count(d) == 1) << "No transction has been started!";
  while (transactions_[d].gen_commands--) {
    solution_->mutable_drone_desc(d)->mutable_drone_command()->RemoveLast();
  }
  transactions_.erase(d);
}

void GaSolver::CommandTransactionCommit(int d) {
  CHECK(transactions_.count(d) == 1) << "No transction has been started!";
  drone_location_[d] = transactions_[d].curr_drone_location;
  drone_busy_until_[d] = transactions_[d].curr_drone_busy_until;
  transactions_.erase(d);
}

bool GaSolver::Load(int d, int w, const std::map<int, int>& what,
                    int wait_time) {
  CHECK(transactions_.count(d) == 1)
      << "Can't generate commands outside of a transaction";

  if (wait_time > 0) {
    if (transactions_[d].curr_drone_busy_until + wait_time > problem_.t()) {
      return false;
    }
    auto* cmd = solution_->mutable_drone_desc(d)->add_drone_command();
    transactions_[d].gen_commands++;
    cmd->set_drone(d);
    cmd->set_type(DroneCommand_CommandType_WAIT);
    cmd->set_duration(wait_time);
    cmd->set_start_time(transactions_[d].curr_drone_busy_until + 1);
    transactions_[d].curr_drone_busy_until += wait_time;
  }

  bool success = true;
  int dist = problem_.dist().src(transactions_[d].curr_drone_location).dst(w);
  for (const auto& pn : what) {
    auto* cmd = solution_->mutable_drone_desc(d)->add_drone_command();
    transactions_[d].gen_commands++;
    cmd->set_drone(d);
    cmd->set_type(DroneCommand_CommandType_LOAD);
    cmd->set_warehouse(w);
    cmd->set_product(pn.first);
    cmd->set_num_items(pn.second);
    cmd->set_start_time(transactions_[d].curr_drone_busy_until + 1);
    transactions_[d].curr_drone_busy_until += dist + 1;
    transactions_[d].curr_drone_location = w;
    dist = 0;
    if (transactions_[d].curr_drone_busy_until > problem_.t()) {
      success = false;
      break;
    }
  }
  return success;
}

bool GaSolver::Unload(int d, int w, const std::map<int, int>& what) {
  bool success = true;
  int dist = problem_.dist().src(transactions_[d].curr_drone_location).dst(w);
  for (const auto& pn : what) {
    auto* cmd = solution_->mutable_drone_desc(d)->add_drone_command();
    transactions_[d].gen_commands++;
    cmd->set_drone(d);
    cmd->set_type(DroneCommand_CommandType_UNLOAD);
    cmd->set_warehouse(w);
    cmd->set_product(pn.first);
    cmd->set_num_items(pn.second);
    cmd->set_start_time(transactions_[d].curr_drone_busy_until + 1);
    transactions_[d].curr_drone_busy_until += dist + 1;
    transactions_[d].curr_drone_location = w;
    dist = 0;
    if (transactions_[d].curr_drone_busy_until > problem_.t()) {
      success = false;
      break;
    }
  }
  return success;
}

bool GaSolver::Deliver(int d, int o, const std::map<int, int>& what) {
  bool success = true;
  int dist = problem_.dist()
                 .src(transactions_[d].curr_drone_location)
                 .dst(problem_.nw() + o);
  for (const auto& pn : what) {
    auto* cmd = solution_->mutable_drone_desc(d)->add_drone_command();
    transactions_[d].gen_commands++;
    cmd->set_drone(d);
    cmd->set_type(DroneCommand_CommandType_DELIVER);
    cmd->set_order(o);
    cmd->set_product(pn.first);
    cmd->set_num_items(pn.second);
    cmd->set_start_time(transactions_[d].curr_drone_busy_until + 1);
    transactions_[d].curr_drone_busy_until += dist + 1;
    transactions_[d].curr_drone_location = problem_.nw() + o;
    dist = 0;
    if (transactions_[d].curr_drone_busy_until > problem_.t()) {
      success = false;
      break;
    }
  }
  return success;
}

bool GaSolver::Wait(int d, int wait_time) {
  CHECK(wait_time > 0);
  if (transactions_[d].curr_drone_busy_until + wait_time > problem_.t()) {
    return false;
  }
  auto* cmd = solution_->mutable_drone_desc(d)->add_drone_command();
  cmd->set_drone(d);
  cmd->set_type(DroneCommand_CommandType_WAIT);
  cmd->set_duration(wait_time);
  cmd->set_start_time(transactions_[d].curr_drone_busy_until + 1);
  transactions_[d].curr_drone_busy_until += wait_time;
  return true;
}

bool GaSolver::ComboMoveAtomic(int d, int w_from, int w_to, int o) {
  auto wt_what =
      CanTake(drone_busy_until_[d] +
                  problem_.dist().src(drone_location_[d]).dst(w_from),
              w_from, o);
  CHECK(!wt_what.second.empty());

  auto split = util::LoadSplitter::SplitOnce(wt_what.second, product_weights_,
                                             problem_.m());
  CommandTransactionStart(d);
  if (!Load(d, w_from, split.single_split, wt_what.first)) {
    CommandTransactionRollBack(d);
    return false;
  }
  int load_arrival =
      transactions_[d].curr_drone_busy_until - split.single_split.size();
  if (!Unload(d, w_to, split.single_split)) {
    CommandTransactionRollBack(d);
    return false;
  }
  int unload_arrival =
      transactions_[d].curr_drone_busy_until - split.single_split.size();
  CommandTransactionCommit(d);

  Take(load_arrival, w_from, o, split.single_split);
  Give(unload_arrival, w_to, o, split.single_split);
  return true;
}

bool GaSolver::ComboDeliverAtomic(int d, int w, int o) {
  auto wt_what = CanTake(
      drone_busy_until_[d] + problem_.dist().src(drone_location_[d]).dst(w), w,
      o);
  CHECK(!wt_what.second.empty());

  auto split = util::LoadSplitter::SplitOnce(wt_what.second, product_weights_,
                                             problem_.m());

  CommandTransactionStart(d);
  if (!Load(d, w, split.single_split, wt_what.first)) {
    CommandTransactionRollBack(d);
    return false;
  }
  int load_arrival =
      transactions_[d].curr_drone_busy_until - split.single_split.size();
  if (!Deliver(d, o, split.single_split)) {
    CommandTransactionRollBack(d);
    return false;
  }
  CommandTransactionCommit(d);

  Take(load_arrival, w, o, split.single_split, true);
  // Relax the order and augment the score if needed.
  latest_delivery_time_[o] =
      std::max(latest_delivery_time_[o], drone_busy_until_[d] - 1);
  bool completed = RelaxOrder(w, o, split.single_split);
  if (completed) {
    int t = latest_delivery_time_[o];
    int to_add = ceil((100.0 * (problem_.t() - t)) / problem_.t());
    score_ += to_add;
  }
  return true;
}

bool GaSolver::Strategy0(int d) {
  int src_w = -1;
  {
    int best_w = -1;
    for (const auto& wx : pending_warehouses_) {
      int w = wx.first;
      if (best_w < 0 || problem_.dist().src(drone_location_[d]).dst(best_w) >
                            problem_.dist().src(drone_location_[d]).dst(w)) {
        best_w = w;
      }
    }
    src_w = best_w;
    if (src_w < 0) {
      return false;
    }
  }
  CHECK(0 <= src_w);
  CHECK(src_w < problem_.nw()) << src_w;

  int dst_o = -1;
  {
    int best_o = -1;
    int best_time = std::numeric_limits<int>::max();
    for (const auto& ox : pending_warehouses_[src_w]) {
      int o = ox.first;
      if (closest_ws_[o][0] == src_w) continue;
      int dist = problem_.dist().src(src_w).dst(closest_ws_[o][0]);
      auto wt_what = CanTake(drone_busy_until_[d] + dist, src_w, o);
      int curr_time = dist + wt_what.first;
      if (curr_time < best_time) {
        best_o = o;
        best_time = curr_time;
      }
    }
    dst_o = best_o;
    if (dst_o < 0) {
      return false;
    }
  }

  return ComboMoveAtomic(d, src_w, closest_ws_[dst_o][0], dst_o);
}

bool GaSolver::Strategy1(int d) {
  int src_w = -1;
  {
    int best_w = -1;
    for (const auto& wx : pending_warehouses_) {
      int w = wx.first;
      if (best_w < 0 || problem_.dist().src(drone_location_[d]).dst(best_w) >
                            problem_.dist().src(drone_location_[d]).dst(w)) {
        best_w = w;
      }
    }
    src_w = best_w;
    if (src_w < 0) {
      return false;
    }
  }
  CHECK(0 <= src_w);
  CHECK(src_w < problem_.nw()) << src_w;

  int dst_o = -1;
  {
    int best_o = -1;
    int best_time = std::numeric_limits<int>::max();
    for (const auto& ox : pending_warehouses_[src_w]) {
      int o = ox.first;
      int dist = problem_.dist().src(src_w).dst(problem_.nw() + o);
      auto wt_what = CanTake(drone_busy_until_[d] + dist, src_w, o);
      int curr_time = dist + wt_what.first;
      if (curr_time < best_time) {
        best_o = o;
        best_time = curr_time;
      }
    }
    dst_o = best_o;
    if (dst_o < 0) {
      return false;
    }
  }

  return ComboDeliverAtomic(d, src_w, dst_o);
}

bool GaSolver::Strategy2(int d) {
  // Find the order that's closest to completion.
  int best_o = -1;
  for (int o = 0; o < problem_.no(); o++) {
    if (pending_orders_weight_[o] < 1) continue;
    if (best_o < 0 ||
        pending_orders_weight_[o] < pending_orders_weight_[best_o]) {
      best_o = o;
    }
  }
  if (best_o < 0) {
    return false;
  }
  CHECK(pending_orders_.count(best_o) == 1);

  // Find the warehouse from which you can deliver first.
  int best_w = -1;
  int best_w_time = std::numeric_limits<int>::max();
  for (const auto& wx : pending_orders_[best_o]) {
    int w = wx.first;
    int arrival =
        drone_busy_until_[d] + problem_.dist().src(drone_location_[d]).dst(w);
    int w_time = problem_.dist().src(drone_location_[d]).dst(w) +
                 problem_.dist().src(w).dst(problem_.nw() + best_o) +
                 CanTake(arrival, w, best_o).first;
    if (best_w_time > w_time) {
      best_w = w;
      best_w_time = w_time;
    }
  }
  if (best_w < 0) {
    return false;
  }

  return ComboDeliverAtomic(d, best_w, best_o);
}

bool GaSolver::Strategy3(int d) {
  // Find the order that's closest to completion.
  int best_o = -1;
  for (int o = 0; o < problem_.no(); o++) {
    if (pending_orders_weight_[o] < 1) continue;
    if (best_o < 0 ||
        pending_orders_weight_[o] < pending_orders_weight_[best_o]) {
      best_o = o;
    }
  }
  if (best_o < 0) {
    return false;
  }
  CHECK(pending_orders_.count(best_o) == 1);

  // Take from some warehouse to one of the closer warehouses (top
  // kNumConsideredWarehouses only).
  constexpr const int kNumConsideredWarehouses = 5;
  int best_w_src = -1;
  int best_w_dst = -1;
  int best_time = std::numeric_limits<int>::max();
  for (const auto& wx : pending_orders_[best_o]) {
    int w_src = wx.first;
    int num_traversed = 0;
    for (int w_dst : closest_ws_[best_o]) {
      if (num_traversed++ >= kNumConsideredWarehouses) break;
      if (w_dst == w_src) continue;
      if (problem_.dist().src(w_dst).dst(problem_.nw() + best_o) >
          problem_.dist().src(w_src).dst(problem_.nw() + best_o))
        continue;
      int arrival = drone_busy_until_[d] +
                    problem_.dist().src(drone_location_[d]).dst(w_src);
      int curr_time = problem_.dist().src(drone_location_[d]).dst(w_src) +
                      problem_.dist().src(w_src).dst(w_dst) +
                      CanTake(arrival, w_src, best_o).first;
      if (curr_time < best_time) {
        best_w_src = w_src;
        best_w_dst = w_dst;
        best_time = curr_time;
      }
    }
  }
  if (best_w_src < 0 || best_w_dst < 0) {
    return false;
  }

  return ComboMoveAtomic(d, best_w_src, best_w_dst, best_o);
}

bool GaSolver::Strategy4(int d) {
  // Find the order that's closest to completion.
  int best_o = -1;
  for (int o = 0; o < problem_.no(); o++) {
    if (pending_orders_weight_[o] < 1) continue;
    if (best_o < 0 ||
        pending_orders_weight_[o] < pending_orders_weight_[best_o]) {
      best_o = o;
    }
  }
  if (best_o < 0) {
    return false;
  }
  CHECK(pending_orders_.count(best_o) == 1);

  int best_w_src = -1;
  int best_time = std::numeric_limits<int>::max();
  for (const auto& wx : pending_orders_[best_o]) {
    int w_src = wx.first;
    if (closest_ws_[best_o][0] == w_src) continue;
    int arrival = drone_busy_until_[d] +
                  problem_.dist().src(drone_location_[d]).dst(w_src);
    int curr_time = problem_.dist().src(drone_location_[d]).dst(w_src) +
                    problem_.dist().src(w_src).dst(closest_ws_[best_o][0]) +
                    CanTake(arrival, w_src, best_o).first;
    if (curr_time < best_time) {
      best_w_src = w_src;
      best_time = curr_time;
    }
  }
  if (best_w_src < 0) {
    return false;
  }
  return ComboMoveAtomic(d, best_w_src, closest_ws_[best_o][0], best_o);
}

bool GaSolver::Strategy5(int d) {
  // Find the fastest possible delivery.
  int best_w = -1;
  int best_o = -1;
  int best_time = std::numeric_limits<int>::max();
  for (const auto& ox : pending_orders_) {
    int o = ox.first;
    for (const auto& wx : pending_orders_[o]) {
      int w = wx.first;
      int arrival =
          drone_busy_until_[d] + problem_.dist().src(drone_location_[d]).dst(w);
      int curr_time = problem_.dist().src(drone_location_[d]).dst(w) +
                      problem_.dist().src(w).dst(problem_.nw() + o) +
                      CanTake(arrival, w, o).first;
      if (best_time > curr_time) {
        best_w = w;
        best_o = o;
        best_time = curr_time;
      }
    }
  }
  if (best_w < 0 || best_o < 0) {
    return false;
  }

  return ComboDeliverAtomic(d, best_w, best_o);
}

bool GaSolver::Strategy6(int d) {
  // Take from some warehouse to one of the closer warehouses (top
  // kNumConsideredWarehouses only) of an order.
  constexpr const int kNumConsideredWarehouses = 5;
  int best_o = -1;
  int best_w_src = -1;
  int best_w_dst = -1;
  int best_time = std::numeric_limits<int>::max();
  for (const auto& ox : pending_orders_) {
    int o = ox.first;
    for (const auto& wx : pending_orders_[o]) {
      int w_src = wx.first;
      int num_traversed = 0;
      for (int w_dst : closest_ws_[o]) {
        if (num_traversed++ >= kNumConsideredWarehouses) break;
        if (w_dst == w_src) continue;
        if (problem_.dist().src(w_dst).dst(problem_.nw() + o) >
            problem_.dist().src(w_src).dst(problem_.nw() + o))
          continue;
        int arrival = drone_busy_until_[d] +
                      problem_.dist().src(drone_location_[d]).dst(w_src);
        int curr_time = problem_.dist().src(drone_location_[d]).dst(w_src) +
                        problem_.dist().src(w_src).dst(w_dst) +
                        CanTake(arrival, w_src, o).first;
        if (curr_time < best_time) {
          best_o = o;
          best_w_src = w_src;
          best_w_dst = w_dst;
          best_time = curr_time;
        }
      }
    }
  }
  if (best_o < 0 || best_w_src < 0 || best_w_dst < 0) {
    return false;
  }

  return ComboMoveAtomic(d, best_w_src, best_w_dst, best_o);
}

bool GaSolver::Strategy7(int d) {
  int best_o = -1;
  int best_w_src = -1;
  int best_time = std::numeric_limits<int>::max();
  for (const auto& ox : pending_orders_) {
    int o = ox.first;
    for (const auto& wx : pending_orders_[o]) {
      int w_src = wx.first;
      if (closest_ws_[o][0] == w_src) continue;
      int arrival = drone_busy_until_[d] +
                    problem_.dist().src(drone_location_[d]).dst(w_src);
      int curr_time = problem_.dist().src(drone_location_[d]).dst(w_src) +
                      problem_.dist().src(w_src).dst(closest_ws_[o][0]) +
                      CanTake(arrival, w_src, o).first;
      if (curr_time < best_time) {
        best_o = o;
        best_w_src = w_src;
        best_time = curr_time;
      }
    }
  }
  if (best_o < 0 || best_w_src < 0) {
    return false;
  }
  return ComboMoveAtomic(d, best_w_src, closest_ws_[best_o][0], best_o);
}

void GaSolver::PackSolution(
    const std::vector<std::vector<int>>& drone_strategies,
    bool log_success_ratios) {
  InitSolution();

  std::vector<int> next_strategy(problem_.nd(), 0);
  std::set<int> skipped_drones;
  std::vector<int> strategy_attempt(strategy_.size(), 0);
  std::vector<int> strategy_success(strategy_.size(), 0);
  std::vector<int> executions_left(problem_.nd(), FLAGS_ga_strategy_repeats);
  while (!pending_orders_.empty() && skipped_drones.size() < problem_.nd()) {
    // Find the next drone.
    int d = -1;
    for (int dd = 0; dd < problem_.nd(); dd++) {
      if (skipped_drones.count(dd)) continue;
      if (d < 0 || drone_busy_until_[dd] < drone_busy_until_[d]) {
        d = dd;
      }
    }
    CHECK(d >= 0);
    CHECK(d < problem_.nd());

    int strategy = drone_strategies[d][next_strategy[d]];
    strategy_attempt[strategy]++;
    bool success = strategy_[strategy](d);
    if (!success) {
      skipped_drones.insert(d);
      executions_left[d] = 0;
    } else {
      skipped_drones.clear();
      strategy_success[strategy]++;
      executions_left[d]--;
    }

    if (executions_left[d] == 0) {
      next_strategy[d] = (next_strategy[d] + 1) % drone_strategies[d].size();
    }
  }

  if (log_success_ratios) {
    LOG(INFO) << "Strategy success ratios (after-filtration indices):";
    for (int strategy = 0; strategy < strategy_.size(); strategy++) {
      LOG(INFO) << absl::Substitute("$0 : $1 / $2", strategy,
                                    strategy_success[strategy],
                                    strategy_attempt[strategy]);
    }
  }
}

// TODO(viktors): Is this too slow?
std::vector<std::vector<bool>> GaSolver::GenerateIndividual(
    int log_num_strategies) {
  std::vector<std::vector<bool>> res(
      problem_.nd(),
      std::vector<bool>(FLAGS_ga_strategies_per_drone * log_num_strategies));
  for (int d = 0; d < problem_.nd(); d++) {
    for (int bit = 0; bit < res[d].size(); bit++) {
      res[d][bit] = static_cast<bool>(random_engine_() % 2);
    }
  }
  return res;
}

int GaSolver::Eval(const std::vector<std::vector<bool>>& individual,
                   int log_num_strategies, bool log_success_ratios) {
  std::vector<std::vector<int>> drone_strategies(problem_.nd());
  for (int d = 0; d < problem_.nd(); d++) {
    for (int i = 0; i < individual[d].size() / log_num_strategies; i++) {
      int s = 0;
      int si = 1;
      for (int j = 0; j < log_num_strategies; j++) {
        int jj = i * log_num_strategies + j;
        if (individual[d][jj]) {
          s += si;
        }
        si <<= 1;
      }
      drone_strategies[d].push_back(s);
    }
  }
  PackSolution(drone_strategies);
  return score_;
}

void GaSolver::RunGA() {
  CHECK(FLAGS_ga_population_size == FLAGS_ga_selection_size +
                                    FLAGS_ga_crossingover_size +
                                    FLAGS_ga_mutation_size);
  using Individual = std::vector<std::vector<bool>>;
  const int log_num_strategies = ceil(log2(strategy_.size()));
  const int bits_per_drone = FLAGS_ga_strategies_per_drone * log_num_strategies;
  std::vector<Individual> population;
  std::vector<int> scores;
  int best_score = -1;
  int best_idx = 0;

  // Generate the first population.
  LOG(INFO) << "Generating the initial population...";
  for (int i = 0; i < FLAGS_ga_population_size; i++) {
    population.push_back(GenerateIndividual(log_num_strategies));
    scores.push_back(-1);
  }
  LOG(INFO) << "Generating the initial population... DONE";

  int eval_from = 0;
  auto do_evals = [&](int generation) {
    int total_evals =
        std::max(static_cast<int>(population.size()) - eval_from, 0);
    int curr_eval = 1;
    for (int i = eval_from; i < population.size(); i++) {
      scores[i] = Eval(population[i], log_num_strategies);
      if (scores[i] > best_score) {
        best_score = scores[i];
        best_idx = i;
      }
      LOG(INFO) << absl::Substitute(
          "Generation $0: eval $1 / $2. GOT: $3 BEST: $4", generation,
          curr_eval++, total_evals, scores[i], best_score);
    }
  };

  for (int gen = 1; gen <= FLAGS_ga_num_generations; gen++) {
    LOG(INFO) << absl::Substitute("===== Generation $0 / $1 =====", gen,
                                  FLAGS_ga_num_generations);
    if (gen > 1) {
      // Selection.
      LOG(INFO) << "Selection...";
      {
        int to_kick_out = population.size() - FLAGS_ga_selection_size;
        while (to_kick_out--) {
          std::uniform_int_distribution<int> gen_idx(0, population.size() - 1);
          int a = gen_idx(random_engine_);
          int b = gen_idx(random_engine_);
          int kick = scores[a] < scores[b] ? a : b;
          population[kick] = population.back();
          population.pop_back();
          scores[kick] = scores.back();
          scores.pop_back();
        }
        eval_from = FLAGS_ga_selection_size;
      }
      // Crossing over.
      LOG(INFO) << "Crossingover...";
      {
        std::uniform_int_distribution<int> gen_idx(0, population.size() - 1);
        CHECK(FLAGS_ga_crossingover_size % 2 == 0)
            << "ga_crossingover_size not even!";
        int to_add = FLAGS_ga_crossingover_size / 2;
        while (to_add--) {
          int num_twists = std::uniform_int_distribution<int>(
              1, FLAGS_ga_max_twists)(random_engine_);
          std::uniform_int_distribution<int> gen_twist(0, bits_per_drone - 1);
          std::vector<int> twists;
          for (int t = 0; t < num_twists; t++) {
            twists.push_back(gen_twist(random_engine_));
          }
          std::sort(twists.begin(), twists.end());

          int a = gen_idx(random_engine_);
          int b = gen_idx(random_engine_);

          Individual child1(problem_.nd());
          Individual child2(problem_.nd());
          for (int d = 0; d < problem_.nd(); d++) {
            int t = 0;
            child1[d].reserve(bits_per_drone);
            child2[d].reserve(bits_per_drone);
            for (int bit = 0; bit < bits_per_drone; bit++) {
              if (t < twists.size() && bit == twists[t]) {
                t++;
                std::swap(a, b);
              }
              child1[d].push_back(population[a][d][bit]);
              child2[d].push_back(population[b][d][bit]);
            }
          }
          population.push_back(child1);
          scores.push_back(-1);
          population.push_back(child2);
          scores.push_back(-1);
        }
      }
      // Mutation.
      LOG(INFO) << "Mutation...";
      {
        std::uniform_int_distribution<int> gen_idx(0, population.size() - 1);
        std::uniform_int_distribution<int> gen_drone(0, problem_.nd() - 1);
        std::uniform_real_distribution<double> gen_mutate(0.0, 1.0);
        int to_add = FLAGS_ga_mutation_size;
        while (to_add--) {
          int a = gen_idx(random_engine_);
          population.push_back(population[a]);
          scores.push_back(-1);
          auto mutate_drone = [&](int d) {
            for (int bit = 0; bit < bits_per_drone; bit++) {
              if (gen_mutate(random_engine_) >= FLAGS_ga_mutation_prob) {
                population.back()[d][bit] = !population.back()[d][bit];
              }
            }
          };
          if (FLAGS_ga_mutate_all_drones) {
            for (int d = 0; d < problem_.nd(); d++) {
              mutate_drone(d);
            }
          } else {
            mutate_drone(gen_drone(random_engine_));
          }
        }
      }
    }
    // Eval.
    LOG(INFO) << "Evaluating...";
    do_evals(gen);
  }
  Eval(population[best_idx], log_num_strategies, true);
}

std::unique_ptr<Solution> GaSolver::Solve() {
  RunGA();
  LOG(INFO) << "Should give: " << score_;
  return std::move(solution_);
}

}  // namespace drones
