#include "solvers/ga_solver/ga_solver.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <set>
#include <thread>
#include <vector>

#include "absl/strings/numbers.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"
#include "absl/strings/substitute.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "solution.pb.h"
#include "solvers/ga_solver/evaluator_service.grpc.pb.h"
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
DEFINE_bool(
    ga_is_standalone, true,
    "If true, both GA logic and the evaluation are done in the same proccess "
    "(this one). If set, ga_is_arbitrator and ga_is_evaluator are ignored. If "
    "false, exactly one of ga_is_arbitrator, ga_is_evaluator has to be true.");
DEFINE_bool(
    ga_is_arbitrator, false,
    "Only considered when ga_is_standalone is false. If set, the proccess is "
    "running as the arbitrator - does all the GA logic and sends all the "
    "evaluation requests to the evaluators via gRPC. If true, ga_is_evaluator "
    "has to be false.");
DEFINE_string(
    ga_evaluator_service_address, "localhost:12345",
    "Only considered when ga_is_arbitrator is true (and ga_is_standalone is "
    "false). Address of the load-balanced evaluator service.");
DEFINE_bool(
    ga_is_evaluator, false,
    "Only considered when ga_is_standalone is false. If set, the proccess is "
    "running as the evaluator and receives requests over gRPC. If true, "
    "ga_is_arbitrator has to be false.");
DEFINE_int32(ga_evaluator_num_threads, 4,
             "Number of threads to dispatch for evaluation.");

namespace drones {
GaSolver::GaSolver(const Problem& problem)
    : ProblemSolver(problem),
      kClosestWarehouses(CalcClosestWarehouses(problem)),
      kProductWeights(PrepareProductWeights(problem)),
      kAlloc(AllocateOrDie(problem)),
      kStrategies(FilterStrategies(this)),
      kLog2NumStrategies(ceil(log2(kStrategies.size()))),
      random_engine_(std::random_device()()) {}

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

std::vector<std::function<bool(GaSolver::SolutionBuilder*, int)>>
GaSolver::FilterStrategies(GaSolver* ga_solver) {
  std::set<std::string> split = absl::StrSplit(FLAGS_ga_forbidden_strategies,
                                               ",", absl::SkipWhitespace());
  std::set<int> forbidden_strategies;
  for (const std::string& s : split) {
    int strategy;
    CHECK(absl::SimpleAtoi(s, &strategy)) << "Can't parse: " << s;
    forbidden_strategies.insert(strategy);
  }

  std::vector<std::function<bool(SolutionBuilder*, int)>> res;
  if (forbidden_strategies.count(0) == 0) {
    res.push_back([=](SolutionBuilder* sb, int d) { return sb->Strategy0(d); });
  }
  if (forbidden_strategies.count(1) == 0) {
    res.push_back([=](SolutionBuilder* sb, int d) { return sb->Strategy1(d); });
  }
  if (forbidden_strategies.count(2) == 0) {
    res.push_back([=](SolutionBuilder* sb, int d) { return sb->Strategy2(d); });
  }
  if (forbidden_strategies.count(3) == 0) {
    res.push_back([=](SolutionBuilder* sb, int d) { return sb->Strategy3(d); });
  }
  if (forbidden_strategies.count(4) == 0) {
    res.push_back([=](SolutionBuilder* sb, int d) { return sb->Strategy4(d); });
  }
  if (forbidden_strategies.count(5) == 0) {
    res.push_back([=](SolutionBuilder* sb, int d) { return sb->Strategy5(d); });
  }
  if (forbidden_strategies.count(6) == 0) {
    res.push_back([=](SolutionBuilder* sb, int d) { return sb->Strategy6(d); });
  }
  if (forbidden_strategies.count(7) == 0) {
    res.push_back([=](SolutionBuilder* sb, int d) { return sb->Strategy7(d); });
  }
  return res;
}

GaSolver::SolutionBuilder::SolutionBuilder(const GaSolver& solver)
    : solver(solver),
      problem(solver.problem_),
      drone_location(problem.nd(), 0),
      drone_busy_until(problem.nd(), 0),
      pending_warehouses(),
      pending_orders(),
      pending_orders_weight(problem.no(), 0),
      transactions(),
      latest_delivery_time(problem.no(), -1),
      score(0) {
  {
    std::lock_guard<std::mutex> l(const_cast<GaSolver&>(solver).mux_kAlloc);
    for (int o = 0; o < problem.no(); o++) {
      for (const auto& wp_i : solver.kAlloc.at(o)) {
        if (wp_i.second > 0) {
          pending_warehouses[wp_i.first.first][o][-1][wp_i.first.second] +=
              wp_i.second;
          pending_orders[o][wp_i.first.first][wp_i.first.second] += wp_i.second;
        }
      }
    }
  }
  for (int o = 0; o < problem.no(); o++) {
    for (int p = 0; p < problem.np(); p++) {
      pending_orders_weight[o] +=
          problem.product(p).m() * problem.order(o).request(p);
    }
  }
  solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem;
  for (int d = 0; d < problem.nd(); d++) {
    solution->add_drone_desc();
  }
}

std::pair<int, std::map<int, int>> GaSolver::SolutionBuilder::CanTake(
    int arrival, int w, int o) {
  CHECK(0 <= w) << w;
  CHECK(w < problem.nw()) << w;
  int when = -1;
  std::map<int, int> what;
  for (const auto& t_pns : pending_warehouses[w][o]) {
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

void GaSolver::SolutionBuilder::Take(int arrival, int w, int o,
                                     std::map<int, int> what,
                                     bool manual_relax_order) {
  CHECK(0 <= w) << w;
  CHECK(w < problem.nw()) << w;

  // Take the latest arrivals first.
  CHECK(!what.empty());
  CHECK(!pending_warehouses[w][o].empty());
  auto it = pending_warehouses[w][o].upper_bound(arrival);
  CHECK(it != pending_warehouses[w][o].begin());
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
      CHECK(it != pending_warehouses[w][o].begin());
      it--;
    }
  }
  for (int t : to_erase) {
    pending_warehouses[w][o].erase(t);
  }
  if (pending_warehouses[w][o].empty()) {
    pending_warehouses[w].erase(o);
    if (pending_warehouses[w].empty()) {
      pending_warehouses.erase(w);
    }
  }
  // This also affects pending_orders becaouse of the warehouse info in it.
  if (!manual_relax_order) {
    for (const auto& pn : what_bak) {
      if ((pending_orders[o][w][pn.first] -= pn.second) == 0) {
        pending_orders[o][w].erase(pn.first);
        if (pending_orders[o][w].empty()) {
          pending_orders[o].erase(w);
          if (pending_orders[o].empty()) {
            pending_orders.erase(o);
          }
        }
      }
    }
  }
}

void GaSolver::SolutionBuilder::Give(int arrival, int w, int o,
                                     const std::map<int, int>& what) {
  CHECK(0 <= w) << w;
  CHECK(w < problem.nw()) << w;

  int t = arrival + 1;
  // Warning: Keep the same order as in the command-gen part.
  for (const auto& pn : what) {
    pending_warehouses[w][o][t++][pn.first] += pn.second;
  }

  // This also affects pending_orders becaouse of the warehouse info in it.
  for (const auto& pn : what) {
    pending_orders[o][w][pn.first] += pn.second;
  }
}

bool GaSolver::SolutionBuilder::RelaxOrder(int w, int o,
                                           std::map<int, int> what) {
  bool order_complete = false;
  for (const auto& pn : what) {
    CHECK(!order_complete);
    CHECK(pending_orders[o][w][pn.first] >= pn.second);
    if ((pending_orders[o][w][pn.first] -= pn.second) == 0) {
      pending_orders[o][w].erase(pn.first);
      if (pending_orders[o][w].empty()) {
        pending_orders[o].erase(w);
        if (pending_orders[o].empty()) {
          pending_orders.erase(o);
          order_complete = true;
        }
      }
    }
    pending_orders_weight[o] -= problem.product(pn.first).m() * pn.second;
  }
  return order_complete;
}

void GaSolver::SolutionBuilder::CommandTransactionStart(int d) {
  CHECK(transactions.count(d) == 0) << "There's another transaction happening.";
  CommandTransactionData data{.curr_drone_location = drone_location[d],
                              .curr_drone_busy_until = drone_busy_until[d],
                              .gen_commands = 0};
  transactions[d] = data;
}

void GaSolver::SolutionBuilder::CommandTransactionRollBack(int d) {
  CHECK(transactions.count(d) == 1) << "No transction has been started!";
  while (transactions[d].gen_commands--) {
    solution->mutable_drone_desc(d)->mutable_drone_command()->RemoveLast();
  }
  transactions.erase(d);
}

void GaSolver::SolutionBuilder::CommandTransactionCommit(int d) {
  CHECK(transactions.count(d) == 1) << "No transction has been started!";
  drone_location[d] = transactions[d].curr_drone_location;
  drone_busy_until[d] = transactions[d].curr_drone_busy_until;
  transactions.erase(d);
}

bool GaSolver::SolutionBuilder::Load(int d, int w,
                                     const std::map<int, int>& what,
                                     int wait_time) {
  CHECK(transactions.count(d) == 1)
      << "Can't generate commands outside of a transaction";

  if (wait_time > 0) {
    if (transactions[d].curr_drone_busy_until + wait_time > problem.t()) {
      return false;
    }
    auto* cmd = solution->mutable_drone_desc(d)->add_drone_command();
    transactions[d].gen_commands++;
    cmd->set_drone(d);
    cmd->set_type(DroneCommand_CommandType_WAIT);
    cmd->set_duration(wait_time);
    cmd->set_start_time(transactions[d].curr_drone_busy_until + 1);
    transactions[d].curr_drone_busy_until += wait_time;
  }

  bool success = true;
  int dist = problem.dist().src(transactions[d].curr_drone_location).dst(w);
  for (const auto& pn : what) {
    auto* cmd = solution->mutable_drone_desc(d)->add_drone_command();
    transactions[d].gen_commands++;
    cmd->set_drone(d);
    cmd->set_type(DroneCommand_CommandType_LOAD);
    cmd->set_warehouse(w);
    cmd->set_product(pn.first);
    cmd->set_num_items(pn.second);
    cmd->set_start_time(transactions[d].curr_drone_busy_until + 1);
    transactions[d].curr_drone_busy_until += dist + 1;
    transactions[d].curr_drone_location = w;
    dist = 0;
    if (transactions[d].curr_drone_busy_until > problem.t()) {
      success = false;
      break;
    }
  }
  return success;
}

bool GaSolver::SolutionBuilder::Unload(int d, int w,
                                       const std::map<int, int>& what) {
  bool success = true;
  int dist = problem.dist().src(transactions[d].curr_drone_location).dst(w);
  for (const auto& pn : what) {
    auto* cmd = solution->mutable_drone_desc(d)->add_drone_command();
    transactions[d].gen_commands++;
    cmd->set_drone(d);
    cmd->set_type(DroneCommand_CommandType_UNLOAD);
    cmd->set_warehouse(w);
    cmd->set_product(pn.first);
    cmd->set_num_items(pn.second);
    cmd->set_start_time(transactions[d].curr_drone_busy_until + 1);
    transactions[d].curr_drone_busy_until += dist + 1;
    transactions[d].curr_drone_location = w;
    dist = 0;
    if (transactions[d].curr_drone_busy_until > problem.t()) {
      success = false;
      break;
    }
  }
  return success;
}

bool GaSolver::SolutionBuilder::Deliver(int d, int o,
                                        const std::map<int, int>& what) {
  bool success = true;
  int dist = problem.dist()
                 .src(transactions[d].curr_drone_location)
                 .dst(problem.nw() + o);
  for (const auto& pn : what) {
    auto* cmd = solution->mutable_drone_desc(d)->add_drone_command();
    transactions[d].gen_commands++;
    cmd->set_drone(d);
    cmd->set_type(DroneCommand_CommandType_DELIVER);
    cmd->set_order(o);
    cmd->set_product(pn.first);
    cmd->set_num_items(pn.second);
    cmd->set_start_time(transactions[d].curr_drone_busy_until + 1);
    transactions[d].curr_drone_busy_until += dist + 1;
    transactions[d].curr_drone_location = problem.nw() + o;
    dist = 0;
    if (transactions[d].curr_drone_busy_until > problem.t()) {
      success = false;
      break;
    }
  }
  return success;
}

bool GaSolver::SolutionBuilder::Wait(int d, int wait_time) {
  CHECK(wait_time > 0);
  if (transactions[d].curr_drone_busy_until + wait_time > problem.t()) {
    return false;
  }
  auto* cmd = solution->mutable_drone_desc(d)->add_drone_command();
  cmd->set_drone(d);
  cmd->set_type(DroneCommand_CommandType_WAIT);
  cmd->set_duration(wait_time);
  cmd->set_start_time(transactions[d].curr_drone_busy_until + 1);
  transactions[d].curr_drone_busy_until += wait_time;
  return true;
}

bool GaSolver::SolutionBuilder::ComboMoveAtomic(int d, int w_from, int w_to,
                                                int o) {
  auto wt_what = CanTake(
      drone_busy_until[d] + problem.dist().src(drone_location[d]).dst(w_from),
      w_from, o);
  CHECK(!wt_what.second.empty());
  const_cast<GaSolver&>(solver).mux_kProductWeights.lock();
  auto split = util::LoadSplitter::SplitOnce(
      wt_what.second, solver.kProductWeights, problem.m());
  const_cast<GaSolver&>(solver).mux_kProductWeights.unlock();
  CommandTransactionStart(d);
  if (!Load(d, w_from, split.single_split, wt_what.first)) {
    CommandTransactionRollBack(d);
    return false;
  }
  int load_arrival =
      transactions[d].curr_drone_busy_until - split.single_split.size();
  if (!Unload(d, w_to, split.single_split)) {
    CommandTransactionRollBack(d);
    return false;
  }
  int unload_arrival =
      transactions[d].curr_drone_busy_until - split.single_split.size();
  CommandTransactionCommit(d);

  Take(load_arrival, w_from, o, split.single_split);
  Give(unload_arrival, w_to, o, split.single_split);
  return true;
}

bool GaSolver::SolutionBuilder::ComboDeliverAtomic(int d, int w, int o) {
  auto wt_what = CanTake(
      drone_busy_until[d] + problem.dist().src(drone_location[d]).dst(w), w, o);
  CHECK(!wt_what.second.empty());

  const_cast<GaSolver&>(solver).mux_kProductWeights.lock();
  auto split = util::LoadSplitter::SplitOnce(
      wt_what.second, solver.kProductWeights, problem.m());
  const_cast<GaSolver&>(solver).mux_kProductWeights.unlock();

  CommandTransactionStart(d);
  if (!Load(d, w, split.single_split, wt_what.first)) {
    CommandTransactionRollBack(d);
    return false;
  }
  int load_arrival =
      transactions[d].curr_drone_busy_until - split.single_split.size();
  if (!Deliver(d, o, split.single_split)) {
    CommandTransactionRollBack(d);
    return false;
  }
  CommandTransactionCommit(d);

  Take(load_arrival, w, o, split.single_split, true);
  // Relax the order and augment the score if needed.
  latest_delivery_time[o] =
      std::max(latest_delivery_time[o], drone_busy_until[d] - 1);
  bool completed = RelaxOrder(w, o, split.single_split);
  if (completed) {
    int t = latest_delivery_time[o];
    int to_add = ceil((100.0 * (problem.t() - t)) / problem.t());
    score += to_add;
  }
  return true;
}

bool GaSolver::SolutionBuilder::Strategy0(int d) {
  int src_w = -1;
  {
    int best_w = -1;
    for (const auto& wx : pending_warehouses) {
      int w = wx.first;
      if (best_w < 0 || problem.dist().src(drone_location[d]).dst(best_w) >
                            problem.dist().src(drone_location[d]).dst(w)) {
        best_w = w;
      }
    }
    src_w = best_w;
    if (src_w < 0) {
      return false;
    }
  }
  CHECK(0 <= src_w);
  CHECK(src_w < problem.nw()) << src_w;

  int dst_o = -1;
  {
    int best_o = -1;
    int best_time = std::numeric_limits<int>::max();
    for (const auto& ox : pending_warehouses[src_w]) {
      int o = ox.first;
      if (solver.kClosestWarehouses[o][0] == src_w) continue;
      int dist = problem.dist().src(src_w).dst(solver.kClosestWarehouses[o][0]);
      auto wt_what = CanTake(drone_busy_until[d] + dist, src_w, o);
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

  return ComboMoveAtomic(d, src_w, solver.kClosestWarehouses[dst_o][0], dst_o);
}

bool GaSolver::SolutionBuilder::Strategy1(int d) {
  int src_w = -1;
  {
    int best_w = -1;
    for (const auto& wx : pending_warehouses) {
      int w = wx.first;
      if (best_w < 0 || problem.dist().src(drone_location[d]).dst(best_w) >
                            problem.dist().src(drone_location[d]).dst(w)) {
        best_w = w;
      }
    }
    src_w = best_w;
    if (src_w < 0) {
      return false;
    }
  }
  CHECK(0 <= src_w);
  CHECK(src_w < problem.nw()) << src_w;

  int dst_o = -1;
  {
    int best_o = -1;
    int best_time = std::numeric_limits<int>::max();
    for (const auto& ox : pending_warehouses[src_w]) {
      int o = ox.first;
      int dist = problem.dist().src(src_w).dst(problem.nw() + o);
      auto wt_what = CanTake(drone_busy_until[d] + dist, src_w, o);
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

bool GaSolver::SolutionBuilder::Strategy2(int d) {
  // Find the order that's closest to completion.
  int best_o = -1;
  for (int o = 0; o < problem.no(); o++) {
    if (pending_orders_weight[o] < 1) continue;
    if (best_o < 0 ||
        pending_orders_weight[o] < pending_orders_weight[best_o]) {
      best_o = o;
    }
  }
  if (best_o < 0) {
    return false;
  }
  CHECK(pending_orders.count(best_o) == 1);

  // Find the warehouse from which you can deliver first.
  int best_w = -1;
  int best_w_time = std::numeric_limits<int>::max();
  for (const auto& wx : pending_orders[best_o]) {
    int w = wx.first;
    int arrival =
        drone_busy_until[d] + problem.dist().src(drone_location[d]).dst(w);
    int w_time = problem.dist().src(drone_location[d]).dst(w) +
                 problem.dist().src(w).dst(problem.nw() + best_o) +
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

bool GaSolver::SolutionBuilder::Strategy3(int d) {
  // Find the order that's closest to completion.
  int best_o = -1;
  for (int o = 0; o < problem.no(); o++) {
    if (pending_orders_weight[o] < 1) continue;
    if (best_o < 0 ||
        pending_orders_weight[o] < pending_orders_weight[best_o]) {
      best_o = o;
    }
  }
  if (best_o < 0) {
    return false;
  }
  CHECK(pending_orders.count(best_o) == 1);

  // Take from some warehouse to one of the closer warehouses (top
  // kNumConsideredWarehouses only).
  constexpr const int kNumConsideredWarehouses = 5;
  int best_w_src = -1;
  int best_w_dst = -1;
  int best_time = std::numeric_limits<int>::max();
  for (const auto& wx : pending_orders[best_o]) {
    int w_src = wx.first;
    int num_traversed = 0;
    for (int w_dst : solver.kClosestWarehouses[best_o]) {
      if (num_traversed++ >= kNumConsideredWarehouses) break;
      if (w_dst == w_src) continue;
      if (problem.dist().src(w_dst).dst(problem.nw() + best_o) >
          problem.dist().src(w_src).dst(problem.nw() + best_o))
        continue;
      int arrival = drone_busy_until[d] +
                    problem.dist().src(drone_location[d]).dst(w_src);
      int curr_time = problem.dist().src(drone_location[d]).dst(w_src) +
                      problem.dist().src(w_src).dst(w_dst) +
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

bool GaSolver::SolutionBuilder::Strategy4(int d) {
  // Find the order that's closest to completion.
  int best_o = -1;
  for (int o = 0; o < problem.no(); o++) {
    if (pending_orders_weight[o] < 1) continue;
    if (best_o < 0 ||
        pending_orders_weight[o] < pending_orders_weight[best_o]) {
      best_o = o;
    }
  }
  if (best_o < 0) {
    return false;
  }
  CHECK(pending_orders.count(best_o) == 1);

  int best_w_src = -1;
  int best_time = std::numeric_limits<int>::max();
  for (const auto& wx : pending_orders[best_o]) {
    int w_src = wx.first;
    if (solver.kClosestWarehouses[best_o][0] == w_src) continue;
    int arrival =
        drone_busy_until[d] + problem.dist().src(drone_location[d]).dst(w_src);
    int curr_time =
        problem.dist().src(drone_location[d]).dst(w_src) +
        problem.dist().src(w_src).dst(solver.kClosestWarehouses[best_o][0]) +
        CanTake(arrival, w_src, best_o).first;
    if (curr_time < best_time) {
      best_w_src = w_src;
      best_time = curr_time;
    }
  }
  if (best_w_src < 0) {
    return false;
  }
  return ComboMoveAtomic(d, best_w_src, solver.kClosestWarehouses[best_o][0],
                         best_o);
}

bool GaSolver::SolutionBuilder::Strategy5(int d) {
  // Find the fastest possible delivery.
  int best_w = -1;
  int best_o = -1;
  int best_time = std::numeric_limits<int>::max();
  for (const auto& ox : pending_orders) {
    int o = ox.first;
    for (const auto& wx : pending_orders[o]) {
      int w = wx.first;
      int arrival =
          drone_busy_until[d] + problem.dist().src(drone_location[d]).dst(w);
      int curr_time = problem.dist().src(drone_location[d]).dst(w) +
                      problem.dist().src(w).dst(problem.nw() + o) +
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

bool GaSolver::SolutionBuilder::Strategy6(int d) {
  // Take from some warehouse to one of the closer warehouses (top
  // kNumConsideredWarehouses only) of an order.
  constexpr const int kNumConsideredWarehouses = 5;
  int best_o = -1;
  int best_w_src = -1;
  int best_w_dst = -1;
  int best_time = std::numeric_limits<int>::max();
  for (const auto& ox : pending_orders) {
    int o = ox.first;
    for (const auto& wx : pending_orders[o]) {
      int w_src = wx.first;
      int num_traversed = 0;
      for (int w_dst : solver.kClosestWarehouses[o]) {
        if (num_traversed++ >= kNumConsideredWarehouses) break;
        if (w_dst == w_src) continue;
        if (problem.dist().src(w_dst).dst(problem.nw() + o) >
            problem.dist().src(w_src).dst(problem.nw() + o))
          continue;
        int arrival = drone_busy_until[d] +
                      problem.dist().src(drone_location[d]).dst(w_src);
        int curr_time = problem.dist().src(drone_location[d]).dst(w_src) +
                        problem.dist().src(w_src).dst(w_dst) +
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

bool GaSolver::SolutionBuilder::Strategy7(int d) {
  int best_o = -1;
  int best_w_src = -1;
  int best_time = std::numeric_limits<int>::max();
  for (const auto& ox : pending_orders) {
    int o = ox.first;
    for (const auto& wx : pending_orders[o]) {
      int w_src = wx.first;
      if (solver.kClosestWarehouses[o][0] == w_src) continue;
      int arrival = drone_busy_until[d] +
                    problem.dist().src(drone_location[d]).dst(w_src);
      int curr_time =
          problem.dist().src(drone_location[d]).dst(w_src) +
          problem.dist().src(w_src).dst(solver.kClosestWarehouses[o][0]) +
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
  return ComboMoveAtomic(d, best_w_src, solver.kClosestWarehouses[best_o][0],
                         best_o);
}

void GaSolver::SolutionBuilder::Build(const DroneStrategies& drone_strategies,
                                      bool log_success_ratios) {
  std::vector<int> next_strategy(problem.nd(), 0);
  std::set<int> skipped_drones;
  std::vector<int> strategy_attempt(solver.kStrategies.size(), 0);
  std::vector<int> strategy_success(solver.kStrategies.size(), 0);
  std::vector<int> executions_left(problem.nd(), FLAGS_ga_strategy_repeats);
  while (!pending_orders.empty() && skipped_drones.size() < problem.nd()) {
    // Find the next drone.
    int d = -1;
    for (int dd = 0; dd < problem.nd(); dd++) {
      if (skipped_drones.count(dd)) continue;
      if (d < 0 || drone_busy_until[dd] < drone_busy_until[d]) {
        d = dd;
      }
    }
    CHECK(d >= 0);
    CHECK(d < problem.nd());

    int strategy = drone_strategies[d][next_strategy[d]];
    strategy_attempt[strategy]++;
    bool success = solver.kStrategies[strategy](this, d);
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
    for (int strategy = 0; strategy < solver.kStrategies.size(); strategy++) {
      LOG(INFO) << absl::Substitute("$0 : $1 / $2", strategy,
                                    strategy_success[strategy],
                                    strategy_attempt[strategy]);
    }
  }
}

GaSolver::SolutionBuilder::DroneStrategies
GaSolver::IndividualToDroneStrategies(const Individual& individual) {
  SolutionBuilder::DroneStrategies drone_strategies(problem_.nd());
  for (int d = 0; d < problem_.nd(); d++) {
    for (int i = 0; i < individual[d].size() / kLog2NumStrategies; i++) {
      int s = 0;
      int si = 1;
      for (int j = 0; j < kLog2NumStrategies; j++) {
        int jj = i * kLog2NumStrategies + j;
        if (individual[d][jj]) {
          s += si;
        }
        si <<= 1;
      }
      drone_strategies[d].push_back(s);
    }
  }
  return drone_strategies;
}

std::vector<std::vector<bool>> GaSolver::GenerateIndividual() {
  std::vector<std::vector<bool>> res(
      problem_.nd(),
      std::vector<bool>(FLAGS_ga_strategies_per_drone * kLog2NumStrategies));
  for (int d = 0; d < problem_.nd(); d++) {
    for (int bit = 0; bit < res[d].size(); bit++) {
      res[d][bit] = static_cast<bool>(random_engine_() % 2);
    }
  }
  return res;
}

int GaSolver::Eval(const std::vector<std::vector<bool>>& individual) {
  SolutionBuilder builder(*this);
  builder.Build(IndividualToDroneStrategies(individual));
  return builder.score;
}

int GaSolver::DoEvals(const std::vector<Individual>& population,
                      int prev_gen_best, int eval_from, int generation,
                      std::vector<int>* scores_out) {
  int total_evals =
      std::max(static_cast<int>(population.size()) - eval_from, 0);
  int best = prev_gen_best;
  int curr_eval = 1;
  for (int i = eval_from; i < population.size(); i++) {
    best = std::max(best, scores_out->at(i) = Eval(population[i]));
    LOG(INFO) << absl::Substitute(
        "Generation $0: eval $1 / $2. GOT: $3 BEST: $4", generation,
        curr_eval++, total_evals, scores_out->at(i), best);
  }
  return best;
}

int GaSolver::DoRemoteEvals(const std::vector<Individual>& population,
                            int prev_gen_best, int eval_from, int generation,
                            std::vector<int>* scores_out) {
  using ga_solver::EvaluationRequest;
  using ga_solver::EvaluationResponse;
  using ga_solver::Evaluator;
  using grpc::ClientAsyncResponseReader;
  using grpc::ClientContext;
  using grpc::CompletionQueue;
  using grpc::CreateChannel;
  using grpc::InsecureChannelCredentials;
  using grpc::Status;

  auto channel = CreateChannel(FLAGS_ga_evaluator_service_address,
                               InsecureChannelCredentials());
  auto stub = Evaluator::NewStub(channel);
  CompletionQueue cq;

  struct PendingCall {
    EvaluationResponse reply;
    ClientContext context;
    Status status;
    std::unique_ptr<ClientAsyncResponseReader<EvaluationResponse>>
        response_reader;
  };
  std::vector<PendingCall> pending_calls(population.size());

  int total_evals =
      std::max(static_cast<int>(population.size()) - eval_from, 0);
  int best = prev_gen_best;

  auto async_complete = [&]() {
    void* got_tag;
    bool ok;
    int total_completed = 0;
    while (cq.Next(&got_tag, &ok)) {
      int i = reinterpret_cast<intptr_t>(got_tag);
      CHECK(ok);
      CHECK(pending_calls[i].status.ok());
      best = std::max(best, scores_out->at(i) = pending_calls[i].reply.score());
      LOG(INFO) << absl::Substitute(
          "Generation $0: eval $1 / $2. GOT: $3 BEST: $4", generation,
          ++total_completed, total_evals, scores_out->at(i), best);
      if (total_completed == total_evals) break;
    }
  };

  auto reader_thread = std::thread(async_complete);

  for (int i = eval_from; i < population.size(); i++) {
    // Reinterpret.
    auto individual = IndividualToDroneStrategies(population[i]);
    // Pack the request.
    EvaluationRequest req;
    for (int d = 0; d < problem_.nd(); d++) {
      auto* strategies = req.add_drone_strategies();
      for (int s : individual[d]) {
        strategies->add_strategy(s);
      }
    }
    // Send out the call.
    pending_calls[i].response_reader =
        stub->PrepareAsyncEval(&pending_calls[i].context, req, &cq);
    pending_calls[i].response_reader->StartCall();
    pending_calls[i].response_reader->Finish(
        &pending_calls[i].reply, &pending_calls[i].status, (void*)i);
  }
  reader_thread.join();
  return best;
}

void GaSolver::RunEvaluator() {
  using ga_solver::EvaluationRequest;
  using ga_solver::EvaluationResponse;
  using ga_solver::Evaluator;
  using grpc::Server;
  using grpc::ServerAsyncResponseWriter;
  using grpc::ServerBuilder;
  using grpc::ServerCompletionQueue;
  using grpc::ServerContext;
  using grpc::Status;

  // TODO(viktors): Check whether this order is required.
  std::unique_ptr<ServerCompletionQueue> cq;
  Evaluator::AsyncService service;
  std::unique_ptr<Server> server;
  ServerBuilder builder;
  builder.AddListeningPort(FLAGS_ga_evaluator_service_address,
                           grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  cq = builder.AddCompletionQueue();
  server = builder.BuildAndStart();
  LOG(INFO) << "Server listening on: " << FLAGS_ga_evaluator_service_address;

  struct PendingCall {
    enum CallStatus { CREATE, PROCESS, FINISH };

    PendingCall(GaSolver* solver, Evaluator::AsyncService* service,
                ServerCompletionQueue* cq)
        : solver(*solver),
          service(service),
          cq(cq),
          responder(&ctx),
          status(CREATE) {
      Proceed();
    }

    GaSolver& solver;
    Evaluator::AsyncService* service;
    ServerCompletionQueue* cq;
    ServerContext ctx;
    EvaluationRequest req;
    EvaluationResponse resp;
    ServerAsyncResponseWriter<EvaluationResponse> responder;
    CallStatus status;

    void Proceed(int thread_id = -1) {
      LOG(INFO) << "Proceed " << thread_id << " st " << status << " this "
                << this;
      switch (status) {
        case CREATE: {
          service->RequestEval(&ctx, &req, &responder, cq, cq, this);
          status = PROCESS;
          break;
        }
        case PROCESS: {
          new PendingCall(&solver, service, cq);

          LOG(INFO) << thread_id << " : Got a request.";

          Individual individual(solver.problem_.nd());
          for (int d = 0; d < solver.problem_.nd(); d++) {
            for (int s : req.drone_strategies(d).strategy()) {
              individual[d].push_back(s);
            }
          }
          resp.set_score(solver.Eval(individual));
          responder.Finish(resp, Status::OK, this);
          status = FINISH;
          LOG(INFO) << thread_id << " : Sent a response.";
          break;
        }
        case FINISH: {
          LOG(INFO) << thread_id << " : Completed request.";
          delete this;
          break;
        }
      }
    }
  };

  std::function<void(int)> work = [&](int thread_id) {
    auto* dummy = new PendingCall(this, &service, cq.get());
    void* tag;
    bool ok;
    while (true) {
      CHECK(cq->Next(&tag, &ok));
      CHECK(ok);
      reinterpret_cast<PendingCall*>(tag)->Proceed(thread_id);
    }
  };
  std::vector<std::thread> workers;
  for (int t = 0; t < FLAGS_ga_evaluator_num_threads; t++) {
    workers.emplace_back(work, t);
  }
  LOG(INFO) << "Dispatched " << workers.size() << " workers.";
  for (auto& t : workers) {
    t.join();
    LOG(WARNING) << "A infinite loop worker thread has exited.";
  }
}

GaSolver::Individual GaSolver::RunGA() {
  CHECK(FLAGS_ga_population_size == FLAGS_ga_selection_size +
                                        FLAGS_ga_crossingover_size +
                                        FLAGS_ga_mutation_size);
  const int bits_per_drone = FLAGS_ga_strategies_per_drone * kLog2NumStrategies;
  std::vector<Individual> population;
  std::vector<int> scores;
  int best_score = -1;
  int eval_from = 0;

  // Generate the first population.
  LOG(INFO) << "Generating the initial population...";
  for (int i = 0; i < FLAGS_ga_population_size; i++) {
    population.push_back(GenerateIndividual());
    scores.push_back(-1);
  }
  LOG(INFO) << "Generating the initial population... DONE";

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
    if (FLAGS_ga_is_standalone) {
      best_score = DoEvals(population, best_score, eval_from, gen, &scores);
    } else {
      CHECK(FLAGS_ga_is_arbitrator) << "Can't be here!";
      best_score =
          DoRemoteEvals(population, best_score, eval_from, gen, &scores);
    }
  }
  return population[std::find(scores.begin(), scores.end(), best_score) -
                    scores.begin()];
}

std::unique_ptr<Solution> GaSolver::Solve() {
  if (FLAGS_ga_is_standalone || FLAGS_ga_is_arbitrator) {
    auto best_individual = RunGA();
    SolutionBuilder builder(*this);
    builder.Build(IndividualToDroneStrategies(best_individual), true);
    LOG(INFO) << "Should give: " << builder.score;
    return std::move(builder.solution);
  } else {
    CHECK(FLAGS_ga_is_evaluator) << "Can't be here";
    RunEvaluator();
    return nullptr;
  }
}

}  // namespace drones
