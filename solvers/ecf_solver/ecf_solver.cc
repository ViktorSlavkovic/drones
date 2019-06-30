#include "solvers/ecf_solver/ecf_solver.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <random>
#include <utility>
#include <vector>

#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "solvers/util/allocator.h"
#include "solvers/util/load_splitter.h"

DEFINE_int32(ecf_split_iter, 1,
             "Number of split iterations, 1 is essential an it's without a "
             "feedback in alloc.");
DEFINE_int32(ecf_perm_iter, 0,
             "Number of order permutation improvement iterations.");
DEFINE_string(ecf_save_initial_alloc, "",
              "If not empty, the initial alloc will be saved to this path.");
DEFINE_string(ecf_save_final_alloc, "",
              "If not empty, the final alloc will be saved to this path.");
DEFINE_string(ecf_load_alloc, "",
              "If not empty, the initial alloc will be loaded from this path.");
DEFINE_int32(ecf_max_extras, 1, "Maximum number of extra orders added/glued.");
DEFINE_int32(ecf_max_extra_attempts, 1,
             "Maximum number of attempts to add add/glue an extra order.");

namespace drones {

EcfSolver::OrderSplits EcfSolver::IterativeSplit(int num_iter) const {
  // Needed for LoadSplitter::Split.
  std::map<int, int> product_weights;
  for (int p = 0; p < problem_.np(); p++) {
    product_weights[p] = problem_.product(p).m();
  }

  // [order][warehouse] -> coefficient
  using Feedback = std::map<int, std::map<int, double>>;

  OrderSplits res;
  Feedback allocator_feedback;

  util::Allocator::DistFn dist_fn = [&](int o, int w, int p) {
    double dx =
        problem_.warehouse(w).location().x() - problem_.order(o).location().x();
    double dy =
        problem_.warehouse(w).location().y() - problem_.order(o).location().y();
    double d = ceil(sqrt(dx * dx + dy * dy)) + 1;
    double coef = 1.0;
    if (allocator_feedback.count(o) && allocator_feedback.at(o).count(w)) {
      coef = allocator_feedback[o][w];
    }
    return 2.0 * d * coef;
  };

  for (int iter = 1; iter <= num_iter; iter++) {
    LOG(INFO) << absl::Substitute("*** Split iter $0 / $1", iter, num_iter);
    auto alloc =
        iter == 1 && !FLAGS_ecf_load_alloc.empty()
            ? util::Allocator::LoadFromFile(problem_, FLAGS_ecf_load_alloc)
            : util::Allocator::AllocateWithDistFn(problem_, dist_fn);
    CHECK(util::Allocator::VerifyAlloc(problem_, alloc)) << "Invalid alloc!";
    LOG(INFO) << "Successfully allocated.";

    if (iter == 1 && !FLAGS_ecf_save_initial_alloc.empty()) {
      util::Allocator::SaveToFile(alloc, problem_,
                                  FLAGS_ecf_save_initial_alloc);
    }

    if (iter == num_iter && !FLAGS_ecf_save_final_alloc.empty()) {
      util::Allocator::SaveToFile(alloc, problem_, FLAGS_ecf_save_final_alloc);
    }

    res.clear();
    res.resize(problem_.no());
    double avg_wo_coef = 0.0;
    double avg_wo_items = 0;
    for (int o = 0; o < problem_.no(); o++) {
      // [product] -> num items
      std::map<int, int> from_curr_w;
      int prev_w = -1;
      auto calc_and_add_split = [&]() {
        if (from_curr_w.empty()) return;
        CHECK(prev_w >= 0);
        CHECK(prev_w < problem_.nw());
        auto split = util::LoadSplitter::Split(from_curr_w, product_weights,
                                               problem_.m());
        res[o][prev_w] = split;
        allocator_feedback[o][prev_w] =
            static_cast<double>(split.total_times) / split.total_num_items;
        avg_wo_coef += split.total_num_items * allocator_feedback[o][prev_w];
        avg_wo_items += split.total_num_items;
      };
      for (const auto& wp_i : alloc[o]) {
        int w = wp_i.first.first;
        int p = wp_i.first.second;
        int n = wp_i.second;
        CHECK(n > 0);
        if (prev_w != w) {
          calc_and_add_split();
          from_curr_w.clear();
        }
        from_curr_w[p] = n;
        prev_w = w;
      }
      calc_and_add_split();
    }
    avg_wo_coef /= avg_wo_items;
    for (int o = 0; o < problem_.no(); o++) {
      for (int w = 0; w < problem_.nw(); w++) {
        if (allocator_feedback[o].count(w) == 0) {
          allocator_feedback[o][w] = avg_wo_coef;
        }
      }
    }
  }
  LOG(INFO) << "Successfully split.";
  return res;
}

std::vector<int> EcfSolver::GenereteOrderPermutation(
    const OrderSplits& order_splits) const {
  // Let's compute approximate completion time for each order:
  //   * we know exact LOAD time for all items except the first one, since we
  //     know all the alloc pairs and we're always coming from the current
  //     order. But, for the sake of simplicity, let's assume the first one is
  //     coming from the current order as well. Summed up,they are Lall
  //   * we know exact DELIVER time for all items, since we know all the alloc
  //     pairs, all summed up, they are Dall
  //   So, order completion time is: T = Lall + Dall
  //
  // Vector order_durations stores pairs {duration, order}, in ascending order.
  std::vector<std::pair<int, int>> order_durations;
  // Sum of all locations devided by num locations - 1, so that we can easily
  // calculate mean of other locations for each location.
  double mox = 0.0;
  double moy = 0.0;
  if (problem_.no() > 1) {
    for (int o1 = 0; o1 < problem_.no(); o1++) {
      mox += static_cast<double>(problem_.order(o1).location().x()) /
             (problem_.no() - 1);
      moy += static_cast<double>(problem_.order(o1).location().y()) /
             (problem_.no() - 1);
    }
  }

  for (int o = 0; o < problem_.no(); o++) {
    int ox = problem_.order(o).location().x();
    int oy = problem_.order(o).location().y();
    // Mean prevoius order location.
    double mpox = 0;
    double mpoy = 0;
    if (problem_.no() > 1) {
      mpox = mox - static_cast<double>(ox) / (problem_.no() - 1);
      mpoy = moy - static_cast<double>(oy) / (problem_.no() - 1);
    }
    // Mean amoung current warehouses.
    double mwx = 0;
    double mwy = 0;
    int completion_time = 0;
    for (const auto& w_split : order_splits[o]) {
      double wx = problem_.warehouse(w_split.first).location().x();
      double wy = problem_.warehouse(w_split.first).location().y();
      mwx += wx / order_splits[o].size();
      mwy += wy / order_splits[o].size();
      double dx = ox - wx;
      double dy = oy - wy;
      completion_time +=
          (2.0 * w_split.second.total_times - 1.0 / order_splits[o].size()) *
          ceil(sqrt(dx * dx + dy * dy));
    }
    double dx = mpox - mwx;
    double dy = mpoy - mwy;
    completion_time += ceil(sqrt(dx * dx + dy * dy));
    order_durations.push_back({completion_time, o});
  }
  std::sort(order_durations.begin(), order_durations.end());

  std::vector<int> order_perm;
  for (const auto& dur_ord : order_durations) {
    order_perm.push_back(dur_ord.second);
  }
  return order_perm;
}

std::unique_ptr<Solution> EcfSolver::PackSolution(
    EcfSolver::OrderSplits order_splits,
    const std::vector<int>& order_permutation, int* score,
    bool enable_log) const {
  auto solution = std::make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  for (int d = 0; d < problem_.nd(); d++) {
    solution->add_drone_desc();
  }

  // Drones book-keeping.
  std::vector<int> drone_busy_until(problem_.nd(), 0);
  std::vector<Location> drone_location(problem_.nd(),
                                       problem_.warehouse(0).location());

  // For each order, we'll repeatedly match a warehouse and a drone (earliest
  // ending time of LOAD + DELIVER) and pack the drone according to the
  // previously calculated split.
  //
  // Note: Sure, finding the best match depends on the number of product types
  // we're packing since loading of each costs 1 time moment, but we'll assume
  // that the cost of travel is much greater than that!

  // Handle orders one by one.
  int num_order = 0;
  int res_score = 0;
  for (int o : order_permutation) {
    num_order++;
    LOG_IF(INFO, enable_log)
        << absl::Substitute("Order: $0 : $1 / $2", o, num_order, problem_.no());

    // Number of commands executed by each drone for this order. Used to
    // roll-back the commands in case of an unsuccessfull matching, so that an
    // another order can be given a chance.
    std::vector<int> num_drone_commands(problem_.nd(), 0);
    auto backup_drone_busy_until = drone_busy_until;
    auto backup_drone_location = drone_location;

    auto calc_total_pending_items = [&]() {
      int res = 0;
      for (const auto& w_split : order_splits[o]) {
        int total_wh = w_split.second.total_num_items;
        CHECK(total_wh > 0);
        res += total_wh;
      }
      return res;
    };

    int total_pending_items = calc_total_pending_items();
    // CHECK(total_pending_items > 0);
    LOG_IF(INFO, enable_log)
        << absl::Substitute("Initial pending items: $0", total_pending_items);
    // +1 is to make sure the check at the loop beginning is valid in the first
    // pass.
    total_pending_items++;
    // This is to calculate the order's score.
    int max_drone_busy_until = -1;

    auto roll_back = [&]() {
      LOG_IF(INFO, enable_log) << "    Rolling back.";
      for (int d = 0; d < problem_.nd(); d++) {
        while (num_drone_commands[d]--) {
          solution->mutable_drone_desc(d)
              ->mutable_drone_command()
              ->RemoveLast();
        }
      }
      drone_location = backup_drone_location;
      drone_busy_until = backup_drone_busy_until;
      max_drone_busy_until = problem_.t() + 1;  // this will give 0 points.
    };

    bool should_roll_back = false;

    while (!order_splits[o].empty()) {
      int curr_total_pending_items = calc_total_pending_items();
      CHECK(0 < curr_total_pending_items &&
            curr_total_pending_items < total_pending_items);
      total_pending_items = curr_total_pending_items;
      // Find the best (warehouse, drone) match.
      int best_w = -1;
      int best_d = -1;
      int best_t = std::numeric_limits<int>::max();  // Earliest ending time.
      bool match_found = false;
      for (const auto& w_split : order_splits[o]) {
        int w = w_split.first;
        for (int d = 0; d < problem_.nd(); d++) {
          int t = drone_busy_until[d];
          // LOAD dutation = dist to warehouse + 1
          {
            int dx =
                drone_location[d].x() - problem_.warehouse(w).location().x();
            int dy =
                drone_location[d].y() - problem_.warehouse(w).location().y();
            int dist = ceil(sqrt(dx * dx + dy * dy));
            t += dist + 1;
          }
          // DELIVER duration = dist warehouse->order + 1
          {
            int dx = problem_.warehouse(w).location().x() -
                     problem_.order(o).location().x();
            int dy = problem_.warehouse(w).location().y() -
                     problem_.order(o).location().y();
            int dist = ceil(sqrt(dx * dx + dy * dy));
            t += dist + 1;
          }
          // First compare by completion time, then by start time.
          if (t <= problem_.t() &&
              (t < best_t || (t == best_t && drone_busy_until[d] >
                                                 drone_busy_until[best_d]))) {
            best_d = d;
            best_w = w;
            best_t = t;
            match_found = true;
          }
        }
      }

      // If there's no match, we can't fit any commands in the remaining time,
      // so we finish here.
      if (!match_found) {
        // Roll-back.
        LOG_IF(INFO, enable_log) << "    No match found.";
        should_roll_back = true;
        break;
      }

      auto load = order_splits[o][best_w].repeated_splits.front().single_split;

      // Let's GLUE.
      std::vector<int> extra_orders = {o};
      std::map<int, int> total_load = load;
      std::vector<std::map<int, int>> extra_what = {load};
      if (order_splits[o][best_w].total_num_items ==
          order_splits[o][best_w].repeated_splits.front().num_items) {
        // This is the last one.
        std::set<int> taken_products;
        int total_weight = 0;
        for (const auto& p_n : load) {
          total_weight += problem_.product(p_n.first).m() * p_n.second;
          taken_products.insert(p_n.first);
        }
        int cap = problem_.m() - total_weight;

        int dur = 2 * taken_products.size();
        {
          int dx = drone_location[best_d].x() -
                   problem_.warehouse(best_w).location().x();
          int dy = drone_location[best_d].y() -
                   problem_.warehouse(best_w).location().y();
          dur += ceil(sqrt(dx * dx + dy * dy));
          dx = problem_.order(o).location().x() -
               problem_.warehouse(best_w).location().x();
          dy = problem_.order(o).location().y() -
               problem_.warehouse(best_w).location().y();
          dur += ceil(sqrt(dx * dx + dy * dy));
        }

        // Needed for LoadSplitter::Split.
        std::map<int, int> product_weights;
        for (int p = 0; p < problem_.np(); p++) {
          product_weights[p] = problem_.product(p).m();
        }

        int prev_o = o;
        int num_extra_attempts = 0;
        int num_extras = 0;
        bool was = false;
        for (int extra_o : order_permutation) {
          if (extra_o == o) {
            was = true;
            continue;
          }
          if (!was) continue;
          if (num_extras >= FLAGS_ecf_max_extras) break;
          if (num_extra_attempts >= FLAGS_ecf_max_extra_attempts) break;
          num_extra_attempts++;

          std::map<int, int> all_it_needs;
          if (order_splits[extra_o].count(best_w) == 0) continue;
          if (order_splits[extra_o][best_w].total_num_items == 0) continue;
          for (const auto& rs : order_splits[extra_o][best_w].repeated_splits) {
            for (const auto& p_n : rs.single_split) {
              all_it_needs[p_n.first] += p_n.second * rs.times;
            }
          }
          auto rsplit =
              util::LoadSplitter::SplitOnce(all_it_needs, product_weights, cap);
          if (rsplit.single_split.empty()) continue;

          int extra_dur = problem_.dist()
                              .src(problem_.nw() + prev_o)
                              .dst(problem_.nw() + extra_o) +
                          rsplit.single_split.size();
          int extra_w = 0;
          for (const auto& p_n : rsplit.single_split) {
            if (taken_products.count(p_n.first) == 0) {
              taken_products.insert(p_n.first);
              extra_dur++;
            }
            extra_w += problem_.product(p_n.first).m() * p_n.second;
          }
          if (drone_busy_until[best_d] + dur + extra_dur > problem_.t())
            continue;

          // staje...
          dur += extra_dur;
          cap -= extra_w;
          extra_orders.push_back(extra_o);
          extra_what.push_back(rsplit.single_split);
          for (const auto& p_n : rsplit.single_split) {
            total_load[p_n.first] += p_n.second;
            if ((all_it_needs[p_n.first] -= p_n.second) == 0) {
              all_it_needs.erase(p_n.first);
            }
          }
          order_splits[extra_o][best_w] = util::LoadSplitter::Split(
              all_it_needs, product_weights, problem_.m());
          if (order_splits[extra_o][best_w].total_times == 0) {
            order_splits[extra_o].erase(best_w);
          }
          prev_o = extra_o;
          num_extras++;
        }
      }

      // Let's generate the LOAD commands.
      {
        int dx = drone_location[best_d].x() -
                 problem_.warehouse(best_w).location().x();
        int dy = drone_location[best_d].y() -
                 problem_.warehouse(best_w).location().y();
        int dist = ceil(sqrt(dx * dx + dy * dy));
        for (const auto& pi : total_load) {
          if (pi.second < 1) continue;
          num_drone_commands[best_d]++;
          auto* cmd = solution->mutable_drone_desc(best_d)->add_drone_command();
          cmd->set_drone(best_d);
          cmd->set_type(DroneCommand_CommandType_LOAD);
          cmd->set_warehouse(best_w);
          cmd->set_product(pi.first);
          cmd->set_num_items(pi.second);
          cmd->set_start_time(drone_busy_until[best_d] + 1);
          drone_busy_until[best_d] += dist + 1;
          drone_location[best_d] = problem_.warehouse(best_w).location();
          if (drone_busy_until[best_d] > problem_.t()) {
            LOG(INFO) << "    Exceeded!";
            should_roll_back = true;
            break;
          }
          dist = 0;
        }
      }

      if (should_roll_back) break;

      // Let's generate the DELIVER commands.
      for (int i = 0; i < extra_orders.size(); i++) {  //
        int o = extra_orders[i];                       //
        auto& load = extra_what[i];                    //
        bool should_break = false;                     //
        {
          int dx =
              problem_.order(o).location().x() - drone_location[best_d].x();
          int dy =
              problem_.order(o).location().y() - drone_location[best_d].y();
          int dist = ceil(sqrt(dx * dx + dy * dy));
          for (const auto& pi : load) {
            if (pi.second < 1) continue;
            num_drone_commands[best_d]++;
            auto* cmd =
                solution->mutable_drone_desc(best_d)->add_drone_command();
            cmd->set_drone(best_d);
            cmd->set_type(DroneCommand_CommandType_DELIVER);
            cmd->set_order(o);
            cmd->set_product(pi.first);
            cmd->set_num_items(pi.second);
            cmd->set_start_time(drone_busy_until[best_d] + 1);
            drone_busy_until[best_d] += dist + 1;
            drone_location[best_d] = problem_.order(o).location();
            max_drone_busy_until =
                std::max(drone_busy_until[best_d], max_drone_busy_until);
            if (drone_busy_until[best_d] > problem_.t()) {
              LOG(INFO) << "    Exceeded!";
              should_roll_back =
                  true;  // ako je fejlovao, to je zbog orgiginalnog, ostale sam
                         // proveravao, tako da je ok
              should_break = true;
              break;
            }
            dist = 0;
          }
        }
        if (should_break) break;
      }

      if (should_roll_back) break;

      // We haven't rolled-back yet, which doesn't mean we won't roll-back later
      // in this order. Still, we have some backed-up/irrelevant updates to do.

      // no updates here anymore...

      // We don't care if order_splits[o] is dirty if we roll back later in
      // this order, since we're going to the next order.
      order_splits[o][best_w].total_times--;
      order_splits[o][best_w].total_num_items -=
          order_splits[o][best_w].repeated_splits.front().num_items;
      if ((order_splits[o][best_w].repeated_splits.front().times -= 1) == 0) {
        order_splits[o][best_w].repeated_splits.pop_front();
        if (order_splits[o][best_w].total_times == 0) {
          order_splits[o].erase(best_w);
          LOG_IF(INFO, enable_log)
              << absl::Substitute("  (o, w) = ($0, $1): Done.", o, best_w);
        }
      }
    }

    if (should_roll_back) {
      roll_back();
      continue;
    }

    res_score +=
        ceil(100.0 * (problem_.t() - max_drone_busy_until + 1) / problem_.t());
  }
  if (score != nullptr) *score = res_score;
  return solution;
}

std::vector<int> EcfSolver::ImporveOrderPermutation(
    const EcfSolver::OrderSplits& order_splits,
    std::vector<int> order_permutation, int num_iter) const {
  if (num_iter == 0) return order_permutation;
  auto eval = [&](const std::vector<int>& perm) {
    int score = -1;
    auto solution = PackSolution(order_splits, perm, &score, false);
    if (solution == nullptr) {
      score = -1;
    }
    return static_cast<double>(score);
  };

  std::random_device rand_dev;
  std::default_random_engine rand_eng(rand_dev());
  std::uniform_int_distribution<int> rand_idx(0, order_permutation.size() - 1);
  std::uniform_real_distribution<double> rand_prob(0.0, 1.0);

  auto curr = order_permutation;
  double curr_val = eval(curr);
  auto best = curr;
  double best_val = curr_val;
  double init_val = curr_val;
  double T = eval(curr) / 100.0;
  double cooling_rate = pow(0.001 / T, 1.0 / num_iter);
  for (int iter = 0; iter < num_iter; iter++) {
    LOG_EVERY_N(INFO, num_iter / 10) << absl::Substitute(
        "ImporveOrderPermutation(): Iter $0 / $1", iter, num_iter);

    auto neighbour = curr;
    int idx1 = rand_idx(rand_eng);
    int idx2 = rand_idx(rand_eng);
    std::swap(neighbour[idx1], neighbour[idx2]);
    double neighbour_val = eval(neighbour);

    double dval = curr_val - neighbour_val;
    double p = (dval == 0.0 ? 0.5 : exp(dval / T));
    if (dval < 0 || p >= rand_prob(rand_eng)) {
      curr = neighbour;
      curr_val = neighbour_val;
      if (curr_val > best_val) {
        best = curr;
        best_val = curr_val;
      }
    }
    T = T * (1 - cooling_rate);
  }
  LOG(INFO) << absl::Substitute("ImporveOrderPermutation(): Result: $0 -> $1",
                                init_val, best_val);
  return best;
}

std::unique_ptr<Solution> EcfSolver::Solve() {
  auto order_splits = IterativeSplit(FLAGS_ecf_split_iter);
  auto order_permutation = GenereteOrderPermutation(order_splits);
  order_permutation = ImporveOrderPermutation(
      order_splits, std::move(order_permutation), FLAGS_ecf_perm_iter);
  return PackSolution(std::move(order_splits), order_permutation);
}

bool EcfSolver::CanSolve(const ProblemType& problem_type) const { return true; }

}  // namespace drones
