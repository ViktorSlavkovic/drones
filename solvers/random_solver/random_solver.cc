#include "solvers/random_solver/random_solver.h"

#include <algorithm>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/strings/str_join.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_int32(random_solver_wait_percentage, 0,
             "The percentage of WAIT commands among all generated commands. "
             "All 4 command percentage flags must sum up to 100.");
DEFINE_int32(random_solver_load_percentage, 48,
             "The percentage of LOAD commands among all generated commands. "
             "All 4 command percentage flags must sum up to 100.");
DEFINE_int32(random_solver_unload_percentage, 5,
             "The percentage of UNLOAD commands among all generated commands. "
             "All 4 command percentage flags must sum up to 100.");
DEFINE_int32(random_solver_deliver_percentage, 47,
             "The percentage of DELIVER commands among all generated commands. "
             "All 4 command percentage flags must sum up to 100.");

namespace drones {

using std::list;
using std::make_pair;
using std::make_unique;
using std::pair;
using std::priority_queue;
using std::uniform_int_distribution;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

RandomSolver::RandomSolver(const Problem& problem) : ProblemSolver(problem) {}

unique_ptr<Solution> RandomSolver::Solve() {
  // Check the command percentage flags' sum.
  CHECK(FLAGS_random_solver_wait_percentage +
            FLAGS_random_solver_load_percentage +
            FLAGS_random_solver_unload_percentage +
            FLAGS_random_solver_deliver_percentage ==
        100)
      << "Command percentage flags must sum up to 100!";

  // A min heap of pairs (t, drone).
  priority_queue<pair<int, int>, vector<pair<int, int>>,
                 std::greater<pair<int, int>>>
      decision_schedule;
  for (int drone = 0; drone < problem_.nd(); drone++) {
    decision_schedule.push(make_pair(1, drone));
  }

  // Random stuff.
  std::random_device rand_dev;
  std::default_random_engine rand_eng(rand_dev());
  uniform_int_distribution<int> rand100(1, 100);
  auto otgen = [&](int a, int b) {
    uniform_int_distribution<int> d(a, b);
    return d(rand_eng);
  };

  // Create a blank solution.
  auto solution = make_unique<Solution>();
  *solution->mutable_problem() = problem_;
  for (int drone = 0; drone < problem_.nd(); drone++) {
    solution->add_drone_desc();
  }

  // How many items of each product type are left to be loaded for delivery?
  // "loaded for delivery" means that only the needed (for any order) products
  // and items count.
  unordered_map<int, int> unreserved_items;
  vector<int> pending_products;
  for (int product = 0; product < problem_.np(); product++) {
    int total_product_items = 0;
    for (int order = 0; order < problem_.no(); order++) {
      int x = problem_.order(order).request(product);
      if (x == 0) continue;
      unreserved_items[product] += x;
      total_product_items += x;
    }
    if (total_product_items > 0) {
      pending_products.push_back(product);
    }
  }

  // How many items of each product type are left to be delivered in each order?
  unordered_map<int, unordered_map<int, int>> unreserved_order_items;
  vector<int> pending_orders;
  for (int order = 0; order < problem_.no(); order++) {
    int total_order_items = 0;
    for (int product = 0; product < problem_.np(); product++) {
      int x = problem_.order(order).request(product);
      if (x == 0) continue;
      unreserved_order_items[order][product] += x;
      total_order_items += x;
    }
    if (total_order_items > 0) {
      pending_orders.push_back(order);
    }
  }

  // How many items of each product type are left unreserved in each warehouse?
  // Product items are reserved on LOAD command start, even if they are going to
  // be unloaded and not delivered. Unloaded items will appear as unlocked at
  // the end of the relevant UNLOAD command.
  unordered_map<int, unordered_map<int, int>> unreserved_warehouse_items;
  vector<int> pending_warehouses;
  for (int warehouse = 0; warehouse < problem_.nw(); warehouse++) {
    int total_warehouse_items = 0;
    for (int product = 0; product < problem_.np(); product++) {
      int x = problem_.warehouse(warehouse).stock(product);
      if (x == 0) continue;
      unreserved_warehouse_items[warehouse][product] += x;
      total_warehouse_items += x;
    }
    if (total_warehouse_items > 0) {
      pending_warehouses.push_back(warehouse);
    }
  }

  // Drone state at the next decision point.
  unordered_map<int, unordered_map<int, int>> drone_inventory;
  vector<int> drone_weight(problem_.nd(), 0);
  // [0, nw-1] are warehouses, [nw, nw+no-1] are orders
  vector<int> drone_loc(problem_.nd(), 0);
  vector<DroneCommand*> drone_prev_cmd(problem_.nd(), nullptr);

  // Auxiliary dist funciton.
  auto dist = [&](int loc1, int loc2) {
    auto actual_loc1 = loc1 < problem_.nw()
                           ? problem_.warehouse(loc1).location()
                           : problem_.order(loc1 - problem_.nw()).location();
    auto actual_loc2 = loc2 < problem_.nw()
                           ? problem_.warehouse(loc2).location()
                           : problem_.order(loc2 - problem_.nw()).location();
    int dx = actual_loc1.x() - actual_loc2.x();
    int dy = actual_loc1.y() - actual_loc2.y();
    return static_cast<int>(ceil(sqrt(dx * dx + dy * dy)));
  };

  // Unload schedule as an unordered_map since -1 in decision_schedule will
  // indicate unloading.
  unordered_map<int, list<DroneCommand*>> unload_schedule;

  auto dbg_print = [&]() {
    LOG(INFO) << "  Next decision schedule: " << decision_schedule.top().first
              << " " << decision_schedule.top().second;
    LOG(INFO) << "  Unreserved items: "
              << absl::StrJoin(unreserved_items, ",", absl::PairFormatter("="));
    LOG(INFO) << "  Pending products: " << absl::StrJoin(pending_products, ",");
    LOG(INFO) << "  Unreserved order items: ";
    for (const auto& p : unreserved_order_items) {
      LOG(INFO) << "    order " << p.first << " : "
                << absl::StrJoin(p.second, ",", absl::PairFormatter("="));
    }
    LOG(INFO) << "  Pending orders: " << absl::StrJoin(pending_orders, ",");
    LOG(INFO) << "  Unreserved warehouse items: ";
    for (const auto& p : unreserved_warehouse_items) {
      LOG(INFO) << "    warehouse " << p.first << " : "
                << absl::StrJoin(p.second, ",", absl::PairFormatter("="));
    }
    LOG(INFO) << "  Pending warehouses: "
              << absl::StrJoin(pending_warehouses, ",");
    LOG(INFO) << "  Drone inventory: ";
    for (const auto& p : drone_inventory) {
      LOG(INFO) << "    drone " << p.first << " : "
                << absl::StrJoin(p.second, ",", absl::PairFormatter("="));
    }
    LOG(INFO) << "  Drone weihgts: " << absl::StrJoin(drone_weight, ",");
    LOG(INFO) << "  Drone loc: " << absl::StrJoin(drone_loc, ",");
  };

  LOG(INFO) << "Starting the schedule loop...";
  int next_log = 0;
  while (!decision_schedule.empty()) {
    // Extract (t, drone).
    int t = decision_schedule.top().first;
    int drone = decision_schedule.top().second;
    if (t >= next_log) {
      next_log += 1000;
      // LOG(INFO) << "t: " << t << " / " << problem_.t();
    }
    if (t > problem_.t()) {
      LOG(WARNING)
          << "Command start time exceeds simulation time, breaking here.";
      break;
    }
    decision_schedule.pop();

    // Handle unloading.
    // TODO(viktors): Make this nicer, sure -1 ensures that unloading happens
    // before everything else, as it should be, but it's ugly to use
    // decision_schedule for that, so a common schedule should be made.
    if (drone == -1) {
      for (auto* cmd : unload_schedule[t]) {
        if (unreserved_warehouse_items.count(cmd->warehouse()) == 0) {
          pending_warehouses.push_back(cmd->warehouse());
        }
        unreserved_warehouse_items[cmd->warehouse()][cmd->product()] +=
            cmd->num_items();
      }
      continue;
    }

    // There's no need to schedule any more commands.
    if (pending_orders.empty()) {
      continue;
    }

    auto* proto_cmd = solution->mutable_drone_desc(drone)->add_drone_command();

    // Decide on the operation.
    {
    choose_command_type:
      int cap = 0;
      int cmd_rand = rand100(rand_eng);
      // Generate a WAIT.
      if (cmd_rand <= (cap += FLAGS_random_solver_wait_percentage)) {
        // Everything's taken, just deliver it if you have it, otherwise, you
        // should wait.
        if (pending_products.empty() && !drone_inventory[drone].empty()) {
          // LOG(INFO) << "WAIT -> DELIVER";
          goto gen_deliver;
        }
      gen_wait:
        // LOG(INFO) << "Generating WAIT    : drone " << drone << " t " << t
        //          << " ol " << pending_orders.size();
        // dbg_print();

        proto_cmd->set_type(DroneCommand_CommandType_WAIT);
        proto_cmd->set_start_time(t);
        proto_cmd->set_drone(drone);
        proto_cmd->set_duration(1);
        drone_prev_cmd[drone] = proto_cmd;
        decision_schedule.push(make_pair(t + 1, drone));

        // LOG(INFO) << "Impact: ";
        // dbg_print();
        continue;
      }
      // Generate a LOAD.
      if (cmd_rand <= (cap += FLAGS_random_solver_load_percentage)) {
        // Everything's taken, just deliver it, if you have it, otherwise, you
        // should wait.
        if (pending_products.empty()) {
          if (drone_inventory[drone].empty()) {
            // LOG(INFO) << "LOAD -> WAIT";
            goto gen_wait;
          } else {
            // LOG(INFO) << "LOAD -> DELIVER";
            goto gen_deliver;
          }
        }

        // LOG(INFO) << "Generating LOAD    : drone " << drone << " t " << t
        //          << " ol " << pending_orders.size();
        // dbg_print();

        proto_cmd->set_type(DroneCommand_CommandType_LOAD);
        proto_cmd->set_start_time(t);
        proto_cmd->set_drone(drone);

        // Choose a random warehouse.
        int warehouse_idx = otgen(0, pending_warehouses.size() - 1);
        int warehouse = pending_warehouses[warehouse_idx];
        // Take whatever you can, but keep it meaningful (only items which are
        // needed for some oreder).
        bool success = false;
        for (int ntry = 0; ntry < 10; ntry++) {
          // Choose a random product, check whether it exists in the warehouse,
          // and how many you can take. If you fail to take any, for any reason,
          // it's a failure, try again.
          int product_idx = otgen(0, pending_products.size() - 1);
          int product = pending_products[product_idx];
          if (unreserved_warehouse_items[warehouse].count(product) == 0) {
            continue;
          }
          int cantake = (problem_.m() - drone_weight[drone]) /
                        problem_.product(product).m();
          cantake =
              std::min(cantake, unreserved_warehouse_items[warehouse][product]);
          if (unreserved_items.count(product) == 0) {
            continue;
          }
          cantake = std::min(cantake, unreserved_items[product]);
          if (cantake == 0) {
            continue;
          }

          // Set the proto.
          proto_cmd->set_warehouse(warehouse);
          proto_cmd->set_product(product);
          proto_cmd->set_num_items(cantake);

          // Take it - book-keeping.
          drone_weight[drone] += cantake * problem_.product(product).m();
          drone_inventory[drone][product] += cantake;
          if ((unreserved_warehouse_items[warehouse][product] -= cantake) ==
              0) {
            unreserved_warehouse_items[warehouse].erase(product);
            if (unreserved_warehouse_items[warehouse].empty()) {
              unreserved_warehouse_items.erase(warehouse);
              pending_warehouses[warehouse_idx] = pending_warehouses.back();
              pending_warehouses.pop_back();
            }
          }
          if ((unreserved_items[product] -= cantake) == 0) {
            unreserved_items.erase(product);
            pending_products[product_idx] = pending_products.back();
            pending_products.pop_back();
          }

          int d = dist(warehouse, drone_loc[drone]);
          decision_schedule.push(make_pair(t + d + 1, drone));
          // If the command will exceed simulation time, just remove it from the
          // solution, to keep it valid.
          if (t + d > problem_.t()) {
            solution->mutable_drone_desc(drone)
                ->mutable_drone_command()
                ->RemoveLast();
          }
          drone_loc[drone] = warehouse;
          drone_prev_cmd[drone] = proto_cmd;
          success = true;
          break;
        }

        if (!success) {
          solution->mutable_drone_desc(drone)
              ->mutable_drone_command()
              ->RemoveLast();
          decision_schedule.push({t, drone});
        }
        // LOG(INFO) << "Success " << success;
        // dbg_print();
        continue;
      }
      // Generate a UNLOAD.
      if (cmd_rand <= (cap += FLAGS_random_solver_unload_percentage)) {
        // Everything's taken, just deliver it, if you have it, otherwise, you
        // should wait.
        if (pending_products.empty()) {
          if (drone_inventory.count(drone) == 0) {
            // LOG(INFO) << "UNLOAD -> WAIT";
            goto gen_wait;
          } else {
            // LOG(INFO) << "UNLOAD -> DELIVER";
            goto gen_deliver;
          }
        }

        // Nothing to UNLOAD, do something else.
        if (drone_inventory[drone].empty()) {
          // LOG(INFO) << "UNLOAD -> CHOOSE AGAIN";
          goto choose_command_type;
        }

        // LOG(INFO) << "Generating UNLOAD  : drone " << drone << " t " << t
        //          << " ol " << pending_orders.size();
        // dbg_print();

        proto_cmd->set_type(DroneCommand_CommandType_UNLOAD);
        proto_cmd->set_start_time(t);
        proto_cmd->set_drone(drone);

        // Choose a random warehouse.
        int warehouse = otgen(0, problem_.nw() - 1);
        // 10 is to make sure finding a product is not too slow.
        int product_idx = otgen(
            0,
            std::min(10, static_cast<int>(drone_inventory[drone].size()) - 1));
        int product = -1;  // drone_inventory[drone].begin()->first;
        for (const auto& p : drone_inventory[drone]) {
          if (product_idx-- == 0) {
            product = p.first;
            break;
          }
        }
        int num_items = otgen(1, drone_inventory[drone][product]);

        // Set the proto.
        proto_cmd->set_warehouse(warehouse);
        proto_cmd->set_product(product);
        proto_cmd->set_num_items(num_items);

        // Leave it - book-keeping.
        drone_weight[drone] -= num_items * problem_.product(product).m();
        if ((drone_inventory[drone][product] -= num_items) == 0) {
          drone_inventory[drone].erase(product);
          // TODO: Check if this is needed at all, I think it's not.
          if (drone_inventory[drone].empty()) {
            drone_inventory.erase(drone);
          }
        }

        // unreserved_warehouse and pending_warehouses will be updated using
        // unloading_schedule. Let's schedule it:
        int d = dist(warehouse, drone_loc[drone]);
        if (unload_schedule.count(t + d + 1) == 0) {
          decision_schedule.push(make_pair(t + d + 1, -1));
        }
        unload_schedule[t + d + 1].push_back(proto_cmd);

        if (unreserved_items[product] == 0) {
          pending_products.push_back(product);
        }
        unreserved_items[product] += num_items;

        decision_schedule.push(make_pair(t + d + 1, drone));
        // If the command will exceed simulation time, just remove it from the
        // solution, to keep it valid.
        if (t + d > problem_.t()) {
          solution->mutable_drone_desc(drone)
              ->mutable_drone_command()
              ->RemoveLast();
        }
        drone_loc[drone] = warehouse;
        drone_prev_cmd[drone] = proto_cmd;
        // LOG(INFO) << "Impact: ";
        // dbg_print();
        continue;
      }
      // Generate a DELIVER.
      if (cmd_rand <= (cap += FLAGS_random_solver_deliver_percentage)) {
        if (drone_inventory.count(drone) == 0) {
          // Nothing to DELIVER, do something else.
          // LOG(INFO) << "DELIVER -> CHOOSE AGAIN";
          goto choose_command_type;
        }

      gen_deliver:

        // LOG(INFO) << "Generating DELIVER : drone " << drone << " t " << t
        //          << " ol " << pending_orders.size();
        // dbg_print();

        proto_cmd->set_type(DroneCommand_CommandType_DELIVER);
        proto_cmd->set_start_time(t);
        proto_cmd->set_drone(drone);

        // Since it can easily happen that the drone is not carrying any items
        // needed for the randomly choosen order, let's do two layers of trying,
        // one for picking an order and the other one for picking a product.
        bool success = false;
        for (int ntry = 0; ntry < 10 && !success; ntry++) {
          // Choose a random order.
          int order_idx = otgen(0, pending_orders.size() - 1);
          int order = pending_orders[order_idx];

          for (int mtry = 0; mtry < 10; mtry++) {
            // 10 is to make sure finding a product is not too slow.
            int product_idx = otgen(
                0,
                std::min(10,
                         static_cast<int>(drone_inventory[drone].size()) - 1));
            int product = -1;  // drone_inventory[drone].begin()->first;
            for (const auto& p : drone_inventory[drone]) {
              if (product_idx-- == 0) {
                product = p.first;
                break;
              }
            }
            if (unreserved_order_items[order].count(product) == 0) {
              continue;
            }
            // At this point, we have a match, order and product are valid.
            // Let's leave all we can.
            int num_items = std::min(unreserved_order_items[order][product],
                                     drone_inventory[drone][product]);

            // Set the proto.
            proto_cmd->set_order(order);
            proto_cmd->set_product(product);
            proto_cmd->set_num_items(num_items);

            // Deliver it - book-keeping.
            drone_weight[drone] -= num_items * problem_.product(product).m();
            if ((drone_inventory[drone][product] -= num_items) == 0) {
              drone_inventory[drone].erase(product);
              // TODO: Check if this is needed at all, I think it's not.
              if (drone_inventory[drone].empty()) {
                drone_inventory.erase(drone);
              }
            }

            if ((unreserved_order_items[order][product] -= num_items) == 0) {
              unreserved_order_items[order].erase(product);
              if (unreserved_order_items[order].empty()) {
                unreserved_order_items.erase(order);
                pending_orders[order_idx] = pending_orders.back();
                pending_orders.pop_back();
              }
            }

            int d = dist(order + problem_.nw(), drone_loc[drone]);
            decision_schedule.push(make_pair(t + d + 1, drone));
            // If the command will exceed simulation time, just remove it from
            // the solution, to keep it valid.
            if (t + d > problem_.t()) {
              solution->mutable_drone_desc(drone)
                  ->mutable_drone_command()
                  ->RemoveLast();
            }
            drone_loc[drone] = order + problem_.nw();
            drone_prev_cmd[drone] = proto_cmd;
            success = true;
            break;
          }
        }

        if (!success) {
          solution->mutable_drone_desc(drone)
              ->mutable_drone_command()
              ->RemoveLast();
          decision_schedule.push({t, drone});
        }

        // LOG(INFO) << "Succcess: " << success;
        // dbg_print();
        continue;
      }
      CHECK(false);
    }
  }

  return solution;
}

}  // namespace drones
