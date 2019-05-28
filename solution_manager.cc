#include "solution_manager.h"

#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/strings/substitute.h"
#include "glog/logging.h"

namespace drones {

// Used to keep track of drones', warehouses' and customres' inventories.
class Inventory final {
 public:
  // Creates an empty inventory.
  Inventory() {}
  // Creates an inventory with the state given as a map: product -> number of
  // items.
  explicit Inventory(const std::map<int, int> inventory)
      : inventory_(inventory) {
    for (auto product : inventory) {
      CHECK(product.second >= 0) << "Nubmber of items can't be negative!";
    }
  }
  // Returns the whole inventory as a map: product -> number of items.
  std::map<int, int> GetSnapshot() const { return inventory_; }
  // Returns item count of the product.
  int GetProductState(int product) const {
    if (inventory_.find(product) == inventory_.end()) {
      return 0;
    }
    return inventory_.at(product);
  }
  // Returns the number of different products, not the total number of items!
  int GetNumProducts() const { return inventory_.size(); }
  // If diff > 0, then diff items of the product are added and if diff < 0,
  // then |diff| items of the product are taken.
  void Update(int product, int diff) {
    inventory_[product] += diff;
    CHECK(inventory_[product] >= 0) << "Can't take more than there is!";
    if (inventory_[product] == 0) {
      inventory_.erase(product);
    }
  }

 private:
  // Maps products to the numbers of their items.
  std::map<int, int> inventory_;
};

// An auxiliary function to compute duration of a command.
int CommandDuration(const Location& src, const Problem& problem,
                    const DroneCommand& cmd) {
  if (cmd.type() == DroneCommand_CommandType_WAIT) {
    return cmd.duration();
  }
  Location dst;
  switch (cmd.type()) {
    case DroneCommand_CommandType_DELIVER:
      dst = problem.order(cmd.order()).location();
      break;
    case DroneCommand_CommandType_LOAD:
    case DroneCommand_CommandType_UNLOAD:
      dst = problem.warehouse(cmd.warehouse()).location();
      break;
    default:
      CHECK(false) << "Invalid command type!";
  }
  int dx = src.x() - dst.x();
  int dy = src.y() - dst.y();
  return ceil(sqrt(dx * dx + dy * dy)) + 1;
}

bool SolutionManager::Simulate(const Solution& solution, int* score) {
  // Maps time moments to vectors of the commands which are starting at those
  // moments. All the time moments in which unloading or delivery should be
  // performed are added here, even if with empty vectors.
  std::map<int, std::vector<DroneCommand>> timeline;
  int max_cpd = 0;  /* max commands per drone */
  for (const auto& drone_desc : solution.drone_desc()) {
    max_cpd = std::max(max_cpd, drone_desc.drone_command_size());
    for (const auto& cmd : drone_desc.drone_command()) {
      timeline[cmd.start_time()].push_back(cmd);
    }
  }
  LOG(INFO) << "Max commands per drone: " << max_cpd;

  const auto& problem = solution.problem();
  // Drone's location next time when it's ready.
  std::vector<Location> drone_loc(problem.nd(),
                                  problem.warehouse(0).location());
  // The last time moment in which the drone is busy.
  // Important: Time moments start from 1, 0 is used as a sentinel.
  std::vector<int> drone_busy_until(problem.nd(), 0);
  // Drone's inventory next time when it's ready.
  std::vector<Inventory> drone_inventory(problem.nd());
  // Drone's weight next time when it's ready.
  std::vector<int> drone_weight(problem.nd());
  // Loading schedule: mapped by time moments, the vectors represent all the
  // LOAD commands finishing at some moment as (warehouse, product, num_items)
  // triplets.
  std::map<int, std::vector<std::tuple<int, int, int>>> load_schedule;
  // Unloading schedule: mapped by time moments, the vectors represent all the
  // UNLOAD commands finishing at some moment as (warehouse, product,
  // num_items) triplets.
  std::map<int, std::vector<std::tuple<int, int, int>>> unload_schedule;
  // Delivery schedule: mapped by time moments, the vectors represent all the
  // DELIVER commands finishing at some moment as (order, product, num_items)
  // triplets.
  std::map<int, std::vector<std::tuple<int, int, int>>> deliver_schedule;
  // Current warehouse inventory.
  std::vector<Inventory> warehouse_inventory(problem.nw());
  for (int wh = 0; wh < problem.nw(); wh++) {
    for (int p = 0; p < problem.np(); p++) {
      warehouse_inventory[wh].Update(p, problem.warehouse(wh).stock(p));
    }
  }
  // Number of items LEFT (to be delivered) in an order.
  std::vector<Inventory> order_inventory(problem.no());
  for (int ord = 0; ord < problem.no(); ord++) {
    for (int prod = 0; prod < problem.np(); prod++) {
      order_inventory[ord].Update(prod, problem.order(ord).request(prod));
    }
  }

  int res = 0;  // The score.
  // Passing through all the relevant moments.
  LOG(INFO) << "Simulate: Passing through all the relevant moments.";
  for (const auto& moment : timeline) {
    if (moment.first > problem.t()) {
      LOG(ERROR) << "Simultaion time exceeded.";
      return false;
    }
    // Handle commands.
    for (const auto& cmd : moment.second) {
      // Check the command timing.
      if (drone_busy_until[cmd.drone()] > moment.first - 1) {
        LOG(ERROR) << "A single drone's commands can't overlap!";
        return false;
      }
      if (drone_busy_until[cmd.drone()] < moment.first - 1) {
        LOG(ERROR) << "Drones can't be idle, add WAIT commands to fill in the "
                   << "gaps.";
        return false;
      }

      // Mark the drone as busy.
      drone_busy_until[cmd.drone()] +=
          CommandDuration(drone_loc[cmd.drone()], problem, cmd);

      switch (cmd.type()) {
        case DroneCommand_CommandType_WAIT: {
          // Nothing to be done here.
          break;
        }
        case DroneCommand_CommandType_LOAD: {
          load_schedule[drone_busy_until[cmd.drone()]].push_back(
              std::make_tuple(cmd.warehouse(), cmd.product(), cmd.num_items()));
          drone_inventory[cmd.drone()].Update(cmd.product(), cmd.num_items());
          drone_weight[cmd.drone()] +=
              cmd.num_items() * problem.product(cmd.product()).m();
          if (drone_weight[cmd.drone()] > problem.m()) {
            LOG(ERROR) << "Drone capacity exceeded.";
            return false;
          }
          // Make sure there's an entry in the timeline, even an empty one, in
          // order to ensure the loading will be handled.
          timeline[drone_busy_until[cmd.drone()]];
          drone_loc[cmd.drone()] =
              problem.warehouse(cmd.warehouse()).location();
          break;
        }
        case DroneCommand_CommandType_UNLOAD: {
          if (drone_inventory[cmd.drone()].GetProductState(cmd.product()) <
              cmd.num_items()) {
            LOG(ERROR) << "Can't unload more than the drone is carrying.";
            return false;
          }
          drone_inventory[cmd.drone()].Update(cmd.product(), -cmd.num_items());
          drone_weight[cmd.drone()] -=
              cmd.num_items() * problem.product(cmd.product()).m();
          unload_schedule[drone_busy_until[cmd.drone()]].push_back(
              std::make_tuple(cmd.warehouse(), cmd.product(), cmd.num_items()));
          // Make sure there's an entry in the timeline, even an empty one, in
          // order to ensure the unloading will be handled.
          timeline[drone_busy_until[cmd.drone()]];
          drone_loc[cmd.drone()] =
              problem.warehouse(cmd.warehouse()).location();
          break;
        }
        case DroneCommand_CommandType_DELIVER: {
          if (drone_inventory[cmd.drone()].GetProductState(cmd.product()) <
              cmd.num_items()) {
            LOG(ERROR) << "Can't deliver more than the drone is carrying.";
            return false;
          }
          drone_inventory[cmd.drone()].Update(cmd.product(), -cmd.num_items());
          drone_weight[cmd.drone()] -=
              cmd.num_items() * problem.product(cmd.product()).m();
          deliver_schedule[drone_busy_until[cmd.drone()]].push_back(
              std::make_tuple(cmd.order(), cmd.product(), cmd.num_items()));
          // Make sure there's an entry in the timeline, even an empty one, in
          // order to ensure the delivery will be handled.
          timeline[drone_busy_until[cmd.drone()]];

          drone_loc[cmd.drone()] = problem.order(cmd.order()).location();
          break;
        }
      }
    }

    // Do loading, unloading and delivery.
    for (const auto& x : load_schedule[moment.first]) {
      if (warehouse_inventory[std::get<0>(x)].GetProductState(std::get<1>(x)) <
          std::get<2>(x)) {
        LOG(ERROR) << "Not enough items in the warehouse.";
        return false;
      }
      warehouse_inventory[std::get<0>(x)].Update(std::get<1>(x),
                                                 -std::get<2>(x));
    }

    for (const auto& x : unload_schedule[moment.first]) {
      warehouse_inventory[std::get<0>(x)].Update(std::get<1>(x),
                                                 std::get<2>(x));
    }

    for (const auto& x : deliver_schedule[moment.first]) {
      if (order_inventory[std::get<0>(x)].GetProductState(std::get<1>(x)) <
          std::get<2>(x)) {
        LOG(ERROR) << "Can't deliver more than it's ordered.";
        return false;
      }
      order_inventory[std::get<0>(x)].Update(std::get<1>(x), -std::get<2>(x));
      if (order_inventory[std::get<0>(x)].GetNumProducts() == 0) {
        // LOG(INFO) << "Order " << std::get<0>(x)
        //          << " completed at t = " << moment.first;
        res += ceil(((problem.t() - moment.first + 1) * 100.0) / problem.t());
      }
    }
  }

  // Check for idle ending.
  for (int t : drone_busy_until) {
    if (t != problem.t()) {
      LOG(WARNING) << "Idle ending detected.";
      break;
    }
  }

  if (score != nullptr) {
    *score = res;
  }
  return true;
}

std::unique_ptr<Solution> SolutionManager::LoadFromSolutionFile(
    const Problem& problem, const std::string& path) {
  auto solution = std::make_unique<Solution>();
  LOG(INFO) << "Reading from: " << path;
  std::ifstream fin(path);
  if (!fin.good()) {
    LOG(ERROR) << "Failed to open file: " << path;
    return nullptr;
  }
  solution->set_allocated_problem(new Problem(problem));
  for (int i = 0; i < problem.nd(); i++) {
    solution->add_drone_desc();
  }
  auto get_int = [&]() {
    int x;
    fin >> x;
    return x;
  };

  std::vector<int> next_drone_command(problem.nd(), 1);
  std::vector<Location> next_drone_location(problem.nd(),
                                            problem.warehouse(0).location());

  int nc = get_int();
  LOG(INFO) << "Reading " << nc << " commands.";
  while (nc--) {
    DroneCommand cmd;
    cmd.set_drone(get_int());
    char c = fin.get();
    while (isspace(c)) c = fin.get();
    switch (tolower(c)) {
      case 'l': {
        cmd.set_type(DroneCommand_CommandType_LOAD);
        cmd.set_warehouse(get_int());
        cmd.set_product(get_int());
        cmd.set_num_items(get_int());
        break;
      }
      case 'u': {
        cmd.set_type(DroneCommand_CommandType_UNLOAD);
        cmd.set_warehouse(get_int());
        cmd.set_product(get_int());
        cmd.set_num_items(get_int());
        break;
      }
      case 'd': {
        cmd.set_type(DroneCommand_CommandType_DELIVER);
        cmd.set_order(get_int());
        cmd.set_product(get_int());
        cmd.set_num_items(get_int());
        break;
      }
      case 'w': {
        cmd.set_type(DroneCommand_CommandType_WAIT);
        cmd.set_duration(get_int());
        break;
      }
    }
    cmd.set_start_time(next_drone_command[cmd.drone()]);
    Location next_location = next_drone_location[cmd.drone()];
    if (cmd.has_warehouse()) {
      next_location = problem.warehouse(cmd.warehouse()).location();
    }
    if (cmd.has_order()) {
      next_location = problem.order(cmd.order()).location();
    }
    int cmd_duration =
        CommandDuration(next_drone_location[cmd.drone()], problem, cmd);
    next_drone_command[cmd.drone()] += cmd_duration;
    next_drone_location[cmd.drone()] = next_location;
    *solution->mutable_drone_desc(cmd.drone())->add_drone_command() = cmd;
  }

  fin.close();
  return solution;
}

bool SolutionManager::SaveToSolutionFile(const Solution& solution,
                                         const std::string& path) {
  std::ofstream fout(path);
  if (!fout.good()) {
    LOG(ERROR) << "Failed to open file: " << path;
    return false;
  }
  int num_drone_cmds = 0;
  for (const auto& dd : solution.drone_desc()) {
    num_drone_cmds += dd.drone_command_size();
  }
  fout << num_drone_cmds << std::endl;
  for (const auto& dd : solution.drone_desc()) {
    for (const auto& cmd : dd.drone_command()) {
      switch (cmd.type()) {
        case DroneCommand_CommandType_WAIT: {
          fout << absl::Substitute("$0 W $1\n", cmd.drone(), cmd.duration());
          break;
        }
        case DroneCommand_CommandType_LOAD: {
          fout << absl::Substitute("$0 L $1 $2 $3\n", cmd.drone(),
                                   cmd.warehouse(), cmd.product(),
                                   cmd.num_items());
          break;
        }
        case DroneCommand_CommandType_UNLOAD: {
          fout << absl::Substitute("$0 U $1 $2 $3\n", cmd.drone(),
                                   cmd.warehouse(), cmd.product(),
                                   cmd.num_items());
          break;
        }
        case DroneCommand_CommandType_DELIVER: {
          fout << absl::Substitute("$0 D $1 $2 $3\n", cmd.drone(), cmd.order(),
                                   cmd.product(), cmd.num_items());
          break;
        }
      }
    }
  }
  fout.close();
  return true;
}

std::unique_ptr<Solution> SolutionManager::LoadFromProtoFile(
    const std::string& path) {
  auto solution = std::make_unique<Solution>();
  std::ifstream fin(path);
  if (!fin.good()) {
    LOG(ERROR) << "Failed to open file: " << path;
    return nullptr;
  }
  if (!solution->ParseFromIstream(&fin)) {
    LOG(ERROR) << "Failed to parse from istream";
    return nullptr;
  }
  return solution;
}

bool SolutionManager::SaveToProtoFile(const Solution& solution,
                                      const std::string& path) {
  std::ofstream fout(path);
  if (!fout.good()) {
    LOG(ERROR) << "Failed to open file: " << path;
    return false;
  }
  if (!solution.SerializeToOstream(&fout)) {
    LOG(ERROR) << "Failed to serialize to ostream";
    return false;
  }
  fout.close();
  return true;
}

}  // namespace drones
