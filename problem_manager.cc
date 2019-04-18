#include "problem_manager.h"

#include <fstream>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_join.h"
#include "absl/strings/substitute.h"
#include "glog/logging.h"

DEFINE_int32(gen_max_ipo, 10000,
             "Max # of items per order. Hashcide says 10^4.");
DEFINE_int32(gen_max_coord, 10000, "Max coordinate value. Don't go over 10^4.");
DEFINE_int32(
    gen_max_nd, 100,
    "Max number of drones. Hashcode says 10^3, but they give up to 30.");
DEFINE_int32(
    gen_max_nw, 100,
    "Max number of warehouses. Hashcode says 10^4, but they give up to 16.");
DEFINE_int32(
    gen_max_np, 400,
    "Max number of products. Hashcode says 10^4, but they give up to 400.");
DEFINE_int32(
    gen_max_no, 2000,
    "Max number of products. Hashcode says 10^4, but they give less than 5K.");
DEFINE_int32(
    gen_max_M, 500,
    "Max drone capacity. Hashcode says 10^4, but they give up to 200.");

namespace drones {

std::unique_ptr<Problem> ProblemManager::GenerateProblem(
    const ProblemType &problem_type, unsigned int seed) {
  auto problem = std::make_unique<Problem>();

  // Random stuff.
  std::default_random_engine rand_eng(seed);
  auto otgen = [&](int a, int b) {
    std::uniform_int_distribution<int> d(a, b);
    return d(rand_eng);
  };

  // Location stuff.
  std::set<std::pair<int, int>> used_locations;
  auto loc_gen = [&]() {
    while (true) {
      int x = otgen(0, FLAGS_gen_max_coord);
      int y = otgen(0, FLAGS_gen_max_coord);
      if (used_locations.count({x, y})) {
        continue;
      }
      used_locations.insert({x, y});
      Location location_proto;
      location_proto.set_x(x);
      location_proto.set_y(y);
      return location_proto;
    }
  };

  problem->set_t(otgen(1, 1000000));
  problem->set_nd(problem_type.nd_1() ? 1 : otgen(1, FLAGS_gen_max_nd));
  problem->set_nw(problem_type.nw_1() ? 1 : otgen(1, FLAGS_gen_max_nw));
  problem->set_np(problem_type.np_1() ? 1 : otgen(1, FLAGS_gen_max_np));
  problem->set_no(problem_type.no_1() ? 1 : otgen(1, FLAGS_gen_max_no));
  problem->set_m(problem_type.m_m() ? 1 : otgen(1, FLAGS_gen_max_M));

  for (int product = 0; product < problem->np(); product++) {
    problem->add_product()->set_m(problem_type.m_m() ? 1
                                                     : otgen(1, problem->m()));
  }

  std::vector<int> total_requested(problem->np(), 0);
  std::vector<int> total_available(problem->np(), 0);

  for (int warehouse = 0; warehouse < problem->nw(); warehouse++) {
    auto *warehouse_proto = problem->add_warehouse();
    *warehouse_proto->mutable_location() = loc_gen();
    for (int product = 0; product < problem->np(); product++) {
      int num_items = problem_type.s0_inf() ? std::numeric_limits<int>::max()
                                            : otgen(0, 10000);
      total_available[product] += num_items;
      warehouse_proto->add_stock(num_items);
    }
  }

  for (int order = 0; order < problem->no(); order++) {
    auto *order_proto = problem->add_order();
    *order_proto->mutable_location() = loc_gen();
    int chosen_product = otgen(0, problem->np() - 1);
    int total_ordered_items = 0;

    for (int product = 0; product < problem->np(); product++) {
      int num_items = 0;
      if (problem_type.ipo_1()) {
        num_items = product == chosen_product ? 1 : 0;
      } else {
        int upper_bound = FLAGS_gen_max_ipo < problem->np()
                              ? FLAGS_gen_max_ipo - total_ordered_items
                              : FLAGS_gen_max_ipo / problem->np();
        num_items = upper_bound < 1
                        ? 0
                        : otgen(product == chosen_product ? 1 : 0, upper_bound);
      }
      total_ordered_items += num_items;
      total_requested[product] += num_items;
      order_proto->add_request(num_items);
    }
  }

  // Make sure that there's enough items of each product available.
  for (int product = 0; product < problem->np(); product++) {
    int to_add = total_requested[product] - total_available[product];
    if (to_add > 0) {
      int warehouse = otgen(0, problem->nw() - 1);
      *problem->mutable_warehouse(warehouse)->mutable_stock()->Mutable(
          product) += to_add;
    }
  }

  *problem->mutable_problem_type() = DetermineProblemType(*problem);
  return problem;
}

std::unique_ptr<Problem> ProblemManager::LoadFromProblemFile(
    const std::string &path) {
  auto problem = std::make_unique<Problem>();
  LOG(INFO) << "Reading from: " << path;
  std::ifstream fin(path);
  if (!fin.good()) {
    LOG(ERROR) << "Failed to open file: " << path;
    return nullptr;
  }
  auto get_int = [&]() {
    int x;
    fin >> x;
    return x;
  };
  get_int();
  get_int();  // Consume R and C.
  problem->set_nd(get_int());
  problem->set_t(get_int());
  problem->set_m(get_int());
  problem->set_np(get_int());

  for (int i = 0; i < problem->np(); i++) {
    problem->add_product()->set_m(get_int());
  }
  problem->set_nw(get_int());

  for (int i = 0; i < problem->nw(); i++) {
    auto *warehouse = problem->add_warehouse();
    warehouse->mutable_location()->set_x(get_int());
    warehouse->mutable_location()->set_y(get_int());
    for (int j = 0; j < problem->np(); j++) {
      warehouse->add_stock(get_int());
    }
  }
  problem->set_no(get_int());
  for (int i = 0; i < problem->no(); i++) {
    auto *order = problem->add_order();
    order->mutable_location()->set_x(get_int());
    order->mutable_location()->set_y(get_int());
    for (int j = 0; j < problem->np(); j++) {
      order->add_request(0);
    }
    int x = get_int();
    for (int j = 0; j < x; j++) {
      int xx = get_int();
      (*order->mutable_request()->Mutable(xx))++;
    }
  }
  fin.close();
  *problem->mutable_problem_type() = DetermineProblemType(*problem);
  return problem;
}

bool ProblemManager::SaveToProblemFile(const Problem &problem,
                                       const std::string &path) {
  LOG(INFO) << "Writing to: " << path;
  std::ofstream fout(path);
  if (!fout.good()) {
    LOG(ERROR) << "Failed to open file: " << path;
    return false;
  }

  fout << absl::Substitute("100000 100000 $0 $1 $2\n", problem.nd(),
                           problem.t(), problem.m());
  fout << problem.np() << std::endl;
  fout << absl::StrJoin(problem.product(), " ",
                        [](std::string *out, const Product &product) {
                          out->append(std::to_string(product.m()));
                        })
       << std::endl;
  fout << problem.nw() << std::endl;
  fout << absl::StrJoin(
      problem.warehouse(), "\n",
      [](std::string *out, const Warehouse &warehouse) {
        out->append(absl::Substitute("$0 $1\n$2\n", warehouse.location().x(),
                                     warehouse.location().y(),
                                     absl::StrJoin(warehouse.stock(), " ")));
      });
  fout << problem.no() << std::endl;
  for (const auto &order : problem.order()) {
    int num_items =
        std::accumulate(order.request().begin(), order.request().end(), 0);
    fout << absl::Substitute("$0 $1\n$2\n", order.location().x(),
                             order.location().y(), num_items);
    for (int product = 0; product < problem.np(); product++) {
      for (int item = 0; item < order.request(product); item++) {
        fout << product << " ";
      }
    }
    fout << std::endl;
  }

  fout.close();
  return true;
}

std::unique_ptr<Problem> ProblemManager::LoadFromProtoFile(
    const std::string &path) {
  auto problem = std::make_unique<Problem>();
  std::ifstream fin(path);
  if (!fin.good()) {
    LOG(ERROR) << "Failed to open file: " << path;
    return nullptr;
  }
  if (!problem->ParseFromIstream(&fin)) {
    LOG(ERROR) << "Failed to parse from istream";
    return nullptr;
  }
  return problem;
}

bool ProblemManager::SaveToProtoFile(const Problem &problem,
                                     const std::string &path) {
  std::ofstream fout(path);
  if (!fout.good()) {
    LOG(ERROR) << "Failed to open file: " << path;
    return false;
  }
  if (!problem.SerializeToOstream(&fout)) {
    LOG(ERROR) << "Failed to serialize to ostream";
    return false;
  }
  fout.close();
  return true;
}

ProblemType ProblemManager::DetermineProblemType(const Problem &problem) {
  ProblemType problem_type;

  problem_type.set_nd_1(problem.nd() == 1);
  problem_type.set_nw_1(problem.nw() == 1);
  problem_type.set_np_1(problem.np() == 1);
  problem_type.set_no_1(problem.no() == 1);

  bool M_m = true;
  for (int product = 1; product < problem.np(); product++) {
    if (problem.product(product).m() != problem.product(0).m()) {
      M_m = false;
      break;
    }
  }
  M_m = M_m && (problem.m() == problem.product(0).m());
  problem_type.set_m_m(M_m);

  bool IPO_1 = true;
  for (int order = 0; order < problem.no() && IPO_1; order++) {
    int num_items = 0;
    for (int product = 0; product < problem.np() && IPO_1; product++) {
      num_items += problem.order(order).request(product);
      if (num_items > 1) {
        IPO_1 = false;
      }
    }
  }
  problem_type.set_ipo_1(IPO_1);

  // [product] -> num_items
  std::map<int, int> total_required;
  for (int o = 0; o < problem.no(); o++) {
    for (int p = 0; p < problem.np(); p++) {
      total_required[p] += problem.order(o).request(p);
    }
  }

  bool S0_inf = true;
  for (int warehouse = 0; warehouse < problem.nw() && S0_inf; warehouse++) {
    for (int product = 0; product < problem.np() && S0_inf; product++) {
      // S0_inf = problem.warehouse(warehouse).stock(product) ==
      //          std::numeric_limits<int>::max();
      S0_inf = problem.warehouse(warehouse).stock(product) >=
               total_required[product];
    }
  }
  problem_type.set_s0_inf(S0_inf);

  return problem_type;
}

}  // namespace drones
