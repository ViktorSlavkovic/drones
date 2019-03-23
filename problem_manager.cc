#include "problem_manager.h"

#include "glog/logging.h"

#include <fstream>

namespace drones {

std::unique_ptr<Problem> ProblemManager::LoadProblemFromFile(
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
  return problem;
}

// TODO: Implement.
std::unique_ptr<Problem> ProblemManager::GenerateProblem() { return nullptr; }

}  // namespace drones
