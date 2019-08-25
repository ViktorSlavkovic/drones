#include "absl/strings/substitute.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "google/protobuf/text_format.h"
#include "problem.pb.h"
#include "problem_manager.h"
#include "solvers/upper_bound/upper_bound.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

DEFINE_int32(num_gen, 1, "The number of problems to generte.");
DEFINE_string(
    problem_file_pattern, "",
    "Problem file (HashCode format) path pattern. The generated problem will "
    "be saved to this file. Insert \"$0\" where you want the ID number to be "
    "placed (range [0, num_gen)).");
DEFINE_string(
    problem_type, "",
    "Type of the problem being generated (ProblemType proto) as a text proto.");
DEFINE_string(
    problem_info_file, "problems.txt",
    "Information on the generated problems will be saved to this file.");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drones::ProblemType problem_type;
  CHECK(google::protobuf::TextFormat::ParseFromString(FLAGS_problem_type,
                                                      &problem_type))
      << "Unable to parse " << FLAGS_problem_type << " as ProblemType.";

  std::ofstream fout(FLAGS_problem_info_file);
  CHECK(fout.good());
  fout << FLAGS_num_gen << "\n";

  for (int tc = 0; tc < FLAGS_num_gen;) {
    LOG(INFO) << absl::Substitute("==== Generating $0/$1 ====", tc,
                                  FLAGS_num_gen);
    auto problem = drones::ProblemManager::GenerateProblem(problem_type);
    CHECK(problem != nullptr) << "Failed to create the problem object.";
    LOG(INFO) << absl::Substitute(R"(
      Nd = $0
      Np = $1
      Nw = $2
      No = $3
      T  = $4
      ProblemType: $5
    )",
                                  problem->nd(), problem->np(), problem->nw(),
                                  problem->no(), problem->t(),
                                  problem->problem_type().DebugString());
    LOG(INFO) << "Calculating upper bound...";
    int ub = drones::UpperBound(*problem).Calc();
    if (ub < 1) continue;
    std::string file = absl::Substitute(FLAGS_problem_file_pattern, tc);
    fout << absl::Substitute("$0 $1 $2 $3 $4 $5 $6 $7\n", file, problem->nd(),
                             problem->np(), problem->nw(), problem->no(),
                             problem->m(), problem->t(), ub);
    CHECK(drones::ProblemManager::SaveToProblemFile(*problem, file));
    tc++;
  }

  fout.close();
  return 0;
}
