#ifndef SOLVERS_UTIL_LP_UTIL_H_
#define SOLVERS_UTIL_LP_UTIL_H_

#include <unordered_map>
#include <string>

#include "google/protobuf/message.h"

namespace drones {
namespace util {
namespace lp {

// Linear combination of the decision variables.
struct LinComb {
  // Maps decision variable name (hash) to it's coefficient. "const" is used
  // as a key for the constant factor.
  std::unordered_map<std::string, double> coef;

  // Overloading basic operators for the Polynomial type.
  LinComb& operator*=(double rhs) {
    for (auto& p : coef) {
      p.second *= rhs;
    }
    return *this;
  }

  LinComb& operator+=(const LinComb& rhs) {
    for (const auto& p : rhs.coef) {
      coef[p.first] += p.second;
    }
    return *this;
  }
};

std::string& SavyProtoHash(const google::protobuf::Message& proto);

}  // namespace lp
}  // namespace util
}  // namespace drones

#endif  // SOLVERS_UTIL_LP_UTIL_H_
