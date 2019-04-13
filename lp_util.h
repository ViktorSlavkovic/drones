#ifndef LP_UTIL_H_
#define LP_UTIL_H_

#include <unordered_map>
#include <string>

#include "google/protobuf/message.h"

namespace drones {
namespace lin_prog {

// Polynomial consisting of the decision variables.
struct Polynomial {
  // Maps decision variable name (hash) to it's coefficient. "const" is used
  // as a key for the constant factor.
  std::unordered_map<std::string, double> coef;

  // Overloading basic operators for the Polynomial type.
  Polynomial& operator*=(double rhs) {
    for (auto& p : coef) {
      p.second *= rhs;
    }
    return *this;
  }

  Polynomial& operator+=(const Polynomial& rhs) {
    for (const auto& p : rhs.coef) {
      coef[p.first] += p.second;
    }
    return *this;
  }
};

std::string& SavyProtoHash(const google::protobuf::Message& proto);

}  // namespace lin_prog
}  // namespace drones

#endif  // LP_UTIL_H_