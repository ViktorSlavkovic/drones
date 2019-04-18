#include "solvers/util/lp_util.h"

#include <string>

#include "glog/logging.h"
#include "google/protobuf/message.h"

namespace drones {
namespace util {
namespace lp {

std::string& SavyProtoHash(const google::protobuf::Message& proto) {
  static std::string hashval;
  CHECK(proto.SerializeToString(&hashval)) << "Failed to serialize.";
  return hashval;
}

}  // namespace lp
}  // namespace util
}  // namespace drones
