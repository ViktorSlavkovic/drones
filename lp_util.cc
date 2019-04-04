#include "lp_util.h"

#include <string>

#include "glog/logging.h"
#include "google/protobuf/message.h"

namespace drones {
namespace lin_prog {

std::string& SavyProtoHash(const google::protobuf::Message& proto) {
  static std::string hashval;
  CHECK(proto.SerializeToString(&hashval)) << "Failed to serialize.";
  return hashval;
}

}  // namespace lin_prog
}  // namespace drones
