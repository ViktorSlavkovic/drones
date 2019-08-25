#ifndef SOLVERS_UPPER_BOUND_UPPER_BOUND_LOW_H_
#define SOLVERS_UPPER_BOUND_UPPER_BOUND_LOW_H_

#include "problem.pb.h"

namespace drones {

class UpperBoundLow {
 public:
  explicit UpperBoundLow(const Problem& problem)
      : problem_(problem), upper_bound_(-1) {}
  virtual ~UpperBoundLow() {}
  int Calc();

 protected:
  Problem problem_;
  int upper_bound_;
};

}  // namespace drones

#endif  //  SOLVERS_UPPER_BOUND_UPPER_BOUND_LOW_H_
