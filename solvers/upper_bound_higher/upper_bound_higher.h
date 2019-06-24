#ifndef SOLVERS_UPPER_BOUND_UPPER_BOUND_HIGHER_H_
#define SOLVERS_UPPER_BOUND_UPPER_BOUND_HIGHER_H_

#include "problem.pb.h"

namespace drones {

class UpperBoundHigher {
 public:
  explicit UpperBoundHigher(const Problem& problem)
      : problem_(problem), upper_bound_(-1) {}
  virtual ~UpperBoundHigher() {}
  int Calc();

 protected:
  Problem problem_;
  int upper_bound_;
};

}  // namespace drones

#endif  //  SOLVERS_UPPER_BOUND_UPPER_BOUND_HIGHER_H_
