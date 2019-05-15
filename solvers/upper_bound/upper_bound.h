#ifndef SOLVERS_UPPER_BOUND_UPPER_BOUND_H_
#define SOLVERS_UPPER_BOUND_UPPER_BOUND_H_

#include "problem.pb.h"

namespace drones {

class UpperBound {
 public:
  explicit UpperBound(const Problem& problem)
      : problem_(problem), upper_bound_(-1) {}
  virtual ~UpperBound() {}
  int Calc();

 protected:
  Problem problem_;
  int upper_bound_;
};

}  // namespace drones

#endif  //  SOLVERS_UPPER_BOUND_UPPER_BOUND_H_
