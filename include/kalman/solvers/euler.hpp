#ifndef KALMAN_SOLVER_EULER_HPP
#define KALMAN_SOLVER_EULER_HPP

#include <kalman/common_types.hpp>

namespace kalman {
namespace solvers {

template <typename CTSystemModel>
struct EulerSolver {
  void operator()(
    double deltaT, 
    SystemState<CTSystemModel>& in_out_state,
    const LinearizedSystemOde<CTSystemModel>& ode_fn) const 
  {
    
    SystemJacobian<CTSystemModel> _;
    SystemState<CTSystemModel> xdot;
    
    ode_fn(in_out_state, xdot, _);
    in_out_state += xdot * deltaT;
  }
};

}; // namespace solvers
}; // namespace kalman

#endif // !KALMAN_SOLVER_EULER_HPP