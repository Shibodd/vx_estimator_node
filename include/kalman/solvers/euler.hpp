#ifndef KALMAN_SOLVER_EULER_HPP
#define KALMAN_SOLVER_EULER_HPP

#include <kalman/common_types.hpp>

namespace kalman {
namespace solvers {

template <typename CTSystemModel>
struct EulerSolver {
  double m_max_step;

  EulerSolver(double max_step) : m_max_step(max_step) {

  }

  void operator()(
    double deltaT,
    SystemState<CTSystemModel>& in_out_state,
    const LinearizedSystemOde<CTSystemModel>& ode_fn) const 
  {
    assert(deltaT >= 0);

    if (deltaT <= 0)
      return;

    SystemJacobian<CTSystemModel> _;
    SystemState<CTSystemModel> xdot;

    int iterations = deltaT / m_max_step;
    for (int i = 0; i < iterations; ++i)
      step(m_max_step, in_out_state, ode_fn);

    double remainder = std::fmod(deltaT, m_max_step);
    if (remainder > 0)
      step(m_max_step, in_out_state, ode_fn);
  }

private:
  void step(double deltaT, SystemState<CTSystemModel>& in_out_state, const LinearizedSystemOde<CTSystemModel>& ode_fn) const {
    SystemJacobian<CTSystemModel> _;
    SystemState<CTSystemModel> xdot;
    
    ode_fn(in_out_state, xdot, _);
    in_out_state += xdot * deltaT;
  }
};

}; // namespace solvers
}; // namespace kalman

#endif // !KALMAN_SOLVER_EULER_HPP