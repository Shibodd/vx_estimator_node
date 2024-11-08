#ifndef KALMAN_FILTERS_EKF_HPP
#define KALMAN_FILTERS_EKF_HPP

#include <kalman/common_types.hpp>

namespace kalman {
namespace filters {

template <typename CTSystemModel, typename OdeSolver>
static void ct_ekf_predict(
    typename CTSystemModel::FloatT deltaT,
    const CTSystemModel& mdl,
    const InputVector<CTSystemModel>& u,
    const SystemCovariance<CTSystemModel>& Q,
    const OdeSolver& solver,
    SystemState<CTSystemModel>& in_out_state)
{
  LinearizedSystemOde<CTSystemModel> ode_fn = [&u, &mdl, &Q](const SystemState<CTSystemModel>& ss, SystemState<CTSystemModel>& out_ss_dot, SystemJacobian<CTSystemModel>& out_jacobian) {
    auto x = system_state_get_state<CTSystemModel>(ss);
    auto P = system_state_get_covariance<CTSystemModel>(ss);

    // Compute xdot and store into the state component of out_ss_dot
    system_state_get_state<CTSystemModel>(out_ss_dot) = mdl.xdot(x, u);

    // Compute the model jacobian
    out_jacobian = mdl.xdot_jacobian(x, u);

    // Compute Pdot and store into the covariance component of out_state_dot
    system_state_get_covariance<CTSystemModel>(out_ss_dot) = (
        out_jacobian * P
        + P * out_jacobian.transpose()
        + Q
      ).eval();
  };

  solver(deltaT, in_out_state, ode_fn);
}

template <typename DTMeasurementModel>
static void dt_ekf_update(
  const DTMeasurementModel& mdl,
  const MeasurementVector<DTMeasurementModel>& z,
  const MeasurementCovariance<DTMeasurementModel>& R,
  SystemState<typename DTMeasurementModel::SystemModel>& in_out_state
) {
  auto x = system_state_get_state<typename DTMeasurementModel::SystemModel>(in_out_state);
  auto P = system_state_get_covariance<typename DTMeasurementModel::SystemModel>(in_out_state);
  
  StateToMeasurementTfJacobian<DTMeasurementModel> H = mdl.tf_state_to_measurement_jacobian(x);
  MeasurementCovariance<DTMeasurementModel> S = H * P * H.transpose() + R;
  KalmanGain<DTMeasurementModel> K = P * H.transpose() * S.inverse();

  x += K * (z - mdl.tf_state_to_measurement(x));
  P -= K * H * P;
}

} // namespace filters
} // namespace kalman

#endif // !KALMAN_FILTERS_EKF_HPP