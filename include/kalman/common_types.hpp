#ifndef KALMAN_COMMONTYPES_HPP
#define KALMAN_COMMONTYPES_HPP

#include <Eigen/Dense>

namespace kalman {

template <typename FloatT, int Rows, int Cols>
using Matrix = Eigen::Matrix<FloatT, Rows, Cols>;

template <typename FloatT, int Dimension>
using Vector = Matrix<FloatT, Dimension, 1>;

template <typename SystemModel>
using SystemState = Matrix<typename SystemModel::FloatT, SystemModel::SystemDimension, SystemModel::SystemDimension + 1>;

template <typename SystemModel>
using StateVector = Vector<typename SystemModel::FloatT, SystemModel::SystemDimension>;

template <typename SystemModel>
using InputVector = Vector<typename SystemModel::FloatT, SystemModel::InputDimension>;

template <typename SystemModel>
using SystemCovariance = Matrix<typename SystemModel::FloatT, SystemModel::SystemDimension, SystemModel::SystemDimension>;

template <typename SystemModel>
using SystemJacobian = Matrix<typename SystemModel::FloatT, SystemModel::SystemDimension, SystemModel::SystemDimension>;

template <typename SystemModel>
using LinearizedSystemOde = std::function<void(const SystemState<SystemModel>&, SystemState<SystemModel>&, SystemJacobian<SystemModel>&)>;

template <typename MeasurementModel>
using MeasurementVector = Vector<typename MeasurementModel::SystemModel::FloatT, MeasurementModel::MeasurementDimension>;

template <typename MeasurementModel>
using MeasurementCovariance = Matrix<typename MeasurementModel::SystemModel::FloatT, MeasurementModel::MeasurementDimension, MeasurementModel::MeasurementDimension>;

template <typename MeasurementModel>
using KalmanGain = Matrix<typename MeasurementModel::SystemModel::FloatT, MeasurementModel::SystemModel::SystemDimension, MeasurementModel::MeasurementDimension>;

template <typename MeasurementModel>
using StateToMeasurementTfJacobian = Matrix<typename MeasurementModel::SystemModel::FloatT, MeasurementModel::MeasurementDimension, MeasurementModel::SystemModel::SystemDimension>;

template <typename SystemModel>
static inline auto system_state_get_state(SystemState<SystemModel>& state) {
  return state.template block<SystemModel::SystemDimension, 1>(0, 0);
}

template <typename SystemModel>
static inline auto system_state_get_state(const SystemState<SystemModel>& state) {
  return state.template block<SystemModel::SystemDimension, 1>(0, 0);
}

template <typename SystemModel>
static inline auto system_state_get_covariance(SystemState<SystemModel>& state) {
  return state.template block<SystemModel::SystemDimension, SystemModel::SystemDimension>(0, 1);
}

template <typename SystemModel>
static inline auto system_state_get_covariance(const SystemState<SystemModel>& state) {
  return state.template block<SystemModel::SystemDimension, SystemModel::SystemDimension>(0, 1);
}

template <typename SystemModel>
static inline SystemState<SystemModel> make_system_state(const StateVector<SystemModel>& x0, const SystemCovariance<SystemModel>& P0) {
  SystemState<SystemModel> ans;
  system_state_get_state<SystemModel>(ans) = x0;
  system_state_get_covariance<SystemModel>(ans) = P0;
  return ans;
}

}; // namespace kalman

#endif // !KALMAN_COMMONTYPES_HPP