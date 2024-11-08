#ifndef VXESTMODEL_HPP
#define VXESTMODEL_HPP

#include <kalman/common_types.hpp>
#include "vx_est_model_public.hpp"

namespace vx_est_model {

struct SystemModel {
  using FloatT = double;

  using States = enums::states::States;
  constexpr static int SystemDimension = enums::states::Dimension;
  constexpr static int InputDimension = 0;

  using SystemState = kalman::SystemState<SystemModel>;
  using InputVector = kalman::InputVector<SystemModel>;
  using StateVector = kalman::StateVector<SystemModel>;
  using SystemJacobian = kalman::SystemJacobian<SystemModel>;
  using SystemCovariance = kalman::SystemCovariance<SystemModel>;

  SystemJacobian xdot_jacobian(StateVector, InputVector) const {
    SystemJacobian ans(SystemJacobian::Zero());
    ans(States::Vx, States::Ax) = 1;
    return ans;
  }

  StateVector xdot(StateVector x, InputVector u) const {
    return xdot_jacobian(x, u) * x;
  }
};

struct ERPMMeasurementModel {
  using SystemModel = SystemModel;

  using Fields = enums::erpm_measurement::Fields;
  constexpr static int MeasurementDimension = enums::erpm_measurement::Dimension;

  using StateToMeasurementTfJacobian = kalman::StateToMeasurementTfJacobian<ERPMMeasurementModel>;
  using MeasurementVector = kalman::MeasurementVector<ERPMMeasurementModel>;
  using MeasurementCovariance = kalman::MeasurementCovariance<ERPMMeasurementModel>;

  StateToMeasurementTfJacobian tf_state_to_measurement_jacobian(SystemModel::StateVector) const {
    StateToMeasurementTfJacobian ans(StateToMeasurementTfJacobian::Zero());
    ans(SystemModel::States::Vx, Fields::ERPM) = 1;
    return ans;
  }

  MeasurementVector tf_state_to_measurement(SystemModel::StateVector x) const {
    return tf_state_to_measurement_jacobian(x) * x;
  }
};


struct IMUAXMeasurementModel {
  using SystemModel = SystemModel;

  using Fields = enums::imu_measurement::Fields;
  constexpr static int MeasurementDimension = enums::imu_measurement::Dimension;

  using StateToMeasurementTfJacobian = kalman::StateToMeasurementTfJacobian<IMUAXMeasurementModel>;
  using MeasurementVector = kalman::MeasurementVector<IMUAXMeasurementModel>;
  using MeasurementCovariance = kalman::MeasurementCovariance<ERPMMeasurementModel>;

  StateToMeasurementTfJacobian tf_state_to_measurement_jacobian(SystemModel::StateVector) const {
    StateToMeasurementTfJacobian ans(StateToMeasurementTfJacobian::Zero());
    ans(Fields::Ax, SystemModel::States::Ax) = 1;
    ans(Fields::Ax, SystemModel::States::ImuAxBias) = -1;
    return ans;
  }

  MeasurementVector tf_state_to_measurement(SystemModel::StateVector x) const {
    return tf_state_to_measurement_jacobian(x) * x;
  }
};

}; // namespace vx_est_model

#endif // !VXESTMODEL_HPP