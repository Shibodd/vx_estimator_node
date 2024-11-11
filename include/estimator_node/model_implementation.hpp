#ifndef ESTIMATORNODE_MODELIMPLEMENTATION_HPP
#define ESTIMATORNODE_MODELIMPLEMENTATION_HPP

#include <kalman/common_types.hpp>
#include <estimator_node/model.hpp>

namespace estimator_node {
namespace model {
namespace impl {

struct SystemModel {
  using FloatT = double;

  using States = states::States;
  constexpr static int SystemDimension = states::Dimension;
  constexpr static int InputDimension = 0;

  using SystemState = kalman::SystemState<SystemModel>;
  using InputVector = kalman::InputVector<SystemModel>;
  using StateVector = kalman::StateVector<SystemModel>;
  using SystemJacobian = kalman::SystemJacobian<SystemModel>;
  using SystemCovariance = kalman::SystemCovariance<SystemModel>;

  SystemJacobian xdot_jacobian(StateVector x, InputVector) const {
    SystemJacobian ans(SystemJacobian::Zero());
    ans(States::Vx, States::Ax) = 1;
    ans(States::Vx, States::Vy) = -x(States::Phidot);
    ans(States::Vx, States::Phidot) = -x(States::Vy);

    ans(States::Vy, States::Ay) = 1;
    ans(States::Vy, States::Vx) = x(States::Phidot);
    ans(States::Vy, States::Phidot) = x(States::Vx);
    return ans;
  }

  StateVector xdot(StateVector x, InputVector u) const {
    StateVector ans;
    ans(States::Vx) = x(States::Ax) - x(States::Phidot) * x(States::Vy);
    ans(States::Vy) = x(States::Ay) + x(States::Phidot) * x(States::Vx);
    return ans;
  }
};

struct ERPMMeasurementModel {
  using SystemModel = SystemModel;

  using Fields = erpm_measurement::Fields;
  constexpr static int MeasurementDimension = erpm_measurement::Dimension;

  using StateToMeasurementTfJacobian = kalman::StateToMeasurementTfJacobian<ERPMMeasurementModel>;
  using MeasurementVector = kalman::MeasurementVector<ERPMMeasurementModel>;
  using MeasurementCovariance = kalman::MeasurementCovariance<ERPMMeasurementModel>;

  StateToMeasurementTfJacobian tf_state_to_measurement_jacobian(SystemModel::StateVector) const {
    StateToMeasurementTfJacobian ans(StateToMeasurementTfJacobian::Zero());
    ans(Fields::ERPM, SystemModel::States::Vx) = 1;
    return ans;
  }

  MeasurementVector tf_state_to_measurement(SystemModel::StateVector x) const {
    return tf_state_to_measurement_jacobian(x) * x;
  }
};


struct IMUMeasurementModel {
  using SystemModel = SystemModel;

  using Fields = imu_measurement::Fields;
  constexpr static int MeasurementDimension = imu_measurement::Dimension;

  using StateToMeasurementTfJacobian = kalman::StateToMeasurementTfJacobian<IMUMeasurementModel>;
  using MeasurementVector = kalman::MeasurementVector<IMUMeasurementModel>;
  using MeasurementCovariance = kalman::MeasurementCovariance<IMUMeasurementModel>;

  StateToMeasurementTfJacobian tf_state_to_measurement_jacobian(SystemModel::StateVector) const {
    StateToMeasurementTfJacobian ans(StateToMeasurementTfJacobian::Zero());

    ans(Fields::Ax, SystemModel::States::Ax) = 1;
    ans(Fields::Ax, SystemModel::States::ImuAxBias) = -1;

    ans(Fields::Ay, SystemModel::States::Ay) = 1;
    ans(Fields::Ay, SystemModel::States::ImuAyBias) = -1;

    ans(Fields::Phidot, SystemModel::States::Phidot) = 1;

    return ans;
  }

  MeasurementVector tf_state_to_measurement(SystemModel::StateVector x) const {
    return tf_state_to_measurement_jacobian(x) * x;
  }
};

}; // namespace impl
}; // namespace model
}; // namespace estimator_node

#endif // !ESTIMATORNODE_MODELIMPLEMENTATION_HPP