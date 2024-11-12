#ifndef THENODE_HPP
#define THENODE_HPP

#include <chrono>
#include <numbers>
#include <functional>
#include <queue>
#include <optional>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <vesc_msgs/msg/vesc_imu_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <kalman/filters/ekf.hpp>
#include <kalman/solvers/euler.hpp>

#include <estimator_node/model_implementation.hpp>


using namespace std::chrono_literals;

namespace estimator_node {

struct MeasurementT {
  std::chrono::nanoseconds timestamp;
  std::function<void(model::impl::SystemModel::SystemState&)> update;

  struct Newer {
    bool operator()(const MeasurementT& lhs, const MeasurementT& rhs) const {
      return lhs.timestamp > rhs.timestamp;
    }
  };
};

class EstimatorNode {
  constexpr static int MAX_QUEUED_MEASUREMENTS = 10;
  std::priority_queue<MeasurementT, std::vector<MeasurementT>, MeasurementT::Newer> m_measurements;

  void push_measurement(const MeasurementT& z) {
    if (m_measurements.size() > MAX_QUEUED_MEASUREMENTS)
      m_measurements.pop();
    m_measurements.push(z);
  }

  kalman::solvers::EulerSolver<model::impl::SystemModel> m_solver;
  model::impl::SystemModel m_sys_mdl;
  model::impl::ERPMMeasurementModel m_erpm_mdl;
  model::impl::IMUMeasurementModel m_imu_mdl;

  model::impl::SystemModel::SystemCovariance m_process_noise;
  model::impl::ERPMMeasurementModel::MeasurementCovariance m_erpm_noise;
  model::impl::IMUMeasurementModel::MeasurementCovariance m_imu_noise;

  std::optional<std::chrono::nanoseconds> m_state_t;
  model::impl::SystemModel::SystemState m_state;

  model::impl::SystemModel::SystemState m_initial_state;

public:
  EstimatorNode(double qvx, double qvy, double qax, double qay, double qphidot, double qimuax, double qimuay)
    : m_solver(0.05) {
    model::impl::SystemModel::StateVector Q;
    Q(model::states::Vx) = qvx;
    Q(model::states::Vy) = qvy;
    Q(model::states::Ax) = qax;
    Q(model::states::Ay) = qay;
    Q(model::states::Phidot) = qphidot;
    Q(model::states::ImuAxBias) = qimuax;
    Q(model::states::ImuAyBias) = qimuay;
    m_process_noise = Q.asDiagonal();

    model::impl::IMUMeasurementModel::MeasurementVector R_imu;
    R_imu(model::imu_measurement::Ax) = 0.5998;
    R_imu(model::imu_measurement::Ay) = 0.7807;
    R_imu(model::imu_measurement::Phidot) = 0.1411;
    m_imu_noise = R_imu.asDiagonal();

    model::impl::ERPMMeasurementModel::MeasurementVector R_erpm;
    R_erpm(model::erpm_measurement::ERPM) = 0.3652;
    m_erpm_noise = R_erpm.asDiagonal();

    model::impl::SystemModel::StateVector P0(model::impl::SystemModel::StateVector::Constant(1e-9));
    P0(model::states::ImuAxBias) = P0(model::states::ImuAyBias) = 1;

    m_initial_state = kalman::make_system_state<model::impl::SystemModel>(
      model::impl::SystemModel::StateVector::Zero(),
      P0.asDiagonal()
    );
    m_state = m_initial_state;
  }

  void reset() {
    m_state_t.reset();
    m_state = m_initial_state;
  }

  std::optional<std::pair<std::chrono::nanoseconds, model::impl::SystemModel::SystemState>> get_state() {
    // Process all queued measurements
    while (m_measurements.size() > 0) {
      MeasurementT z = m_measurements.top();
      m_measurements.pop();

      if (!m_state_t.has_value())
        m_state_t = z.timestamp;

      // Discard measurements older than the last estimate (too late - we don't support rolling back)
      if (z.timestamp < m_state_t) {
        // std::cout << "Measurement was late! Discarding" << std::endl;
        continue;
      }

      // Predict until measurement timestamp
      double deltaT = std::chrono::duration<double>(z.timestamp - *m_state_t).count();

      if (deltaT > 5.0) {
        std::cout << "Tracking lost! Resetting" << std::endl;
        reset();
        m_state_t = z.timestamp;
        deltaT = 0;
      }

      // std::cout << "PREDICT " << deltaT << std::endl;
      kalman::filters::ct_ekf_predict(deltaT, m_sys_mdl, {}, m_process_noise, m_solver, m_state);
      m_state_t = z.timestamp;

      // Perform update
      z.update(m_state);
    }

    if (m_state_t.has_value()) {
      return { { *m_state_t, m_state } };
    } else {
      return std::nullopt;
    }
  }

  void vesc_core_cb(const vesc_msgs::msg::VescStateStamped::ConstSharedPtr& msg) {
    Eigen::Vector<double, 1> z { msg->state.speed * 0.00012 };
    
    std::function fn = [z, &meas_mdl = m_erpm_mdl, &R = m_erpm_noise](model::impl::SystemModel::SystemState& state) {
      // std::cout << "CORE Update" << std::endl;
      kalman::filters::dt_ekf_update(meas_mdl, z, R, state);
    };
    push_measurement({ std::chrono::nanoseconds(rclcpp::Time(msg->header.stamp).nanoseconds()), fn });
  }

  void vesc_imu_cb(const vesc_msgs::msg::VescImuStamped::ConstSharedPtr& msg) {
    constexpr double THETA = 1.2819;
    const Eigen::Matrix2d ROT {
      { std::cos(THETA), -std::sin(THETA) },
      { std::sin(THETA), std::cos(THETA) }
    };

    Eigen::Vector2d z_acc(msg->imu.linear_acceleration.x, msg->imu.linear_acceleration.y);
    z_acc = ROT * 9.81 * z_acc;

    Eigen::Vector<double, model::imu_measurement::Dimension> z;
    z(model::imu_measurement::Ax) = z_acc.x();
    z(model::imu_measurement::Ay) = z_acc.y();
    z(model::imu_measurement::Phidot) = msg->imu.angular_velocity.z * (std::numbers::pi / 180.0);

    std::function fn = [z, &meas_mdl = m_imu_mdl, &R = m_imu_noise](model::impl::SystemModel::SystemState& state) {
      // std::cout << "IMU Update" << std::endl;
      kalman::filters::dt_ekf_update(meas_mdl, z, R, state);
    };
    push_measurement({ std::chrono::nanoseconds(rclcpp::Time(msg->header.stamp).nanoseconds()), fn });
  }
};

} // namespace estimator_node

#endif // !THENODE_HPP