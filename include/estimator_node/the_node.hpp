#ifndef THENODE_HPP
#define THENODE_HPP

#include <algorithm>
#include <chrono>
#include <functional>
#include <queue>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <vesc_msgs/msg/detail/vesc_state__struct.hpp>
#include <vesc_msgs/msg/vesc_imu_stamped.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>


#include <kalman/filters/ekf.hpp>
#include <kalman/solvers/euler.hpp>
#include "vx_est_model_impl.hpp"

using namespace std::chrono_literals;

struct MeasurementT {
  std::chrono::nanoseconds timestamp;
  std::function<void(vx_est_model::SystemModel::SystemState&)> update;

  struct Newer {
    bool operator()(const MeasurementT& lhs, const MeasurementT& rhs) const {
      return lhs.timestamp > rhs.timestamp;
    }
  };
};

class EstimatorNode : public rclcpp::Node {
  constexpr static int MAX_QUEUED_MEASUREMENTS = 10;
  std::priority_queue<MeasurementT, std::vector<MeasurementT>, MeasurementT::Newer> m_measurements;

  std::chrono::nanoseconds now(){
    return std::chrono::nanoseconds(rclcpp::Node::now().nanoseconds());
  }

  void push_measurement(const MeasurementT& z) {
    if (m_measurements.size() > MAX_QUEUED_MEASUREMENTS)
      m_measurements.pop();
    m_measurements.push(z);
    
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_bool_sub;
  rclcpp::Subscription<vesc_msgs::msg::VescImuStamped>::SharedPtr m_vesc_imu_sub;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr m_vesc_state_sub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_speed_pub;
  rclcpp::TimerBase::SharedPtr m_timer;

  kalman::solvers::EulerSolver<vx_est_model::SystemModel> m_solver;

  vx_est_model::SystemModel::SystemCovariance m_process_noise;
  vx_est_model::ERPMMeasurementModel::MeasurementCovariance m_erpm_noise;
  vx_est_model::IMUAXMeasurementModel::MeasurementCovariance m_imuax_noise;
  vx_est_model::SystemModel m_vx_mdl;
  vx_est_model::ERPMMeasurementModel m_erpm_mdl;
  vx_est_model::IMUAXMeasurementModel m_imuax_mdl;

  std::optional<std::chrono::nanoseconds> m_state_t;
  vx_est_model::SystemModel::SystemState m_state;

public:
  EstimatorNode() : rclcpp::Node("estimator") {
    m_vesc_imu_sub = this->create_subscription<vesc_msgs::msg::VescImuStamped>("/sensors/imu", 5, std::bind(&EstimatorNode::vesc_imu_cb, this, std::placeholders::_1));

    m_vesc_state_sub = this->create_subscription<vesc_msgs::msg::VescStateStamped>("/sensors/core", 5, std::bind(&EstimatorNode::vesc_core_cb, this, std::placeholders::_1));
    m_bool_sub = this->create_subscription<std_msgs::msg::Bool>("/print", 1, std::bind(&EstimatorNode::print_cb, this, std::placeholders::_1));
    m_timer = this->create_wall_timer(0.05s, std::bind(&EstimatorNode::timer_cb, this));
    m_speed_pub = this->create_publisher<std_msgs::msg::Float64>("/ekf/speed", 5);
    
    m_process_noise = vx_est_model::SystemModel::StateVector(1e-3, 1e-1, 1e-8).asDiagonal();
    m_erpm_noise = vx_est_model::ERPMMeasurementModel::MeasurementVector(1).asDiagonal();
    m_imuax_noise = vx_est_model::IMUAXMeasurementModel::MeasurementVector(1e-2).asDiagonal();
    m_state = kalman::make_system_state<vx_est_model::SystemModel>(
      vx_est_model::SystemModel::StateVector::Zero(),
      vx_est_model::SystemModel::StateVector(1e-5, 1e-5, 5).asDiagonal()
    );
  }

  void timer_cb() {
    /*
    if (!m_state_t.has_value()) {
      if (m_measurements.size() <= 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), 5000, "Waiting for measurements");
      } else {
        m_state_t = now();
      }
      return;
    }

    //std::chrono::nanoseconds t = now();
    */
    // Process all queued measurements
    while (m_measurements.size() > 0) {
      MeasurementT z = m_measurements.top();
      m_measurements.pop();

      if (!m_state_t.has_value())
        m_state_t = z.timestamp;

      RCLCPP_INFO(this->get_logger(), "Zt = %ld\nSt = %ld\nT = %ld", z.timestamp.count(), m_state_t->count(), 0);

      // Discard measurements older than the last estimate (too late - we don't support rolling back)
      if (z.timestamp < m_state_t) {
        RCLCPP_WARN(this->get_logger(), "Measurement was late! Discarding");
        continue;
      }

      /*
      // Discard measurements newer than the current time (you're doing something wrong)
      if (z.timestamp > t) {
        RCLCPP_WARN(this->get_logger(), "Measurement is from the future! Discarding");
        continue;
      }
      */

      // Predict until measurement timestamp
      double deltaT = std::chrono::duration<double>(z.timestamp - *m_state_t).count();
      std::cout << "PREDICT " << deltaT << std::endl;
      kalman::filters::ct_ekf_predict(deltaT, m_vx_mdl, {}, m_process_noise, m_solver, m_state);
      m_state_t = z.timestamp;

      // Perform update
      z.update(m_state);
    }

  /*
    // Predict until current time
    double deltaT = std::chrono::duration<double>(t - *m_state_t).count();
    std::cout << "PREDICT " << deltaT << std::endl;
    kalman::filters::ct_ekf_predict(deltaT, m_vx_mdl, {}, m_process_noise, m_solver, m_state);
    m_state_t = t;
  */

    std_msgs::msg::Float64 msg;
    msg.data = m_state(vx_est_model::enums::states::Vx, 0);
    m_speed_pub->publish(msg);

    if (m_state_t.has_value())
      std::cerr << m_state << std::endl;
  }

  void print_cb(const std_msgs::msg::Bool::ConstSharedPtr&) {
    
  }

  void vesc_core_cb(const vesc_msgs::msg::VescStateStamped::ConstSharedPtr& msg) {
    Eigen::Vector<double, 1> z { msg->state.speed * 0.00012 };
    
    std::function fn = [z, &meas_mdl = m_erpm_mdl, &R = m_erpm_noise](vx_est_model::SystemModel::SystemState& state) {
      std::cout << "CORE Update" << std::endl;
      kalman::filters::dt_ekf_update(meas_mdl, z, R, state);
    };
    push_measurement({ std::chrono::nanoseconds(rclcpp::Time(msg->header.stamp).nanoseconds()), fn });
  }

  void vesc_imu_cb(const vesc_msgs::msg::VescImuStamped::ConstSharedPtr& msg) {
    Eigen::Vector<double, 1> z { -9.81 * msg->imu.linear_acceleration.y };

    std::function fn = [z, &meas_mdl = m_imuax_mdl, &R = m_imuax_noise](vx_est_model::SystemModel::SystemState& state) {
      std::cout << "IMU Update" << std::endl;
      kalman::filters::dt_ekf_update(meas_mdl, z, R, state);
    };
    push_measurement({ std::chrono::nanoseconds(rclcpp::Time(msg->header.stamp).nanoseconds()), fn });
  }
};

#endif // !THENODE_HPP