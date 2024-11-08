#ifndef MEASUREMENT_TYPES_HPP
#define MEASUREMENT_TYPES_HPP

#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <Eigen/Dense>
#include <estimator_node/vx_est_model_public.hpp>

class Measurement {
  std::chrono::nanoseconds m_t;
  Eigen::VectorXf v;
};

#endif