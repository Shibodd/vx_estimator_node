#ifndef ESTIMATORNODE_MODEL_HPP
#define ESTIMATORNODE_MODEL_HPP

namespace estimator_node {
namespace model {

namespace states {
enum States : int {
  Vx = 0,
  Vy,
  Phidot,
  Ax,
  Ay,
  ImuAxBias,
  ImuAyBias
};
constexpr int Dimension = ImuAyBias + 1;
}; // namespace states

namespace erpm_measurement {
enum Fields : int {
  ERPM = 0
};
constexpr int Dimension = ERPM + 1;
}; // namespace erpm_measurement

namespace imu_measurement {
enum Fields : int {
  Ax = 0,
  Ay,
  Phidot
};
constexpr int Dimension = Phidot + 1;
}; // namespace imu_measurement

}; // namespace model
}; // namespace estimator_node

#endif // !ESTIMATORNODE_MODEL_HPP