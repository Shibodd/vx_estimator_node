namespace vx_est_model {
namespace enums {

namespace states {
enum States : int {
  Vx = 0,
  Ax,
  ImuAxBias
};
constexpr int Dimension = ImuAxBias + 1;
}; // namespace states

namespace erpm_measurement {
enum Fields : int {
  ERPM = 0
};
constexpr int Dimension = ERPM + 1;
}; // namespace erpm_measurement

namespace imu_measurement {
enum Fields : int {
  Ax = 0
};
constexpr int Dimension = Ax + 1;
}; // namespace imu_measurement

}; // namespace enums
}; // namespace vx_est_model