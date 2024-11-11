#include <rclcpp/rclcpp.hpp>

#include <estimator_node/the_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<estimator_node::EstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
