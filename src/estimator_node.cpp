#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <estimator_node/the_node.hpp>
#include <vesc_msgs/msg/detail/vesc_state_stamped__struct.hpp>

#include "rclcpp/serialization.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"

extern "C" int process(uint64_t* out_t, double* out_x, double qvx, double qax, double rvx, double rax)
{
  estimator_node::EstimatorNode node(qvx, qax, rvx, rax);

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = "/home/simonebondi/Downloads/rad/0_manuale_rad";
  auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  reader->open(storage_options);

  std::map<std::string, std::function<void(const rclcpp::SerializedMessage&)>> m = {
    { "/sensors/core", [&node](const rclcpp::SerializedMessage& serialized_msg) {
      static rclcpp::Serialization<vesc_msgs::msg::VescStateStamped> serialization;

      auto msg = std::make_shared<vesc_msgs::msg::VescStateStamped>();
      serialization.deserialize_message(&serialized_msg, msg.get());
      node.vesc_core_cb(msg);
    } },
    { "/sensors/imu", [&node](const rclcpp::SerializedMessage& serialized_msg) {
      static rclcpp::Serialization<vesc_msgs::msg::VescImuStamped> serialization;

      auto msg = std::make_shared<vesc_msgs::msg::VescImuStamped>();
      serialization.deserialize_message(&serialized_msg, msg.get());
      node.vesc_imu_cb(msg);
    } }
  };

  std::optional<std::chrono::nanoseconds> next_t;

  int i = 0;
  while (reader->has_next()) {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();

    auto it = m.find(msg->topic_name);
    if (it == m.end())
      continue;

    if (!next_t.has_value() || msg->time_stamp > next_t->count()) {
      if (!next_t.has_value()) {
        next_t = std::chrono::nanoseconds(msg->time_stamp) + 50ms;
      } else {
        *next_t += 50ms;
      }

      // Write timestamp
      auto state = node.get_state();
      if (state.has_value()) {
        auto t = state->first;
        out_t[i] = t.count();

        // Extract state
        Eigen::Matrix x = state->second.col(0).eval();
        // std::cout << x << std::endl;
        
        // Write state
        double* x_base = (out_x + i * x.rows());
        memcpy(x_base, x.data(), sizeof(decltype(x)::Scalar) * x.rows());

        ++i;

        if (x.hasNaN()) {
          std::cout << "Eggsplosion" << std::endl;
          return i;
        }
      }
    }

    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    it->second(serialized_msg);
  }
  return i;
}