#include <builtin_interfaces/msg/time.hpp>
#include <iomanip>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

rcl_time_point_value_t from_ros_time(const builtin_interfaces::msg::Time & ros_time)
{
  return static_cast<int64_t>(ros_time.sec) * 1000000000 + ros_time.nanosec;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("canlab_benchmark_latency");

  std::array<rclcpp::SubscriptionBase::SharedPtr, 4> subs;
  using sensor_msgs::msg::Image;
  for (int i = 0; i < 4; ++i) {
    subs[i] = node->create_subscription<Image>(
        "cam_" + std::to_string(i) + "/image_raw", rclcpp::SystemDefaultsQoS(),
        [i, &node](const Image::SharedPtr image) {
          const auto local_time = node->get_clock()->now().nanoseconds();
          const auto cam_time = from_ros_time(image->header.stamp);
          const auto diff_ns = local_time - cam_time;
          std::cout << "cam_" << i << " latency = " << std::fixed << std::setprecision(3)
                    << diff_ns / 1000.0 << "us" << std::endl;
        });
  }

  rclcpp::spin(node);

  return 0;
}
