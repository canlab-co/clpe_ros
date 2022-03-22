#include <ClpeClientApi.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "ClpeNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto node = clpe::ClpeNode<ClpeClientApi>::make_shared(ClpeClientApi());
  node->Init();
  rclcpp::spin(node);

  return 0;
}
