#include <ClpeClientApi.h>

#include <rclcpp/rclcpp.hpp>

#include "ClpeNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto node = clpe::ClpeNode<ClpeClientApi>::make_shared(ClpeClientApi());
  node->Init();
  rclcpp::spin(node);

  return 0;
}
