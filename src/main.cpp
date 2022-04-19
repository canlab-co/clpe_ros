#include <ClpeClientApi.h>

#include <ros/ros.h>

#include "ClpeNode.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "clpe_node");

  const auto node = clpe::ClpeNode<ClpeClientApi>::make_shared(ClpeClientApi());
  node->Init();
  ros::spin();

  return 0;
}
