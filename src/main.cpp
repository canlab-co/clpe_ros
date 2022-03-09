#include <ClpeClientApi.h>

#include <rclcpp/rclcpp.hpp>

class ClpeNode : public rclcpp::Node
{
public:
  ClpeNode() : rclcpp::Node("clpe") {}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  ClpeNode node;
  const auto logger = node.get_logger();
  ClpeClientApi clpe_api;

  {
    // FIXME: This requires sudo password!!
    const auto result = clpe_api.Clpe_Connection("");
    if (!result) {
      RCLCPP_FATAL(logger, "Failed to initial the clpe network connection. Error number = ( ",
                   result, " )");
    } else {
      RCLCPP_INFO(logger, "Successfully initialized");
    }
  }

  return 0;
}
