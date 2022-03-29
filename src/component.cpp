#include <ClpeClientApi.h>

#include "ClpeNode.hpp"
#include "clpe_ros_component_export.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace clpe
{
class ClpeComponentNode
{
public:
  CLPE_ROS_COMPONENT_EXPORT
  ClpeComponentNode(const rclcpp::NodeOptions & node)
  {
    this->node_ = ClpeNode<ClpeClientApi>::make_shared(ClpeClientApi());
  }

  CLPE_ROS_COMPONENT_EXPORT
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
  {
    return this->node_->get_node_base_interface();
  }

private:
  std::shared_ptr<ClpeNode<ClpeClientApi>> node_;
};
}  // namespace clpe

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(clpe::ClpeComponentNode)
