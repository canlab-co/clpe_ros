#include <ClpeClientApi.h>

#include "ClpeNode.hpp"
#include "canlab_ros_component_export.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace clpe
{
class ClpeComponentNode : public ClpeNode<ClpeClientApi>
{
public:
  CANLAB_ROS_COMPONENT_EXPORT
  ClpeComponentNode(const rclcpp::NodeOptions & node) : ClpeNode<ClpeClientApi>(ClpeClientApi()) {}
};
}  // namespace clpe

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(clpe::ClpeComponentNode)
