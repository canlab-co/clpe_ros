/*
 * Copyright (C) 2022 Can-lab Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <ClpeClientApi.h>

#include "ClpeNode.hpp"
#include "clpe_ros_component_export.h"
#include "rclcpp_components/register_node_macro.hpp"

//==============================================================================
namespace clpe
{
class ClpeComponentNode
{
public:
  CLPE_ROS_COMPONENT_EXPORT
  ClpeComponentNode(const rclcpp::NodeOptions & options)
  {
    this->node_ = ClpeNode<ClpeClientApi>::make_shared(ClpeClientApi(), options);
    this->node_->Init();
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
