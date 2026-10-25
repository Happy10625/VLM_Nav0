#include "nav2_behavior_tree/bt_service_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "vlm_nav/ranger_navigation_failure.hpp"

namespace vlm_nav
{
class RangerRecoveryNeeded : public nav2_behavior_tree::BtServiceNode<rcl_interfaces::srv::GetParameters>
{
public:
  RangerRecoveryNeeded(const std::string & name,const BT::NodeConfiguration & config)
  : BtServiceNode(name,config) {request_->names={kNavigationFailure};}

  BT::NodeStatus on_completion(rcl_interfaces::srv::GetParameters::Response::SharedPtr response) override
  {
    if (response->values.size()!=1 ||
      response->values[0].type!=rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    {
      return BT::NodeStatus::FAILURE;
    }
    const auto & reason=response->values[0].string_value;
    if (reason!="no_progress" && reason!="no_valid_trajectory") {return BT::NodeStatus::FAILURE;}
    RCLCPP_INFO(node_->get_logger(),"Ranger recovery: %s; attempting checked 180-degree spin",reason.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace vlm_nav

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vlm_nav::RangerRecoveryNeeded>("RangerRecoveryNeeded");
}
