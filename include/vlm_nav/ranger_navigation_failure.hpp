#pragma once

#include "nav2_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace vlm_nav
{
inline constexpr auto kNavigationFailure = "ranger_recovery.reason";

inline void setNavigationFailure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & reason)
{
  nav2_util::declare_parameter_if_not_declared(node,kNavigationFailure,rclcpp::ParameterValue(""));
  if (node->get_parameter(kNavigationFailure).as_string()!=reason) {
    const auto result=node->set_parameter(rclcpp::Parameter(kNavigationFailure,reason));
    if (!result.successful) {throw std::runtime_error("Cannot record navigation failure: "+result.reason);}
  }
}
}  // namespace vlm_nav
