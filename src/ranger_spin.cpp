#include "nav2_behaviors/plugins/spin.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace vlm_nav
{
class RangerSpin : public nav2_behaviors::Spin
{
public:
  nav2_behaviors::Status onRun(const std::shared_ptr<const nav2_msgs::action::Spin::Goal> goal) override
  {
    if (Spin::onRun(goal)!=nav2_behaviors::Status::SUCCEEDED || !remainingTurnIsClear()) {
      stopRobot();return nav2_behaviors::Status::FAILED;
    }
    return nav2_behaviors::Status::SUCCEEDED;
  }

  nav2_behaviors::Status onCycleUpdate() override
  {
    if (!remainingTurnIsClear()) {stopRobot();return nav2_behaviors::Status::FAILED;}
    return Spin::onCycleUpdate();
  }

private:
  bool remainingTurnIsClear()
  {
    geometry_msgs::msg::PoseStamped pose;
    if (!std::isfinite(cmd_yaw_) || !nav2_util::getCurrentPose(
        pose,*tf_,global_frame_,robot_base_frame_,transform_tolerance_)) {return false;}
    const double remaining=std::max(0.,std::abs(cmd_yaw_)-std::abs(relative_yaw_));
    const int steps=std::max(1,static_cast<int>(std::ceil(remaining/.01)));
    geometry_msgs::msg::Pose2D sample;
    sample.x=pose.pose.position.x;sample.y=pose.pose.position.y;
    const double yaw=tf2::getYaw(pose.pose.orientation);
    // Humble's native Spin can skip the initial sweep; check the full remaining turn each cycle.
    for (int i=0;i<=steps;i++) {
      sample.theta=yaw+std::copysign(remaining*i/steps,cmd_yaw_);
      if (!collision_checker_->isCollisionFree(sample,i==0)) {
        RCLCPP_WARN(logger_,"Ranger spin rejected: footprint collision in remaining turn");
        return false;
      }
    }
    return true;
  }
};
}  // namespace vlm_nav

PLUGINLIB_EXPORT_CLASS(vlm_nav::RangerSpin,nav2_core::Behavior)
