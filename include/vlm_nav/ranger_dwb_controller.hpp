#pragma once

#include "dwb_core/dwb_local_planner.hpp"
#include "dwb_plugins/standard_traj_generator.hpp"
#include "nav2_controller/plugins/simple_progress_checker.hpp"

namespace vlm_nav
{
class RangerTrajectoryGenerator : public dwb_plugins::StandardTrajectoryGenerator
{
public:
  void initialize(const nav2_util::LifecycleNode::SharedPtr &, const std::string &) override;
  void setPath(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &,
    double resolution, double radius);
  void startNewIteration(const nav_2d_msgs::msg::Twist2D &) override;
  bool hasMoreTwists() override {return next_ < targets_.size();}
  nav_2d_msgs::msg::Twist2D nextTwist() override {return targets_.at(next_++);}
  dwb_msgs::msg::Trajectory2D generateTrajectory(const geometry_msgs::msg::Pose2D &,
    const nav_2d_msgs::msg::Twist2D &, const nav_2d_msgs::msg::Twist2D &) override;
  double headingGoal() const {return heading_goal_;}

private:
  nav_2d_msgs::msg::Twist2D commandStep(const nav_2d_msgs::msg::Twist2D &,
    const nav_2d_msgs::msg::Twist2D &, double);
  double period_{.05}, lookahead_{.4}, goal_tolerance_{.05};
  double moving_angular_limit_{.29}, min_turn_radius_{.47644}, heading_goal_{0.};
  double goal_distance_{0.}, linear_step_{.01}, angular_step_{.01};
  std::vector<nav_2d_msgs::msg::Twist2D> targets_;
  size_t next_{0};
};

class RangerDWBController : public dwb_core::DWBLocalPlanner
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) override;
  dwb_msgs::msg::TrajectoryScore scoreTrajectory(const dwb_msgs::msg::Trajectory2D &,
    double best_score = -1) override;

protected:
  nav_2d_msgs::msg::Path2D transformGlobalPlan(const nav_2d_msgs::msg::Pose2DStamped &) override;

private:
  std::shared_ptr<RangerTrajectoryGenerator> generator_;
  double heading_scale_{32.};
};

class RangerProgressChecker : public nav2_controller::SimpleProgressChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &) override;
  bool check(geometry_msgs::msg::PoseStamped &) override;
  void reset() override;
private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
};
}  // namespace vlm_nav
