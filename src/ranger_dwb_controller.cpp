#include "vlm_nav/ranger_dwb_controller.hpp"
#include "vlm_nav/ranger_navigation_failure.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "dwb_plugins/one_d_velocity_iterator.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace vlm_nav
{
using Twist = nav_2d_msgs::msg::Twist2D;

static double angle(double value) {return std::atan2(std::sin(value),std::cos(value));}

void RangerTrajectoryGenerator::initialize(
  const nav2_util::LifecycleNode::SharedPtr & node, const std::string & name)
{
  StandardTrajectoryGenerator::initialize(node,name);
  auto parameter = [&](const std::string & key, double value) {
      nav2_util::declare_parameter_if_not_declared(node,name+"."+key,rclcpp::ParameterValue(value));
      return node->get_parameter(name+"."+key).as_double();
    };
  moving_angular_limit_=parameter("max_moving_angular_velocity",.29);
  min_turn_radius_=parameter("chassis_min_turn_radius",.47644);
  lookahead_=parameter("heading_lookahead",.4);
  goal_tolerance_=parameter("xy_goal_tolerance",.05);
  double frequency=20.;
  if (node->has_parameter("controller_frequency")) {
    frequency=node->get_parameter("controller_frequency").as_double();
  }
  for (double v : {moving_angular_limit_,min_turn_radius_,lookahead_,goal_tolerance_,
      frequency,sim_time_,linear_granularity_,angular_granularity_})
  {
    if (!std::isfinite(v) || v<=0.) {throw std::invalid_argument("Ranger motion limits must be finite and positive");}
  }
  if (moving_angular_limit_>=.3) {throw std::invalid_argument("Ranger moving angular limit must be below 0.3 rad/s");}
  period_=1./frequency;
}

void RangerTrajectoryGenerator::setPath(const geometry_msgs::msg::Pose2D & pose,
  const nav_2d_msgs::msg::Path2D & path, double resolution, double radius)
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
    throw nav2_core::PlannerException("Ranger robot pose is nonfinite");
  }
  if (path.poses.empty()) {throw nav2_core::PlannerException("Ranger path is empty");}
  for (const auto & p : path.poses) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.theta)) {
      throw nav2_core::PlannerException("Ranger path contains nonfinite poses");
    }
  }
  if (!std::isfinite(resolution) || resolution<=0. || !std::isfinite(radius) || radius<=0.) {
    throw nav2_core::PlannerException("Ranger costmap / footprint geometry is invalid");
  }
  auto distance=[&](const auto & p) {return std::hypot(p.x-pose.x,p.y-pose.y);};
  size_t nearest=0;
  for (size_t i=1;i<path.poses.size();i++) {
    if (distance(path.poses[i])<distance(path.poses[nearest])) {nearest=i;}
  }
  auto target=path.poses.back();
  double walked=0.;
  for (size_t i=nearest+1;i<path.poses.size();i++) {
    const auto & a=path.poses[i-1];const auto & b=path.poses[i];
    double length=std::hypot(b.x-a.x,b.y-a.y);
    if (length>0. && walked+length>=lookahead_) {
      double fraction=(lookahead_-walked)/length;
      target.x=a.x+fraction*(b.x-a.x);target.y=a.y+fraction*(b.y-a.y);break;
    }
    walked+=length;
  }
  heading_goal_=std::atan2(target.y-pose.y,target.x-pose.x);
  goal_distance_=distance(path.poses.back());
  linear_step_=std::min({linear_granularity_,.01,resolution/2.});
  angular_step_=std::min({angular_granularity_,.01,resolution/(2.*radius)});
}

void RangerTrajectoryGenerator::startNewIteration(const Twist & velocity)
{
  auto k=kinematics_handler_->getKinematics();
  for (double v : {velocity.x,velocity.y,velocity.theta,k.getAccX(),k.getDecelX(),
      k.getAccTheta(),k.getDecelTheta(),k.getMaxX(),k.getMaxTheta()})
  {
    if (!std::isfinite(v)) {throw nav2_core::PlannerException("Nonfinite Ranger motion input");}
  }
  if (k.getAccX()<=0. || k.getDecelX()>=0. || k.getAccTheta()<=0. || k.getDecelTheta()>=0. ||
    k.getMaxX()<0. || k.getMaxTheta()<0. || std::abs(velocity.y)>.01)
  {
    throw nav2_core::PlannerException("Ranger requires valid acceleration limits and no lateral starting motion");
  }
  targets_.assign(1,Twist());next_=0;
  if (goal_distance_<=goal_tolerance_) {return;}
  const double maximum=std::min(k.getMaxX(),std::sqrt(-2.*k.getDecelX()*
      std::max(0.,goal_distance_-goal_tolerance_/2.)));
  for (int i=0;i<=20;i++) {
    const double angular_limit=i==0 ? k.getMaxTheta() : std::min(k.getMaxTheta(),moving_angular_limit_);
    for (int j=-10;j<=10;j++) {
      if (i==0 && j==0) {continue;}
      Twist target;target.x=maximum*i/20.;target.theta=angular_limit*j/10.;
      targets_.push_back(target);
    }
  }
}

Twist RangerTrajectoryGenerator::commandStep(const Twist & start, const Twist & target, double dt)
{
  auto k=kinematics_handler_->getKinematics();
  Twist next;
  next.x=dwb_plugins::projectVelocity(start.x,k.getAccX(),k.getDecelX(),dt,target.x);
  const double angular_target=next.x==0. ? target.theta :
    std::clamp(target.theta,-moving_angular_limit_,moving_angular_limit_);
  next.theta=dwb_plugins::projectVelocity(start.theta,k.getAccTheta(),k.getDecelTheta(),dt,angular_target);
  // A fast spin may translate as soon as angular braking reaches the moving limit.
  if (next.x!=0. && std::abs(next.theta)>=.3) {next.x=0.;}
  return next;
}

dwb_msgs::msg::Trajectory2D RangerTrajectoryGenerator::generateTrajectory(
  const geometry_msgs::msg::Pose2D & start_pose, const Twist & start_vel, const Twist & target)
{
  dwb_msgs::msg::Trajectory2D trajectory;
  auto pose=start_pose;auto velocity=start_vel;
  trajectory.velocity=commandStep(start_vel,target,period_);
  trajectory.poses.push_back(pose);
  trajectory.time_offsets.push_back(rclcpp::Duration::from_seconds(0.));
  double time=0.;
  while (time<sim_time_-1e-9) {
    const double block=std::min(period_,sim_time_-time);
    const Twist command=commandStep(velocity,target,period_);
    auto chassis_command=command;
    // Ranger Mini V3 discards translation below its minimum Ackermann turn radius.
    if (std::abs(command.theta)>=1e-6 && std::abs(command.x/command.theta)<min_turn_radius_) {
      chassis_command.x=0.;
    }
    const double max_v=std::max(std::hypot(velocity.x,velocity.y),std::abs(command.x));
    const double max_w=std::max(std::abs(velocity.theta),std::abs(command.theta));
    const int steps=std::max(1,static_cast<int>(std::ceil(std::max(max_v*block/linear_step_,max_w*block/angular_step_))));
    const double dt=block/steps;
    for (int i=0;i<steps;i++) {
      // Use the same native acceleration model from the measured start to each final command.
      velocity=computeNewVelocity(chassis_command,velocity,dt);
      pose=computeNewPosition(pose,velocity,dt);time+=dt;
      trajectory.poses.push_back(pose);
      trajectory.time_offsets.push_back(rclcpp::Duration::from_seconds(time));
    }
  }
  return trajectory;
}

void RangerDWBController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map)
{
  DWBLocalPlanner::configure(parent,name,tf,map);
  generator_=std::dynamic_pointer_cast<RangerTrajectoryGenerator>(traj_generator_);
  if (!generator_) {throw std::invalid_argument("Ranger controller requires RangerTrajectoryGenerator");}
  auto node=parent.lock();
  setNavigationFailure(node,"");
  nav2_util::declare_parameter_if_not_declared(node,name+".heading_scale",rclcpp::ParameterValue(32.));
  heading_scale_=node->get_parameter(name+".heading_scale").as_double();
  if (!std::isfinite(heading_scale_) || heading_scale_<=0.) {throw std::invalid_argument("Ranger heading_scale must be positive");}
}

geometry_msgs::msg::TwistStamped RangerDWBController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * checker)
{
  setNavigationFailure(node_.lock(),"");
  try {return DWBLocalPlanner::computeVelocityCommands(pose,velocity,checker);}
  catch (const dwb_core::NoLegalTrajectoriesException &) {
    setNavigationFailure(node_.lock(),"no_valid_trajectory");
    throw;
  }
}

nav_2d_msgs::msg::Path2D RangerDWBController::transformGlobalPlan(
  const nav_2d_msgs::msg::Pose2DStamped & pose)
{
  auto path=DWBLocalPlanner::transformGlobalPlan(pose);
  double radius=0.;
  for (const auto & point : costmap_ros_->getRobotFootprint()) {
    radius=std::max(radius,std::hypot(point.x,point.y));
  }
  generator_->setPath(pose.pose,path,costmap_ros_->getCostmap()->getResolution(),radius);
  return path;
}
dwb_msgs::msg::TrajectoryScore RangerDWBController::scoreTrajectory(
  const dwb_msgs::msg::Trajectory2D & trajectory,double best_score)
{
  auto score=DWBLocalPlanner::scoreTrajectory(trajectory,best_score);
  if (!trajectory.poses.empty()) {
    score.total+=heading_scale_*std::abs(angle(generator_->headingGoal()-trajectory.poses.back().theta));
  }
  return score;
}

void RangerProgressChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, const std::string & name)
{
  parent_=parent;
  SimpleProgressChecker::initialize(parent,name);
  setNavigationFailure(parent_.lock(),"");
}
bool RangerProgressChecker::check(geometry_msgs::msg::PoseStamped & pose)
{
  const bool progressing=SimpleProgressChecker::check(pose);
  if (!progressing) {setNavigationFailure(parent_.lock(),"no_progress");}
  return progressing;
}
void RangerProgressChecker::reset()
{
  SimpleProgressChecker::reset();
  setNavigationFailure(parent_.lock(),"");
}
}  // namespace vlm_nav

PLUGINLIB_EXPORT_CLASS(vlm_nav::RangerTrajectoryGenerator,dwb_core::TrajectoryGenerator)
PLUGINLIB_EXPORT_CLASS(vlm_nav::RangerDWBController,nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(vlm_nav::RangerProgressChecker,nav2_core::ProgressChecker)
