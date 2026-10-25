#include <gtest/gtest.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <limits>
#include "vlm_nav/ranger_dwb_controller.hpp"
#include "dwb_critics/obstacle_footprint.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#include "nav2_core/progress_checker.hpp"
#include "pluginlib/class_loader.hpp"

using vlm_nav::RangerTrajectoryGenerator;
using Twist = nav_2d_msgs::msg::Twist2D;

static auto makeCostmap()
{
  auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("ranger_test_costmap");
  map->set_parameters({rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("footprint_padding",.03),rclcpp::Parameter("global_frame","odom")});
  if (map->on_configure(rclcpp_lifecycle::State())!=nav2_util::CallbackReturn::SUCCESS) {
    throw std::runtime_error("test costmap configuration failed");
  }
  map->getCostmap()->resizeMap(800,800,.01,-4.,-4.);
  for (unsigned int x=0;x<800;x++) for(unsigned int y=0;y<800;y++) {
    map->getCostmap()->setCost(x,y,nav2_costmap_2d::FREE_SPACE);
  }
  std::vector<geometry_msgs::msg::Point> footprint;
  for (auto xy : {std::pair<double,double>{.36,.25},{.36,-.25},{-.36,-.25},{-.36,.25}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  map->setRobotFootprint(footprint);
  return map;
}

class RangerControl : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    node = std::make_shared<nav2_util::LifecycleNode>("ranger_controller_test");
    for (const auto & p : std::vector<rclcpp::Parameter>{
        {"controller_frequency",20.}, {"FollowPath.max_vel_x",.2},
        {"FollowPath.max_vel_theta",.4}, {"FollowPath.acc_lim_x",.2},
        {"FollowPath.decel_lim_x",-.2}, {"FollowPath.acc_lim_theta",.3},
        {"FollowPath.decel_lim_theta",-.3}, {"FollowPath.sim_time",1.7},
        {"FollowPath.linear_granularity",.01}, {"FollowPath.angular_granularity",.01}})
    {
      node->declare_parameter(p.get_name(),p.get_parameter_value());
    }
    generator.initialize(node,"FollowPath");
  }

  void path(double heading)
  {
    nav_2d_msgs::msg::Path2D plan;
    geometry_msgs::msg::Pose2D point;
    plan.poses.push_back(point);
    point.x=2*std::cos(heading);point.y=2*std::sin(heading);
    plan.poses.push_back(point);
    generator.setPath({},plan,.025,.5);
  }

  nav2_util::LifecycleNode::SharedPtr node;
  RangerTrajectoryGenerator generator;
};

TEST_F(RangerControl, MixedCandidatesAllowBothTurnDirectionsBelowPointThree)
{
  path(.2);
  for (auto start : std::vector<Twist>{Twist(), [](){Twist v;v.x=.1;v.theta=.2;return v;}(),
      [](){Twist v;v.theta=.4;return v;}(), [](){Twist v;v.theta=-.4;return v;}()}) {
    generator.startNewIteration(start);
    bool left=false,right=false;
    while (generator.hasMoreTwists()) {
      auto target=generator.nextTwist();
      if (target.x!=0.) {
        EXPECT_LT(std::abs(target.theta),.3);
        left=left || target.theta>0.;right=right || target.theta<0.;
      }
      const auto trajectory=generator.generateTrajectory({},start,target);
      EXPECT_TRUE(trajectory.velocity.x==0. || std::abs(trajectory.velocity.theta)<.3);
      EXPECT_EQ(trajectory.velocity.y,0.);
    }
    EXPECT_TRUE(left);EXPECT_TRUE(right);
  }
}

TEST_F(RangerControl, AxisTransitionsAllowMixedMotionWithoutWaitingForZero)
{
  path(.2);Twist target;target.x=.2;target.theta=.2;
  Twist moving;moving.x=.2;
  auto command=generator.generateTrajectory({},moving,target).velocity;
  EXPECT_NEAR(command.x,.2,1e-9);EXPECT_NEAR(command.theta,.015,1e-9);
  moving.x=0.;moving.theta=.2;
  command=generator.generateTrajectory({},moving,target).velocity;
  EXPECT_NEAR(command.x,.01,1e-9);EXPECT_NEAR(command.theta,.2,1e-9);
  moving.theta=.4;
  for (int i=0;i<30;i++) {
    command=generator.generateTrajectory({},moving,target).velocity;
    EXPECT_TRUE(command.x==0. || std::abs(command.theta)<.3);
    if (command.x>0.) {EXPECT_GT(command.theta,0.);return;}
    moving=command;
  }
  FAIL()<<"rotation must not need to stop before translation starts";
}

TEST_F(RangerControl, PureRotationRetainsPointFourAndSmallRadiusMatchesChassis)
{
  path(1.57);Twist start;start.theta=.4;Twist target=start;
  EXPECT_DOUBLE_EQ(generator.generateTrajectory({},start,target).velocity.theta,.4);
  start.theta=.2;target=start;target.x=.005;
  const auto trajectory=generator.generateTrajectory({},start,target);
  EXPECT_GT(trajectory.velocity.x,0.);EXPECT_NEAR(trajectory.velocity.theta,.2,1e-9);
  for (const auto & pose : trajectory.poses) {
    EXPECT_EQ(pose.x,0.);EXPECT_EQ(pose.y,0.);
  }
  EXPECT_GT(trajectory.poses.back().theta,.2);
}

TEST_F(RangerControl, MixedTrajectoryPredictsTheScoredArc)
{
  path(.2);Twist moving;moving.x=.2;moving.theta=.2;
  const auto trajectory=generator.generateTrajectory({},moving,moving);
  EXPECT_DOUBLE_EQ(trajectory.velocity.x,.2);EXPECT_DOUBLE_EQ(trajectory.velocity.theta,.2);
  EXPECT_GT(trajectory.poses.back().x,.2);EXPECT_GT(trajectory.poses.back().y,.02);
  auto map=makeCostmap();
  dwb_critics::ObstacleFootprintCritic critic;
  critic.initialize(node,"ObstacleFootprint","FollowPath",map);
  critic.prepare({}, {}, {}, {});
  EXPECT_NO_THROW(critic.scoreTrajectory(trajectory));
  unsigned int mx,my;ASSERT_TRUE(map->getCostmap()->worldToMap(.60,.44,mx,my));
  map->getCostmap()->setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
  dwb_msgs::msg::Trajectory2D initial;initial.poses.push_back(trajectory.poses.front());
  EXPECT_NO_THROW(critic.scoreTrajectory(initial));
  EXPECT_THROW(critic.scoreTrajectory(trajectory),dwb_core::IllegalTrajectoryException);
  map->on_cleanup(rclcpp_lifecycle::State());
}

TEST_F(RangerControl, InvalidRobotPoseCannotProduceMotionCandidates)
{
  geometry_msgs::msg::Pose2D pose;
  pose.x=std::numeric_limits<double>::quiet_NaN();
  nav_2d_msgs::msg::Path2D plan;plan.poses.resize(2);plan.poses.back().x=1.;
  EXPECT_THROW(generator.setPath(pose,plan,.025,.5),nav2_core::PlannerException);
}

TEST_F(RangerControl, RolloutIncludesMeasuredStartBrakingAndFineRotationSamples)
{
  path(1.57);Twist start;start.x=.2;
  generator.startNewIteration(start);
  auto trajectory=generator.generateTrajectory({},start,generator.nextTwist());
  EXPECT_NEAR(trajectory.velocity.x,.19,1e-9);EXPECT_EQ(trajectory.velocity.theta,0.);
  EXPECT_GT(trajectory.poses.back().x,.05);
  EXPECT_EQ(trajectory.poses.front().x,0.);
  path(1.57);generator.startNewIteration({});
  Twist target;target.theta=.4;
  trajectory=generator.generateTrajectory({}, {}, target);
  EXPECT_NEAR(trajectory.velocity.theta,.015,1e-9);
  for (size_t i=1;i<trajectory.poses.size();i++) {
    EXPECT_LE(std::abs(trajectory.poses[i].theta-trajectory.poses[i-1].theta),.010001);
    EXPECT_GT(trajectory.time_offsets[i].sec+trajectory.time_offsets[i].nanosec*1e-9,
      trajectory.time_offsets[i-1].sec+trajectory.time_offsets[i-1].nanosec*1e-9);
  }
}

TEST_F(RangerControl, ExistingFootprintCriticRejectsIntermediateCornerCollision)
{
  auto map=makeCostmap();
  dwb_critics::ObstacleFootprintCritic critic;
  critic.initialize(node,"ObstacleFootprint","FollowPath",map);
  critic.prepare({}, {}, {}, {});
  path(1.57);generator.startNewIteration({});
  Twist target;target.theta=.4;
  Twist moving;moving.theta=.4;
  auto trajectory=generator.generateTrajectory({}, moving, target);
  EXPECT_NO_THROW(critic.scoreTrajectory(trajectory));
  unsigned int mx,my;ASSERT_TRUE(map->getCostmap()->worldToMap(.45,0.,mx,my));
  map->getCostmap()->setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
  dwb_msgs::msg::Trajectory2D initial;initial.poses.push_back(trajectory.poses.front());
  EXPECT_NO_THROW(critic.scoreTrajectory(initial));
  EXPECT_THROW(critic.scoreTrajectory(trajectory),dwb_core::IllegalTrajectoryException);
  map->on_cleanup(rclcpp_lifecycle::State());
}

static nav_msgs::msg::Path northPath(double heading)
{
  nav_msgs::msg::Path path;path.header.frame_id="odom";
  for (int i=0;i<=40;i++) {
    geometry_msgs::msg::PoseStamped p;p.header=path.header;
    p.pose.position.x=.05*i*std::cos(heading);p.pose.position.y=.05*i*std::sin(heading);
    p.pose.orientation.w=1.;path.poses.push_back(p);
  }
  return path;
}

TEST_F(RangerControl, RealDWBSelectsMixedMotionAndReplanningHasNoModeLatch)
{
  node->declare_parameter("FollowPath.trajectory_generator_name","vlm_nav::RangerTrajectoryGenerator");
  node->declare_parameter("FollowPath.critics",std::vector<std::string>{
      "ObstacleFootprint","BaseObstacle","PathAlign","PathDist","GoalDist"});
  auto map=makeCostmap();vlm_nav::RangerDWBController controller;
  controller.configure(node,"FollowPath",map->getTfBuffer(),map);controller.activate();
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="odom";pose.pose.orientation.w=1.;
  controller.setPlan(northPath(.2));
  auto command=controller.computeVelocityCommands(pose,geometry_msgs::msg::Twist(),nullptr).twist;
  EXPECT_GT(command.linear.x,0.);EXPECT_GT(command.angular.z,0.);EXPECT_LT(command.angular.z,.3);
  controller.setPlan(northPath(-.2));
  command=controller.computeVelocityCommands(pose,geometry_msgs::msg::Twist(),nullptr).twist;
  EXPECT_GT(command.linear.x,0.);EXPECT_LT(command.angular.z,0.);EXPECT_GT(command.angular.z,-.3);
  controller.deactivate();controller.cleanup();map->on_cleanup(rclcpp_lifecycle::State());
}

TEST_F(RangerControl, RecoveryReasonIsOnlySetForNoLegalTrajectoriesAndClearsBeforeOtherFailures)
{
  node->declare_parameter("ranger_recovery.reason",std::string(""));
  node->declare_parameter("FollowPath.trajectory_generator_name","vlm_nav::RangerTrajectoryGenerator");
  node->declare_parameter("FollowPath.critics",std::vector<std::string>{"ObstacleFootprint","BaseObstacle"});
  auto map=makeCostmap();vlm_nav::RangerDWBController controller;
  controller.configure(node,"FollowPath",map->getTfBuffer(),map);controller.activate();
  controller.setPlan(northPath(0.));
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="odom";pose.pose.orientation.w=1.;
  unsigned int mx,my;ASSERT_TRUE(map->getCostmap()->worldToMap(0.,0.,mx,my));
  map->getCostmap()->setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller.computeVelocityCommands(pose,geometry_msgs::msg::Twist(),nullptr),
    dwb_core::NoLegalTrajectoriesException);
  EXPECT_EQ(node->get_parameter("ranger_recovery.reason").as_string(),"no_valid_trajectory");
  map->getCostmap()->setCost(mx,my,nav2_costmap_2d::FREE_SPACE);
  EXPECT_NO_THROW(controller.computeVelocityCommands(pose,geometry_msgs::msg::Twist(),nullptr));
  EXPECT_EQ(node->get_parameter("ranger_recovery.reason").as_string(),"");
  node->set_parameter(rclcpp::Parameter("ranger_recovery.reason","no_valid_trajectory"));
  pose.header.frame_id="missing_frame";
  EXPECT_THROW(controller.computeVelocityCommands(pose,geometry_msgs::msg::Twist(),nullptr),nav2_core::PlannerException);
  EXPECT_EQ(node->get_parameter("ranger_recovery.reason").as_string(),"");
  controller.deactivate();controller.cleanup();map->on_cleanup(rclcpp_lifecycle::State());
}

TEST_F(RangerControl, ProgressFailureIsClassifiedAndNewGoalResetClearsIt)
{
  pluginlib::ClassLoader<nav2_core::ProgressChecker> loader("nav2_core","nav2_core::ProgressChecker");
  auto checker=loader.createSharedInstance("vlm_nav::RangerProgressChecker");
  node->declare_parameter("progress_checker.required_movement_radius",.1);
  node->declare_parameter("progress_checker.movement_time_allowance",.01);
  checker->initialize(node,"progress_checker");
  geometry_msgs::msg::PoseStamped pose;
  checker->reset();EXPECT_TRUE(checker->check(pose));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  EXPECT_FALSE(checker->check(pose));
  EXPECT_EQ(node->get_parameter("ranger_recovery.reason").as_string(),"no_progress");
  checker->reset();EXPECT_TRUE(checker->check(pose));
  EXPECT_EQ(node->get_parameter("ranger_recovery.reason").as_string(),"");
}
