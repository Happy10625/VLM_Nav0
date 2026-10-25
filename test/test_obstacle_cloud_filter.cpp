#include <gtest/gtest.h>

#include <cstring>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "vlm_nav/obstacle_cloud_filter.hpp"

namespace
{
sensor_msgs::msg::PointCloud2 make_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.header.stamp.sec = 17;
  cloud.header.stamp.nanosec = 42;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(5);

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
  const std::vector<std::vector<float>> points{
    {1.0F, 0.0F, 0.01F, 10.0F},       // removed by PassThrough
    {0.0F, 0.0F, 0.50F, 20.0F},       // removed by negative CropBox
    {1.0F, 0.0F, 0.50F, 30.0F},
    {1.01F, 0.0F, 0.50F, 31.0F},      // shares the previous voxel
    {1.2F, 0.0F, 0.50F, 40.0F},
  };
  for (const auto & point : points) {
    *x = point[0];
    *y = point[1];
    *z = point[2];
    *intensity = point[3];
    ++x;
    ++y;
    ++z;
    ++intensity;
  }
  return cloud;
}
}  // namespace

namespace
{
sensor_msgs::msg::PointCloud2 scan(
  double seconds, const std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "odom";
  const auto ns = static_cast<int64_t>(std::llround(seconds * 1e9));
  cloud.header.stamp.sec = ns / 1000000000;
  cloud.header.stamp.nanosec = ns % 1000000000;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto & p : points) {
    *x = p[0]; *y = p[1]; *z = p[2];
    ++x; ++y; ++z;
  }
  return cloud;
}
const std::vector<std::array<float, 3>> hit{{1.01F, .01F, .5F}};
}

TEST(PlanarConfirmation, RequiresThreeCompletedCyclesNotThreeFramesOrPoints)
{
  vlm_nav::PlanarObstacleConfirmation confirmation(.05, .8, 3);
  for (double t : {1., 1.1, 1.2, 1.79, 1.8, 2., 2.59, 2.6, 3.39}) {
    EXPECT_EQ(confirmation.filter(scan(t, {hit[0], hit[0], hit[0]})).width, 0u);
  }
  auto output = confirmation.filter(scan(3.4, hit));
  EXPECT_EQ(output.width, 1u);
  EXPECT_EQ(output.header.frame_id, "odom");
  EXPECT_EQ(output.header.stamp.nanosec, 400000000u);
}

TEST(PlanarConfirmation, CountsXYRegardlessOfHeightAndKeepsOnlyCurrentConfirmedPoints)
{
  vlm_nav::PlanarObstacleConfirmation confirmation(.05, .8, 3);
  EXPECT_EQ(confirmation.filter(scan(1., {{1.01F, .01F, .2F}})).width, 0u);
  EXPECT_EQ(confirmation.filter(scan(1.8, {{1.02F, .02F, 1.2F}})).width, 0u);
  EXPECT_EQ(confirmation.filter(scan(2.6, hit)).width, 0u);
  const auto current = scan(3.4, {{1.03F, .03F, .8F}, {2.01F, .01F, .5F}});
  auto output = confirmation.filter(current);
  EXPECT_EQ(output.width, 1u);
  sensor_msgs::PointCloud2ConstIterator<float> z(output, "z");
  EXPECT_FLOAT_EQ(*z, .8F);
  EXPECT_EQ(output.fields, current.fields);
  EXPECT_EQ(confirmation.filter(scan(3.5, {})).width, 0u);
}

TEST(PlanarConfirmation, MissingCycleAndStreamGapRequireFreshConfirmation)
{
  vlm_nav::PlanarObstacleConfirmation confirmation(.05, .8, 3);
  confirmation.filter(scan(1., hit));
  confirmation.filter(scan(1.8, {}));
  for (double t : {2.6, 3.4, 4.2}) {
    EXPECT_EQ(confirmation.filter(scan(t, hit)).width, 0u);
  }
  EXPECT_EQ(confirmation.filter(scan(5., hit)).width, 1u);
  EXPECT_EQ(confirmation.filter(scan(7.4, hit)).width, 0u);
  EXPECT_EQ(confirmation.filter(scan(8.2, hit)).width, 0u);
  EXPECT_EQ(confirmation.filter(scan(9., hit)).width, 0u);
  EXPECT_EQ(confirmation.filter(scan(9.8, hit)).width, 1u);
}

TEST(PlanarConfirmation, DuplicateOutOfOrderFrameChangesAndResetCannotReuseHistory)
{
  vlm_nav::PlanarObstacleConfirmation confirmation(.05, .8, 3);
  confirmation.filter(scan(1., hit));
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(confirmation.filter(scan(1., hit)).width, 0u);
  }
  confirmation.filter(scan(1.8, hit));
  confirmation.filter(scan(2.6, hit));
  EXPECT_EQ(confirmation.filter(scan(3.4, hit)).width, 1u);
  EXPECT_EQ(confirmation.filter(scan(2., hit)).width, 0u);
  EXPECT_EQ(confirmation.filter(scan(3.5, hit)).width, 0u);
  confirmation.reset();
  EXPECT_EQ(confirmation.filter(scan(3.6, hit)).width, 0u);
  auto other_frame = scan(4.4, hit);
  other_frame.header.frame_id = "map";
  EXPECT_EQ(confirmation.filter(other_frame).width, 0u);
  EXPECT_EQ(confirmation.filter(scan(5.2, hit)).width, 0u);
}

TEST(PlanarConfirmation, RejectsInvalidSettings)
{
  EXPECT_THROW(vlm_nav::PlanarObstacleConfirmation(0., .8, 3), std::invalid_argument);
  EXPECT_THROW(vlm_nav::PlanarObstacleConfirmation(.05, 0., 3), std::invalid_argument);
  EXPECT_THROW(vlm_nav::PlanarObstacleConfirmation(.05, .8, 0), std::invalid_argument);
}

TEST(PlanarConfirmation, LateFirstHitStillRequiresFullConfirmationDuration)
{
  vlm_nav::PlanarObstacleConfirmation confirmation(.05, .8, 3);
  confirmation.filter(scan(1., {}));
  for (double t : {1.79, 1.8, 2.6, 3.4}) {
    EXPECT_EQ(confirmation.filter(scan(t, hit)).width, 0u);
  }
  EXPECT_EQ(confirmation.filter(scan(4.19, hit)).width, 1u);
}

TEST(PlanarConfirmation, AdjacentPlanarCellsDoNotShareEvidence)
{
  vlm_nav::PlanarObstacleConfirmation confirmation(.05, .8, 3);
  confirmation.filter(scan(1., {{-.01F, .01F, .5F}}));
  confirmation.filter(scan(1.8, {{.01F, .01F, .5F}}));
  confirmation.filter(scan(2.6, {{-.01F, .01F, .5F}}));
  EXPECT_EQ(confirmation.filter(scan(3.4, {{.01F, .01F, .5F}})).width, 0u);
}

TEST(ObstacleCloudFilter, RejectsInvalidParameters)
{
  auto parameters = vlm_nav::FilterParameters{};
  parameters.min_height = parameters.max_height;
  EXPECT_THROW(vlm_nav::validate_filter_parameters(parameters), std::invalid_argument);

  parameters = vlm_nav::FilterParameters{};
  parameters.self_crop_min_x = parameters.self_crop_max_x;
  EXPECT_THROW(vlm_nav::validate_filter_parameters(parameters), std::invalid_argument);

  parameters = vlm_nav::FilterParameters{};
  parameters.voxel_leaf_size = 0.0;
  EXPECT_THROW(vlm_nav::validate_filter_parameters(parameters), std::invalid_argument);
}

TEST(ObstacleCloudFilter, PreservesHeaderFieldsAndStageCounts)
{
  auto parameters = vlm_nav::FilterParameters{};
  parameters.voxel_leaf_size = 0.05;
  vlm_nav::FilterStatistics statistics;

  const auto output = vlm_nav::filter_base_link_cloud(make_cloud(), parameters, statistics);

  EXPECT_EQ(output.header.frame_id, "base_link");
  EXPECT_EQ(output.header.stamp.sec, 17);
  EXPECT_EQ(output.header.stamp.nanosec, 42u);
  EXPECT_EQ(statistics.input_points, 5u);
  EXPECT_EQ(statistics.after_height, 4u);
  EXPECT_EQ(statistics.after_self_crop, 3u);
  EXPECT_EQ(statistics.after_voxel, 2u);
  EXPECT_EQ(statistics.output_points, 2u);
  EXPECT_EQ(output.width * output.height, statistics.output_points);

  bool has_intensity = false;
  for (const auto & field : output.fields) {
    has_intensity = has_intensity || field.name == "intensity";
  }
  EXPECT_TRUE(has_intensity);
}
