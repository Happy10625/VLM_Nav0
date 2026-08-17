#include <gtest/gtest.h>

#include <cstring>
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
