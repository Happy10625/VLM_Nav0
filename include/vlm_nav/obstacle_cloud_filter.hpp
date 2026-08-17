#ifndef VLM_NAV__OBSTACLE_CLOUD_FILTER_HPP_
#define VLM_NAV__OBSTACLE_CLOUD_FILTER_HPP_

#include <cstddef>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace vlm_nav
{

struct FilterParameters
{
  double min_height{0.05};
  double max_height{1.50};
  double self_crop_min_x{-0.36};
  double self_crop_max_x{0.36};
  double self_crop_min_y{-0.25};
  double self_crop_max_y{0.25};
  double self_crop_min_z{0.05};
  double self_crop_max_z{1.50};
  double voxel_leaf_size{0.05};
};

struct FilterStatistics
{
  std::size_t input_points{0};
  std::size_t after_height{0};
  std::size_t after_self_crop{0};
  std::size_t after_voxel{0};
  std::size_t output_points{0};
};

void validate_filter_parameters(const FilterParameters & parameters);

sensor_msgs::msg::PointCloud2 filter_base_link_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const FilterParameters & parameters,
  FilterStatistics & statistics);

}  // namespace vlm_nav

#endif  // VLM_NAV__OBSTACLE_CLOUD_FILTER_HPP_
