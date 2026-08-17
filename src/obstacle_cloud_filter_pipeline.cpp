#include "vlm_nav/obstacle_cloud_filter.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

namespace vlm_nav
{
namespace
{
std::size_t point_count(const pcl::PCLPointCloud2 & cloud)
{
  return static_cast<std::size_t>(cloud.width) * cloud.height;
}

void require_finite(double value, const char * name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}
}  // namespace

void validate_filter_parameters(const FilterParameters & parameters)
{
  require_finite(parameters.min_height, "min_height");
  require_finite(parameters.max_height, "max_height");
  require_finite(parameters.self_crop_min_x, "self_crop_min_x");
  require_finite(parameters.self_crop_max_x, "self_crop_max_x");
  require_finite(parameters.self_crop_min_y, "self_crop_min_y");
  require_finite(parameters.self_crop_max_y, "self_crop_max_y");
  require_finite(parameters.self_crop_min_z, "self_crop_min_z");
  require_finite(parameters.self_crop_max_z, "self_crop_max_z");
  require_finite(parameters.voxel_leaf_size, "voxel_leaf_size");
  if (parameters.min_height >= parameters.max_height) {
    throw std::invalid_argument("min_height must be less than max_height");
  }
  if (parameters.self_crop_min_x >= parameters.self_crop_max_x ||
    parameters.self_crop_min_y >= parameters.self_crop_max_y ||
    parameters.self_crop_min_z >= parameters.self_crop_max_z)
  {
    throw std::invalid_argument("every self CropBox minimum must be less than its maximum");
  }
  if (parameters.voxel_leaf_size <= 0.0) {
    throw std::invalid_argument("voxel_leaf_size must be greater than zero");
  }
}

sensor_msgs::msg::PointCloud2 filter_base_link_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const FilterParameters & parameters,
  FilterStatistics & statistics)
{
  validate_filter_parameters(parameters);

  pcl::PCLPointCloud2 pcl_input;
  pcl_conversions::toPCL(cloud, pcl_input);
  statistics.input_points = point_count(pcl_input);

  auto input = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2(pcl_input));
  auto height_output = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());
  pcl::PassThrough<pcl::PCLPointCloud2> height_filter;
  height_filter.setInputCloud(input);
  height_filter.setFilterFieldName("z");
  height_filter.setFilterLimits(parameters.min_height, parameters.max_height);
  height_filter.filter(*height_output);
  statistics.after_height = point_count(*height_output);

  auto crop_output = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());
  pcl::CropBox<pcl::PCLPointCloud2> self_crop;
  self_crop.setInputCloud(height_output);
  self_crop.setMin(Eigen::Vector4f(
      parameters.self_crop_min_x, parameters.self_crop_min_y,
      parameters.self_crop_min_z, 1.0F));
  self_crop.setMax(Eigen::Vector4f(
      parameters.self_crop_max_x, parameters.self_crop_max_y,
      parameters.self_crop_max_z, 1.0F));
  self_crop.setNegative(true);
  self_crop.filter(*crop_output);
  statistics.after_self_crop = point_count(*crop_output);

  pcl::PCLPointCloud2 voxel_output;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
  voxel_filter.setInputCloud(crop_output);
  const auto leaf = static_cast<float>(parameters.voxel_leaf_size);
  voxel_filter.setLeafSize(leaf, leaf, leaf);
  voxel_filter.setDownsampleAllData(true);
  voxel_filter.filter(voxel_output);
  statistics.after_voxel = point_count(voxel_output);
  statistics.output_points = statistics.after_voxel;

  sensor_msgs::msg::PointCloud2 output;
  pcl_conversions::fromPCL(voxel_output, output);
  output.header = cloud.header;
  return output;
}

}  // namespace vlm_nav
