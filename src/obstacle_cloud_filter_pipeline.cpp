#include "vlm_nav/obstacle_cloud_filter.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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

PlanarObstacleConfirmation::PlanarObstacleConfirmation(
  double cell_size, double cycle_seconds, unsigned int cycles)
: cell_size_(cell_size), required_cycles_(cycles)
{
  if (!std::isfinite(cell_size) || cell_size <= 0. || !std::isfinite(cycle_seconds) ||
    cycle_seconds < 1e-9 || cycle_seconds > 3600. || cycles == 0)
  {
    throw std::invalid_argument("positive cell size, cycle in [1ns, 1h], and cycle count required");
  }
  cycle_ns_ = static_cast<int64_t>(std::llround(cycle_seconds * 1e9));
}

void PlanarObstacleConfirmation::reset()
{
  start_ns_.reset();
  last_ns_.reset();
  cycle_ = 0;
  frame_.clear();
  completed_.clear();
  hits_.clear();
}

sensor_msgs::msg::PointCloud2 PlanarObstacleConfirmation::filter(
  const sensor_msgs::msg::PointCloud2 & cloud)
{
  auto output = cloud;
  output.height = 1;
  output.width = output.row_step = 0;
  output.data.clear();
  const int64_t stamp = static_cast<int64_t>(cloud.header.stamp.sec) * 1000000000 +
    cloud.header.stamp.nanosec;
  if (last_ns_ && stamp <= *last_ns_) {
    if (stamp < *last_ns_) {reset();}
    return output;
  }
  if (frame_ != cloud.header.frame_id) {reset();}
  if (!start_ns_) {start_ns_ = stamp; frame_ = cloud.header.frame_id;}
  last_ns_ = stamp;
  const int64_t cycle = (stamp - *start_ns_) / cycle_ns_;
  if (cycle > cycle_) {
    std::map<Cell, Evidence> next;
    // A wholly unobserved cycle breaks every streak; only completed cycles count.
    if (cycle == cycle_ + 1) {
      for (const auto & [cell, first_ns] : hits_) {
        const auto previous = completed_.find(cell);
        const auto evidence = previous == completed_.end() ? Evidence{0, first_ns} : previous->second;
        next[cell] = {evidence.cycles < required_cycles_ ? evidence.cycles + 1 : evidence.cycles,
          evidence.first_ns};
      }
    }
    completed_.swap(next);
    hits_.clear();
    cycle_ = cycle;
  }
  if (cloud.width == 0 || cloud.height == 0) {return output;}
  // The upstream PCL output is packed; preserve all fields of accepted current points.
  if (cloud.row_step != cloud.width * cloud.point_step ||
    cloud.data.size() != static_cast<size_t>(cloud.row_step) * cloud.height)
  {
    reset();
    throw std::invalid_argument("confirmation requires a packed point cloud");
  }
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  size_t index = 0;
  for (; x != x.end(); ++x, ++y, ++z, ++index) {
    const double cx = std::floor(*x / cell_size_), cy = std::floor(*y / cell_size_);
    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(*z) ||
      std::abs(cx) >= static_cast<double>(std::numeric_limits<int64_t>::max()) ||
      std::abs(cy) >= static_cast<double>(std::numeric_limits<int64_t>::max()))
    {continue;}
    const Cell cell{static_cast<int64_t>(cx), static_cast<int64_t>(cy)};
    hits_.emplace(cell, stamp);
    const auto count = completed_.find(cell);
    // A first hit near a cycle boundary must not shorten the minimum confirmation time.
    if (count == completed_.end() || count->second.cycles < required_cycles_ ||
      (stamp - count->second.first_ns) / cycle_ns_ < required_cycles_)
    {continue;}
    const auto begin = cloud.data.begin() + index * cloud.point_step;
    output.data.insert(output.data.end(), begin, begin + cloud.point_step);
    ++output.width;
  }
  output.row_step = output.width * output.point_step;
  return output;
}

}  // namespace vlm_nav
