#ifndef VLM_NAV__OBSTACLE_CLOUD_FILTER_HPP_
#define VLM_NAV__OBSTACLE_CLOUD_FILTER_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>

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

class PlanarObstacleConfirmation
{
public:
  PlanarObstacleConfirmation(double cell_size, double cycle_seconds, unsigned int cycles);
  sensor_msgs::msg::PointCloud2 filter(const sensor_msgs::msg::PointCloud2 & cloud);
  void reset();

private:
  using Cell = std::pair<int64_t, int64_t>;
  double cell_size_;
  int64_t cycle_ns_;
  unsigned int required_cycles_;
  std::optional<int64_t> start_ns_, last_ns_;
  int64_t cycle_{0};
  std::string frame_;
  struct Evidence {unsigned int cycles; int64_t first_ns;};
  std::map<Cell, Evidence> completed_;
  std::map<Cell, int64_t> hits_;
};

}  // namespace vlm_nav

#endif  // VLM_NAV__OBSTACLE_CLOUD_FILTER_HPP_
