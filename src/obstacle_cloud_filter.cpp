#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <message_filters/subscriber.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include "vlm_nav/obstacle_cloud_filter.hpp"

namespace vlm_nav
{

class ObstacleCloudFilterNode : public rclcpp::Node
{
public:
  using Cloud = sensor_msgs::msg::PointCloud2;
  using SteadyClock = std::chrono::steady_clock;

  ObstacleCloudFilterNode()
  : Node("obstacle_cloud_filter"),
    last_diagnostic_at_(SteadyClock::now())
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered_body");
    output_topic_ = declare_parameter<std::string>("output_topic", "/vlm_nav/obstacle_cloud");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    tf_queue_size_ = declare_parameter<int>("tf_queue_size", 20);
    parameters_.min_height = declare_parameter<double>("min_height", 0.05);
    parameters_.max_height = declare_parameter<double>("max_height", 1.50);
    parameters_.self_crop_min_x = declare_parameter<double>("self_crop_min_x", -0.36);
    parameters_.self_crop_max_x = declare_parameter<double>("self_crop_max_x", 0.36);
    parameters_.self_crop_min_y = declare_parameter<double>("self_crop_min_y", -0.25);
    parameters_.self_crop_max_y = declare_parameter<double>("self_crop_max_y", 0.25);
    parameters_.self_crop_min_z = declare_parameter<double>("self_crop_min_z", 0.05);
    parameters_.self_crop_max_z = declare_parameter<double>("self_crop_max_z", 1.50);
    parameters_.voxel_leaf_size = declare_parameter<double>("voxel_leaf_size", 0.05);
    validate_configuration();

    publisher_ = create_publisher<Cloud>(output_topic_, rclcpp::SensorDataQoS());
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_subscriber_ = std::make_shared<message_filters::Subscriber<Cloud>>(
      this, input_topic_, rmw_qos_profile_sensor_data);
    // Humble's MessageFilter waits for the transform at cloud.header.stamp.
    // Its time tolerance remains the default zero: no stamp + tolerance TF is requested.
    cloud_filter_ = std::make_shared<tf2_ros::MessageFilter<Cloud>>(
      *cloud_subscriber_, *tf_buffer_, target_frame_,
      static_cast<std::uint32_t>(tf_queue_size_), get_node_logging_interface(),
      get_node_clock_interface());
    cloud_filter_->setTolerance(rclcpp::Duration::from_seconds(0.0));
    cloud_filter_->registerCallback(
      std::bind(&ObstacleCloudFilterNode::on_cloud, this, std::placeholders::_1));

    updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    updater_->setHardwareID("obstacle_cloud_filter");
    updater_->add(
      "Obstacle cloud preprocessing", this,
      &ObstacleCloudFilterNode::produce_diagnostics);

    RCLCPP_INFO(
      get_logger(), "Filtering %s into %s in frame %s",
      input_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
  }

private:
  struct RuntimeStatistics
  {
    FilterStatistics points;
    std::uint64_t processed_count{0};
    double processing_latency_ms{0.0};
    std::optional<SteadyClock::time_point> last_success;
  };

  void validate_configuration()
  {
    if (input_topic_.empty() || output_topic_.empty() || target_frame_.empty()) {
      throw std::invalid_argument("input_topic, output_topic, and target_frame must not be empty");
    }
    if (tf_queue_size_ <= 0) {
      throw std::invalid_argument("tf_queue_size must be greater than zero");
    }
    validate_filter_parameters(parameters_);
  }

  void on_cloud(const Cloud::ConstSharedPtr & cloud)
  {
    const auto started = SteadyClock::now();
    Cloud transformed;
    try {
      if (!pcl_ros::transformPointCloud(target_frame_, *cloud, transformed, *tf_buffer_)) {
        RCLCPP_ERROR(
          get_logger(), "TF became unavailable while transforming cloud at stamp %d.%09u",
          cloud->header.stamp.sec, cloud->header.stamp.nanosec);
        return;
      }
      transformed.header.stamp = cloud->header.stamp;
      transformed.header.frame_id = target_frame_;

      FilterStatistics points;
      auto output = filter_base_link_cloud(transformed, parameters_, points);
      output.header.stamp = cloud->header.stamp;
      output.header.frame_id = target_frame_;
      publisher_->publish(output);

      const auto completed = SteadyClock::now();
      const auto latency = std::chrono::duration<double, std::milli>(completed - started).count();
      std::lock_guard<std::mutex> lock(statistics_mutex_);
      statistics_.points = points;
      statistics_.processing_latency_ms = latency;
      statistics_.last_success = completed;
      ++statistics_.processed_count;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Obstacle cloud processing failed: %s", error.what());
    }
  }

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    const auto now = SteadyClock::now();
    RuntimeStatistics snapshot;
    double frequency = 0.0;
    {
      std::lock_guard<std::mutex> lock(statistics_mutex_);
      snapshot = statistics_;
      const auto elapsed = std::chrono::duration<double>(now - last_diagnostic_at_).count();
      if (elapsed > 0.0) {
        frequency = static_cast<double>(
          snapshot.processed_count - last_diagnostic_count_) / elapsed;
      }
      last_diagnostic_count_ = snapshot.processed_count;
      last_diagnostic_at_ = now;
    }

    const double last_success_age = snapshot.last_success.has_value() ?
      std::chrono::duration<double>(now - *snapshot.last_success).count() :
      std::numeric_limits<double>::infinity();
    if (!snapshot.last_success.has_value()) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No cloud has been processed");
    } else if (last_success_age > 2.5) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Obstacle cloud stream is stale");
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Obstacle cloud processing active");
    }
    status.add("processed_frequency_hz", frequency);
    status.add("input_points", snapshot.points.input_points);
    status.add("after_height", snapshot.points.after_height);
    status.add("after_self_crop", snapshot.points.after_self_crop);
    status.add("after_voxel", snapshot.points.after_voxel);
    status.add("output_points", snapshot.points.output_points);
    status.add("processing_latency_ms", snapshot.processing_latency_ms);
    status.add("last_success_age_s", last_success_age);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  int tf_queue_size_{20};
  FilterParameters parameters_;

  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<message_filters::Subscriber<Cloud>> cloud_subscriber_;
  std::shared_ptr<tf2_ros::MessageFilter<Cloud>> cloud_filter_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;

  std::mutex statistics_mutex_;
  RuntimeStatistics statistics_;
  SteadyClock::time_point last_diagnostic_at_;
  std::uint64_t last_diagnostic_count_{0};
};

}  // namespace vlm_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<vlm_nav::ObstacleCloudFilterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_cloud_filter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
