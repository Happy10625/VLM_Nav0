// Humble Costmap2DROS has no registered component. Host only native costmaps.
#include <memory>
#include <string>
#include <vector>
#include "class_loader/multi_library_class_loader.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

class AcceptanceCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  AcceptanceCostmap(
    const rclcpp::NodeOptions & options, class_loader::MultiLibraryClassLoader & libraries)
  : nav2_costmap_2d::Costmap2DROS(options), libraries_(libraries) {}

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    // Humble callback-group weak references must die before their layer libraries unload.
    for (const auto & library : plugin_loader_.getRegisteredLibraries()) {
      libraries_.loadLibrary(library);
    }
    return nav2_costmap_2d::Costmap2DROS::on_cleanup(state);
  }

private:
  class_loader::MultiLibraryClassLoader & libraries_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  class_loader::MultiLibraryClassLoader libraries(false);
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<std::shared_ptr<nav2_costmap_2d::Costmap2DROS>> nodes;
  for (const std::string name : {"local_costmap", "global_costmap"}) {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__node:=" + name,
      "-r", "__ns:=/go2_costmap_acceptance/" + name,
      "-r", "tf:=/tf", "-r", "tf_static:=/tf_static"});
    auto node = std::make_shared<AcceptanceCostmap>(options, libraries);
    executor.add_node(node->get_node_base_interface());
    nodes.push_back(node);
  }
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
