cmake_minimum_required(VERSION 3.8)
project(vlm_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS filters)
find_package(dwb_core REQUIRED)
find_package(dwb_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rcl_interfaces REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(ranger_dwb_controller SHARED src/ranger_dwb_controller.cpp src/ranger_spin.cpp)
target_compile_features(ranger_dwb_controller PUBLIC cxx_std_17)
target_include_directories(ranger_dwb_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ranger_dwb_controller dwb_core dwb_plugins pluginlib action_msgs nav2_controller nav2_behaviors)
add_library(ranger_recovery_bt_node SHARED src/ranger_recovery_bt.cpp)
target_compile_features(ranger_recovery_bt_node PUBLIC cxx_std_17)
target_compile_definitions(ranger_recovery_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(ranger_recovery_bt_node PRIVATE include)
ament_target_dependencies(ranger_recovery_bt_node nav2_behavior_tree rcl_interfaces)
pluginlib_export_plugin_description_file(nav2_core ranger_controller_plugins.xml)
pluginlib_export_plugin_description_file(dwb_core ranger_controller_plugins.xml)
install(TARGETS ranger_dwb_controller ranger_recovery_bt_node LIBRARY DESTINATION lib)

add_library(obstacle_cloud_filter_pipeline STATIC src/obstacle_cloud_filter_pipeline.cpp)
target_compile_features(obstacle_cloud_filter_pipeline PUBLIC cxx_std_17)
target_include_directories(obstacle_cloud_filter_pipeline
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(obstacle_cloud_filter_pipeline
  pcl_conversions
  sensor_msgs
)
target_link_libraries(obstacle_cloud_filter_pipeline ${PCL_LIBRARIES})

add_executable(obstacle_cloud_filter src/obstacle_cloud_filter.cpp)
target_compile_features(obstacle_cloud_filter PUBLIC cxx_std_17)
target_include_directories(obstacle_cloud_filter PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(obstacle_cloud_filter
  diagnostic_msgs
  diagnostic_updater
  message_filters
  pcl_ros
  rclcpp
  rmw
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(obstacle_cloud_filter
  obstacle_cloud_filter_pipeline
  ${PCL_LIBRARIES}
)

install(
  TARGETS obstacle_cloud_filter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  PROGRAMS
    scripts/fastlio_odom_adapter
    scripts/vlm_navigator
    scripts/ranger_command_guard
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config behavior_trees DESTINATION share/${PROJECT_NAME})
install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING
    PATTERN "*.sh"
    PATTERN "*.py"
    PATTERN "start_fixed_map_navigation.sh" EXCLUDE
)
install(
  PROGRAMS scripts/start_fixed_map_navigation.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)
install(FILES package.xml pytest.ini DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(dwb_critics REQUIRED)

  ament_add_gtest(test_ranger_controller test/test_ranger_controller.cpp)
  target_compile_features(test_ranger_controller PUBLIC cxx_std_17)
  target_link_libraries(test_ranger_controller ranger_dwb_controller)
  ament_target_dependencies(test_ranger_controller dwb_core dwb_plugins dwb_critics action_msgs)
  set_tests_properties(test_ranger_controller PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=31;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/ranger-controller-test-logs")
  ament_add_pytest_test(test_ranger_command_guard test/test_ranger_command_guard.py)
  ament_add_pytest_test(test_ranger_recovery test/test_ranger_recovery.py TIMEOUT 120)

  ament_add_gtest(test_obstacle_cloud_filter test/test_obstacle_cloud_filter.cpp)
  if(TARGET test_obstacle_cloud_filter)
    target_compile_features(test_obstacle_cloud_filter PUBLIC cxx_std_17)
    target_include_directories(test_obstacle_cloud_filter PRIVATE ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_obstacle_cloud_filter sensor_msgs)
    target_link_libraries(test_obstacle_cloud_filter
      obstacle_cloud_filter_pipeline
      ${PCL_LIBRARIES}
    )
  endif()

  ament_add_pytest_test(test_arm_image_recorder test/test_arm_image_recorder.py)
  ament_add_pytest_test(test_easy_case_config test/test_easy_case_config.py)
  ament_add_pytest_test(test_exploration test/test_exploration.py)
  ament_add_pytest_test(test_fixed_map_config test/test_fixed_map_config.py)
  ament_add_pytest_test(test_geometry test/test_geometry.py)
  ament_add_pytest_test(test_arrival_contract test/test_arrival_contract.py)
  ament_add_pytest_test(test_ranger_local_launch test/test_ranger_local_launch.py)
  ament_add_pytest_test(test_keepout test/test_keepout.py)
  ament_add_pytest_test(test_ranger_check test/test_ranger_check.py)
  ament_add_pytest_test(test_ranger_startup test/test_ranger_startup.py)
  ament_add_pytest_test(test_ranger_cooperative_startup test/test_ranger_cooperative_startup.py)
  ament_add_pytest_test(test_cooperative_startup test/test_cooperative_startup.py)
  ament_add_pytest_test(test_cooperative_preparation_wait test/test_cooperative_preparation_wait.py)
  ament_add_pytest_test(test_ranger_monitor test/test_ranger_monitor.py)
  ament_add_pytest_test(test_latest_frame_worker test/test_latest_frame_worker.py)
  ament_add_pytest_test(test_navigation_gate test/test_navigation_gate.py)
  ament_add_pytest_test(test_ranger_single_robot_parity test/test_ranger_single_robot_parity.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/cooperative_ws/src/multi_agent_nav")
  ament_add_pytest_test(test_ranger_approach_transport test/test_ranger_approach_transport.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/cooperative_ws/src/multi_agent_nav")
  ament_add_pytest_test(test_pipeline test/test_pipeline.py)
  ament_add_pytest_test(test_nav_ready_checkpoint test/test_nav_ready_checkpoint.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/cooperative_ws/src/multi_agent_nav:${CMAKE_CURRENT_SOURCE_DIR}/cooperative_ws/src/multi_agent_nav/tests)
  ament_add_pytest_test(test_vlm_client test/test_vlm_client.py)
  ament_add_pytest_test(test_api_response_probe test/test_api_response_probe.py)
  ament_add_pytest_test(test_obstacle_chain_config test/test_obstacle_chain_config.py)
  ament_add_pytest_test(test_obstacle_cloud_filter_runtime test/test_obstacle_cloud_filter_runtime.py TIMEOUT 20)
endif()

ament_package()
