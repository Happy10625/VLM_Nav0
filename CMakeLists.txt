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

ament_python_install_package(${PROJECT_NAME})

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
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config behavior_trees DESTINATION share/${PROJECT_NAME})
install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  FILES_MATCHING PATTERN "*.sh" PATTERN "*.py"
)
install(FILES package.xml pytest.ini DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

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
  ament_add_pytest_test(test_geometry test/test_geometry.py)
  ament_add_pytest_test(test_latest_frame_worker test/test_latest_frame_worker.py)
  ament_add_pytest_test(test_navigation_gate test/test_navigation_gate.py)
  ament_add_pytest_test(test_pipeline test/test_pipeline.py)
  ament_add_pytest_test(test_vlm_client test/test_vlm_client.py)
  ament_add_pytest_test(test_obstacle_chain_config test/test_obstacle_chain_config.py)
endif()

ament_package()
