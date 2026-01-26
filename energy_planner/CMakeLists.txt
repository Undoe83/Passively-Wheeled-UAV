cmake_minimum_required(VERSION 3.8)
project(energy_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(energy_aware_path_planner src/energy_aware_path_planner.cpp)

ament_target_dependencies(energy_aware_path_planner
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  octomap_msgs
  octomap
)

target_include_directories(energy_aware_path_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(reference_path_publish_node src/reference_path_publish_node.cpp src/reference_path_publisher.cpp)

ament_target_dependencies(reference_path_publish_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_geometry_msgs
)

target_include_directories(reference_path_publish_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
  energy_aware_path_planner
  reference_path_publish_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
