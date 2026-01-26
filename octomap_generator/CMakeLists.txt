cmake_minimum_required(VERSION 3.8)
project(octomap_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)

include_directories(
  include
  ${octomap_INCLUDE_DIRS}
  ${tinyxml2_INCLUDE_DIRS}
)

add_library(octomap_publisher_lib src/octomap_publisher.cpp)
ament_target_dependencies(octomap_publisher_lib
  rclcpp
  std_msgs
  visualization_msgs
  octomap_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(octomap_publisher_lib
  ${octomap_LIBRARIES}
  ${tinyxml2_LIBRARIES}
)

add_executable(octomap_publish_node src/octomap_publish_node.cpp)
ament_target_dependencies(octomap_publish_node
  rclcpp
)
target_link_libraries(octomap_publish_node
  octomap_publisher_lib
)

install(TARGETS
  octomap_publish_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
