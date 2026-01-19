cmake_minimum_required(VERSION 3.8)
project(pwuav_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(casadi REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

# waypoint_generator/waypoint_generator
add_executable(waypoint_generator src/waypoint_generator/waypoint_generator.cpp)
ament_target_dependencies(waypoint_generator rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)
install(TARGETS waypoint_generator DESTINATION lib/${PROJECT_NAME})

# waypoint_generator/reference_path_publisher
add_executable(reference_path_publisher src/waypoint_generator/reference_path_publisher.cpp)
ament_target_dependencies(reference_path_publisher rclcpp nav_msgs tf2 tf2_geometry_msgs)
install(TARGETS reference_path_publisher DESTINATION lib/${PROJECT_NAME})

# controller
add_executable(mpc_controller_node src/controller/mpc.cpp)
target_include_directories(mpc_controller_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(mpc_controller_node rclcpp nav_msgs casadi tf2 tf2_geometry_msgs mavros_msgs)
if(TARGET casadi::casadi)
	target_link_libraries(mpc_controller_node casadi::casadi)
else()
	# Fallback: link against ${casadi_LIBRARIES} if provided by the find_package
	if(DEFINED casadi_LIBRARIES)
		target_link_libraries(mpc_controller_node ${casadi_LIBRARIES})
	endif()
endif()
install(TARGETS mpc_controller_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
