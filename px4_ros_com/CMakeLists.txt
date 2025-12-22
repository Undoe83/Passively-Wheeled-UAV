cmake_minimum_required(VERSION 3.5)
project(px4_ros_com)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(casadi REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# Add frame_transforms lib
add_library(frame_transforms SHARED src/lib/frame_transforms.cpp)
ament_target_dependencies(frame_transforms Eigen3 geometry_msgs sensor_msgs)
target_include_directories(frame_transforms PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)


# examples/listeners/sensor_combined_listener
add_executable(sensor_combined_listener src/examples/listeners/sensor_combined_listener.cpp)
ament_target_dependencies(sensor_combined_listener rclcpp px4_msgs)
install(TARGETS sensor_combined_listener DESTINATION lib/${PROJECT_NAME})

# examples/listeners/vehicle_gps_position_listener
add_executable(vehicle_gps_position_listener src/examples/listeners/vehicle_gps_position_listener.cpp)
ament_target_dependencies(vehicle_gps_position_listener rclcpp px4_msgs)
install(TARGETS vehicle_gps_position_listener DESTINATION lib/${PROJECT_NAME})

# examples/listeners/odometry_converter
add_executable(odometry_converter src/examples/listeners/odometry_converter.cpp)
ament_target_dependencies(odometry_converter rclcpp px4_msgs nav_msgs)
target_link_libraries(odometry_converter frame_transforms)
install(TARGETS odometry_converter DESTINATION lib/${PROJECT_NAME})

# examples/advertisers/debug_vect_advertiser
add_executable(debug_vect_advertiser src/examples/advertisers/debug_vect_advertiser.cpp)
ament_target_dependencies(debug_vect_advertiser rclcpp px4_msgs)
install(TARGETS debug_vect_advertiser DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control
add_executable(offboard_control src/examples/offboard/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs geometry_msgs sensor_msgs)
target_link_libraries(offboard_control frame_transforms)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})

# examples/offboard/offboard_control_srv
add_executable(offboard_control_srv src/examples/offboard/offboard_control_srv.cpp)
ament_target_dependencies(offboard_control_srv rclcpp px4_msgs)
install(TARGETS offboard_control_srv DESTINATION lib/${PROJECT_NAME})

# waypoint_generator/waypoint_generator
add_executable(waypoint_generator src/waypoint_generator/waypoint_generator.cpp)
ament_target_dependencies(waypoint_generator rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)
install(TARGETS waypoint_generator DESTINATION lib/${PROJECT_NAME})

# waypoint_generator/reference_path_publisher
add_executable(reference_path_publisher src/waypoint_generator/reference_path_publisher.cpp)
ament_target_dependencies(reference_path_publisher rclcpp nav_msgs px4_msgs tf2 tf2_geometry_msgs)
install(TARGETS reference_path_publisher DESTINATION lib/${PROJECT_NAME})

# traj_opt/trajectory_optimizer
# add_executable(trajectory_optimizer src/traj_opt/trajectory_optimizer.cpp)
# ament_target_dependencies(trajectory_optimizer rclcpp nav_msgs trajectory_msgs visualization_msgs)
# install(TARGETS trajectory_optimizer DESTINATION lib/${PROJECT_NAME})

# controller
add_executable(mpc_controller_node src/controller/mpc.cpp)
ament_target_dependencies(mpc_controller_node rclcpp px4_msgs nav_msgs casadi tf2 tf2_geometry_msgs)
target_link_libraries(mpc_controller_node frame_transforms)
if(TARGET casadi::casadi)
	target_link_libraries(mpc_controller_node casadi::casadi)
else()
	# Fallback: link against ${casadi_LIBRARIES} if provided by the find_package
	if(DEFINED casadi_LIBRARIES)
		target_link_libraries(mpc_controller_node ${casadi_LIBRARIES})
	endif()
endif()
install(TARGETS mpc_controller_node DESTINATION lib/${PROJECT_NAME})


############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

ament_export_targets(export_frame_transforms HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_libraries(frame_transforms)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(TARGETS frame_transforms
	EXPORT export_frame_transforms
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install tests
install(DIRECTORY test DESTINATION share/${PROJECT_NAME}/)


############
# Testing ##
############

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# uncomment the line when a copyright and license is not present in all source files
	#set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# uncomment the line when this package is not in a git repo
	#set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

###########
# Python ##
###########

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/examples/offboard_py/offboard_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
