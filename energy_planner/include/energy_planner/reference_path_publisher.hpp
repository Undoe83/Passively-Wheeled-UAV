#ifndef ENERGY_PLANNER__REFERENCE_PATH_PUBLISHER_HPP_
#define ENERGY_PLANNER__REFERENCE_PATH_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>
#include <string>
#include <map>

namespace energy_planner
{

class TimeAwareReferencePathPublisher : public rclcpp::Node
{
public:
  TimeAwareReferencePathPublisher(const rclcpp::NodeOptions & options);

private:
  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg, const std::string robot_name);
  void publishReferencePaths();
  void checkAllPathsReceived();
  geometry_msgs::msg::PoseStamped interpolatePose(
    const nav_msgs::msg::Path& path, const rclcpp::Time& path_start_time, const rclcpp::Duration& target_elapsed_time);


  // Parameters
  int horizon_length_;
  double publish_rate_;
  std::vector<std::string> robot_names_;
  
  // ROS
  std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> global_path_subs_;
  std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> ref_path_pubs_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data storage
  std::map<std::string, nav_msgs::msg::Path> global_paths_;
  std::map<std::string, bool> path_received_flags_;
  rclcpp::Time start_time_;
  size_t received_paths_count_ = 0;
};

}  // namespace energy_planner

#endif  // ENERGY_PLANNER__REFERENCE_PATH_PUBLISHER_HPP_
