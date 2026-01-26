#include "energy_planner/reference_path_publisher.hpp"

#include <chrono>
#include <functional>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace energy_planner
{

TimeAwareReferencePathPublisher::TimeAwareReferencePathPublisher(const rclcpp::NodeOptions & options)
: Node("time_aware_reference_path_publisher", options)
{
    // Parameters
    this->declare_parameter<int>("horizon_length", 30);
    this->declare_parameter<double>("publish_rate", 20.0);
    this->declare_parameter<std::vector<std::string>>("robot_names", {"wheelbird1", "wheelbird2", "wheelbird3", "wheelbird4"});

    horizon_length_ = this->get_parameter("horizon_length").as_int();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    robot_names_ = this->get_parameter("robot_names").as_string_array();

    if (robot_names_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No robot names specified. Shutting down.");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Initializing reference path publisher for %zu robots.", robot_names_.size());

    for (const auto& robot_name : robot_names_) {
        path_received_flags_[robot_name] = false;
        ref_path_pubs_[robot_name] = this->create_publisher<nav_msgs::msg::Path>(
            "/" + robot_name + "/reference_path", 10);

        auto callback = [this, robot_name](const nav_msgs::msg::Path::SharedPtr msg) {
            this->globalPathCallback(msg, robot_name);
        };
        global_path_subs_[robot_name] = this->create_subscription<nav_msgs::msg::Path>(
            "/" + robot_name + "/global_path", rclcpp::SystemDefaultsQoS(), callback);
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /%s/global_path", robot_name.c_str());
    }
}

void TimeAwareReferencePathPublisher::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg, const std::string robot_name)
{
    if (path_received_flags_[robot_name]) {
        return;
    }
    if (msg->poses.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Received path for %s has less than 2 poses. Ignoring.", robot_name.c_str());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Received global_path for %s with %zu poses.", robot_name.c_str(), msg->poses.size());
    global_paths_[robot_name] = *msg;
    path_received_flags_[robot_name] = true;
    received_paths_count_++;
    checkAllPathsReceived();
}

void TimeAwareReferencePathPublisher::checkAllPathsReceived()
{
    if (received_paths_count_ == robot_names_.size()) {
        RCLCPP_INFO(this->get_logger(), "All global paths received. Starting synchronized reference path publication.");
        start_time_ = this->get_clock()->now();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&TimeAwareReferencePathPublisher::publishReferencePaths, this));
        global_path_subs_.clear();
    } else {
        RCLCPP_INFO(this->get_logger(), "Received %zu of %zu paths.", received_paths_count_, robot_names_.size());
    }
}

geometry_msgs::msg::PoseStamped TimeAwareReferencePathPublisher::interpolatePose(
    const nav_msgs::msg::Path& path, const rclcpp::Time& path_start_time, const rclcpp::Duration& target_elapsed_time)
{
    if (path.poses.size() < 2) {
        return path.poses.empty() ? geometry_msgs::msg::PoseStamped() : path.poses.back();
    }

    // Find the segment that brackets the target time
    size_t segment_idx = 0;
    for (size_t i = 0; i < path.poses.size() - 1; ++i) {
        rclcpp::Time t1 = path.poses[i].header.stamp;
        if ((t1 - path_start_time) <= target_elapsed_time) {
            segment_idx = i;
        } else {
            break;
        }
    }
    // Ensure we don't go past the second to last pose
    segment_idx = std::min(segment_idx, path.poses.size() - 2);

    const auto& p1 = path.poses[segment_idx];
    const auto& p2 = path.poses[segment_idx + 1];

    rclcpp::Time t1 = p1.header.stamp;
    rclcpp::Time t2 = p2.header.stamp;
    rclcpp::Duration segment_duration = t2 - t1;
    rclcpp::Duration time_into_segment = target_elapsed_time - (t1 - path_start_time);

    double ratio = 0.0;
    if (segment_duration.seconds() > 1e-9) {
        ratio = time_into_segment.seconds() / segment_duration.seconds();
    }
    ratio = std::max(0.0, std::min(1.0, ratio));

    geometry_msgs::msg::PoseStamped interpolated_pose;
    interpolated_pose.header.stamp = path_start_time + target_elapsed_time;
    interpolated_pose.header.frame_id = path.header.frame_id;

    // Position interpolation
    interpolated_pose.pose.position.x = p1.pose.position.x + ratio * (p2.pose.position.x - p1.pose.position.x);
    interpolated_pose.pose.position.y = p1.pose.position.y + ratio * (p2.pose.position.y - p1.pose.position.y);
    interpolated_pose.pose.position.z = p1.pose.position.z + ratio * (p2.pose.position.z - p1.pose.position.z);
    
    // Orientation interpolation (SLERP)
    tf2::Quaternion q1, q2;
    tf2::fromMsg(p1.pose.orientation, q1);
    tf2::fromMsg(p2.pose.orientation, q2);
    interpolated_pose.pose.orientation = tf2::toMsg(q1.slerp(q2, ratio));

    return interpolated_pose;
}

void TimeAwareReferencePathPublisher::publishReferencePaths()
{
    rclcpp::Time current_time = this->get_clock()->now();
    rclcpp::Duration elapsed_time = current_time - start_time_;

    for (const auto& robot_name : robot_names_) {
        const auto& global_path = global_paths_[robot_name];
        if (global_path.poses.empty()) continue;

        rclcpp::Time path_start_time = global_path.poses.front().header.stamp;

        nav_msgs::msg::Path ref_path;
        ref_path.header.stamp = current_time;
        ref_path.header.frame_id = global_path.header.frame_id;
        
        for (int i = 0; i < horizon_length_; ++i) {
            rclcpp::Duration future_offset = rclcpp::Duration::from_seconds(i / publish_rate_);
            rclcpp::Duration target_elapsed_time = elapsed_time + future_offset;
            
            geometry_msgs::msg::PoseStamped future_pose = interpolatePose(global_path, path_start_time, target_elapsed_time);
            ref_path.poses.push_back(future_pose);
        }

        ref_path_pubs_[robot_name]->publish(ref_path);
    }
}

} // namespace energy_planner