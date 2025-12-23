#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sample_waypoints.h" // px4_ros_com/src/waypoint_generator/sample_waypoints.h

#include <string>
#include <vector>
#include <chrono>

class WaypointPublisher : public rclcpp::Node
{
public:
    WaypointPublisher(const std::string& type) : Node("waypoint_generator")
    {
        // Declare and get the waypoint type parameter
        this->declare_parameter<std::string>("waypoint_type", type);
        this->get_parameter("waypoint_type", waypoint_type_);

        // Create a publisher for the waypoints
        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/waypoints", 10);

        // Initialize waypoints based on the selected type
        initialize_waypoints();

        // Set up a timer to publish waypoints at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&WaypointPublisher::publish_waypoints, this));
    }
private:
    void initialize_waypoints()
    {   
        RCLCPP_INFO(this->get_logger(), "Generating waypoints of type: %s", waypoint_type_.c_str());

        if (waypoint_type_ == "point")
        {
            waypoints_msg_ = sample_waypoints::point();
        }
        else if (waypoint_type_ == "circle")
        {
            waypoints_msg_ = sample_waypoints::circle();
        }
        else if (waypoint_type_ == "eight")
        {
            waypoints_msg_ = sample_waypoints::eight();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown waypoint type '%s', defaulting to 'circle'", waypoint_type_.c_str());
            waypoints_msg_ = sample_waypoints::circle();
        }

        waypoints_msg_.header.frame_id = "world";
    }

    void publish_waypoints()
    {
        waypoints_msg_.header.stamp = this->get_clock()->now();
        publisher_->publish(waypoints_msg_);
        RCLCPP_INFO(this->get_logger(), "Published %zu waypoints", waypoints_msg_.poses.size());
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string waypoint_type_;
    nav_msgs::msg::Path waypoints_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string type = "eight";
    if (argc > 1) {
        type = std::string(argv[1]);
    }

    rclcpp::spin(std::make_shared<WaypointPublisher>(type));
    rclcpp::shutdown();
    return 0;
}