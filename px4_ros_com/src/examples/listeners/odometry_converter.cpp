#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_ros_com/frame_transforms.h>

#include <iostream>
#include <Eigen/Dense>

class OdometryConverter : public rclcpp::Node
{
public:
    OdometryConverter() : Node("odometry_converter")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&OdometryConverter::odometryCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/px4/odom", 10);
    }
private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
};

void OdometryConverter::odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
    auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();

    // Header
    odom_msg->header.stamp = this->get_clock()->now();
    odom_msg->header.frame_id = "world";
    odom_msg->child_frame_id = "base_link";

    // Position convert NED to ENU
    const Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);
    const Eigen::Vector3d pos_enu = px4_ros_com::frame_transforms::transform_static_frame(
        pos_ned, px4_ros_com::frame_transforms::StaticTF::NED_TO_ENU);
    
    odom_msg->pose.pose.position.x = pos_enu.x();
    odom_msg->pose.pose.position.y = pos_enu.y();
    odom_msg->pose.pose.position.z = pos_enu.z();

    // Orientation convert NED to ENU
    const Eigen::Quaterniond q_ned_frd(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    const Eigen::Quaterniond q_enu_flu = px4_ros_com::frame_transforms::px4_to_ros_orientation(q_ned_frd);
    // const Eigen::Quaterniond q_enu_flu = px4_ros_com::frame_transforms::transform_orientation(
    //     q_ned_frd, px4_ros_com::frame_transforms::StaticTF::NED_TO_ENU);
    
    odom_msg->pose.pose.orientation.x = q_enu_flu.x();
    odom_msg->pose.pose.orientation.y = q_enu_flu.y();
    odom_msg->pose.pose.orientation.z = q_enu_flu.z();
    odom_msg->pose.pose.orientation.w = q_enu_flu.w();

    //Linear velocity convert FRD to FLU
    const Eigen::Vector3d vel_frd(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    const Eigen::Vector3d vel_flu = px4_ros_com::frame_transforms::transform_static_frame(
        vel_frd, px4_ros_com::frame_transforms::StaticTF::AIRCRAFT_TO_BASELINK);

    odom_msg->twist.twist.linear.x = vel_flu.x();
    odom_msg->twist.twist.linear.y = vel_flu.y();
    odom_msg->twist.twist.linear.z = vel_flu.z();

    // Angular velocity convert FRD to FLU
    const Eigen::Vector3d ang_vel_frd(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]);
    const Eigen::Vector3d ang_vel_flu = px4_ros_com::frame_transforms::transform_static_frame(
        ang_vel_frd, px4_ros_com::frame_transforms::StaticTF::AIRCRAFT_TO_BASELINK);

    odom_msg->twist.twist.angular.x = ang_vel_flu.x();
    odom_msg->twist.twist.angular.y = ang_vel_flu.y();
    odom_msg->twist.twist.angular.z = ang_vel_flu.z();
    
    // Pose covariance
    px4_ros_com::frame_transforms::Covariance6d pose_cov_ned;
    std::copy(std::begin(msg->position_variance), std::end(msg->position_variance), pose_cov_ned.begin());
    px4_ros_com::frame_transforms::Covariance6d pose_cov_enu = px4_ros_com::frame_transforms::transform_static_frame(
        pose_cov_ned, px4_ros_com::frame_transforms::StaticTF::NED_TO_ENU);
    std::copy(pose_cov_enu.begin(), pose_cov_enu.end(), odom_msg->pose.covariance.begin());

    // Twist covariance
    px4_ros_com::frame_transforms::Covariance6d twist_cov_frd;
    std::copy(std::begin(msg->velocity_variance), std::end(msg->velocity_variance), twist_cov_frd.begin());
    px4_ros_com::frame_transforms::Covariance6d twist_cov_flu = px4_ros_com::frame_transforms::transform_static_frame(
        twist_cov_frd, px4_ros_com::frame_transforms::StaticTF::AIRCRAFT_TO_BASELINK);
    std::copy(twist_cov_flu.begin(), twist_cov_flu.end(), odom_msg->twist.covariance.begin());

    RCLCPP_INFO(this->get_logger(), "position (ENU): [%.2f, %.2f, %.2f]",
                odom_msg->pose.pose.position.x,
                odom_msg->pose.pose.position.y,
                odom_msg->pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "orientation (FLU): [%.2f, %.2f, %.2f, %.2f]",
                odom_msg->pose.pose.orientation.x,
                odom_msg->pose.pose.orientation.y,
                odom_msg->pose.pose.orientation.z,
                odom_msg->pose.pose.orientation.w);

    publisher_->publish(*odom_msg);
}

int main(int argc, char *argv[])
{
    std::cout << "Starting odometry republisher node..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryConverter>());
    rclcpp::shutdown();
    return 0;
}