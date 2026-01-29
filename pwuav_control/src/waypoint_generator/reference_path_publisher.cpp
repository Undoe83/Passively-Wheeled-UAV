#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

class ReferencePathPublisher : public rclcpp::Node
{
public:
    ReferencePathPublisher() : Node("reference_path_publisher")
    {
        this->declare_parameter<int>("horizon_length", 30);
        this->declare_parameter<double>("publish_rate", 20);
        this->declare_parameter<double>("v_mean", 0.55);

        horizon_length_ = this->get_parameter("horizon_length").as_int();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        v_mean_ = this->get_parameter("v_mean").as_double();

        waypoints_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/waypoints", 10, std::bind(&ReferencePathPublisher::waypointsCallback, this, std::placeholders::_1));

        ref_pub_ = this->create_publisher<nav_msgs::msg::Path>("/reference_path", (int)publish_rate_);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&ReferencePathPublisher::publishRefPath, this));

        RCLCPP_INFO(this->get_logger(), "ReferencePathPublisher initialized (horizon=%d, rate=%.1fHz) [Odom-Free Mode]", horizon_length_, publish_rate_);
    }

private:
    void waypointsCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (is_global_received_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, 
                                "Ignoring new waypoints, processing the initial path.");
            return;
        }

        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty waypoints path.");
            have_waypoints_ = false;
            return;
        }
        
        waypoints_ = *msg;
        
        this->resampleGlobalPath();

        have_waypoints_ = true;
        start_idx_ = 0;
        is_global_received_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Received and resampled waypoints. Original: %zu poses, Sampled: %zu poses. Resetting index.",
                    waypoints_.poses.size(), sampled_waypoints_.poses.size());
    }

    void resampleGlobalPath()
    {
        if (waypoints_.poses.empty()) return;

        sampled_waypoints_.poses.clear();
        sampled_waypoints_.header = waypoints_.header;

        double prev_node_x = waypoints_.poses[0].pose.position.x;
        double prev_node_y = waypoints_.poses[0].pose.position.y;
        double prev_node_z = waypoints_.poses[0].pose.position.z;

        sampled_waypoints_.poses.push_back(waypoints_.poses[0]);

        for (size_t i = 1; i < waypoints_.poses.size(); ++i)
        {
            double cur_node_x = waypoints_.poses[i].pose.position.x;
            double cur_node_y = waypoints_.poses[i].pose.position.y;
            double cur_node_z = waypoints_.poses[i].pose.position.z;

            double node_distance = std::sqrt(std::pow(prev_node_x - cur_node_x, 2) + std::pow(prev_node_y - cur_node_y, 2) + std::pow(prev_node_z - cur_node_z, 2));

            double desired_dist_per_step = v_mean_ * (1.0 / publish_rate_);
            int sampling_node_num;

            if (desired_dist_per_step < 1e-6)
            {
                sampling_node_num = 1;
            }
            else{
                sampling_node_num = (int)std::ceil(node_distance / desired_dist_per_step);
            }

            if (sampling_node_num <= 0)
            {
                sampling_node_num = 1;
            }

            for (int j = 1; j <= sampling_node_num; ++j)
            {
                geometry_msgs::msg::PoseStamped sampling_node;
                sampling_node.pose.position.x = prev_node_x + ((cur_node_x - prev_node_x) / (double)sampling_node_num) * j;
                sampling_node.pose.position.y = prev_node_y + ((cur_node_y - prev_node_y) / (double)sampling_node_num) * j;
                sampling_node.pose.position.z = prev_node_z + ((cur_node_z - prev_node_z) / (double)sampling_node_num) * j;

                sampled_waypoints_.poses.emplace_back(sampling_node);
            }
            prev_node_x = cur_node_x;
            prev_node_y = cur_node_y;
            prev_node_z = cur_node_z;
        }

        if(sampled_waypoints_.poses.size() > 1)
        {
            double sam_prev_node_x = sampled_waypoints_.poses[0].pose.position.x;
            double sam_prev_node_y = sampled_waypoints_.poses[0].pose.position.y;

            for (size_t num = 1; num < sampled_waypoints_.poses.size(); ++num)
            {
                double ref_angle = atan2(sampled_waypoints_.poses[num].pose.position.y - sam_prev_node_y,
                                        sampled_waypoints_.poses[num].pose.position.x - sam_prev_node_x);
                
                while (ref_angle >= M_PI) ref_angle -= 2.0 * M_PI;
                while (ref_angle < -M_PI) ref_angle += 2.0 * M_PI;
                
                tf2::Quaternion q;
                q.setRPY(0, 0, ref_angle);

                sampled_waypoints_.poses[num - 1].pose.orientation.x = q.x();
                sampled_waypoints_.poses[num - 1].pose.orientation.y = q.y();
                sampled_waypoints_.poses[num - 1].pose.orientation.z = q.z();
                sampled_waypoints_.poses[num - 1].pose.orientation.w = q.w();

                sam_prev_node_x = sampled_waypoints_.poses[num].pose.position.x;
                sam_prev_node_y = sampled_waypoints_.poses[num].pose.position.y;
            }
            sampled_waypoints_.poses.back().pose.orientation = sampled_waypoints_.poses[sampled_waypoints_.poses.size() - 2].pose.orientation;
        }
    }

    void publishRefPath()
    {
        if (!have_waypoints_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for waypoints...");
            return;
        }

        nav_msgs::msg::Path ref;
        ref.header = sampled_waypoints_.header;
        ref.header.stamp = this->get_clock()->now();

        for (int i = 0; i < horizon_length_; ++i)
        {
            size_t current_idx = start_idx_ + i;

            if (current_idx >= sampled_waypoints_.poses.size())
            {
                if (sampled_waypoints_.poses.empty()) break; 
                current_idx = sampled_waypoints_.poses.size() - 1; 
            }
            ref.poses.push_back(sampled_waypoints_.poses[current_idx]);
        }
        
        ref_pub_->publish(ref);

        // [인덱스 증가] 다음 발행을 위해 start_idx_를 1 증가
        // 단, '샘플링된' 경로의 마지막 지점을 넘기지 않도록 함
        // start_idx_++;
        // if (start_idx_ == sampled_waypoints_.poses.size() - 1)
        // {
        //     start_idx_ = 0;
        // }
        
        if (start_idx_ < sampled_waypoints_.poses.size() - 1)
        {
            start_idx_++;
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path waypoints_;
    nav_msgs::msg::Path sampled_waypoints_;
    
    bool have_waypoints_ = false;
    int horizon_length_ = 30;
    size_t start_idx_ = 0;
    bool is_global_received_ = false;
    double publish_rate_;
    double v_mean_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReferencePathPublisher>());
    rclcpp::shutdown();
    return 0;
}