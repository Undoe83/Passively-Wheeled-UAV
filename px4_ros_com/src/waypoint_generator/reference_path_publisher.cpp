#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm> // std::min 사용

using namespace std::chrono_literals;

/**
 * @class ReferencePathPublisher
 * @brief /waypoints (전체 경로)를 구독하고,
 * 이를 v_mean과 publish_rate에 맞춰 고해상도로 리샘플링합니다.
 * 이후 타이머 주기에 맞춰 horizon_length (예: 20개) 만큼의
 * 지역 경로(/reference_path)를 한 스텝씩 이동하며 발행합니다.
 */
class ReferencePathPublisher : public rclcpp::Node
{
public:
    ReferencePathPublisher() : Node("reference_path_publisher")
    {
        // 1. 파라미터 선언
        this->declare_parameter<int>("horizon_length", 30);
        this->declare_parameter<double>("publish_rate", 20);
        this->declare_parameter<double>("v_mean", 0.25);

        horizon_length_ = this->get_parameter("horizon_length").as_int();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        v_mean_ = this->get_parameter("v_mean").as_double();

        // 2. 서브스크라이버 (전체 경로)
        waypoints_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/waypoints", 10, std::bind(&ReferencePathPublisher::waypointsCallback, this, std::placeholders::_1));

        // 3. 퍼블리셔 (MPC용 20개짜리 경로)
        ref_pub_ = this->create_publisher<nav_msgs::msg::Path>("/reference_path", (int)publish_rate_);

        // 4. 타이머 (발행 주기)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&ReferencePathPublisher::publishRefPath, this));

        RCLCPP_INFO(this->get_logger(), "ReferencePathPublisher initialized (horizon=%d, rate=%.1fHz) [Odom-Free Mode]", horizon_length_, publish_rate_);
    }

private:
    /**
     * @brief /waypoints (전체 경로) 콜백
     * 새 경로를 받으면 저장하고, 즉시 전체 경로를 리샘플링합니다.
     */
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
        
        // 원본 경로 저장
        waypoints_ = *msg;
        
        // MODIFIED: 경로를 받자마자 전체 경로를 리샘플링
        this->resampleGlobalPath();

        have_waypoints_ = true;
        start_idx_ = 0;
        is_global_received_ = true;
        
        // MODIFIED: 원본 크기와 샘플링된 크기를 모두 로깅
        RCLCPP_INFO(this->get_logger(), "Received and resampled waypoints. Original: %zu poses, Sampled: %zu poses. Resetting index.",
                    waypoints_.poses.size(), sampled_waypoints_.poses.size());
    }

    // (odomCallback 제거)


    /**
     * @brief [NEW FUNCTION]
     * waypoints_ (원본 글로벌 경로)를 기반으로
     * v_mean과 publish_rate에 맞춰 고해상도 경로를 생성하고
     * sampled_waypoints_에 저장합니다.
     * (multi_ref_path_creater.cpp의 make_formation_ref_trajectory 로직)
     */
    void resampleGlobalPath()
    {
        if (waypoints_.poses.empty()) return;

        // 샘플링된 경로 초기화
        sampled_waypoints_.poses.clear();
        sampled_waypoints_.header = waypoints_.header;

        // 시작점 설정
        double prev_node_x = waypoints_.poses[0].pose.position.x;
        double prev_node_y = waypoints_.poses[0].pose.position.y;
        double prev_node_z = waypoints_.poses[0].pose.position.z;

        // 첫 번째 포인트는 항상 추가
        sampled_waypoints_.poses.push_back(waypoints_.poses[0]);

        // 원본 글로벌 경로의 1번 인덱스부터 순회
        for (size_t i = 1; i < waypoints_.poses.size(); ++i)
        {
            double cur_node_x = waypoints_.poses[i].pose.position.x;
            double cur_node_y = waypoints_.poses[i].pose.position.y;
            double cur_node_z = waypoints_.poses[i].pose.position.z;

            double node_distance = std::sqrt(std::pow(prev_node_x - cur_node_x, 2) + std::pow(prev_node_y - cur_node_y, 2) + std::pow(prev_node_z - cur_node_z, 2));

            // 한 스텝당 원하는 이동 거리
            double desired_dist_per_step = v_mean_ * (1.0 / publish_rate_);
            int sampling_node_num;

            if (desired_dist_per_step < 1e-6)
            {
                sampling_node_num = 1; // 0으로 나누기 방지
            }
            else{
                // multi_ref_path_creater와 동일한 샘플링 계산
                sampling_node_num = (int)std::ceil(node_distance / desired_dist_per_step);
            }

            if (sampling_node_num <= 0)
            {
                sampling_node_num = 1;
            }

            // 계산된 수만큼 점 보간
            for (int j = 1; j <= sampling_node_num; ++j)
            {
                geometry_msgs::msg::PoseStamped sampling_node;
                sampling_node.pose.position.x = prev_node_x + ((cur_node_x - prev_node_x) / (double)sampling_node_num) * j;
                sampling_node.pose.position.y = prev_node_y + ((cur_node_y - prev_node_y) / (double)sampling_node_num) * j;
                sampling_node.pose.position.z = prev_node_z + ((cur_node_z - prev_node_z) / (double)sampling_node_num) * j;

                sampled_waypoints_.poses.emplace_back(sampling_node);
            }
            // 이전 노드 위치 업데이트
            prev_node_x = cur_node_x;
            prev_node_y = cur_node_y;
            prev_node_z = cur_node_z;
        }

        // 샘플링된 전체 경로에 대해 Orientation (Yaw) 계산
        if(sampled_waypoints_.poses.size() > 1)
        {
            double sam_prev_node_x = sampled_waypoints_.poses[0].pose.position.x;
            double sam_prev_node_y = sampled_waypoints_.poses[0].pose.position.y;

            for (size_t num = 1; num < sampled_waypoints_.poses.size(); ++num)
            {
                double ref_angle = atan2(sampled_waypoints_.poses[num].pose.position.y - sam_prev_node_y,
                                        sampled_waypoints_.poses[num].pose.position.x - sam_prev_node_x);
                
                // pi_to_pi
                while (ref_angle >= M_PI) ref_angle -= 2.0 * M_PI;
                while (ref_angle < -M_PI) ref_angle += 2.0 * M_PI;
                
                tf2::Quaternion q;
                q.setRPY(0, 0, ref_angle);

                // [num - 1] (이전 노드)에 방향 설정
                sampled_waypoints_.poses[num - 1].pose.orientation.x = q.x();
                sampled_waypoints_.poses[num - 1].pose.orientation.y = q.y();
                sampled_waypoints_.poses[num - 1].pose.orientation.z = q.z();
                sampled_waypoints_.poses[num - 1].pose.orientation.w = q.w();

                sam_prev_node_x = sampled_waypoints_.poses[num].pose.position.x;
                sam_prev_node_y = sampled_waypoints_.poses[num].pose.position.y;
            }
            // 마지막 노드는 그 직전 노드의 방향을 따라감
            sampled_waypoints_.poses.back().pose.orientation = sampled_waypoints_.poses[sampled_waypoints_.poses.size() - 2].pose.orientation;
        }
    }


    /**
     * @brief 타이머 콜백. 
     * [MODIFIED] 미리 샘플링된 'sampled_waypoints_'에서 20개짜리 기준 경로를 '추출'하여 발행합니다.
     */
    void publishRefPath()
    {
        if (!have_waypoints_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for waypoints...");
            return; // 발행할 경로가 없음 (아직 수신 및 샘플링 안 됨)
        }

        nav_msgs::msg::Path ref;
        ref.header = sampled_waypoints_.header; // 원본 경로의 frame_id 사용
        ref.header.stamp = this->get_clock()->now();

        // [MODIFIED LOGIC]
        // start_idx_부터 horizon_length_ (20개) 만큼 '미리 샘플링된' 경로에서 추출
        for (int i = 0; i < horizon_length_; ++i)
        {
            size_t current_idx = start_idx_ + i;

            // [경로 끝 처리] 경로의 끝을 넘어가면 마지막 지점을 반복 (Padding)
            if (current_idx >= sampled_waypoints_.poses.size())
            {
                // 경로가 비어있는 극단적인 경우 방지
                if (sampled_waypoints_.poses.empty()) break; 
                current_idx = sampled_waypoints_.poses.size() - 1; 
            }
            ref.poses.push_back(sampled_waypoints_.poses[current_idx]);
        }
        
        // 20개짜리 기준 경로 발행
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

    // ROS 2 인터페이스
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 데이터 저장 변수
    nav_msgs::msg::Path waypoints_; // 원본 전체 경로 (ENU)
    nav_msgs::msg::Path sampled_waypoints_; // [NEW] 샘플링된 고해상도 전체 경로
    
    bool have_waypoints_ = false;
    int horizon_length_ = 30;
    size_t start_idx_ = 0; // 현재 발행 중인 경로의 시작 인덱스
    bool is_global_received_ = false; // 전체 경로를 한번만 받음
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