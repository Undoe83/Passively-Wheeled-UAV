#include <rclcpp/rclcpp.hpp>
#include <casadi/casadi.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <std_msgs/msg/float32.hpp>
#include <Eigen/Dense>
#include <numeric>
#include <algorithm>
#include <cmath>

using namespace casadi;

class Mpc : public rclcpp::Node {
public:
    Mpc();
private:
    enum class Mode {UNKNOWN, AERIAL, TERRESTRIAL};
    Mode current_mode_ = Mode::UNKNOWN;
    double z_avg_threshold_ = 0.1; // reference의 z 좌표 평균을 이용 (NED)

    // --- MPC 파라미터 ---
    int N_ = 29;        // 예측 호라이즌 (만약 reference 지점 30개 -> N+1=30)
    double dt_ = 1.0 / 20.0;   // 샘플링 타임 (1.0 / rate of reference path)
    int NX_, NU_;    // 상태, 제어 입력 개수

    // --- CasADi 객체 ---
    Opti opti_;
    MX X_; // 상태 변수 궤적
    MX U_; // 제어 입력 궤적
    MX x0_; // 파라미터: 현재 상태
    MX X_ref_; // 파라미터: 기준 궤적

    // --- ROS 2 ---
    mavros_msgs::msg::State current_state_mavros_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr att_setpoint_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_path_pub_;
    
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Time last_request_time_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    // --- 상태 및 궤적 변수 (모두 NED 프레임 기준) ---
    DM current_state_;      // [x, y, z, yaw] (NED)
    DM X_ref_val_;          // 기준 궤적 (NED, 4x20)
    DM X_ref_unwrapped_;    // Yaw Unwrapping된 기준 궤적 (NED, 4x20)

    bool odom_received_ = false;
    bool path_received_ = false;

    // 지상 모드 피드백 제어 변수
    DM current_velocity_;             // 3x1 [vx_ned, vy_ned, vz_ned]
    double vx_body_current_ = 0.0;    // body frame vx 
    double yaw_rate_current_ = 0.0;
    double pitch_current_ = 0.0;

    const double DELTA_T_ = 0.1;         // 요 오차 보정 시간 (s)
    const double THRUST_HOVER_ = 0.25;        // 호버링 추력 (정규화, 튜닝 필요)
    const double THRUST_MIN_ = 0.1;
    const double M_ = 1.126;                // 질량 (kg)
    const double KF_THRUST_SCALE_ = 24; // 추력 정규화 계수 (튜닝 필요) 
    const double POS_ERR_THRESHOLD_NORM_ = 0.4;  // position error threshold
    const double PITCH_MAX = M_PI / 12.0;
    const double PITCH_MIN = -M_PI / 12.0;
    // 전방 가속도 제어 게인 (튜닝 필요)
    const double Kp_P_ = 0.06;
    const double Kp_V_ = 0.07;
    const double Ki_V_ = 0.01;
    const double Kd_V_ = 0.5;

    const double VEL_INTEGRAL_MIN_ = -0.5;
    const double VEL_INTEGRAL_MAX_ = 0.5;

    double vel_integral_ = 0.0;
    double prev_vel_err_ = 0.0;

    // --- ROS 콜백 함수 ---
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    
    // 메인 MPC 제어 루프 (타이머 기반)
    void control_loop_callback();

    // --- 설정 및 유틸리티 함수 ---
    void offboard_mode();
    void setup_mpc();

    void get_rpy_from_ros_quaternion(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw);
    double unwrap_angle_diff(double ref_angle, double last_angle);
};


Mpc::Mpc() : Node("mpc_controller_node") {
    // CasADi 변수 초기화
    current_state_ = DM::zeros(4, 1);
    X_ref_val_ = DM::zeros(4, N_ + 1);
    X_ref_unwrapped_ = DM::zeros(4, N_ + 1);
    current_velocity_ = DM::zeros(3, 1);

    // ROS 2 publisher, subscriber, timer 초기화
    
    // QoS 설정 (PX4 uORB 메시지용)
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10, 
        std::bind(&Mpc::state_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mavros/local_position/odom", qos,
        std::bind(&Mpc::odom_callback, this, std::placeholders::_1));

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/reference_path", 10, // 일반적인 ROS 토픽 QoS
        std::bind(&Mpc::path_callback, this, std::placeholders::_1));

    setpoint_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10);

    att_setpoint_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
        "/mavros/setpoint_raw/attitude", 10);

    optimal_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "optimal_path", 10);

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    last_request_time_ = this->get_clock()->now();

    // 제어 루프 타이머 (PX4 Offboard mode 최소 요구 속도(2Hz)보다 빠름)
    // MPC 계산 시간(dt_)과 반드시 같을 필요는 없음
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), // 50Hz
        std::bind(&Mpc::control_loop_callback, this));

    RCLCPP_INFO(this->get_logger(), "Bimodal (Aerial/Terrestrial) MPC Controller Node initialized.");
}

void Mpc::setup_mpc() {
    NX_ = 4; // [x, y, z, yaw]
    NU_ = 4; // [vx, vy, vz, yaw_rate]

    opti_ = Opti();

    // 1. 최적화 변수 (States and Controls)
    X_ = opti_.variable(NX_, N_ + 1);
    U_ = opti_.variable(NU_, N_);

    // 2. 파라미터 (Current State and Reference Trajectory)
    x0_ = opti_.parameter(NX_, 1);
    X_ref_ = opti_.parameter(NX_, N_ + 1);
    // U_ref는 0으로 가정 (속도 명령 자체를 최소화)

    // 3. 비용 함수 (Cost Function)
    MX cost = 0;
    // 가중치 행렬 (NED 기준)
    DM Q = DM::diag({2.0, 2.0, 2.0, 2.0});  // [x, y, z, yaw] 오차
    DM R = DM::diag({0.8, 0.8, 0.8, 0.8});  // [vx, vy, vz, yaw_rate] 사용량
    DM R_Delta = DM::diag({0.1, 0.1, 0.1, 0.1}); // smoothness 가중치
    DM P = Q;  // 터미널 가중치

    Slice all;
    for (int k = 0; k < N_; ++k) {
        MX e_x = X_(all, k) - X_ref_(all, k);
        MX e_u = U_(all, k);

        cost += mtimes(e_x.T(), mtimes(Q, e_x)); // tracking cost
        cost += mtimes(e_u.T(), mtimes(R, e_u)); // control effort
        if (k < N_ - 1)                           // control smoothness
        {
            MX delta_u = U_(all, k + 1) - U_(all, k);
            cost += mtimes(delta_u.T(), mtimes(R_Delta, delta_u));
        }
    }
    MX e_N = X_(all, N_) - X_ref_(all, N_);
    cost += mtimes(e_N.T(), mtimes(P, e_N)); // terminal cost

    opti_.minimize(cost);
    
    // 4. 제약 조건 (Constraints)
    
    // 지상모드 제약
    DM u_min, u_max;
    if (current_mode_ == Mode::TERRESTRIAL) {
        u_min = DM::vertcat({-1.0, -1.0, -0.0, -0.5}); // vx, vy, vz, yaw_rate
        u_max = DM::vertcat({ 1.0,  1.0,  0.0,  0.5}); // vx, vy, vz, yaw_rate
    } else {
        // 동역학 제약 (x_{k+1} = x_k + dt * u_k) + 제어 입력 제약
        u_min = DM::vertcat({-1.0, -1.0, -1.0, -0.5}); // vx, vy, vz, yaw_rate
        u_max = DM::vertcat({ 1.0,  1.0,  1.0,  0.5}); // vx, vy, vz, yaw_rate
    }
    for (int k = 0; k < N_; ++k) {
        opti_.subject_to(X_(all, k + 1) == X_(all, k) + dt_ * U_(all, k));
        opti_.subject_to(U_(all, k) >= u_min);
        opti_.subject_to(U_(all, k) <= u_max);
    }

    // 초기 상태 제약
    opti_.subject_to(X_(all, 0) == x0_);

    // 5. 솔버 설정
    Dict solver_opts;
    solver_opts["print_time"] = false;
    solver_opts["ipopt.print_level"] = 2;
    opti_.solver("ipopt", solver_opts);

    // RCLCPP_INFO(this->get_logger(), "MPC (4-State, 4-Control).");
}

void Mpc::state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_mavros_ = *msg;
}

void Mpc::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_state_(0) = msg->pose.pose.position.x; // x
    current_state_(1) = msg->pose.pose.position.y; // y
    current_state_(2) = msg->pose.pose.position.z; // z

    double roll, pitch, yaw;
    get_rpy_from_ros_quaternion(msg->pose.pose.orientation, roll, pitch, yaw);
    current_state_(3) = yaw;
    pitch_current_ = pitch;

    current_velocity_(0) = msg->twist.twist.linear.x; // vx
    current_velocity_(1) = msg->twist.twist.linear.y; // vy
    current_velocity_(2) = msg->twist.twist.linear.z; // vz
    
    // body frame 기준 vx 계산 (지상 모드용)
    double c_y = cos(yaw);
    double s_y = sin(yaw);
    vx_body_current_ = (double)current_velocity_(0) * c_y + (double)current_velocity_(1) * s_y;

    yaw_rate_current_ = msg->twist.twist.angular.z;

    odom_received_ = true;
}

void Mpc::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.size() != static_cast<size_t>(N_ + 1)) {
        RCLCPP_WARN(this->get_logger(), "Received path size (%zu) != N+1 (%d). Discarding.",
                    msg->poses.size(), N_ + 1);
        return;
    }

    double z_sum = 0.0;
    for (int k = 0; k < (N_ + 1); ++k) {
        const auto& pose = msg->poses[k].pose;

        X_ref_val_(0, k) = pose.position.x;
        X_ref_val_(1, k) = pose.position.y;
        X_ref_val_(2, k) = pose.position.z;

        double roll, pitch, yaw;
        get_rpy_from_ros_quaternion(pose.orientation, roll, pitch, yaw);

        // Yaw 값을 -pi ~ pi 범위로 정규화 (Unwrapping 전 단계)
        X_ref_val_(3, k) = atan2(sin(yaw), cos(yaw));

        z_sum += (pose.position.z);
    }
    double z_avg = z_sum / msg->poses.size();

    Mode new_mode = (z_avg < z_avg_threshold_) ? Mode::TERRESTRIAL : Mode::AERIAL;

    if (new_mode != current_mode_ || current_mode_ == Mode::UNKNOWN) {
        RCLCPP_INFO(this->get_logger(), "Mode change detected: %s (z_avg: %.2f). Re-configuring MPC.",
                    (new_mode == Mode::AERIAL ? "AERIAL" : "TERRESTRIAL"), z_avg);
        current_mode_ = new_mode;
    }

    setup_mpc();

    path_received_ = true;
}

void Mpc::control_loop_callback() {
    if (!odom_received_ || !path_received_ || current_mode_ == Mode::UNKNOWN) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "Waiting for odom (%d), path (%d), or mode decision (mode: %d)...",
                             odom_received_, path_received_, (int)current_mode_);
        vel_integral_ = 0.0;
        prev_vel_err_ = 0.0;

        return;
    }

    offboard_mode();
    uint64_t timestamp_now = this->get_clock()->now().nanoseconds() / 1000;

    try {
        X_ref_unwrapped_ = X_ref_val_; // x, y, z 값 복사
        double current_yaw = (double)current_state_(3);
        
        // k=0 (첫 번째 기준점)을 현재 Yaw 기준으로 Unwrap
        double ref_yaw_k0 = (double)X_ref_val_(3, 0);
        double unwrapped_yaw_k0 = current_yaw + unwrap_angle_diff(ref_yaw_k0, current_yaw);
        X_ref_unwrapped_(3, 0) = unwrapped_yaw_k0;

        // k=1부터 N까지, 이전 Unwrapped Yaw 기준으로 다음 Yaw를 순차적으로 Unwrap
        for (int k = 1; k < (N_ + 1); ++k) {
            double ref_yaw_k = (double)X_ref_val_(3, k);
            double prev_unwrapped_yaw = (double)X_ref_unwrapped_(3, k - 1);
            X_ref_unwrapped_(3, k) = prev_unwrapped_yaw + unwrap_angle_diff(ref_yaw_k, prev_unwrapped_yaw);
        }

        opti_.set_value(x0_, current_state_);      // 4D [x, y, z, yaw]
        opti_.set_value(X_ref_, X_ref_unwrapped_); // 4D [x, y, z, yaw]

        OptiSol sol = opti_.solve();

        DM X_optimal = sol.value(X_);

        // --- visualization optimal path ---
        nav_msgs::msg::Path optimal_path_msg;
        optimal_path_msg.header.stamp = this->get_clock()->now();
        optimal_path_msg.header.frame_id = "map";

        for (int k = 0; k < (N_ + 1); ++k) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = (double)X_optimal(0, k);
            pose_stamped.pose.position.y = (double)X_optimal(1, k);
            pose_stamped.pose.position.z = (double)X_optimal(2, k);
            
            tf2::Quaternion q;
            q.setRPY(0, 0, (double)X_optimal(3, k));
            pose_stamped.pose.orientation = tf2::toMsg(q);

            optimal_path_msg.poses.push_back(pose_stamped);
        }
        optimal_path_pub_->publish(optimal_path_msg);
        // -------------------------------

        if (current_mode_ == Mode::AERIAL) {
            DM u_optimal = sol.value(U_(Slice(), 0));  // [vx, vy, vz, yaw_rate]
            geometry_msgs::msg::TwistStamped vel_msg;
            vel_msg.header.stamp = this->get_clock()->now();
            vel_msg.header.frame_id = "base_link"; // 또는 "map"에 맞춰 성분 변환
            vel_msg.twist.linear.x = (double)u_optimal(0);
            vel_msg.twist.linear.y = (double)u_optimal(1);
            vel_msg.twist.linear.z = (double)u_optimal(2);
            vel_msg.twist.angular.z = (double)u_optimal(3);
            setpoint_pub_->publish(vel_msg);
        } else {  // Mode::TERRESTRIAL
            DM x_ref_k1 = sol.value(X_(Slice(), 19)); // MPC가 계획한 17 스텝 상태 [x, y, z, yaw]
            DM u_ref_k1 = sol.value(U_(Slice(), 1)); // MPC가 계획한 1스텝 제어입력 [vx, vy, vz, yaw_rate]

            double yaw_ref_k1 = (double)x_ref_k1(3);
            double x_err = (double)x_ref_k1(0) - (double)current_state_(0);
            double y_err = (double)x_ref_k1(1) - (double)current_state_(1);
            // RCLCPP_INFO(this->get_logger(), "1step - x : %f, y : %f", (double)x_ref_k1(0), (double)x_ref_k1(1));
            double yaw_err = unwrap_angle_diff(yaw_ref_k1, (double)current_state_(3));
            double pos_err_norm = std::sqrt(std::pow(x_err, 2) + std::pow(y_err, 2) + std::pow(yaw_err, 2)); // L2(euclidian) norm
            RCLCPP_INFO(this->get_logger(), "pos error norm: %f", pos_err_norm);
           
            // yaw control
            double yaw_desired_I;
            // RCLCPP_INFO(this->get_logger(), "pos error norm : %f", pos_err_norm);
            if (pos_err_norm <= POS_ERR_THRESHOLD_NORM_) {
                yaw_desired_I = yaw_ref_k1;
                RCLCPP_INFO(this->get_logger(), "yaw_ref : %f", yaw_desired_I);
            } else {
                yaw_desired_I = atan2(y_err, x_err);
                RCLCPP_INFO(this->get_logger(), "atan2 : %f", yaw_desired_I);
            }
            // RCLCPP_INFO(this->get_logger(), "current_yaw : %f", (double)current_state_(3));
            // RCLCPP_INFO(this->get_logger(), "target yaw : %f", yaw_desired_I);

            // adaptive thrust control
            double delta_psi = unwrap_angle_diff(yaw_desired_I, yaw_ref_k1); // -pi ~ pi

            double yaw_accel_max = (2.0 * (delta_psi - (double)u_ref_k1(3) * DELTA_T_)) / (DELTA_T_ * DELTA_T_);
            yaw_accel_max = std::abs(yaw_accel_max);

            // RCLCPP_INFO(this->get_logger(), "yaw accel max: %f", yaw_accel_max);
            double thrust_cmd = 0.027 * yaw_accel_max + 0.0708;
            thrust_cmd = std::clamp(thrust_cmd, THRUST_MIN_, THRUST_HOVER_);

            // attitude
            double vx_ref = (double)u_ref_k1(0);
            double vy_ref = (double)u_ref_k1(1);
            double c_y = cos(current_yaw);
            double s_y = sin(current_yaw);
            double vx_ref_body = vx_ref * c_y + vy_ref * s_y;
            double v_err = vx_ref_body - vx_body_current_;

            // P-V 피드백 제어
            vel_integral_ += v_err * DELTA_T_;
            vel_integral_ = std::clamp(vel_integral_, VEL_INTEGRAL_MIN_, VEL_INTEGRAL_MAX_);
            double vel_derivate = (v_err - prev_vel_err_) / DELTA_T_;
            double acc_desired_x = Kp_V_ * (v_err + Kp_P_ * pos_err_norm) + Ki_V_ * vel_integral_ + Kd_V_ * vel_derivate;
            prev_vel_err_ = v_err;

            // 목표 피치 각도 계산 
            double F_newtons = KF_THRUST_SCALE_ * thrust_cmd; // 정규화된 추력을 뉴턴(N)으로 변환
            
            // sin(theta) = (M * a_x) / F_total
            double sin_pitch = (M_ * acc_desired_x) / F_newtons;
            sin_pitch = std::clamp(sin_pitch, -1.0, 1.0); // asin 입력 범위 제한
            double pitch_desired_I = asin(sin_pitch); // nose down -> positive(ENU)
            
            double pitch_enu = std::clamp(pitch_desired_I, PITCH_MIN, PITCH_MAX);
            double yaw_enu = yaw_desired_I;
            yaw_enu = atan2(sin(yaw_enu), cos(yaw_enu));

            RCLCPP_INFO(this->get_logger(), "| Thrust: %f | Pitch: %f | Yaw: %f |", thrust_cmd, pitch_enu, yaw_enu);

            mavros_msgs::msg::AttitudeTarget att_msg;
            att_msg.header.stamp = this->get_clock()->now();
            att_msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                                mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                                mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
            
            tf2::Quaternion q;
            q.setRPY(0.0, pitch_enu, yaw_enu);
            att_msg.orientation = tf2::toMsg(q);
            att_msg.thrust = thrust_cmd; 

            att_setpoint_pub_->publish(att_msg);

            // RCLCPP_INFO(this->get_logger(), "command | thrust: %f", thrust_cmd);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "MPC optimization failed: %s", e.what());
        return;
    }
}

void Mpc::offboard_mode() {
    auto now = this->get_clock()->now();

    if (now - last_request_time_ < rclcpp::Duration::from_seconds(5.0)) {
        return;
    }

    if (current_state_mavros_.mode != "OFFBOARD") {
        auto set_mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        set_mode_req->custom_mode = "OFFBOARD";
        
        RCLCPP_INFO(this->get_logger(), "Requesting Offboard mode...");
        set_mode_client_->async_send_request(set_mode_req);
        last_request_time_ = now;
    } 
    // 2. Offboard 모드이지만 시동이 안 걸렸을 경우 시동 요청
    else if (!current_state_mavros_.armed) {
        auto arming_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arming_req->value = true;
        
        RCLCPP_INFO(this->get_logger(), "Requesting Arming...");
        arming_client_->async_send_request(arming_req);
        last_request_time_ = now;
    }
}

void Mpc::get_rpy_from_ros_quaternion(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
}

double Mpc::unwrap_angle_diff(double ref_angle, double last_angle) {
    double diff = ref_angle - last_angle;
    // atan2(sin(diff), cos(diff))는 diff를 -pi ~ pi 범위로 정규화함
    return atan2(sin(diff), cos(diff));
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mpc>());
    rclcpp::shutdown();
    return 0;
}