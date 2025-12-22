#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>

#include <chrono>
#include <iostream>
#include <vector>

using namespace std::chrono_literals;

class TrajectoryOptimizer : public rclcpp::Node
{
public:
    TrajectoryOptimizer() : Node("trajectory_optimizer_node")
    {
        // ROS Parameters
        this->declare_parameter<double>("max_velocity", 1.0);
        this->declare_parameter<double>("max_acceleration", 0.5);
        this->declare_parameter<int>("derivative_order", 4); // 3 for min-jerk, 4 for min-snap
        this->declare_parameter<double>("visualization_width", 0.15);

        max_vel_ = this->get_parameter("max_velocity").as_double();
        max_acc_ = this->get_parameter("max_acceleration").as_double();
        dev_order_ = this->get_parameter("derivative_order").as_int();
        vis_traj_width_ = this->get_parameter("visualization_width").as_double();

        poly_order_ = 2 * dev_order_ - 1;
        poly_num1d_ = poly_order_ + 1;

        // ROS Subscribers and Publishers
        waypoint_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/waypoints", 10, std::bind(&TrajectoryOptimizer::waypoint_callback, this, std::placeholders::_1));

        path_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visual_path", 10);
        traj_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/visual_trajectory", 10);

        RCLCPP_INFO(this->get_logger(), "Trajectory Optimizer Node started.");
        RCLCPP_INFO(this->get_logger(), "Listening for waypoints on /waypoints topic...");
    }

private:
    // Member Variables
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_vis_pub_, traj_vis_pub_;
    
    double max_vel_, max_acc_, vis_traj_width_;
    int dev_order_, poly_order_, poly_num1d_;

    Eigen::MatrixXd poly_coeff_;
    Eigen::VectorXd poly_time_;

    // Main callback for receiving waypoints
    void waypoint_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Path must contain at least 2 waypoints.");
            return;
        }

        Eigen::MatrixXd waypoints(msg->poses.size(), 3);
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            waypoints(i, 0) = msg->poses[i].pose.position.x;
            waypoints(i, 1) = msg->poses[i].pose.position.y;
            waypoints(i, 2) = msg->poses[i].pose.position.z;
        }

        RCLCPP_INFO(this->get_logger(), "Received %zu waypoints. Generating trajectory...", msg->poses.size());
        auto time_start = this->get_clock()->now();

        generate_trajectory(waypoints);
        
        auto time_end = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Trajectory generated in %.3f ms.", (time_end - time_start).seconds() * 1000.0);
    }

    // Main logic for trajectory generation
    void generate_trajectory(const Eigen::MatrixXd& path)
    {
        // Boundary conditions (start and end at rest)
        Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 3);
        Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2, 3);

        // Allocate time for each segment
        poly_time_ = time_allocation(path);

        // Solve for polynomial coefficients
        poly_coeff_ = poly_qp_generation(dev_order_, path, vel, acc, poly_time_);

        // Visualize the results
        visualize_path(path);
        visualize_trajectory(poly_coeff_, poly_time_);
    }

    // --- Start of Solver Methods (ported from trajectory_generator_waypoint.cpp) ---
    Eigen::MatrixXd poly_qp_generation(const int d_order, const Eigen::MatrixXd &path, const Eigen::MatrixXd &vel, 
                                       const Eigen::MatrixXd &acc, const Eigen::VectorXd &time)
    {
        int p_order = 2 * d_order - 1;
        int p_num1d = p_order + 1;
        int seg_num = time.size();

        Eigen::MatrixXd poly_coeff = Eigen::MatrixXd::Zero(seg_num, 3 * p_num1d);
        
        Eigen::MatrixXd start_state(d_order, 3);
        Eigen::MatrixXd end_state(d_order, 3);
        start_state.row(0) = path.row(0);
        start_state.row(1) = vel.row(0);
        start_state.row(2) = acc.row(0);
        end_state.row(0) = path.row(path.rows() - 1);
        end_state.row(1) = vel.row(1);
        end_state.row(2) = acc.row(1);
        if (d_order == 4) { // Minimum Snap
            start_state.row(3) = Eigen::Vector3d::Zero();
            end_state.row(3) = Eigen::Vector3d::Zero();
        }

        Eigen::MatrixXd Q_all = Eigen::MatrixXd::Zero(p_num1d * seg_num, p_num1d * seg_num);
        Eigen::MatrixXd M_all = Eigen::MatrixXd::Zero(p_num1d * seg_num, p_num1d * seg_num);
        
        for (int i = 0; i < seg_num; i++) {
            Q_all.block(i * p_num1d, i * p_num1d, p_num1d, p_num1d) = get_q(p_num1d, d_order, time, i);
            M_all.block(i * p_num1d, i * p_num1d, p_num1d, p_num1d) = get_m(p_num1d, d_order, time, i);
        }

        Eigen::MatrixXd Ct = get_ct(seg_num, d_order);

        Eigen::VectorXd Px = closed_form_cal_coeff_1d(Q_all, M_all, Ct, path.col(0), start_state.col(0), end_state.col(0), seg_num, d_order);
        Eigen::VectorXd Py = closed_form_cal_coeff_1d(Q_all, M_all, Ct, path.col(1), start_state.col(1), end_state.col(1), seg_num, d_order);
        Eigen::VectorXd Pz = closed_form_cal_coeff_1d(Q_all, M_all, Ct, path.col(2), start_state.col(2), end_state.col(2), seg_num, d_order);

        for (int i = 0; i < seg_num; i++) {
            poly_coeff.row(i).segment(0, p_num1d) = Px.segment(p_num1d * i, p_num1d);
            poly_coeff.row(i).segment(p_num1d, p_num1d) = Py.segment(p_num1d * i, p_num1d);
            poly_coeff.row(i).segment(2 * p_num1d, p_num1d) = Pz.segment(p_num1d * i, p_num1d);
        }

        return poly_coeff;
    }

    Eigen::MatrixXd get_q(int p_num1d, int d_order, const Eigen::VectorXd &time, int seg_index) {
        Eigen::MatrixXd Q_k = Eigen::MatrixXd::Zero(p_num1d, p_num1d);
        for (int i = d_order; i < p_num1d; i++) {
            for (int j = d_order; j < p_num1d; j++) {
                Q_k(i, j) = (factorial(i) / factorial(i - d_order)) * (factorial(j) / factorial(j - d_order)) /
                            (i + j - 2 * d_order + 1) * pow(time(seg_index), (i + j - 2 * d_order + 1));
            }
        }
        return Q_k;
    }

    Eigen::MatrixXd get_m(int p_num1d, int d_order, const Eigen::VectorXd &time, int seg_index) {
        Eigen::MatrixXd M_k = Eigen::MatrixXd::Zero(p_num1d, p_num1d);
        for (int k = 0; k < d_order; k++) {
            for (int i = k; i < p_num1d; i++) {
                M_k(k, i) = factorial(i) / factorial(i - k); // M_k(0,i) = 1*t^0
            }
        }
        for (int k = d_order; k < p_num1d; k++) {
            int r = k - d_order;
            for (int i = r; i < p_num1d; i++) {
                M_k(k, i) = (factorial(i) / factorial(i - r)) * pow(time(seg_index), i - r);
            }
        }
        return M_k;
    }

    Eigen::MatrixXd get_ct(int seg_num, int d_order) {
        int d_num = 2 * d_order * seg_num;
        int df_and_dp_num = d_order * (seg_num + 1);
        int mid_waypts_num = seg_num - 1;
        int df_num = 2 * d_order + mid_waypts_num;

        Eigen::MatrixXd Ct = Eigen::MatrixXd::Zero(d_num, df_and_dp_num);
        Ct.block(0, 0, d_order, d_order) = Eigen::MatrixXd::Identity(d_order, d_order);
        Ct.block(d_num - d_order, df_num - d_order, d_order, d_order) = Eigen::MatrixXd::Identity(d_order, d_order);

        for (int i = 0; i < mid_waypts_num; i++) {
            // Pos constraints
            Ct(d_order + 2 * d_order * i, d_order + i) = 1;
            Ct(d_order * 3 + 2 * d_order * i, d_order + i) = 1;
            // Continuity constraints (vel, acc, jerk...)
            for (int j = 1; j < d_order; j++) {
                Ct(d_order + j + 2 * d_order * i, df_num + (d_order - 1) * i + j - 1) = 1;
                Ct(d_order * 3 - d_order + j + 2 * d_order * i, df_num + (d_order - 1) * i + j - 1) = 1;
            }
        }
        return Ct;
    }

    Eigen::VectorXd closed_form_cal_coeff_1d(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &M, const Eigen::MatrixXd &Ct,
                                             const Eigen::VectorXd &waypoints_1d, const Eigen::VectorXd &start_state_1d,
                                             const Eigen::VectorXd &end_state_1d, int seg_num, int d_order)
    {
        int mid_waypts_num = seg_num - 1;
        int df_num = 2 * d_order + mid_waypts_num;
        int dp_num = (d_order - 1) * mid_waypts_num;

        Eigen::MatrixXd C = Ct.transpose();
        Eigen::MatrixXd M_inv = M.inverse();
        Eigen::MatrixXd M_inv_tran = M_inv.transpose();

        Eigen::MatrixXd R = C * M_inv_tran * Q * M_inv * Ct;
        Eigen::MatrixXd R_pp = R.block(df_num, df_num, dp_num, dp_num);
        Eigen::MatrixXd R_fp = R.block(0, df_num, df_num, dp_num);

        Eigen::VectorXd dF(df_num);
        dF.head(d_order) = start_state_1d;
        dF.segment(d_order, mid_waypts_num) = waypoints_1d.segment(1, mid_waypts_num);
        dF.tail(d_order) = end_state_1d;

        Eigen::VectorXd dP = -R_pp.inverse() * R_fp.transpose() * dF;
        Eigen::VectorXd d_all(df_num + dp_num);
        d_all << dF, dP;

        return M_inv * Ct * d_all;
    }
    // --- End of Solver Methods ---


    // --- Helper and Visualization Methods ---
    Eigen::VectorXd time_allocation(const Eigen::MatrixXd& path)
    {
        Eigen::VectorXd time(path.rows() - 1);
        for (int i = 0; i < time.rows(); i++) {
            double distance = (path.row(i + 1) - path.row(i)).norm();
            double t = distance / max_vel_;
            time(i) = t;
        }
        return time;
    }

    void visualize_path(const Eigen::MatrixXd& path)
    {
        visualization_msgs::msg::Marker points, line_strip;
        points.header.frame_id = line_strip.header.frame_id = "world";
        points.header.stamp = line_strip.header.stamp = this->get_clock()->now();
        points.ns = line_strip.ns = "trajectory_optimizer";
        points.id = 0;
        line_strip.id = 1;
        points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        points.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
        
        points.scale.x = 0.2; points.scale.y = 0.2; points.scale.z = 0.2;
        points.color.a = 1.0; points.color.r = 1.0; points.color.g = 0.0; points.color.b = 0.0;

        line_strip.scale.x = 0.1;
        line_strip.color.a = 1.0; line_strip.color.r = 0.0; line_strip.color.g = 1.0; line_strip.color.b = 0.0;

        for (int i = 0; i < path.rows(); i++) {
            geometry_msgs::msg::Point p;
            p.x = path(i, 0); p.y = path(i, 1); p.z = path(i, 2);
            points.points.push_back(p);
            line_strip.points.push_back(p);
        }
        path_vis_pub_->publish(points);
        path_vis_pub_->publish(line_strip);
    }

    void visualize_trajectory(const Eigen::MatrixXd& poly_coeff, const Eigen::VectorXd& time)
    {
        visualization_msgs::msg::Marker traj_vis;
        traj_vis.header.stamp = this->get_clock()->now();
        traj_vis.header.frame_id = "world";
        traj_vis.ns = "trajectory_optimizer";
        traj_vis.id = 2;
        traj_vis.type = visualization_msgs::msg::Marker::LINE_STRIP;
        traj_vis.action = visualization_msgs::msg::Marker::ADD;
        traj_vis.scale.x = vis_traj_width_;
        traj_vis.color.a = 1.0; traj_vis.color.r = 0.0; traj_vis.color.g = 0.0; traj_vis.color.b = 1.0;

        for (int i = 0; i < time.size(); i++) {
            for (double t = 0.0; t < time(i); t += 0.01) {
                Eigen::Vector3d pos = get_pos_poly(poly_coeff, i, t);
                geometry_msgs::msg::Point p;
                p.x = pos(0); p.y = pos(1); p.z = pos(2);
                traj_vis.points.push_back(p);
            }
        }
        traj_vis_pub_->publish(traj_vis);
    }

    Eigen::Vector3d get_pos_poly(const Eigen::MatrixXd& poly_coeff, int k, double t)
    {
        Eigen::Vector3d ret;
        for (int dim = 0; dim < 3; dim++) {
            Eigen::VectorXd coeff = (poly_coeff.row(k)).segment(dim * poly_num1d_, poly_num1d_);
            Eigen::VectorXd time_vec = Eigen::VectorXd::Zero(poly_num1d_);
            for (int j = 0; j < poly_num1d_; j++) {
                time_vec(j) = pow(t, j);
            }
            ret(dim) = coeff.dot(time_vec);
        }
        return ret;
    }

    int factorial(int x)
    {
        int fac = 1;
        for (int i = x; i > 0; i--)
            fac = fac * i;
        return fac;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryOptimizer>());
    rclcpp::shutdown();
    return 0;
}