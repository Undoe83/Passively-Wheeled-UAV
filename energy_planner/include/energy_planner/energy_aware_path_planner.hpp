#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>

#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <optional>
#include <array>
#include <map>

namespace energy_planner 
{
struct GridIdx 
{ 
    int x{0}, y{0}, z{0}; 
};

struct MapInfo 
{
    double origin_x{0}, origin_y{0}, origin_z{0};
    double resolution{0.2}, z_resolution{0.2};
    int width{0}, height{0}, depth{0};
    std::vector<std::vector<std::vector<uint8_t>>> grid;
};

enum Mode : uint8_t { GRD=0, AIR=1 };

struct TimedGridIdx 
{
    GridIdx idx; 
    double t{0.0}; 
};

using Path      = std::vector<GridIdx>;
using TimedPath = std::vector<TimedGridIdx>;

struct PlanResult 
{
    Path path;
    double travel_s{0.0};
    TimedPath timed;
};

struct RobotCfg 
{
    std::string name;
    int priority{1};
    std::array<double,3> start_xyz{0,0,0}, goal_xyz{0,0,0};
    std::map<std::string,double> f64;
    std::map<std::string,bool>   b;
    std::map<std::string,std::string> s;
};

inline bool operator==(const GridIdx& a, const GridIdx& b) 
{ 
    return a.x==b.x && a.y==b.y && a.z==b.z; 
}
inline Mode modeFromIndex(int z_index) 
{ 
  return (z_index==0) ? GRD : AIR; 
}



class EnergyPathPlanner 
{
public:
    explicit EnergyPathPlanner(rclcpp::Node::SharedPtr node);

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    bool isMapReady() const { return initialized_; }
    const MapInfo& map() const { return map_; }

    GridIdx worldToGrid(double wx, double wy, double wz) const;
    geometry_msgs::msg::Point gridToWorld(const GridIdx& idx) const;
    std::vector<GridIdx> getNeighbors(const GridIdx& cur) const;
    bool isOccupied(const GridIdx& g) const;
    double gridDistance(const GridIdx& a, const GridIdx& b) const;

    std::optional<PlanResult> runSequentialAstar(const GridIdx& start,
                                                 const GridIdx& goal,
                                                 const std::vector<TimedPath>& reservations);

    double e_grd_jpm_{55.0}, e_air_jpm_{190.0};
    double v_grd_mps_{0.55}, v_air_mps_{0.55};
    double w_e_{1.0}, w_t_{0.0};
    
    double conflict_radius_m_{0.6};
    double goal_hold_s_{0.5};
    double time_bin_s_{0.2};
    double z_sep_m_{0.40};    
    
    bool   allow_wait_{true};
    double e_wait_grd_jps_{8.0};
    double e_hover_jps_{190.0*0.55}; 

    double ground_threshold_m_{0.05}, air_threshold_m_{0.50};
    double turn_penalty_s_{0.1};
    int    coll_samples_{2};   
    double r_hard_ratio_{0.6}; 
    double E_takeoff_{0.0}, E_landing_{0.0};
    double E_takeoff_first_{0.0};
    double air_first_edge_factor_{1.0};
    double dyn_ds_max_m_ = 0.20;   
    double dyn_dt_max_s_ = 0.10;   
    int    adapt_smin_   = 2;      
    int    adapt_smax_   = 50; 

    double gamma_grd_eff_{1.2};
    double gamma_air_eff_{1.0};
    double gamma_wait_eff_{2.0};

    std::string octomap_topic_{"/octomap_binary"};

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr voxel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    MapInfo map_{};
    bool has_voxel_{false};
    bool initialized_{false};
};


class PlanningManager 
{
public:
    PlanningManager(rclcpp::Node::SharedPtr node);

    bool waitForMap();

    void loadAndRun();

private:
    void applyParams(const RobotCfg& bot);
    
    void publishPath(const std::string& name, const TimedPath& tp);
    void publishPathsPeriodically();

    rclcpp::Node::SharedPtr node_;
    EnergyPathPlanner planner_;
    
    bool publish_topics_{true};
    bool exit_after_all_{true};
    std::string res_root_;
    std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> path_publishers_;
    rclcpp::TimerBase::SharedPtr path_publish_timer_;
    std::map<std::string, nav_msgs::msg::Path> robot_paths_;
};
}