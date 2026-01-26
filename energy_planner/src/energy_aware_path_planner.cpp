#include "energy_planner/energy_aware_path_planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/parameter_service.hpp"


namespace energy_planner 
{
    // =========================================================
    // 1. Internal Structs & Constants
    // =========================================================
    constexpr double EPSILON = 1e-9;
    constexpr double INF = std::numeric_limits<double>::infinity();
    constexpr uint8_t OCCUPIED = 255;

    struct Vec3 { double x, y, z; };
    struct Interval { double start, end; };
    struct AABB { double min_x, min_y, max_x, max_y; };

    struct TimeKey 
    { 
        GridIdx idx; 
        int tbin; // 시간 이산화 
    };

    struct TKHash 
    {
        size_t operator()(TimeKey const& k) const 
        {
            size_t h_x = std::hash<int>()(k.idx.x);
            size_t h_y = std::hash<int>()(k.idx.y);
            size_t h_z = std::hash<int>()(k.idx.z);
            size_t h_t = std::hash<int>()(k.tbin);
            return ((h_x ^ (h_y << 1)) >> 1) ^ (h_z << 1) ^ (h_t << 1);
        }
    };
    
    struct TKEq 
    {
        bool operator()(TimeKey const& a, TimeKey const& b) const 
        {
            return (a.tbin == b.tbin) && (a.idx == b.idx);
        }
    };

    struct Node 
    {
        TimeKey key; 
        double g_time{0}, g_cost{0}, f{0};
        TimeKey parent; 
        bool has_parent{false};
        Mode mode{GRD};
        int wait_run = 0; // 연속 대기 횟수 제한용
    };

    struct Cmp 
    {
        bool operator()(const Node& a, const Node& b) const 
        {
            if (std::abs(a.f - b.f) > EPSILON) return a.f > b.f;
            return a.g_cost > b.g_cost;
        }
    };

    struct CellCalendar 
    {
        std::vector<Interval> occ;
        std::vector<Interval> safe;
    };

    struct DenseReservations 
    {
        double bin{0.2};
        std::unordered_map<int, std::vector<Vec3>> by_bin;
        std::unordered_map<int, AABB> bounds;
    };

    struct SmoothParams 
    {
        double v_grd, v_air;
        double goal_hold_s;
        double hard_R;
    };

    // =========================================================
    // 2. Static Helper Functions (Math & Logic)
    // =========================================================

    static inline double getSqr(double x) 
    { 
        return x * x; 
    }
    
    static inline int getGlobalIndex(const GridIdx& g, const MapInfo& m) 
    {
        return (g.z * m.height + g.y) * m.width + g.x;
    }

    static inline Vec3 gridToWorld_internal(const GridIdx& idx, const MapInfo& m) 
    {
        return 
        {
            m.origin_x + (idx.x + 0.5) * m.resolution,
            m.origin_y + (idx.y + 0.5) * m.resolution,
            (idx.z == 0) ? 0.0 : m.origin_z + (idx.z + 0.5) * m.z_resolution
        };
    }

    static double heuristicEnergy(const GridIdx& a, const GridIdx& b, const MapInfo& m,
                                  double e_grd, double e_air) 
    {
        double dx = (a.x - b.x) * m.resolution;
        double dy = (a.y - b.y) * m.resolution;
        double dist_xy = std::hypot(dx, dy);
        double dist_z  = std::abs(a.z - b.z) * m.z_resolution;
        
        return dist_xy * e_grd + dist_z * e_air; 
    }

    static bool isStaticFreeSegment(const GridIdx& a, const GridIdx& b, const MapInfo& m) 
    {
        Vec3 A = gridToWorld_internal(a, m);
        Vec3 B = gridToWorld_internal(b, m);
        double dist = std::sqrt(getSqr(A.x - B.x) + getSqr(A.y - B.y) + getSqr(A.z - B.z));
        double step = std::max(m.resolution * 0.5, 0.05);
        int steps = std::max(1, (int)std::ceil(dist / step));
            
        for (int i = 1; i < steps; i++) 
        {
            double u = (double)i / steps;
            int gx = (int)std::floor(((A.x + (B.x - A.x)*u) - m.origin_x) / m.resolution);
            int gy = (int)std::floor(((A.y + (B.y - A.y)*u) - m.origin_y) / m.resolution);
            int gz = (int)std::floor(((A.z + (B.z - A.z)*u) - m.origin_z) / m.z_resolution);
            
            if (gx < 0 || gx >= m.width || gy < 0 || gy >= m.height || gz < 0 || gz >= m.depth) continue;
            if (m.grid[gz][gy][gx] == OCCUPIED) return false;
        }
        return true;
    }

    static int getIntervalIndexAt(const std::vector<Interval>& intervals, double t) 
    {
        auto it = std::lower_bound(
            intervals.begin(), 
            intervals.end(), 
            t, 
            [](const Interval& i, double val) 
            { 
                return i.end < val; 
            }
        );
        
        if (it != intervals.end() && it->start <= t + EPSILON) 
        {
            return std::distance(intervals.begin(), it);
        }
        return -1;
    }

    static bool isTimeInside(const std::vector<Interval>& intervals, double t) 
    {
        return getIntervalIndexAt(intervals, t) != -1;
    }

    static void mergeIntervals(std::vector<Interval>& intervals) 
    {
        if (intervals.empty()) return;

        std::sort(
            intervals.begin(), 
            intervals.end(), 
            [](const Interval& a, const Interval& b)
            { 
                return a.start < b.start; 
            }
        );

        std::vector<Interval> merged; 
        merged.reserve(intervals.size());
        Interval curr = intervals[0];

        for (size_t i = 1; i < intervals.size(); ++i) 
        {
            if (intervals[i].start <= curr.end + EPSILON) 
            {
                curr.end = std::max(curr.end, intervals[i].end);
            } 
            else 
            {
                merged.push_back(curr);
                curr = intervals[i];
            }
        }
        merged.push_back(curr);
        intervals.swap(merged);
    }

    static void buildSafeFromOcc(CellCalendar& c, double horizon) 
    {
        c.safe.clear();
        if (c.occ.empty()) 
        { 
            c.safe.push_back({0.0, horizon}); 
            return; 
        }

        mergeIntervals(c.occ);

        double t = 0.0;
        for (const auto& o : c.occ) 
        {
            if (t < o.start) c.safe.push_back({t, o.start});
            t = std::max(t, o.end);
        }
        if (t < horizon) c.safe.push_back({t, horizon});
    }

    static bool findFeasibleDeparture(const std::vector<Interval>& U,
                                      const std::vector<Interval>& V,
                                      double t_earliest, double duration, double& t_dep_out)
    {
        if (duration <= 0) 
        { 
            t_dep_out = t_earliest; 
            return true; 
        }

        auto it_u = std::lower_bound(
            U.begin(), 
            U.end(), 
            t_earliest, 
            [](const Interval& i, double v)
            { 
                return i.end < v; 
            }
        );

        for (; it_u != U.end(); ++it_u) 
        {
            double t0_u = std::max(t_earliest, it_u->start);
            if (t0_u > it_u->end - duration + EPSILON) continue;

            for (const auto& v_iv : V) 
            {
                double dep_min = std::max(t0_u, v_iv.start - duration);
                double dep_max = std::min(it_u->end - duration, v_iv.end - duration);
                if (dep_min <= dep_max + EPSILON) 
                {
                    t_dep_out = dep_min;
                    return true;
                }
            }
        }
        return false;
    }

    // --- Octomap & Grid Utils ---
    static bool samplePoseAtTime(const TimedPath& tp, double t, const MapInfo& m, Vec3& out) 
    {
        if (tp.empty()) return false;
        
        if (t <= tp.front().t) 
        { 
            auto p = gridToWorld_internal(tp.front().idx, m); 
            out = Vec3{p.x, p.y, p.z}; 
            return true; 
        }
        if (t >= tp.back().t) 
        { 
            auto p = gridToWorld_internal(tp.back().idx, m); 
            out = Vec3{p.x, p.y, p.z}; 
            return true; 
        }

        auto it = std::upper_bound(
            tp.begin(), 
            tp.end(), 
            t, 
            [](double val, const TimedGridIdx& node)
            { 
                return val < node.t; 
            }
        );
        
        size_t idx = std::distance(tp.begin(), it);
        if (idx == 0) idx = 1;
        
        const auto& prev = tp[idx - 1];
        const auto& next = tp[idx];
        double u = (t - prev.t) / std::max(EPSILON, next.t - prev.t);
        
        auto A = gridToWorld_internal(prev.idx, m);
        auto B = gridToWorld_internal(next.idx, m);
        out = Vec3{ A.x + (B.x - A.x)*u, A.y + (B.y - A.y)*u, A.z + (B.z - A.z)*u };
        return true;
    }

    static DenseReservations buildDenseReservations(const std::vector<TimedPath>& reservations,
                                                    const MapInfo& m, double bin, double hold_s) 
    {
        DenseReservations DR;
        DR.bin = std::max(1e-6, bin);
        
        for (const auto& tp : reservations) 
        {
            if (tp.empty()) continue;
            int tb_start = (int)std::floor(tp.front().t / DR.bin);
            int tb_end   = (int)std::floor((tp.back().t + hold_s) / DR.bin);

            for (int tb = tb_start; tb <= tb_end; ++tb) 
            {
                Vec3 p;
                if (!samplePoseAtTime(tp, tb * DR.bin, m, p)) continue;
                
                DR.by_bin[tb].push_back(p);
                auto& bb = DR.bounds[tb];
                if (DR.by_bin[tb].size() == 1) 
                {
                    bb = {p.x, p.y, p.x, p.y};
                } 
                else 
                {
                    bb.min_x = std::min(bb.min_x, p.x); bb.max_x = std::max(bb.max_x, p.x);
                    bb.min_y = std::min(bb.min_y, p.y); bb.max_y = std::max(bb.max_y, p.y);
                }
            }
        }
        return DR;
    }

    static std::vector<CellCalendar> buildCalendar(const std::vector<TimedPath>& reservations,
                                                   const MapInfo& m, double bin, double hold_s,
                                                   double R, double z_sep, double horizon)
    {
        int N = m.width * m.height * m.depth;
        std::vector<CellCalendar> calendar(N);
        if (reservations.empty()) 
        {
            for (auto& c : calendar) c.safe.push_back({0.0, horizon});
            return calendar;
        }

        DenseReservations DR = buildDenseReservations(reservations, m, bin, hold_s);
        int r_xy = std::max(0, (int)std::ceil(R / m.resolution));
        int r_z  = std::max(0, (int)std::ceil(z_sep / m.z_resolution));
        double half_bin = 0.5 * bin;

        auto clamp = [](int v, int max_v) 
        { 
            return std::max(0, std::min(max_v - 1, v)); 
        };

        for (const auto& [tb, points] : DR.by_bin) 
        {
            double t = tb * DR.bin;
            double ts = std::max(0.0, t - half_bin);
            double te = std::min(horizon, t + half_bin);

            for (const auto& q : points) 
            {
                int gx = clamp((int)((q.x - m.origin_x) / m.resolution), m.width);
                int gy = clamp((int)((q.y - m.origin_y) / m.resolution), m.height);
                int gz = clamp((int)((q.z - m.origin_z) / m.z_resolution), m.depth);

                for (int dz = -r_z; dz <= r_z; ++dz) 
                {
                    int z = gz + dz; 
                    if (z < 0 || z >= m.depth) continue;
                    
                    double wz = m.origin_z + (z + 0.5) * m.z_resolution;
                    if (std::abs(wz - q.z) > z_sep + EPSILON) continue;

                    for (int dy = -r_xy; dy <= r_xy; ++dy) 
                    {
                        int y = gy + dy; if (y < 0 || y >= m.height) continue;
                        for (int dx = -r_xy; dx <= r_xy; ++dx) 
                        {
                            int x = gx + dx; if (x < 0 || x >= m.width) continue;
                            
                            double cx = m.origin_x + (x + 0.5) * m.resolution;
                            double cy = m.origin_y + (y + 0.5) * m.resolution;
                            if (getSqr(cx - q.x) + getSqr(cy - q.y) <= R * R + EPSILON) 
                            {
                                calendar[getGlobalIndex({x, y, z}, m)].occ.push_back({ts, te});
                            }
                        }
                    }
                }
            }
        }
        for (auto& c : calendar) buildSafeFromOcc(c, horizon);
        return calendar;
    }

    // --- Smoothing Logic ---
    static bool isDynamicSafeSegment(const GridIdx& a, const GridIdx& b,
                                     double t0, double t1,
                                     const MapInfo& m, const std::vector<CellCalendar>& cal,
                                     double time_bin) 
    {
        if (t1 <= t0) return true;
        auto A = gridToWorld_internal(a, m); 
        auto B = gridToWorld_internal(b, m); 
        double dist = std::sqrt(getSqr(A.x - B.x) + getSqr(A.y - B.y) + getSqr(A.z - B.z));
        
        double dt_max = std::max(1e-3, 0.5 * std::max(1e-6, time_bin));
        double ds_max = std::max(1e-3, 0.33 * std::min(m.resolution, m.z_resolution));
        
        int K = std::max({2, (int)std::ceil((t1 - t0) / dt_max), (int)std::ceil(dist / ds_max)});

        for (int i = 0; i <= K; ++i) 
        {
            double u = (double)i / K;
            double t = t0 + (t1 - t0) * u;
            
            int gx = (int)((A.x + (B.x - A.x)*u - m.origin_x) / m.resolution);
            int gy = (int)((A.y + (B.y - A.y)*u - m.origin_y) / m.resolution);
            int gz = (int)((A.z + (B.z - A.z)*u - m.origin_z) / m.z_resolution);
            
            if(gx<0||gx>=m.width || gy<0||gy>=m.height || gz<0||gz>=m.depth) continue;
            const auto& safe = cal[getGlobalIndex({gx, gy, gz}, m)].safe;
            if (!isTimeInside(safe, t)) return false;
        }
        return true;
    }

    static TimedPath smoothShortcutRetimed_sipp(const TimedPath& tp,
                                                const MapInfo& m,
                                                const std::vector<CellCalendar>& cal,
                                                const SmoothParams& sp,
                                                double time_bin) 
    {
        if (tp.size() < 2) return tp;

        TimedPath out; out.reserve(tp.size());
        out.push_back(tp.front());
        
        size_t i = 0;
        double t_curr = tp.front().t;

        while (i < tp.size() - 1) 
        {
            size_t best_next = i + 1;
            
            // 1. Static Check
            for (size_t j = i + 2; j < tp.size(); ++j) 
            {
                if (modeFromIndex(tp[j].idx.z) != modeFromIndex(tp[i].idx.z)) break;
                if (!isStaticFreeSegment(tp[i].idx, tp[j].idx, m)) break;
                best_next = j;
            }

            GridIdx u = tp[i].idx;
            const auto& safe_u = cal[getGlobalIndex(u, m)].safe;
            bool success = false;
            size_t j_try = best_next;

            // 2. Dynamic Check
            while (j_try > i) 
            {
                GridIdx v = tp[j_try].idx;
                double v_ref = (modeFromIndex(u.z) == AIR) ? sp.v_air : sp.v_grd;
                double dist = std::sqrt(getSqr((u.x-v.x)*m.resolution) + getSqr((u.y-v.y)*m.resolution) + getSqr((u.z-v.z)*m.z_resolution));
                double dt = dist / std::max(1e-6, v_ref);
                
                const auto& safe_v = cal[getGlobalIndex(v, m)].safe;
                double t_dep;
                
                if (!findFeasibleDeparture(safe_u, safe_v, t_curr, dt, t_dep)) 
                {
                    --j_try; continue;
                }
                
                double t_arr = t_dep + dt;
                if (!isDynamicSafeSegment(u, v, t_dep, t_arr, m, cal, time_bin)) 
                {
                    --j_try; continue;
                }

                if (t_dep > t_curr + 1e-4) 
                {
                   if (t_dep > t_curr) t_curr = t_dep; 
                }

                out.push_back({v, t_arr});
                t_curr = t_arr;
                i = j_try;
                success = true;
                break;
            }

            if (!success) 
            {
                GridIdx v = tp[i+1].idx;
                double v_ref = (modeFromIndex(u.z) == AIR) ? sp.v_air : sp.v_grd;
                double dist = std::sqrt(getSqr((u.x-v.x)*m.resolution) + getSqr((u.y-v.y)*m.resolution) + getSqr((u.z-v.z)*m.z_resolution));
                double dt = dist / std::max(1e-6, v_ref);
                
                const auto& safe_v = cal[getGlobalIndex(v, m)].safe;
                double t_dep;
                if (findFeasibleDeparture(safe_u, safe_v, t_curr, dt, t_dep)) 
                {
                    out.push_back({v, t_dep + dt});
                    t_curr = t_dep + dt;
                } 
                else 
                {
                     out.push_back({v, std::max(t_curr + dt, tp[i+1].t)});
                     t_curr = out.back().t;
                }
                i++;
            }
        }
        
        if (!out.empty() && sp.goal_hold_s > EPSILON) 
        {
            out.push_back({out.back().idx, out.back().t + sp.goal_hold_s});
        }
        return out;
    }


    // =========================================================
    // 3. EnergyPathPlanner Implementation
    // =========================================================

    EnergyPathPlanner::EnergyPathPlanner(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        map_.origin_x = 0.0; map_.origin_y = -5.0; map_.origin_z = 0.0;
        map_.resolution = 0.2; map_.z_resolution = 0.2;
        map_.width = 100; map_.height = 50; map_.depth = 20;
        map_.grid.assign(map_.depth, std::vector<std::vector<uint8_t>>(map_.height, std::vector<uint8_t>(map_.width, 0)));

        e_grd_jpm_ = node_->declare_parameter("e_grd_jpm", e_grd_jpm_);
        e_air_jpm_ = node_->declare_parameter("e_air_jpm", e_air_jpm_);
        v_grd_mps_ = node_->declare_parameter("v_grd_mps", v_grd_mps_);
        v_air_mps_ = node_->declare_parameter("v_air_mps", v_air_mps_);
        w_e_ = node_->declare_parameter("w_e", w_e_);
        w_t_ = node_->declare_parameter("w_t", w_t_);
        conflict_radius_m_ = node_->declare_parameter("conflict_radius_m", conflict_radius_m_);
        goal_hold_s_ = node_->declare_parameter("goal_hold_s", goal_hold_s_);
        time_bin_s_ = node_->declare_parameter("time_bin_s", time_bin_s_);
        allow_wait_ = node_->declare_parameter("allow_wait", allow_wait_);
        e_wait_grd_jps_ = node_->declare_parameter("e_wait_grd_jps", e_wait_grd_jps_);
        e_hover_jps_ = node_->declare_parameter("e_hover_jps", e_hover_jps_);
        gamma_air_eff_ = node_->declare_parameter("gamma_air_eff_", gamma_air_eff_);
        octomap_topic_ = node_->declare_parameter("octomap_topic", octomap_topic_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        voxel_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic_, qos, std::bind(&EnergyPathPlanner::octomapCallback, this, std::placeholders::_1));

        path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("global_path", 1);
    }

    void EnergyPathPlanner::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        // 1. ROS 메시지 -> Octomap 변환
        std::unique_ptr<octomap::AbstractOcTree> abs(
            msg->binary ? octomap_msgs::binaryMsgToMap(*msg) : octomap_msgs::fullMsgToMap(*msg));
        if (!abs) { RCLCPP_WARN(node_->get_logger(), "Failed to convert Octomap"); return; }
        
        auto* tree = dynamic_cast<octomap::OcTree*>(abs.get());
        if (!tree) { RCLCPP_WARN(node_->get_logger(), "Octomap is not OcTree"); return; }

        // 2. 맵 설정 (생성자 값 사용)
        const int D = map_.depth;
        const int H = map_.height;
        const int W = map_.width;

        map_.resolution = tree->getResolution();
        map_.z_resolution = map_.resolution;

        // 3. 메모리 할당
        auto& grid = map_.grid;
        if ((int)grid.size() != D || 
            (D > 0 && (int)grid[0].size() != H) || 
            (H > 0 && (int)grid[0][0].size() != W))
        {
            grid.assign(D, std::vector<std::vector<uint8_t>>(H, std::vector<uint8_t>(W, 0)));
        }
        else
        {
            for(auto& layer : grid)
                for(auto& row : layer)
                    std::fill(row.begin(), row.end(), 0);
        }

        for (int z = 0; z < D; ++z) 
        {
            for (int y = 0; y < H; ++y) 
            {
                for (int x = 0; x < W; ++x) 
                {
                    const double wx = map_.origin_x + (x + 0.5) * map_.resolution;
                    const double wy = map_.origin_y + (y + 0.5) * map_.resolution;
                    const double wz = map_.origin_z + (z + 0.5) * map_.z_resolution;

                    octomap::OcTreeNode* node = tree->search(octomap::point3d(wx, wy, wz));
                    
                    if (node != NULL && tree->isNodeOccupied(node)) 
                    {
                        grid[z][y][x] = 255;
                    }
                    else 
                    {
                        grid[z][y][x] = 0;
                    }
                }
            }
        }
        has_voxel_ = true;
        initialized_ = true; 
        voxel_sub_.reset(); 

        RCLCPP_INFO(node_->get_logger(), "Octomap received & saved. Subscriber shutdown (One-shot mode).");
    }

    GridIdx EnergyPathPlanner::worldToGrid(double wx, double wy, double wz) const 
    {
        int x = (int)std::floor((wx - map_.origin_x) / map_.resolution);
        int y = (int)std::floor((wy - map_.origin_y) / map_.resolution);
        int z = (int)std::floor((wz - map_.origin_z) / map_.z_resolution);
        return 
        {
            std::max(0, std::min(map_.width - 1, x)), 
            std::max(0, std::min(map_.height - 1, y)), 
            std::max(0, std::min(map_.depth - 1, z)) 
        };
    }

    geometry_msgs::msg::Point EnergyPathPlanner::gridToWorld(const GridIdx& idx) const 
    {
        geometry_msgs::msg::Point p;
        p.x = map_.origin_x + (idx.x + 0.5) * map_.resolution;
        p.y = map_.origin_y + (idx.y + 0.5) * map_.resolution;
        p.z = map_.origin_z + (idx.z == 0 ? 0 : (idx.z + 0.5) * map_.z_resolution);
        return p;
    }

    double EnergyPathPlanner::gridDistance(const GridIdx& a, const GridIdx& b) const 
    {
        double dx = (a.x - b.x) * map_.resolution;
        double dy = (a.y - b.y) * map_.resolution;
        double dz = (a.z - b.z) * map_.z_resolution;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    std::vector<GridIdx> EnergyPathPlanner::getNeighbors(const GridIdx& cur) const 
    {
        std::vector<GridIdx> ns; 
        ns.reserve(26);
        for (int dz = -1; dz <= 1; ++dz) 
        {
            for (int dy = -1; dy <= 1; ++dy) 
            {
                for (int dx = -1; dx <= 1; ++dx) 
                {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    if (cur.z == 0 && dz < 0) continue; 

                    GridIdx n{cur.x + dx, cur.y + dy, cur.z + dz};
                    if (n.x < 0 || n.y < 0 || n.z < 0 ||
                        n.x >= map_.width || n.y >= map_.height || n.z >= map_.depth)
                      continue;
            
                    ns.push_back(n);
                }
            }
        }
        return ns;
    }
    
    bool EnergyPathPlanner::isOccupied(const GridIdx& g) const 
    {
        if (g.x < 0 || g.x >= map_.width || g.y < 0 || g.y >= map_.height || g.z < 0 || g.z >= map_.depth) return true;
        if (map_.grid.empty()) return false;
        return map_.grid[g.z][g.y][g.x] == OCCUPIED;
    }

    std::optional<PlanResult> EnergyPathPlanner::runSequentialAstar(
        const GridIdx& start, const GridIdx& goal, const std::vector<TimedPath>& reservations) 
    {
        if (map_.width == 0) return std::nullopt;

        bool use_time = !reservations.empty();
        node_->get_parameter("v_grd_mps", v_grd_mps_);
        node_->get_parameter("v_air_mps", v_air_mps_);
        double v_max = std::max(v_grd_mps_, v_air_mps_);
        node_->get_parameter("time_bin_s", time_bin_s_);
        double time_bin = std::max(1e-6, time_bin_s_);

        // 1. Calendar Building
        std::vector<CellCalendar> calendar(map_.width * map_.height * map_.depth);
        if (use_time) 
        {
            double dist = gridDistance(start, goal);
            double horizon = 3.0 * (dist / std::max(1e-6, v_max)) + goal_hold_s_ + 60.0;
            calendar = buildCalendar(reservations, map_, time_bin, goal_hold_s_, conflict_radius_m_, z_sep_m_, horizon);
        }

        // 2. A* Init
        std::priority_queue<Node, std::vector<Node>, Cmp> open;
        std::unordered_map<TimeKey, double, TKHash, TKEq> best_g;
        std::unordered_map<TimeKey, Node, TKHash, TKEq> all_nodes;

        Node s_node;
        s_node.key.idx = start;
        s_node.mode = modeFromIndex(start.z);
        
        if (use_time) 
        {
            const auto& safe = calendar[getGlobalIndex(start, map_)].safe;
            if (safe.empty()) return std::nullopt;
            int idx = getIntervalIndexAt(safe, 0.0);
            s_node.key.tbin = (idx < 0) ? 0 : idx;
            s_node.g_time = (idx < 0) ? safe[0].start : 0.0;
        } 
        else 
        {
            s_node.key.tbin = 0; s_node.g_time = 0.0;
        }
        
        s_node.f = s_node.g_cost + heuristicEnergy(start, goal, map_, e_grd_jpm_, e_air_jpm_*gamma_air_eff_);

        open.push(s_node);
        best_g[s_node.key] = 0;
        all_nodes[s_node.key] = s_node;

        while (!open.empty()) 
        {
            Node cur = open.top(); 
            open.pop();

            if (best_g.count(cur.key) && cur.g_cost > best_g[cur.key] + EPSILON) continue;

            // Goal Reached
            if (cur.key.idx == goal) 
            {
                PlanResult res;
                res.travel_s = cur.g_time;
                TimeKey k = cur.key;
                while (all_nodes.count(k)) 
                {
                    res.path.push_back(all_nodes[k].key.idx);
                    res.timed.push_back({all_nodes[k].key.idx, all_nodes[k].g_time});
                    if (!all_nodes[k].has_parent) break;
                    k = all_nodes[k].parent;
                }
                std::reverse(res.path.begin(), res.path.end());
                std::reverse(res.timed.begin(), res.timed.end());
                return res;
            }

            // Current Interval Info
            Interval I_curr{0, INF};
            if (use_time) 
            {
                const auto& safe = calendar[getGlobalIndex(cur.key.idx, map_)].safe;
                if (cur.key.tbin >= 0 && cur.key.tbin < (int)safe.size()) 
                {
                    I_curr = safe[cur.key.tbin];
                }
                else continue;
            }

            // Expand Neighbors
            for (const auto& nb : getNeighbors(cur.key.idx)) 
            {
                if (isOccupied(nb)) continue;
                if (cur.mode == GRD && nb.z < cur.key.idx.z) continue; 
                
                if (use_time && !isStaticFreeSegment(cur.key.idx, nb, map_)) continue;

                double dist = gridDistance(cur.key.idx, nb);
                double v_ref = (modeFromIndex(nb.z) == AIR) ? v_air_mps_ : v_grd_mps_;
                double dt = dist / std::max(1e-6, v_ref);
                
                double power_move = (modeFromIndex(nb.z) == AIR) ? gamma_air_eff_*e_air_jpm_ : e_grd_jpm_;
                double cost_move = w_e_ * (power_move * dist);

                double t_dep = cur.g_time;
                bool dep_ok = true;
                int next_tbin = 0;

                if (use_time) 
                {
                    double t0 = std::max(cur.g_time, I_curr.start);
                    if (t0 > I_curr.end - dt + EPSILON) dep_ok = false;
                    else 
                    {
                        const auto& safe_nb = calendar[getGlobalIndex(nb, map_)].safe;
                        if (!findFeasibleDeparture({I_curr}, safe_nb, t0, dt, t_dep)) dep_ok = false;
                        if (dep_ok) 
                        {
                            next_tbin = getIntervalIndexAt(safe_nb, t_dep + dt);
                            if (next_tbin == -1) dep_ok = false;
                        }
                    }
                }

                if (!dep_ok) continue;

                if (t_dep > cur.g_time) 
                {
                    node_->get_parameter("allow_wait", allow_wait_);
                    if (!allow_wait_) continue;
                    double power_wait = (cur.mode == AIR) ? e_hover_jps_ : e_wait_grd_jps_;
                    cost_move += w_e_ * power_wait * (t_dep - cur.g_time);
                }

                Node nxt;
                nxt.key = {nb, next_tbin};
                nxt.g_time = t_dep + dt;
                nxt.g_cost = cur.g_cost + cost_move;
                nxt.parent = cur.key; nxt.has_parent = true;
                nxt.mode = modeFromIndex(nb.z);
                nxt.f = nxt.g_cost + heuristicEnergy(nb, goal, map_, e_grd_jpm_, e_air_jpm_*gamma_air_eff_); // Heuristic

                if (!best_g.count(nxt.key) || nxt.g_cost < best_g[nxt.key] - EPSILON) 
                {
                    best_g[nxt.key] = nxt.g_cost;
                    all_nodes[nxt.key] = nxt;
                    open.push(nxt);
                }
            } 

            if (use_time)
            {
                node_->get_parameter("allow_wait", allow_wait_);
                if(allow_wait_)
                {
                    const auto& safe_curr = calendar[getGlobalIndex(cur.key.idx, map_)].safe;
                    int next_interval_idx = cur.key.tbin + 1;

                    if (next_interval_idx < (int)safe_curr.size()) 
                    {
                        double t_next_start = safe_curr[next_interval_idx].start;
                        if (t_next_start > cur.g_time) 
                        {
                            double wait_duration = t_next_start - cur.g_time;
                            double power_wait = (cur.mode == AIR) ? e_hover_jps_ : e_wait_grd_jps_;
                            double wait_cost = w_e_ * power_wait * wait_duration;

                            Node wait_node;
                            wait_node.key = {cur.key.idx, next_interval_idx}; 
                            wait_node.g_time = t_next_start;
                            wait_node.g_cost = cur.g_cost + wait_cost;
                            wait_node.parent = cur.key; wait_node.has_parent = true;
                            wait_node.mode = cur.mode;
                            wait_node.f = wait_node.g_cost + heuristicEnergy(cur.key.idx, goal, map_, e_grd_jpm_, e_air_jpm_*gamma_air_eff_);
                            
                            if (!best_g.count(wait_node.key) || wait_node.g_cost < best_g[wait_node.key] - EPSILON) 
                            {
                                best_g[wait_node.key] = wait_node.g_cost;
                                all_nodes[wait_node.key] = wait_node;
                                open.push(wait_node);
                            }
                        }
                    }
                } 
            }
        }
        return std::nullopt;
    }


    // =========================================================
    // 4. PlanningManager Implementation
    // =========================================================

    PlanningManager::PlanningManager(rclcpp::Node::SharedPtr node)
        : node_(node), planner_(node)
    {
        publish_topics_ = node_->declare_parameter("energy_aware_path_planner.common.publish_topics", true);
        exit_after_all_ = node_->declare_parameter("energy_aware_path_planner.common.exit_after_all", true);
        res_root_ = node_->declare_parameter("energy_aware_path_planner.common.reservation_param_root", "/reservations");

        path_publish_timer_ = node_->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PlanningManager::publishPathsPeriodically, this));
    }

    bool PlanningManager::waitForMap() 
    {
        RCLCPP_INFO(node_->get_logger(), "Waiting for Map...");
        while (rclcpp::ok() && !planner_.isMapReady()) 
        {
            rclcpp::spin_some(node_);
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
        return rclcpp::ok();
    }

    void PlanningManager::applyParams(const RobotCfg& bot) 
    {
        if (bot.f64.count("v_grd_mps")) planner_.v_grd_mps_ = bot.f64.at("v_grd_mps");
        if (bot.f64.count("v_air_mps")) planner_.v_air_mps_ = bot.f64.at("v_air_mps");
        if (bot.f64.count("e_grd_jpm")) planner_.e_grd_jpm_ = bot.f64.at("e_grd_jpm");
        if (bot.f64.count("e_air_jpm")) planner_.e_air_jpm_ = bot.f64.at("e_air_jpm");
        // [추가] 대기 에너지 파라미터 적용
        if (bot.f64.count("e_wait_grd_jps")) planner_.e_wait_grd_jps_ = bot.f64.at("e_wait_grd_jps");
        if (bot.f64.count("e_hover_jps")) planner_.e_hover_jps_ = bot.f64.at("e_hover_jps");
    }

    void PlanningManager::publishPath(const std::string& name, const TimedPath& tp) 
    {
        node_->get_parameter("energy_aware_path_planner.common.publish_topics", publish_topics_);
        if (!publish_topics_) return;
        nav_msgs::msg::Path msg; 
        msg.header.frame_id = "map";
        msg.header.stamp = node_->get_clock()->now();

        for (const auto& p : tp) 
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = msg.header;
            ps.header.stamp = rclcpp::Time(p.t * 1e9);
            ps.pose.position = planner_.gridToWorld(p.idx);
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        
        std::string topic = name + "/global_path";
        if (path_publishers_.find(topic) == path_publishers_.end()) 
        {
            rclcpp::QoS qos(rclcpp::KeepLast(1));
            qos.transient_local();
            path_publishers_[topic] = node_->create_publisher<nav_msgs::msg::Path>(topic, qos);
        }
        path_publishers_[topic]->publish(msg);
        robot_paths_[name] = msg; // Store for periodic publishing
    }

    void PlanningManager::publishPathsPeriodically()
    {
        for (auto& pair : robot_paths_)
        {
            const std::string& name = pair.first;
            nav_msgs::msg::Path& msg = pair.second;
            std::string topic = name + "/global_path";

            if (path_publishers_.find(topic) != path_publishers_.end())
            {
                msg.header.stamp = node_->get_clock()->now();
                // The timestamps within the poses do not need to be updated for RViz,
                // but if other nodes use them, they should be updated.
                // For now, we just update the header stamp.
                path_publishers_[topic]->publish(msg);
            }
        }
    }

    void PlanningManager::loadAndRun() 
    {
        std::vector<RobotCfg> robots;
        
        node_->declare_parameter("robots", rclcpp::ParameterValue(std::vector<std::string>()));
        std::vector<std::string> robot_names = node_->get_parameter("robots").as_string_array();

        for(const auto& robot_name : robot_names)
        {
            RobotCfg cfg;
            cfg.name = robot_name;
            
            node_->declare_parameter("energy_aware_path_planner." + robot_name + ".priority", 0);
            cfg.priority = node_->get_parameter("energy_aware_path_planner." + robot_name + ".priority").as_int();

            node_->declare_parameter("energy_aware_path_planner." + robot_name + ".start_xyz", std::vector<double>{0.0, 0.0, 0.0});
            std::vector<double> start_xyz = node_->get_parameter("energy_aware_path_planner." + robot_name + ".start_xyz").as_double_array();
            cfg.start_xyz = {start_xyz[0], start_xyz[1], start_xyz[2]};

            node_->declare_parameter("energy_aware_path_planner." + robot_name + ".goal_xyz", std::vector<double>{0.0, 0.0, 0.0});
            std::vector<double> goal_xyz = node_->get_parameter("energy_aware_path_planner." + robot_name + ".goal_xyz").as_double_array();
            cfg.goal_xyz = {goal_xyz[0], goal_xyz[1], goal_xyz[2]};
            
            robots.push_back(cfg);
        }

        if(robots.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "No 'robots' param found. Using Default Test Robot.");
            robots.push_back({"test_bot", 1, {0, -5, 0}, {0, 5, 0}});
        }
        
        std::sort(robots.begin(), robots.end(), [](const RobotCfg& a, const RobotCfg& b)
        {
            return a.priority < b.priority;
        });

        std::vector<TimedPath> reservations;
        
        // 2. Sequential Planning Loop
        for (const auto& bot : robots) 
        {
            RCLCPP_INFO(node_->get_logger(), "Planning for %s [P:%d]...", bot.name.c_str(), bot.priority);
            applyParams(bot);

            GridIdx s = planner_.worldToGrid(bot.start_xyz[0], bot.start_xyz[1], bot.start_xyz[2]);
            GridIdx g = planner_.worldToGrid(bot.goal_xyz[0], bot.goal_xyz[1], bot.goal_xyz[2]);

            // Plan
            auto res = planner_.runSequentialAstar(s, g, reservations);
            
            if (res) 
            {
                // Smooth
                SmoothParams sp{planner_.v_grd_mps_, planner_.v_air_mps_, planner_.goal_hold_s_, planner_.r_hard_ratio_};
                double horizon = res->travel_s + 60.0;
                auto cal = buildCalendar(reservations, planner_.map(), planner_.time_bin_s_, planner_.goal_hold_s_, 
                                         planner_.conflict_radius_m_, planner_.z_sep_m_, horizon);
                
                auto smoothed_path = smoothShortcutRetimed_sipp(res->timed, planner_.map(), cal, sp, planner_.time_bin_s_);
                
                // Result
                reservations.push_back(smoothed_path);
                publishPath(bot.name, smoothed_path);
                RCLCPP_INFO(node_->get_logger(), "Success! Travel Time: %.2fs (Smoothed)", smoothed_path.back().t - smoothed_path.front().t);
            } 
            else 
            {
                RCLCPP_WARN(node_->get_logger(), "Failed to find path for %s", bot.name.c_str());
            }
        }
        node_->get_parameter("energy_aware_path_planner.common.exit_after_all", exit_after_all_);
        
        // If we have paths to publish periodically, we should not exit.
        if (!robot_paths_.empty()) {
            exit_after_all_ = false;
        }
        
        if (exit_after_all_) 
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            rclcpp::shutdown();
        } 
        else 
        {
            RCLCPP_INFO(node_->get_logger(), "Continuing to spin for periodic path publication.");
            rclcpp::spin(node_);
        }
    }

} // namespace energy_planner 끝

// =========================================================
// 5. Main Function
// =========================================================
int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("energy_path_planner");
    energy_planner::PlanningManager manager(node);
    
    if (manager.waitForMap()) 
    {
        manager.loadAndRun();
    }

    return 0;
}