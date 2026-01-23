#include <tinyxml2.h>
#include <iostream>
#include <vector>
#include <string>
#include <tuple>
#include <queue>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "octomap/OcTree.h"
#include "octomap/octomap.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace tinyxml2;
using std::vector;
using std::string;

struct BoxObstacle 
{
  tf2::Transform transform;
  tf2::Vector3 size;       
};

struct GridParams 
{
  int width = 100;
  int height = 50;
  int depth = 20;
  double resolution = 0.2;
  double z_resolution = 0.2;
  double origin_x = 0.0;
  double origin_y = -5.0;
  double origin_z = 0.0;
};

struct GridIdx { int x, y, z; }; 

GridIdx worldToGrid(double wx, double wy, double wz, const GridParams& g) 
{
    int x = static_cast<int>(std::floor((wx - g.origin_x) / g.resolution));
    int y = static_cast<int>(std::floor((wy - g.origin_y) / g.resolution));
    int z = static_cast<int>(std::floor((wz - g.origin_z) / g.z_resolution));
    x = std::max(0, std::min(g.width  - 1, x));
    y = std::max(0, std::min(g.height - 1, y));
    z = std::max(0, std::min(g.depth  - 1, z));
    return {x, y, z};
}

class OctomapGeneratorNode : public rclcpp::Node
{
public:
  OctomapGeneratorNode()
  : Node("octomap_publish_node")
  {
    // 1. 파라미터 로드
    this->declare_parameter("sdf_file", "/home/ubuntu/PX4-Autopilot/Tools/simulation/gz/worlds/obstacles_wall.sdf");
    std::string sdf_file = this->get_parameter("sdf_file").as_string();

    // 2. 파싱 및 맵 생성
    GridParams grid;
    auto obstacles = parseSdfFile(sdf_file);
    RCLCPP_INFO(this->get_logger(), "Parsed %zu obstacles from SDF file: %s", obstacles.size(), sdf_file.c_str());

    auto octree = std::make_shared<octomap::OcTree>(grid.resolution);
    fillFreeSpace(grid, octree);
    insertKnownBoxesIntoOctoMap(obstacles, octree, grid);
    octree->updateInnerOccupancy();

    // 3. Publisher 생성 (멤버 변수에 저장)
    rclcpp::QoS qos_profile(1);
    qos_profile.transient_local(); // Latch 기능

    // [수정 포인트] 멤버 변수(octomap_pub_, obstacle_vis_pub_)에 할당
    octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/octomap_binary", qos_profile);
    obstacle_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("known_obstacles", qos_profile);

    // 4. 메시지 발행
    octomap_msgs::msg::Octomap octo_msg;
    if (octomap_msgs::binaryMapToMsg(*octree, octo_msg)) 
    {
      octo_msg.header.frame_id = "map";
      octo_msg.header.stamp = this->now();
      octomap_pub_->publish(octo_msg);
      RCLCPP_INFO(this->get_logger(), "OctoMap published.");
    } 
    else 
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert octree to message.");
    }

    visualization_msgs::msg::MarkerArray boxes_markers;
    int id = 0;
    for (const auto& box : obstacles) 
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = this->now();
      m.ns = "known_box";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      
      tf2::toMsg(box.transform, m.pose);
      
      m.scale.x = box.size.x();
      m.scale.y = box.size.y();
      m.scale.z = box.size.z();
      m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; m.color.a = 0.2;
      boxes_markers.markers.push_back(m);
    }
    obstacle_vis_pub_->publish(boxes_markers);
    RCLCPP_INFO(this->get_logger(), "Obstacle markers published.");
  }

private:
  // [수정 포인트] Publisher를 멤버 변수로 선언하여 생명 주기 유지
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_vis_pub_;

  vector<BoxObstacle> parseSdfFile(const string& filename) 
  {
    vector<BoxObstacle> obstacles;
    XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) 
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load SDF file: %s", filename.c_str());
      return obstacles;
    }

    auto* root = doc.FirstChildElement("sdf");
    if (!root) 
    {
        RCLCPP_ERROR(this->get_logger(), "Could not find <sdf> element in file.");
        return obstacles;
    }

    XMLElement* models_container = root->FirstChildElement("world");
    if (!models_container) {
      models_container = root; 
    }

    for (auto* model = models_container->FirstChildElement("model");
         model; model = model->NextSiblingElement("model")) 
    {
      const char* name = model->Attribute("name");
      if (name && (strcmp(name, "ground_plane") == 0)) 
      {
          continue;
      }

      tf2::Transform model_transform;
      model_transform.setIdentity();
      if (auto* p = model->FirstChildElement("pose")) 
      {
        double x=0, y=0, z=0, r=0, pi=0, ya=0;
        sscanf(p->GetText(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &r, &pi, &ya);
        model_transform.setOrigin(tf2::Vector3(x, y, z));
        tf2::Quaternion q;
        q.setRPY(r, pi, ya);
        model_transform.setRotation(q);
      }

      for (auto* link = model->FirstChildElement("link");
           link; link = link->NextSiblingElement("link")) 
      {
        tf2::Transform link_transform;
        link_transform.setIdentity();
        if (auto* lp = link->FirstChildElement("pose")) 
        {
          double x=0, y=0, z=0, r=0, pi=0, ya=0;
          sscanf(lp->GetText(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &r, &pi, &ya);
          link_transform.setOrigin(tf2::Vector3(x, y, z));
          tf2::Quaternion q;
          q.setRPY(r, pi, ya);
          link_transform.setRotation(q);
        }
        
        for (auto* col = link->FirstChildElement("collision");
             col; col = col->NextSiblingElement("collision"))
        {
            tf2::Transform collision_transform;
            collision_transform.setIdentity();
            if (auto* cp = col->FirstChildElement("pose")) 
            {
              double x=0, y=0, z=0, r=0, pi=0, ya=0;
              sscanf(cp->GetText(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &r, &pi, &ya);
              collision_transform.setOrigin(tf2::Vector3(x, y, z));
              tf2::Quaternion q;
              q.setRPY(r, pi, ya);
              collision_transform.setRotation(q);
            }

            auto* geo = col->FirstChildElement("geometry");
            if (!geo) continue;
            auto* box = geo->FirstChildElement("box");
            if (!box) continue;
            auto* sz_elem  = box->FirstChildElement("size");
            if (!sz_elem) continue;

            double sx, sy, sz;
            sscanf(sz_elem->GetText(), "%lf %lf %lf", &sx, &sy, &sz);

            BoxObstacle ob;
            ob.size = tf2::Vector3(sx, sy, sz);
            ob.transform = model_transform * link_transform * collision_transform;
            obstacles.push_back(ob);
        }
      }
    }
    return obstacles;
  }

  void fillFreeSpace(const GridParams& g, std::shared_ptr<octomap::OcTree>& octree) 
  {
    for (int ix = 0; ix < g.width; ++ix) 
    {
      for (int iy = 0; iy < g.height; ++iy) 
      {
        for (int iz = 0; iz < g.depth; ++iz) 
        {
          double x = g.origin_x + (ix + 0.5) * g.resolution;
          double y = g.origin_y + (iy + 0.5) * g.resolution;
          double z = g.origin_z + (iz + 0.5) * g.z_resolution;
          octomap::point3d p(x, y, z);
          octree->updateNode(p, false);
        }
      }
    }
  }

  void insertKnownBoxesIntoOctoMap
  (
    const vector<BoxObstacle>& boxes,
    std::shared_ptr<octomap::OcTree>& octree,
    const GridParams& g)
  {
    const double robot_radius = 0.5;

    const double h_res_x = g.resolution / 2.0;
    const double h_res_y = g.resolution / 2.0;
    const double h_res_z = g.z_resolution / 2.0;
    
    const std::vector<tf2::Vector3> sample_offsets = 
    {
      tf2::Vector3(0.0,     0.0,     0.0),    
      tf2::Vector3( h_res_x,  h_res_y,  h_res_z),
      tf2::Vector3( h_res_x,  h_res_y, -h_res_z),
      tf2::Vector3( h_res_x, -h_res_y,  h_res_z),
      tf2::Vector3( h_res_x, -h_res_y, -h_res_z),
      tf2::Vector3(-h_res_x,  h_res_y,  h_res_z),
      tf2::Vector3(-h_res_x,  h_res_y, -h_res_z),
      tf2::Vector3(-h_res_x, -h_res_y,  h_res_z),
      tf2::Vector3(-h_res_x, -h_res_y, -h_res_z) 
    };

    for (const auto& box : boxes) 
    {
      double hx = box.size.x() / 2.0 + robot_radius;
      double hy = box.size.y() / 2.0 + robot_radius;
      double hz = box.size.z() / 2.0 + robot_radius + 0.2;

      vector<tf2::Vector3> local_corners;
      local_corners.push_back(tf2::Vector3( hx,  hy,  hz)); local_corners.push_back(tf2::Vector3( hx,  hy, -hz));
      local_corners.push_back(tf2::Vector3( hx, -hy,  hz)); local_corners.push_back(tf2::Vector3( hx, -hy, -hz));
      local_corners.push_back(tf2::Vector3(-hx,  hy,  hz)); local_corners.push_back(tf2::Vector3(-hx,  hy, -hz));
      local_corners.push_back(tf2::Vector3(-hx, -hy,  hz)); local_corners.push_back(tf2::Vector3(-hx, -hy, -hz));
      
      tf2::Vector3 min_pt( 1e6,  1e6,  1e6);
      tf2::Vector3 max_pt(-1e6, -1e6, -1e6);
      for(const auto& lc : local_corners) {
          tf2::Vector3 wc = box.transform * lc;
          min_pt.setX(std::min(min_pt.x(), wc.x())); min_pt.setY(std::min(min_pt.y(), wc.y())); min_pt.setZ(std::min(min_pt.z(), wc.z()));
          max_pt.setX(std::max(max_pt.x(), wc.x())); max_pt.setY(std::max(max_pt.y(), wc.y())); max_pt.setZ(std::max(max_pt.z(), wc.z()));
      }

      tf2::Transform box_inverse_transform = box.transform.inverse();

      GridIdx min_idx = worldToGrid(min_pt.x(), min_pt.y(), min_pt.z(), g);
      GridIdx max_idx = worldToGrid(max_pt.x(), max_pt.y(), max_pt.z(), g);

      for (int ix = min_idx.x; ix <= max_idx.x; ++ix) 
      {
        for (int iy = min_idx.y; iy <= max_idx.y; ++iy) 
        {
          for (int iz = min_idx.z; iz <= max_idx.z; ++iz) 
          {
            double x = g.origin_x + (ix + 0.5) * g.resolution;
            double y = g.origin_y + (iy + 0.5) * g.resolution;
            double z = g.origin_z + (iz + 0.5) * g.z_resolution;

            tf2::Vector3 voxel_center_world(x, y, z);
            bool is_occupied = false;
            
            for (const auto& offset : sample_offsets) {
              tf2::Vector3 sample_point_world = voxel_center_world + offset;
              tf2::Vector3 sample_point_local = box_inverse_transform * sample_point_world;
              
              if (std::abs(sample_point_local.x()) <= hx + 1e-3 &&
                  std::abs(sample_point_local.y()) <= hy + 1e-3 &&
                  std::abs(sample_point_local.z()) <= hz + 1e-3)
              {
                is_occupied = true;
                break;
              }
            }
            
            if (is_occupied) {
              octree->updateNode(octomap::point3d(x, y, z), true);
            }
          }
        }
      }
    }
  }
};

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OctomapGeneratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}