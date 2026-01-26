#ifndef OCTOMAP_PUBLISHER_HPP_
#define OCTOMAP_PUBLISHER_HPP_

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

GridIdx worldToGrid(double wx, double wy, double wz, const GridParams& g);

class OctomapPublisher : public rclcpp::Node
{
public:
  OctomapPublisher();

private:
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_vis_pub_;

  vector<BoxObstacle> parseSdfFile(const string& filename);
  void fillFreeSpace(const GridParams& g, std::shared_ptr<octomap::OcTree>& octree);
  void insertKnownBoxesIntoOctoMap(
    const vector<BoxObstacle>& boxes,
    std::shared_ptr<octomap::OcTree>& octree,
    const GridParams& g);
};

#endif // OCTOMAP_PUBLISHER_HPP_