#include "energy_planner/reference_path_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<energy_planner::TimeAwareReferencePathPublisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}