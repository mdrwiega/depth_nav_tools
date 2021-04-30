#include <laserscan_kinect/laserscan_kinect_node.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<laserscan_kinect::LaserScanKinectNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
