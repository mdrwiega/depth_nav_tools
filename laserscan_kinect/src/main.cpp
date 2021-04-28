#include <laserscan_kinect/laserscan_kinect_node.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("laserscan_kinect");

  laserscan_kinect::LaserScanKinectNode converter(node);

  ros::spin();

  return 0;
}
