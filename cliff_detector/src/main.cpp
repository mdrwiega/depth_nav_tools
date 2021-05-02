#include <cliff_detector/cliff_detector_node.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::shared_ptr<cliff_detector::CliffDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
