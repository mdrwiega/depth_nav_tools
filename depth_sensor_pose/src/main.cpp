#include <depth_sensor_pose/depth_sensor_pose_node.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::shared_ptr<depth_sensor_pose::DepthSensorPoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
