#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <std_msgs/msg/float64.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <depth_sensor_pose/depth_sensor_pose.h>

namespace depth_sensor_pose {

class DepthSensorPoseNode : public rclcpp::Node {
public:
  DepthSensorPoseNode();
  ~DepthSensorPoseNode();

  DepthSensorPoseNode (const DepthSensorPoseNode &) = delete;
  DepthSensorPoseNode & operator= (const DepthSensorPoseNode &) = delete;

 protected:
  /**
   * @brief depthCb is callback which is called when new depth image appear
   *
   * Callback for depth image and camera info.
   * It runs sensor mount parameters estimation algorithms
   *
   * @param image Depth image provided by image_transport.
   * @param info_msg CameraInfo provided by image_transport.
   */
  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);
 private:

  /**
   * @brief parametersCallback is a node reconfigure callback
   *
   * Callback is necessary to set ROS parameters dynamically.
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  image_transport::CameraSubscriber subscriber_; ///< Subscriber for image_transport

  /// Publisher for estimated sensor height
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_height_;
  /// Publisher for estimated sensor tilt angle
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_angle_;
  ///< Publisher for depth image
  image_transport::Publisher pub_;

  depth_sensor_pose::DepthSensorPose estimator_;
};

} // namespace depth_sensor_pose