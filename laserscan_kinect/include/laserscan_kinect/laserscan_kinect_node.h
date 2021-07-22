#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <laserscan_kinect/laserscan_kinect.h>

namespace laserscan_kinect {

class LaserScanKinectNode : public rclcpp::Node {
 public:
  /**
   * @brief LaserScanKinectNode constructor.
   */
  LaserScanKinectNode();
  ~LaserScanKinectNode();

private:

  /**
   * @brief depthCb is a callback which is called when new depth image appear
   *
   * Callback for depth image and camera info.
   * It converts depth image to laserscan and publishes it at the end.
   *
   * @param image Depth image provided by image_transport.
   * @param info CameraInfo provided by image_transport.
   */
  void depthCb(const sensor_msgs::msg::Image::ConstSharedPtr& image,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);
  /**
   * @brief parametersCallback is a node reconfigure callback
   *
   * Callback is necessary to set ROS parameters dynamically.
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  /// Subscribes to synchronized Image CameraInfo pairs.
  image_transport::CameraSubscriber subscriber_;
  /// Publisher for output LaserScan messages
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  /// Publisher for image_transport
  image_transport::Publisher pub_dbg_img_;
  /// Object which convert depth image to laserscan and store all parameters
  laserscan_kinect::LaserScanKinect converter_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

} // namespace laserscan_kinect