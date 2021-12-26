// Copyright 2016-2021 Michał Drwięga (drwiega.michal@gmail.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LASERSCAN_KINECT__LASERSCAN_KINECT_NODE_HPP_
#define LASERSCAN_KINECT__LASERSCAN_KINECT_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "laserscan_kinect/laserscan_kinect.hpp"

namespace laserscan_kinect
{

class LaserScanKinectNode : public rclcpp::Node
{
public:
  /**
   * @brief LaserScanKinectNode constructor
   */
  LaserScanKinectNode();
  ~LaserScanKinectNode();

private:
  /**
   * @brief It is a callback which is called when new depth image is received
   *
   * Callback for depth image and camera info.
   * It converts depth image to laserscan and publishes it.
   *
   * @param image Depth image provided by image_transport.
   * @param info CameraInfo provided by image_transport.
   */
  void depthCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info);
  /**
   * @brief parametersCallback is a node reconfigure callback
   *
   * Callback is necessary to set ROS parameters dynamically.
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

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

}  // namespace laserscan_kinect

#endif  // LASERSCAN_KINECT__LASERSCAN_KINECT_NODE_HPP_
