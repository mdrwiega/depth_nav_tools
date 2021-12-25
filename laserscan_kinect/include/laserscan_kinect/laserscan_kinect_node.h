// Software License Agreement (BSD License)
//
// Copyright (c) 2016-2021, Michal Drwiega (drwiega.michal@gmail.com)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     1. Redistributions of source code must retain the above copyright
//        notice, this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived
//        from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <laserscan_kinect/laserscan_kinect.h>

namespace laserscan_kinect
{

class LaserScanKinectNode : public rclcpp::Node {
public:
  /**
   * @brief LaserScanKinectNode constructor
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
