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

#ifndef DEPTH_SENSOR_POSE__DEPTH_SENSOR_POSE_NODE_HPP_
#define DEPTH_SENSOR_POSE__DEPTH_SENSOR_POSE_NODE_HPP_

#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "std_msgs/msg/float64.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "depth_sensor_pose/depth_sensor_pose.hpp"

namespace depth_sensor_pose
{

class DepthSensorPoseNode : public rclcpp::Node
{
public:
  DepthSensorPoseNode();
  ~DepthSensorPoseNode();

  DepthSensorPoseNode(const DepthSensorPoseNode &) = delete;
  DepthSensorPoseNode & operator=(const DepthSensorPoseNode &) = delete;

protected:
  /**
   * @brief depthCb is callback which is called when new depth image appear
   *
   * Callback for depth image and camera info.
   * It runs sensor mount parameters estimation algorithms
   *
   * @param image Depth image provided by image_transport.
   * @param info CameraInfo provided by image_transport.
   */
  void depthCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info);

private:
  /**
   * @brief parametersCallback is a node reconfigure callback
   *
   * Callback to set ROS parameters dynamically.
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  ///< Subscriber for image_transport
  image_transport::CameraSubscriber subscriber_;

  /// Publisher for estimated sensor height
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_height_;
  /// Publisher for estimated sensor tilt angle
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_angle_;
  ///< Publisher for depth image
  image_transport::Publisher pub_;

  depth_sensor_pose::DepthSensorPose estimator_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

}  // namespace depth_sensor_pose

#endif  // DEPTH_SENSOR_POSE__DEPTH_SENSOR_POSE_NODE_HPP_
