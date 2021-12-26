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

#ifndef CLIFF_DETECTOR__CLIFF_DETECTOR_NODE_HPP_
#define CLIFF_DETECTOR__CLIFF_DETECTOR_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "cliff_detector/cliff_detector.hpp"

namespace cliff_detector
{

class CliffDetectorNode : public rclcpp::Node
{
public:
  CliffDetectorNode();
  ~CliffDetectorNode();

  CliffDetectorNode(const CliffDetectorNode &) = delete;
  CliffDetectorNode & operator=(const CliffDetectorNode &) = delete;

protected:
  /**
   * @brief depthCb is callback which is called when new depth image appear
   *
   * @param image Depth image provided by image_transport.
   * @param info CameraInfo provided by image_transport.
   */
  void depthCb(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info);

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /// Subscriber for depth image
  image_transport::CameraSubscriber image_sub_;
  /// Publisher for debug image
  image_transport::Publisher pub_;
  /// Publisher for publishing messages with stairs points
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_points_;
  /// Contains cliff detection method implementation
  cliff_detector::CliffDetector detector_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

}  // namespace cliff_detector

#endif  // CLIFF_DETECTOR__CLIFF_DETECTOR_NODE_HPP_
