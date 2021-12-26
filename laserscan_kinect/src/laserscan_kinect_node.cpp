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

#include "laserscan_kinect/laserscan_kinect_node.hpp"

#include <functional>
#include <vector>
#include <algorithm>

namespace laserscan_kinect
{

LaserScanKinectNode::LaserScanKinectNode()
: Node("laserscan_kinect")
{
  params_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&LaserScanKinectNode::parametersCallback, this, std::placeholders::_1));

  // Declare all node parameters
  declare_parameter("output_frame_id", "camera_depth_frame");
  declare_parameter("range_min", 0.5);
  declare_parameter("range_max", 5.0);
  declare_parameter("scan_height", 440);
  declare_parameter("depth_img_row_step", 2);
  declare_parameter("cam_model_update", false);
  declare_parameter("sensor_mount_height", 0.4);
  declare_parameter("sensor_tilt_angle", 0.0);
  declare_parameter("ground_remove_en", false);
  declare_parameter("ground_margin", 0.05);
  declare_parameter("tilt_compensation_en", false);
  declare_parameter("publish_dbg_info", false);
  declare_parameter("threads_num", 1);

  // Subscription to depth image topic
  publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

  using namespace std::placeholders;
  subscriber_ = image_transport::create_camera_subscription(
    this, "image", std::bind(&LaserScanKinectNode::depthCallback, this, _1, _2), "raw");

  // Debug depth image publisher
  pub_dbg_img_ = image_transport::create_publisher(this, "debug_image");

  RCLCPP_INFO(this->get_logger(), "Node laserscan_kinect initialized.");
}

LaserScanKinectNode::~LaserScanKinectNode()
{
  subscriber_.shutdown();
}

void LaserScanKinectNode::depthCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
{
  try {
    auto laserscan_msg = converter_.getLaserScanMsg(image, info);
    publisher_->publish(*laserscan_msg);

    // Publish debug image
    if (converter_.getPublishDbgImgEnable()) {
      auto dbg_image = converter_.getDbgImage();
      if (dbg_image != nullptr) {
        pub_dbg_img_.publish(dbg_image);
      }
    }
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(), "Could not convert depth image to laserscan: %s", e.what());
  }
}

rcl_interfaces::msg::SetParametersResult LaserScanKinectNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "output_frame_id") {
        converter_.setOutputFrame(parameter.as_string());
      } else if (parameter.get_name() == "range_min") {
        converter_.setMinRange(parameter.as_double());
      } else if (parameter.get_name() == "range_max") {
        converter_.setMaxRange(parameter.as_double());
      } else if (parameter.get_name() == "scan_height") {
        converter_.setScanHeight(parameter.as_int());
      } else if (parameter.get_name() == "depth_img_row_step") {
        converter_.setDepthImgRowStep(parameter.as_int());
      } else if (parameter.get_name() == "cam_model_update") {
        converter_.setCamModelUpdate(parameter.as_bool());
      } else if (parameter.get_name() == "sensor_mount_height") {
        converter_.setSensorMountHeight(parameter.as_double());
      } else if (parameter.get_name() == "sensor_tilt_angle") {
        converter_.setSensorTiltAngle(parameter.as_double());
      } else if (parameter.get_name() == "ground_remove_en") {
        converter_.setGroundRemove(parameter.as_bool());
      } else if (parameter.get_name() == "ground_margin") {
        converter_.setGroundMargin(parameter.as_double());
      } else if (parameter.get_name() == "tilt_compensation_en") {
        converter_.setTiltCompensation(parameter.as_bool());
      } else if (parameter.get_name() == "publish_dbg_info") {
        converter_.setPublishDbgImgEnable(parameter.as_bool());
      } else if (parameter.get_name() == "threads_num") {
        converter_.setThreadsNum(parameter.as_int());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }

  return result;
}

}  // namespace laserscan_kinect
