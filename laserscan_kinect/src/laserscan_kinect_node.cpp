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

#include <laserscan_kinect/laserscan_kinect_node.h>

#include <functional>
#include <vector>
#include <algorithm>

namespace laserscan_kinect {

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
  subscriber_ = image_transport::create_camera_subscription(this, "image",
          std::bind(&LaserScanKinectNode::depthCb, this, _1, _2), "raw");

  // Debug depth image publisher
  pub_dbg_img_ = image_transport::create_publisher(this, "debug_image");

  RCLCPP_INFO(this->get_logger(), "Node laserscan_kinect initialized.");
}

LaserScanKinectNode::~LaserScanKinectNode() {
  subscriber_.shutdown();
}

void LaserScanKinectNode::depthCb(
  const sensor_msgs::msg::Image::ConstSharedPtr& image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
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
  }
  catch (std::runtime_error& e) {
    RCLCPP_ERROR_ONCE(this->get_logger(),
      "Could not convert depth image to laserscan: %s", e.what());
  }
}

rcl_interfaces::msg::SetParametersResult LaserScanKinectNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "output_frame_id") {
        converter_.setOutputFrame(parameter.as_string());
      }
      if (parameter.get_name() == "range_min") {
        converter_.setMinRange(parameter.as_double());
      }
      if (parameter.get_name() == "range_max") {
        converter_.setMaxRange(parameter.as_double());
      }
      if (parameter.get_name() == "scan_height") {
        converter_.setScanHeight(parameter.as_int());
      }
      if (parameter.get_name() == "depth_img_row_step") {
        converter_.setDepthImgRowStep(parameter.as_int());
      }
      if (parameter.get_name() == "cam_model_update") {
        converter_.setCamModelUpdate(parameter.as_bool());
      }
      if (parameter.get_name() == "sensor_mount_height") {
        converter_.setSensorMountHeight(parameter.as_double());
      }
      if (parameter.get_name() == "sensor_tilt_angle") {
        converter_.setSensorTiltAngle(parameter.as_double());
      }
      if (parameter.get_name() == "ground_remove_en") {
        converter_.setGroundRemove(parameter.as_bool());
      }
      if (parameter.get_name() == "ground_margin") {
        converter_.setGroundMargin(parameter.as_double());
      }
      if (parameter.get_name() == "tilt_compensation_en") {
        converter_.setTiltCompensation(parameter.as_bool());
      }
      if (parameter.get_name() == "publish_dbg_info") {
        converter_.setScanConfigurated(parameter.as_bool());
      }
      if (parameter.get_name() == "threads_num") {
        converter_.setThreadsNum(parameter.as_int());
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }

  return result;
}

}  // namespace laserscan_kinect
