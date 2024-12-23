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

#include <functional>
#include <vector>

#include "depth_sensor_pose/depth_sensor_pose_node.hpp"

namespace depth_sensor_pose
{

DepthSensorPoseNode::DepthSensorPoseNode()
: Node("depth_sensor_pose")
{
  params_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&DepthSensorPoseNode::parametersCallback, this, std::placeholders::_1));

  declare_parameter("output_frame_id", "camera_depth_frame");
  declare_parameter("range_min", 0.5);
  declare_parameter("range_max", 5.0);
  declare_parameter("mount_height_min", 0.4);
  declare_parameter("mount_height_max", 0.8);
  declare_parameter("tilt_angle_min", 30.0);
  declare_parameter("tilt_angle_max", 35.0);
  declare_parameter("cam_model_update", false);
  declare_parameter("used_depth_height", 400);
  declare_parameter("depth_img_step_row", 8);
  declare_parameter("depth_img_step_col", 8);
  declare_parameter("ground_max_points", 2500);
  declare_parameter("ransac_max_iter", 2000);
  declare_parameter("ransac_dist_thresh", 0.001);

  // Tilt angle and height publishers
  pub_height_ = this->create_publisher<std_msgs::msg::Float64>("height", 2);
  pub_angle_ = this->create_publisher<std_msgs::msg::Float64>("tilt_angle", 2);

  using namespace std::placeholders;
  subscriber_ = image_transport::create_camera_subscription(
    this, "image", std::bind(&DepthSensorPoseNode::depthCallback, this, _1, _2), "raw");

  // Debug depth image publisher
  pub_ = image_transport::create_publisher(this, "debug_image");
}

DepthSensorPoseNode::~DepthSensorPoseNode()
{
  subscriber_.shutdown();
}

void DepthSensorPoseNode::depthCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
{
  try {
    // Estimation of sensor pose
    estimator_.estimateParams(image, info);

    std_msgs::msg::Float64 height, tilt_angle;
    height.data = estimator_.getSensorMountHeight();
    tilt_angle.data = estimator_.getSensorTiltAngle();

    pub_height_->publish(height);
    pub_angle_->publish(tilt_angle);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Publish sensor height (%.2f) and tilt angle (%.2f)", height.data, tilt_angle.data);

    // Publish debug image
    if (estimator_.getPublishDepthEnable()) {
      auto dbg_image = estimator_.getDbgImage();
      if (dbg_image) {
        pub_.publish(dbg_image);
      }
    }
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Could not to run estimation procedure: %s", e.what());
  }
}

rcl_interfaces::msg::SetParametersResult DepthSensorPoseNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "range_min") {
        estimator_.setMinRange(parameter.as_double());
      } else if (parameter.get_name() == "range_max") {
        estimator_.setMaxRange(parameter.as_double());
      } else if (parameter.get_name() == "mount_height_min") {
        estimator_.setSensorMountHeightMin(parameter.as_double());
      } else if (parameter.get_name() == "mount_height_max") {
        estimator_.setSensorMountHeightMax(parameter.as_double());
      } else if (parameter.get_name() == "tilt_angle_min") {
        estimator_.setSensorTiltAngleMin(parameter.as_double());
      } else if (parameter.get_name() == "tilt_angle_max") {
        estimator_.setSensorMountHeightMax(parameter.as_double());
      } else if (parameter.get_name() == "cam_model_update") {
        estimator_.setCamModelUpdate(parameter.as_bool());
      } else if (parameter.get_name() == "used_depth_height") {
        estimator_.setUsedDepthHeight(parameter.as_int());
      } else if (parameter.get_name() == "depth_img_step_row") {
        estimator_.setDepthImgStepRow(parameter.as_int());
      } else if (parameter.get_name() == "depth_img_step_col") {
        estimator_.setDepthImgStepCol(parameter.as_int());
      } else if (parameter.get_name() == "publish_dbg_info") {
        estimator_.setPublishDepthEnable(parameter.as_bool());
      } else if (parameter.get_name() == "ransac_dist_thresh") {
        estimator_.setRansacDistanceThresh(parameter.as_double());
      } else if (parameter.get_name() == "ransac_max_iter") {
        estimator_.setRansacMaxIter(parameter.as_int());
      } else if (parameter.get_name() == "ground_max_points") {
        estimator_.setGroundMaxPoints(parameter.as_int());
      }
    }
    estimator_.setReconfParamsUpdated(true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }

  return result;
}

}  // namespace depth_sensor_pose
