#include <functional>

#include "depth_sensor_pose/depth_sensor_pose_node.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace depth_sensor_pose;

DepthSensorPoseNode::DepthSensorPoseNode()
  : Node("depth_sensor_pose")
{
  set_on_parameters_set_callback(
      std::bind(&DepthSensorPoseNode::parametersCallback, this, std::placeholders::_1));

  // Tilt angle and height publishers
  pub_height_ = this->create_publisher<std_msgs::Float64>("height", 2);
  pub_angle_ = this->create_publisher<std_msgs::Float64>("tilt_angle", 2);

  using namespace std::placeholders;
  subscriber_ = image_transport::create_camera_subscription(this, "image",
          std::bind(&DepthSensorPoseNode::depthCallback, this, _1, _2), "raw");

  // Debug depth image publisher
  pub_ = image_transport::create_publisher(this, "debug_image");
}

DepthSensorPoseNode::~DepthSensorPoseNode() {
  sub_.shutdown();
}

void DepthSensorPoseNode::depthCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                                        const sensor_msgs::CameraInfoConstPtr& info_msg) {
  try {
    // Estimation of parameters -- sensor pose
    estimator_.estimateParams(depth_msg, info_msg);

    std_msgs::Float64 height, tilt_angle;
    height.data = estimator_.getSensorMountHeight();
    tilt_angle.data = estimator_.getSensorTiltAngle();

    pub_height_->publish(*height);
    pub_angle_->publish(*tilt_angle);
    RCLCPP_DEBUG(this->get_logger(),
      "Publish sensor height (%.2f) and tilt angle (%.2f)", height.data, tilt_angle.data);

    // Publish debug image
    if (estimator_.getPublishDepthEnable()) {
      auto dbg_image = estimator_.getDbgImage();
      if (dbg_image != nullptr) {
        pub_.publish(dbg_image);
      }
    }
  }
  catch (std::runtime_error& e) {
    ROS_ERROR_THROTTLE(1.0, "Could not to run estimation procedure: %s", e.what());
  }
}

rcl_interfaces::msg::SetParametersResult DepthSensorPoseNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    for (const auto &parameter : parameters) {
        if (parameter.get_name() == "range_min") {
            // converter_.setMinRange(parameter.as_double());
        }
        if (parameter.get_name() == "range_max") {
            // converter_.setMaxRange(parameter.as_double());
        }
        if (parameter.get_name() == "scan_height") {
            // converter_.setScanHeight(parameter.as_int());
        }
        if (parameter.get_name() == "depth_img_row_step") {
            // converter_.setDepthImgRowStep(parameter.as_int());
        }
        if (parameter.get_name() == "cam_model_update") {
            // converter_.setCamModelUpdate(parameter.as_bool());
        }
        if (parameter.get_name() == "sensor_mount_height") {
            // converter_.setSensorMountHeight(parameter.as_double());
        }
        if (parameter.get_name() == "sensor_tilt_angle") {
            // converter_.setSensorTiltAngle(parameter.as_double());
        }
        if (parameter.get_name() == "ground_remove_en") {
            // converter_.setGroundRemove(parameter.as_bool());
        }
        if (parameter.get_name() == "ground_margin") {
            // converter_.setGroundMargin(parameter.as_double());
        }
        if (parameter.get_name() == "tilt_compensation_en") {
            // converter_.setTiltCompensation(parameter.as_bool());
        }
        if (parameter.get_name() == "publish_dbg_info") {
            // converter_.setScanConfigurated(parameter.as_bool());
        }
        if (parameter.get_name() == "threads_num") {
        }

      // estimator_.setSensorMountHeightMin(config.mount_height_min);
      // estimator_.setSensorMountHeightMax(config.mount_height_max);
      // estimator_.setSensorTiltAngleMin(config.tilt_angle_min);
      // estimator_.setSensorTiltAngleMax(config.tilt_angle_max);

      // estimator_.setPublishDepthEnable(config.publish_dbg_info);
      // estimator_.setCamModelUpdate(config.cam_model_update);
      // estimator_.setUsedDepthHeight((unsigned int)config.used_depth_height);
      // estimator_.setDepthImgStepRow(config.depth_img_step_row);
      // estimator_.setDepthImgStepCol(config.depth_img_step_col);

      // estimator_.setRansacDistanceThresh(config.ransac_dist_thresh);
      // estimator_.setRansacMaxIter(config.ransac_max_iter);
      // estimator_.setGroundMaxPoints(config.ground_max_points);

      // estimator_.setReconfParamsUpdated(true);
    }
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }

  return result;
}