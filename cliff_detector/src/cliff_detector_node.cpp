#include <cliff_detector/cliff_detector_node.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace cliff_detector {

CliffDetectorNode::CliffDetectorNode()
  : Node("cliff_detector")

  // New depth image publisher
  pub_ = image_transport::create_publisher(this, "depth");

  // Publisher for stairs points msg
  pub_points_ = this->create_publisher<depth_nav_msgs::Point32List>("points", 2);

  using namespace std::placeholders;
  subscriber_ = image_transport::create_camera_subscription(this, "image",
                std::bind(&DepthSensorPoseNode::depthCb, this, _1, _2), "raw");
}

CliffDetectorNode::~CliffDetectorNode() {
  sub_.shutdown();
}

void CliffDetectorNode::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                const sensor_msgs::CameraInfoConstPtr& info_msg) {
  try {
    // Run cliff detector based on depth image
    detector_.detectCliff(depth_msg, info_msg);

    // Publish stairs points msg
    pub_points_.publish(detector_.stairs_points_msg_);

    // Publishes new depth image with added cliff
    if (detector_.getPublishDepthEnable()) {
      pub_.publish(detector_.new_depth_msg_);
    }
  }
  catch (std::runtime_error& e) {
    // ROS_ERROR_THROTTLE(1.0, "Could not perform stairs detection: %s", e.what());
  }
}

rcl_interfaces::msg::SetParametersResult CliffDetectorNode::parametersCallback(
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
//   detector_.setRangeLimits(config.range_min, config.range_max);
//   detector_.setSensorMountHeight(config.sensor_mount_height);
//   detector_.setSensorTiltAngle(config.sensor_tilt_angle);
//   detector_.setPublishDepthEnable(config.publish_depth);
//   detector_.setCamModelUpdate(config.cam_model_update);

//   detector_.setUsedDepthHeight((unsigned int)config.used_depth_height);
//   detector_.setBlockSize(config.block_size);
//   detector_.setBlockPointsThresh(config.block_points_thresh);
//   detector_.setDepthImgStepRow(config.depth_img_step_row);
//   detector_.setDepthImgStepCol(config.depth_img_step_col);
//   detector_.setGroundMargin(config.ground_margin);

//   detector_.setParametersConfigurated(false);
    }
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }

  return result;
}

} // namespace cliff_detector