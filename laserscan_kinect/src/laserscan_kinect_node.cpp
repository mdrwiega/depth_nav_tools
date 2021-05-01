#include <laserscan_kinect/laserscan_kinect_node.h>
#include <boost/bind.hpp>
#include <functional>

namespace laserscan_kinect {

LaserScanKinectNode::LaserScanKinectNode()
  : Node("laserscan_kinect")
  // , it_(this)
    // pnh_(pnh), it_(pnh), srv_(pnh)
{
  RCLCPP_ERROR(this->get_logger(), "Initialize laserscan_kinect node");
  std::lock_guard<std::mutex> lock(connect_mutex_);

  set_on_parameters_set_callback(
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
  // pub_ = pnh.advertise<sensor_msgs::LaserScan>("scan", 10,
  //                                             std::bind(&LaserScanKinectNode::connectCb, this),
  //                                             std::bind(&LaserScanKinectNode::disconnectCb, this));

  // New depth image publisher
  // pub_dbg_img_ = it_.advertise("debug_image", 1,
  //   std::bind(&LaserScanKinectNode::connectCb, this),
  //   std::bind(&LaserScanKinectNode::disconnectCb, this));
}

LaserScanKinectNode::~LaserScanKinectNode() {
  // sub_.shutdown();
}

void LaserScanKinectNode::depthCb(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
  // try {
  //   sensor_msgs::LaserScanPtr laserscan_msg = converter_.getLaserScanMsg(depth_msg, info_msg);
  //   pub_.publish(laserscan_msg);

  //   // Publish debug image
  //   if (converter_.getPublishDbgImgEnable()) {
  //     auto dbg_image = converter_.getDbgImage();
  //     if (dbg_image != nullptr) {
  //       pub_dbg_img_.publish(dbg_image);
  //     }
  //   }
  // }
  // catch (std::runtime_error& e) {
  //   RCLCPP_ERROR_THROTTLE(1.0, "Could not convert depth image to laserscan: %s", e.what());
  // }
}

void LaserScanKinectNode::connectCb() {
  std::lock_guard<std::mutex> lock(connect_mutex_);

  // if (sub_ == nullptr && (pub_.getNumSubscribers() > 0 || pub_dbg_img_.getNumSubscribers() > 0)) {
  //   RCLCPP_DEBUG("Connecting to depth topic.");
  //   image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
  //   sub_ = it_.subscribeCamera("image", 10, &LaserScanKinectNode::depthCb, this, hints);
  // }
}

void LaserScanKinectNode::disconnectCb() {
  std::lock_guard<std::mutex> lock(connect_mutex_);

  // if (pub_.getNumSubscribers() == 0 && pub_dbg_img_.getNumSubscribers() == 0) {
  //   RCLCPP_DEBUG("Unsubscribing from depth topic.");
  //   // sub_.shutdown();
  // }
}

rcl_interfaces::msg::SetParametersResult LaserScanKinectNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &parameter : parameters) {
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

      // RCLCPP_INFO(get_logger(), "Parameter %s changed: %s",
      //   parameter.get_name().c_str(), parameter.as_string().c_str());
  }
  return result;
}

} // namespace laserscan_kinect