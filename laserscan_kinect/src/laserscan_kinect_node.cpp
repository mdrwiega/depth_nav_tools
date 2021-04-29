#include <laserscan_kinect/laserscan_kinect_node.h>
#include <boost/bind.hpp>
#include <functional>

namespace laserscan_kinect {

LaserScanKinectNode::LaserScanKinectNode()
  : Node("laserscan_kinect")
  // , it_(this)
    // pnh_(pnh), it_(pnh), srv_(pnh)
{
  std::lock_guard<std::mutex> lock(connect_mutex_);

  // Dynamic reconfigure server callback
  // srv_.setCallback(boost::bind(&LaserScanKinectNode::reconfigureCb, this, _1, _2));

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

// void LaserScanKinectNode::reconfigureCb(laserscan_kinect::LaserscanKinectConfig& config,
//                                         [[maybe_unused]] uint32_t level) {
//   converter_.setOutputFrame(config.output_frame_id);
//   converter_.setRangeLimits(config.range_min, config.range_max);
//   converter_.setScanHeight(config.scan_height);
//   converter_.setDepthImgRowStep(config.depth_img_row_step);
//   converter_.setCamModelUpdate(config.cam_model_update);

//   converter_.setSensorMountHeight(config.sensor_mount_height);
//   converter_.setSensorTiltAngle(config.sensor_tilt_angle);
//   converter_.setGroundRemove(config.ground_remove_en);
//   converter_.setGroundMargin(config.ground_margin);
//   converter_.setTiltCompensation(config.tilt_compensation_en);

//   converter_.setScanConfigurated(false);
//   converter_.setPublishDbgImgEnable(config.publish_dbg_info);
//   converter_.setThreadsNum(config.threads_num);
// }

} // namespace laserscan_kinect