#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// #include <laserscan_kinect/LaserscanKinectConfig.h>
#include <laserscan_kinect/laserscan_kinect.h>

namespace laserscan_kinect {

class LaserScanKinectNode : public rclcpp::Node {
 public:
  /**
   * @brief LaserScanKinectNode constructor.
   *
   * @param pnh Private node handler.
   */
  LaserScanKinectNode();
  ~LaserScanKinectNode();

private:
  /**
   * @brief depthCb is callback which is called when new depth image appear
   *
   * Callback for depth image and camera info.
   * It converts depth image to laserscan and publishes it at the end.
   *
   * @param depth_msg Depth image provided by image_transport.
   * @param info_msg CameraInfo provided by image_transport.
   */
  void depthCb(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg);
  /**
   * @brief connectCb is callback which is called when new subscriber connected.
   *
   * It allow to subscribe depth image and publish laserscan message only when
   * is laserscan subscriber appear.
   */
  void connectCb();
  /**
   * @brief disconnectCb is called when a subscriber stop subscribing
   *
   * When no one subscribers subscribe laserscan topic, then it stop to subscribe depth image.
   */
  void disconnectCb();
  /**
   * @brief reconfigureCb is dynamic reconfigure callback
   *
   * Callback is necessary to set ROS parameters with dynamic reconfigure server.
   *
   * @param config Dynamic Reconfigure object.
   * @param level Dynamic Reconfigure level.
   */
  // void reconfigureCb(laserscan_kinect::LaserscanKinectConfig &config, uint32_t level);

  /// Subscribes to synchronized Image CameraInfo pairs.
  // image_transport::ImageTransport it_;
  /// Subscriber for image_transport
  // image_transport::CameraSubscriber sub_;
  /// Publisher for output LaserScan messages
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  /// Publisher for image_transport
  image_transport::Publisher pub_dbg_img_;
  /// Dynamic reconfigure server
  // dynamic_reconfigure::Server<laserscan_kinect::LaserscanKinectConfig> srv_;
  /// Object which convert depth image to laserscan and store all parameters
  laserscan_kinect::LaserScanKinect converter_;
  /// Prevents the connectCb and disconnectCb from being called until everything is initialized.
  std::mutex connect_mutex_;
};

} // namespace laserscan_kinect