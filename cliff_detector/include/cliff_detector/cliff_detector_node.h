#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <cliff_detector/cliff_detector.h>

namespace cliff_detector {

class CliffDetectorNode : public rclcpp::Node {
 public:
  CliffDetectorNode();
  ~CliffDetectorNode();

  CliffDetectorNode (const CliffDetectorNode&) = delete;
  CliffDetectorNode & operator= (const CliffDetectorNode&) = delete;

 protected:
  /**
   * @brief depthCb is callback which is called when new depth image appear
   *
   * @param image Depth image provided by image_transport.
   * @param info CameraInfo provided by image_transport.
   */
  void depthCb(const sensor_msgs::msg::Image::ConstSharedPtr& image,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  /// Subscriber for depth image
  image_transport::CameraSubscriber image_sub_;
  /// Publisher for debug image
  image_transport::Publisher pub_;
  /// Publisher for publishing messages with stairs points
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_points_;
  /// Contains cliff detection method implementation
  cliff_detector::CliffDetector detector_;
};

} // namespace cliff_detector