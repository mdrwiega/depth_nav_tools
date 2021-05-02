#pragma once

#include <image_transport/image_transport.h>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <depth_nav_msgs/Point32List.h>

#include <cliff_detector/cliff_detector.h>

namespace cliff_detector {

class CliffDetectorNode {
 public:
  CliffDetectorNode();
  ~CliffDetectorNode();

 protected:
  /**
   * @brief depthCb is callback which is called when new depth image appear
   *
   * @param depth_msg Depth image provided by image_transport.
   * @param info_msg CameraInfo provided by image_transport.
   */
  void depthCb( const sensor_msgs::ImageConstPtr& depth_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg);

  /// Subscriber for image_transport
  image_transport::CameraSubscriber sub_;
  /// Publisher for image_transport
  image_transport::Publisher pub_;
  /// Publisher for publishing messages with stairs points
  ros::Publisher pub_points_;
  /// Contains cliff detection method implementation
  cliff_detector::CliffDetector detector_;
};

} // namespace cliff_detector