#pragma once

#include <mutex>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <depth_sensor_pose/DepthSensorPoseConfig.h>
#include <depth_sensor_pose/depth_sensor_pose.h>

namespace depth_sensor_pose {

class DepthSensorPoseNode
{
public:
  DepthSensorPoseNode(ros::NodeHandle& n, ros::NodeHandle& pnh);

  ~DepthSensorPoseNode();

  DepthSensorPoseNode (const DepthSensorPoseNode &) = delete;
  DepthSensorPoseNode & operator= (const DepthSensorPoseNode &) = delete;

  /**
   * @brief setNodeRate sets rate of processing data loop in node.
   *
   * @param rate Frequency of data processing loop in Hz.
   */
  void setNodeRate(const float rate);
  /**
   * @brief getNodeRate gets rate of data processing loop in node.
   *
   * @return Returns data processing loop rate in Hz.
   */
  float getNodeRate();

 protected:
  /**
   * @brief depthCb is callback which is called when new depth image appear
   *
   * Callback for depth image and camera info.
   * It runs sensor mount parameters estimation algorithms
   *
   * @param depth_msg Depth image provided by image_transport.
   * @param info_msg CameraInfo provided by image_transport.
   */
  void depthCallback(const sensor_msgs::ImageConstPtr& depth_msg,
                     const sensor_msgs::CameraInfoConstPtr& info_msg);
  /**
   * @brief connectCb is callback which is called when new subscriber connected.
   *
   * It allow to subscribe depth image and publish prepared message only when
   * is subscriber appear.
   *
   * @param pub Publisher which subscribers are checked.
   */
  void connectCallback();
  /**
   * @brief disconnectCb is called when a subscriber stop subscribing
   *
   * When no one subscribers subscribe topics, then it stop to subscribe depth image.
   *
   * @param pub Publisher which subscribers are checked.
   */
  void disconnectCallback();
  /**
   * @brief reconfigureCb is dynamic reconfigure callback
   *
   * Callback is necessary to set ROS parameters with dynamic reconfigure server.
   *
   * @param config Dynamic Reconfigure object.
   * @param level Dynamic Reconfigure level.
   */
  void reconfigureCallback(depth_sensor_pose::DepthSensorPoseConfig& config, uint32_t level);

 private:

  float node_rate_hz_{1};                 ///< Node loop frequency in Hz
  ros::NodeHandle pnh_;                   ///< Private node handler

  image_transport::ImageTransport it_;    ///< Subscribes to synchronized Image CameraInfo pairs
  image_transport::CameraSubscriber sub_; ///< Subscriber for image_transport

  ros::Publisher pub_height_;             ///< Publisher for estimated sensor height
  ros::Publisher pub_angle_;              ///< Publisher for estimated sensor tilt angle
  image_transport::Publisher pub_;        ///< Publisher for image_transport

  /// Dynamic reconfigure server
  dynamic_reconfigure::Server<depth_sensor_pose::DepthSensorPoseConfig> dyn_rec_srv_;

  /// Object which contain estimation methods
  depth_sensor_pose::DepthSensorPose estimator_;

  /// Prevents the connectCb and disconnectCb from being called until everything is initialized
  std::mutex connection_mutex_;
};

} // namespace depth_sensor_pose