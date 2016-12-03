/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 * @file   depth_sensor_pose_node.h
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @brief  depth_sensor_pose package
 */

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
private:
  /**
   * @brief depthCb is callback which is called when new depth image appear
   *
   * Callback for depth image and camera info.
   * It runs sensor mount parameters estimation algorithms
   *
   * @param depth_msg Depth image provided by image_transport.
   * @param info_msg CameraInfo provided by image_transport.
   */
  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
  /**
   * @brief connectCb is callback which is called when new subscriber connected.
   *
   * It allow to subscribe depth image and publish prepared message only when
   * is subscriber appear.
   *
   * @param pub Publisher which subscribers are checked.
   */
  void connectCb();
  /**
   * @brief disconnectCb is called when a subscriber stop subscribing
   *
   * When no one subscribers subscribe topics, then it stop to subscribe depth image.
   *
   * @param pub Publisher which subscribers are checked.
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
  void reconfigureCb(depth_sensor_pose::DepthSensorPoseConfig& config, uint32_t level);
//-------------------------------------------------------------------------------------------------
private:

  float node_rate_hz_{1};                 ///< Node loop frequency in Hz
  ros::NodeHandle pnh_;                   ///< Private node handler

  image_transport::ImageTransport it_;    ///< Subscribes to synchronized Image CameraInfo pairs
  image_transport::CameraSubscriber sub_; ///< Subscriber for image_transport

  ros::Publisher pub_height_;             ///< Publisher for estimated sensor height
  ros::Publisher pub_angle_;              ///< Publisher for estimated sensor tilt angle
  image_transport::Publisher pub_;        ///< Publisher for image_transport

  /// Dynamic reconfigure server
  dynamic_reconfigure::Server<depth_sensor_pose::DepthSensorPoseConfig> srv_;

  /// Object which contain estimation methods
  depth_sensor_pose::DepthSensorPose estimator_;

  /// Prevents the connectCb and disconnectCb from being called until everything is initialized
  std::mutex connection_mutex_;
};
}

