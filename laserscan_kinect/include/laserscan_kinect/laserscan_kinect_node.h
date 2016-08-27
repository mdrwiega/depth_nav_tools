/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Michal Drwiega (drwiega.michal@gmail.com)
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
 * @file   main.cpp
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @brief  laserscan_kinect package
 */

#pragma once

#include <mutex>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>

#include <laserscan_kinect/LaserscanKinectConfig.h>
#include <laserscan_kinect/laserscan_kinect.h>

namespace laserscan_kinect
{ 
class LaserScanKinectNode
{
public:
  /**
   * @brief LaserScanKinectNode constructor.
   *
   * @param n  Node handler.
   * @param pnh Private node handler.
   */
  LaserScanKinectNode(ros::NodeHandle& n, ros::NodeHandle& pnh);
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
  void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);
  /**
   * @brief connectCb is callback which is called when new subscriber connected.
   *
   * It allow to subscribe depth image and publish laserscan message only when
   * is laserscan subscriber appear.
   *
   * @param pub Publisher which subscribers are checked.
   */
  void connectCb(const ros::SingleSubscriberPublisher& pub);
  /**
   * @brief disconnectCb is called when a subscriber stop subscribing
   *
   * When no one subscribers subscribe laserscan topic, then it stop to subscribe depth image.
   *
   * @param pub Publisher which subscribers are checked.
   */
  void disconnectCb(const ros::SingleSubscriberPublisher& pub);
  /**
   * @brief reconfigureCb is dynamic reconfigure callback
   *
   * Callback is necessary to set ROS parameters with dynamic reconfigure server.
   *
   * @param config Dynamic Reconfigure object.
   * @param level Dynamic Reconfigure level.
   */
  void reconfigureCb(laserscan_kinect::LaserscanKinectConfig &config, uint32_t level);

  /// Private node handler used to generate the transport hints in the connectCb.
  ros::NodeHandle pnh_;
  /// Subscribes to synchronized Image CameraInfo pairs.
  image_transport::ImageTransport it_;
  /// Subscriber for image_transport
  image_transport::CameraSubscriber sub_;
  /// Publisher for output LaserScan messages
  ros::Publisher pub_;
  /// Dynamic reconfigure server
  dynamic_reconfigure::Server<laserscan_kinect::LaserscanKinectConfig> srv_;
  /// Object which convert depth image to laserscan and store all parameters
  laserscan_kinect::LaserScanKinect converter_;
  /// Prevents the connectCb and disconnectCb from being called until everything is initialized.
  std::mutex connect_mutex_;
};
};
