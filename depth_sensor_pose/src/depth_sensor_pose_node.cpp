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

#include "depth_sensor_pose/depth_sensor_pose_node.h"

constexpr int MAX_NODE_RATE = 30;

using namespace depth_sensor_pose;

//=================================================================================================
DepthSensorPoseNode::DepthSensorPoseNode(ros::NodeHandle& n, ros::NodeHandle& pnh)
  : pnh_(pnh), it_(n), srv_(pnh)
{
  std::lock_guard<std::mutex> lock(connection_mutex_);
  
  // Dynamic reconfigure server callback
  srv_.setCallback(boost::bind(&DepthSensorPoseNode::reconfigureCb, this, _1, _2));
  
  // Lazy subscription implementation
  // Tilt angle and height publisher
  pub_height_ = n.advertise<std_msgs::Float64>("depth_sensor_pose/height", 2,
        boost::bind(&DepthSensorPoseNode::connectCb, this),
        boost::bind(&DepthSensorPoseNode::disconnectCb, this));

  pub_angle_ = n.advertise<std_msgs::Float64>("depth_sensor_pose/tilt_angle", 2,
        boost::bind(&DepthSensorPoseNode::connectCb, this),
        boost::bind(&DepthSensorPoseNode::disconnectCb, this));

  // New depth image publisher
  pub_ = it_.advertise("depth_sensor_pose/depth", 1,
                       boost::bind(&DepthSensorPoseNode::connectCb, this),
                       boost::bind(&DepthSensorPoseNode::disconnectCb, this));
}

//=================================================================================================
DepthSensorPoseNode::~DepthSensorPoseNode()
{
  sub_.shutdown();
}

//=================================================================================================
void DepthSensorPoseNode::setNodeRate(const float rate)
{
  if (rate <= MAX_NODE_RATE)
    node_rate_hz_ = rate;
  else
    node_rate_hz_ = MAX_NODE_RATE;
}

//=================================================================================================
float DepthSensorPoseNode::getNodeRate()
{
  return node_rate_hz_;
}

//=================================================================================================
void DepthSensorPoseNode::depthCb( const sensor_msgs::ImageConstPtr& depth_msg,
                                   const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  try
  {
    // Estimation of parameters -- sensor pose
    estimator_.estimateParams(depth_msg, info_msg);

    std_msgs::Float64 height, tilt_angle;
    height.data = estimator_.getSensorMountHeight();
    tilt_angle.data = estimator_.getSensorTiltAngle();

    ROS_ERROR("height = %.4f angle = %.4f", estimator_.getSensorMountHeight(),
              estimator_.getSensorTiltAngle());

    pub_height_.publish(height);
    pub_angle_.publish(tilt_angle);

    // Publishes new depth image with added downstairs
    if (estimator_.getPublishDepthEnable())
      pub_.publish(estimator_.new_depth_msg_);
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR_THROTTLE(1.0, "Could not to run estimatation procedure: %s", e.what());
  }
}

//=================================================================================================
void DepthSensorPoseNode::connectCb()
{
  std::lock_guard<std::mutex> lock(connection_mutex_);
  if (!sub_ && (pub_height_.getNumSubscribers() > 0 || pub_angle_.getNumSubscribers() > 0
                || pub_.getNumSubscribers() > 0))
  {
    ROS_DEBUG("Connecting to depth topic.");
    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
    sub_ = it_.subscribeCamera("image", 1, &DepthSensorPoseNode::depthCb, this, hints);
  }
}

//=================================================================================================
void DepthSensorPoseNode::disconnectCb()
{
  std::lock_guard<std::mutex> lock(connection_mutex_);
  if (pub_height_.getNumSubscribers() == 0 && pub_angle_.getNumSubscribers() == 0
      && pub_.getNumSubscribers() == 0)
  {
    ROS_DEBUG("Unsubscribing from depth topic.");
    sub_.shutdown();
  }
}

//=================================================================================================
void DepthSensorPoseNode::reconfigureCb( depth_sensor_pose::DepthSensorPoseConfig& config,
                                         uint32_t level )
{
  node_rate_hz_ = (float) config.rate;

  estimator_.setRangeLimits(config.range_min, config.range_max);
  estimator_.setSensorMountHeightMin(config.mount_height_min);
  estimator_.setSensorMountHeightMax(config.mount_height_max);
  estimator_.setSensorTiltAngleMin(config.tilt_angle_min);
  estimator_.setSensorTiltAngleMax(config.tilt_angle_max);

  estimator_.setPublishDepthEnable(config.publish_depth);
  estimator_.setCamModelUpdate(config.cam_model_update);
  estimator_.setUsedDepthHeight((unsigned int)config.used_depth_height);
  estimator_.setDepthImgStepRow(config.depth_img_step_row);
  estimator_.setDepthImgStepCol(config.depth_img_step_col);

  estimator_.setRansacDistanceThresh(config.ransac_dist_thresh);
  estimator_.setRansacMaxIter(config.ransac_max_iter);
  estimator_.setGroundMaxPoints(config.ground_max_points);

  estimator_.setReconfParamsUpdated(true);
}
