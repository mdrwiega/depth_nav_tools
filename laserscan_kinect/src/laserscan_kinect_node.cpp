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
 * @file   laserscan_kinect_node.cpp
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @brief  laserscan_kinect package
 */

#include <laserscan_kinect/laserscan_kinect_node.h>
#include <boost/bind.hpp>

namespace laserscan_kinect {

//=================================================================================================
LaserScanKinectNode::LaserScanKinectNode(ros::NodeHandle& n, ros::NodeHandle& pnh) :
    pnh_(pnh), it_(n), srv_(pnh)
{
    std::lock_guard<std::mutex> lock(connect_mutex_);

    // Dynamic reconfigure server callback
    srv_.setCallback(boost::bind(&LaserScanKinectNode::reconfigureCb, this, _1, _2));

    // Subscription to depth image topic
    pub_ = n.advertise<sensor_msgs::LaserScan>("scan", 10,
                                               boost::bind(&LaserScanKinectNode::connectCb, this, _1),
                                               boost::bind(&LaserScanKinectNode::disconnectCb, this, _1));
}

//=================================================================================================
LaserScanKinectNode::~LaserScanKinectNode()
{
    sub_.shutdown();
}

//=================================================================================================
void LaserScanKinectNode::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    try
    {
        sensor_msgs::LaserScanPtr laserscan_msg = converter_.getLaserScanMsg(depth_msg, info_msg);
        pub_.publish(laserscan_msg);
    }
    catch (std::runtime_error& e)
    {
        ROS_ERROR_THROTTLE(1.0, "Could not convert depth image to laserscan: %s", e.what());
    }
}

//=================================================================================================
void LaserScanKinectNode::connectCb(const ros::SingleSubscriberPublisher& pub)
{
    std::lock_guard<std::mutex> lock(connect_mutex_);

    if (!sub_ && pub_.getNumSubscribers() > 0)
    {
        ROS_DEBUG("Connecting to depth topic.");
        image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
        sub_ = it_.subscribeCamera("image", 10, &LaserScanKinectNode::depthCb, this, hints);
    }
}

//=================================================================================================
void LaserScanKinectNode::disconnectCb(const ros::SingleSubscriberPublisher& pub)
{
    std::lock_guard<std::mutex> lock(connect_mutex_);

    if (pub_.getNumSubscribers() == 0)
    {
        ROS_DEBUG("Unsubscribing from depth topic.");
        sub_.shutdown();
    }
}

//=================================================================================================
void LaserScanKinectNode::reconfigureCb(laserscan_kinect::LaserscanKinectConfig& config,
                                        uint32_t level)
{
    converter_.setOutputFrame(config.output_frame_id);
    converter_.setRangeLimits(config.range_min, config.range_max);
    converter_.setScanHeight(config.scan_height);
    converter_.setDepthImgRowStep(config.depth_img_row_step);
    converter_.setCamModelUpdate(config.cam_model_update);

    converter_.setSensorMountHeight(config.sensor_mount_height);
    converter_.setSensorTiltAngle(config.sensor_tilt_angle);
    converter_.setGroundRemove(config.ground_remove_en);
    converter_.setGroundMargin(config.ground_margin);
    converter_.setTiltCompensation(config.tilt_compensation_en);

    converter_.setScanConfigurated(false);
}

//=================================================================================================
} // end of namespace
