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
 * @file   main.cpp
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @date   11.2015
 * @brief  depth_sensor_pose package
 */

#include <depth_sensor_pose/depth_sensor_pose_node.h>
#include <nodelet/nodelet.h>


namespace depth_sensor_pose
{

class DepthSensorPoseNodelet : public nodelet::Nodelet
{
public:
  DepthSensorPoseNodelet()  {};

  ~DepthSensorPoseNodelet() {}

private:
  virtual void onInit()
  {
    calibrator.reset(new DepthSensorPoseNode(getNodeHandle(), getPrivateNodeHandle()));
  };
  
  boost::shared_ptr<DepthSensorPoseNode> calibrator;
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(depth_sensor_pose, DepthSensorPoseNodelet,depth_sensor_pose::DepthSensorPoseNodelet, nodelet::Nodelet);

