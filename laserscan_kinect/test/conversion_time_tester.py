#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2017, Michal Drwiega
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#####################################################################

import rospy
import numpy as np
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, LaserScan

class DepthImagePublisher(object):
  msg_counter = 0

  def __init__(self, depth_topic, info_topic):
    self.pub_img = rospy.Publisher(depth_topic, Image, queue_size=1)
    self.pub_info = rospy.Publisher(info_topic, CameraInfo, queue_size=1)

  def generateDepthImageAndInfo(self):
    self.msg_counter += 1
    width = 640
    height = 480
    img = np.empty((height, width), np.uint16)
    img.fill(1400)

    for i in range(width):
      for j in range(height / 2 - 1, height / 2 + 1):
        img[j][i] = 520

    depthimg = Image()
    depthimg.header.frame_id = 'depthmap_test'
    depthimg.header.seq = self.msg_counter
    depthimg.header.stamp = rospy.Time.now()
    depthimg.height = height
    depthimg.width = width
    depthimg.encoding = "16UC1"
    depthimg.step = depthimg.width * 2
    depthimg.data = img.tostring()

    info = CameraInfo()
    info.header = depthimg.header
    info.height = height
    info.width = width
    info.distortion_model = "plumb_bob"
    info.K[0] = 570; info.K[2] = 314; info.K[4] = 570
    info.K[5] = 239; info.K[8] = 1.0
    info.R[0] = 1.0; info.R[4] = 1.0; info.R[8] = 1.0
    info.P[0] = 570; info.P[2] = 314; info.P[5] = 570
    info.P[6] = 235; info.P[10] = 1.0
    return depthimg, info

  def publishDepthImageAndInfo(self):
    depth, info = self.generateDepthImageAndInfo()
    self.pub_img.publish(depth)
    self.pub_info.publish(info)
    t = rospy.Time.now()
    return depth.header.seq, depth.header.stamp


class LaserScanSubsciber(object):
  start_time = 0
  msg_seq = 0
  msg_counter = 0
  time_diff = 0
  received = False

  def __init__(self, scan_topic):
    self.sub_scan = rospy.Subscriber(scan_topic, LaserScan, self.callback)

  def setTimeAndSeq(self, time, msg_seq):
    self.start_time = time
    self.msg_seq = msg_seq

  def callback(self, data):
    self.msg_counter += 1
    self.received = True

#===============================================================================
if __name__ == '__main__':
  rospy.init_node('depthimage_conversion_time_tester')
  rospy.loginfo('Depthimage tester started')

  pub = DepthImagePublisher("/camera/depth/image_raw", "/camera/depth/camera_info")
  sub = LaserScanSubsciber("/scan")
  rospy.sleep(1)

  rate_hz = 20
  iter_max = 1000
  iter = 0
  tc = 0
  rate = rospy.Rate(rate_hz)
  while not rospy.is_shutdown() and iter < iter_max:
    start = rospy.Time.now()
    seq, time = pub.publishDepthImageAndInfo()
    sub.setTimeAndSeq(start, seq)

    r = rospy.Duration(nsecs=50)
    rec = False
    while (not sub.received) and (rospy.Time.now() - start).to_sec() < 0.05:
      rospy.sleep(r)

    if sub.received:
      sub.received = False
      iter += 1
      tc += ((rospy.Time.now() - start).to_nsec() / 10.0**3)

    rate.sleep()

  print 'Mean time:', format((tc / iter_max)/10.0**3, '.2f'), 'ms' \
        ' Iterations:', iter_max, ' Rate:', rate_hz, 'hz'\
        ' Published:', pub.msg_counter, ' Received:', sub.msg_counter