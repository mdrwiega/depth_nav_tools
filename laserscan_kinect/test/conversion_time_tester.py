# Copyright 2016-2021 Michał Drwięga (drwiega.michal@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import rospy

from sensor_msgs.msg import CameraInfo, Image, LaserScan


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
        depthimg.encoding = '16UC1'
        depthimg.step = depthimg.width * 2
        depthimg.data = img.tostring()

        info = CameraInfo()
        info.header = depthimg.header
        info.height = height
        info.width = width
        info.distortion_model = 'plumb_bob'
        info.K[0] = 570
        info.K[2] = 314
        info.K[4] = 570
        info.K[5] = 239
        info.K[8] = 1.0
        info.R[0] = 1.0
        info.R[4] = 1.0
        info.R[8] = 1.0
        info.P[0] = 570
        info.P[2] = 314
        info.P[5] = 570
        info.P[6] = 235
        info.P[10] = 1.0
        return depthimg, info

    def publishDepthImageAndInfo(self):
        depth, info = self.generateDepthImageAndInfo()
        self.pub_img.publish(depth)
        self.pub_info.publish(info)
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


if __name__ == '__main__':
    rospy.init_node('depthimage_conversion_time_tester')
    rospy.loginfo('Depthimage tester started')

    pub = DepthImagePublisher('/camera/depth/image_raw', '/camera/depth/camera_info')
    sub = LaserScanSubsciber('/scan')
    rospy.sleep(1)

    rate_hz = 20
    iter_max = 1000
    iteration = 0
    tc = 0
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown() and iteration < iter_max:
        start = rospy.Time.now()
        seq, time = pub.publishDepthImageAndInfo()
        sub.setTimeAndSeq(start, seq)

        r = rospy.Duration(nsecs=50)
        rec = False
        while (not sub.received) and (rospy.Time.now() - start).to_sec() < 0.05:
            rospy.sleep(r)

        if sub.received:
            sub.received = False
            iteration += 1
            tc += ((rospy.Time.now() - start).to_nsec() / 10.0**3)

        rate.sleep()

    print('Mean time:', format((tc / iter_max)/10.0**3, '.2f'), 'ms'
          ' Iterations:', iter_max, ' Rate:', rate_hz, 'hz'
          ' Published:', pub.msg_counter, ' Received:', sub.msg_counter)
