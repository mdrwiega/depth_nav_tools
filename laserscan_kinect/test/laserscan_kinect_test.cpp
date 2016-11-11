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
 * @file   laserscan_kinect_test.cpp
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 */

#include <laserscan_kinect/laserscan_kinect.h>

#include <gtest/gtest.h>

class LaserScanKinectTest : public ::testing::Test
{
public:
    sensor_msgs::ImagePtr depth_msg;
    sensor_msgs::CameraInfoPtr info_msg;
    laserscan_kinect::LaserScanKinect converter;

    unsigned img_height { 480 };
    unsigned img_width { 640 };
    unsigned scan_height { 400 };

    LaserScanKinectTest() {
        defaultConverterConfiguration();
        setDefaultInfoMsg();
    }

    void defaultConverterConfiguration() {
        converter.setOutputFrame("kinect_frame");
        converter.setRangeLimits(0.5, 5.0);
        converter.setScanHeight(scan_height);
        converter.setDepthImgRowStep(1);
        converter.setCamModelUpdate(false);

        converter.setSensorMountHeight(0.4);
        converter.setSensorTiltAngle(5.0);
        converter.setGroundRemove(false);
        converter.setGroundMargin(0.05);
        converter.setTiltCompensation(false);
        converter.setScanConfigurated(false);
    }

    void setDefaultInfoMsg() {
        info_msg.reset(new sensor_msgs::CameraInfo);
        info_msg->header.frame_id = "depth_frame";
        info_msg->header.seq = 100;
        info_msg->height = img_height;
        info_msg->width = img_width;
        info_msg->distortion_model = "plumb_bob";
        info_msg->D.resize(5);
        info_msg->K[0] = 570;
        info_msg->K[2] = 314;
        info_msg->K[4] = 570;
        info_msg->K[5] = 239;
        info_msg->K[8] = 1.0;
        info_msg->R[0] = 1.0;
        info_msg->R[4] = 1.0;
        info_msg->R[8] = 1.0;
        info_msg->P[0] = 570;
        info_msg->P[2] = 314;
        info_msg->P[5] = 570;
        info_msg->P[6] = 235;
        info_msg->P[10] = 1.0;
    }

    template<typename T>
    void setDefaultDepthMsg(T value) {
        depth_msg.reset(new sensor_msgs::Image);
        depth_msg->header.frame_id = "depth_frame";
        depth_msg->header.seq = 100;
        depth_msg->header.stamp.fromSec(10.0);
        depth_msg->height = img_height;
        depth_msg->width = img_width;
        depth_msg->is_bigendian = false;
        depth_msg->step = depth_msg->width * sizeof(T);

        if (typeid(T) == typeid(uint16_t))
            depth_msg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        else if (typeid(T) == typeid(float))
            depth_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

        depth_msg->data.resize(
                depth_msg->width * depth_msg->height * sizeof(T));
        T* depth_row = reinterpret_cast<T*>(&depth_msg->data[0]);
        for (size_t i = 0; i < depth_msg->width * depth_msg->height; ++i) {
            depth_row[i] = value;
        }
    }

    template<typename T>
    void setDepthMsgWithTheSameSmallestValueInEachColumn(T low_value,
            T high_value) {
        setDefaultDepthMsg<T>(high_value);

        T* depth_row = reinterpret_cast<T*>(&depth_msg->data[0]);
        const unsigned row_size = depth_msg->width;
        const int offset = (int) (info_msg->K[5] - scan_height / 2);

        // Change one pixel in each column to smaller value
        bool down = true;
        size_t row = std::max(0, offset);
        for (size_t col = 0; col < depth_msg->width; ++col) {
            depth_row[row_size * row + col] = low_value;
            if (down) {
                if ((row + 1) <= (offset + scan_height))
                    row++;
                else
                    down = false;
            } else {
                if ((row - 1) >= offset)
                    row--;
                else
                    down = true;
            }
        }
    }
};

TEST_F(LaserScanKinectTest, encodingSupport)
{
    setDefaultDepthMsg<uint16_t>(1);
    converter.getLaserScanMsg(depth_msg, info_msg);

    setDefaultDepthMsg<float>(1);
    converter.getLaserScanMsg(depth_msg, info_msg);

    depth_msg->encoding = sensor_msgs::image_encodings::MONO16;
    EXPECT_ANY_THROW(converter.getLaserScanMsg(depth_msg, info_msg));
}

TEST_F(LaserScanKinectTest, minInEachColumn_U16_FeaturesDisabled)
{
    uint16_t low = 800, high = 3000;

    setDepthMsgWithTheSameSmallestValueInEachColumn<uint16_t>(low, high);

    sensor_msgs::LaserScanPtr scan_msg = converter.getLaserScanMsg(depth_msg,
            info_msg);

    // Check if in each column minimum value was selected
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        ASSERT_EQ(true,
                scan_msg->ranges[i] <= low + 1
                        || std::isnan(scan_msg->ranges[i]));
    }
}

TEST_F(LaserScanKinectTest, minInEachColumn_F32_FeaturesDisabled)
{
    float low = 0.6, high = 4.5;

    setDepthMsgWithTheSameSmallestValueInEachColumn<float>(low, high);

    sensor_msgs::LaserScanPtr scan_msg = converter.getLaserScanMsg(depth_msg,
            info_msg);

    float tmp = info_msg->K[2] * high / info_msg->K[0];
    float max_depth = sqrt(tmp * tmp + high * high);

    // Check if in each column minimum value was selected
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        ASSERT_EQ(true,
                scan_msg->ranges[i] <= max_depth
                        || std::isnan(scan_msg->ranges[i]));
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
