// Copyright 2016-2021 Michał Drwięga (drwiega.michal@gmail.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <iostream>
#include <algorithm>

#include "gtest/gtest.h"

#include "sensor_msgs/image_encodings.hpp"

#include "laserscan_kinect/laserscan_kinect.hpp"

using std::chrono::high_resolution_clock;
using std::chrono::duration;

class LaserScanKinectTestable : public laserscan_kinect::LaserScanKinect
{
public:
  template<typename T>
  float getSmallestValueInColumn(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg, int col)
  {
    return laserscan_kinect::LaserScanKinect::getSmallestValueInColumn<T>(depth_msg, col);
  }
};

class LaserScanKinectTest : public ::testing::Test
{
public:
  sensor_msgs::msg::Image::SharedPtr depth_msg;
  sensor_msgs::msg::CameraInfo::SharedPtr info_msg;
  LaserScanKinectTestable converter;

  unsigned img_height {480};
  unsigned img_width {640};
  unsigned scan_height {420};

  LaserScanKinectTest()
  {
    defaultConverterConfiguration();
    setDefaultInfoMsg();
  }

  void defaultConverterConfiguration()
  {
    converter.setOutputFrame("kinect_frame");
    converter.setMinRange(0.5);
    converter.setMaxRange(5.0);
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

  void setDefaultInfoMsg()
  {
    info_msg.reset(new sensor_msgs::msg::CameraInfo);
    info_msg->header.frame_id = "depth_frame";
    info_msg->height = img_height;
    info_msg->width = img_width;
    info_msg->distortion_model = "plumb_bob";
    info_msg->d.resize(5);
    info_msg->k[0] = 570;
    info_msg->k[2] = 314;
    info_msg->k[4] = 570;
    info_msg->k[5] = 239;
    info_msg->k[8] = 1.0;
    info_msg->r[0] = 1.0;
    info_msg->r[4] = 1.0;
    info_msg->r[8] = 1.0;
    info_msg->p[0] = 570;
    info_msg->p[2] = 314;
    info_msg->p[5] = 570;
    info_msg->p[6] = 235;
    info_msg->p[10] = 1.0;
  }

  template<typename T>
  void setDefaultDepthMsg(T value)
  {
    depth_msg.reset(new sensor_msgs::msg::Image);
    depth_msg->header.frame_id = "depth_frame";
    depth_msg->height = img_height;
    depth_msg->width = img_width;
    depth_msg->is_bigendian = false;
    depth_msg->step = depth_msg->width * sizeof(T);

    if (typeid(T) == typeid(uint16_t)) {
      depth_msg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    } else if (typeid(T) == typeid(float)) {
      depth_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    }

    depth_msg->data.resize(depth_msg->width * depth_msg->height * sizeof(T));
    auto depth_row = reinterpret_cast<T *>(&depth_msg->data[0]);
    for (size_t i = 0; i < depth_msg->width * depth_msg->height; ++i) {
      depth_row[i] = value;
    }
  }

  template<typename T>
  void setDepthMsgWithTheSameSmallestValueInEachColumn(T low_value, T high_value)
  {
    setDefaultDepthMsg<T>(high_value);

    T * depth_row = reinterpret_cast<T *>(&depth_msg->data[0]);
    const int row_size = depth_msg->width;
    const int offset = static_cast<int>(info_msg->k[5] - scan_height / 2.0);

    // Change one pixel in each column to smaller value
    bool down = true;
    int row = std::max(0, offset);
    for (size_t col = 0; col < depth_msg->width; ++col) {
      depth_row[row_size * row + col] = low_value;
      if (down) {
        if ((row + 1) <= (offset + static_cast<int>(scan_height))) {
          row++;
        } else {
          down = false;
        }
      } else {
        if ((row - 1) >= offset) {
          row--;
        } else {
          down = true;
        }
      }
    }
  }

  // Set low, high or ground value
  template<typename T>
  void setDepthMsgWithTheSameSmallestValueAndGroundInEachColumn(
    T low_value, T high_value, T ground_value)
  {
    setDefaultDepthMsg<T>(high_value);

    T * depth_row = reinterpret_cast<T *>(&depth_msg->data[0]);
    const int row_size = depth_msg->width;

    // Change one pixel in each column to smaller value
    for (size_t col = 0; col < depth_msg->width; ++col) {
      unsigned row = img_height / 4;
      depth_row[row_size * row + col] = low_value;

      row = img_height / 2;
      depth_row[row_size * row + col] = ground_value;
    }
  }
};


TEST_F(LaserScanKinectTest, encodingSupport)
{
  setDefaultDepthMsg<uint16_t>(1);
  converter.getLaserScanMsg(depth_msg, info_msg);

  setDefaultDepthMsg<float>(1);
  converter.getLaserScanMsg(depth_msg, info_msg);
}

TEST_F(LaserScanKinectTest, unsupportedEncoding)
{
  setDefaultDepthMsg<uint16_t>(1);
  depth_msg->encoding = sensor_msgs::image_encodings::MONO16;
  EXPECT_ANY_THROW(converter.getLaserScanMsg(depth_msg, info_msg));
}

TEST_F(LaserScanKinectTest, getSmallestValueInColumn_U16_FeaturesOff)
{
  converter.setCamModelUpdate(true);
  uint16_t low = 0.5 * 1000, high = 2000;

  setDepthMsgWithTheSameSmallestValueInEachColumn<uint16_t>(low, high);

  EXPECT_EQ(
    static_cast<float>(low) / 1000,
    converter.getSmallestValueInColumn<uint16_t>(depth_msg, 1));
}

TEST_F(LaserScanKinectTest, getSmallestValueInColumn_F32_FeaturesOff)
{
  converter.setCamModelUpdate(true);
  float low = 0.5, high = 2;

  setDepthMsgWithTheSameSmallestValueInEachColumn<float>(low, high);

  EXPECT_EQ(low, converter.getSmallestValueInColumn<float>(depth_msg, 1));
}

TEST_F(LaserScanKinectTest, DISABLED_getSmallestValueInColumn_U16_GroundDetection)
{
  converter.setCamModelUpdate(true);
  converter.setGroundRemove(true);
  converter.setSensorMountHeight(1.0);
  converter.setSensorTiltAngle(45);
  setDefaultDepthMsg<uint16_t>(1);
  converter.getLaserScanMsg(depth_msg, info_msg);
  // Low value should be skipped (it's ground value)
  // z >= h * sin(pi/2 - delta)/cos(pi/2 - delta - alpha)
  // delta=0, alpha=45 => z >= sqrt(2) = 1.41
  uint16_t ground = 1600, low = 1700, high = 3000;

  setDepthMsgWithTheSameSmallestValueAndGroundInEachColumn(low, high, ground);

  EXPECT_EQ(
    static_cast<float>(low) / 1000, converter.getSmallestValueInColumn<uint16_t>(depth_msg, 0));
}

TEST_F(LaserScanKinectTest, DISABLED_getSmallestValueInColumn_F32_GroundDetection)
{
  converter.setCamModelUpdate(true);
  converter.setGroundRemove(true);
  converter.setSensorMountHeight(1.0);
  converter.setSensorTiltAngle(45);
  setDefaultDepthMsg<float>(1);
  converter.getLaserScanMsg(depth_msg, info_msg);
  float ground = 1.6, low = 1.7, high = 3;

  setDepthMsgWithTheSameSmallestValueAndGroundInEachColumn(low, high, ground);

  EXPECT_EQ(low, converter.getSmallestValueInColumn<float>(depth_msg, 0));
}

TEST_F(LaserScanKinectTest, minInEachColumn_U16_FeaturesDisabled)
{
  uint16_t low = 800, high = 3000;

  setDepthMsgWithTheSameSmallestValueInEachColumn<uint16_t>(low, high);

  auto scan_msg = converter.getLaserScanMsg(depth_msg, info_msg);
  size_t nan_counter = 0;

  // Check if in each column minimum value was selected
  for (const float range : scan_msg->ranges) {
    if (std::isnan(range)) {
      nan_counter++;
    }
    ASSERT_EQ(true, range <= low || std::isnan(range));
  }
  EXPECT_LE(nan_counter, static_cast<size_t>(scan_msg->ranges.size() / 10));
}

TEST_F(LaserScanKinectTest, minInEachColumn_F32_FeaturesDisabled)
{
  float low = 0.6, high = 4.5;

  setDepthMsgWithTheSameSmallestValueInEachColumn<float>(low, high);

  auto scan_msg = converter.getLaserScanMsg(depth_msg, info_msg);

  float tmp = info_msg->k[2] * high / info_msg->k[0];
  float max_depth = sqrt(tmp * tmp + high * high);

  // Check if in each column minimum value was selected
  for (const float & range : scan_msg->ranges) {
    ASSERT_EQ(true, range <= max_depth || std::isnan(range));
  }
}

TEST_F(LaserScanKinectTest, timeMeasurement_U16_FeaturesDisabled)
{
  uint16_t low = 800, high = 3000;

  setDepthMsgWithTheSameSmallestValueInEachColumn<uint16_t>(low, high);

  unsigned iter = 1000;
  duration<double> time{0};

  for (size_t i = 0; i < iter; ++i) {
    auto start = high_resolution_clock::now();

    auto scan_msg = converter.getLaserScanMsg(depth_msg, info_msg);
    time += (high_resolution_clock::now() - start);
  }
  std::cout << "Mean processing time: " <<
    duration<double, std::milli>(time).count() / iter << " ms.\n";
}

TEST_F(LaserScanKinectTest, timeMeasurement_U16_FeaturesEnabled)
{
  uint16_t low = 800, high = 3000;
  converter.setCamModelUpdate(true);
  converter.setGroundRemove(true);
  converter.setTiltCompensation(true);

  setDepthMsgWithTheSameSmallestValueInEachColumn<uint16_t>(low, high);

  unsigned iter = 1000;
  duration<double> time{0};

  for (size_t i = 0; i < iter; ++i) {
    auto start = high_resolution_clock::now();

    auto scan_msg = converter.getLaserScanMsg(depth_msg, info_msg);
    time += (high_resolution_clock::now() - start);
  }
  std::cout << "Mean processing time: " <<
    duration<double, std::milli>(time).count() / iter << " ms.\n";
}

TEST_F(LaserScanKinectTest, timeMeasurement_F32_FeaturesDisabled)
{
  float low = 0.6, high = 4.5;

  setDepthMsgWithTheSameSmallestValueInEachColumn<float>(low, high);

  unsigned iter = 1000;
  duration<double> time{0};

  for (size_t i = 0; i < iter; ++i) {
    auto start = high_resolution_clock::now();

    auto scan_msg = converter.getLaserScanMsg(depth_msg, info_msg);
    time += (high_resolution_clock::now() - start);
  }
  std::cout << "Mean processing time: " <<
    duration<double, std::milli>(time).count() / iter << " ms.\n";
}

TEST_F(LaserScanKinectTest, timeMeasurement_F32_FeaturesEnabled)
{
  float low = 0.6, high = 4.5;
  converter.setCamModelUpdate(true);
  converter.setGroundRemove(true);
  converter.setTiltCompensation(true);

  setDepthMsgWithTheSameSmallestValueInEachColumn<float>(low, high);

  unsigned iter = 1000;
  duration<double> time{0};

  for (size_t i = 0; i < iter; ++i) {
    auto start = high_resolution_clock::now();

    auto scan_msg = converter.getLaserScanMsg(depth_msg, info_msg);
    time += (high_resolution_clock::now() - start);
  }

  std::cout << "Mean processing time: " <<
    duration<double, std::milli>(time).count() / iter << " ms.\n";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
