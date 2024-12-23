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

#include <vector>

#include "cliff_detector/cliff_detector.hpp"

#include "sensor_msgs/image_encodings.hpp"

namespace cliff_detector
{

double toRad(double alpha)
{
  return alpha * M_PI / 180.0;
}

double squaredLength(const cv::Point3d & vec)
{
  return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
}

double length(const cv::Point3d & vec)
{
  return sqrt(squaredLength(vec));
}

geometry_msgs::msg::PolygonStamped CliffDetector::detectCliff(
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
{
  // Update data based on depth sensor parameters only if new params values
  // or turned on continuous data calculations
  if (!depth_sensor_params_update || cam_model_update_) {
    camera_model_.fromCameraInfo(info);

    double angle_min, angle_max, vertical_fov;
    const auto cx = camera_model_.cx();
    const auto cy = camera_model_.cy();

    // Calculate field of views angles - vertical and horizontal
    fieldOfView(angle_min, angle_max, cx, 0, cx, cy, cx, image->height - 1);
    vertical_fov = angle_max - angle_min;

    // Calculate angles between optical axis and rays for each row of image
    calcDeltaAngleForImgRows(vertical_fov);

    // Calculate ground distances for every row of depth image
    calcGroundDistancesForImgRows(vertical_fov);

    // Sensor tilt compensation
    calcTiltCompensationFactorsForImgRows();

    // Check scan_height vs image_height
    if (used_depth_height_ > image->height) {
      // ROS_ERROR("Parameter used_depth_height is higher than height of image.");
      used_depth_height_ = image->height;
    }
    depth_sensor_params_update = true;
  }

  // Check the image encoding
  if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    findCliffInDepthImage<uint16_t>(image);
  } else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    findCliffInDepthImage<float>(image);
  } else {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << image->encoding;
    throw std::runtime_error(ss.str());
  }

  return stairs_points_msg_;
}

void CliffDetector::setMinRange(const float rmin)
{
  if (rmin >= 0) {
    range_min_ = rmin;
  } else {
    range_min_ = 0;
    throw std::runtime_error("Incorrect value of min range parameter. Set default value: 0.");
  }
}

void CliffDetector::setMaxRange(const float rmax)
{
  if (rmax >= 0) {
    range_max_ = rmax;
  } else {
    range_max_ = 10;
    throw std::runtime_error("Incorrect value of max range parameter. Set default value: 10.");
  }
}

void CliffDetector::setSensorMountHeight(const float height)
{
  if (height > 0) {
    sensor_mount_height_ = height;
  } else {
    sensor_mount_height_ = 0;
    throw std::runtime_error(
            "Incorrect value of sensor mount height parameter. Set default value: 0.");
  }
}

void CliffDetector::setSensorTiltAngle(const float angle)
{
  if (angle < 90 && angle > -90) {
    sensor_tilt_angle_ = angle;
  } else {
    sensor_tilt_angle_ = 0;
    throw std::runtime_error(
            "Incorrect value of sensor tilt angle parameter. Set default value: 0.");
  }
}

void CliffDetector::setUsedDepthHeight(const unsigned height)
{
  if (height > 0) {
    used_depth_height_ = height;
  } else {
    used_depth_height_ = 100;
    throw std::runtime_error(
            "Incorrect value of used depth height parameter. Set default value: 100.");
  }
}

void CliffDetector::setBlockSize(const int size)
{
  if (size > 0 && (size % 2 == 0)) {
    block_size_ = size;
  } else {
    block_size_ = 8;
    throw std::runtime_error("Incorrect value of block size. Set default value: 8.");
  }
}

void CliffDetector::setBlockPointsThresh(const int thresh)
{
  if (thresh > 0) {
    block_points_thresh_ = thresh;
  } else {
    block_points_thresh_ = 1;
    throw std::runtime_error(
            "Incorrect value of block points threshold parameter. Set default value: 1.");
  }
}

void CliffDetector::setDepthImgStepRow(const int step)
{
  if (step > 0) {
    depth_image_step_row_ = step;
  } else {
    depth_image_step_row_ = 1;
    throw std::runtime_error(
            "Incorrect value depth image row step parameter. Set default value: 1.");
  }
}

void CliffDetector::setDepthImgStepCol(const int step)
{
  if (step > 0) {
    depth_image_step_col_ = step;
  } else {
    depth_image_step_col_ = 1;
    throw std::runtime_error(
            "Incorrect value depth image column step parameter. Set default value: 1.");
  }
}

void CliffDetector::setGroundMargin(const float margin)
{
  if (margin > 0) {
    ground_margin_ = margin;
  } else {
    ground_margin_ = 0;
    throw std::runtime_error("Incorrect value of ground margin parameter. Set default value: 0.");
  }
}

double CliffDetector::angleBetweenRays(
  const cv::Point3d & ray1, const cv::Point3d & ray2) const
{
  double dot = ray1.x * ray2.x + ray1.y * ray2.y + ray1.z * ray2.z;
  return acos(dot / (length(ray1) * length(ray2)));
}

void CliffDetector::fieldOfView(
  double & min, double & max, double x1, double y1,
  double xc, double yc, double x2, double y2)
{
  const cv::Point2d raw_pixel_left(x1, y1);
  const auto rect_pixel_left = camera_model_.rectifyPoint(raw_pixel_left);
  const auto left_ray = camera_model_.projectPixelTo3dRay(rect_pixel_left);

  const cv::Point2d raw_pixel_right(x2, y2);
  const auto rect_pixel_right = camera_model_.rectifyPoint(raw_pixel_right);
  const auto right_ray = camera_model_.projectPixelTo3dRay(rect_pixel_right);

  const cv::Point2d raw_pixel_center(xc, yc);
  const auto rect_pixel_center = camera_model_.rectifyPoint(raw_pixel_center);
  const auto center_ray = camera_model_.projectPixelTo3dRay(rect_pixel_center);

  min = -angleBetweenRays(center_ray, right_ray);
  max = angleBetweenRays(left_ray, center_ray);
}

void CliffDetector::calcDeltaAngleForImgRows(double vertical_fov)
{
  const unsigned img_height = camera_model_.fullResolution().height;

  delta_row_.resize(img_height);

  // Angle between ray and optical center
  for (unsigned i = 0; i < img_height; i++) {
    delta_row_[i] = vertical_fov * (i - camera_model_.cy() - 0.5) /
      (static_cast<double>(img_height) - 1);
  }
}

void CliffDetector::calcGroundDistancesForImgRows([[maybe_unused]] double vertical_fov)
{
  const double alpha = toRad(sensor_tilt_angle_);
  const unsigned img_height = camera_model_.fullResolution().height;

  dist_to_ground_.resize(img_height);

  // Calculations for each row of image
  for (unsigned i = 0; i < img_height; i++) {
    // Angle between ray and optical center
    if ((delta_row_[i] + alpha) > 0) {
      dist_to_ground_[i] = sensor_mount_height_ * sin(M_PI / 2 - delta_row_[i]) /
        cos(M_PI / 2 - delta_row_[i] - alpha);
    } else {
      dist_to_ground_[i] = 100.0;
    }
  }
}

void CliffDetector::calcTiltCompensationFactorsForImgRows()
{
  const double alpha = toRad(sensor_tilt_angle_);
  const unsigned img_height = camera_model_.fullResolution().height;

  tilt_compensation_factor_.resize(img_height);

  for (unsigned i = 0; i < img_height; i++) {  // Process all rows
    tilt_compensation_factor_[i] = sin(M_PI / 2 - delta_row_[i] - alpha) /
      sin(M_PI / 2 - delta_row_[i]);
  }
}

sensor_msgs::msg::Image CliffDetector::getDebugDepthImage() const
{
  return debug_depth_msg_;
}

template<typename T>
void CliffDetector::findCliffInDepthImage(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{
  enum Point { Row, Col, Depth };

  const T * data = reinterpret_cast<const T *>(&depth_msg->data[0]);
  const unsigned row_size = depth_msg->width;
  const unsigned img_height = camera_model_.fullResolution().height;
  const unsigned img_width = camera_model_.fullResolution().width;

  if ((block_size_ % 2) != 0) {
    // ROS_ERROR("Block size should be even number. Value will be decreased by one.");
    block_size_--;
  }

  const unsigned block_cols_nr = img_width / block_size_;
  const unsigned block_rows_nr = used_depth_height_ / block_size_;

  // Check if points thresh isn't too big
  if (block_points_thresh_ >= (block_size_ * block_size_ / depth_image_step_col_ /
    depth_image_step_row_))
  {
    // ROS_ERROR("Block points threshold is too big. Maximum admissible value will be set.");
    block_points_thresh_ = block_size_ * block_size_ / depth_image_step_col_ /
      depth_image_step_row_;
  }

  // Vector for block, constains rows, cols, depth values
  std::vector<std::vector<int>> tpoints(block_size_ * block_size_, std::vector<int>(3));

  // Rows, cols and depth values for points which apply to stairs
  std::vector<std::vector<int>> stairs_points;

  // Indicates which point from tpoints vector corresponds to which pixel in block
  std::vector<int> px_nr;
  px_nr.resize(block_size_ * block_size_);

  // Four pixels in center of block
  const unsigned c = block_size_ / 2;
  std::vector<unsigned> center_points(4);
  center_points[0] = c * block_size_ + c - 1;
  center_points[1] = c * block_size_ + c;
  center_points[2] = (c - 1) * block_size_ + c - 1;
  center_points[3] = (c - 1) * block_size_ + c;

  // Loop over each block row in image, bj - block column
  for (unsigned bj = 0; bj < block_rows_nr; ++bj) {
    // Loop over each block column in image, bi - block row
    for (unsigned bi = 0; bi < block_cols_nr; ++bi) {
      // Block processing
      unsigned block_cnt = 0;
      std::fill(px_nr.begin(), px_nr.end(), -1);

      // Loop over each row in block, j - column
      for (unsigned j = 0; j < block_size_; j += depth_image_step_row_) {
        // Loop over each column in block, i - row
        for (unsigned i = 0; i < block_size_; i += depth_image_step_col_) {
          // Rows from bottom of image
          unsigned row = (img_height - 1 ) - (bj * block_size_ + j);
          unsigned col = bi * block_size_ + i;

          float d = 0.0;

          if (typeid(T) == typeid(uint16_t)) {
            unsigned depth_raw_mm = static_cast<unsigned>(data[row_size * row + col]);
            d = static_cast<float>(depth_raw_mm) / 1000.0;
          } else if (typeid(T) == typeid(float)) {
            d = static_cast<float>(data[row_size * row + col]);
          }

          // Check if distance to point is greater than distance to ground plane
          if (d > (dist_to_ground_[row] + ground_margin_) && d > range_min_ && d < range_max_) {
            tpoints[block_cnt][Row] = row;
            tpoints[block_cnt][Col] = col;
            tpoints[block_cnt][Depth] = d;
            px_nr[j * block_size_ + i] = block_cnt;
            block_cnt++;
          }
        }
      }

      // Check if number of stairs points in block exceeded threshold value
      if (block_cnt >= block_points_thresh_) {
        // Block size is even. So first we check four pixels in center of block.
        if (px_nr[center_points[0]] > 0) {
          stairs_points.push_back(tpoints[px_nr[center_points[0]]]);
        } else if (px_nr[center_points[1]] > 0) {
          stairs_points.push_back(tpoints[px_nr[center_points[1]]]);
        } else if (px_nr[center_points[2]] > 0) {
          stairs_points.push_back(tpoints[px_nr[center_points[2]]]);
        } else if (px_nr[center_points[3]] > 0) {
          stairs_points.push_back(tpoints[px_nr[center_points[3]]]);
        } else {  // Otherwise add all points from block to stairs points vector
          stairs_points.insert(stairs_points.end(), tpoints.begin(), tpoints.begin() + block_cnt);
        }
      }
    }
  }

  if (publish_depth_enable_) {
    debug_depth_msg_ = *depth_msg;
  }

  T * new_depth_row = reinterpret_cast<T *>(&debug_depth_msg_.data[0]);

  // Set header and size of points list in message
  geometry_msgs::msg::Point32 pt;
  stairs_points_msg_.header = depth_msg->header;
  stairs_points_msg_.polygon.points.clear();
  pt.y = 0;

  const double sensor_tilt = toRad(sensor_tilt_angle_);

  std::vector<std::vector<int>>::iterator it;
  for (it = stairs_points.begin(); it != stairs_points.end(); ++it) {
    // Calculate point in XZ plane -- depth (z)
    unsigned row = (*it)[Row];
    pt.z = sensor_mount_height_ / std::tan(sensor_tilt + delta_row_[row]);

    // Calculate x value
    const double depth = sensor_mount_height_ / std::sin(sensor_tilt + delta_row_[row]);
    pt.x = ((*it)[Col] - camera_model_.cx()) * depth / camera_model_.fx();

    // Add point to message
    stairs_points_msg_.polygon.points.push_back(pt);

    if (publish_depth_enable_) {
      new_depth_row[row_size * (*it)[Row] + (*it)[Col]] = 10000U;
    }
  }
}

template void CliffDetector::findCliffInDepthImage<uint16_t>(
  const sensor_msgs::msg::Image::ConstSharedPtr &);
template void CliffDetector::findCliffInDepthImage<float>(
  const sensor_msgs::msg::Image::ConstSharedPtr &);

}  // namespace cliff_detector
