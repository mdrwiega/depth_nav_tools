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

#include <utility>
#include <list>
#include <vector>
#include <memory>

#include "depth_sensor_pose/depth_sensor_pose.hpp"

namespace depth_sensor_pose
{

void DepthSensorPose::estimateParams(
  const sensor_msgs::msg::Image::ConstSharedPtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
{
  // Update data based on depth sensor parameters only if new params values
  // or turned on continuous data calculations
  if (reconf_serv_params_updated_ || cam_model_update_) {
    camera_model_.fromCameraInfo(info);

    const double cx = camera_model_.cx();
    const double cy = camera_model_.cy();
    double vert_min, vert_max;

    // Calculate the vertical field of view of the depth sensor
    fieldOfView(vert_min, vert_max, cx, 0, cx, cy, cx, image->height - 1);
    const double vertical_fov = vert_max - vert_min;

    calcDeltaAngleForImgRows(vertical_fov);
    calcGroundDistancesForImgRows(mount_height_max_, tilt_angle_min_, dist_to_ground_max_);
    calcGroundDistancesForImgRows(mount_height_min_, tilt_angle_max_, dist_to_ground_min_);

    reconf_serv_params_updated_ = false;
  }

  sensorPoseCalibration(image, tilt_angle_est_, mount_height_est_);
}

void DepthSensorPose::setMinRange(const float rmin)
{
  if (rmin >= 0) {
    range_min_ = rmin;
  } else {
    range_min_ = 0;
    throw std::runtime_error(
            "Incorrect value of range minimal parameter. Set default value: 0.");
  }
}

void DepthSensorPose::setMaxRange(const float rmax)
{
  if (rmax >= 0) {
    range_max_ = rmax;
  } else {
    range_max_ = 10;
    throw std::runtime_error(
            "Incorrect value of range maximum parameter. Set default value: 10.");
  }
}

void DepthSensorPose::setSensorMountHeightMin(const float height)
{
  if (height > 0) {
    mount_height_min_ = height;
  } else {
    mount_height_min_ = 0;
    throw std::runtime_error(
            "Incorrect value of sensor mount height parameter. Set default value: 0.");
  }
}

void DepthSensorPose::setSensorMountHeightMax(const float height)
{
  if (height > 0) {
    mount_height_max_ = height;
  } else {
    mount_height_max_ = 1;
    throw std::runtime_error(
            "Incorrect value of sensor mount height parameter. Set default value: 1m.");
  }
}

void DepthSensorPose::setSensorTiltAngleMin(const float angle)
{
  if (angle < 90 && angle > -90) {
    tilt_angle_min_ = angle;
  } else {
    tilt_angle_min_ = 0;
    throw std::runtime_error(
            "Incorrect value of sensor tilt angle parameter. Set default value: 0.");
  }
}

void DepthSensorPose::setSensorTiltAngleMax(const float angle)
{
  if (angle < 90 && angle > -90) {
    tilt_angle_max_ = angle;
  } else {
    tilt_angle_max_ = 0;
    throw std::runtime_error(
            "Incorrect value of sensor tilt angle parameter. Set default value: 0.");
  }
}

void DepthSensorPose::setUsedDepthHeight(const unsigned height)
{
  if (height > 0) {
    used_depth_height_ = height;
  } else {
    used_depth_height_ = 200;
    throw std::runtime_error(
            "Incorrect value of used depth height parameter. Set default value: 200.");
  }
}

void DepthSensorPose::setDepthImgStepRow(const int step)
{
  if (step > 0) {
    depth_image_step_row_ = step;
  } else {
    depth_image_step_row_ = 1;
    throw std::runtime_error(
            "Incorrect value depth image row step parameter. Set default value: 1.");
  }
}

void DepthSensorPose::setDepthImgStepCol(const int step)
{
  if (step > 0) {
    depth_image_step_col_ = step;
  } else {
    depth_image_step_col_ = 1;
    throw std::runtime_error(
            "Incorrect value depth image column step parameter. Set default value: 1.");
  }
}

sensor_msgs::msg::Image::ConstSharedPtr DepthSensorPose::getDbgImage() const
{
  return dbg_image_;
}

double DepthSensorPose::lengthOfVector(const cv::Point3d & vec) const
{
  return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

double DepthSensorPose::angleBetweenRays(
  const cv::Point3d & ray1, const cv::Point3d & ray2) const
{
  double dot = ray1.x * ray2.x + ray1.y * ray2.y + ray1.z * ray2.z;

  return acos(dot / (lengthOfVector(ray1) * lengthOfVector(ray2)));
}

void DepthSensorPose::fieldOfView(
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

void DepthSensorPose::calcDeltaAngleForImgRows(double vertical_fov)
{
  const unsigned img_height = camera_model_.fullResolution().height;
  delta_row_.resize(img_height);

  // Angle between the sensor optical axis and ray to specific cell on the camera matrix
  for (unsigned i = 0; i < img_height; i++) {
    delta_row_[i] = vertical_fov * (i - camera_model_.cy() - 0.5) /
      (static_cast<double>(img_height) - 1);
  }
}

void DepthSensorPose::calcGroundDistancesForImgRows(
  double mount_height, double tilt_angle, std::vector<double> & distances)
{
  const double alpha = tilt_angle * M_PI / 180.0;  // Sensor tilt angle in radians
  const unsigned img_height = camera_model_.fullResolution().height;
  distances.resize(img_height);

  // Calculations for each row of image
  for (unsigned i = 0; i < img_height; i++) {
    if ((delta_row_[i] + alpha) > 0) {
      distances[i] = mount_height * sin(M_PI / 2 - delta_row_[i]) /
        cos(M_PI / 2 - delta_row_[i] - alpha);
    } else {
      distances[i] = 100;
    }
  }
}

template<typename T>
void DepthSensorPose::getGroundPoints(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
  std::list<std::pair<unsigned, unsigned>> & points_indices)
{
  enum Point { Row, Col, Depth };

  const unsigned img_height = depth_msg->height;
  const unsigned img_width = depth_msg->width;

  const T * data = reinterpret_cast<const T *>(&depth_msg->data[0]);

  // Set how many rows (from bottom of image) are used in ground finding
  unsigned processed_rows = depth_image_step_row_;
  if (used_depth_height_ < img_height && used_depth_height_ > 0) {
    processed_rows = img_height - used_depth_height_;
  }

  // Loop over each row in image from bottom of img
  for (unsigned row = img_height - 1; row > processed_rows; row -= depth_image_step_row_) {
    // Loop over each column in image
    for (unsigned col = 0; col < img_width; col += depth_image_step_col_) {
      float d = 0.0;

      if (typeid(T) == typeid(uint16_t)) {
        unsigned depth_raw_mm = static_cast<unsigned>(data[img_width * row + col]);
        d = static_cast<float>(depth_raw_mm) / 1000.0;
      } else if (typeid(T) == typeid(float)) {
        d = static_cast<float>(data[img_width * row + col]);
      }

      // Check if distance to point is greater than distance to ground plane
      if (points->size() < max_ground_points_ && d > range_min_ && d < range_max_ &&
        d > dist_to_ground_min_[row] && d < dist_to_ground_max_[row])
      {
        double z = d;
        double x = z * (static_cast<float>(row) - camera_model_.cx()) / camera_model_.fx();
        double y = z * (static_cast<float>(col) - camera_model_.cy()) / camera_model_.fy();

        points->push_back(pcl::PointXYZ(x, y, z));
        points_indices.push_back({row, col});
      }
    }
  }
}

void DepthSensorPose::sensorPoseCalibration(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  [[maybe_unused]] double & tilt_angle, [[maybe_unused]] double & height)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
  std::list<std::pair<unsigned, unsigned>> points_indices;

  // Check image encoding
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    // getGroundPoints<uint16_t>(depth_msg, ground_points, points_indices);
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    // getGroundPoints<float>(depth_msg, ground_points, points_indices);
  } else {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }

  // Generate and publish debug image if necessary
  if (publish_depth_enable_) {
    dbg_image_ = prepareDbgImage(depth_msg, points_indices);
  }

  if (ground_points->size() >= 3) {
    // Estimate model parameters with RANSAC
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
      model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(ground_points));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(ransacDistanceThresh_);
    ransac.setMaxIterations(ransacMaxIter_);
    ransac.computeModel();

    Eigen::VectorXf ground_coeffs(4);
    ransac.getModelCoefficients(ground_coeffs);

    // Calculate height and
    float a = ground_coeffs[0], b = ground_coeffs[1];
    float c = ground_coeffs[2], d = ground_coeffs[3];

    // Dot product two vectors v=[a,b,c], w=[1,1,0]
    tilt_angle = std::acos(
      (b * b + a * a) / (std::sqrt(b * b + a * a) *
      std::sqrt(a * a + b * b + c * c))) * 180.0 / M_PI;
    height = std::abs(d) / std::sqrt(a * a + b * b + c * c);
  }

  if (height < mount_height_min_ || height > mount_height_max_) {
    height = mount_height_min_;
  }

  if (tilt_angle < tilt_angle_min_ || tilt_angle > tilt_angle_max_) {
    tilt_angle = tilt_angle_min_;
  }
}

sensor_msgs::msg::Image::SharedPtr DepthSensorPose::prepareDbgImage(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const std::list<std::pair<unsigned, unsigned>> & ground_points_indices)
{
  auto img = std::make_shared<sensor_msgs::msg::Image>();
  img->header = depth_msg->header;
  img->height = depth_msg->height;
  img->width = depth_msg->width;
  img->encoding = "rgb8";  // image_encodings::RGB8
  img->is_bigendian = depth_msg->is_bigendian;
  img->step = img->width * 3;  // 3 bytes per pixel

  img->data.resize(img->step * img->height);
  uint8_t(*rgb_data)[3] = reinterpret_cast<uint8_t(*)[3]>(&img->data[0]);

  // Convert depth image to RGB
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    const uint16_t * depth_data = reinterpret_cast<const uint16_t *>(&depth_msg->data[0]);
    for (unsigned i = 0; i < (img->width * img->height); ++i) {
      // Scale value to cover full range of RGB 8
      uint8_t val = 255 * (depth_data[i] - range_min_ * 1000) /
        (range_max_ * 1000 - range_min_ * 1000);
      rgb_data[i][0] = 255 - val;
      rgb_data[i][1] = 255 - val;
      rgb_data[i][2] = 255 - val;
    }
  } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    const float * depth_data = reinterpret_cast<const float *>(&depth_msg->data[0]);
    for (unsigned i = 0; i < (img->width * img->height); ++i) {
      // Scale value to cover full range of RGB 8
      uint8_t val = 255 * (depth_data[i] - range_min_) / (range_max_ - range_min_);
      rgb_data[i][0] = 255 - val;
      rgb_data[i][1] = 255 - val;
      rgb_data[i][2] = 255 - val;
    }
  } else {
    throw std::runtime_error("Unsupported depth image encoding");
  }

  // Add ground points to debug image (as red points)
  for (const auto & pt : ground_points_indices) {
    const auto row = pt.first;
    const auto col = pt.second;
    rgb_data[row * img->width + col][0] = 255;
    rgb_data[row * img->width + col][1] = 0;
    rgb_data[row * img->width + col][2] = 0;
  }

  // Add line which is the border of the ground detection area
  std::list<std::pair<unsigned, unsigned>> pts;
  for (unsigned i = 0; i < img->width; ++i) {
    const int line_row = img->height - used_depth_height_;
    if (line_row >= 0 && line_row < static_cast<int>(img->height)) {
      pts.push_back({line_row, i});
    }
  }

  for (const auto & pt : pts) {
    const auto row = pt.first;
    const auto col = pt.second;
    rgb_data[row * img->width + col][0] = 0;
    rgb_data[row * img->width + col][1] = 255;
    rgb_data[row * img->width + col][2] = 0;
  }

  return img;
}

}  // namespace depth_sensor_pose
