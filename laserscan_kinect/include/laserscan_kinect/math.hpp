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

#ifndef LASERSCAN_KINECT__MATH_HPP_
#define LASERSCAN_KINECT__MATH_HPP_

#include <image_geometry/pinhole_camera_model.h>

namespace laserscan_kinect
{

inline double toRad(double alpha)
{
  return alpha * M_PI / 180.0;
}

/**
* @brief A length of the 3D vector
*/
inline double lengthOfVector(const cv::Point3d & vec)
{
  return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

/**
* @brief angleBetweenRays calculates angle between two rays in degrees
*/
inline double angleBetweenRays(const cv::Point3d & ray1, const cv::Point3d & ray2)
{
  const double dot = ray1.x * ray2.x + ray1.y * ray2.y + ray1.z * ray2.z;
  return acos(dot / (lengthOfVector(ray1) * lengthOfVector(ray2)));
}

/**
* @brief fieldOfView calculates field of view (angle)
*/
inline void calcFieldOfView(
  const image_geometry::PinholeCameraModel & camera_model,
  const cv::Point2d && left, const cv::Point2d && center,
  const cv::Point2d && right, double & min, double & max)
{
  const auto rect_pixel_left = camera_model.rectifyPoint(left);
  const auto left_ray = camera_model.projectPixelTo3dRay(rect_pixel_left);

  const auto rect_pixel_right = camera_model.rectifyPoint(right);
  const auto right_ray = camera_model.projectPixelTo3dRay(rect_pixel_right);

  const auto rect_pixel_center = camera_model.rectifyPoint(center);
  const auto center_ray = camera_model.projectPixelTo3dRay(rect_pixel_center);

  min = -angleBetweenRays(center_ray, right_ray);
  max = angleBetweenRays(left_ray, center_ray);
}

}  // namespace laserscan_kinect

#endif  // LASERSCAN_KINECT__MATH_HPP_
