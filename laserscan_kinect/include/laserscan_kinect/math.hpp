// Software License Agreement (BSD License)
//
// Copyright (c) 2016-2021, Michal Drwiega (drwiega.michal@gmail.com)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     1. Redistributions of source code must retain the above copyright
//        notice, this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived
//        from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef LASERSCAN_KINECT__MATH_HPP_
#define LASERSCAN_KINECT__MATH_HPP_

#include <image_geometry/pinhole_camera_model.h>

namespace laserscan_kinect
{

/**
* @brief lengthOfVector calculate the length of the 3D vector
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
