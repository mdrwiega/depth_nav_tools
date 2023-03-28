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

#ifndef CLIFF_DETECTOR__CLIFF_DETECTOR_HPP_
#define CLIFF_DETECTOR__CLIFF_DETECTOR_HPP_

#include <vector>

#include "sensor_msgs/msg/image.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "geometry_msgs/msg/polygon_stamped.hpp"

namespace cliff_detector
{

/**
 * @brief The CliffDetector class detect cliff based on depth image
 */
class CliffDetector
{
public:
  CliffDetector() = default;
  ~CliffDetector() = default;
  /**
   * @brief detectCliff detects descending stairs based on the information in a depth image
   *
   * This function detects descending stairs based on the information
   * in the depth encoded image. It requires the synchronized
   * Image/CameraInfo pair associated with the image.
   *
   * @param image UInt16 encoded depth image.
   * @param info CameraInfo associated with depth_msg
   */
  geometry_msgs::msg::PolygonStamped detectCliff(
    const sensor_msgs::msg::Image::ConstSharedPtr & image,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info);
  /**
   * @brief setMinRange sets depth sensor min range
   *
   * @param rmin Minimum sensor range (below it is death zone) in meters.
   */
  void setMinRange(const float rmin);
  /**
   * @brief setMaxRange sets depth sensor max range
   *
   * @param rmax Maximum sensor range in meters.
   */
  void setMaxRange(const float rmax);
  /**
   * @brief setSensorMountHeight sets the height of sensor mount (in meters) from ground
   *
   * @param height Value of sensor mount height (in meters).
   */
  void setSensorMountHeight(const float height);
  /**
   * @brief getSensorMountHeight gets sensor mount height
   *
   * @return Return sensor mount height in meters
   */
  float getSensorMountHeight() {return sensor_mount_height_;}
  /**
   * @brief setSensorTiltAngle sets the sensor tilt angle (in degrees)
   * @param angle
   */
  void setSensorTiltAngle(const float angle);
  /**
   * @brief getSensorTiltAngle gets sensor tilt angle in degrees
   *
   * @return Return sensor tilt angle in degrees
   */
  float getSensorTiltAngle() {return sensor_tilt_angle_;}
  /**
   * @brief setPublishDepthEnable
   * @param enable
   */
  void setPublishDepthEnable(const bool enable) {publish_depth_enable_ = enable;}
  /**
   * @brief getPublishDepthEnable
   * @return
   */
  bool getPublishDepthEnable() const {return publish_depth_enable_;}
  /**
   * @brief setCamModelUpdate
   */
  void setCamModelUpdate(const bool u) {cam_model_update_ = u;}
  /**
   * @brief setUsedDepthHeight
   * @param height
   */
  void setUsedDepthHeight(const unsigned height);
  /**
   * @brief setBlockSize sets size of square block (subimage) used in cliff detector
   *
   * @param size Size of square block in pixels
   */
  void setBlockSize(const int size);
  /**
   * @brief setBlockPointsThresh sets threshold value of pixels in block to set block as stairs
   *
   * @param thresh Value of threshold in pixels.
   */
  void setBlockPointsThresh(const int thresh);
  /**
   * @brief setDepthImgStepRow
   * @param step
   */
  void setDepthImgStepRow(const int step);
  /**
   * @brief setDepthImgStepCol
   * @param step
   */
  void setDepthImgStepCol(const int step);
  /**
   * @brief setGroundMargin sets the floor margin (in meters)
   * @param margin
   */
  void setGroundMargin(const float margin);
  /**
   * @brief setParametersConfigurated
   * @param u
   */
  void setParametersConfigurated(const bool u) {depth_sensor_params_update = u;}
  /**
   * @brief Get depth image for debug purposes
   */
  sensor_msgs::msg::Image getDebugDepthImage() const;

protected:
  /**
    * Computes the angle between two cv::Point3d
    *
    * Computes the angle of two cv::Point3d assumed to be vectors starting
    * at the origin (0,0,0). Uses the following equation:
    * angle = arccos(a*b/(|a||b|)) where a = ray1 and b = ray2.
    *
    * @param ray1 The first ray
    * @param ray2 The second ray
    * @return The angle between the two rays (in radians)
    */
  double angleBetweenRays(const cv::Point3d & ray1, const cv::Point3d & ray2) const;
  /**
    * Find 2D points relative to robots where stairs are detected
    *
    * This uses a method
    *
    * @param depth_msg The UInt16 encoded depth message.
    */
  template<typename T>
  void findCliffInDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);
  /**
   * @brief calcDeltaAngleForImgRows
   * @param vertical_fov
   */
  void calcDeltaAngleForImgRows(double vertical_fov);
  /**
    * @brief Calculates distances to ground for every row of depth image
    *
    * Calculates distances to ground for rows of depth image based on known height of sensor
    * mount and tilt angle. It assume that sensor height and tilt angle in relative to ground
    * is constant.
    *
    * Calculated values are placed in vector dist_to_ground_.
    */
  void calcGroundDistancesForImgRows(double vertical_fov);
  /**
   * @brief calcTiltCompensationFactorsForImgRows calculate factors used in tilt compensation
   */
  void calcTiltCompensationFactorsForImgRows();

private:
  float range_min_;  ///< Stores the current minimum range to use
  float range_max_;  ///< Stores the current maximum range to use
  float sensor_mount_height_;  ///< Height of sensor mount from ground
  float sensor_tilt_angle_;  ///< Sensor tilt angle (degrees)
  bool publish_depth_enable_;  ///< Determines if depth should be republished
  bool cam_model_update_;  ///< Determines if continuously cam model update required
  unsigned used_depth_height_;  ///< Used depth height from img bottom (px)
  unsigned block_size_;  //< Square block (subimage) size (px).
  unsigned block_points_thresh_;  ///< Threshold value of points in block to admit stairs
  unsigned depth_image_step_row_;  ///< Rows step in depth processing (px).
  unsigned depth_image_step_col_;  ///< Columns step in depth processing (px).
  float ground_margin_;  ///< Margin for ground points feature detector (m)

  bool depth_sensor_params_update = false;
  /// Class for managing sensor_msgs/CameraInfo messages
  image_geometry::PinholeCameraModel camera_model_;
  /// Calculated distances to ground for every row of depth image in mm
  std::vector<double> dist_to_ground_;
  std::vector<double> tilt_compensation_factor_;
  std::vector<double> delta_row_;

  sensor_msgs::msg::Image debug_depth_msg_;

  /// Store points which contain obstacle
  geometry_msgs::msg::PolygonStamped stairs_points_msg_;
  double horizontal_fov;
};

}  // namespace cliff_detector

#endif  // CLIFF_DETECTOR__CLIFF_DETECTOR_HPP_
