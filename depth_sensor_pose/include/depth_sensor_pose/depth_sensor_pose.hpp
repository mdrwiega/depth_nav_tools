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

#ifndef DEPTH_SENSOR_POSE__DEPTH_SENSOR_POSE_HPP_
#define DEPTH_SENSOR_POSE__DEPTH_SENSOR_POSE_HPP_

#include <list>
#include <vector>
#include <cmath>
#include <utility>

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "image_geometry/pinhole_camera_model.h"

#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/segmentation/sac_segmentation.h"

namespace depth_sensor_pose
{

class DepthSensorPose
{
public:
  DepthSensorPose() = default;
  ~DepthSensorPose() = default;

  DepthSensorPose(const DepthSensorPose &) = delete;
  DepthSensorPose & operator=(const DepthSensorPose &) = delete;

  /**
    * Estimates depth sensor height and tilt angle
    *
    * @param image UInt16 or Float32 encoded depth image.
    * @param info CameraInfo associated with the image
    */
  void estimateParams(
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
  void setSensorMountHeightMin(const float height);
  /**
   * @brief setSensorMountHeight sets the height of sensor mount (in meters) from ground
   *
   * @param height Value of sensor mount height (in meters).
   */
  void setSensorMountHeightMax(const float height);
  /**
   * @brief setSensorTiltAngle sets the sensor tilt angle (in degrees)
   * @param angle
   */
  void setSensorTiltAngleMin(const float angle);
  /**
   * @brief setSensorTiltAngle sets the sensor tilt angle (in degrees)
   * @param angle
   */
  void setSensorTiltAngleMax(const float angle);
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
   * @param u
   */
  void setCamModelUpdate(const bool u) {cam_model_update_ = u;}
  /**
   * @brief setUsedDepthHeight
   * @param height
   */
  void setUsedDepthHeight(const unsigned height);
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
   * @brief setReconfParamsUpdated
   * @param updated
   */
  void setReconfParamsUpdated(bool updated) {reconf_serv_params_updated_ = updated;}

  void setRansacMaxIter(const unsigned u) {ransacMaxIter_ = u;}

  void setRansacDistanceThresh(const float u) {ransacDistanceThresh_ = u;}

  void setGroundMaxPoints(const unsigned u) {max_ground_points_ = u;}

  float getSensorTiltAngle() const {return tilt_angle_est_;}

  float getSensorMountHeight() const {return mount_height_est_;}

  sensor_msgs::msg::Image::ConstSharedPtr getDbgImage() const;

protected:
  /**
     * Computes euclidean length of a cv::Point3d (as a ray from origin)
     *
     * This function computes the length of a cv::Point3d assumed to be a vector starting at the origin (0,0,0).
     *
     * @param ray The ray for which the magnitude is desired.
     * @return Returns the magnitude of the ray.
     *
     */
  double lengthOfVector(const cv::Point3d & vec) const;
  /**
     * Computes the angle between two cv::Point3d
     *
     * Computes the angle of two cv::Point3d assumed to be vectors starting at the origin (0,0,0).
     * Uses the following equation: angle = arccos(a*b/(|a||b|)) where a = ray1 and b = ray2.
     *
     * @param ray1 The first ray
     * @param ray2 The second ray
     * @return The angle between the two rays (in radians)
     */
  double angleBetweenRays(const cv::Point3d & ray1, const cv::Point3d & ray2) const;
  /**
  * Calculate vertical angle_min and angle_max by measuring angles between the top
  * ray, bottom ray, and optical center ray
  *
  * @param camModel The image_geometry camera model for this image.
  * @param min_angle The minimum vertical angle
  * @param max_angle The maximum vertical angle
  */
  void fieldOfView(
    double & min, double & max, double x1, double y1,
    double xc, double yc, double x2, double y2);

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
  void calcGroundDistancesForImgRows(
    double mount_height, double tilt_angle, std::vector<double> & distances);

  template<typename T>
  void getGroundPoints(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & points,
    std::list<std::pair<unsigned, unsigned>> & points_indices);

  void sensorPoseCalibration(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    double & tilt_angle, double & height);

  sensor_msgs::msg::Image::SharedPtr prepareDbgImage(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const std::list<std::pair<unsigned, unsigned>> & ground_points_indices);

private:
  float range_min_{0};                ///< Stores the current minimum range to use
  float range_max_{0};                ///< Stores the current maximum range to use
  float mount_height_min_{0};         ///< Min height of sensor mount from ground
  float mount_height_max_{0};         ///< Max height of sensor mount from ground
  float tilt_angle_min_{0};           ///< Min angle of sensor tilt in degrees
  float tilt_angle_max_{0};           ///< Max angle of sensor tilt in degrees

  bool publish_depth_enable_{false};  ///< Determines if debug image should be published
  bool cam_model_update_{false};      ///< Determines if continuously cam model update is required
  unsigned used_depth_height_{0};        ///< Used depth height from img bottom (px)
  unsigned depth_image_step_row_{0};     ///< Rows step in depth processing (px).
  unsigned depth_image_step_col_{0};     ///< Columns step in depth processing (px).

  unsigned max_ground_points_{0};
  unsigned ransacMaxIter_{0};
  float ransacDistanceThresh_{0};
  float groundDistTolerance_{0};

  ///< Class for managing sensor_msgs/CameraInfo messages
  image_geometry::PinholeCameraModel camera_model_;

  sensor_msgs::msg::Image::SharedPtr dbg_image_;

  double mount_height_est_{0};
  double tilt_angle_est_{0};
  bool reconf_serv_params_updated_{true};

  std::vector<double> delta_row_;
  std::vector<double> dist_to_ground_max_;
  std::vector<double> dist_to_ground_min_;
};

}  // namespace depth_sensor_pose

#endif  // DEPTH_SENSOR_POSE__DEPTH_SENSOR_POSE_HPP_
