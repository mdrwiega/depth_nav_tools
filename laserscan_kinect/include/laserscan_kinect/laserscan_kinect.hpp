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

#ifndef LASERSCAN_KINECT__LASERSCAN_KINECT_HPP_
#define LASERSCAN_KINECT__LASERSCAN_KINECT_HPP_

#include <vector>
#include <string>
#include <mutex>
#include <list>
#include <utility>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "image_geometry/pinhole_camera_model.h"

namespace laserscan_kinect
{

class LaserScanKinect
{
public:
  LaserScanKinect()
  : scan_msg_(new sensor_msgs::msg::LaserScan())
  {
  }

  ~LaserScanKinect() = default;

  /**
   * @brief prepareLaserScanMsg converts depthimage and prepare new LaserScan message
   *
   * @param depth_msg Message that contains depth image which will be converted to LaserScan.
   * @param info_msg Message which contains depth sensor parameters.
   *
   * @return Return pointer to LaserScan message.
   */
  sensor_msgs::msg::LaserScan::SharedPtr getLaserScanMsg(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);
  /**
   * @brief setOutputFrame sets the frame to output laser scan
   * @param frame
   */
  void setOutputFrame(const std::string & frame) {output_frame_id_ = frame;}
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
   * @brief setScanHeight sets height of depth image which will be used in conversion process
   *
   * @param scan_height Height of used part of depth image in pixels.
   */
  void setScanHeight(const int scan_height);
  /**
   * @brief setDepthImgRowStep
   *
   * @param row_step
   */
  void setDepthImgRowStep(const int row_step);
  /**
   * @brief setCamModelUpdate sets the camera parameters
   *
   * @param enable
   */
  void setCamModelUpdate(const bool enable) {cam_model_update_ = enable;}
  /**
   * @brief setSensorMountHeight sets the height of sensor mount (in meters)
   */
  void setSensorMountHeight(const float height);
  /**
   * @brief setSensorTiltAngle sets the sensor tilt angle (in degrees)
   *
   * @param angle
   */
  void setSensorTiltAngle(const float angle);
  /**
   * @brief setGroundRemove enables or disables the feature which remove ground from scan
   *
   * @param enable
   */
  void setGroundRemove(const bool enable) {ground_remove_enable_ = enable;}
  /**
   * @brief setGroundMargin sets the floor margin (in meters)
   *
   * @param margin
   */
  void setGroundMargin(const float margin);
  /**
   * @brief setTiltCompensation enables or disables the feature which compensates sensor tilt
   *
   * @param enable
   */
  void setTiltCompensation(const bool enable) {tilt_compensation_enable_ = enable;}
  /**
    * @brief setScanConfigurated sets the configuration status
    *
    * @param enable
    */
  void setScanConfigurated(const bool configured) {is_scan_msg_configured_ = configured;}
  /**
   * @brief setPublishDbgImgEnable
   * @param enable
   */
  void setPublishDbgImgEnable(const bool enable) {publish_dbg_image_ = enable;}

  void setThreadsNum(unsigned threads_num);

  bool getPublishDbgImgEnable() const;

  sensor_msgs::msg::Image::SharedPtr getDbgImage() const;

protected:
  /**
  * @brief calcGroundDistancesForImgRows calculate coefficients used in ground removing from scan
  *
  * @param vertical_fov
  */
  void calcGroundDistancesForImgRows(double vertical_fov);
  /**
  * @brief calcTiltCompensationFactorsForImgRows calculate factors used in tilt compensation
  *
  * @param vertical_fov
  */
  void calcTiltCompensationFactorsForImgRows(double vertical_fov);
  /**
  * @brief calcScanMsgIndexForImgCols
  *
  * @param depth_msg
  */
  void calcScanMsgIndexForImgCols(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);
  /**
  * @brief getSmallestValueInColumn finds smallest values in depth image columns
    */
  template<typename T>
  float getSmallestValueInColumn(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg, int col);
  /**
  * @brief convertDepthToPolarCoords converts depth map to 2D
  */
  template<typename T>
  void convertDepthToPolarCoords(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

  sensor_msgs::msg::Image::SharedPtr prepareDbgImage(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const std::list<std::pair<int, int>> & min_dist_points_indices);

private:
  // ROS parameters configurated with configuration file or dynamic_reconfigure
  std::string output_frame_id_;           ///< Output frame_id for laserscan message.
  float range_min_{0};                    ///< Stores the current minimum range to use
  float range_max_{0};                    ///< Stores the current maximum range to use
  unsigned scan_height_{0};               ///< Number of pixel rows used to scan computing
  unsigned depth_img_row_step_{0};        ///< Row step in depth map processing
  bool cam_model_update_{false};         ///< If continously calibration update required
  float sensor_mount_height_{0};          ///< Height of sensor mount from ground
  float sensor_tilt_angle_{0};            ///< Angle of sensor tilt
  bool ground_remove_enable_{false};     ///< Determines if remove ground from output scan
  float ground_margin_{0};                ///< Margin for floor remove feature (in meters)
  bool tilt_compensation_enable_{false};  ///< Determines if tilt compensation feature is on
  bool publish_dbg_image_{false};        ///< Determines if debug image should be published
  unsigned threads_num_{1};                ///< Determines threads number used in image processing

  /// Published scan message
  sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;

  /// Class for managing CameraInfo messages
  image_geometry::PinholeCameraModel cam_model_;

  /// Determines if laser scan message is configurated
  bool is_scan_msg_configured_{false};

  /// Calculated laser scan msg indexes for each depth image column
  std::vector<unsigned> scan_msg_index_;

  /// Calculated maximal distances for measurements not included as floor
  std::vector<float> dist_to_ground_corrected;

  /// Calculated sensor tilt compensation factors
  std::vector<float> tilt_compensation_factor_;

  /// The vertical offset of image based on calibration data
  int image_vertical_offset_{0};

  sensor_msgs::msg::Image::SharedPtr dbg_image_;
  std::list<std::pair<int, int>> min_dist_points_indices_;

  std::mutex points_indices_mutex_;
  std::mutex scan_msg_mutex_;
};

}  // namespace laserscan_kinect

#endif  // LASERSCAN_KINECT__LASERSCAN_KINECT_HPP_
