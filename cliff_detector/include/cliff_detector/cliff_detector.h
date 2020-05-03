#pragma once

#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_nav_msgs/Point32List.h>
#include <geometry_msgs/Point32.h>

#include <sstream>
#include <limits.h>
#include <math.h>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <utility>
#include <cstdlib>
#include <vector>

namespace cliff_detector {
/**
 * @brief The CliffDetector class detect cliff based on depth image
 */
class CliffDetector {
 public:
  CliffDetector();
  /**
   * @brief detectCliff detects descending stairs based on the information in a depth image
   *
   * This function detects descending stairs based on the information
   * in the depth encoded image (UInt16 encoding). To do this, it requires
   * the synchornized Image/CameraInfo pair associated with the image.
   *
   * @param depth_msg UInt16 encoded depth image.
   * @param info_msg CameraInfo associated with depth_msg
   */
  void detectCliff(const sensor_msgs::ImageConstPtr& depth_msg,
                   const sensor_msgs::CameraInfoConstPtr& info_msg);
  /**
   * @brief setRangeLimits sets the minimum and maximum range of depth value from RGBD sensor.
   *
   * @param rmin Minimum of depth value which will be used in data processing.
   * @param rmin Maximum of depth value which will be used in data processing.
   */
  void setRangeLimits(const float rmin, const float rmax);
  /**
   * @brief setSensorMountHeight sets the height of sensor mount (in meters) from ground
   *
   * @param height Value of sensor mount height (in meters).
   */
  void setSensorMountHeight (const float height);
  /**
   * @brief getSensorMountHeight gets sensor mount height
   *
   * @return Return sensor mount height in meters
   */
  float getSensorMountHeight() { return sensor_mount_height_; }
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
  float getSensorTiltAngle() { return sensor_tilt_angle_; }
  /**
   * @brief setPublishDepthEnable
   * @param enable
   */
  void setPublishDepthEnable(const bool enable) { publish_depth_enable_ = enable; }
  /**
   * @brief getPublishDepthEnable
   * @return
   */
  bool getPublishDepthEnable() const { return publish_depth_enable_; }
  /**
     * @brief Sets the number of image rows to use in data processing.
     *
     * scan_height is the number of rows (pixels) to use in the output.
     *
     * @param scan_height Number of pixels centered around the center of
     * the image to data processing
     *
     */
  /**
   * @brief setCamModelUpdate
   */
  void setCamModelUpdate(const bool u) { cam_model_update_ = u; }
  /**
   * @brief setUsedDepthHeight
   * @param height
   */
  void setUsedDepthHeight(const unsigned int height);
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
  void setGroundMargin (const float margin);
  /**
   * @brief setParametersConfigurated
   * @param u
   */
  void setParametersConfigurated (const bool u) { depth_sensor_params_update = u; }

 protected:
  /**
   * @brief lengthOfVector calculates length of 3D vector.
   *
   * Method calculates the length of 3D vector assumed to be a vector with start at the (0,0,0).
   *
   * @param vec Vector 3D which lenght will be calculated.
   * @return Returns the length of 3D vector.
   */
  double lengthOfVector(const cv::Point3d& vec) const;
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
  double angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2) const;
  /**
    * Find 2D points relative to robots where stairs are detected
    *
    * This uses a method
    *
    * @param depth_msg The UInt16 encoded depth message.
    */
  void findCliffInDepthImage(const sensor_msgs::ImageConstPtr& depth_msg);
  /**
    * Calculate vertical angle_min and angle_max by measuring angles between
    * the top ray, bottom ray, and optical center ray
    *
    * @param camModel The image_geometry camera model for this image.
    * @param min_angle The minimum vertical angle
    * @param max_angle The maximum vertical angle
    */
  void fieldOfView(double & min, double & max, double x1, double y1,
                   double xc, double yc, double x2, double y2);
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
  void calcTiltCompensationFactorsForImgRows(double vertical_fov);

 private:
  // ROS parameters configurated with config file or dynamic_reconfigure
  std::string  outputFrameId_;        ///< Output frame_id for laserscan.
  float        range_min_;            ///< Stores the current minimum range to use
  float        range_max_;            ///< Stores the current maximum range to use
  float        sensor_mount_height_;  ///< Height of sensor mount from ground
  float        sensor_tilt_angle_;    ///< Sensor tilt angle (degrees)
  bool         publish_depth_enable_; ///< Determines if depth should be republished
  bool         cam_model_update_;     ///< Determines if continuously cam model update required
  unsigned int used_depth_height_;    ///< Used depth height from img bottom (px)
  unsigned int block_size_;           ///< Square block (subimage) size (px).
  unsigned int block_points_thresh_;  ///< Threshold value of points in block to admit stairs
  unsigned int depth_image_step_row_; ///< Rows step in depth processing (px).
  unsigned int depth_image_step_col_; ///< Columns step in depth processing (px).
  float        ground_margin_;        ///< Margin for ground points feature detector (m)

  bool depth_sensor_params_update;
  /// Class for managing sensor_msgs/CameraInfo messages
  image_geometry::PinholeCameraModel camera_model_;
  /// Calculated distances to ground for every row of depth image in mm
  std::vector<unsigned int> dist_to_ground_;
  /// Calculated sensor tilt compensation factors
  std::vector<double> tilt_compensation_factor_;

  std::vector<double> delta_row_;

 public:
  sensor_msgs::Image new_depth_msg_;
  sensor_msgs::ImageConstPtr depth_msg_to_pub_;

  ///< Store points which contain stairs
  depth_nav_msgs::Point32List stairs_points_msg_;
};

}
