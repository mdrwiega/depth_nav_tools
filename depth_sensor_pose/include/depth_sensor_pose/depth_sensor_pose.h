#pragma once

#include <sstream>
#include <limits>
#include <vector>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include <ros/console.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Core>

//#define     DEBUG 1
//#define 		DATA_TO_FILE
//#define 		DEBUG_CALIBRATION

namespace depth_sensor_pose {

class DepthSensorPose
{
public:
  DepthSensorPose() = default;
  ~DepthSensorPose() = default;

  DepthSensorPose (const DepthSensorPose &) = delete;
  DepthSensorPose & operator= (const DepthSensorPose &) = delete;

  /**
     * Converts the information in a depth image (sensor_msgs::Image) to a sensor_msgs::LaserScan.
     *
     * This function converts the information in the depth encoded image (UInt16) into
     * a sensor_msgs::LaserScan as accurately as possible.  To do this, it requires the
     * synchornized Image/CameraInfo pair associated with the image.
     *
     * @param depth_msg UInt16 or Float32 encoded depth image.
     * @param info_msg CameraInfo associated with depth_msg
     * @return sensor_msgs::LaserScanPtr for the center row(s) of the depth image.
     *
     */
  void estimateParams(const sensor_msgs::ImageConstPtr& depth_msg,
                   const sensor_msgs::CameraInfoConstPtr& info_msg);
  /**
     * Sets the minimum and maximum range for the sensor_msgs::LaserScan.
     *
     * rangeMin is used to determine how close of a value to allow through when multiple radii
     * correspond to the same angular increment.  rangeMax is used to set the output message.
     *
     * @param rangeMin Minimum range to assign points to the laserscan, also minimum range to use points in the output scan.
     * @param rangeMax Maximum range to use points in the output scan.
     *
     */
  void setRangeLimits(const float rmin, const float rmax);

  /**
   * @brief setSensorMountHeight sets the height of sensor mount (in meters) from ground
   *
   * @param height Value of sensor mount height (in meters).
   */
  void setSensorMountHeightMin (const float height);
  /**
   * @brief setSensorMountHeight sets the height of sensor mount (in meters) from ground
   *
   * @param height Value of sensor mount height (in meters).
   */
  void setSensorMountHeightMax (const float height);
  /**
   * @brief setSensorTiltAngle sets the sensor tilt angle (in degrees)
   * @param angle
   */
  void setSensorTiltAngleMin (const float angle);
  /**
   * @brief setSensorTiltAngle sets the sensor tilt angle (in degrees)
   * @param angle
   */
  void setSensorTiltAngleMax (const float angle);
  /**
   * @brief setPublishDepthEnable
   * @param enable
   */
  void setPublishDepthEnable (const bool enable) { publish_depth_enable_ = enable; }
  /**
   * @brief getPublishDepthEnable
   * @return
   */
  bool getPublishDepthEnable () const { return publish_depth_enable_; }
  /**
   * @brief setCamModelUpdate
   * @param u
   */
  void setCamModelUpdate (const bool u) { cam_model_update_ = u; }
  /**
   * @brief setUsedDepthHeight
   * @param height
   */
  void setUsedDepthHeight(const unsigned int height);
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
  void setReconfParamsUpdated (bool updated) {reconf_serv_params_updated_ = updated; }

  void setRansacMaxIter (const unsigned int u) { ransacMaxIter_ = u; }

  void setRansacDistanceThresh (const float u) { ransacDistanceThresh_ = u; }

  void setGroundMaxPoints (const unsigned int u) { max_ground_points_ = u; }

  float getSensorTiltAngle () const { return tilt_angle_est_; }

  float getSensorMountHeight () const { return mount_height_est_; }

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
  double lengthOfVector( const cv::Point3d& vec) const;
  /**
     * Computes the angle between two cv::Point3d
     *
     * Computes the angle of two cv::Point3d assumed to be vectors starting at the origin (0,0,0).
     * Uses the following equation: angle = arccos(a*b/(|a||b|)) where a = ray1 and b = ray2.
     *
     * @param ray1 The first ray
     * @param ray2 The second ray
     * @return The angle between the two rays (in radians)
     *
     */
  double angleBetweenRays( const cv::Point3d& ray1, const cv::Point3d& ray2) const;
  /**
  * Calculate vertical angle_min and angle_max by measuring angles between the top
  * ray, bottom ray, and optical center ray
  *
  * @param camModel The image_geometry camera model for this image.
  * @param min_angle The minimum vertical angle
  * @param max_angle The maximum vertical angle
  */
  void fieldOfView( double & min, double & max, double x1, double y1,
                    double xc, double yc, double x2, double y2 );

  void calcDeltaAngleForImgRows( double vertical_fov);
  /**
    * @brief Calculates distances to ground for every row of depth image
    *
    * Calculates distances to ground for rows of depth image based on known height of sensor
    * mount and tilt angle. It assume that sensor height and tilt angle in relative to ground
    * is constant.
    *
    * Calculated values are placed in vector dist_to_ground_.
    */
  void calcGroundDistancesForImgRows( double mount_height, double tilt_angle,
                                      std::vector<unsigned int>& distances);

  void getGroundPoints(const sensor_msgs::ImageConstPtr& depth_msg,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& points);

  void sensorPoseCalibration(const sensor_msgs::ImageConstPtr& depth_msg,
                             double & tilt, double & height);

public:
  sensor_msgs::Image new_depth_msg_;

private:
  // ROS parameters configurated with config files or dynamic_reconfigure
  float        range_min_{0};                ///< Stores the current minimum range to use
  float        range_max_{0};                ///< Stores the current maximum range to use
  float        mount_height_min_{0};         ///< Min height of sensor mount from ground
  float        mount_height_max_{0};         ///< Max height of sensor mount from ground
  float        tilt_angle_min_{0};           ///< Min angle of sensor tilt in degrees
  float        tilt_angle_max_{0};           ///< Max angle of sensor tilt in degrees

  bool         publish_depth_enable_{false}; ///< Determines if modified depth image should be published
  bool         cam_model_update_{false};     ///< Determines if continuously cam model update is required
  unsigned int used_depth_height_{0};        ///< Used depth height from img bottom (px)
  unsigned int depth_image_step_row_{0};     ///< Rows step in depth processing (px).
  unsigned int depth_image_step_col_{0};     ///< Columns step in depth processing (px).

  unsigned int max_ground_points_{0};
  unsigned int ransacMaxIter_{0};
  float ransacDistanceThresh_{0};
  float groundDistTolerance_{0};

  ///< Class for managing sensor_msgs/CameraInfo messages
  image_geometry::PinholeCameraModel camera_model_;

  float mount_height_est_{0};
  float tilt_angle_est_{0};
  bool 	reconf_serv_params_updated_{true};

  std::vector<double> delta_row_;
  std::vector<unsigned int>dist_to_ground_max_;
  std::vector<unsigned int>dist_to_ground_min_;
};

} // namespace depth_sensor_pose