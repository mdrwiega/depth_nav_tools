/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder nor the names of its
 *        contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 * @file   laserscan_kinect.h
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @brief  laserscan_kinect package
 */

#pragma once

#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

#include <vector>
#include <string>

constexpr double SCAN_TIME = 1.0 / 30.0;

namespace laserscan_kinect
{ 
class LaserScanKinect
{
public:
    LaserScanKinect(): scan_msg_(new sensor_msgs::LaserScan()) { }
    ~LaserScanKinect() = default;

    /**
   * @brief prepareLaserScanMsg converts depthimage and prepare new LaserScan message
   *
   * @param depth_msg Message that contains depth image which will be converted to LaserScan.
   * @param info_msg Message which contains depth sensor parameters.
   *
   * @return Return pointer to LaserScan message.
   */
    sensor_msgs::LaserScanPtr getLaserScanMsg(const sensor_msgs::ImageConstPtr& depth_msg,
                                                  const sensor_msgs::CameraInfoConstPtr& info_msg);
    /**
   * @brief setOutputFrame sets the frame to output laser scan
   * @param frame
   */
    void setOutputFrame (const std::string frame) { output_frame_id_ = frame; }
    /**
   * @brief setRangeLimits sets depth sensor min and max ranges
   *
   * @param rmin Minimum sensor range (below it is death zone) in meters.
   * @param rmax Maximum sensor range in meters.
   */
    void setRangeLimits(const float rmin, const float rmax);
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
    void setCamModelUpdate (const bool enable) { cam_model_update_ = enable; }
    /**
    * @brief setSensorMountHeight sets the height of sensor mount (in meters)
    */
    void setSensorMountHeight (const float height);
    /**
   * @brief setSensorTiltAngle sets the sensor tilt angle (in degrees)
   *
   * @param angle
   */
    void setSensorTiltAngle (const float angle);
    /**
   * @brief setGroundRemove enables or disables the feature which remove ground from scan
   *
   * @param enable
   */
    void setGroundRemove (const bool enable) { ground_remove_enable_ = enable; }
    /**
   * @brief setGroundMargin sets the floor margin (in meters)
   *
   * @param margin
   */
    void setGroundMargin (const float margin);
    /**
   * @brief setTiltCompensation enables or disables the feature which compensates sensor tilt
   *
   * @param enable
   */
    void setTiltCompensation (const bool enable) { tilt_compensation_enable_ = enable; }
    /**
   * @brief setScanConfigurated sets the configuration status
   *
   * @param enable
   */
    void setScanConfigurated (const bool configurated) { is_scan_msg_configurated_ = configurated; }

protected:
    /**
    * @brief lengthOfVector calculate length of 3D vector
    *
    * @param ray
    * @return
    */
    double lengthOfVector(const cv::Point3d& ray) const;
    /**
    * @brief angleBetweenRays calculate angle between two rays in degrees
    * @return
    */
    double angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2) const;
    /**
    * @brief fieldOfView calculate field of view (angle)
    */
    void calcFieldOfView( const cv::Point2d && left, const cv::Point2d && center,
                          const cv::Point2d && right, double & min, double & max);
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
    void calcScanMsgIndexForImgCols(const sensor_msgs::ImageConstPtr& depth_msg);

    /**
    * @brief getSmallestValueInColumn finds smallest values in depth image columns
     */
    template <typename T>
    float getSmallestValueInColumn(const T* depth_row, int row_size, int col)
    {
        float depth_min = std::numeric_limits<float>::max();
        const unsigned range_min_mm = range_min_ * 1000;
        const unsigned range_max_mm = range_max_ * 1000;

        // Loop over pixels in column. Calculate z_min in column
        for (size_t i = image_vertical_offset_; i < image_vertical_offset_ + scan_height_;
                                                                                i += depth_img_row_step_)
        {
            unsigned depth_raw_mm;
            float depth_m;

            if (typeid(T) == typeid(uint16_t))
            {
                depth_raw_mm = static_cast<unsigned>(depth_row[row_size * i + col]);
                depth_m = static_cast<float>(depth_raw_mm) / 1000.0;
            }
            else if (typeid(T) == typeid(float))
            {
                depth_m = static_cast<float>(depth_row[row_size * i + col]);
                depth_raw_mm = static_cast<unsigned>(depth_m * 1000.0);
            }

            if (tilt_compensation_enable_) // Check if tilt compensation is enabled
            {
                depth_m *= tilt_compensation_factor_[i];
            }

            // Check if point is in ranges and find min value in column
            if (depth_raw_mm >= range_min_mm && depth_raw_mm <= range_max_mm)
            {
                if (ground_remove_enable_)
                {
                    if (depth_m < depth_min && depth_raw_mm < dist_to_ground_corrected[i])
                    {
                        depth_min = depth_m;
                    }
                }
                else
                {
                    if (depth_m < depth_min)
                    {
                        depth_min = depth_m;
                    }
                }
            }
        }
        return depth_min;
    }
    /**
    * @brief convertDepthToPolarCoords converts depth map to 2D
    *
    * @param depth_msg
    */
    template <typename T>
    void convertDepthToPolarCoords(const sensor_msgs::ImageConstPtr& depth_msg);

private:
    //-----------------------------------------------------------------------------------------------
    // ROS parameters configurated with configuration file or dynamic_reconfigure
    std::string output_frame_id_;           ///< Output frame_id for laserscan message.
    float range_min_{0};                    ///< Stores the current minimum range to use
    float range_max_{0};                    ///< Stores the current maximum range to use
    unsigned scan_height_{0};               ///< Number of pixel rows used to scan computing
    unsigned depth_img_row_step_{0};        ///< Row step in depth map processing
    bool  cam_model_update_{false};         ///< If continously calibration update required
    float sensor_mount_height_{0};          ///< Height of sensor mount from ground
    float sensor_tilt_angle_{0};            ///< Angle of sensor tilt
    bool  ground_remove_enable_{false};     ///< Determines if remove ground from output scan
    float ground_margin_{0};                ///< Margin for floor remove feature (in meters)
    bool  tilt_compensation_enable_{false}; ///< Determines if tilt compensation feature is on
    //-----------------------------------------------------------------------------------------------

    /// Published scan message
    sensor_msgs::LaserScanPtr scan_msg_;

    /// Class for managing CameraInfo messages
    image_geometry::PinholeCameraModel cam_model_;

    /// Determines if laser scan message is configurated
    bool is_scan_msg_configurated_{false};

    /// Calculated laser scan msg indexes for each depth image column
    std::vector<unsigned> scan_msg_index_;

    /// Calculated maximal distances for measurements not included as floor
    std::vector<unsigned> dist_to_ground_corrected;

    /// Calculated sensor tilt compensation factors
    std::vector<float> tilt_compensation_factor_;

    /// The vertical offset of image based on calibration data
    int image_vertical_offset_{0};
};

};

