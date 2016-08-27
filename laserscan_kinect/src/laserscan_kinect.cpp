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
 * @file   laserscan_kinect.cpp
 * @author Michal Drwiega (drwiega.michal@gmail.com)
 * @brief  laserscan_kinect package
 */

#include <laserscan_kinect/laserscan_kinect_node.h>

#include <cmath>
#include <algorithm>
#include <typeinfo>

#define TIME_MEASUREMENT 0 /// Measurement of processing time

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#if TIME_MEASUREMENT
#include <chrono>
#endif

namespace laserscan_kinect {

//=================================================================================================
sensor_msgs::LaserScanPtr LaserScanKinect::prepareLaserScanMsg(
        const sensor_msgs::ImageConstPtr & depth_msg,
        const sensor_msgs::CameraInfoConstPtr & info_msg)
{
#if TIME_MEASUREMENT
    auto start = std::chrono::high_resolution_clock::now();
#endif
    // Configure message if necessary
    if(!is_scan_msg_configurated_ || cam_model_update_)
    {
        cam_model_.fromCameraInfo(info_msg);

        double min_angle, max_angle;
        using Point = cv::Point2d;

        // Calculate vertical field of view angles
        calcFieldOfView( Point(cam_model_.cx(), 0),
                         Point(cam_model_.cx(), cam_model_.cy()),
                         Point(cam_model_.cx(), depth_msg->height - 1), min_angle, max_angle);
        double vertical_fov = max_angle - min_angle;

        // Calculate horizontal field of view angles
        calcFieldOfView( Point(0,                    cam_model_.cy()),
                         Point(cam_model_.cx(),      cam_model_.cy()),
                         Point(depth_msg->width - 1, cam_model_.cy()), min_angle, max_angle);

        if ( ground_remove_enable_ )      // Remove floor from scan
            calcGroundDistancesForImgRows(vertical_fov);

        if ( tilt_compensation_enable_ )  // Sensor tilt compensation
            calcTiltCompensationFactorsForImgRows(vertical_fov);

        scan_msg_->angle_min = min_angle;
        scan_msg_->angle_max = max_angle;
        scan_msg_->angle_increment = (max_angle - min_angle) / (depth_msg->width - 1);
        scan_msg_->time_increment = 0.0;
        scan_msg_->scan_time = SCAN_TIME;

        // Set min and max range in preparing message
        if (tilt_compensation_enable_)
        {
            scan_msg_->range_min = range_min_ * *std::min_element(tilt_compensation_factor_.begin(),
                                                                  tilt_compensation_factor_.end());
            scan_msg_->range_max = range_max_ * *std::max_element(tilt_compensation_factor_.begin(),
                                                                  tilt_compensation_factor_.end());
        }
        else
        {
            scan_msg_->range_min = range_min_;
            scan_msg_->range_max = range_max_;
        }

        calcScanMsgIndexForImgCols(depth_msg);

        // Check if scan_height is in image_height
        if(scan_height_ / 2 > cam_model_.cy() || scan_height_ / 2 > depth_msg->height - cam_model_.cy())
        {
            std::stringstream ss;
            ss << "scan_height ( " << scan_height_ << " pixels) is too large for the image height.";
            throw std::runtime_error(ss.str());
        }
        is_scan_msg_configurated_ = true;
    }

    // Prepare laser scan message
    scan_msg_->header = depth_msg->header;
    if(output_frame_id_.length() > 0)
        scan_msg_->header.frame_id = output_frame_id_;

    scan_msg_->ranges.assign(depth_msg->width, std::numeric_limits<float>::quiet_NaN());

    // Check if image encoding is correctly
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
        convertDepthToPolarCoords<uint16_t>(depth_msg);
    }
    else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        convertDepthToPolarCoords<float>(depth_msg);
    }
    else
    {
        std::stringstream ss;
        ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
        throw std::runtime_error(ss.str());
    }

#if TIME_MEASUREMENT
    auto diff = std::chrono::high_resolution_clock::now() - start;
    ROS_DEBUG_STREAM("\nProcessing time: " << chrono::duration <double, milli> (diff).count() << " ms.");
#endif

    return scan_msg_;
}

//=================================================================================================
void LaserScanKinect::setRangeLimits(const float rmin, const float rmax)
{
    if (rmin >= 0 && rmin < rmax)
        range_min_ = rmin;
    else
    {
        range_min_ = 0;
        ROS_ERROR("Incorrect value of range minimal parameter. Set default value: 0.");
    }
    if (rmax >= 0 && rmin < rmax)
        range_max_ = rmax;
    else
    {
        range_max_ = 10;
        ROS_ERROR("Incorrect value of range maximum parameter. Set default value: 10.");
    }
}

//=================================================================================================
void LaserScanKinect::setScanHeight(const int scan_height)
{
    if(scan_height > 0)
        scan_height_ = scan_height;
    else
    {
        scan_height_ = 10;
        ROS_ERROR("Incorrect value of scan height parameter. Set default value: 100.");
    }
}

//=================================================================================================
void LaserScanKinect::setDepthImgRowStep(const int row_step)
{
    if( row_step > 0 )
        depth_img_row_step_ = row_step;
    else
    {
        depth_img_row_step_ = 1;
        ROS_ERROR("Incorrect value depth imgage row step parameter. Set default value: 1.");
    }
}

//=================================================================================================
void LaserScanKinect::setSensorMountHeight (const float height)
{
    if( height > 0)
        sensor_mount_height_ = height;
    else
    {
        sensor_mount_height_ = 0;
        ROS_ERROR("Incorrect value of sensor mount height parameter. Set default value: 0.");
    }
}

//=================================================================================================
void LaserScanKinect::setSensorTiltAngle (const float angle)
{
    if( angle < 90 && angle > -90)
        sensor_tilt_angle_ 	= angle;
    else
    {
        sensor_tilt_angle_ 	= 0;
        ROS_ERROR("Incorrect value of sensor tilt angle parameter. Set default value: 0.");
    }
}

//=================================================================================================
void LaserScanKinect::setGroundMargin (const float margin)
{
    if( margin > 0)
        ground_margin_ = margin;
    else
    {
        ground_margin_ = 0;
        ROS_ERROR("Incorrect value of ground margin parameter. Set default value: 0.");
    }
}

//=================================================================================================
double LaserScanKinect::lengthOfVector(const cv::Point3d& vec) const
{
    return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

//=================================================================================================
double LaserScanKinect::angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2) const
{
    double dot = ray1.x * ray2.x + ray1.y * ray2.y + ray1.z * ray2.z;

    return acos(dot / (lengthOfVector(ray1) * lengthOfVector(ray2)));
}

//=================================================================================================
void LaserScanKinect::calcFieldOfView( const cv::Point2d && left, const cv::Point2d && center,
                                       const cv::Point2d && right, double & min, double & max)
{
    cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(left);
    cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(rect_pixel_left);

    cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(right);
    cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(rect_pixel_right);

    cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(center);
    cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);

    min = -angleBetweenRays(center_ray, right_ray);
    max = angleBetweenRays(left_ray, center_ray);

    ROS_ASSERT(min < 0 && max > 0);
}

//=================================================================================================
void LaserScanKinect::calcGroundDistancesForImgRows(double vertical_fov)
{
    const double alpha = sensor_tilt_angle_ * M_PI / 180.0; // Sensor tilt angle in radians
    const int img_height = cam_model_.fullResolution().height;

    ROS_ASSERT(img_height >= 0);

    dist_to_ground_.resize(img_height);

    // Coefficients calculations for each row of image
    for(int i = 0; i < img_height; ++i)
    {
        // Angle between ray and optical center
        double delta = vertical_fov * (i - cam_model_.cy() - 0.5)
                     / (static_cast<double>(img_height) - 1);

        if ((delta + alpha) > 0)
        {
            dist_to_ground_[i] = sensor_mount_height_ * sin(M_PI / 2 - delta) * 1000
                               / cos(M_PI / 2 - delta - alpha);

            ROS_ASSERT(dist_to_ground_[i] > 0);
        }
        else
            dist_to_ground_[i] = 100 * 1000;
    }
}

//=================================================================================================
void LaserScanKinect::calcTiltCompensationFactorsForImgRows(double vertical_fov)
{
    const double alpha = sensor_tilt_angle_ * M_PI / 180.0;
    const int img_height = cam_model_.fullResolution().height;

    ROS_ASSERT(img_height >= 0);

    tilt_compensation_factor_.resize(img_height);

    for(int i = 0; i < img_height; ++i) // Processing all rows
    {
        double delta = vertical_fov * (i - cam_model_.cy() - 0.5) / ((double)img_height - 1);

        tilt_compensation_factor_[i] = sin(M_PI/2 - delta - alpha) / sin(M_PI/2 - delta);
        ROS_ASSERT(tilt_compensation_factor_[i] > 0 && tilt_compensation_factor_[i] < 10);
    }
}

//=================================================================================================
void LaserScanKinect::calcScanMsgIndexForImgCols(const sensor_msgs::ImageConstPtr& depth_msg)
{
    scan_msg_index_.resize((int)depth_msg->width);

    for (int u = 0; u < (int)depth_msg->width; u++)
    {
        double th = -atan2((double)(u - cam_model_.cx()) * 0.001f / cam_model_.fx(), 0.001f);
        scan_msg_index_[u] = (th - scan_msg_->angle_min) / scan_msg_->angle_increment;
    }
}

//=================================================================================================
template <typename T>
void LaserScanKinect::convertDepthToPolarCoords(const sensor_msgs::ImageConstPtr &depth_msg)
{
    const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
    const int offset = (int)(cam_model_.cy()- scan_height_ / 2);
    const int row_size = depth_msg->step / sizeof(T);

    const unsigned range_min_mm = range_min_ * 1000;
    const unsigned range_max_mm = range_max_ * 1000;
    const int ground_margin_mm = ground_margin_ * 1000;

    // Correction of distance to ground for each row of image
    std::vector<int> dist_to_ground_corrected(cam_model_.fullResolution().height);
    for (unsigned i = 0; i < dist_to_ground_.size(); ++i)
    {
        dist_to_ground_corrected[i] = dist_to_ground_[i] - ground_margin_mm;
    }

    // Loop over each column in image
    for (unsigned j = 0; j < (int)depth_msg->width; ++j)
    {
        float depth_min = std::numeric_limits<T>::max();

        // Loop over pixels in column. Calculate z_min in column
        for (unsigned i = offset; i < offset + scan_height_; i += depth_img_row_step_)
        {
            int depth_raw_mm;
            float depth_m;

            if (typeid(T) == typeid(uint16_t))
            {
                depth_raw_mm = depth_row[row_size * i + j];
                depth_m = static_cast<float>(depth_raw_mm) / 1000.0;
            }
            else if (typeid(T) == typeid(float))
            {
                depth_m = depth_row[row_size * i + j];
                depth_raw_mm = static_cast<unsigned>(depth_m * 1000.0);
            }

            if (tilt_compensation_enable_) // Check if tilt compensation is enabled
            {
                depth_m *= tilt_compensation_factor_[i];
            }

            if (ground_remove_enable_) // Enabled remove floor from scan feature
            {
                // Find min values in columns
                if (depth_raw_mm >= range_min_mm && depth_raw_mm <= range_max_mm &&
                    depth_raw_mm < dist_to_ground_corrected[i] && depth_m < depth_min)
                {
                    depth_min = depth_m;
                }
            }
            else // Disabled remove floor from scan feature
            {
                // Find min values in columns
                if (depth_raw_mm >= range_min_mm && depth_raw_mm <= range_max_mm &&
                    depth_m < depth_min)
                {
                    depth_min = depth_m;
                }
            }
        }
        // When the smallest distance in column found then conversion to polar coords
        if (depth_min != std::numeric_limits<T>::max())
        {
            // Calculate x in XZ ( z = depth )
            float x = (j - cam_model_.cx()) * depth_min  / cam_model_.fx();

            // Calculate distance in polar coordinates
            scan_msg_->ranges[scan_msg_index_[j]] = sqrt(x * x + depth_min * depth_min);
        }
        else // No information about distances in j column
            scan_msg_->ranges[scan_msg_index_[j]] = NAN;
    }
}

} // end of namespace
