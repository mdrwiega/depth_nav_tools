/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Michal Drwiega (drwiega.michal@gmail.com)
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
 * @date   10.2015
 * @brief  laserscan_kinect package
 */

#include <laserscan_kinect/laserscan_kinect_node.h>

#define TIME_MEASUREMENT 0 /// Measurement of processing time

#if TIME_MEASUREMENT
#include <boost/chrono.hpp>
#endif

using namespace laserscan_kinect;

//=================================================================================================
// Public methods
//=================================================================================================
LaserScanKinect::LaserScanKinect():
  is_scan_msg_configurated_(false), scan_msg_(new sensor_msgs::LaserScan()) { }

//=================================================================================================
sensor_msgs::LaserScanPtr LaserScanKinect::prepareLaserScanMsg(
    const sensor_msgs::ImageConstPtr& depth_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
#if TIME_MEASUREMENT
  boost::chrono::high_resolution_clock::time_point start = boost::chrono::high_resolution_clock::now();
#endif
  // Configure message if necessary
  if(!is_scan_msg_configurated_ || cam_model_update_)
  {
    camera_model_.fromCameraInfo(info_msg);

    const double cx = camera_model_.cx(), cy = camera_model_.cy();
    double min_angle, max_angle, vertical_fov;

    // Calculate vertical field of view angles
    fieldOfView(min_angle, max_angle, cx, 0, cx, cy, cx, depth_msg->height - 1);
    vertical_fov = max_angle - min_angle;

    // Calculate horizontal field of view angles
    fieldOfView(min_angle, max_angle, 0, cy, cx, cy, depth_msg->width - 1, cy);

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
      scan_msg_->range_min = range_min_ * *std::min_element( tilt_compensation_factor_.begin(),
                                                             tilt_compensation_factor_.end() );
      scan_msg_->range_max = range_max_ * *std::max_element( tilt_compensation_factor_.begin(),
                                                             tilt_compensation_factor_.end() );
    }
    else
    {
      scan_msg_->range_min = range_min_;
      scan_msg_->range_max = range_max_;
    }

    // Calculate scan message indexes for each depth image column
    calcScanMsgIndexForImgCols(depth_msg);

    // Check if scan_height is in image_height
    if(scan_height_ / 2 > cy || scan_height_ / 2 > depth_msg->height - cy)
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
    convertDepthToPolarCoords(depth_msg);
  }
  else
  {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }

#if TIME_MEASUREMENT // End of time measurement
  boost::chrono::milliseconds ms = boost::chrono::duration_cast<boost::chrono::milliseconds>
      (boost::chrono::high_resolution_clock::now() - start);
  ROS_DEBUG_STREAM("\nProcessing takes " << ms.count() << " ms.");
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
// Private methods
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
void LaserScanKinect::fieldOfView( double & min, double & max, double x1, double y1,
                                   double xc, double yc, double x2, double y2)
{
  cv::Point2d raw_pixel_left(x1, y1);
  cv::Point2d rect_pixel_left = camera_model_.rectifyPoint(raw_pixel_left);
  cv::Point3d left_ray = camera_model_.projectPixelTo3dRay(rect_pixel_left);

  cv::Point2d raw_pixel_right(x2, y2);
  cv::Point2d rect_pixel_right = camera_model_.rectifyPoint(raw_pixel_right);
  cv::Point3d right_ray = camera_model_.projectPixelTo3dRay(rect_pixel_right);

  cv::Point2d raw_pixel_center(xc, yc);
  cv::Point2d rect_pixel_center = camera_model_.rectifyPoint(raw_pixel_center);
  cv::Point3d center_ray = camera_model_.projectPixelTo3dRay(rect_pixel_center);

  min = -angleBetweenRays(center_ray, right_ray);
  max = angleBetweenRays(left_ray, center_ray);

  ROS_ASSERT(min < 0 && max > 0);
}

//=================================================================================================
void LaserScanKinect::calcGroundDistancesForImgRows(double vertical_fov)
{
  const double alpha = sensor_tilt_angle_ * M_PI / 180.0; // Sensor tilt angle in radians
  const int img_height = camera_model_.fullResolution().height;

  ROS_ASSERT(img_height >= 0);

  dist_to_ground_.resize(img_height);

  for(int i = 0; i < img_height; i++) // Coefficients calculations for each row of image
  {
    // Angle between ray and optical center
    double delta = vertical_fov * (i - camera_model_.cy() - 0.5) / ((double)img_height - 1);

    if ((delta + alpha) > 0)
    {
      dist_to_ground_[i] = sensor_mount_height_ * sin(M_PI/2-delta) * 1000
          / cos(M_PI/2-delta-alpha);
      ROS_ASSERT(dist_to_ground_[i] > 0);
    }
    else
      dist_to_ground_[i] = 100 * 1000;
  }
}

//=================================================================================================
void LaserScanKinect::calcTiltCompensationFactorsForImgRows(double vertical_fov)
{
  const double alpha = sensor_tilt_angle_ * M_PI / 180.0; // Sensor tilt angle in radians
  const int img_height = camera_model_.fullResolution().height;

  ROS_ASSERT(img_height >= 0);

  tilt_compensation_factor_.resize(img_height);

  for(int i = 0; i < img_height; i++) // Process all rows
  {
    double delta = vertical_fov * (i - camera_model_.cy() - 0.5) / ((double)img_height - 1);

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
    double th = -atan2((double)(u - camera_model_.cx()) * 0.001f / camera_model_.fx(), 0.001f);
    scan_msg_index_[u] = (th - scan_msg_->angle_min) / scan_msg_->angle_increment;
  }
}

//=================================================================================================
void LaserScanKinect::convertDepthToPolarCoords(const sensor_msgs::ImageConstPtr &depth_msg)
{
  const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
  const int offset = (int)(camera_model_.cy()- scan_height_ / 2);
  const int row_size = depth_msg->step / sizeof(uint16_t);

  const unsigned int range_min_mm = range_min_ * 1000;
  const unsigned int range_max_mm = range_max_ * 1000;
  const int ground_margin_mm = ground_margin_ * 1000;

  // Loop over each column in image
  for (int j = 0; j < (int)depth_msg->width; j++)
  {
    float depth_min = MAX_UINT16;

    // Loop over pixels in column. Calculate z_min in column
    for (int i = offset; i < offset + scan_height_; i += depth_img_row_step_)
    {
      uint16_t depth_raw = depth_row[row_size * i + j];
      float depth;

      // Check if tilt compensation is enabled
      if ( tilt_compensation_enable_ )
        depth = depth_raw * tilt_compensation_factor_[i] / 1000.0f;
      else
        depth = depth_raw / 1000.0f;

      // Enabled remove floor from scan feature
      if ( ground_remove_enable_ )
      {
        // Find min values in columns
        if( depth_raw >= range_min_mm && depth_raw <= range_max_mm &&
            depth < depth_min && depth_raw < (dist_to_ground_[i] - ground_margin_mm) )
        {
          depth_min = depth;
        }
      }
      else // Disabled remove floor from scan feature
      {
        // Find min values in columns
        if( depth_raw >= range_min_mm && depth_raw <= range_max_mm && depth < depth_min )
        {
          depth_min = depth;
        }
      }
    }
    // When the smallest distance in column found then conversion to polar coords
    if (depth_min != MAX_UINT16)
    {
      // Calculate x in XZ ( z = depth )
      float x = (j - camera_model_.cx()) * depth_min  / camera_model_.fx();

      // Calculate distance in polar coords
      scan_msg_->ranges[ scan_msg_index_[j] ] = sqrt(x * x + depth_min * depth_min);
    }
    else // No information about distances in j column
      scan_msg_->ranges[scan_msg_index_[j]] = NAN;
  }
}
