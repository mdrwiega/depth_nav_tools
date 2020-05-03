#include <cliff_detector/cliff_detector_node.h>

namespace cliff_detector {

CliffDetector::CliffDetector(): depth_sensor_params_update(false) {}

void CliffDetector::detectCliff( const sensor_msgs::ImageConstPtr& depth_msg,
                                           const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  // Update data based on depth sensor parameters only if new params values
  // or turned on continuous data calculations
  if(!depth_sensor_params_update || cam_model_update_)
  {
    camera_model_.fromCameraInfo(info_msg);

    double angle_min, angle_max, vertical_fov;
    double cx = camera_model_.cx(), cy = camera_model_.cy();

    ROS_ASSERT(cx > 0 && cy > 0);

    // Calculate fields of views angles - vertical and horizontal
    fieldOfView(angle_min, angle_max, cx, 0, cx, cy, cx, depth_msg->height -1);
    vertical_fov = angle_max - angle_min;

    ROS_ASSERT(vertical_fov > 0);

    // Calculate angles between optical axis and rays for each row of image
    calcDeltaAngleForImgRows(vertical_fov);

    // Calculate ground distances for every row of depth image
    calcGroundDistancesForImgRows(vertical_fov);

    // Sensor tilt compensation
    calcTiltCompensationFactorsForImgRows(vertical_fov);

    // Check scan_height vs image_height
    if (used_depth_height_ > depth_msg->height)
    {
      ROS_ERROR("Parameter used_depth_height is higher than height of image.");
      used_depth_height_ = depth_msg->height;
    }
    depth_sensor_params_update = true;
  }

  // Check if image encoding is correctly
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    findCliffInDepthImage(depth_msg);
  }
  else
  {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }
}

void CliffDetector::setRangeLimits( const float rmin, const float rmax )
{
  if (rmin >= 0 && rmin < rmax)
  {
    range_min_ = rmin;
  }
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

void CliffDetector::setSensorMountHeight (const float height)
{
  if (height > 0)
  {
    sensor_mount_height_ = height;
  }
  else
  {
    sensor_mount_height_ = 0;
    ROS_ERROR("Incorrect value of sensor mount height parameter. Set default value: 0.");
  }
}

void CliffDetector::setSensorTiltAngle (const float angle)
{
  if (angle < 90 && angle > -90)
  {
    sensor_tilt_angle_ 	= angle;
  }
  else
  {
    sensor_tilt_angle_ 	= 0;
    ROS_ERROR("Incorrect value of sensor tilt angle parameter. Set default value: 0.");
  }
}

void CliffDetector::setUsedDepthHeight(const unsigned int height)
{
  if (height > 0)
  {
    used_depth_height_ = height;
  }
  else
  {
    used_depth_height_ = 100;
    ROS_ERROR("Incorrect value of used depth height parameter. Set default value: 100.");
  }
}

void CliffDetector::setBlockSize(const int size)
{
  if (size > 0 && (size % 2 == 0))
  {
    block_size_ = size;
  }
  else
  {
    block_size_ = 8;
    ROS_ERROR("Incorrect value of block size. Set default value: 8.");
  }
}

void CliffDetector::setBlockPointsThresh(const int thresh)
{
  if (thresh > 0)
  {
    block_points_thresh_ = thresh;
  }
  else
  {
    block_points_thresh_ = 1;
    ROS_ERROR("Incorrect value of block points threshold parameter. Set default value: 1.");
  }
}

void CliffDetector::setDepthImgStepRow(const int step)
{
  if (step > 0)
  {
    depth_image_step_row_ = step;
  }
  else
  {
    depth_image_step_row_ = 1;
    ROS_ERROR("Incorrect value depth image row step parameter. Set default value: 1.");
  }
}

void CliffDetector::setDepthImgStepCol(const int step)
{
  if (step > 0 )
  {
    depth_image_step_col_ = step;
  }
  else
  {
    depth_image_step_col_ = 1;
    ROS_ERROR("Incorrect value depth image column step parameter. Set default value: 1.");
  }
}

void CliffDetector::setGroundMargin (const float margin)
{
  if (margin > 0)
  {
    ground_margin_ = margin;
  }
  else
  {
    ground_margin_ = 0;
    ROS_ERROR("Incorrect value of ground margin parameter. Set default value: 0.");
  }
}

double CliffDetector::lengthOfVector(const cv::Point3d& vec) const
{
  return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

double CliffDetector::angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2) const
{
  double dot = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
  return acos(dot / (lengthOfVector(ray1) * lengthOfVector(ray2)));
}

void CliffDetector::fieldOfView(double & min, double & max, double x1, double y1,
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

void CliffDetector::calcDeltaAngleForImgRows(double vertical_fov)
{
  const unsigned int img_height = camera_model_.fullResolution().height;

  delta_row_.resize(img_height);

  // Angle between ray and optical center
  for(unsigned int i = 0; i < img_height; i++) {
    delta_row_[i] = vertical_fov * (i - camera_model_.cy() - 0.5) / ((double)img_height - 1);
  }
}

void CliffDetector::calcGroundDistancesForImgRows(double vertical_fov)
{
  const double alpha = sensor_tilt_angle_ * M_PI / 180.0; // Sensor tilt angle in radians
  const unsigned int img_height = camera_model_.fullResolution().height;

  ROS_ASSERT(img_height >= 0);

  dist_to_ground_.resize(img_height);

  // Calculations for each row of image
  for(unsigned int i = 0; i < img_height; i++)
  {
    // Angle between ray and optical center
    if ((delta_row_[i] + alpha) > 0)
    {
      dist_to_ground_[i] = sensor_mount_height_ * sin(M_PI/2 - delta_row_[i]) * 1000
          / cos(M_PI/2 - delta_row_[i] - alpha);
      ROS_ASSERT(dist_to_ground_[i] > 0);
    }
    else
    {
      dist_to_ground_[i] = 100 * 1000;
    }
  }
}

void CliffDetector::calcTiltCompensationFactorsForImgRows(double vertical_fov)
{
  const double alpha = sensor_tilt_angle_ * M_PI / 180.0; // Sensor tilt angle in radians
  const unsigned int img_height = camera_model_.fullResolution().height;

  ROS_ASSERT(img_height >= 0);

  tilt_compensation_factor_.resize(img_height);

  for(unsigned int i = 0; i < img_height; i++) // Process all rows
  {
    tilt_compensation_factor_[i] = sin(M_PI/2 - delta_row_[i] - alpha)
                                    / sin(M_PI/2 - delta_row_[i]);
    ROS_ASSERT(tilt_compensation_factor_[i] > 0 && tilt_compensation_factor_[i] < 10);
  }
}

void CliffDetector::findCliffInDepthImage( const sensor_msgs::ImageConstPtr &depth_msg)
{
  enum point { Row, Col, Depth };

  const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);

  const unsigned int row_size = depth_msg->step / sizeof(uint16_t);

  const unsigned int img_height = camera_model_.fullResolution().height;
  const unsigned int img_width = camera_model_.fullResolution().width;

  if ((block_size_ % 2) != 0)
  {
    ROS_ERROR("Block size should be even number. Value will be decreased by one.");
    block_size_--;
  }

  const unsigned int block_cols_nr = img_width / block_size_;
  const unsigned int block_rows_nr = used_depth_height_ / block_size_;

  const int ground_margin_mm = ground_margin_ * 1000;
  const unsigned int range_min_mm = range_min_ * 1000;
  const unsigned int range_max_mm = range_max_ * 1000;

  // Check if points thresh isn't too big
  if (block_points_thresh_ >= (block_size_ * block_size_ / depth_image_step_col_
                               / depth_image_step_row_))
  {
    ROS_ERROR("Block points threshold is too big. Maximum admissible value will be set.");
    block_points_thresh_ = block_size_*block_size_ / depth_image_step_col_ / depth_image_step_row_;
  }

  // Vector for block, constains rows, cols, depth values
  std::vector<std::vector<int> >tpoints (block_size_ * block_size_, std::vector<int>(3));

  // Rows, cols and depth values for points which apply to stairs
  std::vector<std::vector<int> > stairs_points;

  // Indicates which point from tpoints vector corresponds to which pixel in block
  std::vector<int> px_nr;

  // Resize to size of block
  px_nr.resize(block_size_ * block_size_);

  // Four pixels in center of block
  const unsigned int c = block_size_ / 2;
  std::vector<unsigned int> center_points(4);
  center_points[0] = c * block_size_ + c-1;
  center_points[1] = c * block_size_ + c;
  center_points[2] = (c-1) * block_size_ + c-1;
  center_points[3] = (c-1)*block_size_ + c ;

  // Loop over each block row in image, bj - block column
  for (unsigned int bj = 0; bj < block_rows_nr; ++bj)
  {
    // Loop over each block column in image, bi - block row
    for (unsigned int bi = 0; bi < block_cols_nr; ++bi)
    {
      // Start of block processing
      unsigned int block_cnt = 0;
      std::fill(px_nr.begin(), px_nr.end(), -1);

      // Loop over each row in block, j - column
      for (unsigned int j = 0; j < block_size_; j += depth_image_step_row_)
      {
        // Loop over each column in block, i - row
        for (unsigned int i = 0; i < block_size_; i += depth_image_step_col_)
        {
          // Rows from bottom of image
          unsigned int row = (img_height - 1 ) - ( bj * block_size_+ j );
          unsigned int col = bi * block_size_ + i;
          ROS_ASSERT(row < img_height && col < img_width);

          unsigned int d = depth_row[row_size * row + col];

          // Check if distance to point is greater than distance to ground plane
          if (d > (dist_to_ground_[row] + ground_margin_mm) &&
              d > range_min_mm && d < range_max_mm)
          {
            tpoints[block_cnt][Row] = row;
            tpoints[block_cnt][Col] = col;
            tpoints[block_cnt][Depth] = d;
            px_nr[j * block_size_ + i] = block_cnt;
            block_cnt++;
          }
        }
      }

      // Check if number of stairs points in block exceeded threshold value
      if (block_cnt >= block_points_thresh_)
      {
        // Block size is even. So first we check four pixels in center of block.
        if (px_nr[center_points[0]] > 0)
          stairs_points.push_back(tpoints[px_nr[center_points[0]]]);
        else if (px_nr[center_points[1]] > 0)
          stairs_points.push_back(tpoints[px_nr[center_points[1]]]);
        else if (px_nr[center_points[2]] > 0)
          stairs_points.push_back(tpoints[px_nr[center_points[2]]]);
        else if (px_nr[center_points[3]] > 0)
          stairs_points.push_back(tpoints[px_nr[center_points[3]]]);
        else
        { // Otherwise add all points from block to stairs points vector
          stairs_points.insert(stairs_points.end(), tpoints.begin(), tpoints.begin() + block_cnt);
        }
      }
      block_cnt = 0;
    }
  }

  std::vector<std::vector<int> >::iterator it;
  geometry_msgs::Point32 pt;

  if (publish_depth_enable_)
  {
    new_depth_msg_ = *depth_msg;
  }

  uint16_t* new_depth_row = reinterpret_cast<uint16_t*>(&new_depth_msg_.data[0]);

  // Set header and size of points list in message
  stairs_points_msg_.header = depth_msg->header;
  stairs_points_msg_.size = (unsigned int) stairs_points.size();
  stairs_points_msg_.points.clear();
  pt.y = 0;

  for(it = stairs_points.begin(); it != stairs_points.end(); ++it)
  {
    // Calculate point in XZ plane -- depth (z)
    unsigned int row = (*it)[Row];
    pt.z = sensor_mount_height_ / std::tan(sensor_tilt_angle_ * M_PI / 180.0 + delta_row_[row]);

    // Calculate x value
    double depth = sensor_mount_height_ / std::sin(sensor_tilt_angle_*M_PI/180.0 + delta_row_[row]);

    pt.x = ((*it)[Col] - camera_model_.cx()) * depth / camera_model_.fx();

    // Add point to message
    stairs_points_msg_.points.push_back(pt);

    if (publish_depth_enable_)
    {
      ROS_ASSERT(row_size * (*it)[Row] + (*it)[Col] < (new_depth_msg_.height * new_depth_msg_.width));
      new_depth_row[row_size * (*it)[Row] + (*it)[Col]] = 10000U;
    }
  }
  ROS_DEBUG_STREAM("Stairs points: " << stairs_points.size());
}

}