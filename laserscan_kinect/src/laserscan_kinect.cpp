#include <laserscan_kinect/laserscan_kinect_node.h>

#include <cmath>
#include <algorithm>
#include <typeinfo>
#include <thread>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

namespace laserscan_kinect {

sensor_msgs::LaserScanPtr LaserScanKinect::getLaserScanMsg(
        const sensor_msgs::ImageConstPtr & depth_msg,
        const sensor_msgs::CameraInfoConstPtr & info_msg) {
  // Configure message if necessary
  if (!is_scan_msg_configured_ || cam_model_update_) {
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

    if (ground_remove_enable_) {
      calcGroundDistancesForImgRows(vertical_fov);
    }

    if (tilt_compensation_enable_) {
      calcTiltCompensationFactorsForImgRows(vertical_fov);
    }

    scan_msg_->angle_min = min_angle;
    scan_msg_->angle_max = max_angle;
    scan_msg_->angle_increment = (max_angle - min_angle) / (depth_msg->width - 1);
    scan_msg_->time_increment = 0.0;
    scan_msg_->scan_time = 1.0 / 30.0;

    // Set min and max range in preparing message
    if (tilt_compensation_enable_) {
      scan_msg_->range_min = range_min_ * *std::min_element(tilt_compensation_factor_.begin(),
                                                            tilt_compensation_factor_.end());
      scan_msg_->range_max = range_max_ * *std::max_element(tilt_compensation_factor_.begin(),
                                                            tilt_compensation_factor_.end());
    }
    else {
      scan_msg_->range_min = range_min_;
      scan_msg_->range_max = range_max_;
    }

    calcScanMsgIndexForImgCols(depth_msg);

    // Check if scan_height is in image_height
    if (scan_height_ / 2.0 > cam_model_.cy() || scan_height_ / 2.0 > depth_msg->height - cam_model_.cy()) {
      std::stringstream ss;
      ss << "scan_height ( " << scan_height_ << " pixels) is too large for the image height.";
      throw std::runtime_error(ss.str());
    }
    image_vertical_offset_ = static_cast<int>(cam_model_.cy()- scan_height_ / 2.0);

    is_scan_msg_configured_ = true;
  }

  // Prepare laser scan message
  scan_msg_->header = depth_msg->header;
  if (output_frame_id_.length() > 0) {
    scan_msg_->header.frame_id = output_frame_id_;
  }

  scan_msg_->ranges.assign(depth_msg->width, std::numeric_limits<float>::quiet_NaN());

  // Check if image encoding is correct
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    convertDepthToPolarCoords<uint16_t>(depth_msg);
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    convertDepthToPolarCoords<float>(depth_msg);
  }
  else {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }

  // Generate and publish debug image if necessary
  if (publish_dbg_image_) {
    dbg_image_ = prepareDbgImage(depth_msg, min_dist_points_indices_);
  }
  min_dist_points_indices_.clear();

  return scan_msg_;
}

void LaserScanKinect::setRangeLimits(const float rmin, const float rmax) {
  if (rmin >= 0 && rmin < rmax) {
    range_min_ = rmin;
  }
  else {
    range_min_ = 0;
    ROS_ERROR("Incorrect value of range minimal parameter. Set default value: 0.");
  }
  if (rmax >= 0 && rmin < rmax) {
    range_max_ = rmax;
  }
  else {
    range_max_ = 10;
    ROS_ERROR("Incorrect value of range maximum parameter. Set default value: 10.");
  }
}

void LaserScanKinect::setScanHeight(const int scan_height) {
  if (scan_height > 0) {
    scan_height_ = scan_height;
  }
  else {
    scan_height_ = 10;
    ROS_ERROR("Incorrect value of scan height parameter. Set default value: 100.");
  }
}

void LaserScanKinect::setDepthImgRowStep(const int row_step) {
  if (row_step > 0) {
    depth_img_row_step_ = row_step;
  }
  else {
    depth_img_row_step_ = 1;
    ROS_ERROR("Incorrect value depth imgage row step parameter. Set default value: 1.");
  }
}

void LaserScanKinect::setSensorMountHeight (const float height) {
  if ( height > 0) {
    sensor_mount_height_ = height;
  }
  else {
    sensor_mount_height_ = 0;
    ROS_ERROR("Incorrect value of sensor mount height parameter. Set default value: 0.");
  }
}

void LaserScanKinect::setSensorTiltAngle (const float angle) {
  if (angle < 90 && angle > -90) {
    sensor_tilt_angle_ 	= angle;
  }
  else {
    sensor_tilt_angle_ 	= 0;
    ROS_ERROR("Incorrect value of sensor tilt angle parameter. Set default value: 0.");
  }
}

void LaserScanKinect::setGroundMargin (const float margin) {
  if (margin > 0) {
    ground_margin_ = margin;
  }
  else {
    ground_margin_ = 0;
    ROS_ERROR("Incorrect value of ground margin parameter. Set default value: 0.");
  }
}

sensor_msgs::ImageConstPtr LaserScanKinect::getDbgImage() const {
  return dbg_image_;
}

double LaserScanKinect::lengthOfVector(const cv::Point3d& vec) const {
  return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

double LaserScanKinect::angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2) const {
  double dot = ray1.x * ray2.x + ray1.y * ray2.y + ray1.z * ray2.z;

  return acos(dot / (lengthOfVector(ray1) * lengthOfVector(ray2)));
}

void LaserScanKinect::calcFieldOfView(const cv::Point2d && left, const cv::Point2d && center,
                                      const cv::Point2d && right, double & min, double & max) {
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

void LaserScanKinect::calcGroundDistancesForImgRows(double vertical_fov) {
  const double alpha = sensor_tilt_angle_ * M_PI / 180.0; // Sensor tilt angle in radians
  const int img_height = cam_model_.fullResolution().height;

  ROS_ASSERT(img_height >= 0);

  dist_to_ground_corrected.resize(img_height);

  // Coefficients calculations for each row of image
  for(int i = 0; i < img_height; ++i) {
    // Angle between ray and optical center
    double delta = vertical_fov * (i - cam_model_.cy() - 0.5) / (static_cast<double>(img_height) - 1);

    if ((delta - alpha) > 0) {
      dist_to_ground_corrected[i] = sensor_mount_height_ * sin(M_PI / 2 - delta) / cos(M_PI / 2 - delta + alpha);

      ROS_ASSERT(dist_to_ground_corrected[i] > 0);
    }
    else {
      dist_to_ground_corrected[i] = 100;
    }

    dist_to_ground_corrected[i] -= ground_margin_;
  }

  std::ostringstream s;
  s << " calcGroundDistancesForImgRows:";
  for (int i = 0; i < img_height; i+=10) {
    s << i << " : " << dist_to_ground_corrected[i] << "  ";
  }
  ROS_INFO_STREAM_THROTTLE(1, s.str());
}

void LaserScanKinect::calcTiltCompensationFactorsForImgRows(double vertical_fov) {
  const double alpha = sensor_tilt_angle_ * M_PI / 180.0;
  const int img_height = cam_model_.fullResolution().height;

  ROS_ASSERT(img_height >= 0);

  tilt_compensation_factor_.resize(img_height);

  for(int i = 0; i < img_height; ++i) { // Processing all rows
    double delta = vertical_fov * (i - cam_model_.cy() - 0.5) / ((double)img_height - 1);

    tilt_compensation_factor_[i] = sin(M_PI/2 - delta - alpha) / sin(M_PI/2 - delta);
    ROS_ASSERT(tilt_compensation_factor_[i] > 0 && tilt_compensation_factor_[i] < 10);
  }
}

void LaserScanKinect::calcScanMsgIndexForImgCols(const sensor_msgs::ImageConstPtr& depth_msg)
{
  scan_msg_index_.resize((int)depth_msg->width);

  for (size_t u = 0; u < static_cast<size_t>(depth_msg->width); u++) {
    double th = -atan2((double)(u - cam_model_.cx()) * 0.001f / cam_model_.fx(), 0.001f);
    scan_msg_index_[u] = (th - scan_msg_->angle_min) / scan_msg_->angle_increment;
  }
}

template <typename T>
float LaserScanKinect::getSmallestValueInColumn(const sensor_msgs::ImageConstPtr &depth_msg, int col) {
  float depth_min = std::numeric_limits<float>::max();
  int depth_min_row = -1;

  const int row_size = depth_msg->width;
  const T* data = reinterpret_cast<const T*>(&depth_msg->data[0]);

  // Loop over pixels in column. Calculate z_min in column
  for (size_t i = image_vertical_offset_; i < image_vertical_offset_ + scan_height_; i += depth_img_row_step_) {

    float depth_raw = 0.0;
    float depth_m = 0.0;

    if (typeid(T) == typeid(uint16_t)) {
      unsigned depth_raw_mm = static_cast<unsigned>(data[row_size * i + col]);
      depth_raw = static_cast<float>(depth_raw_mm) / 1000.0;
    }
    else if (typeid(T) == typeid(float)) {
      depth_raw = static_cast<float>(data[row_size * i + col]);
    }

    if (tilt_compensation_enable_) { // Check if tilt compensation is enabled
      depth_m = depth_raw * tilt_compensation_factor_[i];
    }
    else {
      depth_m = depth_raw;
    }

    // Check if point is in ranges and find min value in column
    if (depth_raw >= range_min_ && depth_raw <= range_max_) {

      if (ground_remove_enable_) {
        if (depth_m < depth_min && depth_raw < dist_to_ground_corrected[i]) {
          depth_min = depth_m;
          depth_min_row = i;
        }
      }
      else {
        if (depth_m < depth_min) {
          depth_min = depth_m;
          depth_min_row = i;
        }
      }
    }
  }

  min_dist_points_indices_.push_back({depth_min_row, col});
  return depth_min;
}

template float LaserScanKinect::getSmallestValueInColumn<uint16_t>(const sensor_msgs::ImageConstPtr&, int);
template float LaserScanKinect::getSmallestValueInColumn<float>(const sensor_msgs::ImageConstPtr&, int);

template <typename T>
void LaserScanKinect::convertDepthToPolarCoords(const sensor_msgs::ImageConstPtr &depth_msg) {

  // Converts depth from specific column to polar coordinates
  auto convert_to_polar = [&](size_t col, float depth) -> float {
    if (depth != std::numeric_limits<T>::max()) {
      // Calculate x in XZ ( z = depth )
      float x = (col - cam_model_.cx()) * depth  / cam_model_.fx();

      // Calculate distance in polar coordinates
      return sqrt(x * x + depth * depth);
    }
    return NAN; // No information about distances in column
  };

  // Processing for specified columns from [left, right]
  auto process_columns = [&](size_t left, size_t right) {
    for (size_t i = left; i < right; ++i) {
      float depth_min = getSmallestValueInColumn<T>(depth_msg, i);
      scan_msg_->ranges[scan_msg_index_[i]] = convert_to_polar(i, depth_min);
    }
  };

  #if MULTITHREAD
    const size_t thread_num = 2;
    std::vector<std::thread> workers;
    size_t step = static_cast<size_t>(depth_msg->width) / thread_num;
    size_t left = 0;
    for (size_t i = 0; i < thread_num - 1; ++i)
    {
        workers.push_back(std::thread(process_columns, left, left + step - 1));
        left = step;
    }
    process_columns(left, static_cast<size_t>(depth_msg->width));
    for (auto &t : workers) { t.join(); }
  #else
    process_columns(0, static_cast<size_t>(depth_msg->width));
  #endif
}

sensor_msgs::ImagePtr LaserScanKinect::prepareDbgImage(const sensor_msgs::ImageConstPtr& depth_msg,
  const std::list<std::pair<int, int>>& min_dist_points_indices) {

  sensor_msgs::ImagePtr img(new sensor_msgs::Image);
  img->header = depth_msg->header;
  img->height = depth_msg->height;
  img->width = depth_msg->width;
  img->encoding = "rgb8"; // image_encodings::RGB8
  img->is_bigendian = depth_msg->is_bigendian;
  img->step = img->width * 3; // 3 bytes per pixel

  img->data.resize(img->step * img->height);
  uint8_t(*rgb_data)[3] = reinterpret_cast<uint8_t(*)[3]>(&img->data[0]);

  // Convert depth image to RGB
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
    for (unsigned i = 0; i < (img->width * img->height); ++i) {
      // Scale value to cover full range of RGB 8
      uint8_t val = 255 * (depth_data[i] - range_min_ * 1000)  / (range_max_ * 1000 - range_min_ * 1000);
      rgb_data[i][0] = 255 - val;
      rgb_data[i][1] = 255 - val;
      rgb_data[i][2] = 255 - val;
    }
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    const float* depth_data = reinterpret_cast<const float*>(&depth_msg->data[0]);
    for (unsigned i = 0; i < (img->width * img->height); ++i) {
      // Scale value to cover full range of RGB 8
      uint8_t val = 255 * (depth_data[i] - range_min_)  / (range_max_ - range_min_);
      rgb_data[i][0] = 255 - val;
      rgb_data[i][1] = 255 - val;
      rgb_data[i][2] = 255 - val;
    }
  }
  else {
    throw std::runtime_error("Unsupported depth image encoding");
  }

  // Add ground points to debug image (as red points)
  for (const auto pt : min_dist_points_indices) {
    const auto row = pt.first;
    const auto col = pt.second;
    if (row >= 0 && col >= 0) {
      rgb_data[row * img->width + col][0] = 255;
      rgb_data[row * img->width + col][1] = 0;
      rgb_data[row * img->width + col][2] = 0;
    }
  }

  // Add line which is the border of the detection area
  std::list<std::pair<unsigned, unsigned>> pts;
  for (unsigned i = 0; i < img->width; ++i) {
    const auto line1_row = cam_model_.cy() - scan_height_ / 2.0;
    const auto line2_row = cam_model_.cy() + scan_height_ / 2.0;
    if (line1_row >= 0 && line1_row < img->height && line2_row >= 0 && line2_row < img->height) {
      pts.push_back({line1_row, i});
      pts.push_back({line2_row, i});
    }
  }

  for (const auto pt : pts) {
    const auto row = pt.first;
    const auto col = pt.second;
    rgb_data[row * img->width + col][0] = 0;
    rgb_data[row * img->width + col][1] = 255;
    rgb_data[row * img->width + col][2] = 0;
  }

  return img;
}

} // namespace laserscan_kinect
