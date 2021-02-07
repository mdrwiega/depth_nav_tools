#include <depth_sensor_pose/depth_sensor_pose.h>

// #define 		DEBUG_CALIBRATION

namespace depth_sensor_pose {

DepthSensorPose::DepthSensorPose()
  : dbg_image_(new sensor_msgs::Image()) {
}

void DepthSensorPose::estimateParams(const sensor_msgs::ImageConstPtr& depth_msg,
                                    const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // Update data based on depth sensor parameters only if new params values
  // or turned on continuous data calculations
  if (reconf_serv_params_updated_ || cam_model_update_) {
    camera_model_.fromCameraInfo(info_msg);

    double cx = camera_model_.cx(), cy = camera_model_.cy(), vert_min, vert_max;

    // Calculate the vertical field of view of the depth sensor
    fieldOfView(vert_min, vert_max, cx, 0, cx, cy, cx, depth_msg->height -1);
    const double vertical_fov = vert_max - vert_min;

    calcDeltaAngleForImgRows(vertical_fov);
    calcGroundDistancesForImgRows(mount_height_max_, tilt_angle_min_, dist_to_ground_max_);
    calcGroundDistancesForImgRows(mount_height_min_, tilt_angle_max_, dist_to_ground_min_);

    ROS_DEBUG_STREAM("Recalculate parameters because of camera model update. Parameters:\n"
                      << "\n cx = " << camera_model_.cx() << " cy = " << camera_model_.cy()
                      << "\nvertical FOV: " << vertical_fov );

    reconf_serv_params_updated_ = false;
  }

  double tilt = 0;
  double height = 0;

  sensorPoseCalibration(depth_msg, tilt, height);

  tilt_angle_est_ = tilt;
  mount_height_est_ = height;
}

void DepthSensorPose::setRangeLimits(const float rmin, const float rmax) {
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

void DepthSensorPose::setSensorMountHeightMin(const float height) {
  if (height > 0) {
      mount_height_min_ = height;
  }
  else {
    mount_height_min_ = 0;
    ROS_ERROR("Incorrect value of sensor mount height parameter. Set default value: 0.");
  }
}

void DepthSensorPose::setSensorMountHeightMax(const float height) {
  if( height > 0) {
    mount_height_max_ = height;
  }
  else {
    mount_height_max_ = 1;
    ROS_ERROR("Incorrect value of sensor mount height parameter. Set default value: 1m.");
  }
}

void DepthSensorPose::setSensorTiltAngleMin(const float angle) {
  if( angle < 90 && angle > -90) {
    tilt_angle_min_ 	= angle;
  }
  else {
    tilt_angle_min_ 	= 0;
    ROS_ERROR("Incorrect value of sensor tilt angle parameter. Set default value: 0.");
  }
}

void DepthSensorPose::setSensorTiltAngleMax(const float angle) {
  if( angle < 90 && angle > -90) {
    tilt_angle_max_ 	= angle;
  }
  else {
    tilt_angle_max_ 	= 0;
    ROS_ERROR("Incorrect value of sensor tilt angle parameter. Set default value: 0.");
  }
}

void DepthSensorPose::setUsedDepthHeight(const unsigned height) {
  if( height > 0) {
    used_depth_height_ = height;
  }
  else {
    used_depth_height_ = 200;
    ROS_ERROR("Incorrect value of used depth height parameter. Set default value: 200.");
  }
}

void DepthSensorPose::setDepthImgStepRow(const int step) {
  if (step > 0) {
    depth_image_step_row_ = step;
  }
  else {
    depth_image_step_row_ = 1;
    ROS_ERROR("Incorrect value depth image row step parameter. Set default value: 1.");
  }
}

void DepthSensorPose::setDepthImgStepCol(const int step) {
  if (step > 0) {
    depth_image_step_col_ = step;
  }
  else {
    depth_image_step_col_ = 1;
    ROS_ERROR("Incorrect value depth image column step parameter. Set default value: 1.");
  }
}

sensor_msgs::ImageConstPtr DepthSensorPose::getDbgImage() const {
  return dbg_image_;
}

double DepthSensorPose::lengthOfVector(const cv::Point3d& vec) const {
  return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

double DepthSensorPose::angleBetweenRays(const cv::Point3d& ray1,
                                         const cv::Point3d& ray2) const {
  double dot = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;

  return acos(dot / (lengthOfVector(ray1) * lengthOfVector(ray2)));
}

void DepthSensorPose::fieldOfView(double & min, double & max, double x1, double y1,
                                  double xc, double yc, double x2, double y2) {
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

void DepthSensorPose::calcDeltaAngleForImgRows(double vertical_fov) {
  const unsigned img_height = camera_model_.fullResolution().height;
  delta_row_.resize(img_height);

  // Angle between the sensor optical axis and ray to specific cell on the camera matrix
  for (unsigned i = 0; i < img_height; i++) {
    delta_row_[i] = vertical_fov * (i - camera_model_.cy() - 0.5) / (static_cast<double>(img_height) - 1);
  }
}

void DepthSensorPose::calcGroundDistancesForImgRows(double mount_height, double tilt_angle,
                                                    std::vector<unsigned int>& distances) {
  const double alpha = tilt_angle * M_PI / 180.0; // Sensor tilt angle in radians
  const unsigned img_height = camera_model_.fullResolution().height;

  ROS_ASSERT(img_height >= 0 && mount_height > 0);

  distances.resize(img_height);

  // Calculations for each row of image
  for(unsigned i = 0; i < img_height; i++) {
    if ((delta_row_[i] + alpha) > 0) {
      distances[i] = mount_height * sin(M_PI/2 - delta_row_[i])
          / cos(M_PI/2 - delta_row_[i] - alpha);
      ROS_ASSERT(distances[i] > 0);
    }
    else {
      distances[i] = 100;
    }
  }
}

template<typename T>
void DepthSensorPose::getGroundPoints(const sensor_msgs::ImageConstPtr& depth_msg,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
                                      std::list<std::pair<unsigned, unsigned>>& points_indices) {
  enum Point { Row, Col, Depth };

#ifdef DEBUG_CALIBRATION
  std::vector<double> deltaVec;
#endif
  const unsigned img_height = depth_msg->height;
  const unsigned img_width = depth_msg->width;

  const T* data = reinterpret_cast<const T*>(&depth_msg->data[0]);

  // Loop over each row in image from bottom of img
  for (unsigned j = 0; j < img_height; j += depth_image_step_row_) {
    // Loop over each column in image
    for (unsigned col = 0; col < img_width; col += depth_image_step_col_) {
      unsigned row = img_height - 1 - j;
      ROS_ASSERT(row < img_height);

      float d = 0.0;

      if (typeid(T) == typeid(uint16_t)) {
        unsigned depth_raw_mm = static_cast<unsigned>(data[img_width * row + col]);
        d = static_cast<float>(depth_raw_mm) / 1000.0;
      }
      else if (typeid(T) == typeid(float)) {
        d = static_cast<float>(data[img_width * row + col]);
      }

      #ifdef DEBUG_CALIBRATION
        std::ostringstream s;
        if (col % (depth_image_step_col_ * 10) == 0 && j % (depth_image_step_row_ * 10) == 0) {
          s << "GroundPoints point: (" << col << ", " << row << ") "
            << "d: " << d << " dist_to_ground min/max: (" << dist_to_ground_min_[row]
            << ", " << dist_to_ground_max_[row] << ")";
          ROS_INFO_STREAM(s.str());
        }
      #endif

      // Check if distance to point is greater than distance to ground plane
      if (points->size() <= max_ground_points_ && d > range_min_ && d < range_max_ &&
          d > dist_to_ground_min_[row] && d < dist_to_ground_max_[row]) {

        double z = d * 0.001f;
        double x = z * (static_cast<float>(j) - camera_model_.cx()) / camera_model_.fx();
        double y = z * (static_cast<float>(col) - camera_model_.cy()) / camera_model_.fy();

        points->push_back(pcl::PointXYZ(x, y, z));
        points_indices.push_back({j, col});
      }
    }
  }
}

void DepthSensorPose::sensorPoseCalibration(
    const sensor_msgs::ImageConstPtr& depth_msg,
    [[maybe_unused]] double& tilt, [[maybe_unused]] double& height) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
  std::list<std::pair<unsigned, unsigned>> points_indices;

  // Check image encoding
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    getGroundPoints<uint16_t>(depth_msg, ground_points, points_indices);
  }
  else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    getGroundPoints<float>(depth_msg, ground_points, points_indices);
  }
  else {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }

  // Generate and publish debug image if necessary
  if (publish_depth_enable_) {
    dbg_image_ = prepareDbgImage(depth_msg, points_indices);
  }

  if (ground_points->size() >= 3) {
    // Estimate model parameters with RANSAC
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
        model (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (ground_points));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);
    ransac.setDistanceThreshold (ransacDistanceThresh_);
    ransac.setMaxIterations(ransacMaxIter_);
    ransac.computeModel();

    Eigen::VectorXf ground_coeffs(4);
    ransac.getModelCoefficients(ground_coeffs);

    // Calculate height and
    float a = ground_coeffs[0], b = ground_coeffs[1];
    float c = ground_coeffs[2], d = ground_coeffs[3];

    // Dot product two vectors v=[a,b,c], w=[1,1,0]
    tilt_angle_est_ =  std::acos ((b*b + a*a) / (std::sqrt(b*b + a*a) * std::sqrt(a*a+b*b+c*c))) * 180.0 / M_PI;
    mount_height_est_ = std::abs(d) / std::sqrt(a*a+b*b+c*c);

    ROS_DEBUG("height = %.4f angle = %.4f", mount_height_est_, tilt_angle_est_);

    #ifdef DEBUG_CALIBRATION
    std::ostringstream s;
    s << " sensorLocationCalibration: points_size = " << ground_points->size()
      << "\n a = " << ground_coeffs[0]
      << "\n b = " << ground_coeffs[1]
      << "\n c = " << ground_coeffs[2]
      << "\n d = " << ground_coeffs[3]
      << "\n t1 = " << acos((a+b)/(sqrt(2)*sqrt(a*a+b*b+c*c))) * 180.0 / M_PI
      << "\n t2 = " << acos((a+c)/(sqrt(2)*sqrt(a*a+b*b+c*c))) * 180.0 / M_PI
      << "\n t3 = " << acos((b+c)/(sqrt(2)*sqrt(a*a+b*b+c*c))) * 180.0 / M_PI
      << "\n t4 = " << acos((a)/(sqrt(1)*sqrt(a*a+b*b+c*c))) * 180.0 / M_PI
      << "\n t5 = " << acos((b)/(sqrt(1)*sqrt(a*a+b*b+c*c))) * 180.0 / M_PI
      << "\n t6 = " << acos((c)/(sqrt(1)*sqrt(a*a+b*b+c*c))) * 180.0 / M_PI
      << "\n height = " << height
      << "\n tilt = " << tilt;
    ROS_INFO_STREAM_THROTTLE(1, s.str());
    #endif
  }
  else {
    ROS_ERROR("Ground points not detected");
  }
}

sensor_msgs::ImagePtr DepthSensorPose::prepareDbgImage(const sensor_msgs::ImageConstPtr& depth_msg,
  const std::list<std::pair<unsigned, unsigned>>& ground_points_indices) {

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
  for (const auto pt : ground_points_indices) {
    const auto row = pt.first;
    const auto col = pt.second;
    rgb_data[row * img->width + col][0] = 255;
  }

  // Add line which is the border of the ground detection area
  std::list<std::pair<unsigned, unsigned>> pts;
  for (unsigned i = 0; i < img->width; ++i) {
    pts.push_back({img->height - used_depth_height_, i});
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

} // namespace depth_sensor_pose
