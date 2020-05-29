#include <depth_sensor_pose/depth_sensor_pose.h>

namespace depth_sensor_pose {

void DepthSensorPose::estimateParams( const sensor_msgs::ImageConstPtr& depth_msg,
                                      const sensor_msgs::CameraInfoConstPtr& info_msg ) {
  ROS_DEBUG("Start estimation procedure");

  // Update data based on depth sensor parameters only if new params values
  // or turned on continuous data calculations
  if (reconf_serv_params_updated_ || cam_model_update_) {
    camera_model_.fromCameraInfo(info_msg);

    double cx = camera_model_.cx(), cy = camera_model_.cy(), vert_min, vert_max;

    // Calculate vertical field of view angles
    fieldOfView(vert_min, vert_max, cx, 0, cx, cy, cx, depth_msg->height -1);
    double vertical_fov = vert_max - vert_min;

    ROS_DEBUG("Recalculate distance to ground coefficients for image rows.");

    calcDeltaAngleForImgRows(vertical_fov);
    dist_to_ground_max_.resize(camera_model_.fullResolution().height);
    dist_to_ground_min_.resize(camera_model_.fullResolution().height);
    calcGroundDistancesForImgRows(mount_height_max_,tilt_angle_min_, dist_to_ground_max_);
    calcGroundDistancesForImgRows(mount_height_min_,tilt_angle_max_, dist_to_ground_min_);

    reconf_serv_params_updated_ = false;
#ifdef DEBUG
    std::ostringstream s;
    for(int v = 0; v < depth_msg->height; v+=8)
      s << " " << rowFloorThreshold_[v];
    ROS_INFO_STREAM_THROTTLE(2,"rowFloorThreshold = " << s.str());
#endif

#ifdef DEBUG
    const uint16_t* depthRoww = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
    int rowStepp = depth_msg->step / sizeof(uint16_t);
    std::ostringstream s, sss;
    for(int v = 0; v < depth_msg->height; v+=40, depthRoww += (rowStepp*40))
      sss << " " << (int)depthRoww[320] * tiltCompensationFactor_[v];
    for(int v = 0; v < depth_msg->height; v+=8)
      s << " " << tiltCompensationFactor_[v];
    ROS_INFO_STREAM_THROTTLE(2,"tiltCompensation = " << s.str());
    ROS_INFO_STREAM_THROTTLE(0.4,"depth_tilt_compensation" << sss.str());
#endif
  }

#ifdef DEBUG
  const uint16_t* depthRow = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
  int rowStep = depth_msg->step / sizeof(uint16_t);
  std::ostringstream ss, sss, stream;
  for(int v = 0; v < depth_msg->height; v+=40, depthRow += (rowStep*40))
  {
    ss << " " << (int)depthRow[320] * 0.001f;
  }
  ROS_INFO_STREAM_THROTTLE(0.4,"depth" << ss.str());
#endif
#ifdef DATA_TO_FILE
  depthRow = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
  for(int v = 0; v < depth_msg->height; v++, depthRow += rowStep)
    stream << " " << (int)depthRow[320] * 0.001f;
  std::ofstream file("/home/themin/kinect_data.txt", std::ios::out | std::ios::app);
  file << stream.str() << "\n";
  file.close();
#endif

  // Check if image encoding is correctly
  if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    double tilt = 0;
    double height = 0;

    sensorPoseCalibration(depth_msg, tilt, height);

    tilt_angle_est_ = tilt;
    mount_height_est_ = height;
  }
  else {
    std::stringstream ss;
    ss << "Depth image has unsupported encoding: " << depth_msg->encoding;
    throw std::runtime_error(ss.str());
  }
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

void DepthSensorPose::setUsedDepthHeight(const unsigned int height) {
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
  const unsigned int img_height = camera_model_.fullResolution().height;

  delta_row_.resize(img_height);

  // Angle between ray and optical center
  for (unsigned i = 0; i < img_height; i++) {
    delta_row_[i] = vertical_fov * (i - camera_model_.cy() - 0.5) / (static_cast<double>(img_height) - 1);
  }
}

void DepthSensorPose::calcGroundDistancesForImgRows(double mount_height, double tilt_angle,
                                                    std::vector<unsigned int>& distances) {
  const double alpha = tilt_angle * M_PI / 180.0; // Sensor tilt angle in radians
  const unsigned int img_height = camera_model_.fullResolution().height;

  ROS_ASSERT(img_height >= 0 && mount_height > 0);

  distances.resize(img_height);

  // Calculations for each row of image
  for(unsigned int i = 0; i < img_height; i++) {
    if ((delta_row_[i] + alpha) > 0) {
      distances[i] = mount_height * sin(M_PI/2 - delta_row_[i]) * 1000
          / cos(M_PI/2 - delta_row_[i] - alpha);
      ROS_ASSERT(distances[i] > 0);
    }
    else {
      distances[i] = 100 * 1000;
    }
  }
}

void DepthSensorPose::getGroundPoints( const sensor_msgs::ImageConstPtr& depth_msg,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& points) {
  enum Point { Row, Col, Depth };

#ifdef DEBUG_CALIBRATION
  std::vector<double> deltaVec;
#endif
  const unsigned int img_height = camera_model_.fullResolution().height;

  const unsigned int range_min_mm = range_min_ * 1000;
  const unsigned int range_max_mm = range_max_ * 1000;

  const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
  const unsigned int row_size = depth_msg->step / sizeof(uint16_t);

  if (publish_depth_enable_) {
    new_depth_msg_ = *depth_msg;
  }

  uint16_t* new_depth_row = reinterpret_cast<uint16_t*>(&new_depth_msg_.data[0]);

  // Loop over each row in image from bottom of img
  for (unsigned int j = 0; j < (unsigned int)depth_msg->height; j += depth_image_step_row_) {
    // Loop over each column in image
    for (unsigned int i = 0; i < (unsigned int)depth_msg->width; i += depth_image_step_col_) {
      unsigned int row = img_height - 1 - j;
      ROS_ASSERT(row < img_height);

      unsigned int d = (depth_row[row_size * row + i]);

      // Check if distance to point is greater than distance to ground plane
      if (points->size() <= max_ground_points_ && d > range_min_mm && d < range_max_mm &&
          d > dist_to_ground_min_[row] && d < dist_to_ground_max_[row]) {
        double z = d * 0.001f;
        double x = z * ((double)j - camera_model_.cx()) / camera_model_.fx();
        double y = z * ((double)i - camera_model_.cy()) / camera_model_.fy();

        points->push_back(pcl::PointXYZ(x, y, z));

        if (publish_depth_enable_) {
          new_depth_row[row_size * row + i] = 10000U;
        }
      }
    }
  }

#ifdef DEBUG_CALIBRATION
  std::ostringstream s;
  s << " getGroundPoints: imgHeight = " << imgH
    << "\n coarseTilt = " << alpha << " fov = " << fov
    << "\n cx = " << camModel_.cx() << " cy = " << camModel_.cy()
    << "\n rowsThresh_size = " << rowThreshCalibration.size()
    << "\n deltaVec_size = " << rowThreshCalibration.size()
    << "\n deltaVec[] = ";
  for(int v = 0; v < deltaVec.size(); v++)
    s << " " << deltaVec[v];
  s  << "\n rowThreshCalibration[] = ";
  for(int v = 0; v < rowThreshCalibration.size(); v++)
    s << " " << rowThreshCalibration[v];
  ROS_INFO_STREAM_THROTTLE(2, s.str());
#endif
}

void DepthSensorPose::sensorPoseCalibration(
    const sensor_msgs::ImageConstPtr& depth_msg,
    [[maybe_unused]] double& tilt, [[maybe_unused]] double& height) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);

  // Get ground points
  getGroundPoints(depth_msg, points);

  if (points->size() >= 3) {
    // Estimate model parameters with RANSAC
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
        model (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (points));

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
    tilt_angle_est_ =  std::acos ((b*b + a*a) /
                                  (std::sqrt(b*b + a*a) * std::sqrt(a*a+b*b+c*c))) * 180.0 / M_PI;
    mount_height_est_ = std::abs(d) / std::sqrt(a*a+b*b+c*c);

    ROS_DEBUG("height = %.4f angle = %.4f", mount_height_est_, tilt_angle_est_);

    #ifdef DEBUG_CALIBRATION
    std::ostringstream s;
    s << " sensorLocationCalibration: points_size = " << points->size()
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

} // namespace depth_sensor_pose
