namespace laserscan_kinect {

#include <image_geometry/pinhole_camera_model.h>

/**
* @brief lengthOfVector calculate the length of the 3D vector
*/
inline double lengthOfVector(const cv::Point3d& vec) {
  return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z);
}

/**
* @brief angleBetweenRays calculates angle between two rays in degrees
*/
inline double angleBetweenRays(const cv::Point3d& ray1, const cv::Point3d& ray2) {
  double dot = ray1.x * ray2.x + ray1.y * ray2.y + ray1.z * ray2.z;
  return acos(dot / (lengthOfVector(ray1) * lengthOfVector(ray2)));
}

/**
* @brief fieldOfView calculates field of view (angle)
*/
inline void calcFieldOfView(const image_geometry::PinholeCameraModel& camera_model,
  const cv::Point2d && left, const cv::Point2d && center,
  const cv::Point2d && right, double & min, double & max) {

    cv::Point2d rect_pixel_left = camera_model.rectifyPoint(left);
    cv::Point3d left_ray = camera_model.projectPixelTo3dRay(rect_pixel_left);

    cv::Point2d rect_pixel_right = camera_model.rectifyPoint(right);
    cv::Point3d right_ray = camera_model.projectPixelTo3dRay(rect_pixel_right);

    cv::Point2d rect_pixel_center = camera_model.rectifyPoint(center);
    cv::Point3d center_ray = camera_model.projectPixelTo3dRay(rect_pixel_center);

    min = -angleBetweenRays(center_ray, right_ray);
    max = angleBetweenRays(left_ray, center_ray);
  }

}