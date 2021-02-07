#include <ros/ros.h>
#include <depth_sensor_pose/depth_sensor_pose_node.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_sensor_pose");
  ros::NodeHandle pnh("~");

  depth_sensor_pose::DepthSensorPoseNode estimator(pnh);

  while (ros::ok()) {
    ros::Rate rate(estimator.getNodeRate());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
