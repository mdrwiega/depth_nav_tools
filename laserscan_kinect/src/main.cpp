#include <laserscan_kinect/laserscan_kinect_node.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserscan_kinect");
  ros::NodeHandle pnh("~");

  laserscan_kinect::LaserScanKinectNode converter(pnh);

  ros::spin();

  return 0;
}
