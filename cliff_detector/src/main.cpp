#include <cliff_detector/cliff_detector_node.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cliff_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  cliff_detector::CliffDetectorNode detector(nh, pnh);

  while (ros::ok())
  {
    ros::Rate rate(detector.getNodeRate());
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
