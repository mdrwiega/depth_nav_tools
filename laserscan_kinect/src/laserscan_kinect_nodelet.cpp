#include <laserscan_kinect/laserscan_kinect_node.h>

#include <memory>

#include <nodelet/nodelet.h>

namespace laserscan_kinect
{

class LaserScanKinectNodelet : public nodelet::Nodelet
{
public:
  LaserScanKinectNodelet()  = default;
  ~LaserScanKinectNodelet() = default;

private:
  virtual void onInit()
  {
    converter.reset(new LaserScanKinectNode(getNodeHandle(), getPrivateNodeHandle()));
  };

  std::shared_ptr<LaserScanKinectNode> converter;
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(laserscan_kinect, LaserScanKinectNodelet,laserscan_kinect::LaserScanKinectNodelet, nodelet::Nodelet);

