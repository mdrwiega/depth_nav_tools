#include <cliff_detector/cliff_detector_node.h>

namespace cliff_detector {

CliffDetectorNode::CliffDetectorNode(ros::NodeHandle& n, ros::NodeHandle& pnh):
  node_rate_hz_(2), pnh_(pnh), it_(n), reconf_srv_(pnh)
{
  boost::mutex::scoped_lock lock(connection_mutex_);

  // Set callback for dynamic reconfigure server
  reconf_srv_.setCallback(boost::bind(&CliffDetectorNode::reconfigureCb, this, _1, _2));

  // New depth image publisher
  pub_ = it_.advertise("cliff_detector/depth", 1,
                       boost::bind(&CliffDetectorNode::connectCb, this),
                       boost::bind(&CliffDetectorNode::disconnectCb, this));

  // Publisher for stairs points msg
  pub_points_ = n.advertise<depth_nav_msgs::Point32List>("cliff_detector/points", 2);
}

CliffDetectorNode::~CliffDetectorNode()
{
  sub_.shutdown();
}

void CliffDetectorNode::setNodeRate(const unsigned int rate)
{
  if (rate <= 30)
    node_rate_hz_ = rate;
  else
    node_rate_hz_ = 30;
}

unsigned int CliffDetectorNode::getNodeRate()
{
  return node_rate_hz_;
}

void CliffDetectorNode::depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                                const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  try
  {
    // Run cliff detector based on depth image
    detector_.detectCliff(depth_msg, info_msg);

    // Publish stairs points msg
    pub_points_.publish(detector_.stairs_points_msg_);

    // Publishes new depth image with added cliff
    if (detector_.getPublishDepthEnable())
      pub_.publish(detector_.new_depth_msg_);
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR_THROTTLE(1.0, "Could not perform stairs detection: %s", e.what());
  }
}

void CliffDetectorNode::connectCb()
{
  boost::mutex::scoped_lock lock(connection_mutex_);
  if (!sub_ && pub_.getNumSubscribers() > 0)
  {
    ROS_DEBUG("Connecting to depth topic.");
    image_transport::TransportHints hints("raw", ros::TransportHints(), pnh_);
    sub_ = it_.subscribeCamera("image", 1, &CliffDetectorNode::depthCb, this, hints);
  }
}

void CliffDetectorNode::disconnectCb()
{
  boost::mutex::scoped_lock lock(connection_mutex_);
  if (pub_.getNumSubscribers() == 0)
  {
    ROS_DEBUG("Unsubscribing from depth topic.");
    sub_.shutdown();
  }
}

void CliffDetectorNode::reconfigureCb(
    cliff_detector::CliffDetectorConfig& config, uint32_t level)
{
  node_rate_hz_ = (unsigned int) config.rate;

  detector_.setRangeLimits(config.range_min, config.range_max);
  detector_.setSensorMountHeight(config.sensor_mount_height);
  detector_.setSensorTiltAngle(config.sensor_tilt_angle);
  detector_.setPublishDepthEnable(config.publish_depth);
  detector_.setCamModelUpdate(config.cam_model_update);

  detector_.setUsedDepthHeight((unsigned int)config.used_depth_height);
  detector_.setBlockSize(config.block_size);
  detector_.setBlockPointsThresh(config.block_points_thresh);
  detector_.setDepthImgStepRow(config.depth_img_step_row);
  detector_.setDepthImgStepCol(config.depth_img_step_col);
  detector_.setGroundMargin(config.ground_margin);

  detector_.setParametersConfigurated(false);
}

}